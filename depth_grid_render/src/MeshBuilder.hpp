#ifndef MESHBUILDER_H
#define MESHBUILDER_H

#include <opencv2/opencv.hpp>
#include "OBJWriter.hpp"
#include "CameraUtils.hpp"

template <class T, class PredFun>
cv::Mat createMask(const cv::Mat &depth_img, T min_depth, T max_depth, PredFun f, float correction_factor=1);

template <class T, class PredFun>
cv::Mat createMask(const cv::Mat &depth_img, T min_depth, T max_depth, PredFun f, float correction_factor) {
    const T fov = static_cast<T>(45) / 180 * M_PI;
    const T cx = depth_img.cols / 2.0f;
    const T cy = depth_img.rows / 2.0f;
    const T constant_x = 2 * (tan(fov/static_cast<T>(2))) / depth_img.cols * correction_factor;
    const T constant_y = constant_x; //assume uniform pinhole model

    cv::Mat mask = cv::Mat::zeros(cv::Size(depth_img.cols, depth_img.rows), CV_8UC3);

    for (int v=0; v<depth_img.rows; v++) {
        for (int u=0; u<depth_img.cols;u++) {
            T depth = depth_img.at<T>(v, u);
            if (depth >= max_depth || depth <= min_depth) {
                continue;
            }
            T px = (u - cx) * constant_x;
            T py = -(v - cy) * constant_y;
            depth /= sqrt(1+px*px+py*py);
            px *= depth;
            py *= depth;
            if (f(Vector3<T>(px, py, -depth))) {
                mask.at<cv::Vec3b>(v, u) = cv::Vec3b(255,255,255);
            }
        }
    }
    return mask;
}

namespace {
    template<class T>
    OBJMesh<T> createMesh(const cv::Mat depth_img, T min_depth, T max_depth, T occlusion_threshold, int format, float correction_factor) {
        const T fov = static_cast<T>(45) / 180 * M_PI;
        const T cx = depth_img.cols / 2.0f;
        const T cy = depth_img.rows / 2.0f;
        const T constant_x = 2 * (tan(fov / static_cast<T>(2))) / depth_img.cols;
        const T constant_y = constant_x; //assume uniform pinhole model

        T greatest_z = std::numeric_limits<T>::min();
        T smallest_z = std::numeric_limits<T>::max();
        T greatest_y = std::numeric_limits<T>::min();
        T smallest_y = std::numeric_limits<T>::max();
        int index = 1;
        cv::Mat inds(depth_img.rows, depth_img.cols, format);
        int discarded = 0;

        OBJMesh<T> mesh;

        for (int v = 0; v < depth_img.rows; v++) {
            for (int u = 0; u < depth_img.cols; u++) {
                T depth = depth_img.at<T>(v, u);
                if (depth >= max_depth || depth <= min_depth) {
                    discarded++;
                    inds.at<T>(v, u) = -1;
                    continue;
                }

                T px = (u - cx) * constant_x;
                T py = -(v - cy) * constant_y;

                depth /= sqrt(1 + px * px + py * py);
                depth *= correction_factor;

                px *= depth;
                py *= depth;

                greatest_z = std::max(greatest_z, depth);
                smallest_z = std::min(smallest_z, depth);
                greatest_y = std::max(greatest_y, py);
                smallest_y = std::min(smallest_y, py);


                Vector3<T> point(px, py, -depth);
                Vector2<T> uv(u / (static_cast<T>(depth_img.cols - 1)), 1 - (v / (static_cast<T>(depth_img.rows - 1))));
                mesh.AddVertex(point, uv);

                inds.at<T>(v, u) = index;
                index++;
            }
        }

        std::cout << "maximum depth: " << greatest_z << "; smallest depth: " << smallest_z << std::endl;
        std::cout << "maximum y: " << greatest_y << "; smallest y: " << smallest_y << std::endl;

        std::cout << "processing " << depth_img.cols * depth_img.rows - discarded << " points (discarded " << discarded
                  << ")" << std::endl;

        std::cout << "populating indices" << std::endl;
        for (int v = 0; v < depth_img.rows - 1; v++) {
            for (int u = 0; u < depth_img.cols - 1; u++) {
                int i00 = (int) round(inds.at<T>(v, u));
                int i01 = (int) round(inds.at<T>(v, u + 1));
                int i10 = (int) round(inds.at<T>(v + 1, u));
                int i11 = (int) round(inds.at<T>(v + 1, u + 1));
                if (i00 <= 0 || i01 <= 0 || i10 <= 0 || i11 <= 0) continue;
                //compute bounds

                T depth00 = depth_img.at<T>(v, u);
                T depth01 = depth_img.at<T>(v, u + 1);
                T depth10 = depth_img.at<T>(v + 1, u);
                T depth11 = depth_img.at<T>(v + 1, u + 1);

                T mindepth = depth00;
                T maxdepth = depth00;

                mindepth = std::min(mindepth, depth01);
                mindepth = std::min(mindepth, depth10);
                mindepth = std::min(mindepth, depth11);
                maxdepth = std::max(maxdepth, depth01);
                maxdepth = std::max(maxdepth, depth10);
                maxdepth = std::max(maxdepth, depth11);
                if (maxdepth - mindepth > occlusion_threshold) {
                    continue;
                }
                Eigen::Vector3i tri1(i00, i11, i01);
                Eigen::Vector3i tri2(i00, i10, i11);
                mesh.AddTri(tri1);
                mesh.AddTri(tri2);

            }
        }

        return mesh;
    }
}



/**
 * Create a displaced grid mesh approximating the geometry in the input depth map.
 * @tparam T numerical precision (supports float and double)
 * @param depth_img
 * @param max_depth cutoff depth
 * @param occlusion_threshold maximum depth difference for connected vertices
 * @param displacement amount of mesh displacement/infilling at occlusion boundaries
 * @return
 */
template <class T>
OBJMesh<T> createMesh(const cv::Mat depth_img, T min_depth, T max_depth, T occlusion_threshold, float correction_factor=1);

template <>
OBJMesh<float> createMesh<float>(const cv::Mat depth_img, float min_depth, float max_depth, float occlusion_threshold, float correction_factor) {
    std::cout << "using single precision" << std::endl;
    return createMesh<float>(depth_img, min_depth, max_depth, occlusion_threshold, CV_32FC1, correction_factor);
}

template <>
OBJMesh<double> createMesh<double>(const cv::Mat depth_img, double min_depth, double max_depth, double occlusion_threshold, float correction_factor) {
    std::cout << "using double precision" << std::endl;
    return createMesh<double>(depth_img, min_depth, max_depth, occlusion_threshold, CV_64FC1, correction_factor);
}

#endif