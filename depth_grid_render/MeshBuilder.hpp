#ifndef MESHBUILDER_H
#define MESHBUILDER_H

#include <opencv2/opencv.hpp>
#include "OBJWriter.hpp"
#include "CameraUtils.hpp"

/**
 * Displace vertex in the plane of its neighbors to fill holes (very badly)
 * @tparam T
 * @param inds mesh indices in a 2D matrix for convenient neighbor lookup
 * @param mesh reference OBJ Mesh structure
 * @param outMesh OBJ mesh to modify (must be distinct from input mesh)
 * @param u vertex row
 * @param v vertex column
 * @param maxdist maximum depth difference to consider two vertices neighbors
 * @param fac displacement distance
 */
template <typename T>
void displace(const cv::Mat &inds, const OBJMesh<T> &mesh, OBJMesh<T> &outMesh, int u, int v, T maxdist, T fac) {
    int index = (int) round(inds.at<float>(v, u));
    if (index < 1) return;
    const Vector3<T> &vert = mesh.GetVertex(index);
    Vector3<T> &outVert = outMesh.GetVertex(index);
    std::vector<int> neighbors;
    neighbors.push_back((int) round(inds.at<float>(v, u+1)));
    neighbors.push_back((int) round(inds.at<float>(v+1, u)));
    neighbors.push_back((int) round(inds.at<float>(v-1, u)));
    neighbors.push_back((int) round(inds.at<float>(v, u-1)));
    //diagonal neighbors
    /*neighbors.push_back((int) round(inds.at<float>(v+1, u+1)));
    neighbors.push_back((int) round(inds.at<float>(v-1, u+1)));
    neighbors.push_back((int) round(inds.at<float>(v+1, u-1)));
    neighbors.push_back((int) round(inds.at<float>(v-1, u-1)));*/
    bool skip = true;
    for (const int i : neighbors) {
        if (i < 1) skip = false;
        if (std::fabs(mesh.GetVertex(i)[2] - vert[2]) > maxdist) skip = false;
    }
    if (skip) return;

    for (const int i : neighbors) {
        if (i < 1) continue;
        const Vector3<T> &neighbor = mesh.GetVertex(i);
        if (std::fabs(neighbor[2] - vert[2]) < maxdist) {
            Vector3<T> disp = vert - neighbor;
            disp.normalize();
            outVert = outVert + disp * fac;
        }
    }

}

template <class T>
OBJMesh<T> createMesh(const cv::Mat &depth_img, T max_depth, T occlusion_threshold, int format, T displacement=0) {
    const T fov = static_cast<T>(45) / 180 * M_PI;
    const T cx = depth_img.cols / 2.0f;
    const T cy = depth_img.rows / 2.0f;
    const T constant_x = 2 * (tan(fov/static_cast<T>(2))) / depth_img.cols;
    const T constant_y = constant_x; //assume uniform pinhole model

    T greatest_z = std::numeric_limits<T>::min();
    T smallest_z = std::numeric_limits<T>::max();
    T greatest_y = std::numeric_limits<T>::min();
    T smallest_y = std::numeric_limits<T>::max();
    int index = 1;
    cv::Mat inds(depth_img.rows, depth_img.cols, format);
    int discarded = 0;

    OBJMesh<T> mesh;

    for (int v=0; v<depth_img.rows; v++) {
        for (int u=0; u<depth_img.cols;u++) {
            T depth = depth_img.at<T>(v, u);
            if (depth >= max_depth) {
                discarded++;
                inds.at<T>(v, u) = -1;
                continue;
            }

            T px = (u - cx) * constant_x;
            T py = -(v - cy) * constant_y;

            depth /= sqrt(1+px*px+py*py);

            px *= depth;
            py *= depth;

            greatest_z = std::max(greatest_z, depth);
            smallest_z = std::min(smallest_z, depth);
            greatest_y = std::max(greatest_y, py);
            smallest_y = std::min(smallest_y, py);


            Vector3<T> point(px, py, -depth);
            mesh.AddVertex(point, Vector2<T>(u/(static_cast<T>(depth_img.cols-1)),1 - (v/(static_cast<T>(depth_img.rows-1)))));

            inds.at<T>(v, u) = index;
            index++;
        }
    }

    std::cout << "maximum depth: " << greatest_z << "; smallest depth: " << smallest_z << std::endl;
    std::cout << "maximum y: " << greatest_y << "; smallest y: " << smallest_y << std::endl;

    std::cout << "processing " << depth_img.cols*depth_img.rows - discarded << " points (discarded " << discarded << ")" << std::endl;

    if (displacement > 0) {
        std::cout << "dilating mesh boundaries" << std::endl;
        OBJMesh<T> mesh_displaced(mesh);
        for (int v = 1; v < depth_img.rows - 1; v++) {
            for (int u = 1; u < depth_img.cols - 1; u++) {
                displace(inds, mesh, mesh_displaced, u, v, occlusion_threshold, displacement);
            }
        }
        mesh = mesh_displaced;
    }

    std::cout << "populating indices" << std::endl;
    for (int v=0; v<depth_img.rows-1; v++) {
        for (int u=0; u<depth_img.cols-1;u++) {
            int i00 = (int) round(inds.at<T>(v, u));
            int i01 = (int) round(inds.at<T>(v, u+1));
            int i10 = (int) round(inds.at<T>(v+1, u));
            int i11 = (int) round(inds.at<T>(v+1, u+1));
            if (i00 <= 0 || i01 <= 0 || i10 <= 0 || i11 <= 0) continue;
            //compute bounds

            T depth00 = depth_img.at<T>(v, u);
            T depth01 = depth_img.at<T>(v, u+1);
            T depth10 = depth_img.at<T>(v+1, u);
            T depth11 = depth_img.at<T>(v+1, u+1);

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
            mesh.AddTri(Eigen::Vector3i(i00, i11, i01));
            mesh.AddTri(Eigen::Vector3i(i00, i10, i11));

        }
    }

    return mesh;
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
OBJMesh<T> createMesh(const cv::Mat &depth_img, T max_depth, T occlusion_threshold, T displacement=0);

template <>
OBJMesh<float> createMesh<float>(const cv::Mat &depth_img, float max_depth, float occlusion_threshold, float displacement) {
    std::cout << "using single precision" << std::endl;
    return createMesh<float>(depth_img, max_depth, occlusion_threshold, CV_32FC1, displacement);
}

template <>
OBJMesh<double> createMesh<double>(const cv::Mat &depth_img, double max_depth, double occlusion_threshold, double displacement) {
    std::cout << "using double precision" << std::endl;
    return createMesh<double>(depth_img, max_depth, occlusion_threshold, CV_64FC1, displacement);
}

#endif
