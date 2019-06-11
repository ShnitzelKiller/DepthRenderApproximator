#ifndef MESHBUILDER_H
#define MESHBUILDER_H

#include <opencv2/opencv.hpp>
#include "OBJWriter.hpp"
#include "CameraUtils.hpp"
#include <string>
#include "typedefs.hpp"

template <class T>
int ransac(const OBJMesh<T> &mesh, std::default_random_engine &generator, Vector3<T> &p, Vector3<T> &n) {
  size_t N = mesh.GetNumVertices();
  if (N > 0) {
    std::uniform_int_distribution<size_t> dis(1, N);
    Vector3<T> v0 = mesh.GetVertex((int) dis(generator));
    Vector3<T> v1 = mesh.GetVertex((int) dis(generator));
    Vector3<T> v2 = mesh.GetVertex((int) dis(generator));
    n = (v1-v0).cross(v2-v0).normalized();
    p = (v0+v1+v2)/3.0f;
    Matrix3X<T> allverts(3, N);
    //Matrix3X<T> allnorms(3, N);
    for (int i=0; i < N; i++) {
      allverts.col(i) = mesh.GetVertex(i+1);
    }
    /*
    for (int i=0; i < N; i++) {
      allnorms.col(i) = mesh.GetNormal(i+1);
      }*/
    const T threshold = 1.0f;
    int inliers;
    for (int i=0; i < 10; i++) {
      inliers=0;
      std::vector<int> indices;
      Eigen::ArrayXf errors = n.transpose() * (allverts.colwise() - p);
      //Eigen::ArrayXf nerrors = n.transpose() * allnorms;
      p = Vector3<T>::Zero(3);
      n = Vector3<T>::Zero(3);
      for (int j=0; j<N; j++) {
	if (std::fabs(errors(j)) < threshold) {
	  p += allverts.col(j);
	  indices.push_back(j);
	  inliers++;
	}
      }
      p /= inliers;
      Matrix3X<T> inlierm(3, inliers);
      for (int j=0; j<inliers; j++) {
	inlierm.col(j) = allverts.col(indices[j]);
      }
      inlierm = inlierm.colwise() - p;
      //build covariance matrix
      Matrix3X<T> cov = inlierm * inlierm.transpose() / inliers;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> pca(cov);
      Vector3<T> eigenvalues = pca.eigenvalues();
      Matrix3X<T> eigenvectors = pca.eigenvectors();
      //std::cout << "eigenvalues: " << eigenvalues[0] << " " << eigenvalues[1] << " " << eigenvalues[2] << std::endl;
      Vector3<T> e1;
      Vector3<T> e2;
      T v1=std::numeric_limits<T>::lowest();
      T v2 = v1;
      for (int j=0; j<3; j++) {
	if (eigenvalues[j] > v1) {
	  v2 = v1;
	  e2 = e1;
	  v1 = eigenvalues[j];
	  e1 = eigenvectors.col(j);
	} else if (eigenvalues[j] > v2) {
	  v2 = eigenvalues[j];
	  e2 = eigenvectors.col(j);
	}
      }
      n = e1.cross(e2).normalized();
	      
      //std::cout << "inliers: " << inliers << std::endl;
    }
    n.normalize();
    return inliers;
  } else {
    return 0;
  }
}

template <class T, class PredFun>
cv::Mat createMask(const cv::Mat &depth_img, T min_depth, T max_depth, PredFun f, T correction_factor=1, bool range_correction=true, T fov=45);

template <class T, class PredFun>
cv::Mat createMask(const cv::Mat &depth_img, T min_depth, T max_depth, PredFun f, T correction_factor, bool range_correction, T fov) {
    fov = fov / 180 * M_PI;
    const T cx = depth_img.cols / 2.0f;
    const T cy = depth_img.rows / 2.0f;
    const T constant_x = 2 * (tan(fov/static_cast<T>(2))) / depth_img.cols;
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
	    if (range_correction)
	      depth /= sqrt(1+px*px+py*py);
	    depth *= correction_factor;
            px *= depth;
            py *= depth;
            if (f(Vector3<T>(px, py, -depth))) {
                mask.at<cv::Vec3b>(v, u) = cv::Vec3b(255,255,255);
            }
        }
    }
    return mask;
}


std::string type2str(int type) {
  using namespace cv;
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
  case CV_8U:  r = "8U"; break;
  case CV_8S:  r = "8S"; break;
  case CV_16U: r = "16U"; break;
  case CV_16S: r = "16S"; break;
  case CV_32S: r = "32S"; break;
  case CV_32F: r = "32F"; break;
  case CV_64F: r = "64F"; break;
  default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

namespace {
    template<class T>
    OBJMesh<T> createMesh(const cv::Mat depth_img, T min_depth, T max_depth, T occlusion_threshold, int format, T correction_factor, bool range_correction, T fov) {
        fov = fov / 180 * M_PI;
        const T cx = depth_img.cols / 2.0f;
        const T cy = depth_img.rows / 2.0f;
        const T constant_x = 2 * (tan(fov / static_cast<T>(2))) / depth_img.cols;
        const T constant_y = constant_x; //assume uniform pinhole model

        T greatest_z = std::numeric_limits<T>::lowest();
        T smallest_z = std::numeric_limits<T>::max();
        T greatest_y = std::numeric_limits<T>::lowest();
        T smallest_y = std::numeric_limits<T>::max();
        int index = 1;
	using namespace cv;
        Mat inds(depth_img.rows, depth_img.cols, format);
        int discarded = 0;
        OBJMesh<T> mesh;
	uchar mattype = depth_img.type() & CV_MAT_DEPTH_MASK;

	std::cout << "element type: " << type2str(mattype) << std::endl;

        for (int v = 0; v < depth_img.rows; v++) {
            for (int u = 0; u < depth_img.cols; u++) {
	      T depth;
	      switch(mattype) {
	      case CV_8U:
		depth = depth_img.at<uchar>(v, u); break;
	      default:
		depth = depth_img.at<T>(v, u); break;
	      }
	      //std::cout << "depth: " << depth << std::endl;
	      if (depth >= max_depth || depth <= min_depth) {
                    discarded++;
                    inds.at<T>(v, u) = -1;
                    continue;
	      }

                T px = (u - cx) * constant_x;
                T py = -(v - cy) * constant_y;
		if (range_correction)
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
	int discarded2 = 0;
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
		
                /*if (maxdepth - mindepth > occlusion_threshold) {
		  discarded2++;
                    continue;
		    }*/
                Eigen::Vector3i tri1(i00, i11, i01);
                Eigen::Vector3i tri2(i00, i10, i11);
                mesh.AddTri(tri1);
                mesh.AddTri(tri2);

            }
        }
	std::cout << "discarded " << discarded2 << " faces based on occlusion threshold" << std::endl;
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
OBJMesh<T> createMesh(const cv::Mat depth_img, T min_depth, T max_depth, T occlusion_threshold, T correction_factor=1, bool range_correction=true, T fov=45);

template <>
OBJMesh<float> createMesh<float>(const cv::Mat depth_img, float min_depth, float max_depth, float occlusion_threshold, float correction_factor, bool range_correction, float fov) {
    std::cout << "using single precision" << std::endl;
    return createMesh<float>(depth_img, min_depth, max_depth, occlusion_threshold, CV_32FC1, correction_factor, range_correction, fov);
}

template <>
OBJMesh<double> createMesh<double>(const cv::Mat depth_img, double min_depth, double max_depth, double occlusion_threshold, double correction_factor, bool range_correction, double fov) {
    std::cout << "using double precision" << std::endl;
    return createMesh<double>(depth_img, min_depth, max_depth, occlusion_threshold, CV_64FC1, correction_factor, range_correction, fov);
}

#endif
