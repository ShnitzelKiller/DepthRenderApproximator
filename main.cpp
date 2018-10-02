#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include "CameraUtils.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/compute_average_spacing.h>
#include <map>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef CGAL::First_of_pair_property_map<PointVectorPair> Point_map;
typedef CGAL::Second_of_pair_property_map<PointVectorPair> Vector_map;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

int main() {
    //cv::Mat depth = cv::imread("/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/Depth57429_Theta334_Phi47.exr");
    cv::Mat depth = cv::imread("/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/suzanne.png");
    cv::resize(depth, depth, cv::Size(0, 0), 0.125, 0.125);
    std::cout << "width: " << depth.cols << " height: " << depth.rows << std::endl;

    double phi = 47.0 / 180 * M_PI;
    double theta = 334.0 / 180 * M_PI;
    double fov = 90.0;

    double cx = depth.cols / 2.0;
    double cy = depth.rows / 2.0;

    double radius = 26;
    double camX = radius * cos(theta) * cos(phi);
    double camY = radius * sin(phi);
    double camZ = radius * sin(theta) * cos(phi);

    Eigen::Matrix<double, 3, 1> center(0, 1, 0);
    Eigen::Matrix<double, 3, 1> up(0, 1, 0);
    Eigen::Matrix<double, 3, 1> eye(camX, camY, camZ);

    //Eigen::Matrix<double, 4, 4> T_cam = lookAt(eye, center, up);

    //std::cout << T_cam << std::endl;
    cv::Mat z;
    cv::Mat x(depth.rows, depth.cols, CV_64FC1);
    cv::Mat y(depth.rows, depth.cols, CV_64FC1);

    depth.convertTo(z, CV_64FC1);

    double constant_x = 2 * tan(fov/2.0) / depth.cols;
    double constant_y = constant_x; //assume uniform pinhole model

    for (int v=0; v<z.rows; v++) {
        for (int u=0; u<z.cols;u++) {
            x.at<double>(v, u) = (u - cx) * z.at<double>(v, u) * constant_x;
            y.at<double>(v, u) = (v - cy) * z.at<double>(v, u) * constant_y;
        }
    }

    std::vector<PointVectorPair> points;

    for (int v=0; v<z.rows; v++) {
        for (int u=0; u<z.cols;u++) {
            double px = x.at<double>(v, u);
            double py = y.at<double>(v, u);
            double pz = z.at<double>(v, u);
            Point_3 point(px, py, pz);
            Vector_3 n(0, 1, 0);
            points.push_back(PointVectorPair(point, n));
        }
    }


    std::cout << "estimating normals" << std::endl;
    CGAL::jet_estimate_normals<CGAL::Parallel_tag>(points, 3,
            CGAL::parameters::point_map(Point_map()).normal_map(Vector_map()));
    std::cout << "orienting normals" << std::endl;
    CGAL::mst_orient_normals(points, 3,
            CGAL::parameters::point_map(Point_map()).normal_map(Vector_map()));

    std::cout << "computing average spacing";
    double average_spacing = CGAL::compute_average_spacing<CGAL::Parallel_tag>(points, 6, CGAL::parameters::point_map(Point_map()));
    std::cout << ": " << average_spacing << std::endl;

    std::cout << "computing implicit surface" << std::endl;
    Polyhedron outputMesh;
    if (CGAL::poisson_surface_reconstruction_delaunay(points.begin(), points.end(), Point_map(), Vector_map(), outputMesh, average_spacing)) {
        std::ofstream of("output_mesh.off");
        of << outputMesh;
    } else {
        std::cout << "failed to create surface" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "finished" << std::endl;

    return 0;
}