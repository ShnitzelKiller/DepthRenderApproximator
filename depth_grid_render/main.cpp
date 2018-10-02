#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

int main(int argc, char** argv) {

    double max_depth = 150;
    if (argc > 1) {
        max_depth = std::stoi(argv[1]);
        std::cout << "arg max_depth: " << max_depth << std::endl;
    }

    cv::Mat depth_img = cv::imread("/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/suzanne.png");
    cv::resize(depth_img, depth_img, cv::Size(0, 0), 0.5, 0.5);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    double phi = 47.0 / 180 * M_PI;
    double theta = 334.0 / 180 * M_PI;
    double fov = 90.0;

    double cx = depth_img.cols / 2.0;
    double cy = depth_img.rows / 2.0;

    double radius = 26;
    double camX = radius * cos(theta) * cos(phi);
    double camY = radius * sin(phi);
    double camZ = radius * sin(theta) * cos(phi);

    Eigen::Matrix<double, 3, 1> center(0, 1, 0);
    Eigen::Matrix<double, 3, 1> up(0, 1, 0);
    Eigen::Matrix<double, 3, 1> eye(camX, camY, camZ);

    cv::Mat z;
    cv::Mat x(depth_img.rows, depth_img.cols, CV_64FC1);
    cv::Mat y(depth_img.rows, depth_img.cols, CV_64FC1);

    depth_img.convertTo(z, CV_64FC1);

    double constant_x = 2 * tan(fov/2.0) / depth_img.cols;
    double constant_y = constant_x; //assume uniform pinhole model
    double greatest_z = 0;
    double smallest_z = 1000;
    for (int v=0; v<z.rows; v++) {
        for (int u=0; u<z.cols;u++) {
            double depth = z.at<double>(v, u);
            x.at<double>(v, u) = (u - cx) * depth * constant_x;
            y.at<double>(v, u) = (v - cy) * depth * constant_y;
            if (depth > greatest_z) greatest_z = depth;
            if (depth < smallest_z) smallest_z = depth;
        }
    }

    std::cout << "maximum depth: " << greatest_z << "; smallest depth: " << smallest_z << std::endl;

    cv::Mat inds(depth_img.rows, depth_img.cols, CV_64FC1);
    int discarded = 0;
    std::ofstream of("../output_mesh.obj");

    int index = 0;
    for (int v=0; v<z.rows; v++) {
        for (int u=0; u<z.cols;u++) {
            if (z.at<double>(v, u) > max_depth) {
                discarded++;
                inds.at<double>(v, u) = -1;
                continue;
            }
            double px = x.at<double>(v, u);
            double py = y.at<double>(v, u);
            double pz = z.at<double>(v, u);
            of << "v " << px << ' ' << py << ' ' << pz << std::endl;
            inds.at<double>(v, u) = index;
            index++;
        }
    }
    std::cout << "processing " << z.cols*z.rows - discarded << " points (discarded " << discarded << ")" << std::endl;


    for (int v=0; v<z.rows-1; v++) {
        for (int u=0; u<z.cols-1;u++) {
            int i00 = (int) inds.at<double>(v, u);
            int i01 = (int) inds.at<double>(v, u+1);
            int i10 = (int) inds.at<double>(v+1, u);
            int i11 = (int) inds.at<double>(v+1, u+1);
            if (i00 < 0 || i01 < 0 || i10 < 0 || i11 < 0) continue;
            of << "f " << i00 << ' ' << i01 << ' ' << i11 << std::endl;
            of << "f " << i00 << ' ' << i11 << ' ' << i10 << std::endl;
        }
    }

    std::cout << "finished" << std::endl;

    return 0;
}