#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

int main(int argc, char** argv) {
    std::string filename = "/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/maxdepth100/Depth38809_Theta165_Phi48_S.exr";
    double max_depth = 1;
    if (argc > 1) {
        filename = argv[1];
        if (argc > 2) {
            max_depth = std::stoi(argv[2]);
            std::cout << "arg max_depth: " << max_depth << std::endl;
        }
    } else {
        std::cout << "Usage: " << argv[0] << " filename [max_depth]" << std::endl;
    }


    //cv::Mat depth_img = cv::imread("/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/tabledepth.png");
    cv::Mat depth_img = cv::imread(filename, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    //cv::Mat depth_img = cv::imread("/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/Depth57429_Theta334_Phi47.exr", cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    depth_img = max_depth-depth_img;
    cv::resize(depth_img, depth_img, cv::Size(0, 0), 0.25, 0.25);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    /*cv::namedWindow("Display Window");
    cv::imshow("Display Window", depth_img);
    cv::waitKey(0); */

    double phi = 47.0 / 180 * M_PI;
    double theta = 334.0 / 180 * M_PI;
    double fov = 38.58 / 180 * M_PI;

    double cx = depth_img.cols / 2.0;
    double cy = depth_img.rows / 2.0;

    double radius = 26;
    double camX = radius * cos(theta) * cos(phi);
    double camY = radius * sin(phi);
    double camZ = radius * sin(theta) * cos(phi);

    Eigen::Matrix<double, 3, 1> center(0, 1, 0);
    Eigen::Matrix<double, 3, 1> up(0, 1, 0);
    Eigen::Matrix<double, 3, 1> eye(camX, camY, camZ);

    double constant_x = 2 * tan(fov/2.0) / depth_img.cols;
    double constant_y = constant_x; //assume uniform pinhole model

    double greatest_z = std::numeric_limits<double>::min();
    double smallest_z = std::numeric_limits<double>::max();
    double greatest_y = std::numeric_limits<double>::min();
    double smallest_y = std::numeric_limits<double>::max();
    int index = 1;
    cv::Mat inds(depth_img.rows, depth_img.cols, CV_64FC1);
    int discarded = 0;
    std::ofstream of("../output_mesh.obj");
    for (int v=0; v<depth_img.rows; v++) {
        for (int u=0; u<depth_img.cols;u++) {
            double depth = depth_img.at<float>(v, u);
            if (depth >= max_depth) {
                discarded++;
                inds.at<double>(v, u) = -1;
                continue;
            }
            greatest_z = std::max(greatest_z, depth);
            smallest_z = std::min(smallest_z, depth);
            double px = (u - cx) * depth  * constant_x;
            double py = (v - cy) * depth  * constant_y;
            greatest_y = std::max(greatest_y, py);
            smallest_y = std::min(smallest_y, py);
            of << "v " << -px << ' ' << -py << ' ' << depth << std::endl;
            inds.at<double>(v, u) = index;
            index++;
        }
    }

    std::cout << "maximum depth: " << greatest_z << "; smallest depth: " << smallest_z << std::endl;
    std::cout << "maximum y: " << greatest_y << "; smallest y: " << smallest_y << std::endl;

    std::cout << "processing " << depth_img.cols*depth_img.rows - discarded << " points (discarded " << discarded << ")" << std::endl;


    for (int v=0; v<depth_img.rows-1; v++) {
        for (int u=0; u<depth_img.cols-1;u++) {
            int i00 = (int) round(inds.at<double>(v, u));
            int i01 = (int) round(inds.at<double>(v, u+1));
            int i10 = (int) round(inds.at<double>(v+1, u));
            int i11 = (int) round(inds.at<double>(v+1, u+1));
            if (i00 <= 0 || i01 <= 0 || i10 <= 0 || i11 <= 0) continue;
            of << "f " << i00 << ' ' << i11 << ' ' << i01 << std::endl;
            of << "f " << i00 << ' ' << i10 << ' ' << i11 << std::endl;
        }
    }

    std::cout << "finished creating mesh" << std::endl;

    //system("mitsuba /Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/depth_grid_render/scene.xml");

    return 0;
}