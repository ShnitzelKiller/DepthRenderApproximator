#include <iostream>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

bool endsWith(const std::string& a, const std::string& b) {
    if (b.size() > a.size()) return false;
    return std::equal(b.rbegin(), b.rend(), a.rbegin());
}
bool isHdr(const std::string& filename) {
    return endsWith(filename, ".exr") || endsWith(filename, ".hdr");
}
int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Usage: quotient_image image1.exr image2.exr quotient.exr" << std::endl;
        return 0;
    }
    cv::Mat im1 = cv::imread(argv[1], cv::IMREAD_ANYDEPTH | cv::IMREAD_COLOR);
    if (!isHdr(argv[1])) {
        im1.convertTo(im1, CV_32F);
        im1 /= 255;
    }

    cv::Mat im2 = cv::imread(argv[2], cv::IMREAD_ANYDEPTH | cv::IMREAD_COLOR);
    if (!isHdr(argv[2])) {
        im2.convertTo(im2, CV_32F);
        im2 /= 255;
    }

    cv::Mat quot = im1/im2;
    quot = cv::min(quot, 1);
    if (!isHdr(argv[3])) {
        quot *= 255;
        quot.convertTo(quot, CV_8U);
    }
    cv::imwrite(argv[3], quot);
}
