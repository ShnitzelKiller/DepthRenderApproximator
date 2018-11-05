#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include "CameraUtils.hpp"
#include "XMLWriter.hpp"
#include "UnderscoreString.hpp"
#include <sstream>

#define indices(i, width) (i-1) / width, (i-1) % width

using namespace boost::filesystem;

std::shared_ptr<XMLElement> buildScene(std::string envmap, Eigen::Matrix<double, 4, 4> const &fromWorld, double alpha) {
    using namespace std;

    ostringstream mat;
    mat << fromWorld(0, 0) << " " << fromWorld(0, 1) << " " << fromWorld(0, 2) << " " << fromWorld(0, 3) << " "
        << fromWorld(1, 0) << " " << fromWorld(1, 1) << " " << fromWorld(1, 2) << " " << fromWorld(1, 3) << " "
        << fromWorld(2, 0) << " " << fromWorld(2, 1) << " " << fromWorld(2, 2) << " " << fromWorld(2, 3) << " "
        << fromWorld(3, 0) << " " << fromWorld(3, 1) << " " << fromWorld(3, 2) << " " << fromWorld(3, 3);

    auto scene = make_shared<XMLElement>("scene");
    scene->AddProperty("version", "0.5.0");

    auto camera = make_shared<XMLElement>("sensor", "perspective");
    auto sampler = make_shared<XMLElement>("sampler", "independent");
    sampler->AddChild(make_shared<XMLElement>("integer", "sampleCount", "128"));
    camera->AddChild(sampler);
    camera->AddChild(make_shared<XMLElement>("float", "fov", "45"));
    auto film = make_shared<XMLElement>("film", "hdrfilm");
    film->AddChild(make_shared<XMLElement>("integer", "width", "960"));
    film->AddChild(make_shared<XMLElement>("integer", "height", "540"));
    film->AddChild(make_shared<XMLElement>("boolean", "banner", "false"));
    camera->AddChild(film);

    auto shape = make_shared<XMLElement>("shape", "obj");
    shape->AddChild(make_shared<XMLElement>("string", "filename", "output_mesh.obj"));
    auto bsdf = make_shared<XMLElement>("bsdf", "roughplastic");
    bsdf->AddChild(make_shared<XMLElement>("float", "alpha", to_string(alpha))); //TODO: set this based on filename
    shape->AddChild(bsdf);

    auto emitter = make_shared<XMLElement>("emitter", "envmap");
    emitter->AddChild(make_shared<XMLElement>("string", "filename", envmap));

    auto transform = make_shared<XMLElement>("transform");
    transform->AddProperty("name", "toWorld");
    auto matrix = make_shared<XMLElement>("matrix");
    matrix->AddProperty("value", mat.str());
    transform->AddChild(matrix);
    emitter->AddChild(transform);

    scene->AddChild(camera);
    scene->AddChild(shape);
    scene->AddChild(emitter);

    return scene;
}

int main(int argc, char** argv) {
    std::string filename = "/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/maxdepth100/Depth00002_Theta228_Phi52_ALL.exr";
    const std::string mesh_path = "../output_mesh.obj";
    const std::string scene_path = "../scene_gen.xml";
    const bool render = false;
    const double scale_factor = 0.5;
    const double alpha = 0.4;
    const double occlusion_threshold = 1;

    double max_depth = 100;
    if (argc > 1) {
        filename = argv[1];
        if (argc > 2) {
            max_depth = std::stoi(argv[2]);
            std::cout << "arg max_depth: " << max_depth << std::endl;
        }
    } else {
        std::cout << "Usage: " << argv[0] << " filename [max_depth]" << std::endl;
    }


    //parse filename
    path inpath = filename;
    auto file = inpath.filename();
    std::string filename_string = file.string();
    std::istringstream is(filename_string);
    std::vector<std::string> results((std::istream_iterator<UnderscoreString>(is)), std::istream_iterator<UnderscoreString>());
    for (const auto &result : results) {
        std::cout << result << std::endl;
    }


    cv::Mat depth_img = cv::imread(filename, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    depth_img = max_depth * (1-depth_img);
    cv::resize(depth_img, depth_img, cv::Size(0, 0), scale_factor, scale_factor);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    const double phi = 52.0 / 180 * M_PI;
    const double theta = 228.0 / 180 * M_PI;
    const double fov = 45.0 / 180 * M_PI;

    const double cx = depth_img.cols / 2.0;
    const double cy = depth_img.rows / 2.0;

    const double radius = 26;
    const double camZ = radius * sin(theta) * cos(phi);
    const double camX = radius * cos(theta) * cos(phi);
    const double camY = radius * sin(phi);

    const Eigen::Matrix<double, 3, 1> center(0, 1, 0);
    const Eigen::Matrix<double, 3, 1> up(0, 1, 0);
    const Eigen::Matrix<double, 3, 1> eye(camX, camY, camZ);

    Eigen::Matrix<double, 4, 4> lookat = geom::lookAt(eye, center, up).inverse();

    double constant_x = 2 * tan(fov/2.0) / depth_img.cols;
    double constant_y = constant_x; //assume uniform pinhole model

    double greatest_z = std::numeric_limits<double>::min();
    double smallest_z = std::numeric_limits<double>::max();
    double greatest_y = std::numeric_limits<double>::min();
    double smallest_y = std::numeric_limits<double>::max();
    int index = 1;
    cv::Mat inds(depth_img.rows, depth_img.cols, CV_64FC1);
    int discarded = 0;
    
    std::ofstream of(mesh_path);
    for (int v=0; v<depth_img.rows; v++) {
        for (int u=0; u<depth_img.cols;u++) {
            double depth = depth_img.at<float>(v, u);
            if (depth >= max_depth) {
                discarded++;
                inds.at<double>(v, u) = -1;
                continue;
            }
            double py = (v - cy) * depth  * constant_y;

            greatest_z = std::max(greatest_z, depth);
            smallest_z = std::min(smallest_z, depth);
            greatest_y = std::max(greatest_y, py);
            smallest_y = std::min(smallest_y, py);
            double px = (u - cx) * depth  * constant_x;

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
            //compute bounds
            /*
            double mindepth = depth_img.at<double>(indices(i00, depth_img.rows));
            double maxdepth = depth_img.at<double>(indices(i00, depth_img.rows));
            mindepth = std::min(mindepth, depth_img.at<double>(indices(i01, depth_img.rows)));
            mindepth = std::min(mindepth, depth_img.at<double>(indices(i11, depth_img.rows)));
            mindepth = std::min(mindepth, depth_img.at<double>(indices(i10, depth_img.rows)));
            maxdepth = std::max(maxdepth, depth_img.at<double>(indices(i01, depth_img.rows)));
            maxdepth = std::max(maxdepth, depth_img.at<double>(indices(i11, depth_img.rows)));
            maxdepth = std::max(maxdepth, depth_img.at<double>(indices(i10, depth_img.rows)));
            if (std::fabs(mindepth - maxdepth) > occlusion_threshold) continue; */
            of << "f " << i00 << ' ' << i11 << ' ' << i01 << std::endl;
            of << "f " << i00 << ' ' << i10 << ' ' << i11 << std::endl;
        }
    }

    std::cout << "finished creating mesh " << mesh_path << std::endl;
    of.close();

    auto scene = buildScene("/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/hdr_maps/canada_montreal_thea.exr", lookat, alpha);
    std::ofstream sceneof(scene_path);
    scene->SaveXML(sceneof);
    sceneof.close();

    if (render) {
        path outpath = "../output/";
        outpath /= file;
        outpath += ".exr";
        system(("mitsuba " + scene_path + " -o " + (outpath).string()).c_str());
    }

    return 0;
}
