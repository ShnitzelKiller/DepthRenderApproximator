#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include "CameraUtils.hpp"
#include "XMLWriter.hpp"
#include <sstream>

void usage(char* program_name) {
    std::cout << "Usage: " << program_name << " filename envmap theta phi alpha [light_theta light_phi [occlusion_threshold] [scene_format_version]]" << std::endl;
    exit(0);
}

std::shared_ptr<XMLElement> buildScene(std::string envmap, Eigen::Matrix<double, 4, 4> const &fromWorld, double alpha, std::string scene_version = "0.6.0", bool pointlight = false, Eigen::Vector3d light_pos = Eigen::Vector3d()) {
    using namespace std;

    ostringstream mat;
    mat << fromWorld(0, 0) << " " << fromWorld(0, 1) << " " << fromWorld(0, 2) << " " << fromWorld(0, 3) << " "
        << fromWorld(1, 0) << " " << fromWorld(1, 1) << " " << fromWorld(1, 2) << " " << fromWorld(1, 3) << " "
        << fromWorld(2, 0) << " " << fromWorld(2, 1) << " " << fromWorld(2, 2) << " " << fromWorld(2, 3) << " "
        << fromWorld(3, 0) << " " << fromWorld(3, 1) << " " << fromWorld(3, 2) << " " << fromWorld(3, 3);

    auto scene = make_shared<XMLElement>("scene");
    scene->AddProperty("version", "0.6.0");

    auto camera = make_shared<XMLElement>("sensor", "perspective");
    auto sampler = make_shared<XMLElement>("sampler", "ldsampler");
    sampler->AddChild(make_shared<XMLElement>("integer", "sampleCount", "128"));
    camera->AddChild(sampler);
    camera->AddChild(make_shared<XMLElement>("float", "fov", "45"));
    auto film = make_shared<XMLElement>("film", "ldrfilm");
    film->AddChild(make_shared<XMLElement>("integer", "width", "960"));
    film->AddChild(make_shared<XMLElement>("integer", "height", "540"));
    film->AddChild(make_shared<XMLElement>("boolean", "banner", "false"));
    camera->AddChild(film);

    auto shape = make_shared<XMLElement>("shape", "obj");
    shape->AddChild(make_shared<XMLElement>("string", "filename", "output_mesh.obj"));
    auto bsdf = make_shared<XMLElement>("bsdf", "roughplastic");
    bsdf->AddChild(make_shared<XMLElement>("float", "alpha", to_string(alpha)));
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

    if (pointlight) {
        auto light = make_shared<XMLElement>("emitter", "point");
        auto light_trans = make_shared<XMLElement>("transform");
        light_trans->AddProperty("name", "toWorld");
        auto translate = make_shared<XMLElement>("translate");
        translate->AddProperty("x", std::to_string(light_pos[0]));
        translate->AddProperty("y", std::to_string(light_pos[1]));
        translate->AddProperty("z", std::to_string(light_pos[2]));
        light_trans->AddChild(translate);
	light_trans->AddChild(matrix); //also transform into camera space
        light->AddChild(light_trans);
	light->AddChild(make_shared<XMLElement>("spectrum", "intensity", "100"));
        scene->AddChild(light);
    }

    return scene;
}

int main(int argc, char** argv) {
    std::string filename = "/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/maxdepth100/Depth00002_Theta228_Phi52_ALL.exr";
    std::string envmap = "/bugger.exr";
    const std::string mesh_path = "../output_mesh.obj";
    const std::string scene_path = "../scene_gen.xml";
    const double scale_factor = 0.5;
    double phi = 52.0 / 180 * M_PI;
    double theta = 228.0 / 180 * M_PI;
    double alpha = 0.4;
    double light_theta = 0;
    double light_phi = 0;
    const double light_radius = 26;
    double occlusion_threshold = 1;
    const double max_depth = 100;
    bool light = false;
    std::string scene_version("0.6.0");

    //parse arguments
    if (argc > 5) {
        filename = argv[1];
	envmap = argv[2];
	theta = std::stod(argv[3]) / 180.0 * M_PI;
	phi = std::stod(argv[4]) / 180.0 * M_PI;
	alpha = std::stod(argv[5]) / 10000.0;
	if (argc > 6) {
	    if (argc <= 7) {
	        usage(argv[0]);
	    }
	    light_theta = std::stod(argv[6]) / 180 * M_PI;
	    light_phi = std::stod(argv[7]) / 180 * M_PI;
	    light = true;
	    if (argc > 8) {
	      occlusion_threshold = std::stoi(argv[8]);
	      std::cout << "arg occlusion_threshold: " << occlusion_threshold << std::endl;
	      if (argc > 9) {
		scene_version = argv[9];
	      }
	    }
	}
    } else {
        usage(argv[0]);
	return 0;
    }

    cv::Mat depth_img = cv::imread(filename, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    if (!depth_img.data) {
      std::cout << "image not found: " << filename << std::endl;
      return 1;
    }
    depth_img = max_depth * (1-depth_img);
    cv::resize(depth_img, depth_img, cv::Size(0, 0), scale_factor, scale_factor);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    const double fov = 45.0 / 180 * M_PI;

    const double cx = depth_img.cols / 2.0;
    const double cy = depth_img.rows / 2.0;

    const double radius = 26;
    const double camZ = radius * sin(theta) * cos(phi);
    const double camX = radius * cos(theta) * cos(phi);
    const double camY = radius * sin(phi);

    const double lightZ = light_radius * sin(light_theta) * cos(light_phi);
    const double lightX = light_radius * cos(light_theta) * cos(light_phi);
    const double lightY = light_radius * sin(light_phi);

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
    cv::Mat inds(depth_img.rows, depth_img.cols, CV_32FC1);
    int discarded = 0;
    
    std::ofstream of(mesh_path);
    for (int v=0; v<depth_img.rows; v++) {
        for (int u=0; u<depth_img.cols;u++) {
            double depth = depth_img.at<float>(v, u);
            if (depth >= max_depth) {
                discarded++;
                inds.at<float>(v, u) = -1;
                continue;
            }
            double py = (v - cy) * depth  * constant_y;

            greatest_z = std::max(greatest_z, depth);
            smallest_z = std::min(smallest_z, depth);
            greatest_y = std::max(greatest_y, py);
            smallest_y = std::min(smallest_y, py);
            double px = (u - cx) * depth  * constant_x;

            of << "v " << -px << ' ' << -py << ' ' << depth << std::endl;
            inds.at<float>(v, u) = index;
            index++;
        }
    }

    std::cout << "maximum depth: " << greatest_z << "; smallest depth: " << smallest_z << std::endl;
    std::cout << "maximum y: " << greatest_y << "; smallest y: " << smallest_y << std::endl;

    std::cout << "processing " << depth_img.cols*depth_img.rows - discarded << " points (discarded " << discarded << ")" << std::endl;


    for (int v=0; v<depth_img.rows-1; v++) {
        for (int u=0; u<depth_img.cols-1;u++) {
            int i00 = (int) round(inds.at<float>(v, u));
            int i01 = (int) round(inds.at<float>(v, u+1));
            int i10 = (int) round(inds.at<float>(v+1, u));
            int i11 = (int) round(inds.at<float>(v+1, u+1));
            if (i00 <= 0 || i01 <= 0 || i10 <= 0 || i11 <= 0) continue;
            //compute bounds

            float depth00 = depth_img.at<float>(v, u);
            float depth01 = depth_img.at<float>(v, u+1);
            float depth10 = depth_img.at<float>(v+1, u);
            float depth11 = depth_img.at<float>(v+1, u+1);

            float mindepth = depth00;
            float maxdepth = depth00;

            mindepth = std::min(mindepth, depth01);
            mindepth = std::min(mindepth, depth10);
            mindepth = std::min(mindepth, depth11);
            maxdepth = std::max(maxdepth, depth01);
            maxdepth = std::max(maxdepth, depth10);
            maxdepth = std::max(maxdepth, depth11);
            if (maxdepth - mindepth > occlusion_threshold) continue;
            of << "f " << i00 << ' ' << i11 << ' ' << i01 << std::endl;
            of << "f " << i00 << ' ' << i10 << ' ' << i11 << std::endl;
        }
    }

    std::cout << "finished creating mesh " << mesh_path << std::endl;
    of.close();

    auto scene = buildScene(envmap, lookat, alpha, scene_version, light, Eigen::Vector3d(lightX, lightY, lightZ));
    std::ofstream sceneof(scene_path);
    scene->SaveXML(sceneof);
    sceneof.close();

    std::cout << "wrote scene file to " << scene_path << std::endl;
    return 0;
}
