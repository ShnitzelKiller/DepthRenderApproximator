#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include "CameraUtils.hpp"
#include "XMLWriter.hpp"
#include "OBJWriter.hpp"
#include <sstream>
#include "OptionParser.hpp"

void usage(char* program_name) {
    std::cout << "Usage: " << program_name << " filename envmap theta phi alpha [-ltheta <value> -lphi <value>] [-c <occlusion_threshold>] [-d <displacement_factor>] [-s <scene_format_version>]" << std::endl;
}

std::shared_ptr<XMLElement> buildScene(std::string envmap, Eigen::Matrix<float, 4, 4> const &fromWorld, float alpha, std::string scene_version = "0.6.0", bool pointlight = false, Eigen::Vector3d light_pos = Eigen::Vector3d()) {
    using namespace std;

    ostringstream mat;
    mat << fromWorld(0, 0) << " " << fromWorld(0, 1) << " " << fromWorld(0, 2) << " " << fromWorld(0, 3) << " "
        << fromWorld(1, 0) << " " << fromWorld(1, 1) << " " << fromWorld(1, 2) << " " << fromWorld(1, 3) << " "
        << fromWorld(2, 0) << " " << fromWorld(2, 1) << " " << fromWorld(2, 2) << " " << fromWorld(2, 3) << " "
        << fromWorld(3, 0) << " " << fromWorld(3, 1) << " " << fromWorld(3, 2) << " " << fromWorld(3, 3);

    auto scene = make_shared<XMLElement>("scene");
    scene->AddProperty("version", std::move(scene_version));

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

template <typename T>
void displace(cv::Mat &inds, const OBJMesh<T> &mesh, OBJMesh<T> &outMesh, int u, int v, T maxdist, T fac) {
    int index = (int) round(inds.at<float>(v, u));
    if (index < 1) return;
    const Vector3<T> vert = mesh.GetVertex(index);
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

int main(int argc, char** argv) {
    std::string filename = "/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/maxdepth100/Depth00002_Theta228_Phi52_ALL.exr";
    std::string envmap = "/bugger.exr";
    const std::string mesh_path = "output_mesh.obj";
    const std::string scene_path = "scene_gen.xml";
    const float scale_factor = 0.5;
    float phi = 0;
    float theta = 0;
    float alpha = 0;
    float light_theta = 0;
    float light_phi = 0;
    const float light_radius = 26;
    float occlusion_threshold = 1;
    const float max_depth = 100;
    float displacement = 0;
    bool light = false;
    std::string scene_version("0.6.0");

    //parse arguments

    OptionParser parser(argc, argv);
    std::cout << parser.getNumArguments() << " arguments" << std::endl;
    if (parser.getNumArguments() != 5) {
        usage(argv[0]);
        return 0;
    }
    filename = parser.getPositionalArgument(0);
    envmap = parser.getPositionalArgument(1);
    theta = std::stof(parser.getPositionalArgument(2)) / 180.0f * (float) M_PI;
    phi = std::stof(parser.getPositionalArgument(3)) / 180.0f * (float) M_PI;
    alpha = std::stof(parser.getPositionalArgument(4)) / 10000.0f;

    if (parser.cmdOptionExists("ltheta") && parser.cmdOptionExists("lphi")) {
        light = true;
        light_theta = std::stof(parser.getCmdOption("ltheta")) / 180.0f * (float) M_PI;
        light_phi = std::stof(parser.getCmdOption("lphi")) / 180.0f * (float) M_PI;
        std::cout << "using point light with theta " << parser.getCmdOption("ltheta") << " and phi " << parser.getCmdOption("lphi") << std::endl;
    }

    if (parser.cmdOptionExists("c")) {
        occlusion_threshold = std::stof(parser.getCmdOption("c"));
        std::cout << "occlusion threshold: " << occlusion_threshold << std::endl;
    }

    if (parser.cmdOptionExists("d")) {
        displacement = std::stof(parser.getCmdOption("d"));
        if (displacement > 0) {
            std::cout << "displacing occlusion boundaries by " << displacement << " units" << std::endl;
        }
    }

    if (parser.cmdOptionExists("s")) {
        scene_version = parser.getCmdOption("s");
        std::cout << "using scene version " << scene_version << std::endl;
    }

    cv::Mat depth_img = cv::imread(filename, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    if (!depth_img.data) {
      std::cout << "image not found: " << filename << std::endl;
      return 1;
    }
    depth_img = max_depth * (1-depth_img);
    cv::resize(depth_img, depth_img, cv::Size(0, 0), scale_factor, scale_factor);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    const float fov = 45.0f / 180 * ((float) M_PI);

    const float cx = depth_img.cols / 2.0f;
    const float cy = depth_img.rows / 2.0f;

    const float radius = 26;
    const float camZ = radius * sin(theta) * cos(phi);
    const float camX = radius * cos(theta) * cos(phi);
    const float camY = radius * sin(phi);

    const float lightZ = light_radius * sin(light_theta) * cos(light_phi);
    const float lightX = light_radius * cos(light_theta) * cos(light_phi);
    const float lightY = light_radius * sin(light_phi);

    const Eigen::Matrix<float, 3, 1> center(0, 1, 0);
    const Eigen::Matrix<float, 3, 1> up(0, 1, 0);
    const Eigen::Matrix<float, 3, 1> eye(camX, camY, camZ);

    Eigen::Matrix<float, 4, 4> lookat = geom::lookAt(eye, center, up).inverse();

    float constant_x = 2 * ((float)tan(fov/2.0)) / depth_img.cols;
    float constant_y = constant_x; //assume uniform pinhole model

    float greatest_z = std::numeric_limits<float>::min();
    float smallest_z = std::numeric_limits<float>::max();
    float greatest_y = std::numeric_limits<float>::min();
    float smallest_y = std::numeric_limits<float>::max();
    int index = 1;
    cv::Mat inds(depth_img.rows, depth_img.cols, CV_32FC1);
    int discarded = 0;
    
    std::ofstream of(mesh_path);
    OBJMesh<float> mesh;

    for (int v=0; v<depth_img.rows; v++) {
        for (int u=0; u<depth_img.cols;u++) {
            float depth = depth_img.at<float>(v, u);
            if (depth >= max_depth) {
                discarded++;
                inds.at<float>(v, u) = -1;
                continue;
            }
            float py = (v - cy) * depth  * constant_y;

            greatest_z = std::max(greatest_z, depth);
            smallest_z = std::min(smallest_z, depth);
            greatest_y = std::max(greatest_y, py);
            smallest_y = std::min(smallest_y, py);
            float px = (u - cx) * depth  * constant_x;

            Eigen::Vector3f point(-px, -py, depth);
            mesh.AddVertex(point);

            inds.at<float>(v, u) = index;
            index++;
        }
    }

    std::cout << "maximum depth: " << greatest_z << "; smallest depth: " << smallest_z << std::endl;
    std::cout << "maximum y: " << greatest_y << "; smallest y: " << smallest_y << std::endl;

    std::cout << "processing " << depth_img.cols*depth_img.rows - discarded << " points (discarded " << discarded << ")" << std::endl;

    if (displacement > 0) {
        std::cout << "dilating mesh boundaries" << std::endl;
        OBJMesh<float> mesh_displaced(mesh);
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
            if (maxdepth - mindepth > occlusion_threshold) {
                continue;
            }
            mesh.AddTri(Eigen::Vector3i(i00, i11, i01));
            mesh.AddTri(Eigen::Vector3i(i00, i10, i11));

        }
    }

    mesh.SaveOBJ(of);
    //mesh.SaveOBJ(of);

    std::cout << "finished creating mesh " << mesh_path << std::endl;
    of.close();

    auto scene = buildScene(envmap, lookat, alpha, scene_version, light, Eigen::Vector3d(lightX, lightY, lightZ));
    std::ofstream sceneof(scene_path);
    scene->SaveXML(sceneof);
    sceneof.close();

    std::cout << "wrote scene file to " << scene_path << std::endl;
    return 0;
}
