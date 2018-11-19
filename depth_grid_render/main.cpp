#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include "CameraUtils.hpp"
#include "XMLWriter.hpp"
#include "OBJWriter.hpp"
#include <sstream>
#include "OptionParser.hpp"
#include "MeshBuilder.hpp"
#include <math.h>
#include <limits>

void usage(char* program_name) {
    std::cout << "Usage: " << program_name << " filename envmap theta phi alpha [-ltheta <value> -lphi <value>] [-c <occlusion_threshold>] [-d <displacement_factor>] [-s <scene_format_version>] [-r <resize_factor>]" << std::endl;
}

std::shared_ptr<XMLElement> buildScene(std::string envmap, float alpha, const Eigen::Vector3f &camOrigin, const Eigen::Vector3f &planeOrigin, const Eigen::Vector2f &planeScale, std::string scene_version = "0.6.0", bool pointlight = false, Eigen::Vector3d light_pos = Eigen::Vector3d(), std::string meshTexture="") {
    using namespace std;
    ostringstream eye;
    eye << camOrigin[0] << ", " << camOrigin[1] << ", " << camOrigin[2];

    auto scene = make_shared<XMLElement>("scene");
    scene->AddProperty("version", std::move(scene_version));

    auto camera = make_shared<XMLElement>("sensor", "perspective");
    auto sampler = make_shared<XMLElement>("sampler", "ldsampler");
    sampler->AddChild(make_shared<XMLElement>("integer", "sampleCount", "128"));
    camera->AddChild(sampler);
    camera->AddChild(make_shared<XMLElement>("float", "fov", "45"));
    auto film = make_shared<XMLElement>("film", "hdrfilm");
    film->AddChild(make_shared<XMLElement>("integer", "width", "960"));
    film->AddChild(make_shared<XMLElement>("integer", "height", "540"));
    film->AddChild(make_shared<XMLElement>("boolean", "banner", "false"));
    camera->AddChild(film);
    auto cam_trans = make_shared<XMLElement>("transform");
    cam_trans->AddProperty("name", "toWorld");
    auto look_at = make_shared<XMLElement>("lookat");
    look_at->AddProperty("origin", eye.str());
    look_at->AddProperty("target", "0, 1, 0");
    look_at->AddProperty("up", "0, 1, 0");
    cam_trans->AddChild(look_at);
    camera->AddChild(cam_trans);

    auto shape = make_shared<XMLElement>("shape", "obj");
    shape->AddChild(make_shared<XMLElement>("string", "filename", "output_mesh.obj"));
    auto bsdf = make_shared<XMLElement>("bsdf", "diffuse");
    bsdf->AddChild(make_shared<XMLElement>("spectrum", "reflectance", "1"));
    if (!meshTexture.empty()) {
        auto texture = make_shared<XMLElement>("texture", "bitmap");
        texture->AddProperty("name", "diffuseReflectance");
        texture->AddChild(make_shared<XMLElement>("string", "filename", meshTexture));
        bsdf->AddChild(texture);
    }
    shape->AddChild(bsdf);

    auto plane = make_shared<XMLElement>("shape", "rectangle");
    auto plane_trans = make_shared<XMLElement>("transform");
    plane_trans->AddProperty("name", "toWorld");
    auto plane_scale = make_shared<XMLElement>("scale");
    plane_scale->AddProperty("x", std::to_string(planeScale[0]));
    plane_scale->AddProperty("y", std::to_string(planeScale[1]));
    auto plane_rotate = make_shared<XMLElement>("rotate");
    plane_rotate->AddProperty("x", "1");
    plane_rotate->AddProperty("angle", "-90");
    auto plane_translate = make_shared<XMLElement>("translate");
    plane_translate->AddProperty("x", std::to_string(planeOrigin[0]));
    plane_translate->AddProperty("y", std::to_string(planeOrigin[1]));
    plane_translate->AddProperty("z", std::to_string(planeOrigin[2]));
    plane_trans->AddChild(plane_scale);
    plane_trans->AddChild(plane_rotate);
    plane_trans->AddChild(plane_translate);
    plane->AddChild(plane_trans);
    auto plane_bsdf = make_shared<XMLElement>("bsdf", "roughplastic");
    plane_bsdf->AddChild(make_shared<XMLElement>("float", "alpha", to_string(alpha)));
    plane->AddChild(plane_bsdf);

    auto emitter = make_shared<XMLElement>("emitter", "envmap");
    emitter->AddChild(make_shared<XMLElement>("string", "filename", envmap));

    scene->AddChild(camera);
    scene->AddChild(shape);
    scene->AddChild(emitter);
    scene->AddChild(plane);

    if (pointlight) {
        auto light = make_shared<XMLElement>("emitter", "point");
        auto light_trans = make_shared<XMLElement>("transform");
        light_trans->AddProperty("name", "toWorld");
        auto translate = make_shared<XMLElement>("translate");
        translate->AddProperty("x", std::to_string(light_pos[0]));
        translate->AddProperty("y", std::to_string(light_pos[1]));
        translate->AddProperty("z", std::to_string(light_pos[2]));
        light_trans->AddChild(translate);
        light->AddChild(light_trans);
	light->AddChild(make_shared<XMLElement>("spectrum", "intensity", "100"));
        scene->AddChild(light);
    }

    return scene;
}

int main(int argc, char** argv) {
    std::string filename = "/Users/jamesnoeckel/Documents/C++sandbox/points_from_depth/data/maxdepth100/Depth00002_Theta228_Phi52_ALL.exr";
    std::string envmap = "/bugger.exr";
    const std::string mesh_path = "output_mesh.obj";
    const std::string scene_path = "scene_gen.xml";
    float scale_factor = 0.5;
    const float floorEps = 3e-2;
    const std::string textured_scene_path = "scene_gen_tex.xml";
    const std::string texture_image = "texture.exr";
    float phi = 0;
    float theta = 0;
    float alpha = 0;
    float light_theta = 0;
    float light_phi = 0;
    const float light_radius = 26;
    float occlusion_threshold = 1;
    const float max_depth = 100;
    const float min_depth = 1;
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
	    std::cout << "WARNING: this looks terrible and doesn't work" << std::endl;
        }
    }

    if (parser.cmdOptionExists("s")) {
        scene_version = parser.getCmdOption("s");
        std::cout << "using scene version " << scene_version << std::endl;
    }

    if (parser.cmdOptionExists("r")) {
        scale_factor = std::stof(parser.getCmdOption("r"));
        std::cout << "scale factor: " << scale_factor << std::endl;
    }

    cv::Mat depth_img = cv::imread(filename, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    if (!depth_img.data) {
      std::cout << "image not found: " << filename << std::endl;
      return 1;
    }
    //depth_img = max_depth * (1-depth_img); //if transformation is needed
    cv::resize(depth_img, depth_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    OBJMesh<float> mesh = createMesh(depth_img, min_depth, max_depth, occlusion_threshold);

    std::cout << "finished creating mesh " << std::endl;

    std::cout << "generating scene" << std::endl;
    const float radius = 26;
    const Eigen::Matrix<float, 3, 1> center(0, 1, 0);
    const Eigen::Matrix<float, 3, 1> up(0, 1, 0);
    const float camZ = radius * sin(theta) * cos(phi);
    const float camX = radius * cos(theta) * cos(phi);
    const float camY = radius * sin(phi);
    const Eigen::Matrix<float, 3, 1> eye(camX, camY, camZ);
    Eigen::Matrix<float, 4, 4> camToWorld = geom::lookAt(eye, center, up);

    std::cout << "camera position: " << std::endl << eye << std::endl;
    std::cout << "transforming mesh" << std::endl;

    std::vector<float> heights;
    std::vector<Eigen::Vector3f> planePoints;
    const size_t n = mesh.GetNumVertices();
    for (int i=1; i<=n; i++) {
      Vector3<float> &vert = mesh.GetVertex(i);
      Eigen::Vector4f vert4;
      vert4.head(3) = vert;
      vert4[3] = 1;
      vert4 = camToWorld * vert4;
      vert = vert4.head(3);
      planePoints.push_back(vert);
      heights.push_back(vert[1]);
    }

    std::sort(heights.begin(), heights.end());
    const size_t smallIndex = std::max(depth_img.rows, depth_img.cols) * 2;
    const size_t largeIndex = n - 1 - smallIndex;
    const float minHeight = heights[smallIndex];
    std::cout << "deleting below " << minHeight << std::endl;
    const size_t oldSize = mesh.GetNumElements();
    mesh.DeleteBelowY(minHeight + floorEps);
    const size_t newSize = mesh.GetNumElements();
    std::cout << "deleted " << oldSize - newSize << " faces out of " << oldSize << ", leaving " << newSize << std::endl;

    std::vector<float> xs, zs;
    for (const auto &point : planePoints) {
        if (point[1] < minHeight + 3*floorEps && point[1] > minHeight - 3*floorEps) {
            xs.push_back(point[0]);
            zs.push_back(point[2]);
        }
    }
    std::cout << xs.size() << " points in the plane" << std::endl;
    std::sort(xs.begin(), xs.end());
    std::sort(zs.begin(), zs.end());

    const float minX = xs[0];
    const float maxX = xs[xs.size()-1];
    const float minZ = zs[0];
    const float maxZ = zs[zs.size()-1];
    std::cout << "plane dimensions: X [" << minX << ", " << maxX << "], Z [" << minZ << ", " << maxZ << "]" << std::endl;
    const float scaleX = (maxX - minX) / 2;
    const float scaleZ = (maxZ - minZ) / 2;
    const float originX = (maxX + minX) / 2;
    const float originZ = (maxZ + minZ) / 2;
    const Eigen::Vector2f planeScale(scaleX, scaleZ);
    const Eigen::Vector3f planeOrigin(originX, minHeight, originZ);

    std::ofstream of(mesh_path);
    mesh.SaveOBJ(of);
    of.close();    
    
    std::cout << "saved mesh at " << mesh_path << std::endl;

    const float lightZ = light_radius * sin(light_theta) * cos(light_phi);
    const float lightX = light_radius * cos(light_theta) * cos(light_phi);
    const float lightY = light_radius * sin(light_phi);
    auto scene = buildScene(envmap, alpha, eye, planeOrigin, planeScale, scene_version, light, Eigen::Vector3d(lightX, lightY, lightZ));
    std::ofstream sceneof(scene_path);
    scene->SaveXML(sceneof);
    sceneof.close();
    auto texscene = buildScene(envmap, alpha, eye, planeOrigin, planeScale, scene_version, light, Eigen::Vector3d(lightX, lightY, lightZ), texture_image);
    std::ofstream texsceneof(textured_scene_path);
    texscene->SaveXML(texsceneof);
    texsceneof.close();

    std::cout << "wrote scene files to " << std::endl << scene_path << std::endl << textured_scene_path << std::endl;
    return 0;
}
