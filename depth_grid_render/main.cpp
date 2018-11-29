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
    std::cout << "Usage: " << program_name << " filename envmap theta phi alpha maskfilename [-ltheta <value> -lphi <value>] [-c <occlusion_threshold>] [-d <displacement_factor>] [-s <scene_format_version>] [-r <resize_factor>] [-rand <angle_randomness_magnitude_in_degrees>]" << std::endl;
}

std::shared_ptr<XMLElement> buildScene(int width, int height, std::string envmap, float alpha, const Eigen::Vector3f &camOrigin, float plane_height, std::string scene_version = "0.6.0", bool pointlight = false, Eigen::Vector3f light_pos = Eigen::Vector3f(), std::string meshPath="", std::string meshTexture="", Eigen::Vector3f random_axis=Eigen::Vector3f(), float random_angle=0, Eigen::Vector3f random_axis_light=Eigen::Vector3f(), float random_angle_light=0, float flipped=false) {
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
    film->AddChild(make_shared<XMLElement>("integer", "width", std::to_string(width)));
    film->AddChild(make_shared<XMLElement>("integer", "height", std::to_string(height)));
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
    shape->AddChild(make_shared<XMLElement>("string", "filename", meshPath));
    auto bsdf = make_shared<XMLElement>("bsdf", "diffuse");
    bsdf->AddChild(make_shared<XMLElement>("spectrum", "reflectance", "1"));
    if (!meshTexture.empty()) {
        auto texture = make_shared<XMLElement>("texture", "bitmap");
        texture->AddProperty("name", "diffuseReflectance");
        texture->AddChild(make_shared<XMLElement>("string", "filename", meshTexture));
        bsdf->AddChild(texture);
    }
    shape->AddChild(bsdf);
    auto shape_transform = make_shared<XMLElement>("transform");
    shape_transform->AddProperty("name", "toWorld");
    auto shape_translate = make_shared<XMLElement>("translate");
    shape_translate->AddProperty("y", std::to_string(-plane_height));
    shape_transform->AddChild(shape_translate);
    if (flipped) {
      auto shape_scale = make_shared<XMLElement>("scale");
      shape_scale->AddProperty("y", "-1");
      shape_transform->AddChild(shape_scale);
      shape->AddChild(make_shared<XMLElement>("boolean", "flipNormals", "true"));
    }
    shape->AddChild(shape_transform);

    if (!flipped) {
      auto plane = make_shared<XMLElement>("shape", "rectangle");
      auto plane_trans = make_shared<XMLElement>("transform");
      plane_trans->AddProperty("name", "toWorld");
      auto plane_scale = make_shared<XMLElement>("scale");
      plane_scale->AddProperty("x", "8");
      plane_scale->AddProperty("y", "8");
      auto plane_rotate = make_shared<XMLElement>("rotate");
      plane_rotate->AddProperty("x", "1");
      plane_rotate->AddProperty("angle", "-90");
      plane_trans->AddChild(plane_scale);
      plane_trans->AddChild(plane_rotate);
      plane->AddChild(plane_trans);
      auto plane_bsdf = make_shared<XMLElement>("bsdf", "roughplastic");
      plane_bsdf->AddChild(make_shared<XMLElement>("float", "alpha", to_string(alpha)));
      plane->AddChild(plane_bsdf);
      scene->AddChild(plane);
    }

    auto emitter = make_shared<XMLElement>("emitter", "envmap");
    emitter->AddChild(make_shared<XMLElement>("string", "filename", envmap));
    if (random_angle != 0) {
        auto env_transform = make_shared<XMLElement>("transform");
        env_transform->AddProperty("name", "toWorld");
        auto env_rotate = make_shared<XMLElement>("rotate");
        env_rotate->AddProperty("x", std::to_string(random_axis.x()));
        env_rotate->AddProperty("y", std::to_string(random_axis.y()));
        env_rotate->AddProperty("z", std::to_string(random_axis.z()));
        env_rotate->AddProperty("angle", std::to_string(random_angle));
        env_transform->AddChild(env_rotate);
        emitter->AddChild(env_transform);
    }

    if (flipped) {
      //TODO: world Y position integrator or some visualization that captures the reflected geometry
    } else {
      auto integrator = make_shared<XMLElement>("integrator", "path");
      integrator->AddChild(make_shared<XMLElement>("integer", "maxDepth", "3"));
      scene->AddChild(integrator);
    }
    

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
        if (random_angle_light != 0) {
            auto light_rotate = make_shared<XMLElement>("rotate");
            light_rotate->AddProperty("x", std::to_string(random_axis_light.x()));
            light_rotate->AddProperty("y", std::to_string(random_axis_light.y()));
            light_rotate->AddProperty("z", std::to_string(random_axis_light.z()));
            light_rotate->AddProperty("angle", std::to_string(random_angle_light));
            light_trans->AddChild(light_rotate);
        }
        light->AddChild(light_trans);
	    light->AddChild(make_shared<XMLElement>("spectrum", "intensity", "400"));
        scene->AddChild(light);
    }

    return scene;
}

int main(int argc, char** argv) {
    srand((unsigned int) time(0));

    bool output_masks = false;
    bool output_scenes = true;
    std::string filename, mask_filename;
    std::string envmap;
    const float correction_factor = 8.0f/8.72551f; //hand measured error factor
    
    const std::string object_mask_path = "objectmask.png";
    const std::string plane_mask_path = "planemask.png";
    const std::string mesh_path = "output_mesh.obj";
    const std::string meshwo_path = "output_meshwo.obj";
    const std::string scene_path = "scene_gen.xml";
    const std::string textured_scene_path = "scene_gen_tex.xml";
    const std::string textured_scenewo_path = "scenewo_gen_tex.xml";
    const std::string flipped_scene_path = "scene_gen_flipped.xml";
    const std::string flipped_scenewo_path = "scenewo_gen_flipped.xml";
    
    float scale_factor = 0.5;
    const float floorEps = 5e-2;
    const float floor_normal_angle_range = 60;
    const std::string texture_image = "texture.exr";
    float phi = 0;
    float theta = 0;
    float alpha = 0;
    float light_theta = 0;
    float light_phi = 0;
    const float light_radius = 52;
    float occlusion_threshold = 1;
    const float max_depth = 1000;
    const float min_depth = 1;
    float displacement = 0;
    float angle_random_magnitude = 0;
    float alpha_random_std = 0;
    Eigen::Vector3f random_axis;
    float random_angle = 0;
    Eigen::Vector3f random_axis_light;
    float random_angle_light = 0;
    bool light = false;
    std::string scene_version("0.6.0");

    //parse arguments

    OptionParser parser(argc, argv);
    std::cout << parser.getNumArguments() << " arguments" << std::endl;
    if (parser.getNumArguments() != 6) {
        usage(argv[0]);
        return 0;
    }
    filename = parser.getPositionalArgument(0);
    envmap = parser.getPositionalArgument(1);
    theta = std::stof(parser.getPositionalArgument(2)) / 180.0f * (float) M_PI;
    phi = std::stof(parser.getPositionalArgument(3)) / 180.0f * (float) M_PI;
    alpha = std::stof(parser.getPositionalArgument(4)) / 10000.0f;
    mask_filename = parser.getPositionalArgument(5);

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

    if (parser.cmdOptionExists("output_masks")) {
        output_masks = true;
    }
    if (parser.cmdOptionExists("no_output_scene")) {
        output_scenes = false;
    }
    if (parser.cmdOptionExists("s")) {
        scene_version = parser.getCmdOption("s");
        std::cout << "using scene version " << scene_version << std::endl;
    }

    if (parser.cmdOptionExists("r")) {
        scale_factor = std::stof(parser.getCmdOption("r"));
        std::cout << "scale factor: " << scale_factor << std::endl;
    }

    if (parser.cmdOptionExists("randang")) {
        angle_random_magnitude = std::stof(parser.getCmdOption("randang"));
        std::cout << "random angle magnitude: " << angle_random_magnitude << std::endl;
        if (angle_random_magnitude > 0) {
            geom::randomAngleAxis(angle_random_magnitude, random_axis, random_angle);
            std::cout << "random axis1: " << std::endl << random_axis << std::endl;
            std::cout << "random angle1: " << random_angle << std::endl;

            geom::randomAngleAxis(angle_random_magnitude, random_axis_light, random_angle_light);
            std::cout << "random axis2: " << std::endl << random_axis_light << std::endl;
            std::cout << "random angle2: " << random_angle_light << std::endl;
        }
    }

    if (parser.cmdOptionExists("randalpha")) {
        alpha_random_std = std::stof(parser.getCmdOption("randalpha"));
        std::cout << "random alpha magnitude: " << alpha_random_std << std::endl;
        if (alpha_random_std > 0) {
            std::default_random_engine generator((unsigned int) time(0));
            std::normal_distribution<float> distribution(0, alpha_random_std);
            alpha = std::max(0.0f, alpha - std::fabs(distribution(generator)));
            std::cout << "perturbed alpha: " << alpha << std::endl;
        }
    }

    // Read Depth image
    cv::Mat depth_img = cv::imread(filename, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    if (!depth_img.data) {
      std::cout << "image not found: " << filename << std::endl;
      return 1;
    }
    //depth_img = max_depth * (1-depth_img); //if transformation is needed

    // Read Mask image
    cv::Mat mask_img = cv::imread(mask_filename, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    if (!mask_img.data) {
      std::cout << "image not found: " << mask_filename << std::endl;
      return 1;
    }
    std::cout << "depth_img: (" << depth_img.cols << ", " << depth_img.rows << ")" << std::endl << "mask_img: (" << mask_img.cols << ", " << mask_img.rows << ")" << std::endl;

    if (mask_img.cols != depth_img.cols || mask_img.rows != depth_img.rows) {
        std::cout << "depth image and mask image dimensions do not match, aborting" << std::endl;
        return 1;
    }

    // Subtract out mask
    cv::Mat mask = (depth_img != mask_img) | (depth_img == 0);
    if (output_masks) {
        cv::imwrite(object_mask_path, mask);
    }
    mask.convertTo(mask, depth_img.type(), 1/255.f);
    cv::Mat depthwo_img = depth_img.mul(mask);

    const int original_width = depth_img.cols;
    const int original_height = depth_img.rows;
    cv::Mat resampled_depth_img;
    cv::resize(depth_img, resampled_depth_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
    cv::resize(depthwo_img, depthwo_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    OBJMesh<float> mesh = createMesh(resampled_depth_img, min_depth, max_depth, occlusion_threshold, correction_factor);
    OBJMesh<float> meshwo = createMesh(depthwo_img, min_depth, max_depth, occlusion_threshold, correction_factor);

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
    const size_t n = mesh.GetNumVertices();
    for (int i=1; i<=n; i++) {
      Vector3<float> &vert = mesh.GetVertex(i);
      Eigen::Vector4f vert4;
      vert4.head(3) = vert;
      vert4[3] = 1;
      vert4 = camToWorld * vert4;
      vert = vert4.head(3);
      heights.push_back(vert[1]);
    }
    for (int i=1; i<=meshwo.GetNumVertices(); i++) {
      Vector3<float> &vert = meshwo.GetVertex(i);
      Eigen::Vector4f vert4;
      vert4.head(3) = vert;
      vert4[3] = 1;
      vert4 = camToWorld * vert4;
      vert = vert4.head(3);
    }

    std::sort(heights.begin(), heights.end());
    const size_t smallIndex = std::max(depth_img.rows, depth_img.cols) * 2;
    const float minHeight = heights[smallIndex];
    std::cout << "deleting below " << minHeight << std::endl;
    const size_t oldSize = mesh.GetNumElements();
    mesh.DeleteBelowY(minHeight + floorEps, true, floor_normal_angle_range);
    meshwo.DeleteBelowY(minHeight + floorEps, true, floor_normal_angle_range);
    const size_t newSize = mesh.GetNumElements();
    std::cout << "deleted " << oldSize - newSize << " faces out of " << oldSize << ", leaving " << newSize << std::endl;
    if (output_masks) {
        cv::Mat mask = createMask(depth_img, min_depth, max_depth, [&](Vector3<float> p) -> bool { Eigen::Vector4f pp; pp.head(3)=p; pp[3]=1; return (camToWorld*pp)[1] < minHeight + floorEps; }, correction_factor);
        cv::imwrite(plane_mask_path, mask);
    }

    if (output_scenes) {
        std::ofstream of(mesh_path);
        mesh.SaveOBJ(of);
        of.close();
        std::cout << "saved mesh at " << mesh_path << std::endl;
        std::ofstream ofwo(meshwo_path);
        meshwo.SaveOBJ(ofwo);
        ofwo.close();
        std::cout << "saved (wo) mesh at " << meshwo_path << std::endl;

        const float lightZ = light_radius * sin(light_theta) * cos(light_phi);
        const float lightX = light_radius * cos(light_theta) * cos(light_phi);
        const float lightY = light_radius * sin(light_phi);
        auto scene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight,  scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, "", random_axis, random_angle, random_axis_light, random_angle_light);
        std::ofstream sceneof(scene_path);
        scene->SaveXML(sceneof);
        sceneof.close();
	
        auto texscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, texture_image, random_axis, random_angle, random_axis_light, random_angle_light);
        std::ofstream texsceneof(textured_scene_path);
        texscene->SaveXML(texsceneof);
        texsceneof.close();
	
        auto woscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path, texture_image, random_axis, random_angle, random_axis_light, random_angle_light);
        std::ofstream texscenewoof(textured_scenewo_path);
        woscene->SaveXML(texscenewoof);
        texscenewoof.close();
	
	auto flippedscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight,  scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, "", random_axis, random_angle, random_axis_light, random_angle_light, true);
	std::ofstream flippedsceneof(flipped_scene_path);
	flippedscene->SaveXML(flippedsceneof);
	flippedsceneof.close();

	auto flippedwoscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight,  scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path, "", random_axis, random_angle, random_axis_light, random_angle_light, true);
	std::ofstream flippedscenewoof(flipped_scenewo_path);
	flippedwoscene->SaveXML(flippedscenewoof);
	flippedscenewoof.close();


        std::cout << "wrote scene files to " << std::endl << scene_path << std::endl << textured_scene_path << std::endl << textured_scenewo_path << std::endl << flipped_scene_path << std::endl << flipped_scenewo_path << std::endl;
    }
    return 0;
}
