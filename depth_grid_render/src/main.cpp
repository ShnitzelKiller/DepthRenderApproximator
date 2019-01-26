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
#include <string>

#define NUM_SCENES 12

enum SceneMode {normal, flip, specular, directDiffuse, directSpec, texture_only, texture_inf};

void usage(char* program_name) {
    std::cout << "Usage: " << program_name << " filename envmap theta phi alpha maskfilename [-ltheta <value> -lphi <value>] [-c <occlusion_threshold>] [-d <displacement_factor>] [-s <scene_format_version>] [-r <resize_factor>] [-rand <angle_randomness_magnitude_in_degrees>] [-scenes (1|0){12}] [-save <name>]" << std::endl;
}

std::shared_ptr<XMLElement> buildScene(int width, int height, std::string envmap, float alpha, const Eigen::Vector3f &camOrigin, float plane_height, std::string scene_version = "0.6.0", bool pointlight = false, Eigen::Vector3f light_pos = Eigen::Vector3f(), std::string meshPath="", std::string meshTexture="", Eigen::Vector3f random_axis=Eigen::Vector3f(), float random_angle=0, Eigen::Vector3f random_axis_light=Eigen::Vector3f(), float random_angle_light=0, SceneMode mode = normal, std::string planeTexture="") {
    using namespace std;
    ostringstream eye;
    eye << camOrigin[0] << ", " << camOrigin[1] << ", " << camOrigin[2];

    auto scene = make_shared<XMLElement>("scene");
    scene->AddProperty("version", move(scene_version));

    auto camera = make_shared<XMLElement>("sensor", "perspective");
    auto sampler = make_shared<XMLElement>("sampler", "ldsampler");
    sampler->AddChild(make_shared<XMLElement>("integer", "sampleCount", "128"));
    camera->AddChild(sampler);
    camera->AddChild(make_shared<XMLElement>("float", "fov", "45"));
    auto film = make_shared<XMLElement>("film", "hdrfilm");
    film->AddChild(make_shared<XMLElement>("integer", "width", to_string(width)));
    film->AddChild(make_shared<XMLElement>("integer", "height", to_string(height)));
    film->AddChild(make_shared<XMLElement>("boolean", "banner", "false"));
    camera->AddChild(film);
    auto cam_trans = XMLElement::Transform("toWorld");
    auto look_at = make_shared<XMLElement>("lookat");
    look_at->AddProperty("origin", eye.str());
    look_at->AddProperty("target", "0, 1, 0");
    look_at->AddProperty("up", "0, 1, 0");
    cam_trans->AddChild(look_at);
    camera->AddChild(cam_trans);

    if (mode != texture_only && mode != texture_inf) {
        auto shape = make_shared<XMLElement>("shape", "obj");
        shape->AddChild(make_shared<XMLElement>("string", "filename", meshPath));
        if (mode == normal) {
            auto bsdf = make_shared<XMLElement>("bsdf", "diffuse");
            bsdf->AddChild(make_shared<XMLElement>("spectrum", "reflectance", "1"));
            if (!meshTexture.empty()) {
                auto texture = make_shared<XMLElement>("texture", "bitmap");
                texture->AddProperty("name", "diffuseReflectance");
                texture->AddChild(make_shared<XMLElement>("string", "filename", meshTexture));
                bsdf->AddChild(texture);
            }
            shape->AddChild(bsdf);
        } else if (mode == specular) {
            auto area_emitter = make_shared<XMLElement>("emitter", "area");
            area_emitter->AddChild(make_shared<XMLElement>("spectrum", "radiance", "1"));
            shape->AddChild(area_emitter);
        }

        auto shape_transform = XMLElement::Transform("toWorld");
        if (mode != specular) {
            //only correct if not emissive to workaround error?
            auto shape_translate = XMLElement::Translation(0.0f, -plane_height, 0.0f);
            shape_transform->AddChild(shape_translate);
        }
        if (mode == flip) {
            auto shape_scale = XMLElement::Scale(1, -1, 1);
            shape_transform->AddChild(shape_scale);
            shape->AddChild(make_shared<XMLElement>("boolean", "flipNormals", "true"));
        }
        shape->AddChild(shape_transform);
        scene->AddChild(shape);

    }

    if (mode != flip) {
      auto plane = make_shared<XMLElement>("shape", "rectangle");
      auto plane_trans = XMLElement::Transform("toWorld");
      auto plane_scale = XMLElement::Scale(8, 8, 1);
      auto plane_rotate = XMLElement::Rotation(0, -90);
      plane_trans->AddChild(plane_scale);
      plane_trans->AddChild(plane_rotate);
      plane->AddChild(plane_trans);

      if (mode == texture_only || mode == texture_inf) {
          //TODO: load actual texture for plane
          auto plane_bsdf = make_shared<XMLElement>("bsdf", "diffuse");
          //area_emitter->AddChild(make_shared<XMLElement>("spectrum", "radiance", "1"));
          auto texture = make_shared<XMLElement>("texture", "bitmap");
          if (mode == texture_inf) {
              texture->AddChild(make_shared<XMLElement>("float", "uscale", "10"));
              texture->AddChild(make_shared<XMLElement>("float", "vscale", "10"));
              plane_trans->AddChild(XMLElement::Scale(10, 1, 10));
          }
          texture->AddProperty("name", "reflectance");
          texture->AddChild(make_shared<XMLElement>("string", "filename", planeTexture));
          plane_bsdf->AddChild(texture);
          plane->AddChild(plane_bsdf);
          auto constant_emitter = make_shared<XMLElement>("emitter", "constant");
          constant_emitter->AddChild(make_shared<XMLElement>("spectrum", "radiance", "1"));
          scene->AddChild(constant_emitter);
      } else {
          auto plane_bsdf = make_shared<XMLElement>("bsdf", "roughplastic");
          plane_bsdf->AddChild(make_shared<XMLElement>("float", "alpha", to_string(alpha)));
          if (mode == specular || mode == directSpec) {
              plane_bsdf->AddChild(make_shared<XMLElement>("spectrum", "diffuseReflectance",
                                                           "0")); //disable diffuse component for rendering reflection image
          } else if (mode == directDiffuse) {
              plane_bsdf->AddChild(make_shared<XMLElement>("spectrum", "specularReflectance",
                                                           "0")); //disable specular component for rendering diffuse image
          }
          plane->AddChild(plane_bsdf);
      }
      scene->AddChild(plane);
    }

    if (mode != specular && mode != texture_only && mode != texture_inf) {
        auto emitter = make_shared<XMLElement>("emitter", "envmap");
        emitter->AddChild(make_shared<XMLElement>("string", "filename", envmap));
        if (random_angle != 0) {
            auto env_transform = XMLElement::Transform("toWorld");
            auto env_rotate = XMLElement::Rotation(random_axis.x(), random_axis.y(), random_axis.z(), random_angle);
            env_transform->AddChild(env_rotate);
            emitter->AddChild(env_transform);
        }
        scene->AddChild(emitter);
        if (pointlight) {
            auto light = make_shared<XMLElement>("emitter", "point");
            auto light_trans = XMLElement::Transform("toWorld");
            auto translate = XMLElement::Translation(light_pos.x(), light_pos.y(), light_pos.z());
            light_trans->AddChild(translate);
            if (random_angle_light != 0) {
                auto light_rotate = XMLElement::Rotation(random_axis_light.x(), random_axis_light.y(), random_axis_light.z(), random_angle_light);
                light_trans->AddChild(light_rotate);
            }
            light->AddChild(light_trans);
            light->AddChild(make_shared<XMLElement>("spectrum", "intensity", "400"));
            scene->AddChild(light);
        }
    }


    if (mode == flip) {
        //TODO: world Y position integrator or some visualization that captures the reflected geometry
        //also remove the emitters if we do that?
    } else if (mode == specular || mode == directSpec || mode == directDiffuse) {
        auto integrator = make_shared<XMLElement>("integrator", "path");
        integrator->AddChild(make_shared<XMLElement>("integer", "maxDepth", "2"));
        scene->AddChild(integrator);
    } else if (mode == normal) {
        auto integrator = make_shared<XMLElement>("integrator", "path");
        integrator->AddChild(make_shared<XMLElement>("integer", "maxDepth", "3"));
        scene->AddChild(integrator);
    }
    

    scene->AddChild(camera);

    return scene;
}

int main(int argc, char** argv) {
    srand((unsigned int) time(0));

    bool output_masks = false;
    bool model = true;
    std::string filename, mask_filename;
    std::string envmap;
    const float correction_factor = 8.0f/8.72551f; //hand measured error factor
    
    std::string object_mask_path = "objectmask.png";
    std::string plane_mask_path = "planemask.png";

    float scale_factor = 0.5;
    const float floorEps = 5e-2;
    const float floor_normal_angle_range = 60;
    std::string texture_image = "texture.png";
    std::string plane_texture = "placeholder.jpg";
    float alpha = 0;
    float phi = 0;
    float theta = 0;
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
    bool randomness_alpha = false;
    bool randomness_angle = false;
    bool light = false;
    std::string scene_version("0.6.0");
    std::string scene_mask;
    for (int i=0; i<NUM_SCENES; i++) {
        scene_mask.push_back('1');
    }

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
    if (parser.cmdOptionExists("scenes")) {
        std::string scene_mask0 = parser.getCmdOption("scenes");
        if (scene_mask0.size() != NUM_SCENES) {
            std::cout<<"mask should have " << NUM_SCENES << " characters" << std::endl;
        } else {
            scene_mask = scene_mask0;
        }
    }
    if (parser.cmdOptionExists("s")) {
        scene_version = parser.getCmdOption("s");
        std::cout << "using scene version " << scene_version << std::endl;
    }

    if (parser.cmdOptionExists("planetex")) {
        plane_texture = parser.getCmdOption("planetex");
        std::cout << "using plane texture " << plane_texture << std::endl;
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
            randomness_angle = true;
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
        randomness_alpha = true;
    }

    std::string basename;
    std::string mesh_path = "output_mesh.obj";
    std::string meshwo_path = "output_meshwo.obj";
    //const std::string meshobj_path = "output_meshobj.obj";
    std::string filenames[] = {"scene_gen.xml",
                               "scene_gen_tex.xml",
                               "scenewo_gen_tex.xml",
                               "scene_gen_flipped.xml",
                               "scenewo_gen_flipped.xml",
                               "scene_gen_spec.xml",
                               "scenewo_gen_spec.xml",
                               "scene_gen_directspec.xml",
                               "scenewo_gen_directspec.xml",
                               "scene_gen_directdiffuse.xml",
                               "scenewo_gen_directdiffuse.xml",
                               "scene_gen_texonly.xml"};

    if (parser.cmdOptionExists("save")) {
        basename = parser.getCmdOption("save");

        basename.push_back('_');
        for (int i=0; i<NUM_SCENES; i++) {
            filenames[i].insert(0, basename);
        }

        mesh_path.insert(0, basename);
        meshwo_path.insert(0, basename);
        object_mask_path.insert(0, basename);
        plane_mask_path.insert(0, basename);

        if (randomness_alpha || randomness_angle) {
            std::ofstream paramof(basename + "PARAM.txt");
            if (randomness_angle) {
                paramof << "random angle: " << std::endl;
                paramof << random_angle << std::endl;
                paramof << "random axis: " << std::endl;
                paramof << random_axis << std::endl;
                paramof << "random angle (light): " << std::endl;
                paramof << random_angle_light << std::endl;
                paramof << "random axis (light): " << std::endl;
                paramof << random_axis_light << std::endl;
            }
            if (randomness_alpha) {
                paramof << "perturbed alpha: " << std::endl;
                paramof << alpha << std::endl;
            }
            paramof.close();
        }
    }

    if (parser.cmdOptionExists("tex")) {
        texture_image = parser.getCmdOption("tex");
        std::cout << "using texture " << texture_image << std::endl;
    }

    if (parser.cmdOptionExists("nomodel")) {
        model = false;
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
    std::cout << "depth_img: (" << depth_img.cols << ", " << depth_img.rows << ")" << std::endl << "mask_img: ("
              << mask_img.cols << ", " << mask_img.rows << ")" << std::endl;

    if (mask_img.cols != depth_img.cols || mask_img.rows != depth_img.rows) {
        std::cout << "depth image and mask image dimensions do not match, aborting" << std::endl;
        return 1;
    }

    // Subtract out mask
    cv::Mat mask = (depth_img != mask_img) | (depth_img == 0);
    if (output_masks) {
        cv::imwrite(object_mask_path, mask);
    }
    mask.convertTo(mask, depth_img.type(), 1 / 255.f);
    cv::Mat depthwo_img = depth_img.mul(mask);

    int original_width = depth_img.cols;
    int original_height = depth_img.rows;

    if (parser.cmdOptionExists("width") && parser.cmdOptionExists("height")) {
        original_width = std::stoi(parser.getCmdOption("width"));
        original_height = std::stoi(parser.getCmdOption("height"));
    }

    cv::Mat resampled_depth_img;
    cv::resize(depth_img, resampled_depth_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
    cv::resize(depthwo_img, depthwo_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
    cv::resize(mask_img, mask_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
    std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

    const float radius = 26;
    const Eigen::Matrix<float, 3, 1> center(0, 1, 0);
    const Eigen::Matrix<float, 3, 1> up(0, 1, 0);
    const float camZ = radius * sin(theta) * cos(phi);
    const float camX = radius * cos(theta) * cos(phi);
    const float camY = radius * sin(phi);
    const Eigen::Matrix<float, 3, 1> eye(camX, camY, camZ);
    Eigen::Matrix<float, 4, 4> camToWorld = geom::lookAt(eye, center, up);

    std::cout << "camera position: " << std::endl << eye << std::endl;
    float minHeight = 0;
    if (model) {
        OBJMesh<float> mesh = createMesh(resampled_depth_img, min_depth, max_depth, occlusion_threshold,
                                         correction_factor);
        OBJMesh<float> meshwo = createMesh(depthwo_img, min_depth, max_depth, occlusion_threshold, correction_factor);
        //OBJMesh<float> meshobj = createMesh(mask_img, min_depth, max_depth, occlusion_threshold, correction_factor);

        std::cout << "finished creating mesh " << std::endl;

        std::cout << "transforming mesh" << std::endl;

        std::vector<float> heights;

        mesh.Transform(camToWorld);
        meshwo.Transform(camToWorld);
        //meshobj.Transform(camToWorld);

        const size_t n = mesh.GetNumVertices();
        for (int i = 1; i <= n; i++) {
            Vector3<float> &vert = mesh.GetVertex(i);
            heights.push_back(vert[1]);
        }

        std::sort(heights.begin(), heights.end());
        const size_t smallIndex = std::max(depth_img.rows, depth_img.cols) * 2;
        minHeight = heights[smallIndex];
        std::cout << "deleting below " << minHeight << std::endl;
        const size_t oldSize = mesh.GetNumElements();
        mesh.DeleteBelowY(minHeight + floorEps, true, floor_normal_angle_range);
        meshwo.DeleteBelowY(minHeight + floorEps, true, floor_normal_angle_range);
        //meshobj.DeleteBelowY(minHeight + floorEps, true, floor_normal_angle_range);
        const size_t newSize = mesh.GetNumElements();
        std::cout << "deleted " << oldSize - newSize << " faces out of " << oldSize << ", leaving " << newSize
                  << std::endl;
        if (output_masks) {
            cv::Mat mask = createMask(depth_img, min_depth, max_depth, [&](Vector3<float> p) -> bool {
                Eigen::Vector4f pp;
                pp.head(3) = p;
                pp[3] = 1;
                return (camToWorld * pp)[1] < minHeight + floorEps;
            }, correction_factor);
            cv::imwrite(plane_mask_path, mask);
        }

        std::ofstream of(mesh_path);
        mesh.SaveOBJ(of);
        of.close();
        std::cout << "saved mesh at " << mesh_path << std::endl;
        std::ofstream ofwo(meshwo_path);
        meshwo.SaveOBJ(ofwo);
        ofwo.close();
        std::cout << "saved (wo) mesh at " << meshwo_path << std::endl;
        /*std::ofstream ofobj(meshobj_path);
        meshobj.SaveOBJ(ofobj);
        ofobj.close();
        std::cout << "saved (obj) mesh at " << meshobj_path << std::endl; */
    }

    const float lightZ = light_radius * sin(light_theta) * cos(light_phi);
    const float lightX = light_radius * cos(light_theta) * cos(light_phi);
    const float lightY = light_radius * sin(light_phi);

    std::cout << "saving scenes according to mask " << scene_mask << ": " << std::endl;

    if (scene_mask[0] != '0') {

        auto scene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, "", random_axis,
                                random_angle, random_axis_light, random_angle_light);
        std::ofstream sceneof(filenames[0]);
        scene->SaveXML(sceneof);
        sceneof.close();
        std::cout << filenames[0] << std::endl;
    }

    if (scene_mask[1] != '0') {

        auto texscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                   light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, texture_image,
                                   random_axis, random_angle, random_axis_light, random_angle_light);
        std::ofstream texsceneof(filenames[1]);
        texscene->SaveXML(texsceneof);
        texsceneof.close();
        std::cout << filenames[1] << std::endl;
    }

    if (scene_mask[2] != '0') {

        auto woscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                  light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path, texture_image,
                                  random_axis, random_angle, random_axis_light, random_angle_light);
        std::ofstream texscenewoof(filenames[2]);
        woscene->SaveXML(texscenewoof);
        texscenewoof.close();
        std::cout << filenames[2] << std::endl;
    }

    if (scene_mask[3] != '0') {

        auto flippedscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight,
                                       scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, "",
                                       random_axis, random_angle, random_axis_light, random_angle_light, flip);
        std::ofstream flippedsceneof(filenames[3]);
        flippedscene->SaveXML(flippedsceneof);
        flippedsceneof.close();
        std::cout << filenames[3] << std::endl;
    }

    if (scene_mask[4] != '0') {

        auto flippedwoscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight,
                                         scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path,
                                         "", random_axis, random_angle, random_axis_light, random_angle_light,
                                         flip);
        std::ofstream flippedscenewoof(filenames[4]);
        flippedwoscene->SaveXML(flippedscenewoof);
        flippedscenewoof.close();
        std::cout << filenames[4] << std::endl;
    }

    if (scene_mask[5] != '0') {

        auto specscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight,
                                       scene_version, light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path,
                                       "", random_axis, random_angle, random_axis_light, random_angle_light,
                                       specular);
        std::ofstream specsceneof(filenames[5]);
        specscene->SaveXML(specsceneof);
        specsceneof.close();
        std::cout << filenames[5] << std::endl;
    }

    if (scene_mask[6] != '0') {

        auto specwoscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                      light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path, "", random_axis,
                                      random_angle, random_axis_light, random_angle_light, specular);
        std::ofstream specscenewoof(filenames[6]);
        specwoscene->SaveXML(specscenewoof);
        specscenewoof.close();
        std::cout << filenames[6] << std::endl;
    }

    if (scene_mask[7] != '0') {

        auto directspecscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                      light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, "", random_axis,
                                      random_angle, random_axis_light, random_angle_light, directSpec);
        std::ofstream directspecsceneof(filenames[7]);
        directspecscene->SaveXML(directspecsceneof);
        directspecsceneof.close();
        std::cout << filenames[7] << std::endl;
    }

    if (scene_mask[8] != '0') {

        auto directspecwoscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                          light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path, "", random_axis,
                                          random_angle, random_axis_light, random_angle_light, directSpec);
        std::ofstream directspecscenewoof(filenames[8]);
        directspecwoscene->SaveXML(directspecscenewoof);
        directspecscenewoof.close();
        std::cout << filenames[8] << std::endl;
    }

    if (scene_mask[9] != '0') {

        auto directdiffusescene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                          light, Eigen::Vector3f(lightX, lightY, lightZ), mesh_path, "", random_axis,
                                          random_angle, random_axis_light, random_angle_light, directDiffuse);
        std::ofstream directdiffusesceneof(filenames[9]);
        directdiffusescene->SaveXML(directdiffusesceneof);
        directdiffusesceneof.close();
        std::cout << filenames[9] << std::endl;
    }

    if (scene_mask[10] != '0') {
        auto directdiffusewoscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                            light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path, "", random_axis,
                                            random_angle, random_axis_light, random_angle_light, directDiffuse);
        std::ofstream directdiffusescenewoof(filenames[10]);
        directdiffusewoscene->SaveXML(directdiffusescenewoof);
        directdiffusescenewoof.close();
        std::cout << filenames[10] << std::endl;
    }
    if (scene_mask[11] != '0') {
        auto texonlyscene = buildScene(original_width, original_height, envmap, alpha, eye, minHeight, scene_version,
                                               light, Eigen::Vector3f(lightX, lightY, lightZ), meshwo_path, "", random_axis,
                                               random_angle, random_axis_light, random_angle_light, texture_inf, plane_texture);
        std::ofstream texonlysceneof(filenames[11]);
        texonlyscene->SaveXML(texonlysceneof);
        texonlysceneof.close();
        std::cout << filenames[11] << std::endl;
    }
    return 0;
}
