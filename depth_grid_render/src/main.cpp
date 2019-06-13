#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "CameraUtils.hpp"
#include "XMLWriter.hpp"
#include "OBJWriter.hpp"
#include <sstream>
#include "OptionParser.hpp"
#include "MeshBuilder.hpp"
#include <math.h>
#include <limits>
#include <string>

#define NUM_SCENES 13

enum SceneMode {normal, flip, specular, directDiffuse, directSpec, texture_only, texture_inf, plane_only};

void usage(char* program_name) {
  std::cout << "Usage: " << program_name << " filename envmap alpha maskfilename [-theta <value>] [-phi <value>] [-ltheta <value> -lphi <value>] [-c <occlusion threshold>] [-rt <ransac threshold>] [-nt <face angle threshold>] [-envscale <value>] [-norange 1] [-d <displacement factor>] [-output masks 1] [-s <scene format version>] [-correction <fac>] [-planetex <plane texture filename>] [-r <resize factor>] [-randang <angle randomness magnitude in degrees>] [-randalpha <std>] [-nomodel 1] [-width <w>] [-height <h>] [-scenes (1|0){13}] [-save <name>]" << std::endl;
}

std::shared_ptr<XMLElement> buildScene(int width, int height, float alpha, const Eigen::Matrix<float, 4, 4> &mat, OptionParser &config, float plane_height=0, std::string meshPath="", std::string meshTexture="", Eigen::Vector3f random_axis=Eigen::Vector3f(), float random_angle=0, Eigen::Vector3f random_axis_light=Eigen::Vector3f(), float random_angle_light=0, SceneMode mode = normal) {
  
  using namespace std;
  auto frame_matrix = XMLElement::Matrix(mat);
    
  bool pointlight = false;
  float light_theta,light_phi;
  if (config.cmdOptionExists("ltheta") && config.cmdOptionExists("lphi")) {
    pointlight = true;
    light_theta = std::stof(config.get("ltheta")) / 180.0f * (float) M_PI;
    light_phi = std::stof(config.get("lphi")) / 180.0f * (float) M_PI;
    std::cout << "using point light with theta " << config.get("ltheta") << " and phi " << config.get("lphi") << std::endl;
  }

  auto scene = make_shared<XMLElement>("scene");
  scene->AddProperty("version", config.get("s"));

  auto camera = make_shared<XMLElement>("sensor", "perspective");
  auto sampler = make_shared<XMLElement>("sampler", "ldsampler");
  sampler->AddChild(make_shared<XMLElement>("integer", "sampleCount", "128"));
  camera->AddChild(sampler);
  camera->AddChild(make_shared<XMLElement>("float", "fov", config.get("fov")));
  auto film = make_shared<XMLElement>("film", "hdrfilm");
  film->AddChild(make_shared<XMLElement>("integer", "width", to_string(width)));
  film->AddChild(make_shared<XMLElement>("integer", "height", to_string(height)));
  film->AddChild(make_shared<XMLElement>("boolean", "banner", "false"));
  camera->AddChild(film);
  auto cam_trans = XMLElement::Transform("toWorld");
  cam_trans->AddChild(XMLElement::Rotation(1, 180));
  camera->AddChild(cam_trans);

  //add shape
  if (mode != texture_only && mode != texture_inf && mode != plane_only) {
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
    
  //add plane
  if (mode != flip) {
    auto plane = make_shared<XMLElement>("shape", "rectangle");
    if (config.cmdOptionExists("flip"))
      plane->AddChild(make_shared<XMLElement>("boolean", "flipNormals", "true"));
    auto plane_trans = XMLElement::Transform("toWorld");
    const float plane_scale_factor = std::stof(config.get("planescale"));
    auto plane_scale = XMLElement::Scale(plane_scale_factor, plane_scale_factor, 1.0f);
    auto plane_rotate = XMLElement::Rotation(0, -90);
    plane_trans->AddChild(plane_scale);
    plane_trans->AddChild(plane_rotate);
    plane->AddChild(plane_trans);

    if (mode == texture_only || mode == texture_inf || mode == plane_only) {
      //TODO: load actual texture for plane
      if (mode != plane_only) {  
	auto plane_bsdf = make_shared<XMLElement>("bsdf", "diffuse");
	auto texture = make_shared<XMLElement>("texture", "bitmap");
	if (mode == texture_inf) {
	  texture->AddChild(make_shared<XMLElement>("float", "uscale", "10"));
	  texture->AddChild(make_shared<XMLElement>("float", "vscale", "10"));
	  plane_trans->AddChild(XMLElement::Scale(10, 1, 10));
	}
	texture->AddProperty("name", "reflectance");
	texture->AddChild(make_shared<XMLElement>("string", "filename", config.get("planetex")));
	plane_bsdf->AddChild(texture);
	plane->AddChild(plane_bsdf);
	auto constant_emitter = make_shared<XMLElement>("emitter", "constant");
	constant_emitter->AddChild(make_shared<XMLElement>("spectrum", "radiance", "1"));
	scene->AddChild(constant_emitter);
      } else {
	auto area_emitter = make_shared<XMLElement>("emitter", "area");
	area_emitter->AddChild(make_shared<XMLElement>("spectrum", "radiance", "1"));
	plane->AddChild(area_emitter);
      }
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
    plane_trans->AddChild(frame_matrix);
    scene->AddChild(plane);
  }
    
  //add lighting
  if (mode != specular && mode != texture_only && mode != texture_inf && mode != plane_only) {
    auto emitter = make_shared<XMLElement>("emitter", "envmap");
    emitter->AddChild(make_shared<XMLElement>("string", "filename", config.get(1)));
    emitter->AddChild(make_shared<XMLElement>("float", "scale", config.get("envscale")));
    auto env_transform = XMLElement::Transform("toWorld");
    env_transform->AddChild(XMLElement::Rotation(1, std::stof(config.get("envrot"))));
    env_transform->AddChild(frame_matrix);
    if (random_angle != 0) {
      auto env_rotate = XMLElement::Rotation(random_axis.x(), random_axis.y(), random_axis.z(), random_angle);
      env_transform->AddChild(env_rotate);
    }
    emitter->AddChild(env_transform);
    scene->AddChild(emitter);
    if (pointlight) {
      const float light_radius = 52;
      const float lightZ = light_radius * sin(light_theta) * cos(light_phi);
      const float lightX = light_radius * cos(light_theta) * cos(light_phi);
      const float lightY = light_radius * sin(light_phi);

      auto light = make_shared<XMLElement>("emitter", "point");
      auto light_trans = XMLElement::Transform("toWorld");
      auto translate = XMLElement::Translation(lightX, lightY, lightZ);
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
  bool model = true;
  std::string filename, mask_filename, depthwo_filename;
    
  std::string object_mask_path = "objectmask.png";
  std::string plane_mask_path = "planemask.png";
  std::string plane_maskwo_path = "planemaskwo.png";
  const float floorEps = 5e-2;
    
  std::default_random_engine generator((unsigned int) time(0));

  //parse arguments

  OptionParser parser(argc, argv);
  std::cout << parser.getNumArguments() << " arguments" << std::endl;
  if (parser.getNumArguments() != 4) {
    usage(argv[0]);
    return 0;
  }

  filename = parser.get(0);
  float alpha = std::stof(parser.get(2)) / 10000.0f;
  bool randomness_alpha = false;
  if (parser.cmdOptionExists("randalpha")) {
    randomness_alpha = true;
    const float alpha_random_std = std::stof(parser.get("randalpha"));
    std::cout << "random alpha magnitude: " << alpha_random_std << std::endl;
    if (alpha_random_std > 0) {
      std::normal_distribution<float> distribution(0, alpha_random_std);
      alpha = std::max(0.0f, alpha - std::fabs(distribution(generator)));
      std::cout << "perturbed alpha: " << alpha << std::endl;
    }
  }
  mask_filename = parser.get(3);

  parser.addArg("s", "0.6.0");
  parser.addArg("fov", "45");
  parser.addArg("planescale", "8");
  const float fov = std::stof(parser.get("fov"));
  std::cout << "fov: " << fov << std::endl;

  parser.addArg("envscale", "1");
  std::cout << "environment map scale: " << parser.get("envscale") <<std::endl;
  parser.addArg("envrot", "0");

  parser.addArg("r", "0.5");
  const float scale_factor = std::stof(parser.get("r"));
  std::cout << "scale factor: " << scale_factor << std::endl;
  

  parser.addArg("d", "0");
  const float displacement = std::stof(parser.get("d"));
  if (displacement > 0) {
    std::cout << "displacing occlusion boundaries by " << displacement << " units" << std::endl;
    std::cout << "WARNING: this looks terrible and doesn't work" << std::endl;
  }

  bool range_correction=true;
  if (parser.cmdOptionExists("norange")) {
    std::cout << "assuming depth input" << std::endl;
    range_correction=false;
  } else {
    std::cout << "assuming range input" << std::endl;
  }

  float min_depth = std::numeric_limits<float>::lowest();
  if (parser.cmdOptionExists("mindepth")) {
    min_depth = std::stof(parser.get("mindepth"));
    std::cout << "min depth: " << min_depth << std::endl;
  }
  float max_depth = std::numeric_limits<float>::max();
  if (parser.cmdOptionExists("maxdepth")) {
    max_depth = std::stof(parser.get("maxdepth"));
    std::cout << "max depth: " << max_depth << std::endl;
  }

  parser.addArg("rt", "1");
  const float ransac_threshold = std::stof(parser.get("rt"));

  parser.addArg("nt", "60");
  const float floor_normal_angle_range = std::stof(parser.get("nt"));

  parser.addArg("dt", "1");
  const float delete_threshold = std::stof(parser.get("dt"));

  bool use_camera_angle = false;
  float phi = 0;
  float theta = 0;
  if (parser.cmdOptionExists("theta") && parser.cmdOptionExists("phi")) {
    use_camera_angle = true;
    theta = std::stof(parser.get("theta")) / 180.0f * (float) M_PI;
    phi = std::stof(parser.get("phi")) / 180.0f * (float) M_PI;
  } else {
    std::cout << "ransac threshold: " << ransac_threshold << std::endl;
    std::cout << "deletion threshold: " << delete_threshold << std::endl;
    std::cout << "floor normal angle deletion threshold: " << floor_normal_angle_range << std::endl;
  }

  parser.addArg("c", "1");
  const float occlusion_threshold = std::stof(parser.get("c"));
  std::cout << "occlusion threshold: " << occlusion_threshold << std::endl;

  bool output_masks = false;
  if (parser.cmdOptionExists("output_masks")) {
    output_masks = true;
    depthwo_filename = parser.get("output_masks");
  }
  std::string scene_mask;
  for (int i=0; i<NUM_SCENES; i++) {
    scene_mask.push_back('1');
  }
  if (parser.cmdOptionExists("scenes")) {
    std::string scene_mask0 = parser.get("scenes");
    if (scene_mask0.size() != NUM_SCENES) {
      std::cout<<"mask should have " << NUM_SCENES << " characters" << std::endl;
    } else {
      scene_mask = scene_mask0;
    }
  }

  float correction_factor = 8.0f/8.72551f; //hand measured error factor
  if (parser.cmdOptionExists("correction")) {
    correction_factor = std::stof(parser.get("correction"));
  }

  parser.addArg("planetex", "placeholder.jpg");
  const std::string plane_texture = parser.get("planetex");
  std::cout << "using plane texture " << plane_texture << std::endl;

  float angle_random_magnitude = 0;
  Eigen::Vector3f random_axis;
  float random_angle = 0;
  bool randomness_angle = false;
  Eigen::Vector3f random_axis_light;
  float random_angle_light = 0;
  if (parser.cmdOptionExists("randang")) {
    angle_random_magnitude = std::stof(parser.get("randang"));
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

  std::string basename;
  std::string mesh_path = "output_mesh.obj";
  std::string meshwo_path = "output_meshwo.obj";
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
			     "scene_gen_texonly.xml",
			     "scene_gen_texonly_inf.xml"};

  if (parser.cmdOptionExists("save")) {
    basename = parser.get("save");

    basename.push_back('_');
    for (int i=0; i<NUM_SCENES; i++) {
      filenames[i].insert(0, basename);
    }

    mesh_path.insert(0, basename);
    meshwo_path.insert(0, basename);
    object_mask_path.insert(0, basename);
    plane_mask_path.insert(0, basename);
    plane_maskwo_path.insert(0, basename);

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
  parser.addArg("tex", "texture.png");
  const std::string texture_image = parser.get("tex");
  std::cout << "using texture " << texture_image << std::endl;


  if (parser.cmdOptionExists("nomodel")) {
    model = false;
  }
  if (!model && !use_camera_angle) {
    std::cout << "if not using model, must supply -theta and -phi arguments" << std::endl;
    return 1;
  }
  int options = cv::IMREAD_GRAYSCALE;
  bool ldr = false;
  if (!parser.cmdOptionExists("ldr")) {
    ldr = true;
    std::cout << "not reading float image" << std::endl;
    options = options | cv::IMREAD_ANYDEPTH;
  }

  // Read Depth image
  cv::Mat depth_img = cv::imread(filename, options);
  if (!depth_img.data) {
    std::cout << "image not found: " << filename << std::endl;
    return 1;
  }
  //depth_img = max_depth * (1-depth_img); //if transformation is needed

  // Read Mask image
  cv::Mat mask_img = cv::imread(mask_filename, options);
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
    original_width = std::stoi(parser.get("width"));
    original_height = std::stoi(parser.get("height"));
  }

  cv::Mat resampled_depth_img;
  cv::resize(depth_img, resampled_depth_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
  cv::resize(depthwo_img, depthwo_img, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_NEAREST);
  std::cout << "width: " << depth_img.cols << " height: " << depth_img.rows << std::endl;

  Eigen::Matrix<float, 4, 4> camToWorld;
  Eigen::Matrix<float, 4, 4> worldToCam;
  if (use_camera_angle) {
    const float radius = 26;
    const Eigen::Matrix<float, 3, 1> center(0, 1, 0);
    const Eigen::Matrix<float, 3, 1> up(0, 1, 0);
    const float camZ = radius * sin(theta) * cos(phi);
    const float camX = radius * cos(theta) * cos(phi);
    const float camY = radius * sin(phi);
    const Eigen::Matrix<float, 3, 1> eye(camX, camY, camZ);
    std::cout << "camera position: " << std::endl << eye << std::endl;
    camToWorld = geom::lookAt(eye, center, up);
    worldToCam = camToWorld.inverse();
  }
  float minHeight = 0;
  if (model) {
    cv::imwrite("testimg.png", resampled_depth_img);
    OBJMesh<float> mesh = createMesh(resampled_depth_img, min_depth, max_depth, occlusion_threshold,
				     correction_factor, range_correction, fov);
    OBJMesh<float> meshwo = createMesh(depthwo_img, min_depth, max_depth, occlusion_threshold, correction_factor, range_correction, fov);

    std::cout << "finished creating mesh " << std::endl;

    if (!use_camera_angle) {
      std::cout << "inferring transformation using RANSAC plane" << std::endl;
	  
      Eigen::Vector3f n(0,1,0);
      Eigen::Vector3f p(0,0,0);
	  
      int maxinliers = 0;
      for (int i=0; i<8; i++) {
	Eigen::Vector3f n0,p0;
	int inliers = ransac(mesh, ransac_threshold, generator, p0, n0);
	std::cout << "inliers: " << inliers << std::endl;
	if (inliers > maxinliers) {
	  maxinliers = inliers;
	  n = n0;
	  p = p0;
	}
      }

      //create debug mesh
      OBJMesh<float> debugmesh;
      Eigen::Vector3f dup(1,0,0);
      Eigen::Vector3f left = dup.cross(n).normalized();
      Eigen::Vector3f right = left.cross(n);
      left *= 100;
      right *= 100;
      Eigen::Vector3f p11 = p+left+right;
      Eigen::Vector3f p10 = p+left-right;
      Eigen::Vector3f p00 = p-left-right;
      Eigen::Vector3f p01 = p-left+right;
      debugmesh.AddVertex(p11);
      debugmesh.AddVertex(p10);
      debugmesh.AddVertex(p00);
      debugmesh.AddVertex(p01);
      debugmesh.AddTri(1, 2, 3);
      debugmesh.AddTri(1, 3, 4);
      debugmesh.SaveOBJ("debug.obj");

      //create transformation
      Eigen::Vector3f target = p+n;
      Eigen::Matrix4f temp = geom::lookAt(p, target, Eigen::Matrix<float, 3, 1>(0, 1, 0));
      worldToCam.col(0) = temp.col(1);
      worldToCam.col(1) = temp.col(2);
      worldToCam.col(2) = temp.col(0);
      worldToCam.col(3) = temp.col(3);
      worldToCam.row(3) = temp.row(3);
      camToWorld = worldToCam.inverse();
	  
      //remove mesh elements inside floor
      mesh.DeletePlane(delete_threshold, p, n, floor_normal_angle_range);
      meshwo.DeletePlane(delete_threshold, p, n, floor_normal_angle_range);

    } else {

      std::cout << "transforming mesh" << std::endl;

      std::vector<float> heights;

      mesh.Transform(camToWorld);
      meshwo.Transform(camToWorld);

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
      const size_t newSize = mesh.GetNumElements();
      std::cout << "deleted " << oldSize - newSize << " faces out of " << oldSize << ", leaving " << newSize
		<< std::endl;
      std::cout << "untransforming mesh" << std::endl;
      mesh.Transform(worldToCam);
      meshwo.Transform(worldToCam);
    }
	
    if (output_masks) {
      auto predfun = [&](Vector3<float> p) -> bool {
	Eigen::Vector4f pp;
	pp.head(3) = p;
	pp[3] = 1;
	return (camToWorld * pp)[1] < minHeight + floorEps;
      };
      cv::Mat mask = createMask(depth_img, min_depth, max_depth, predfun, correction_factor, range_correction, fov);
      cv::imwrite(plane_mask_path, mask);

      cv::Mat depthwo = cv::imread(depthwo_filename, options);
      cv::Mat maskwo = createMask(depthwo, min_depth, max_depth, predfun, correction_factor, range_correction, fov);
      cv::imwrite(plane_maskwo_path, maskwo);
    }

    std::ofstream of(mesh_path);
    mesh.SaveOBJ(of);
    of.close();
    std::cout << "saved mesh at " << mesh_path << std::endl;
    std::ofstream ofwo(meshwo_path);
    meshwo.SaveOBJ(ofwo);
    ofwo.close();
    std::cout << "saved (wo) mesh at " << meshwo_path << std::endl;
  }

  std::cout << "saving scenes according to mask " << scene_mask << ": " << std::endl;

  if (scene_mask[0] != '0') {

    auto scene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, mesh_path, "", random_axis, random_angle, random_axis_light, random_angle_light);
    scene->SaveXML(filenames[0]);
    std::cout << filenames[0] << std::endl;
  }

  if (scene_mask[1] != '0') {

    auto texscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, mesh_path, texture_image, random_axis, random_angle, random_axis_light, random_angle_light);
    texscene->SaveXML(filenames[1]);
    std::cout << filenames[1] << std::endl;
  }

  if (scene_mask[2] != '0') {

    auto woscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
      meshwo_path, texture_image,
      random_axis, random_angle, random_axis_light, random_angle_light);
    woscene->SaveXML(filenames[2]);
    std::cout << filenames[2] << std::endl;
  }

  if (scene_mask[3] != '0') {

    auto flippedscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight,
   mesh_path, "",
   random_axis, random_angle, random_axis_light, random_angle_light, flip);
    flippedscene->SaveXML(filenames[3]);
    std::cout << filenames[3] << std::endl;
  }

  if (scene_mask[4] != '0') {

    auto flippedwoscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight,
     meshwo_path,
     "", random_axis, random_angle, random_axis_light, random_angle_light,
     flip);
    flippedwoscene->SaveXML(filenames[4]);
    std::cout << filenames[4] << std::endl;
  }

  if (scene_mask[5] != '0') {

    auto specscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight,
mesh_path,
"", random_axis, random_angle, random_axis_light, random_angle_light,
specular);
    specscene->SaveXML(filenames[5]);
    std::cout << filenames[5] << std::endl;
  }

  if (scene_mask[6] != '0') {

    auto specwoscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
  meshwo_path, "", random_axis,
  random_angle, random_axis_light, random_angle_light, specular);
    specwoscene->SaveXML(filenames[6]);
    std::cout << filenames[6] << std::endl;
  }

  if (scene_mask[7] != '0') {

    auto directspecscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
                                      mesh_path, "", random_axis,
                                      random_angle, random_axis_light, random_angle_light, directSpec);
    directspecscene->SaveXML(filenames[7]);
    std::cout << filenames[7] << std::endl;
  }

  if (scene_mask[8] != '0') {

    auto directspecwoscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
meshwo_path, "", random_axis,
random_angle, random_axis_light, random_angle_light, directSpec);
    directspecwoscene->SaveXML(filenames[8]);
    std::cout << filenames[8] << std::endl;
  }

  if (scene_mask[9] != '0') {

    auto directdiffusescene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
 mesh_path, "", random_axis,
 random_angle, random_axis_light, random_angle_light, directDiffuse);
    directdiffusescene->SaveXML(filenames[9]);
    std::cout << filenames[9] << std::endl;
  }

  if (scene_mask[10] != '0') {
    auto directdiffusewoscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
   meshwo_path, "", random_axis,
   random_angle, random_axis_light, random_angle_light, directDiffuse);
    directdiffusewoscene->SaveXML(filenames[10]);
    std::cout << filenames[10] << std::endl;
  }
  if (scene_mask[11] != '0') {
    auto planeonlyscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
     meshwo_path, "", random_axis,
     random_angle, random_axis_light, random_angle_light, texture_only);
    planeonlyscene->SaveXML(filenames[11]);
    std::cout << filenames[11] << std::endl;
  }
  if (scene_mask[12] != '0') {
    auto texonlyscene = buildScene(original_width, original_height, alpha, worldToCam, parser, minHeight, 
   meshwo_path, "", random_axis,
   random_angle, random_axis_light, random_angle_light, texture_inf);
    texonlyscene->SaveXML(filenames[12]);
    std::cout << filenames[12] << std::endl;
  }
  return 0;
}
