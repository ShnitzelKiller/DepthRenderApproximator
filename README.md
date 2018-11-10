# Approximate Rendering Data Generation

The tools contained in this package are meant to generate renders using single depth maps as input, along with material and lighting parameters to be used with Mitsuba to generate the final image. The only functioning tool resides in the depth_grid_render directory, and a script to batch process data is in the scripts directory. Depth_grid_render takes some specifications of light and camera positions along with a depth map and generates a Wavefront OBJ mesh and a Mitsuba scene in XML format for rendering the recontsructed scene.

## Building
```bash
cd depth_grid_render
mkdir build
cd build
cmake ..
make
```

## Running the code
```
depth_grid_render filename envmap theta phi alpha [-ltheta <value> -lphi <value>] [-c <occlusion_threshold>] [-d <displacement_factor>] [-s <scene_format_version>]
```
will create a scene_gen.xml and output_mesh.obj in the current directory.

scripts/runall_v3.sh parses the particular filenames of the datasets we're using and runs the depth_grid_render followed by Mitsuba on each resulting reconstruction to produce a series of renders that should approximate the source scene.