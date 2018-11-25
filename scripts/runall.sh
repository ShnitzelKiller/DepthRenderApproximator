#!/bin/bash

source /projects/grail/jamesn8/Dependencies/mitsuba-master/setpath.sh

render_cmd=/projects/grail/jamesn8/projects/DepthRenderApproximator/depth_grid_render/build/depth_grid_render
quotient_cmd=/projects/grail/jamesn8/projects/DepthRenderApproximator/quotient_image/build/quotient_image

datadir=/local1/edzhang/dataset
hdrdir=/local1/edzhang/HDRMaps
outdir=/projects/grail/jamesn8/projects/DepthRenderApproximator/output/full

suffix=_ALL

cd $hdrdir
allenvmaps=$(ls)
cd -

cd $datadir

allfiles=$(ls)
keyfiles=$(printf "$allfiles" | grep _Y.exr$)
depthfiles=$(printf "$allfiles" | grep Depth)
tmpfile=tmp.exr

echo total files: $(printf "${allfiles}\n" | wc -l)
echo total depth maps: $(printf "${depthfiles}\n" | wc -l)
echo total key files: $(printf "${keyfiles}\n" | wc -l)

cd -

for filename in ${datadir}/*_Y.exr; do
    if [ -f scene_gen.xml ]; then
	rm *.xml
	rm *.exr
	rm *.mtl
	rm *.obj
    fi
    filename=${filename##*/}
    outfile=$outdir/${filename%%_Y.exr}${suffix}.exr
    if [ -f $outfile ]
    then
	echo $outfile already detected, skipping
	continue
    fi

    printf "processing $filename\n\n"
    
    #retrieve depth map
    tag=$(echo $filename | cut -d'_' -f 1)
    echo tag: $tag
    depth_map=$(printf "$depthfiles" | grep ^${tag} | grep $suffix)
    num_depth_maps=$(printf "${depth_map}\n" | wc -l)
    if [ ! $num_depth_maps -eq 1 ]; then
	echo "warning: single depth map not found (found $num_depth_maps):"
	printf "${depth_map}\n"
	continue
    fi
    echo "depth map: $depth_map"
    
    #retrieve env map
    env_map_name=${filename#${tag}_????_?????}
    env_map_name=${env_map_name%%_Theta*}
    echo env map name: $env_map_name
    env_map=$(printf "$allenvmaps" | grep $env_map_name)
    num_env_maps=$(printf "${env_map}\n" | wc -l)
    if [ ! $num_env_maps -eq 1 ]; then
	echo "warning: single env map not found (found $num_env_maps)"
	printf "${env_map}\n"
	env_map=$(printf "${env_map}\n" | grep -v mip$)
	if [ -z $env_map ]; then
	    continue
	fi
	echo discarding MIP file
    fi
    echo "env map: $env_map"

    params=${filename##*${env_map_name}_}
    echo $params

    theta=$(echo $params | cut -d'_' -f 1)
    theta=${theta:5}
    echo theta: $theta

    phi=$(echo $params | cut -d'_' -f 2)
    phi=${phi%Light}
    phi=${phi:3}
    echo phi: $phi

    light_theta=$(echo $params | cut -d'_' -f 3)
    light_theta=${light_theta:5}
    echo light theta: $light_theta

    light_phi=$(echo $params | cut -d'_' -f 4)
    light_phi=${light_phi:3}
    echo light phi: $light_phi

    alpha=$(echo $params | cut -d'_' -f 5)
    echo alpha: $alpha

    $render_cmd $datadir/$depth_map $hdrdir/$env_map $theta $phi $alpha -ltheta $light_theta -lphi $light_phi
    echo "Render lighting image"
    mitsuba scene_gen.xml -o $tmpfile
    echo "Compute diffuse reflectance"
    $quotient_cmd $datadir/$filename $tmpfile texture.exr
    echo "Render full image"
    mitsuba scene_gen_tex.xml -o $outfile
    #exit #remove when this actually works
done
