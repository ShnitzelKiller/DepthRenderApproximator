#!/bin/bash

source /projects/grail/jamesn8/Dependencies/mitsuba-master/setpath.sh

render_cmd=/projects/grail/jamesn8/projects/DepthRenderApproximator/depth_grid_render/build/depth_grid_render
quotient_cmd=/projects/grail/jamesn8/projects/DepthRenderApproximator/quotient_image/build/quotient_image

datadir=/local1/edzhang/dataset
hdrdir=/local1/edzhang/HDRMaps
outdir=/projects/grail/jamesn8/projects/DepthRenderApproximator/output/full-randomized-augmented

masksuffix=_OBJ
depthsuffix=_ALL
outputsuffix=_ALL
outputwosuffix=_WO
outputspecsuffix=_SPEC
outputwospecsuffix=_WOSPEC
outputdirectspecsuffix=_DSPEC
outputwodirectspecsuffix=_WODSPEC
outputdirectdiffusesuffix=_DDIFF
outputwodirectdiffusesuffix=_WODDIFF
texsuffix=_TEX
paramsuffix=_PARAM

#### TODO: EDIT THESE TO CHANGE OUTPUTS (CURRENTLY UNUSED) ####

render_base=0
render_tex=0
render_wotex=0
render_flipped=0
render_woflipped=0
render_spec=1
render_wospec=1
render_directspec=1
render_wodirectspec=1
render_directdiffuse=1
render_wodirectdiffuse=1

######################################

cd $hdrdir
allenvmaps=$(ls)
cd -

cd $datadir

maxcpus=12
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
    namebase=${filename%%_Y.exr}
    outfilew=$outdir/${namebase}${outputsuffix}.exr
    outfilewo=$outdir/${namebase}${outputwosuffix}.exr
    outfilespec=$outdir/${namebase}${outputspecsuffix}.exr
    outfilewospec=$outdir/${namebase}${outputwospecsuffix}.exr
    outfiledirectspec=$outdir/${namebase}${outputdirectspecsuffix}.exr
    outfilewodirectspec=$outdir/${namebase}${outputwodirectspecsuffix}.exr
    outfiledirectdiffuse=$outdir/${namebase}${outputdirectdiffusesuffix}.exr
    outfilewodirectdiffuse=$outdir/${namebase}${outputwodirectdiffusesuffix}.exr
    outtex=$outdir/${namebase}${texsuffix}.exr
    if [ -f $outfilew ]
    then
	echo $outfilew already detected, skipping #TODO: Render only relevant missing outputs, but don't skip
	continue
    fi

    printf "processing $filename\n\n"
    
    tag=$(echo $filename | cut -d'_' -f 1)
    echo tag: $tag
    files=$(printf "$depthfiles" | grep ^${tag})

    #retrieve depth map
    depth_map=$(printf "$files" | grep $depthsuffix)
    num_depth_maps=$(printf "${depth_map}\n" | wc -l)
    if [ ! $num_depth_maps -eq 1 ]; then
	echo "warning: single depth map not found (found $num_depth_maps):"
	printf "${depth_map}\n"
	continue
    fi
    echo "depth map: $depth_map"

    #retrieve mask
    mask_map=$(printf "$files" | grep $masksuffix)
    num_mask_maps=$(printf "${mask_map}\n" | wc -l)
    if [ ! $num_mask_maps -eq 1 ]; then
	echo "warning: single mask map not found (found $num_mask_maps):"
	printf "${mask_map}\n"
	continue
    fi
    echo "mask image: $mask_map"
    
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

    $render_cmd $datadir/$depth_map $hdrdir/$env_map $theta $phi $alpha $datadir/$mask_map -tex $outtex -ltheta $light_theta -lphi $light_phi -randalpha 0.01 -randang 5 -save $tag -output_masks 1 #-scenes ${render_base}${render_tex}${render_wotex}${render_flipped}${render_woflipped}${render_spec}${render_wospec}${render_directspec}${render_wodirectspec}${render_directdiffuse}${render_wodirectdiffuse}

    #TODO: properly detect what renders to run based on the mask! also see above scenes argument
    echo "Render lighting image"
    mitsuba scene_gen.xml -o $tmpfile -p $maxcpus
    echo "Compute diffuse reflectance"
    $quotient_cmd $datadir/$filename $tmpfile $outtex
    echo "Render full image"
    mitsuba scene_gen_tex.xml -o $outfilew -p $maxcpus
    echo "Render full image without object"
    mitsuba scenewo_gen_tex.xml -o $outfilewo -p $maxcpus
    echo "Render specular map"
    mitsuba scene_gen_spec.xml -o $outfilespec -p $maxcpus
    echo "Render specular map without object"
    mitsuba scenewo_gen_spec.xml -o $outfilewospec -p $maxcpus
    echo "Render direct diffuse"
    mitsuba scene_gen_directdiffuse.xml -o $outfiledirectdiffuse -p $maxcpus
    echo "Render direct diffuse without object"
    mitsuba scenewo_gen_directdiffuse.xml -o $outfilewodirectdiffuse -p $maxcpus
    echo "Render direct specular"
    mitsuba scene_gen_directspec.xml -o $outfiledirectspec -p $maxcpus
    echo "Render direct specular without object"
    mitsuba scenewo_gen_directspec.xml -o $outfilewodirectspec -p $maxcpus
    #exit #remove when this actually works
done
