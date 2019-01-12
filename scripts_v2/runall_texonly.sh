#!/bin/bash

source /projects/grail/jamesn8/Dependencies/mitsuba-master/setpath.sh
render_cmd=/projects/grail/jamesn8/projects/DepthRenderApproximator/depth_grid_render/build-sanka/depth_grid_render
quotient_cmd=/projects/grail/jamesn8/projects/DepthRenderApproximator/quotient_image/build/quotient_image
mitsuba_cmd=/projects/grail/jamesn8/Dependencies/mitsuba-master/dist/mitsuba

datadir=/projects/grail/edzhang/differential/data
hdrdir=/projects/grail/edzhang/hdrmaps
outdir=/projects/grail/jamesn8/projects/DepthRenderApproximator/output/texonly_hires
texdir=/projects/grail/edzhang/texturedata/textures/train

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
    outfile=$outdir/${namebase}_TEXONLY.exr
    
    if [ -f $outfile ]
    then
	echo $outfile already detected, skipping #TODO: Render only relevant missing outputs, but don't skip
	continue
    fi

    printf "processing $filename\n\n"
    
    tag=$(echo $filename | cut -d'_' -f 1)
    echo tag: $tag
    files=$(printf "$depthfiles" | grep ^${tag})

    #retrieve texture
    texturename=$(echo $filename | cut -d'_' -f 2)
    textureid=${texturename:(-4)}
    texturetype=${texturename%$textureid}
    echo texture: $texturename
    echo texture type: $texturetype
    echo texture id: $textureid
    for tex in $texdir/$texturetype/*${textureid}.png; do
	texture=$tex
    done
    echo texture path: $texture

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
    env_map_name=$(echo $filename | cut -d'_' -f 3)
    env_map_name=${env_map_name#?????}
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
    
    $render_cmd $datadir/$depth_map $hdrdir/$env_map $theta $phi $alpha $datadir/$mask_map -ltheta $light_theta -lphi $light_phi -save $outdir/$tag -planetex $texture -nomodel 1 -width 1280 -height 720 -scenes 000000000001 #-scenes ${render_base}${render_tex}${render_wotex}${render_flipped}${render_woflipped}${render_spec}${render_wospec}${render_directspec}${render_wodirectspec}${render_directdiffuse}${render_wodirectdiffuse}
    
    echo "Rendering textures only"
    mitsuba ${outdir}/${tag}_scene_gen_texonly.xml -o $outfile
done
