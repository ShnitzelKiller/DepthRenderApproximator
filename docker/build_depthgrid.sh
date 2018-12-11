#!/bin/bash
if [ $# -lt 1 ]; then
    echo usage: build_depthgrid.sh depthgrid_root
    exit 1
fi

src=$1
echo source: $src

if [ ! -d /root/depthgrid ]; then
    mkdir /root/depthgrid
fi

if [ ! -f /root/depthgrid/CMakeLists.txt ]; then
    ln -s $src/depth_grid_render/CMakeLists.txt /root/depthgrid
fi
if [ ! -d /root/depthgrid/src ]; then
    ln -s $src/depth_grid_render/src /root/depthgrid
fi
if [ ! -d /root/depthgrid/scripts ]; then
    ln -s $src/scripts /root/depthgrid
fi

cd /root/depthgrid
mkdir bin
cd bin
cmake ..
make
