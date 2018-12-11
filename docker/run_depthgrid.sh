#!/bin/bash

docker container inspect depthgrid &> /dev/null

if [ $? == 0 ]; then
    docker container start -i depthgrid
else
    if [ $# -lt 1 ]; then
	echo usage: run_depthgrid.sh depthgrid_root
	exit 1
    fi

    path=$1
    echo host source path: $path

    docker run -it --name depthgrid -v $depthgrid_root:/data opencv-mitsuba-custom
fi
