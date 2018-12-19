FROM ubuntu:18.04
RUN apt-get update
RUN apt-get install -y git scons build-essential libpng-dev libjpeg-dev libilmbase-dev libxerces-c-dev libboost-all-dev libopenexr-dev libglewmx-dev libxxf86vm-dev libpcrecpp0v5 libeigen3-dev libfftw3-dev libglu-dev
RUN apt-get install -y cmake
RUN cd /root && git clone --single-branch -b 3.4 https://github.com/opencv/opencv.git
RUN cd /root/opencv && mkdir build && cd build && cmake ..
RUN cd /root/opencv/build && make
RUN cd /root/opencv/build && make install

