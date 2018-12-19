FROM gcr.io/depthgrid-225300/opencv:latest

RUN cd /root && git clone --single-branch --branch revert https://github.com/kyzyx/mitsuba.git
RUN cp /root/mitsuba/build/config-linux-gcc.py /root/mitsuba/config.py
RUN cd /root/mitsuba && scons