FROM ubuntu:16.04
ENV DEBIAN_FRONTEND noninteractive
COPY ./ODO ./ODO
EXPOSE 8000
# Environment variables
ENV domain localhost
ENV LC_CTYPE en_US.UTF-8
RUN apt-get update -y \ && apt-get install -y \
                    software-properties-common \
                    \ x2goserver \ x2goserver-xsession \ kubuntu-desktop \
                && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
                && apt-get update -y\
                && apt-get install -y \
                    make \
                    git \
                    curl \
                    vim \
                    vim-gnome \
                && apt-get install -y cmake=3.5.1-1ubuntu3 \
                && apt-get install -y \ gcc-6 g++-6 gcc-6-base && add-apt-repository ppa:libreoffice/libreoffice-6-0 -y \
# Setup scripts for LibreOffice Online
RUN export CXX=/usr/bin/g++-6 && export CC=/usr/bin/gcc-6
RUN apt-get install build-essential gfortran pkg-config liblapack-dev libblas-dev
RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.tgz
RUN tar xvzf Ipopt-3.12.8.tgz
RUN cd Ipopt-3.12.8/ && cd ThirdParty/Mumps && ./get.Mumps && cd ../.. && ./configure --prefix=/usr/local && make -j5 && make install && pwd && cd ../ODO && cd ./ext_lib/xlnt-1.3.0 && mkdir build && cd build && cmake .. && make -j5 && cd ../../../ && mkdir build && cd build && cmake ..

EXPOSE 9980

# Entry point
CMD bash
