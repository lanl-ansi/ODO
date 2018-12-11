FROM ubuntu:16.04
ENV DEBIAN_FRONTEND noninteractive
COPY ODO ./ODO
EXPOSE 8000
# Environment variables
ENV domain localhost
ENV LC_CTYPE en_US.UTF-8

RUN         apt-get update \ && apt-get install -y \
                    software-properties-common \
                    wget \
                && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
                && apt-get update \
                && apt-get install -y \
                    make \
                    git \
                    curl \
                    vim \
                    vim-gnome \
                && apt-get install -y cmake=3.5.1-1ubuntu3 \
                && apt-get install -y \
                    gcc-6 g++-6 gcc-6-base \
                && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 100 \
                && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 100

# Setup scripts for LibreOffice Online
ADD /scripts/install-libreoffice.sh /
ADD /scripts/start-libreoffice.sh /
RUN bash install-libreoffice.sh

EXPOSE 9980

# Entry point
CMD bash start-libreoffice.sh &
