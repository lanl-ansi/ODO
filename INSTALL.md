# RUN IN DOCKER

**ODO** is deployed on [Docker](www.docker.com) to support multiplatforms.

To get the tool running, you just need to install Docker:

For Windows users follow this [link](https://hub.docker.com/editions/community/docker-ce-desktop-windows)

For Mac users follow this [link](https://hub.docker.com/editions/community/docker-ce-desktop-mac)

For Linux users follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce)

Once docker is installed and running, open a terminal (in Windows use PowerShell or Commad Prompt) and enter the following:

`docker pull hhijazi01/odo`

`docker run -it hhijazi01/odo`

This will take you to the Docker container that contains the ODO tool.

Now you can run the tool by entering:

`ODO/bin/odo`

That's it!

Everytime there's an update to the tool, it will be pushed to the Docker container so make sure to follow the steps above including the pull command.

# COMPILE FROM SOURCE CODE

DEPENDENCIES
-------
cmake: http://www.cmake.org (Version 3.2 or better)

gcc: https://gcc.gnu.org/install/index.html or clang: https://clang.llvm.org/get_started.html 

gfortran: https://gcc.gnu.org/wiki/GFortranBinaries

Ipopt: https://projects.coin-or.org/Ipopt

Included ThirdParty Libraries
-------


Armadillo: https://github.com/conradsnicta/armadillo-code

XLNT: https://github.com/tfussell/xlnt

cpp_option_parser: https://github.com/nerzadler/cpp_option_parser


OPTIONAL
-------


Bonmin: https://projects.coin-or.org/Bonmin

Cplex: https://www-01.ibm.com/software/commerce/optimization/cplex-optimizer/


-------

First, make sure cmake, gfortran, gcc-6 or clang and ipopt are installed.

### Windows Users

We are working on adding instructions for Windows users, check back soon!

### Mac Users

For Mac users, simply download and run the installers for the dependencies from the websites:

cmake: http://www.cmake.org (Version 3.2 or better)

clang: https://clang.llvm.org/get_started.html 

gfortran: https://gcc.gnu.org/wiki/GFortranBinaries

Ipopt: https://www.coin-or.org/Ipopt/documentation/node10.html

### Linux Users 

Here are a set of instructions to install cmake, gfortran and gcc-6 on Ubuntu:


* `sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y`

* `sudo apt update`

* `sudo apt-get upgrade`

* `sudo apt-get install cmake`

* `sudo apt-get install gfortran`

* `sudo apt-get install libgfortran3`

* `sudo apt-get install gcc-6 g++-6`


If you're having trouble with gcc-6 try installing clang:

* `wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -`

* `sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-6.0 main"`

* `sudo apt-get update`

* `sudo apt-get install -y clang-6.0`

* `sudo apt-get install cmake`

* `sudo apt-get install gfortran`

* `sudo apt-get install libgfortran3`



To build **ODO**, follow the instructions below:

### Compiling the ODO tool

If you're using gcc-6, enter:

* `export CC=/usr/bin/gcc-6`

* `export CXX=/usr/bin/g++-6`

If using clang, enter:

* `export CC=/usr/bin/clang`

* `export CXX=/usr/bin/clang`

Now, we're ready to compile **Ipopt** and its dependencies, enter:

* `sudo apt-get install build-essential gfortran pkg-config liblapack-dev libblas-dev`

* `wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.tgz`

* `tar xvzf Ipopt-3.12.8.tgz`

* `cd Ipopt-3.12.8/`

* `cd ThirdParty/Mumps`

* `./get.Mumps`

* `cd ../..`

* `./configure --prefix=/usr/local`

* `make -j5`

* `sudo make install`

* `cd ../`


Now, we're ready to compile **ODO** and its dependencies, enter:

* `cd ./ext_lib/xlnt-1.3.0`

* `mkdir build && cd build && cmake ..`

* `make -j5 && cd ../../../`

* `mkdir build && cd build && cmake ..`

* `make -j5`

#### Linux Virtual Machine

If you're using a Linux virtual machine, enter the following command before running the tool:

* `cp ../ext_lib/Ipopt/lib/* /usr/local/lib`


The **ODO** executable will can be found under `ODO/bin`.

For running the tool, enter:

* `../bin/odo`



