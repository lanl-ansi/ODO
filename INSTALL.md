DEPENDENCIES
-------
cmake: http://www.cmake.org (Version 3.2 or better)

gcc: https://gcc.gnu.org/install/index.html or clang: https://clang.llvm.org/get_started.html 

gfortran: https://gcc.gnu.org/wiki/GFortranBinaries


Included ThirdParty Libraries
-------


Armadillo: https://github.com/conradsnicta/armadillo-code

Ipopt: https://projects.coin-or.org/Ipopt

XLNT: https://github.com/tfussell/xlnt

cpp_option_parser: https://github.com/nerzadler/cpp_option_parser


OPTIONAL
-------


Bonmin: https://projects.coin-or.org/Bonmin

Cplex: https://www-01.ibm.com/software/commerce/optimization/cplex-optimizer/


-------

First, make sure cmake, gfortran and gcc-6 or clang are installed.

### Windows Users

We are working on adding binaries for Windows users, check back soon!

### Mac Users

For Mac users, simply download and run the installers for the dependencies from the websites:

cmake: http://www.cmake.org (Version 3.2 or better)

clang: https://clang.llvm.org/get_started.html 

gfortran: https://gcc.gnu.org/wiki/GFortranBinaries


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

If you're using gcc-6, enter:

* `export CC=/usr/bin/gcc-6`

* `export CXX=/usr/bin/g++-6`

If using clang, enter:

* `export CC=/usr/bin/clang`

* `export CXX=/usr/bin/clang`

### Compiling the ODO tool

Now, we're ready to compile **ODO** and its dependencies, enter:

* `cd ./ext_lib/xlnt-1.3.0`

* `mkdir build && cd build && cmake ..`

* `make -j5 && cd ../../../`

* `mkdir build && cd build && cmake ..`

* `make -j5`

#### Linux Virtual Machine

If you're using a Linux virtual machine, enter the following command before running the tool:

* `cd ../ext_lib/Ipopt/lib && cp * /usr/local/lib`



The **ODO** executable will can be found under `ODO/bin`.

For running the tool, enter:

* `../bin/odo`



