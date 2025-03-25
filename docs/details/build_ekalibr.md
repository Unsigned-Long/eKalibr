<div style="text-align: center;">
    <img src="../img/logo.svg" style="width: 100vw; height: auto;">
</div>

---

<h3 align="center">Tutorial: Prepare eKalibr Environment for Calibration</h3>
<p align="center">
    <a href="https://github.com/Unsigned-Long"><strong>Author » Shuolong Chen</strong></a>
</p>

---

**Attention, attention!** If you are using `Ubuntu 20.04`, please directly jump to ***<u>A Tested Install Pipeline On Ubuntu 20.04</u>*** section (at the bottom of this file), where you will find the tested environment dependencies and versions of third-party libraries. If you are using a different version of`Ubuntu`, you can also prepare `eKalibr` environment in a similar way, but you may need to further adapt the library versions.

**Key point** (you can't skip this part):

+ both `spdlog` and `Sophus` would involve the `fmt` library, and would lead to conflict if the versions of involved `fmt` in `spdlog` and `Sophus` are different. To solve this, we recommend that you set the `cmake` option `SOPHUS_USE_BASIC_LOGGING` as `ON` when compile `Sophus`, this would avoid to involve `fmt` logger dependency in `Sophus`. If conflict about `fmt` still exists, it is recommended that you try to compile the `spdlog` library from [source code](https://github.com/gabime/spdlog.git) (directly compile and install), so that `spdlog` will use the internal `fmt` library.
+ Third-party dependency libraries that do not specify version numbers use the system default version. For example, the
  default `pcl` version of `Ubuntu 20.04` is `1.10` (`pcl` can be installed together with `ROS`).



<p align="left">
    <a><strong>Clone eKalibr and Its Modules »</strong></a>
</p> 



+ create a ros workspace if needed and clone `eKalibr` to `src` directory as `ekalibr`:

  ```sh
  mkdir -p ~/eKalibr/src
  cd ~/eKalibr/src
  
  git clone --recursive https://github.com/Unsigned-Long/eKalibr.git ekalibr
  ```

  **Key point** (you can't skip this part): the ros package name is `ekalibr` (the letter `k` is lowercase), rather than
  `eKalibr` (the letter `K` is uppercase).

  change directory to '`ekalibr`', and run '`build_thirdparty.sh`'.

  ```sh
  cd ekalibr
  chmod +x build_thirdparty.sh
  ./build_thirdparty.sh
  ```

  this would build sub module libraries, namely `tiny-viewer`, `ctraj`, `veta`, and `opengv`.




<p align="left">
    <a><strong>Compile eKalibr »</strong></a>
</p> 



+ change directory to catkin workspace, and generate the ros self-defined messages in `ekalibr`:

  ```sh
  cd ../..
  catkin_make ekalibr_generate_messages
  ```

+ compile `ekalibr` package:

  ```sh
  catkin_make -j8
  ```

Congratulations :clap:, if everything goes well and no error happened.



<p align="left">
    <a><strong>A Tested Install Pipeline On Ubuntu 20.04 »</strong></a>
</p> 



Considering the common issues reported by most people regarding the configuration and compilation of the `eKalibr`
environment, I will provide here a step-by-step guide for setting up the `eKalibr` environment from scratch on an
`Ubuntu 20.04` system. All steps have been tested and successfully compiled `eKalibr` on `Ubuntu 20.04`.

**Attention:** This install pipeline has already been, and only been tested on `Ubuntu 20.04`.

```sh
# ros 1 (noetic), add install source and key first, see https://wiki.ros.org/noetic/Installation/Ubuntu
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# cmake (3.30.1)
wget https://github.com/Kitware/CMake/releases/download/v3.30.1/cmake-3.30.1.tar.gz && \
    tar -zxvf cmake-3.30.1.tar.gz && \
    cd cmake-3.30.1 && \
    ./bootstrap && \
    make -j$(nproc) && \
    make install && \
    cd .. && \
    rm -rf cmake-3.30.1 cmake-3.30.1.tar.gz

# eigen (3.4, for glomap lib)
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -zxvf eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# ceres (2.2.0)
git clone --branch 2.2.0 --single-branch --recurse-submodules https://github.com/ceres-solver/ceres-solver
cd ceres-solver
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
# tips: to find glog lib correctly in colmap compiling, please refine the 'FindGlog.cmake', i.e., 
# modilfy file'/usr/local/lib/cmake/Ceres/FindGlog.cmake', line 348 with (add 'if' and 'endif'):
# -----------------------------------------------------------------------
# if(NOT TARGET glog::glog)
#   add_library(glog::glog INTERFACE IMPORTED)
#   target_include_directories(glog::glog INTERFACE ${GLOG_INCLUDE_DIRS})
#   target_link_libraries(glog::glog INTERFACE ${GLOG_LIBRARY})
# endif()
# -----------------------------------------------------------------------

# Sophus (1.22.10)
git clone --branch 1.22.10 --single-branch  https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc)
sudo make install

# magic_enum (v0.9.6)
git clone --branch v0.9.6 --single-branch https://github.com/Neargye/magic_enum.git
cd magic_enum
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# pangolin (v0.8)
git clone --branch v0.8 --single-branch --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
mkdir build && cd build
cmake .. -GNinja
ninja
sudo ninja install

# spdlog (the newest, use internal fmt)
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake ..
cmake --build .
sudo make install

# install cereal and yaml-cpp
sudo apt-get install libcereal-dev
sudo apt-get install libyaml-cpp-dev

# ros packages
sudo apt-get install ros-noetic-cv-bridge

# clone eKalibr
mkdir -p ~/eKalibr/src
cd ~/eKalibr/src
git clone --recursive https://github.com/Unsigned-Long/eKalibr.git ekalibr

# build thirdparty
cd ekalibr
chmod +x build_thirdparty.sh
./build_thirdparty.sh

# build eKalibr
cd ../..
catkin_make ekalibr_generate_messages
catkin_make -j8
```
