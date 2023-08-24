# Compliling on Linux  
## 1.Install PCL
* Run these commands to install dependencies first:

           sudo apt-get update
           sudo apt-get install git build-essential linux-libc-dev -y
           sudo apt-get install cmake -y
           sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev -y
           sudo apt-get install mpi-default-dev openmpi-bin openmpi-common -y
           sudo apt-get install libflann1.9 libflann-dev -y
           sudo apt-get install libeigen3-dev -y
           sudo apt-get install libboost-all-dev -y
           sudo apt-get install libvtk7.1p-qt libvtk7.1p libvtk7-qt-dev -y
           sudo apt-get install libqhull* libgtest-dev -y
           sudo apt-get install freeglut3-dev pkg-config -y
           sudo apt-get install libxmu-dev libxi-dev -y
           sudo apt-get install mono-complete -y
           sudo apt-get install openjdk-8-jdk openjdk-8-jre -y
           
* Then visit [PCL Docs](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html) to build and install PCL.
## 2. Install igraph
* Tutorials of install igraph can be found at [igraph Reference Manual](https://igraph.org/c/doc/igraph-Installation.html).
```
# Tested in Ubuntu 20.04

# download the latest cmake for 3D MAC. igraph 0.9.9 is satisfied with cmake 3.16.
wget https://github.com/Kitware/CMake/releases/download/v3.27.4/cmake-3.27.4-linux-x86_64.tar.gz
tar -zxvf cmake-3.27.4-linux-x86_64.tar.gz -C /home/jhuai/Documents/slam_devel

# install components for cmake 3.27.4
sudo apt-get install -y bison flex # Unknown CMake command "bison_target", "flex_target".

git clone --recursive git@github.com:igraph/igraph.git
cd igraph
git checkout tags/0.9.9
mkdir build && cd build

/home/jhuai/Documents/slam_devel/cmake-3.27.4-linux-x86_64/bin/cmake .. -DCMAKE_INSTALL_PREFIX=/home/jhuai/Documents/slam_devel
make -j4
make install

```

## 3. Build MAC
- Option 1 (purely on the command line): Use CMake to generate Makefiles and then `make`.
    - You can simply run
      ```
      $ cd Linux
      $ mkdir Release
      $ cd Release
      $ /home/jhuai/Documents/slam_devel/cmake-3.27.4-linux-x86_64/bin/cmake .. -DCMAKE_BUILD_TYPE=Release -Digraph_DIR=/home/jhuai/Documents/slam_devel/lib/cmake/igraph
      $ make
      ```
- Option 2: Use any IDE that can directly handle CMakeLists files to open the `CMakeLists.txt` in the **root** directory of MAC. Then you should have obtained a usable project and just build it. I recommend using [CLion](https://www.jetbrains.com/clion/).
- NOTICE: Please compile in **RELEASE** mode!
