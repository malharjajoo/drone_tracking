# Installation Script
This is an installation script for the various libraries required for this project implementation code. Most of it can be easily found using StackOverflow or other similar resources, but still has been provided here for convenience of the user. However, if there are any issues, the user is requested to kindly first look up StackOverflow (or similar forums) or contact me ( find my contact from my [website](https://malharjajoo.github.io/myPortfolio/) ).

#### OpenCV installation:

Most of below script for opencv installation is obtained from the script provided by 
Francesco Piscani [here](https://github.com/cesco345/StemApks/blob/master/TutorialsNotebook%20(1).ipynb).


``` sh
sudo apt-get update
sudo apt-get upgrade

###### 1. Download Scipy Stack:
sudo apt-get install python-numpy  python-matplotlib python-pandas python-sympy python-nose python-scipy
       
##### 2. Download opencv:

#install basic development environment
sudo apt-get install build-essential cmake pkg-config unzip

# install dependencies of opencv .... 
sudo apt-get install build-essential cmake git
sudo apt-get install pkg-config unzip ffmpeg  python-dev python3-dev python-numpy python3-numpy
sudo apt-get install libopencv-dev libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev 
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev 
sudo apt-get install libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libtheora-dev 
sudo apt-get install libvorbis-dev libxvidcore-dev v4l-utils libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install libjasper-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install libjpeg8-dev libx264-dev libatlas-base-dev gfortran


# download and unzip openCV source folder.
cd ~
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.1.zip
unzip opencv.zip

# compile opencv
cd opencv-3.3.1/
mkdir build
cd build

####### Note: Imp! - Do NOT forget the ".." at the end of the command below.
cmake -D CMAKE_BUILD_TYPE=RELEASE -D INSTALL_C_EXAMPLES=ON  
        -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON
        -D WITH_QT=ON -D CMAKE_INSTALL_PREFIX=/usr/local
        -D WITH_OPENGL=ON -D WITH_V4L=ON -D WITH_CUDA=ON
        -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_TBB=ON ..
       
        
#The -j option simply speeds up compilation on a multi-core processor.
#Check Manual/Man page for make.
make -j4 
sudo make install

sudo nano /etc/ld.so.conf.d/opencv.conf

#type this into the file
/usr/local/lib

sudo ldconfig
sudo nano /etc/bash.bashrc

PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH

```

---

#### ROS Kinetic installation:

The ROS wiki page ([see here](http://wiki.ros.org/kinetic/Installation/Ubuntu)) provides all details required. 
Kindly install the Desktop-full version. 

---

#### libVLC SDK installaton:

More information regarding the installation can be found at [VLC official installation guide](https://wiki.videolan.org/LibVLC_Tutorial/).

``` sh
# Optional command to find dev package, if library below isn't found.
apt-cache search libvlc 

sudo apt-get install libvlc-dev

#To check installed version
pkg-config --modversion libvlc
```
---

#### Eigen installaton:  

``` sh
sudo apt install libeigen3-dev

#To check installed version
pkg-config --modversion eigen3
```

---


#### Simple DirectMedia Layer (SDL) installation: 

``` sh
sudo apt-get install libsdl1.2-dev

#To check installed version
pkg-config --modversion sdl
``` 

