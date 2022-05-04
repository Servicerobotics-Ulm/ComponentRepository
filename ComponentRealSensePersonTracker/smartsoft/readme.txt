################ Ubuntu 20.04 ######################################
ComponentRealSensePersonTracker is using the person tracker library provided by Intel RealSense.
Intel RealSense had stopped providing the updates for person tracking library and there is not official support for Ubuntu 20.04.
Person tracking libraries from Ubuntu 16.04 are used to link with the component.

1)Compile and install OpenCV 3.1
wget https://github.com/opencv/opencv/archive/refs/tags/3.1.0.tar.gz
tar -xvzf 3.1.0.tar.gz
cd opencv-3.1.0/
mkdir build
cd build
cmake -DWITH_VTK=OFF -DWITH_FFMPEG=OFF -DENABLE_PRECOMPILED_HEADERS=OFF ..
make
sudo make install

2) Copy the data folder (from src/3rdParty) to any location and provide the path in 'pt_data' parameter.
################ Ubuntu 16.04(Deprecated) ######################################
Install librealsense + realsense_sdk_zr300:

https://github.com/IntelRealSense/realsense_sdk_zr300
https://software.intel.com/sites/products/realsense/intro/getting_started.html


==========================================
Since the realsense sdk is using opencv3.0, the most easy installation is installing everything from deb packages:

==========================================
COMMON
==========================================

=========================================
1. Add repo

echo 'deb "http://realsense-alm-public.s3.amazonaws.com/apt-repo" xenial main' | sudo tee /etc/apt/sources.list.d/realsense-latest.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key D6FB2970 
sudo apt update

=========================================
2. Uddate the opencv install from 2.4 to 3.1

# Solve conflicts be removing cv2.4, use aptitude!

=========================================
3. Remove opencv2.4 java packages

libopencv2.4-jni libopencv2.4-java

==========================================
FOR RUNTIME
==========================================

=========================================
1. Install opencv 3.1 packages (run-time)

libopencv-highgui3.1


=========================================
2. Install realsense packges (run-time)
librealsense1
lirealsense-persontracking0


==========================================
FOR DEVELOPMENT
==========================================

=========================================
1. Install realsense packages

librealsense-dev
librealsense-persontracking-dev


=========================================
2. Install opencv dev packages

#install opencv3.1 dev packages, solve conflicts be removing 2.4:
libopencv-dev

