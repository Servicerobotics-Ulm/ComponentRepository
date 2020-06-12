
Installing libfreenect for Kinect v1
_____________________________________________________________

#Via debian packages

sudo apt-get install freenect 

# install mrpt and opencv dependencies
sudo apt-get install libmrpt-vision1.3 
sudo apt-get install libmrpt-hwdrivers1.3 
sudo apt-get install libopencv-calib3d3.1 


______________________________________________________________

#Via Source building

sudo apt-get install git cmake build-essential libusb-1.0-0-dev

#only if you are building the examples:

sudo apt-get install freeglut3-dev libxmu-dev libxi-dev

git clone https://github.com/OpenKinect/libfreenect

cd libfreenect

mkdir build

cd build

cmake -L .. # -L lists all the project options

make

sudo make install_____________________________________________________________________________________________

Installation online info : https://github.com/OpenKinect/libfreenect#linux
_____________________________________________________________________________________________

libfreenect license :  https://github.com/OpenKinect/libfreenect/blob/master/APACHE20
_____________________________________________________________________________________________
