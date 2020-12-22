##check if unicap libunicap2-dev from 18.04/20.04 is working 




Requires Unicap 0.9.8 from source.

git clone https://github.com/unicap/unicap.git
cd unicap
git checkout unicap-0-9-8 
./configure
make all
sudo make install

Additionally, install from ubuntu 12.04:
apt-get install libv4l-dev intltool

Compilation error, videodev.h missing:

$ cd /usr/include/linux
$ sudo ln -s ../libv4l1-videodev.h videodev.h

Test your camera with cheese. Edit->preferences->device will stat the name of the device as requested in component parameters.
