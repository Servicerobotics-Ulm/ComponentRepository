Install full details:

librealsense2 from git repo:
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

STEPS (16.04)
===========================


echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo apt-get update
sudo apt-get install realsense-uvcvideo librealsense2-utils librealsense2-dev librealsense2-dbg

(reboot)
===========================
