#!/bin/bash
cd ros_ws/src

# Being libfreenect2 installation 
git clone https://github.com/OpenKinect/libfreenect2.git

sudo apt-get install build-essential libturbojpeg libjpeg-turbo8-dev libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev automake

cd libfreenect2/depends
sh install_ubuntu.sh
sudo dpkg -i libglfw3*_3.0.4-1_*.deb

sudo add-apt-repository ppa:xorg-edgers
sudo apt-get install opencl-headers

cd ~/ros_ws/src/libfreenect2 
mkdir build && cd build
cmake .. -DENABLE_CX11=ON
make
sudo make install

cd ~/ros_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
# End libfreenect2 installation

# Begin iai_kinect2 installation
cd ~/ros_ws/src
sudo cp libfreenect2/rules/90-kinect2.rules /etc/udev/rules.d/ 
read -p "Unplug and then reconnect the Kinect2 sensor, then press any key to continue..." -n1 -s
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/ros_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
# End iai_kinect2 installation
