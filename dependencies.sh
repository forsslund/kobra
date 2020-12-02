# For H3D Ubuntu 20.04 TLS
sudo apt install libalut-dev libopenal-dev libglew-dev freeglut3 freeglut3-dev
sudo apt install libcurl4-openssl-dev libaudiofile-dev libfontconfig1-dev libfontconfig1
sudo apt install libmotif-common libvorbis-dev python-dev python-all-dev
sudo apt install libfreeimage-dev libxmu-dev libxi-dev libwxgtk3.0-gtk3-dev bzip2 libpng-dev libbz2-dev
sudo apt install doxygen bison flex texlive-latex-extra texlive-xetex
sudo apt install python3-scipy autoconf automake libtool
sudo apt install subversion python-tk patch libwxsmithlib-dev libteem-dev
sudo apt install libglw1-mesa libxxf86vm-dev ssh qt5-default qtcreator libusb-1.0-0-dev 
sudo apt install cmake gcc g++ libxerces-c-dev libftgl-dev libopenal1 libalut0

# Download and install boost manually (to get version prior to 1.70)
#https://sourceforge.net/projects/boost/files/boost/1.62.0/boost_1_62_0.tar.bz2/download

# in /usr/local
#  sudo tar -xjvf boost_1_62_0.tar.bz2 boost_1_62_0/
#  cd boost_1_62_0/
#
#  sudo ./bootstrap.sh 
#  sudo ./b2 address-model=64
#  sudo ./b2 install


# For Ubuntu 18.04 LTS
#sudo apt install libalut-dev libopenal-dev libglew-dev freeglut3 freeglut3-dev
#sudo apt install libcurl4-openssl-dev libaudiofile-dev libfontconfig1-dev libfontconfig1
#sudo apt install libmotif-common libvorbis-dev python-dev python-all-dev
#sudo apt install libfreeimage-dev libxmu-dev libxi-dev libwxgtk3.0-dev bzip2 libpng-dev libbz2-dev
#sudo apt install doxygen bison flex texlive-latex-extra texlive-xetex
#sudo apt install python-scipy python-scipy autoconf automake libtool
#sudo apt install subversion python-tk patch libwxsmithlib-dev libteem-dev
#sudo apt install libglw1-mesa libxxf86vm-dev ssh qt5-default qtcreator libusb-1.0-0-dev libboost-all-dev
#sudo apt install cmake gcc g++ libxerces-c-dev libftgl-dev libopenal1 libalut0

# For Haptikfabriken (and building it)
sudo apt install libudev-dev libboost-all-dev libusb-1.0-0-dev
sudo apt install qt5-qmake qt5-default gcc-multilib



