sudo apt install gcc-multilib
tar -xjvf sdk_826_linux_3.3.11.tar.bz2 
cd sdk_826_linux_3.3.11/

make modules
sudo make install
modprobe s826
make lib
sudo make lib_install

Lib (both static and dynamic) and headers are in
middleware/


