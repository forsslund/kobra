# Install all basic dependencies
sh dependencies.sh

# Extra h3d dependencie of right version
cd external_linux/spidermonkey_js_engine
sh install.sh
cd ../../

# Build haptikfabriken api
cd haptikfabrikenapi
sh dependencies.sh
sh install_usb_permissions.sh
qmake
make -j5
sudo make install
sudo ldconfig
cd ../

# Build h3d
cd h3d/build
cmake .
make -j5
sudo make install
cd ../../

# Build forssim



