Haptikfabriken API version 0.1
The API is currently under development and might change.

Install dependencies:

See the depenencies.sh script. It is a few standard ubuntu packages 
(including boost) and the Sensoray DAQ PCIe card driver+lib that 
is provided in the external/ folder (and the libhid is there too
but it is compiled in so you wont have to do anything). 
If you are lucky you can just run the ./dependencies.sh script and 
it will install the packages, compile the sensoray driver+library 
and install them. If the latter fails, please read their documentation
in the provided .tar.bz. 


Build & install:

qmake
make
sudo make install

This should place the libhaptikfabrikenapi.so in /usr/local/lib and the
only needed header file haptikfabrikenapi.h in /usr/local/include.

Good luck :)
Jonas Forsslund (jonas@forsslundsystems.se) 2019-03-07

