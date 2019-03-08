# H3DUtil
H3DUtil is a cross-platform utility library used by HAPI and H3DAPI.
For information on how you are allowed to use H3DUtil see the LICENSE
file in the same folder as this file.

This file contains complete installation instructions for Linux, Mac and Windows.


## External Libraries
H3DUtil depends on other libraries. Some of them are required for H3DUtil to
compile others are optional. The build system used by H3DUtil will detect which
libraries are installed on the system and only enable the features that the
system can support. For Windows most libraries are included. Those missing must
be downloaded, compiled and installed on the system used if that feature is
desired. The libraries used by H3DUtil are:
 
**Required libraries :** Without these H3DUtil will not function.
  - pthread. Included with Windows distribution, most probably installed on
    other systems.
 
**Optional libraries :** Without these some features of H3DUtil will be disabled
and some of the distributed examples will not work.
  - FreeImage. Used to read images. http://freeimage.sourceforge.net
  - zlib. Required for parsing zipped files. http://www.zlib.net/
  - Teem. Required for reading the Nrrd file format. http://teem.sourceforge.net/
  - DICOM toolkit. Required for reading dicom files. http://dicom.offis.de/dcmtk
  
  Note that all haptics libraries does not exist for all operating systems.


## Installation on Linux
Using debian packages:
1. Modify sources.list ( or whatever file is used to specify urls to debian
   packages ) by adding ftp://www.h3dapi.org/pub/releases/linux/debian/
   to it.  
   On Ubuntu 18.04 simply use:  
     ``` 
     sudo gedit /etc/apt/sources.list
     ```
   Then add
     ``` 
     deb ftp://www.h3dapi.org/pub/releases/linux/debian bionic sensegraphics
     deb-src ftp://www.h3dapi.org/pub/releases/linux/debian bionic sensegraphics
     ``` 
   If you use a different linux version simply switch "bionic" for the name
   of your distribution.
     
2. ` sudo apt-get update `
3. ` sudo apt-get install libh3dutil1 `

### Building from source:
Since you have this file you have already obtained and unpacked the source for
H3DUtil. To build H3DUtil on Linux follow these steps.
1. Install version 2.8.7 or later of CMake. To do this on Ubuntu open a
   terminal and write:
   ```
   sudo apt-get install cmake
   ```

2. H3DUtil depends on other libraries. The build system used by H3DUtil will 
   detect which libraries are installed on the system and only enable the
   features that the system can support. If more features are desired the
   corresponding library has to be installed. Note that some libraries are
   required for H3DUtil to compile. See the list of external libraries in the
   beginning of this README.
   
3. In the terminal change folder to the H3DUtil/build folder. Write:
   ```
   cmake .
   ```
   This will generate a make file. To use the makefile write:
   ```
   make
   ```
   H3DUtil will be built. When the make finished write:
   ```
   sudo make install
   ```
   H3DUtil libraries are now installed on your system. But there is no
   application installed that use H3DUtil libraries.


## Installation on Windows
To install H3DUtil on Windows follow these steps:
1. Go to www.h3d.org.
2. Download latest release of H3DAPI.
3. In the installer, on the page where you choose what to install,
   expand the H3DAPI and HAPI line and deselect and then select
   H3DUtil runtime. Then proceed with installation.

### Building from source:
1. Go to www.h3d.org.
2. Download latest release of H3DAPI.
3. In the installer, on the page where you choose what to install,
   expand the H3DAPI and HAPI line and deselect and then select
   H3DUtil (all features of H3DUtil). Then proceed with installation.
4. Use CMake to generate project files for your compiler. Tested with visual
   studio 2010, 2012, 2013, 2015 and 2017.  
5. Open the solution file and build the INSTALL target.


## Installation on MacOS X
Currently you need to build H3DUtil on MacOS X yourself. Since you have this
file you have already obtained and unpacked the source for H3DUtil. To build
H3DUtil on MacOS X follow the steps below.

### Building from source:
1. Install version 2.8.7 or later of CMake.
2. Install XCode. If gcc is an unrecognized command in the terminal after
   installing XCode there is a high probability that CMake will not be able to
   generate build files for gcc and/or XCode. Another version of XCode could be
   used or fix it in some other way.
3. H3DUtil depends on other libraries. The build system used by H3DUtil will 
   detect which libraries are installed on the system and only enable the
   features that the system can support. If more features are desired the
   corresponding library has to be installed. Note that some libraries are
   required for H3DUtil to compile. See the list of external libraries in the
   beginning of this README.
4. Generate build files for your build system using CMake. Either use the
   console version as in the guide for linux above or use the GUI version in
   which the first textbox should contain the location of the CMakeLists.txt
   file. This file is located in H3DUtil/build. The second box is the location
   of where the build files will be created. Use for example H3DUtil/build/OSX.
   Press configure, choose which system to generate build files for wait and
   press configure again until the generate button can be used. Press generate
   and then the build files will be in the chosen folder. If "Unix Makefile"
   are chosen then proceed from step 3 in the linux guide above.
