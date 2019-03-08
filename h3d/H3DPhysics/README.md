# H3DPhysics
H3DPhysics is a cross-platform implementation of the RigidBodyPhysics
component of X3D for use with H3DAPI. H3DPhysics also contains nodes for 
defining softbody physics scenes. For information on how you are allowed
to use H3DPhysics see the LICENSE file in the same directory as this
file.

This file contains complete installation instructions for Linux, Mac and
Windows.


## External Libraries
H3DPhysics depends on other libraries. Some of them are required for
H3DPhysics to compile others are optional. The build system used by
H3DPhysics will detect which libraries are installed on the system and
only enable the features that the system can support. The missing libraries
must be downloaded, compiled and installed on the system used if that feature
is desired. The libraries used by H3DPhysics are:
 
**Required libraries :** Without these H3DPhysics will not function.
  - All libraries required by H3DAPI.
 
**Optional libraries :** Without these some features of H3DPhysics will be
disabled and some of the distributed examples will not work.
  - PhysX. Needed for running H3DPhysics with PhysX as the physics
    engine. http://developer.nvidia.com/object/physx_downloads.html
    There is separate support for PhysX 2 and 3.
  - Bullet. Needed for running H3DPhysics with bullet as the physics
    engine. http://www.bulletphysics.com/wordpress/
    Included with Windows distribution.
  - ODE (Open Dynamics Engine). Included with Windows distribution. Can be
    downloaded from http://www.ode.org/ At least version 0.10.0 is required.
    This physics engine only support rigidbody capabilities.
  - SOFA (Simulation Open Framework Architecture). This support is in alpha state
    as such you will have to find the library yourself.


## Installation on Linux
Using debian packages:  
Currently only ode support if H3DPhysics installed this way.
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
3. ` sudo apt-get install libh3dphysics1 `
4. ` sudo apt-get install h3dviewer `
5. Start H3DViewer and add /usr/lib/libh3dphysics.version.so to list of plugins.
6. Open an x3d file which uses nodes from H3DPhysics.

### Building from source:
Since you have this file you have already obtained and unpacked the source for
H3DPhysics. To build H3DPhysics on Linux follow these steps.

1. Install version 2.8.7 or later of CMake. To do this on Ubuntu open a
   terminal and write:
   ```
   sudo apt-get install cmake
   ```

2. Install H3DAPI. See installation instructions for H3DAPI for this.

3. H3DPhysics depends on other libraries. Install these, either through
   the apt-get feature (on Ubuntu) or by downloading the source code for the
   required version and build and install.

4. In the terminal change directory to the H3DPhysics/build directory.
   Create a new directory named linux.  
   Navigate to the new directory.  
   Write:
   ```
   cmake ../
   ```
   This will generate a make file. To build and install write:
   ```
   sudo make install
   ```

5. There are example files in this distribution that use H3DPhysics.
   There are two ways to use these:
   - Start H3DViewer and add the H3DPhysics library as a plugin
   and then open an example file.
   - Open an example x3d file using H3DLoad from a terminal, e.g.
     ```
     H3DLoad path_to_example.x3d
     ```


## Installation on Windows
To install H3DPhysics on Windows follow these steps:
1. Go to www.h3d.org.
2. Download latest release of H3DViewer
3. Install H3DViewer.
4. Open any x3d file with H3DPhysics capabilities.

### Building from source:
1. Go to www.h3d.org.
2. Download latest release of H3DAPI.
3. Install full package.
4. If PhysX support is desired download and install.
5. Use CMake to generate project files for your compiler. Tested with visual
   studio 2010, 2012, 2013, 2015 and 2017. If you use visual studio 2010 or above, you
   automatically get ode and bullet support out of the box. For previous visual
   studio versions, you need to download and build those libraries.  
6. Open the solution file and build the INSTALL target.
7. Run example in one of two ways.
   - Use H3DViewer to add the dll to the plugins list either by moving the dll
     to the plugin folder for H3DViewer or adding it manually through the menu
     interface.
   - Use H3DLoad to run the file.


## Installation on MacOS X
Currently you need to build H3DPhysics on MacOS X yourself. Since you
have this file you have already obtained and unpacked the source for
H3DPhysics. To build H3DPhysics on MacOS X follow the steps below.

### Building from source:
1. Install version 2.8.7 or later of CMake.
2. Install H3DAPI.
3. Install XCode. If gcc is an unrecognized command in the terminal after
   installing XCode there is a high probability that CMake will not be able to
   generate build files for gcc and/or XCode. Another version of XCode could be
   used or fix it in some other way.
4. Generate build files for your build system using CMake. Either use the
   console version as in the guide for linux above or use the GUI version in
   which the first textbox should contain the location of the CMakeLists.txt
   file. This file is located in H3DPhysics/build. The second box is the
   location of where the build files will be created. Use for example
   H3DPhysics/build/OSX.
   Press configure, choose which system to generate build files for wait and
   press configure again until the generate button can be used. Press generate
   and then the build files will be in the chosen directory. If
   "Unix Makefile" are chosen then proceed from step 4 in the linux guide above.
5. Run example in one of two ways.
   - Use H3DViewer to add the library as a plugin to H3DViewer manually
     through the menu interface.
   - Use H3DLoad to run the file. Use the ImportLibrary of X3D to edit the x3d
     example file in order load the library when the file is loaded.
