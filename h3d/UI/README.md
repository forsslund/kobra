# UI

UI is a cross-platform, device independent easily extendable API that can be
used to add haptics to an application. For information on how you are allowed
to use UI see the LICENSE file in the same folder as this file.

This file contains complete installation instructions for Linux, Mac and Windows.


## External Libraries
UI depends on H3DAPI and HAPI. Both needs to be download and installed in
order to build UI and run the examples distributed with this package.


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
3. ` sudo apt-get install libh3dui1 `
4. ` sudo apt-get install h3dviewer `
5. Start H3DViewer and add /usr/lib/libh3dui.version.so to list of plugins.
6. Open an x3d file which uses nodes from UI.

### Building from source:
Since you have this file you have already obtained and unpacked the source for
UI. To build UI on Linux follow these steps.

1. Install version 2.8.7 or later of CMake. To do this on Ubuntu open a
   terminal and write:
   ```
    sudo apt-get install cmake
    ```
2. Install H3DAPI. See installation instructions for H3DAPI for this.

3. In the terminal change directory to the UI/build directory  
   Create a new directory named linux. Navigate to the new directory.  
   Write:
    ```
    cmake ../ 
    ```
   This will generate a make file. To build and install write:
    ```
    sudo make install
    ```

4. There are example files in this distribution that use UI.  
   There are two ways to use these:
   - Start H3DViewer and add the UI library as a plugin
   and then open an example file.
   - Open an example x3d file using H3DLoad from a terminal, e.g.
       ``` 
         H3DLoad path_to_example.x3d
       ```


## Installation on Windows
To install UI on Windows follow these steps:
1. Go to www.h3d.org.
2. Download latest release of H3DViewer
3. Install H3DViewer.
4. Open any x3d file with UI capabilities.

### Building from source:
1. Go to www.h3d.org.
2. Download latest release of H3DAPI.
3. Install full package.
4. Use CMake to generate project files for your compiler. Tested with visual
   studio 2010, 2012, 2013, 2015 and 2017.  
5. Open the solution file and build the INSTALL target.
6. Run example in one of two ways.
   - Use H3DViewer to add the dll to the plugins list either by moving the dll
     to the plugin folder for H3DViewer or adding it manually through the menu
     interface.
   - Use H3DLoad to run the file.


## Installation on MacOS X
Currently you need to build UI on MacOS X yourself. Since you have this
file you have already obtained and unpacked the source for UI.   

### Building from source:
To build UI on MacOS X follow these steps:
1. Install version 2.8.7 or later of CMake.
2. Install XCode. If gcc is an unrecognized command in the terminal after
   installing XCode there is a high probability that CMake will not be able to
   generate build files for gcc and/or XCode. Another version of XCode could be
   used or fix it in some other way.
3. Download and build H3DAPI and HAPI, they can be downloaded from www.h3d.org.
4. Generate build files for your build system using CMake. Either use the
   console version as in the guide for linux above or use the GUI version in
   which the first textbox should contain the location of the CMakeLists.txt
   file. This file is located in UI/build. The second box is the location
   of where the build files will be created. Use for example UI/build/OSX.
   Press configure, choose which system to generate build files for wait and
   press configure again until the generate button can be used. Press generate
   and then the build files will be in the chosen folder. If "Unix Makefile"
   are chosen then proceed from step 3 in the linux guide above.
5. To test the UI navigation change folder to UI/x3d in the terminal and write:
   ``` 
   H3DLoad buttons.x3d 
   ```
