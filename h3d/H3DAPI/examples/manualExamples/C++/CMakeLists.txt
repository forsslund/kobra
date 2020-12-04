cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "ManualExamples". CMakeLists files in this project can
# refer to the root source directory of the project as ${ManualExamples_SOURCE_DIR} and
# to the root binary directory of the project as ${ManualExamples_BINARY_DIR}.
project( ManualExamples )

# Where to find modules used to find libraries.
set( CMAKE_MODULE_PATH "${ManualExamples_SOURCE_DIR}/../../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${ManualExamples_SOURCE_DIR}/../../../.. )

setupRPathForLib()

# Optional libraries to link against are added to this variable.
set( optional_libs )

# Required libraries to link against are added to this variable.
set( required_libs )

include( StripAndAddLibraryDirectories )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

# Optional extra library. Makes is possible to used GLUT windows handling with H3D API.
find_package( GLUT REQUIRED )

if( GLUT_FOUND )
  include_directories( ${GLUT_INCLUDE_DIR} )
  if( WIN32 )
    # This is used in order for the compiler to find freeglut.lib.
    # If freeglut.lib is in any other place than glut32 on your system
    # either this CMakeLists.txt or FindGLUTWin needs to be changed
    # to allow for choice of extra library directory.
    stripAndAddLibraryDirectories( ${GLUT_LIBRARIES} )
  else()
    set( required_libs ${required_libs} ${GLUT_LIBRARIES} )
  endif()
endif()

# Build files created. Two executables. Sphere_X3D and Spheres_X3D.
add_executable( Sphere_X3D ${ManualExamples_SOURCE_DIR}/Sphere_X3D.cpp )
add_executable( Spheres_X3D ${ManualExamples_SOURCE_DIR}/Spheres_X3D.cpp )

# Link against libraries.
target_link_libraries( Sphere_X3D ${required_libs} ${optional_libs} )
target_link_libraries( Spheres_X3D ${required_libs} ${optional_libs} )

# Debug versions have _d postfix.
set_target_properties( Sphere_X3D PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( Spheres_X3D PROPERTIES DEBUG_POSTFIX "_d" )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

# Set compile and link properties for projects.
set( ManualExamples_COMPILE_FLAGS )

addCommonH3DCompileFlags( ManualExamples_COMPILE_FLAGS TRUE )

set_target_properties( Sphere_X3D PROPERTIES COMPILE_FLAGS "${ManualExamples_COMPILE_FLAGS}" )
set_target_properties( Spheres_X3D PROPERTIES COMPILE_FLAGS "${ManualExamples_COMPILE_FLAGS}" )


# Install to these directories.
install( TARGETS Sphere_X3D
         LIBRARY DESTINATION ${default_lib_install}
         RUNTIME DESTINATION ${default_bin_install}
         COMPONENT H3DAPI_cpack_examples_runtime )

install( TARGETS Spheres_X3D
         LIBRARY DESTINATION ${default_lib_install}
         RUNTIME DESTINATION ${default_bin_install}
         COMPONENT H3DAPI_cpack_examples_runtime )

