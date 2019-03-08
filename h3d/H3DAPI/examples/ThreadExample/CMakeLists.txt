cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )
if( APPLE AND POLICY CMP0042 )
  cmake_policy( SET CMP0042 NEW )
endif()

# The name of our project is "ThreadExample". CMakeLists files in this project can
# refer to the root source directory of the project as ${ThreadExample_SOURCE_DIR} and
# to the root binary directory of the project as ${ThreadExample_BINARY_DIR}.
project( ThreadExample )

# Where to find modules used to find libraries.
set( CMAKE_MODULE_PATH "${ThreadExample_SOURCE_DIR}/../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${ThreadExample_SOURCE_DIR}/../../.. )

setupRPathForLib()

# Optional libraries to link against are added to this variable.
set( optional_libs )

# Required libraries to link against are added to this variable.
set( required_libs )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

# Build files created.
add_library( ThreadExample SHARED ${ThreadExample_SOURCE_DIR}/ThreadExample.h
                                 ${ThreadExample_SOURCE_DIR}/ThreadExample.cpp )

# Link against libraries.
target_link_libraries( ThreadExample ${required_libs} ${optional_libs} )

# Debug versions have _d postfix.
set_target_properties( ThreadExample PROPERTIES DEBUG_POSTFIX "_d" )

if( UNIX )
  set_target_properties( ThreadExample PROPERTIES OUTPUT_NAME threadexample )
endif()

set( ThreadExample_COMPILE_FLAGS )

addCommonH3DCompileFlags( ThreadExample_COMPILE_FLAGS )

set_target_properties( ThreadExample PROPERTIES COMPILE_FLAGS "${ThreadExample_COMPILE_FLAGS}" )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

# Install to these directories.
install( TARGETS ThreadExample
         LIBRARY DESTINATION ${default_lib_install}
         RUNTIME DESTINATION ${default_bin_install}
         COMPONENT H3DAPI_cpack_examples_runtime )

