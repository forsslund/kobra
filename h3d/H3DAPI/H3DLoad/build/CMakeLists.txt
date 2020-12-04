cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "H3DLoad".  CMakeLists files in this project can
# refer to the root source directory of the project as ${H3DLoad_SOURCE_DIR} and
# to the root binary directory of the project as ${H3DLoad_BINARY_DIR}.
project( H3DLoad )

# Where to find modules, used when finding packages.
set( CMAKE_MODULE_PATH "${H3DLoad_SOURCE_DIR}/../../build/modules" )

set( H3DLoad_COMPILE_FLAGS "" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${H3DLoad_SOURCE_DIR}/../../.. )

setupRPathForLib()

set( H3DLoad_MAJOR_VERSION "2" )
set( H3DLoad_MINOR_VERSION "4" )
set( H3DLoad_BUILD_VERSION "1" )

# Where to find Source files.
set( H3DLoad_SRCS "${H3DLoad_SOURCE_DIR}/../src/H3DLoad.cpp" )

# Add optional libs to this variable.
set( optional_libs )

# Include directories.
include_directories( ../src )

# Add the H3DLoad.rc resource file if Visual Studio
if( MSVC )
  set( H3DLoad_SRCS ${H3DLoad_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/H3DLoad.rc )
endif()

# Add required libs to this variable.
set( required_libs )

include( StripAndAddLibraryDirectories )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} ) 

# Glut.
find_package( GLUT REQUIRED )

if( GLUT_FOUND )
  include_directories( ${GLUT_INCLUDE_DIR} )
  if( WIN32 )
    stripAndAddLibraryDirectories( ${GLUT_LIBRARIES} )
  else()
    set( required_libs ${required_libs} ${GLUT_LIBRARIES} )
  endif()
endif()
  
# OpenGL
find_package( OpenGL REQUIRED )

if( OPENGL_FOUND )
  include_directories( ${OPENGL_INCLUDE_DIR} ) 
  set( required_libs ${required_libs} ${OPENGL_LIBRARIES} )
endif()

# Create build filess.
add_executable( H3DLoad  ${H3DLoad_SRCS} )
target_link_libraries( H3DLoad ${required_libs} ${optional_libs} )

# make sure that the H3DLoad.rc contains the correct svn-version
setupResourceFile( H3DLoad
                   VERSION_PREFIX H3DLoad
                   SVN_DIR_CANDIDATE ${H3DLoad_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${H3DLoad_SOURCE_DIR}/H3DLoad.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/H3DLoad.rc
                   UPDATERESOURCEFILE_EXE ${H3DLoad_SOURCE_DIR}/../../build/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "H3DLoad_SOURCE_DIR" ${H3DLoad_SOURCE_DIR} "H3DLoad_Output_Name" "$(TargetFileName)" )

# Debug version should have _d postfix.
set_target_properties( H3DLoad PROPERTIES DEBUG_POSTFIX "_d" )

addCommonH3DCompileFlags( H3DLoad_COMPILE_FLAGS TRUE )

# set the install directory to the H3D directory on Windows
if( MSVC ) 
  set_target_properties( H3DLoad PROPERTIES LINK_FLAGS_DEBUG "/INCREMENTAL:NO /NODEFAULTLIB:msvcrt" )
  set_target_properties( H3DLoad PROPERTIES FOLDER "H3D" )
endif()

set_target_properties( H3DLoad PROPERTIES COMPILE_FLAGS "${H3DLoad_COMPILE_FLAGS}" )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

# Install to this destination.
install( TARGETS H3DLoad
         LIBRARY DESTINATION ${default_lib_install}
         RUNTIME DESTINATION ${default_bin_install}
         COMPONENT H3DAPI_cpack_runtime )

