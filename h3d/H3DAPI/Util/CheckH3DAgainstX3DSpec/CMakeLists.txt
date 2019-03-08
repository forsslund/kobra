cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "CheckH3DAgainstX3DSpec".  CMakeLists files in this project can
# refer to the root source directory of the project as ${CheckH3DAgainstX3DSpec_SOURCE_DIR} and
# to the root binary directory of the project as ${CheckH3DAgainstX3DSpec_BINARY_DIR}.
project( CheckH3DAgainstX3DSpec )

# Where to find Source files.
set( CheckH3DAgainstX3DSpec_SRCS "${CheckH3DAgainstX3DSpec_SOURCE_DIR}/CheckH3DAgainstX3DSpec.cpp" )

# Add optional libs to this variable.
set( optional_libs )

# Include directories.
include_directories( . )

# Add required libs to this variable.
set( required_libs )

# Where to find modules, used when finding packages.
set( CMAKE_MODULE_PATH "${CheckH3DAgainstX3DSpec_SOURCE_DIR}/../../build/modules" )

include( H3DCommonFunctions )
set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

#Geospatial
option( INCLUDE_Geospatial
        "Add Geospatial nodes to list"
        OFF )
if( INCLUDE_Geospatial )
  if( TARGET Geospatial )
    set( Geospatial_SEARCH_PATH "${CMAKE_INSTALL_PREFIX}/bin" )
  else()
    find_file( Geospatial_DLL "Geospatial" )

    if( Geospatial_DLL )
      get_filename_component( Geospatial_SEARCH_PATH ${Geospatial_DLL} PATH )
      string( REGEX REPLACE "/([^/])*$" "/bin" Geospatial_SEARCH_PATH "${Geospatial_SEARCH_PATH}" )
    endif()
  endif()
endif()

#H3DPhysics
option( INCLUDE_H3DPhysics
        "Add H3DPhysics nodes to list"
        OFF )
if( INCLUDE_H3DPhysics )
  if( TARGET H3DPhysics )
    set( H3DPhysics_SEARCH_PATH "${CMAKE_INSTALL_PREFIX}/${default_bin_install}" )
  else()
    find_package( H3DPhysics )

    if( H3DPhysics_FOUND )
      get_filename_component( H3DPhysics_SEARCH_PATH ${H3DPhysics_LIBRARY} PATH )
      string( REGEX REPLACE "/([^/])*$" "/${default_bin_install}" H3DPhysics_SEARCH_PATH "${H3DPhysics_SEARCH_PATH}" )
    endif()
  endif()
endif()

#MedX3D
option( INCLUDE_MedX3D
        "Add MedX3D nodes to list"
        OFF )
if( INCLUDE_MedX3D )
  if( TARGET MedX3D )
    set( MedX3D_SEARCH_PATH "${CMAKE_INSTALL_PREFIX}/${default_bin_install}" )
  else()
    find_package( MedX3D )

    if( MedX3D_FOUND )
      get_filename_component( MedX3D_SEARCH_PATH ${MEDX3D_LIBRARY} PATH )
      string( REGEX REPLACE "/([^/])*$" "/${default_bin_install}" MedX3D_SEARCH_PATH "${MedX3D_SEARCH_PATH}" )
    endif()
  endif()
endif()

configure_file( ${CheckH3DAgainstX3DSpec_SOURCE_DIR}/CheckH3DAgainstX3DSpec.cmake ${CheckH3DAgainstX3DSpec_SOURCE_DIR}/CheckH3DAgainstX3DSpec.cpp )

# Create build files.
add_executable( CheckH3DAgainstX3DSpec  ${CheckH3DAgainstX3DSpec_SRCS} )
target_link_libraries( CheckH3DAgainstX3DSpec ${required_libs} ${optional_libs} )

# Debug version should have _d postfix.
set_target_properties( CheckH3DAgainstX3DSpec PROPERTIES DEBUG_POSTFIX "_d" )

# Variable used to set compile properties.
set( CheckH3DAgainstX3DSpec_COMPILE_FLAGS "" )

option( DONT_LIST_INITIALIZE_TO_INPUT_OUTPUT
        "If true and X3D specification lists a field access type as INITIALIZE_ONLY and H3D lists it as INPUT_OUTPUT then don't show this difference"
        OFF )
if( DONT_LIST_INITIALIZE_TO_INPUT_OUTPUT )
    set( CheckH3DAgainstX3DSpec_COMPILE_FLAGS "${CheckH3DAgainstX3DSpec_COMPILE_FLAGS} -DDONT_LIST_INITIALIZE_TO_INPUT_OUTPUT" )
endif()

# set the install directory to the H3D directory on Windows
if( WIN32 )
  if( Geospatial_DLL )
    set( CheckH3DAgainstX3DSpec_COMPILE_FLAGS "${CheckH3DAgainstX3DSpec_COMPILE_FLAGS} -DHAVE_Geospatial" )
    add_dependencies( CheckH3DAgainstX3DSpec Geospatial )
  endif()

  if( H3DPhysics_FOUND OR TARGET H3DPhysics )
    set( CheckH3DAgainstX3DSpec_COMPILE_FLAGS "${CheckH3DAgainstX3DSpec_COMPILE_FLAGS} -DHAVE_H3DPhysics" )
    add_dependencies( CheckH3DAgainstX3DSpec H3DPhysics )
  endif()

  if( MedX3D_FOUND OR TARGET MedX3D )
    set( CheckH3DAgainstX3DSpec_COMPILE_FLAGS "${CheckH3DAgainstX3DSpec_COMPILE_FLAGS} -DHAVE_MedX3D" )
    add_dependencies( CheckH3DAgainstX3DSpec MedX3D )
  endif()

  # Set compile and link properties for projects.
  if( MSVC80 )
    # Remove some warnings about deprecated functions. Do not treat wchar_t as built in type
    set( CheckH3DAgainstX3DSpec_COMPILE_FLAGS "${CheckH3DAgainstX3DSpec_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE /Zc:wchar_t-" )
  elseif( MSVC90 )
    # Remove some warnings about deprecated functions. Do not treat wchar_t as built in type. Compile on multiple processors/threads if possible.
    set( CheckH3DAgainstX3DSpec_COMPILE_FLAGS "${CheckH3DAgainstX3DSpec_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE /Zc:wchar_t- /MP" )
  endif()

  set_target_properties( CheckH3DAgainstX3DSpec PROPERTIES COMPILE_FLAGS "${CheckH3DAgainstX3DSpec_COMPILE_FLAGS}" )

endif()
