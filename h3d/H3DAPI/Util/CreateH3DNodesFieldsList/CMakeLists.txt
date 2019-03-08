cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "CreateH3DNodesFieldsList".  CMakeLists files in this project can
# refer to the root source directory of the project as ${CreateH3DNodesFieldsList_SOURCE_DIR} and
# to the root binary directory of the project as ${CreateH3DNodesFieldsList_BINARY_DIR}.
project( CreateH3DNodesFieldsList )

# Where to find Source files.
set( CreateH3DNodesFieldsList_SRCS "${CreateH3DNodesFieldsList_SOURCE_DIR}/CreateH3DNodesFieldsList.cpp" )

# Add optional libs to this variable.
set( optional_libs )

# Include directories.
include_directories( . )

# Add required libs to this variable.
set( required_libs )

# Where to find modules, used when finding packages.
set( CMAKE_MODULE_PATH "${CreateH3DNodesFieldsList_SOURCE_DIR}/../../build/modules" )

include( H3DCommonFunctions )
set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )
getMSVCPostFix( h3d_msvc_postfix ) # h3d_msvc_postfix variable is used by CreateH3DNodesFieldsList.cmake

set( CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL "${CreateH3DNodesFieldsList_SOURCE_DIR}/../nodesFieldsList" )

#UI
option( INCLUDE_UI
        "Add UI nodes to list"
        OFF )
if( INCLUDE_UI )
  if( TARGET UI )
    set( UI_SEARCH_PATH "${CMAKE_INSTALL_PREFIX}/${default_bin_install}" )
    if( EXISTS "${CreateH3DNodesFieldsList_SOURCE_DIR}/../../../UI/doc" )
      set( CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL ${CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL}
                                                                 "${CreateH3DNodesFieldsList_SOURCE_DIR}/../../../UI/doc/nodesFieldsList" )
    endif()
  else()
    find_package( UI )

    if( UI_FOUND )
      get_filename_component( UI_SEARCH_PATH ${UI_LIBRARY} PATH )
      string( REGEX REPLACE "/([^/])*$" "/${default_bin_install}" UI_SEARCH_PATH "${UI_SEARCH_PATH}" )
      if( EXISTS "${UI_INCLUDE_DIR}/../../../UI/doc" )
        set( CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL ${CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL}
                                                                  "${UI_INCLUDE_DIR}/../../../UI/doc/nodesFieldsList" )
      endif()
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
    if( EXISTS "${CreateH3DNodesFieldsList_SOURCE_DIR}/../../../H3DPhysics/doc" )
      set( CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL ${CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL}
                                                                 "${CreateH3DNodesFieldsList_SOURCE_DIR}/../../../H3DPhysics/doc/nodesFieldsList" )
    endif()
  else()
    find_package( H3DPhysics )

    if( H3DPhysics_FOUND )
      get_filename_component( H3DPhysics_SEARCH_PATH ${H3DPhysics_LIBRARY} PATH )
      string( REGEX REPLACE "/([^/])*$" "/${default_bin_install}" H3DPhysics_SEARCH_PATH "${H3DPhysics_SEARCH_PATH}" )
      if( EXISTS "${H3DPhysics_INCLUDE_DIR}/../../../H3DPhysics/doc" )
        set( CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL ${CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL}
                                                                  "${H3DPhysics_INCLUDE_DIR}/../../../H3DPhysics/doc/nodesFieldsList" )
      endif()
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
    if( EXISTS "${CreateH3DNodesFieldsList_SOURCE_DIR}/../../../MedX3D/doc" )
      set( CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL ${CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL}
                                                                 "${CreateH3DNodesFieldsList_SOURCE_DIR}/../../../MedX3D/doc/nodesFieldsList" )
    endif()
  else()
    find_package( MedX3D )

    if( MedX3D_FOUND )
      get_filename_component( MedX3D_SEARCH_PATH ${MEDX3D_LIBRARY} PATH )
      string( REGEX REPLACE "/([^/])*$" "/${default_bin_install}" MedX3D_SEARCH_PATH "${MedX3D_SEARCH_PATH}" )
      if( EXISTS "${MedX3D_INCLUDE_DIR}/../../../MedX3D/doc" )
        set( CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL ${CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL}
                                                                  "${MedX3D_INCLUDE_DIR}/../../../MedX3D/doc/nodesFieldsList" )
      endif()
    endif()
  endif()
endif()

# Add a cache variable CreateH3DNodesFieldsList_EXTRA_BINARIES used
# to give extra release libraries which have a database which
# dot-files should be generated for.
if( NOT DEFINED CreateH3DNodesFieldsList_EXTRA_BINARIES )
  set( CreateH3DNodesFieldsList_EXTRA_BINARIES "" CACHE STRING "Add paths to extra release binaries that contain additional H3D nodes for which to generate dot-files. Each binary path must be enclosed in \" and separated by semi colon( ; )" )
  mark_as_advanced( CreateH3DNodesFieldsList_EXTRA_BINARIES )
endif()

set( CreateH3DNodesFieldsList_EXTRA_BINARIES_CODE )
foreach( CreateH3DNodesFieldsList_EXTRA_BINARY ${CreateH3DNodesFieldsList_EXTRA_BINARIES} )
  file( TO_CMAKE_PATH ${CreateH3DNodesFieldsList_EXTRA_BINARY} CreateH3DNodesFieldsList_EXTRA_BINARY_CMAKE_PATH )
  set( CreateH3DNodesFieldsList_EXTRA_BINARIES_CODE "${CreateH3DNodesFieldsList_EXTRA_BINARIES_CODE}\n  extra_libraries.push_back( \"${CreateH3DNodesFieldsList_EXTRA_BINARY_CMAKE_PATH}\" );" )
endforeach()

if( NOT DEFINED CreateH3DNodesFieldsList_EXTRA_BINARIES_nodesFieldsList_OUTPUT_DIR )
  set( CreateH3DNodesFieldsList_EXTRA_BINARIES_nodesFieldsList_OUTPUT_DIR "${CreateH3DNodesFieldsList_SOURCE_DIR}/../" CACHE PATH "Path to the directory in which to output list of nodes and fields for extra binaries." )
  mark_as_advanced( CreateH3DNodesFieldsList_EXTRA_BINARIES_nodesFieldsList_OUTPUT_DIR )
endif()

set( CreateH3DNodesFieldsList_OUTPUT_FILE_CODE )
foreach( OUTPUT_FILE ${CreateH3DNodesFieldsList_OUTPUT_FILE_DEFAULT_INTERNAL} )
  file( TO_CMAKE_PATH ${OUTPUT_FILE} OUTPUT_FILE_CMAKE_PATH )
  set( CreateH3DNodesFieldsList_OUTPUT_FILE_CODE "${CreateH3DNodesFieldsList_OUTPUT_FILE_CODE}\n  nodes_fields_output_file.push_back( \"${OUTPUT_FILE_CMAKE_PATH}\" );" )
endforeach()

if( NOT DEFINED CreateH3DNodesFieldsList_OUTPUT_ABREVIATION_FILE )
  set( CreateH3DNodesFieldsList_OUTPUT_ABREVIATION_FILE "" CACHE STRING "Path to file in which to output node names and comma separated def name abreviation suggestions." )
  mark_as_advanced( CreateH3DNodesFieldsList_OUTPUT_ABREVIATION_FILE )
endif()

if( CreateH3DNodesFieldsList_OUTPUT_ABREVIATION_FILE )
  set( OUTPUT_ABREVIATION_OR_NAME_CLASH ON )
else()
  set( OUTPUT_ABREVIATION_OR_NAME_CLASH OFF )
endif()

if( NOT DEFINED CreateH3DNodesFieldsList_OUTPUT_NAME_CLASH_FILE )
  set( CreateH3DNodesFieldsList_OUTPUT_NAME_CLASH_FILE "" CACHE STRING "Path to file in which to output node names clashes between libraries." )
  mark_as_advanced( CreateH3DNodesFieldsList_OUTPUT_NAME_CLASH_FILE )
endif()

if( NOT OUTPUT_ABREVIATION_OR_NAME_CLASH )
  if( CreateH3DNodesFieldsList_OUTPUT_NAME_CLASH_FILE )
    set( OUTPUT_ABREVIATION_OR_NAME_CLASH ON )
  else()
    set( OUTPUT_ABREVIATION_OR_NAME_CLASH OFF )
  endif()
endif()

configure_file( ${CreateH3DNodesFieldsList_SOURCE_DIR}/CreateH3DNodesFieldsList.cmake ${CreateH3DNodesFieldsList_SOURCE_DIR}/CreateH3DNodesFieldsList.cpp )

# Create build filess.
add_executable( CreateH3DNodesFieldsList  ${CreateH3DNodesFieldsList_SRCS} )
target_link_libraries( CreateH3DNodesFieldsList ${required_libs} ${optional_libs} )

# Debug version should have _d postfix.
set_target_properties( CreateH3DNodesFieldsList PROPERTIES DEBUG_POSTFIX "_d" )

# Variable used to set compile properties.
  set( CreateH3DNodesFieldsList_COMPILE_FLAGS )

# set the install directory to the H3D directory on Windows
if( WIN32 )

  if( UI_FOUND OR TARGET UI )
    set( CreateH3DNodesFieldsList_COMPILE_FLAGS "${CreateH3DNodesFieldsList_COMPILE_FLAGS} -DHAVE_UI" )
    add_dependencies( CreateH3DNodesFieldsList UI )
  endif()

  if( H3DPhysics_FOUND OR TARGET H3DPhysics )
    set( CreateH3DNodesFieldsList_COMPILE_FLAGS "${CreateH3DNodesFieldsList_COMPILE_FLAGS} -DHAVE_H3DPhysics" )
    add_dependencies( CreateH3DNodesFieldsList H3DPhysics )
  endif()

  if( MedX3D_FOUND OR TARGET MedX3D )
    set( CreateH3DNodesFieldsList_COMPILE_FLAGS "${CreateH3DNodesFieldsList_COMPILE_FLAGS} -DHAVE_MedX3D" )
    add_dependencies( CreateH3DNodesFieldsList MedX3D )
  endif()
endif()

addCommonH3DCompileFlags( CreateH3DNodesFieldsList_COMPILE_FLAGS )

set_target_properties( CreateH3DNodesFieldsList PROPERTIES COMPILE_FLAGS "${CreateH3DNodesFieldsList_COMPILE_FLAGS}" )
