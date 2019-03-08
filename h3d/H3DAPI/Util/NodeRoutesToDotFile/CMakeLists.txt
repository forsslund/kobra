cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "NodeRoutesToDotFile".  CMakeLists files in this project can
# refer to the root source directory of the project as ${NodeRoutesToDotFile_SOURCE_DIR} and
# to the root binary directory of the project as ${NodeRoutesToDotFile_BINARY_DIR}.
project( NodeRoutesToDotFile )

handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES NodeRoutesToDotFile_extra_release_libraries NodeRoutesToDotFile_extra_debug_libraries
                                              OLD_VARIABLE_NAMES NodeRoutesToDotFile_EXTRA_RELEASE_LIBRARIES NodeRoutesToDotFile_EXTRA_DEBUG_LIBRARIES
                                              DOC_STRINGS "Add paths to extra release libraries that contain additional H3D nodes for which to generate dot-files. Each library path must be enclosed in \" and separated by semi colon( ; )"
                                                          "Add paths to extra debug libraries that contain additional H3D nodes for which to generate dot-files. Each library path must be enclosed in \" and separated by semi colon( ; )" )

# Add a cache variable NodeRoutesToDotFile_EXTRA_RELEASE_LIBRARIES used
# to give extra release libraries which have a database which
# dot-files should be generated for.
if( NOT DEFINED NodeRoutesToDotFile_extra_release_libraries )
  set( NodeRoutesToDotFile_extra_release_libraries "" CACHE STRING "Add paths to extra release libraries that contain additional H3D nodes for which to generate dot-files. Each library path must be enclosed in \" and separated by semi colon( ; )" )
  mark_as_advanced( NodeRoutesToDotFile_extra_release_libraries )
endif()

# Add a cache variable NodeRoutesToDotFile_EXTRA_DEBUG_LIBRARIES used
# to give extra debug libraries which have a database which
# dot-files should be generated for.
if( NOT DEFINED NodeRoutesToDotFile_extra_debug_libraries )
  set( NodeRoutesToDotFile_extra_debug_libraries "" CACHE STRING "Add paths to extra debug libraries that contain additional H3D nodes for which to generate dot-files. Each library path must be enclosed in \" and separated by semi colon( ; )" )
  mark_as_advanced( NodeRoutesToDotFile_extra_debug_libraries )
endif()

set( NodeRoutesToDotFile_EXTRA_RELEASE_LIBRARIES_CODE )
foreach( NodeRoutesToDotFile_EXTRA_LIBRARY ${NodeRoutesToDotFile_extra_release_libraries} )
  file( TO_CMAKE_PATH ${NodeRoutesToDotFile_EXTRA_LIBRARY} NodeRoutesToDotFile_EXTRA_LIBRARY_CMAKE_PATH )
  set( NodeRoutesToDotFile_EXTRA_RELEASE_LIBRARIES_CODE "${NodeRoutesToDotFile_EXTRA_RELEASE_LIBRARIES_CODE}\n  extra_libraries.push_back( ${NodeRoutesToDotFile_EXTRA_LIBRARY_CMAKE_PATH} );" )
endforeach()

set( NodeRoutesToDotFile_EXTRA_DEBUG_LIBRARIES_CODE )
foreach( NodeRoutesToDotFile_EXTRA_LIBRARY ${NodeRoutesToDotFile_extra_debug_libraries} )
  file( TO_CMAKE_PATH ${NodeRoutesToDotFile_EXTRA_LIBRARY} NodeRoutesToDotFile_EXTRA_LIBRARY_CMAKE_PATH )
  set( NodeRoutesToDotFile_EXTRA_DEBUG_LIBRARIES_CODE "${NodeRoutesToDotFile_EXTRA_DEBUG_LIBRARIES_CODE}\n extra_libraries.push_back( ${NodeRoutesToDotFile_EXTRA_LIBRARY_CMAKE_PATH} );" )
endforeach()


# Where to find Source files.
set( NodeRoutesToDotFile_SRCS "${NodeRoutesToDotFile_SOURCE_DIR}/NodeRoutesToDotFile.cpp" )

# Add optional libs to this variable.
set( optional_libs )

# Include directories.
include_directories( . )

# Add required libs to this variable.
set( required_libs )

# Where to find modules, used when finding packages.
set( CMAKE_MODULE_PATH "${NodeRoutesToDotFile_SOURCE_DIR}/../../build/modules" )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI UI H3DPhysics
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

# Optional extra library. Used for sound in H3D API.
find_package( OpenAL )
if( OPENAL_FOUND )
  include_directories( ${OPENAL_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${OPENAL_LIBRARIES} )
endif()

configure_file( ${NodeRoutesToDotFile_SOURCE_DIR}/NodeRoutesToDotFile.cmake ${NodeRoutesToDotFile_SOURCE_DIR}/NodeRoutesToDotFile.cpp )

# Create build filess.
add_executable( NodeRoutesToDotFile  ${NodeRoutesToDotFile_SRCS} )
target_link_libraries( NodeRoutesToDotFile ${required_libs} ${optional_libs} )

# Debug version should have _d postfix.
set_target_properties( NodeRoutesToDotFile PROPERTIES DEBUG_POSTFIX "_d" )

# Variable used to set compile properties.
set( NODEROUTESTODOTFILE_COMPILE_FLAGS "-DH3D_GENERATE_DOTROUTE_FILES" )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

if( WIN32 )

  if( UI_FOUND )
    set( NODEROUTESTODOTFILE_COMPILE_FLAGS "${NODEROUTESTODOTFILE_COMPILE_FLAGS} -DHAVE_UI" )
  endif()

  if( H3DPhysics_FOUND )
    set( NODEROUTESTODOTFILE_COMPILE_FLAGS "${NODEROUTESTODOTFILE_COMPILE_FLAGS} -DHAVE_H3DPhysics" )
  endif()

  # Treat wchar_t as built in type for all visual studio versions.
  # This is default for every version above 7 (so far) but we still set it for all.
  set( NODEROUTESTODOTFILE_COMPILE_FLAGS "${NODEROUTESTODOTFILE_COMPILE_FLAGS} /Zc:wchar_t" )

  if( ${MSVC_VERSION} GREATER 1399 )
    # Remove compiler warnings about deprecation for visual studio versions 8 and above.
    set( NODEROUTESTODOTFILE_COMPILE_FLAGS "${NODEROUTESTODOTFILE_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
  endif()

  if( ${MSVC_VERSION} GREATER 1499 )
    # Build using several threads for visual studio versions 9 and above.
    set( NODEROUTESTODOTFILE_COMPILE_FLAGS "${NODEROUTESTODOTFILE_COMPILE_FLAGS} /MP" )
  endif()

endif()

set_target_properties( NodeRoutesToDotFile PROPERTIES COMPILE_FLAGS "${NODEROUTESTODOTFILE_COMPILE_FLAGS}" )

install( TARGETS NodeRoutesToDotFile
         LIBRARY DESTINATION ${default_lib_install}
         RUNTIME DESTINATION ${default_bin_install} )