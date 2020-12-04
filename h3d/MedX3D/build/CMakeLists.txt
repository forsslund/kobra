cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( MedX3D )

set( CMAKE_MODULE_PATH "${MedX3D_SOURCE_DIR}/localModules" "${MedX3D_SOURCE_DIR}/modules" )

include( H3DCommonFunctions )
setCommonH3DCMakePolicies()
setupRPathForLib()
enableCpp11( FAIL_MESSAGE "Enabling C++11 support. This compiler lacks such support." )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${MedX3D_SOURCE_DIR}/.. GENERATE_H3D_PACKAGE_PROJECT H3D_PREFER_STATIC_LIBRARIES )
set( MedX3D_COMPILE_FLAGS "-DMEDX3D_EXPORTS" )

include( "${MedX3D_SOURCE_DIR}/MedX3DSourceFiles.txt" )

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include ${MedX3D_SOURCE_DIR}/../include )

set( optional_libs )
set( required_libs )

# Version of MedX3D
set( MEDX3D_MAJOR_VERSION 1 )
set( MEDX3D_MINOR_VERSION 5 )
set( MEDX3D_BUILD_VERSION 1 )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

# Find Glew library. Required to compile.
find_package( GLEW )
if( GLEW_FOUND )
  include_directories( ${GLEW_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${GLEW_LIBRARIES} )
endif()

# autogenerate MedX3D.h depending on the libraries available.
if( EXISTS ${MedX3D_SOURCE_DIR}/../include/H3D/MedX3D/MedX3D.h )
  file( REMOVE ${MedX3D_SOURCE_DIR}/../include/H3D/MedX3D/MedX3D.h )
endif()
configure_file( ${MedX3D_SOURCE_DIR}/../include/H3D/MedX3D/MedX3D.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/MedX3D/MedX3D.h )

if( NOT GENERATE_H3D_PACKAGE_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/MedX3D/MedX3D.h
           DESTINATION ${MedX3D_SOURCE_DIR}/../include/H3D/MedX3D )
endif()

if( MSVC )
  # Add the MedX3D.rc resource file if Visual Studio
  set( MedX3D_SRCS ${MedX3D_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/MedX3D.rc )
endif()

populateProjectIncludeDirectoriesCacheVar( VARIABLE_NAME MedX3D_INCLUDE_DIRS
                                           DEPRECATED_VARIABLE_NAMES MEDX3D_INCLUDE_DIR )

add_library( MedX3D SHARED ${MedX3D_SRCS} ${MedX3D_HEADERS} )

target_link_libraries( MedX3D ${required_libs} ${optional_libs} )

handleCommonCacheVar( USE_thread_lock_debug FALSE )

# make the name of debug libraries end in _d.
set_target_properties( MedX3D PROPERTIES DEBUG_POSTFIX "_d" )

if( UNIX )
  set_target_properties( MedX3D PROPERTIES OUTPUT_NAME h3dmedx3d )
endif()

# make sure that the MedX3D.rc contains the correct svn-version
setupResourceFile( MedX3D
                   VERSION_PREFIX MEDX3D
                   SVN_DIR_CANDIDATE ${MedX3D_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${MedX3D_SOURCE_DIR}/MedX3D.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/MedX3D.rc
                   UPDATERESOURCEFILE_EXE ${MedX3D_SOURCE_DIR}/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "MedX3D_Output_Name" "$(TargetFileName)" )

if( MSVC )
  # Create variables
  set( MedX3D_LINK_FLAGS_DEBUG "" )

  include( H3DCommonFunctions )
  # change the name depending on compiler to be able to tell them apart
  # since they are not compatible with each other.
  setH3DMSVCOutputName( MedX3D "MedX3D" )
  # Add common H3D compile flags for microsoft visual studio
  addCommonH3DMSVCCompileFlags( MedX3D_COMPILE_FLAGS )

  # Seems like MedX3D needs to have incremental linking shut off for vc8. Since this is
  # not the most important feature it is shut off for all visual studio versions.
  set_target_properties( MedX3D PROPERTIES LINK_FLAGS_DEBUG "${MedX3D_LINK_FLAGS_DEBUG} /INCREMENTAL:NO" )
  
elseif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
  addCommonH3DGNUCompileFlags( MedX3D_COMPILE_FLAGS TRUE )
endif()

set( MEDX3D_FULL_VERSION ${MEDX3D_MAJOR_VERSION}.${MEDX3D_MINOR_VERSION}.${MEDX3D_BUILD_VERSION} )
set_target_properties( MedX3D PROPERTIES VERSION ${MEDX3D_FULL_VERSION} )

# set compile flags for MedX3D project
set_target_properties( MedX3D PROPERTIES COMPILE_FLAGS "${MedX3D_COMPILE_FLAGS}" )

if( UNIX )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${MedX3D_HEADERS}
           DESTINATION include/H3D/MedX3D )
endif()

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

# Install directories.
install( TARGETS MedX3D
         LIBRARY DESTINATION ${default_lib_install} COMPONENT MedX3D_cpack_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT MedX3D_cpack_runtime
         ARCHIVE DESTINATION ${default_lib_install} COMPONENT MedX3D_cpack_libraries )

handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES GENERATE_MedX3DDemo_PROJECT GENERATE_H3DBrowserPlugin_PROJECT
                                              OLD_VARIABLE_NAMES MedX3D_Demo_PROJECT MedX3D_Plugin_PROJECTS
                                              DOC_STRINGS "If set to YES MedX3DDemo project will be included in the build."
                                                          "If set to YES the MedX3D plugin projects will be included in this build." )

if( NOT DEFINED GENERATE_MedX3DDemo_PROJECT )
  set( GENERATE_MedX3DDemo_PROJECT YES CACHE BOOL "If set to YES MedX3DDemo project will be included in the build." )
endif()

if( GENERATE_MedX3DDemo_PROJECT )
  set( WXWINDOWS_USE_GL 1 )
  find_package( wxWidgets COMPONENTS base core )

  if( wxWidgets_FOUND )
    message( STATUS "Including MedX3D_Demo" )
    add_subdirectory( ${MedX3D_SOURCE_DIR}/../demo/build
                      ${CMAKE_CURRENT_BINARY_DIR}/MedX3DDemo )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} MedX3DDemo )
  else()
    message( WARNING "wxWidgets was not found. MedX3DDemo will not be generated." )
  endif()
endif()

if( EXISTS "${MedX3D_SOURCE_DIR}/../plugin" )
  if( NOT DEFINED GENERATE_H3DBrowserPlugin_PROJECT )
    set( GENERATE_H3DBrowserPlugin_PROJECT "NO" CACHE BOOL "If set to YES the MedX3D plugin projects will be included in this build." )
  endif()

  if( GENERATE_H3DBrowserPlugin_PROJECT )
    message( STATUS "Including MedX3D plugins" )
    add_subdirectory( ${MedX3D_SOURCE_DIR}/../plugin/build
                      ${CMAKE_CURRENT_BINARY_DIR}/MedX3DPlugin )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} MedX3DPlugin )
  endif()
endif()

if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "MedX3D" )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PARENT_SCOPE )
else()
  include( ${MedX3D_SOURCE_DIR}/MedX3DCPack.cmake )
endif()
