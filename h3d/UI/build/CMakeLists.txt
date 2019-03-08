cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( UI )

# Set here already in case a special NSIS template needs to be used by cpack.
set( CMAKE_MODULE_PATH "${UI_SOURCE_DIR}/localModules" "${UI_SOURCE_DIR}/modules" )

include( H3DCommonFunctions )
setCommonH3DCMakePolicies()
setupRPathForLib()
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${UI_SOURCE_DIR}/.. GENERATE_H3D_PACKAGE_PROJECT )

include( "${UI_SOURCE_DIR}/UISourceFiles.txt" )
handleUnityBuild( PROJECT_NAME ${PROJECT_NAME}
                  SOURCE_FILES ${UI_SRCS}
                  OUTPUT_VARIABLE UI_SRCS )
handlePrecompiledHeaders( PROJECT_NAME ${PROJECT_NAME}
                          HEADERS_VARIABLE UI_HEADERS
                          SRCS_VARIABLE UI_SRCS
                          STDAFX_HEADER_LOCATION ${UI_SOURCE_DIR}/../include/H3D/UI/
                          STDAFX_SOURCE_LOCATION ${UI_SOURCE_DIR}/../src/ )

set( UI_MAJOR_VERSION 2 )
set( UI_MINOR_VERSION 4 )
set( UI_BUILD_VERSION 1 )


# Needed to convert from \ to / on windows. Not needed in the FindModules since
# call to macro takes care of that there.
include( StripAndAddLibraryDirectories )

# If cpack should be configured.

set( UI_FULL_VERSION
${UI_MAJOR_VERSION}.${UI_MINOR_VERSION}.${UI_BUILD_VERSION} )

# add the UI.rc resource file if Visual Studio
if( MSVC )
  set( UI_SRCS ${UI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/UI.rc )
endif()

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include ${UI_SOURCE_DIR}/../include )

# add all optional libraries to this variable
set( optional_libs )

# add all required libraries to this variable
set( required_libs )

if( WIN32 )
  set( required_libs ${required_libs} )
endif()

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

find_package( XercesC )

if( XercesC_FOUND )
  include_directories( ${XercesC_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${XercesC_LIBRARIES} )
endif()

find_package( FTGL )
if( FTGL_FOUND )
  include_directories( ${FTGL_INCLUDE_DIRS} )
  if( WIN32 )
    stripAndAddLibraryDirectories( ${FTGL_LIBRARIES} )
  else()
    set( optional_libs ${optional_libs} ${FTGL_LIBRARIES} )
  endif()
endif()

find_package( Freetype )
if( FREETYPE_FOUND )
  include_directories( ${FREETYPE_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${FREETYPE_LIBRARIES} )
endif()

populateProjectIncludeDirectoriesCacheVar( VARIABLE_NAME UI_INCLUDE_DIRS
                                           DEPRECATED_VARIABLE_NAMES UI_INCLUDE_DIR )

# UI shared library definition
add_library( UI SHARED ${UI_SRCS} ${UI_HEADERS} )

# make sure that the UI.rc contains the correct svn-version
setupResourceFile( UI
                   VERSION_PREFIX UI
                   SVN_DIR_CANDIDATE ${UI_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${UI_SOURCE_DIR}/UI.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/UI.rc
                   UPDATERESOURCEFILE_EXE ${UI_SOURCE_DIR}/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "UI_Output_Name" "$(TargetFileName)" )

# make sure symbols are exported.
set( UI_COMPILE_FLAGS "-DUI_EXPORTS" )

# add the libraries needed for linking
target_link_libraries( UI ${required_libs} ${optional_libs} )

handleCommonCacheVar( USE_thread_lock_debug FALSE )

# make the name of debug libraries end in _d.
set_target_properties( UI PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
set_target_properties( UI PROPERTIES VERSION ${UI_FULL_VERSION} )

set( UI_LINK_FLAGS_DEBUG "" )

if( UNIX )
  set_target_properties( UI PROPERTIES OUTPUT_NAME h3dui )
endif()

if( MSVC )
  include( H3DCommonFunctions )
  # change the name depending on compiler to be able to tell them apart
  # since they are not compatible with each other.
  setH3DMSVCOutputName( UI "UI" )
  # Add common H3D compile flags for microsoft visual studio
  addCommonH3DMSVCCompileFlags( UI_COMPILE_FLAGS )

  # Seems like UI needs to have incremental linking shut off for vc8. Since this is
  # not the most important feature it is shut off for all visual studio versions.
  set_target_properties( UI PROPERTIES LINK_FLAGS_DEBUG "${UI_LINK_FLAGS_DEBUG} /INCREMENTAL:NO" )
  
elseif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
  addCommonH3DGNUCompileFlags( UI_COMPILE_FLAGS )
endif()

set_target_properties( UI PROPERTIES COMPILE_FLAGS "${UI_COMPILE_FLAGS}" )

if( MSVC )
  set_target_properties( UI PROPERTIES FOLDER "H3D" )
endif()

# autogenerate UI.h depending on the libraries available.
if( EXISTS ${UI_SOURCE_DIR}/../include/H3D/UI/UI.h )
  file( REMOVE ${UI_SOURCE_DIR}/../include/H3D/UI/UI.h )
endif()
configure_file( ${UI_SOURCE_DIR}/../include/H3D/UI/UI.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/UI/UI.h )

if( NOT GENERATE_H3D_PACKAGE_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/UI/UI.h
           DESTINATION ${UI_SOURCE_DIR}/../include/H3D/UI )
endif()

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS UI
         LIBRARY DESTINATION ${default_lib_install} COMPONENT UI_cpack_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT UI_cpack_runtime
         ARCHIVE DESTINATION ${default_lib_install} COMPONENT UI_cpack_libraries )

if( NOT ( WIN32 OR GENERATE_H3D_PACKAGE_PROJECT ) )
  # install header files on non-windows system
  install( FILES ${UI_HEADERS}
           DESTINATION include/H3D/UI )
endif()

if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "UI" )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PARENT_SCOPE )
else()
  include( ${UI_SOURCE_DIR}/UICPack.cmake )
endif()

