cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "H3DViewer".  CMakeLists files in this project can
# refer to the root source directory of the project as ${H3DViewer_SOURCE_DIR} and
# to the root binary directory of the project as ${H3DViewer_BINARY_DIR}.
project( H3DViewer )

# Where to find cmake modules used to find packages.
set( CMAKE_MODULE_PATH "${H3DViewer_SOURCE_DIR}/modules" "${H3DViewer_SOURCE_DIR}/../../build/modules" )

set( H3DViewer_COMPILE_FLAGS "" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${H3DViewer_SOURCE_DIR}/../../.. GENERATE_H3D_PACKAGE_PROJECT )

setupRPathForLib()

set( H3DViewer_MAJOR_VERSION "2" )
set( H3DViewer_MINOR_VERSION "4" )
set( H3DViewer_BUILD_VERSION "1" )
set( H3DVIEWER_APP_NAME "H3DViewer ${H3DViewer_MAJOR_VERSION}.${H3DViewer_MINOR_VERSION}.${H3DViewer_BUILD_VERSION}" )

# Add a cache variable USE_PYTHON_CONSOLE to have the choice of enabling the
# interactive Python console in H3DViewer. Default is NO.
handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES USE_python_console GENERATE_H3DViewer_PACKAGE_PROJECT
                                              OLD_VARIABLE_NAMES USE_PYTHON_CONSOLE GENERATE_H3DVIEWER_CPACK_PROJECT
                                              DOC_STRINGS "Decides if the interactive Python console is enabled in H3DViewer."
                                                          "Decides if a cpack project for H3DViewer should be generated." )
if( NOT DEFINED USE_python_console )
  set( USE_python_console "NO" CACHE BOOL "Decides if the interactive Python console is enabled in H3DViewer." )
endif()
set( USE_PYTHON_CONSOLE ${USE_python_console} ) # For H3DViewerConfig.h to be generated properly

# Add a cache variable GENERATE_H3DVIEWER_CPACK_PROJECT to have the choice of generating a project
# for packaging H3DViewer. Default is NO since most people will not use this.
if( NOT DEFINED GENERATE_H3DViewer_PACKAGE_PROJECT )
  set( GENERATE_H3DViewer_PACKAGE_PROJECT "NO" CACHE BOOL "Decides if a cpack project for H3DViewer should be generated." )
  mark_as_advanced( GENERATE_H3DViewer_PACKAGE_PROJECT )
endif()

# Include directories.
include_directories( ../src )

include( "${H3DViewer_SOURCE_DIR}/H3DViewerSourceFiles.txt" )
handleUnityBuild( PROJECT_NAME ${PROJECT_NAME}
                  SOURCE_FILES ${H3DViewer_SRCS}
                  OUTPUT_VARIABLE H3DViewer_SRCS )
handlePrecompiledHeaders( PROJECT_NAME ${PROJECT_NAME}
                          HEADERS_VARIABLE H3DViewer_HEADERS
                          SRCS_VARIABLE H3DViewer_SRCS
                          STDAFX_HEADER_LOCATION ${H3DViewer_SOURCE_DIR}/../src/
                          STDAFX_SOURCE_LOCATION ${H3DViewer_SOURCE_DIR}/../src/ )

# Add the H3DViewer.rc resource file if Visual Studio
if( MSVC )
  set( H3DViewer_SRCS ${H3DViewer_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/H3DViewer.rc
                                        ${H3DViewer_SOURCE_DIR}/H3DViewer.ico )
endif()

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include )

# Add required libs to this variable.
set( required_libs )

include( StripAndAddLibraryDirectories )


set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} ) 

# WxWidgets.
set( WXWINDOWS_USE_GL 1 )
# find wx library propgrid
# The propgrid code does not work properly on Ubuntu 11.10 with the installed CMake and wx (2.8).
# It returns true even though propgrid does not exist, it probably ignores USE_LIBS that does not exist.
# Since there is no wxWidgets_VERSION variable or something like that there is code to check the end of
# include path. I have no idea if this would work on all other UNIX systems as well. NOTE, report
# this to CMAKE unless it is reported already.
find_package( wxWidgets QUIET COMPONENTS propgrid )
if( wxWidgets_FOUND )
  if( ${CMAKE_SYSTEM_NAME} STREQUAL "Linux" )
    set( wxwidgets_version_above_28 FALSE )
    foreach( TMP_INCLUDE_DIR ${wxWidgets_INCLUDE_DIRS} )
      string( REGEX MATCH "wx[-][0-9][.][0-9]+$" wx_version_match ${TMP_INCLUDE_DIR} )
      if( NOT ${wx_version_match} STREQUAL "" )
        string( REGEX MATCH "[0-9][.][0-9]+$" wx_version_match_version ${wx_version_match} )
        if( ${wx_version_match_version} VERSION_GREATER "2.8" )
          set( wxwidgets_version_above_28 TRUE )
        endif()
      endif()
    endforeach()
    if( NOT wxwidgets_version_above_28 )
      set( wxWidgets_FOUND 0 )
    endif()
  endif()
endif()

set( wx_components )
if( wxWidgets_FOUND )
  set( HAVE_WXPROPGRID 1 )
  set( wx_components base core gl adv richtext html propgrid qa )  
else()
  message( STATUS "wxWidgets propgrid not found. Needed for better tree view and program setting choice in H3DViewer. Part of wxWidgets 2.9." )
  set( wx_components base core gl adv richtext html qa )
endif()

# reset the wxWidgets_FOUND variable before running FIND_PACKAGE again.
set( wxWidgets_FOUND 0 )

find_package( wxWidgets REQUIRED COMPONENTS ${wx_components} )
if( wxWidgets_FOUND )
  if( WIN32 )
    include_directories( ${wxWidgets_INCLUDE_DIR} ) 
  else()
    include( ${wxWidgets_USE_FILE} )
  endif()
  set( required_libs ${required_libs} ${wxWidgets_LIBRARIES} )
endif()

find_package( DirectX )
if( DirectX_FOUND )
  set( HAVE_DX9 1 )
  include_directories( ${DirectX_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${DirectX_LIBRARIES} )
endif()

# Create build files.
if( GENERATE_H3DViewer_PACKAGE_PROJECT )
  # if generating CPACK project using BUNDLE we should not create the bundle here instead
  # it is built with CPack 
  add_executable( H3DViewer WIN32 ${H3DViewer_SRCS} ${H3DViewer_HEADERS} )
else()
  add_executable( H3DViewer WIN32 MACOSX_BUNDLE ${H3DViewer_SRCS} ${H3DViewer_HEADERS} ${H3DViewer_SOURCE_DIR}/H3DViewer.icns )
endif()

# make sure that the H3DViewer.rc contains the correct svn-version
setupResourceFile( H3DViewer
                   VERSION_PREFIX H3DViewer
                   SVN_DIR_CANDIDATE ${H3DViewer_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${H3DViewer_SOURCE_DIR}/H3DViewer.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/H3DViewer.rc
                   UPDATERESOURCEFILE_EXE ${H3DViewer_SOURCE_DIR}/../../build/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "H3DViewer_SOURCE_DIR" ${H3DViewer_SOURCE_DIR} "H3DViewer_Output_Name" "$(TargetFileName)" )


if( EXISTS ${H3DViewer_SOURCE_DIR}/../src/H3DViewerConfig.h )
  file( REMOVE ${H3DViewer_SOURCE_DIR}/../src/H3DViewerConfig.h )
endif()
configure_file( ${H3DViewer_SOURCE_DIR}/../src/H3DViewerConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/H3DViewerConfig.h )

if( NOT ( GENERATE_H3D_PACKAGE_PROJECT OR GENERATE_H3DViewer_PACKAGE_PROJECT ) )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/H3DViewerConfig.h
           DESTINATION ${H3DViewer_SOURCE_DIR}/../src )
endif()

# Link against required_libs.
target_link_libraries( H3DViewer ${required_libs} )

handleCommonCacheVar( USE_thread_lock_debug FALSE )

# Set postfix for debug version of H3DViewer.
set_target_properties( H3DViewer PROPERTIES DEBUG_POSTFIX "_d" )

set_target_properties( H3DViewer
    PROPERTIES
    MACOSX_BUNDLE_BUNDLE_NAME H3DViewer
    MACOSX_BUNDLE_ICON_FILE H3DViewer.icns
)

if( MSVC )

  set( H3DViewer_COMPILE_FLAGS "-DWXUSINGDLL" )
  if( GENERATE_H3DViewer_PACKAGE_PROJECT )
    set( H3DViewer_COMPILE_FLAGS "${H3DViewer_COMPILE_FLAGS} -DH3DVIEWER_STANDALONE" )
  endif()

  # Treat wchar_t as built in type for all visual studio versions.
  # This is default for every version above 7 (so far) but we still set it for all.
  set( H3DViewer_COMPILE_FLAGS "${H3DViewer_COMPILE_FLAGS} /Zc:wchar_t" )

  if( MSVC80 )
    # This might be useful for visual studio 2005 users that often recompile the api.
    if( NOT DEFINED USE_VC8_MP_FLAG )
      set( USE_VC8_MP_FLAG "NO" CACHE BOOL "In visual studio 8 the MP flag exists but is not documented. Maybe it is unsafe to use. If you want to use it then set this flag to yes." )
    endif()

    if( USE_VC8_MP_FLAG )
      set( H3DViewer_COMPILE_FLAGS "${H3DViewer_COMPILE_FLAGS} /MP" )
    endif()
  endif()

  if( ${MSVC_VERSION} GREATER 1399 )
    # Remove compiler warnings about deprecation for visual studio versions 8 and above.
    set( H3DViewer_COMPILE_FLAGS "${H3DViewer_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
  endif()

  if( ${MSVC_VERSION} GREATER 1499 )
    # Build using several threads for visual studio versions 9 and above.
    set( H3DViewer_COMPILE_FLAGS "${H3DViewer_COMPILE_FLAGS} /MP" )
  endif()

  if( ${MSVC_VERSION} LESS 1600 )
    # Turn off optimization for visual studio versions 8 and below.
    # Old setting which I (Markus) have no idea why it was added.
    set( H3DViewer_COMPILE_FLAGS "${H3DViewer_COMPILE_FLAGS} /Od" )
  endif()

  # H3D API is now to big to link incrementally with vc. And remove link warning in debug build.
  set_target_properties( H3DViewer PROPERTIES LINK_FLAGS_DEBUG "/INCREMENTAL:NO /NODEFAULTLIB:msvcrt" )
  set_target_properties( H3DViewer PROPERTIES LINK_FLAGS "/INCREMENTAL:NO" )

  set_target_properties( H3DViewer PROPERTIES FOLDER "H3D" )
  
elseif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
  addCommonH3DGNUCompileFlags( H3DViewer_COMPILE_FLAGS )
endif()

set_target_properties( H3DViewer PROPERTIES COMPILE_FLAGS "${H3DViewer_COMPILE_FLAGS}" )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

# Install in these folders.
if( APPLE )
  handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3DViewer_OSX_BUNDLE_INSTALL_DESTINATION H3DViewer_OSX_BUNDLE_INSTALL_DESTINATION
                                                OLD_VARIABLE_NAMES OSX_BUNDLE_INSTALL_DESTINATION
                                                DOC_STRINGS "Directory into which to install the H3DViewer.app bundle." )
  if( NOT DEFINED H3DViewer_OSX_BUNDLE_INSTALL_DESTINATION )
    set( H3DViewer_OSX_BUNDLE_INSTALL_DESTINATION /Applications CACHE PATH "Directory into which to install the H3DViewer.app bundle." )
  endif()


  set_source_files_properties( ${H3DViewer_SOURCE_DIR}/H3DViewer.icns
                               PROPERTIES
                               MACOSX_PACKAGE_LOCATION Resources )

  install( TARGETS H3DViewer
           LIBRARY DESTINATION ${default_lib_install}
           RUNTIME DESTINATION ${default_bin_install}
           BUNDLE DESTINATION ${H3DViewer_OSX_BUNDLE_INSTALL_DESTINATION}
           COMPONENT H3DAPI_cpack_runtime )
else()
  set( H3DViewer_package_install_dir "" )
  if( WIN32 AND GENERATE_H3DViewer_PACKAGE_PROJECT )
    set( H3DViewer_package_install_dir "H3DViewer/" )
  endif()

  set( H3DVIEWER_INSTALL_COMPONENT_NAME "H3DViewer_cpack_runtime" )
  if( GENERATE_H3D_PACKAGE_PROJECT )
    set( H3DVIEWER_INSTALL_COMPONENT_NAME "H3DAPI_cpack_runtime" )
  endif()

  install( TARGETS H3DViewer
           LIBRARY DESTINATION ${default_lib_install}
           RUNTIME DESTINATION ${H3DViewer_package_install_dir}${default_bin_install} COMPONENT ${H3DVIEWER_INSTALL_COMPONENT_NAME} )
endif()

if( GENERATE_H3DViewer_PACKAGE_PROJECT )
  include( ${H3DViewer_SOURCE_DIR}/H3DViewerCPack.cmake )
endif()
