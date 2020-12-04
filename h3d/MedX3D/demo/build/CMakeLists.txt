cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( MedX3DDemo )

set( CMAKE_MODULE_PATH "${MedX3DDemo_SOURCE_DIR}/../../build/modules" )

set( MedX3DDemo_COMPILE_FLAGS "" )

include( H3DCommonFunctions )
enableCpp11( FAIL_MESSAGE "Using MedX3DDemo requires C++11 support. This compiler lacks such support." )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${MedX3DDemo_SOURCE_DIR}/../.. )

setupRPathForLib()

set( MedX3DDemo_SRCS "${MedX3DDemo_SOURCE_DIR}/../src/WxConsoleDialog.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/WxConsoleDialog.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemo.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemo.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoApp.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoApp.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoMainDialog.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoMainDialog.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoMainFrame.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoMainFrame.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoBoundaryEnhancementVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoBoundaryEnhancementVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoBlendedVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoBlendedVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoCartoonVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoCartoonVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoComposedVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoComposedVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoComposedVolumeStyleOptionsPart.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoComposedVolumeStyleOptionsPart.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoEdgeEnhancementVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoEdgeEnhancementVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoLoadRawImageDialog.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoLoadRawImageDialog.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoMIPVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoMIPVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoOpacityMapVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoOpacityMapVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoShadedVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoShadedVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoToneMappedVolumeStyleOptions.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoToneMappedVolumeStyleOptions.h"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoStyleDialog.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/MedX3DDemoStyleDialog.h"


                     "${MedX3DDemo_SOURCE_DIR}/../src/WxWidgetsWindow.cpp"
                     "${MedX3DDemo_SOURCE_DIR}/../src/WxWidgetsWindow.h" )

# Add the MedX3DDemo.rc resource file if Visual Studio
if( MSVC )
  set( MedX3DDemo_SRCS ${MedX3DDemo_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/MedX3DDemo.rc )
endif()

set( MedX3DDemo_MAJOR_VERSION "1" )
set( MedX3DDemo_MINOR_VERSION "4" )
set( MedX3DDemo_BUILD_VERSION "0" )

include_directories( ../include )

set( required_libs )

include( StripAndAddLibraryDirectories )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI MedX3D
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI MedX3D )
include_directories( ${tmp_h3d_include_dirs} )

set( WXWINDOWS_USE_GL 1 )
find_package( wxWidgets REQUIRED COMPONENTS base core gl adv )
if( wxWidgets_FOUND )
  if( WIN32 )
    include_directories( ${wxWidgets_INCLUDE_DIR} )
  else()
    include( ${wxWidgets_USE_FILE} )
  endif()
  set( required_libs ${required_libs} ${wxWidgets_LIBRARIES} )
endif()

add_executable( MedX3DDemo WIN32 MACOSX_BUNDLE ${MedX3DDemo_SRCS} )

# make sure that the MedX3DDemo.rc contains the correct svn-version
setupResourceFile( MedX3DDemo
                   VERSION_PREFIX MedX3DDemo
                   SVN_DIR_CANDIDATE ${MedX3DDemo_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${MedX3DDemo_SOURCE_DIR}/MedX3DDemo.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/MedX3DDemo.rc
                   UPDATERESOURCEFILE_EXE ${MedX3DDemo_SOURCE_DIR}/../../build/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "MedX3DDemo_SOURCE_DIR" ${MedX3DDemo_SOURCE_DIR} "MedX3DDemo_Output_Name" "$(TargetFileName)" )

target_link_libraries( MedX3DDemo ${required_libs} ${optional_libs} )

# make the name of debug libraries end in _d.
set_target_properties( MedX3DDemo PROPERTIES DEBUG_POSTFIX "_d" )

if( MSVC )
  # Create variables
  set( MedX3DDemo_LINK_FLAGS_DEBUG "" )
  set( MedX3DDemo_COMPILE_FLAGS "-DWXUSINGDLL -DUNICODE -D_UNICODE" )

  # Treat wchar_t as built in type for all visual studio versions.
  # This is default for every version above 7 (so far) but we still set it for all.
  set( MedX3DDemo_COMPILE_FLAGS "${MedX3DDemo_COMPILE_FLAGS} /Zc:wchar_t" )

  if( ${MSVC_VERSION} GREATER 1399 )
    # Remove compiler warnings about deprecation for visual studio versions 8 and above.
    set( MedX3DDemo_COMPILE_FLAGS "${MedX3DDemo_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
  endif()

  if( ${MSVC_VERSION} GREATER 1499 )
    # Build using several threads for visual studio versions 9 and above.
    set( MedX3DDemo_COMPILE_FLAGS "${MedX3DDemo_COMPILE_FLAGS} /MP" )
    # Remove link against default library for debug builds.
    set( MedX3DDemo_LINK_FLAGS_DEBUG "${MedX3DDemo_LINK_FLAGS_DEBUG} /NODEFAULTLIB:msvcrt" )
  endif()

  # link flags for MedX3DDemo project
  set_target_properties( MedX3DDemo PROPERTIES LINK_FLAGS_DEBUG "${MedX3DDemo_LINK_FLAGS_DEBUG} /INCREMENTAL:NO"
                                               LINK_FLAGS "/INCREMENTAL:NO" )
                                               
elseif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  addCommonH3DGNUCompileFlags( MedX3DDemo_COMPILE_FLAGS TRUE )
endif()

set_target_properties( MedX3DDemo PROPERTIES COMPILE_FLAGS "${MedX3DDemo_COMPILE_FLAGS}" )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

if( APPLE )
  install( TARGETS MedX3DDemo
           LIBRARY DESTINATION ${default_lib_install}
           RUNTIME DESTINATION ${default_bin_install}
           BUNDLE DESTINATION "/Applications" )
else()
  install( TARGETS MedX3DDemo
           LIBRARY DESTINATION ${default_lib_install}
           RUNTIME DESTINATION ${default_bin_install}
           COMPONENT MedX3D_cpack_examples_runtime )
endif()

