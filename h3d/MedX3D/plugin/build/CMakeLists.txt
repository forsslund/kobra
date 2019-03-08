cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( H3DPlugin )

set( CMAKE_MODULE_PATH "${H3DPlugin_SOURCE_DIR}/modules" "${H3DPlugin_SOURCE_DIR}/../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${H3DPlugin_SOURCE_DIR}/../.. GENERATE_H3D_PACKAGE_PROJECT )

if( WIN32 )
  set( H3DPluginActiveX_COMPILE_FLAGS "-D_AFXDLL" )
  set( H3DPluginNetscape_COMPILE_FLAGS "-D_X86_" )
endif()

if( APPLE )
  set( H3DPluginNetscape_COMPILE_FLAGS "-DXP_MACOSX" )
else()
  if( UNIX )
    set( H3DPluginNetscape_COMPILE_FLAGS "-DMOZ_X11 -DXP_UNIX -DPRINT_ERR_FILE" )
  endif()
endif()
set( H3DPlugin_SRCS "${H3DPlugin_SOURCE_DIR}/../src/CommonInclude.h"
                    "${H3DPlugin_SOURCE_DIR}/../src/H3DPluginInstance.cpp"
                    "${H3DPlugin_SOURCE_DIR}/../src/H3DPluginInstance.h"
                    "${H3DPlugin_SOURCE_DIR}/../src/keys.cpp"
                    "${H3DPlugin_SOURCE_DIR}/../src/keys.h"
                    "${H3DPlugin_SOURCE_DIR}/../src/PluginWindow.cpp"
                    "${H3DPlugin_SOURCE_DIR}/../src/PluginWindow.h"
                    "${H3DPlugin_SOURCE_DIR}/../src/PluginWindowOSX.cpp"
                    "${H3DPlugin_SOURCE_DIR}/../src/PluginWindowOSX.h"
                    "${H3DPlugin_SOURCE_DIR}/../src/PluginWindowLinux.cpp"
                    "${H3DPlugin_SOURCE_DIR}/../src/PluginWindowLinux.h" )

set( H3DPluginNetscape_SRCS "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscape.cpp"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscape.h"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscapeWin.cpp"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscapeWin.h"
          "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscapeOSX.cpp"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscapeOSX.h"
          "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscapeLinux.cpp"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscapeLinux.h"
          "${H3DPlugin_SOURCE_DIR}/../src/netscape/NPAPIFunctions.cpp"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/Resource.h"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/stdafx.cpp"
                            "${H3DPlugin_SOURCE_DIR}/../src/netscape/stdafx.h" )


set( NPAPI_SRCS "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/jni.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/jni_md.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/jri.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/jritypes.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/jri_md.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/npapi.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/npn_gate.cpp"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/npplat.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/npp_gate.cpp"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/npupp.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/np_entry.cpp"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/pluginbase.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/prcpucfg.h"
                "${H3DPlugin_SOURCE_DIR}/../src/netscape/npapi/prtypes.h" )

set( H3DPluginActiveX_SRCS "${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveX.cpp"
                           "${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveX.h"
                           "${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveXCtrl.cpp"
                           "${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveXCtrl.h"
                           "${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveXCtrlMisc.cpp"
                           "${H3DPlugin_SOURCE_DIR}/../src/activex/Resource.h"
                           "${H3DPlugin_SOURCE_DIR}/../src/activex/stdafx.cpp"
                           "${H3DPlugin_SOURCE_DIR}/../src/activex/stdafx.h" )

set( optional_libs )

include_directories( ../src )

set( required_libs )

set( h3d_msvc_version "" )
if( MSVC )
  set( h3d_msvc_version 6 )
  set( temp_msvc_version 1299 )
  while( ${MSVC_VERSION} GREATER ${temp_msvc_version} )
    math( EXPR h3d_msvc_version "${h3d_msvc_version} + 1" )
    math( EXPR temp_msvc_version "${temp_msvc_version} + 100" )
  endwhile()

  if( MSVC70 OR MSVC71 )
    # This is stupid but for visual studio 7 the dll ends in 1.
    set( required_libs optimized mfcs${h3d_msvc_version}1 debug mfcs${h3d_msvc_version}1d )
  else()
    set( required_libs optimized mfcs${h3d_msvc_version}0 debug mfcs${h3d_msvc_version}0d )
  endif()
endif()

# Version of Plugins
set( H3DPluginNetscape_MAJOR_VERSION 1 )
set( H3DPluginNetscape_MINOR_VERSION 0 )
set( H3DPluginNetscape_BUILD_VERSION 0 )

set( H3DPluginActiveX_MAJOR_VERSION 1 )
set( H3DPluginActiveX_MINOR_VERSION 0 )
set( H3DPluginActiveX_BUILD_VERSION 0 )

# Needed to convert from \ to / on windows. Not needed in the FindModules since
# call to macro takes care of that there.
include( StripAndAddLibraryDirectories )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI MedX3D
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs_tmp
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
set( required_libs ${required_libs} ${required_libs_tmp} )
include_directories( ${tmp_h3d_include_dirs} )

if( MedX3D_FOUND )
  set( H3DPluginActiveX_COMPILE_FLAGS "${H3DPluginActiveX_COMPILE_FLAGS} -DHAVE_MEDX3D" )
  set( H3DPluginNetscape_COMPILE_FLAGS "${H3DPluginNetscape_COMPILE_FLAGS} -DHAVE_MEDX3D" )
endif()

# Optional extra library. Without this no X3D scene can be loaded. H3DAPI
# must be built with this feature in order to work.
find_package( XercesC REQUIRED )
if( XercesC_FOUND )
  include_directories( ${XercesC_INCLUDE_DIRS} )
  if( WIN32 )
    stripAndAddLibraryDirectories( ${XercesC_LIBRARIES} )
  else()
    set( required_libs ${required_libs} ${XercesC_LIBRARIES} )
  endif()
endif()

# Find OpenGL library. Required to compile.
find_package( OpenGL REQUIRED )
if( OPENGL_FOUND )
  set( HAVE_OPENGL 1 )
  include_directories( ${OPENGL_INCLUDE_DIR} )
  set( required_libs ${required_libs} ${OPENGL_LIBRARIES} )
endif()

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

# If cpack should be configured.
if( GENERATE_H3D_PACKAGE_PROJECT )
  # Set information properties about the project to install.
  set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "Plugin for viewing X3D files with H3D content in IE and Netscape based browsers such as FireFox and Opera." )
  set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
  set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  set( CPACK_PACKAGE_DESCRIPTION_FILE "${H3DPlugin_SOURCE_DIR}/../README.md" )
  set( CPACK_RESOURCE_FILE_LICENSE "${H3DPlugin_SOURCE_DIR}/../LICENSE" )

  # Project to install.
  set( CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};H3DPlugin;ALL;/" )

  # Installation directory for package.
  set( CPACK_PACKAGE_INSTALL_DIRECTORY "H3DPlugin" )

  # File patterns to ignore, common for all operating systems.
  set( H3DPlugin_CPACK_IGNORE_PATTERNS /\\\\.svn/
                                       \\\\.obj$
                                       \\\\.ncb$
                                       \\\\.log$
                                       \\\\.suo$
                                       \\\\.zip$
                                       \\\\.dir/
                                       \\\\.user$
                                       \\\\.cv$
                                       "/Debug(.)*/"
                                       "/debug(.)*/"
                                       /Release
                                       /release
                                       /linux
                                       /build/win32/
                                       /build/vc8
                                       /build/vc7
                                       /build/vc9
                                       /osx
                                       "~$" )

  set( CPACK_PACKAGE_VERSION_MAJOR ${H3DPluginNetscape_MAJOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_MINOR ${H3DPluginNetscape_MINOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_PATCH ${H3DPluginNetscape_BUILD_VERSION} )

  set( CPACK_SOURCE_IGNORE_FILES ${H3DPlugin_CPACK_IGNORE_PATTERNS} /${default_bin_install}/ )
  set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${H3DPlugin_SOURCE_DIR}/..;." )

  if( WIN32 AND NOT UNIX )
    set( CPACK_PACKAGE_START_MENU_NAME "H3D Plugin" )
    #Extra links to start menu if values are "ON"
    set( CPACK_ADD_PLUGINDOC_LINKS "ON" )
    set( CPACK_ADD_PLUGIN_DEMOS_LINKS "ON" )
    set( CPACK_REGISTER_PLUGINS "ON" )

    set( converted_h3d_root "" )
    if( EXISTS $ENV{H3D_ROOT} )
      file( TO_CMAKE_PATH $ENV{H3D_ROOT} converted_h3d_root )
    endif()

    if( H3DAPI_FOUND )
      get_filename_component( H3DAPI_FILENAME ${H3DAPI_LIBRARY} NAME_WE )
      get_filename_component( H3DAPI_PATH ${H3DAPI_LIBRARY} PATH )
      find_path( H3DBrowserPlugin_PACKAGE_h3d_binaries ${H3DAPI_FILENAME}.dll ${H3DAPI_PATH}/../${default_bin_install}
                                                     ${converted_h3d_root}/../${default_bin_install}
                                                     ${H3DPlugin_SOURCE_DIR}/../../../${default_bin_install} )
      mark_as_advanced( H3DBrowserPlugin_PACKAGE_h3d_binaries )
      if( NOT H3DBrowserPlugin_PACKAGE_h3d_binaries )
        message( FATAL_ERROR "H3D_BINARIES not set which means that the packaged release will most likely not work" )
      endif()
    endif()

    if( MedX3D_FOUND )
      get_filename_component( MEDX3D_FILENAME ${MEDX3D_LIBRARY} NAME_WE )
      get_filename_component( MEDX3D_PATH ${MEDX3D_LIBRARY} PATH )
      find_path( H3DBrowserPlugin_PACKAGE_MedX3D_binaries ${MEDX3D_FILENAME}.dll ${MEDX3D_PATH}/../${default_bin_install}
                                                        ${converted_h3d_root}/../MedX3D/${default_bin_install}
                                                        ${H3DPlugin_SOURCE_DIR}/../../${default_bin_install} )
      mark_as_advanced( H3DBrowserPlugin_PACKAGE_MedX3D_binaries )
      if( NOT H3DBrowserPlugin_PACKAGE_MedX3D_binaries )
        message( FATAL_ERROR "H3DBrowserPlugin_PACKAGE_MedX3D_binaries not set which means that the packaged release will most likely not work" )
      endif()
    endif()

    set( CPACK_MSVCR71_BINARIES "ON" )
    set( CPACK_MSVCR80_BINARIES "ON" )
    set( CPACK_MSVCR90_BINARIES "ON" )

    if( NOT MSVC70 AND NOT MSVC71 )
      set( H3DPlugin_CPACK_IGNORE_PATTERNS ${H3DPlugin_CPACK_IGNORE_PATTERNS}
                                           _vc7
                                           /msvcp71\\\\.dll
                                           /MSVCP71\\\\.DLL )
      set( CPACK_MSVCR71_BINARIES "OFF" )
    endif()
    if( NOT MSVC80 )
      set( H3DPlugin_CPACK_IGNORE_PATTERNS ${H3DPlugin_CPACK_IGNORE_PATTERNS}
                                           _vc8
                                           /Microsoft\\\\.VC80\\\\.CRT\\\\.manifest$
                                           /msvcp80\\\\.dll
                                           /msvcr80\\\\.dll )
      set( CPACK_MSVCR80_BINARIES "OFF" )
    endif()
    if( NOT MSVC90 )
      set( H3DPlugin_CPACK_IGNORE_PATTERNS ${H3DPlugin_CPACK_IGNORE_PATTERNS}
                                           _vc9
                                           /Microsoft\\\\.VC90\\\\.CRT\\\\.manifest$
                                           /msvcp90\\\\.dll
                                           /msvcr90\\\\.dll )
      set( CPACK_MSVCR90_BINARIES "OFF" )
    endif()

    set( converted_h3d_external_root "" )
    if( EXISTS $ENV{H3D_EXTERNAL_ROOT} )
      file( TO_CMAKE_PATH $ENV{H3D_EXTERNAL_ROOT} converted_h3d_external_root )
    endif()
    find_path( external_binaries pthreadVC2.dll
                                 ${converted_h3d_root}/../External/${default_bin_install}
                                 ${converted_h3d_external_root}/${default_bin_install}
                                 ${H3DPlugin_SOURCE_DIR}/../../../External/${default_bin_install} )
    mark_as_advanced( external_binaries )
    if( NOT external_binaries )
        message( FATAL_ERROR "external_binaries not set which means that the packaged release will most likely not work" )
      endif()

    set( H3D_PREFER_STATIC_LIBRARIES NO CACHE BOOL "Set this to true if H3DAPI that is used was compiled with H3D_PREFER_STATIC_LIBRARIES set to true." )
    mark_as_advanced( H3D_PREFER_STATIC_LIBRARIES )

    # Done outside if-case since this one is most likely needed because other
    # libraries depend on it.
    find_file( MSVCR71_DLL NAMES msvcr71.dll MSVCR71.DLL )
    mark_as_advanced( MSVCR71_DLL )

    if( MSVC70 OR MSVCR71 )
      find_file( MSVCP71_DLL NAMES msvcp71.dll MSVCP71.DLL )
      mark_as_advanced( MSVCP71_DLL )

      find_file( MFC_BINARY NAMES mfc71.dll MFC71.DLL )
      mark_as_advanced( MFC_BINARY )

      if( MSVCP71_DLL )
        get_filename_component( MSVCP71_DLL_NAME ${MSVCP71_DLL} NAME )
        configure_file( ${MSVCP71_DLL} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MSVCP71_DLL_NAME} COPYONLY )
      endif()
    elseif( MSVC80 )
      if( EXISTS $ENV{VS80COMNTOOLS} )
        file( TO_CMAKE_PATH $ENV{VS80COMNTOOLS} CONVERTED_VS80COMNTOOLS )
        find_file( MSVCR80_DLL NAMES msvcr80.dll MSVCR80.DLL
                              PATHS "${CONVERTED_VS80COMNTOOLS}/../../VC/redist/x86/Microsoft.VC80.CRT" )
        mark_as_advanced( MSVCR80_DLL )

        find_file( MSVCP80_DLL NAMES msvcp80.dll MSVCP80.DLL
                              PATHS "${CONVERTED_VS80COMNTOOLS}/../../VC/redist/x86/Microsoft.VC80.CRT" )
        mark_as_advanced( MSVCP80_DLL )

        find_file( CRT_MANIFEST NAMES Microsoft.VC80.CRT.manifest
                                PATHS "${CONVERTED_VS80COMNTOOLS}/../../VC/redist/x86/Microsoft.VC80.CRT" )
        mark_as_advanced( CRT_MANIFEST )

        find_file( MFC_BINARY NAMES mfc80.dll MFC80.DLL
                              PATHS "${CONVERTED_VS80COMNTOOLS}/../../VC/redist/x86/Microsoft.VC80.MFC" )
        mark_as_advanced( MFC_BINARY )

        find_file( MFC_MANIFEST NAMES Microsoft.VC80.MFC.manifest
                                PATHS "${CONVERTED_VS80COMNTOOLS}/../../VC/redist/x86/Microsoft.VC80.MFC" )
        mark_as_advanced( MFC_MANIFEST )
      else()
        set( MSVCR80_DLL "" CACHE FILEPATH  "Set to msvcr80 binary." )
        mark_as_advanced( MSVCR80_DLL )
        set( MSVCP80_DLL "" CACHE FILEPATH  "Set to msvcp80 binary." )
        mark_as_advanced( MSVCP80_DLL )
        set( CRT_MANIFEST "" CACHE FILEPATH  "Set to crt manifest." )
        mark_as_advanced( CRT_MANIFEST )
        set( MFC_BINARY "" CACHE FILEPATH  "Set to mfc binary." )
        mark_as_advanced( MFC_BINARY )
        set( MFC_MANIFEST "" CACHE FILEPATH  "Set to mfc manifest if needed." )
        mark_as_advanced( MFC_MANIFEST )
      endif()

      if( MSVCR80_DLL )
        get_filename_component( MSVCR80_DLL_NAME ${MSVCR80_DLL} NAME )
        configure_file( ${MSVCR80_DLL} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MSVCR80_DLL_NAME} COPYONLY )
      endif()

      if( MSVCP80_DLL )
        get_filename_component( MSVCP80_DLL_NAME ${MSVCP80_DLL} NAME )
        configure_file( ${MSVCP80_DLL} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MSVCP80_DLL_NAME} COPYONLY )
      endif()
    elseif( MSVC90 )
      if( EXISTS $ENV{VS90COMNTOOLS} )
        file( TO_CMAKE_PATH $ENV{VS90COMNTOOLS} CONVERTED_VS90COMNTOOLS )
        find_file( MSVCR90_DLL NAMES msvcr90.dll MSVCR90.DLL
                              PATHS "${CONVERTED_VS90COMNTOOLS}/../../VC/redist/x86/Microsoft.VC90.CRT" )
        mark_as_advanced( MSVCR90_DLL )

        find_file( MSVCP90_DLL NAMES msvcp90.dll MSVCP90.DLL
                              PATHS "${CONVERTED_VS90COMNTOOLS}/../../VC/redist/x86/Microsoft.VC90.CRT" )
        mark_as_advanced( MSVCP90_DLL )

        find_file( CRT_MANIFEST NAMES Microsoft.VC90.CRT.manifest
                                PATHS "${CONVERTED_VS90COMNTOOLS}/../../VC/redist/x86/Microsoft.VC90.CRT" )
        mark_as_advanced( CRT_MANIFEST )

        find_file( MFC_BINARY NAMES mfc90.dll MFC90.DLL
                              PATHS "${CONVERTED_VS90COMNTOOLS}/../../VC/redist/x86/Microsoft.VC90.MFC" )
        mark_as_advanced( MFC_BINARY )

        find_file( MFC_MANIFEST NAMES Microsoft.VC90.MFC.manifest
                                PATHS "${CONVERTED_VS90COMNTOOLS}/../../VC/redist/x86/Microsoft.VC90.MFC" )
        mark_as_advanced( MFC_MANIFEST )
      else()
        set( MSVCR90_DLL "" CACHE FILEPATH  "Set to msvcr90 binary." )
        mark_as_advanced( MSVCR90_DLL )
        set( MSVCP90_DLL "" CACHE FILEPATH  "Set to msvcp90 binary." )
        mark_as_advanced( MSVCP90_DLL )
        set( CRT_MANIFEST "" CACHE FILEPATH  "Set to crt manifest." )
        mark_as_advanced( CRT_MANIFEST )
        set( MFC_BINARY "" CACHE FILEPATH  "Set to mfc binary." )
        mark_as_advanced( MFC_BINARY )
        set( MFC_MANIFEST "" CACHE FILEPATH  "Set to mfc manifest if needed." )
        mark_as_advanced( MFC_MANIFEST )
      endif()

      if( MSVCR90_DLL )
        get_filename_component( MSVCR90_DLL_NAME ${MSVCR90_DLL} NAME )
        configure_file( ${MSVCR90_DLL} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MSVCR90_DLL_NAME} COPYONLY )
      endif()

      if( MSVCP90_DLL )
        get_filename_component( MSVCP90_DLL_NAME ${MSVCP90_DLL} NAME )
        configure_file( ${MSVCP90_DLL} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MSVCP90_DLL_NAME} COPYONLY )
      endif()
    endif()

    if( MSVCR71_DLL )
      get_filename_component( MSVCR71_DLL_NAME ${MSVCR71_DLL} NAME )
      configure_file( ${MSVCR71_DLL} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MSVCR71_DLL_NAME} COPYONLY )
    endif()

    if( CRT_MANIFEST )
      get_filename_component( CRT_MANIFEST_NAME ${CRT_MANIFEST} NAME )
      configure_file( ${CRT_MANIFEST} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${CRT_MANIFEST_NAME} COPYONLY )
    endif()

    if( MFC_BINARY )
      get_filename_component( MFC_BINARY_NAME ${MFC_BINARY} NAME )
      configure_file( ${MFC_BINARY} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MFC_BINARY_NAME} COPYONLY )
    endif()

    if( MFC_MANIFEST )
      get_filename_component( MFC_MANIFEST_NAME ${MFC_MANIFEST} NAME )
      configure_file( ${MFC_MANIFEST} ${H3DPlugin_SOURCE_DIR}/../${default_bin_install}/${MFC_MANIFEST_NAME} COPYONLY )
    endif()

    find_package( PythonLibs )
    if( PYTHON_INCLUDE_PATH AND PYTHON_LIBRARIES )
      get_filename_component( PYTHON_FILE_TO_FIND ${PYTHON_LIBRARY} NAME_WE )
      find_file( PYTHON_BINARY NAMES ${PYTHON_FILE_TO_FIND}.dll )
    else()
      set( PYTHON_BINARY "" CACHE FILEPATH  "Set to python binary if needed." )
    endif()
    mark_as_advanced( PYTHON_BINARY )

    if( PYTHON_BINARY )
      get_filename_component( PYTHON_BINARY_NAME ${PYTHON_BINARY} NAME )
      configure_file( ${PYTHON_BINARY} ${H3DPlugin_SOURCE_DIR}/../bin/${PYTHON_BINARY_NAME} COPYONLY )
    endif()

    set( H3DPlugin_CPACK_IGNORE_PATTERNS ${H3DPlugin_CPACK_IGNORE_PATTERNS}
                                         \\\\.\\\\./src/
                                         \\\\.\\\\./build/
                                         _d
                                         /H3DLoad\\\\.exe$
                                         /H3DViewer\\\\.exe$
                                         HapticAPI2
                                         /pthreadGC2\\\\.dll$
                                         /pthreadGCE2\\\\.dll$
                                         /pthreadVSE2\\\\.dll$
                                         /xerces-c_2_7\\\\.dll$
                                         /xerces-c_2_7D\\\\.dll$
                                         glut )
    if( H3D_PREFER_STATIC_LIBRARIES )
      set( H3DPlugin_CPACK_IGNORE_PATTERNS ${H3DPlugin_CPACK_IGNORE_PATTERNS}
                                           /xerces
                                           /glew
                                           FreeImage
                                           curl
                                           ogg
                                           vorbis )
    endif()
    set( CPACK_IGNORE_FILES ${H3DPlugin_CPACK_IGNORE_PATTERNS} )
    set( CPACK_INSTALLED_DIRECTORIES "${H3DBrowserPlugin_PACKAGE_h3d_binaries};${default_bin_install}"
                                     "${H3DBrowserPlugin_PACKAGE_MedX3D_binaries};${default_bin_install}"
                                     "${external_binaries};${default_bin_install}"
                                     "${H3DPlugin_SOURCE_DIR}/..;."
                                     "${H3DPlugin_SOURCE_DIR}/../${default_bin_install};${default_bin_install}" )
  endif()

  include( CPack )
endif()


# Create library
if( MSVC )
  if( NOT CMAKE_CL_64 )
    add_library( H3DPluginNetscape SHARED ${H3DPlugin_SRCS} ${H3DPluginNetscape_SRCS} ${NPAPI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/H3DPluginNetscape.rc )
    target_link_libraries( H3DPluginNetscape ${required_libs} ${optional_libs} )
  endif()
  set( CMAKE_MFC_FLAG 2 )
  add_library( H3DPluginActiveX SHARED ${H3DPlugin_SRCS} ${H3DPluginActiveX_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/H3DPluginActiveX.rc )
  target_link_libraries( H3DPluginActiveX ${required_libs} ${optional_libs} )
else()
  add_library( H3DPluginNetscape SHARED ${H3DPlugin_SRCS}  ${NPAPI_SRCS} ${H3DPluginNetscape_SRCS} )
  target_link_libraries( H3DPluginNetscape ${required_libs} ${optional_libs} )
endif()

if( TARGET MedX3D )
  # The reason for adding dependencies here is that CMake seem to have a bug which
  # creates two ALL_BUILD projects in a solution file in visual studio 2010.
  # This additional line changes the order of those ALL_BUILDS such that the
  # first one contain the correct dependencies which means that when you exit the
  # solution visual studio modifies the solution files (if you press yes) in such a way
  # that the correct dependencies are preserved.
  if( TARGET H3DPluginNetscape )
    add_dependencies( H3DPluginNetscape MedX3D )
  endif()
  if( TARGET H3DPluginActiveX )
    add_dependencies( H3DPluginActiveX MedX3D )
  endif()
endif()

# make the name of debug libraries end in _d.
if( TARGET H3DPluginNetscape )
  set_target_properties( H3DPluginNetscape PROPERTIES DEBUG_POSTFIX "d" )
  set( H3DPluginNetscape_outputName npH3D32 )
  set_target_properties( H3DPluginNetscape PROPERTIES OUTPUT_NAME ${H3DPluginNetscape_outputName} )
endif()

if( MSVC )
  set_target_properties( H3DPluginActiveX PROPERTIES DEBUG_POSTFIX "_d" )

  # Create variables
  set( H3DPluginActiveX_LINK_FLAGS "/DEF:\"${H3DPlugin_SOURCE_DIR}\\..\\src\\activex\\H3DPluginActiveX.def\"" )
  set( H3DPluginNetscape_LINK_FLAGS "/DEF:\"${H3DPlugin_SOURCE_DIR}\\..\\src\\netscape\\H3DPluginNetscape.def\"" )

  set( H3DPluginActiveX_outputName H3DPluginActiveX_vc${h3d_msvc_version} )

  set( H3D_BUILD_ENVIRONMENT MSVC${h3d_msvc_version}0 )
  if( ${MSVC_VERSION} GREATER 1599 )
    set( H3D_BUILD_ENVIRONMENT MSVC${h3d_msvc_version} )
  endif()

  # Add compile and link properties for projects.

  # Treat wchar_t as built in type for all visual studio versions.
  # This is default for every version above 7 (so far) but we still set it for all.
  set( H3DPluginActiveX_COMPILE_FLAGS "${H3DPluginActiveX_COMPILE_FLAGS} /Zc:wchar_t" )
  set( H3DPluginNetscape_COMPILE_FLAGS "${H3DPluginNetscape_COMPILE_FLAGS} /Zc:wchar_t" )

  if( ${MSVC_VERSION} GREATER 1399 )
    # Remove compiler warnings about deprecation for visual studio versions 8 and above.
    set( H3DPluginActiveX_COMPILE_FLAGS "${H3DPluginActiveX_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
    set( H3DPluginNetscape_COMPILE_FLAGS "${H3DPluginNetscape_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
  endif()

  if( ${MSVC_VERSION} GREATER 1499 )
    # Build using several threads for visual studio versions 9 and above.
    set( H3DPluginActiveX_COMPILE_FLAGS "${H3DPluginActiveX_COMPILE_FLAGS} /MP" )
    set( H3DPluginNetscape_COMPILE_FLAGS "${H3DPluginNetscape_COMPILE_FLAGS} /MP" )
  endif()

  set_target_properties( H3DPluginActiveX PROPERTIES OUTPUT_NAME ${H3DPluginActiveX_outputName} )

  configure_file( ${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveX.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/H3DPluginActiveX.rc )
  configure_file( ${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscape.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/H3DPluginNetscape.rc )

  add_custom_command( TARGET H3DPluginActiveX
                      PRE_BUILD
                      COMMAND midl /D "WIN32" /D "_WINDOWS" /D "_DEBUG" /D "_AFXDLL" /D "_CRT_SECURE_NO_DEPRECATE" /D "H3DPluginActiveX_EXPORTS" /nologo /char signed /env win32 /Oicf  /tlb "H3DPluginActiveX.tlb" /out "${CMAKE_CURRENT_BINARY_DIR}" /h "H3DPluginActiveXidl.h" /iid "H3DPluginActiveX_i.c" /proxy "H3DPluginActiveX_p.c" ${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveX.idl )

  if( TARGET H3DPluginActiveX )
    # make sure that the H3DPluginActiveXVersion.rc contains the correct svn-version
    setupResourceFile( H3DPluginActiveX
                       VERSION_PREFIX H3DPluginActiveX
                       SVN_DIR_CANDIDATE ${H3DPlugin_SOURCE_DIR}/../
                       RESOURCE_FILE_CMAKE_TEMPLATE ${H3DPlugin_SOURCE_DIR}/../src/activex/H3DPluginActiveXVersion.rc.cmake
                       RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/H3DPluginActiveXVersion.rc
                       UPDATERESOURCEFILE_EXE ${H3DPlugin_SOURCE_DIR}/UpdateResourceFile
                       UPDATERESOURCEFILE_EXE_EXTRA_ARGS "H3DPluginActiveX_outputName" "$(TargetFileName)" "H3D_BUILD_ENVIRONMENT" ${H3D_BUILD_ENVIRONMENT} )
  endif()

  if( TARGET H3DPluginNetscape )
    # make sure that the H3DPluginNetscapeVersion.rc contains the correct svn-version
    setupResourceFile( H3DPluginNetscape
                       VERSION_PREFIX H3DPluginNetscape
                       SVN_DIR_CANDIDATE ${H3DPlugin_SOURCE_DIR}/../
                       RESOURCE_FILE_CMAKE_TEMPLATE ${H3DPlugin_SOURCE_DIR}/../src/netscape/H3DPluginNetscapeVersion.rc.cmake
                       RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/H3DPluginNetscapeVersion.rc
                       UPDATERESOURCEFILE_EXE ${H3DPlugin_SOURCE_DIR}/UpdateResourceFile
                       UPDATERESOURCEFILE_EXE_EXTRA_ARGS "H3DPluginNetscape_outputName" "$(TargetFileName)" "H3D_BUILD_ENVIRONMENT" ${H3D_BUILD_ENVIRONMENT} )
  endif()

  # The added project will only register/unregister the release build since CMake
  # seem to not provide a way of specifying post build events for each configuration.
  set( CMAKE_MFC_FLAG 0 )
  add_custom_target( RegisterActiveXPlugin DEPENDS H3DPluginActiveX INSTALL )
  add_custom_command( TARGET RegisterActiveXPlugin
                      POST_BUILD
                      COMMAND regsvr32 ${CMAKE_INSTALL_PREFIX}/${default_bin_install}/${H3DPluginActiveX_outputName}.dll )

  add_custom_target( UnRegisterActiveXPlugin DEPENDS H3DPluginActiveX INSTALL )
  add_custom_command( TARGET UnRegisterActiveXPlugin
                      POST_BUILD
                      COMMAND regsvr32 -u ${CMAKE_INSTALL_PREFIX}/${default_bin_install}/${H3DPluginActiveX_outputName}.dll )

  set_target_properties( H3DPluginActiveX PROPERTIES COMPILE_FLAGS "${H3DPluginActiveX_COMPILE_FLAGS}" )
  set_target_properties( H3DPluginActiveX PROPERTIES LINK_FLAGS "${H3DPluginActiveX_LINK_FLAGS}" )
endif()

# set compile flags for projects
if( TARGET H3DPluginNetscape )
  set_target_properties( H3DPluginNetscape PROPERTIES COMPILE_FLAGS "${H3DPluginNetscape_COMPILE_FLAGS}" )
  set_target_properties( H3DPluginNetscape PROPERTIES LINK_FLAGS "${H3DPluginNetscape_LINK_FLAGS}" )
endif()

# Install directories.
if( MSVC )
  set( targets_to_install H3DPluginActiveX )
  if( TARGET H3DPluginNetscape )
    set( targets_to_install ${targets_to_install} H3DPluginActiveX )
  endif()
  install( TARGETS ${targets_to_install}
           LIBRARY DESTINATION ${default_lib_install}
           RUNTIME DESTINATION ${default_bin_install}
           ARCHIVE DESTINATION ${default_lib_install} )
else()
  install( TARGETS H3DPluginNetscape
           LIBRARY DESTINATION ${default_lib_install}/opera/plugins
           RUNTIME DESTINATION ${default_bin_install}
           ARCHIVE DESTINATION ${default_lib_install}/opera/plugins )
endif()

