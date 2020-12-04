cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( H3DWithToolkitsWin )

if( "$ENV{H3D_ROOT}" STREQUAL "" )
  set( ENV{H3D_ROOT} ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI )
endif()

if( WIN32 AND( "$ENV{H3D_EXTERNAL_ROOT}" STREQUAL "" ) )
  set( ENV{H3D_EXTERNAL_ROOT} ${H3DWithToolkitsWin_SOURCE_DIR}/../External )
endif()

set( CMAKE_MODULE_PATH "${H3DWithToolkitsWin_SOURCE_DIR}/../H3DUtil/build/modules" )
include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${H3DWithToolkitsWin_SOURCE_DIR}/.. GENERATE_H3D_PACKAGE_PROJECT )

set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )

# Version of H3DWithToolkitsWin
set( H3DWithToolkitsWin_MAJOR_VERSION 2 )
set( H3DWithToolkitsWin_MINOR_VERSION 4 )
set( H3DWithToolkitsWin_BUILD_VERSION 1 )
if( NOT WIN32 )
  if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DUtil/build )
    message( STATUS "Including H3DUtil" )
    add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DUtil/build
                      ${CMAKE_CURRENT_BINARY_DIR}/H3DUtil )
  endif()

  if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../HAPI/build )
    message( STATUS "Including HAPI" )
    add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../HAPI/build
                      ${CMAKE_CURRENT_BINARY_DIR}/HAPI )
  endif()
endif()

if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/build )
  message( STATUS "Including H3DAPI" )
  add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/build
                    ${CMAKE_CURRENT_BINARY_DIR}/H3DAPI )
  include( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/build/H3DAPICPack.cmake )
elseif( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../HAPI/build )
  message( STATUS "Including HAPI" )
  add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../HAPI/build
                    ${CMAKE_CURRENT_BINARY_DIR}/HAPI )
elseif( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DUtil/build )
  message( STATUS "Including H3DUtil" )
  add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DUtil/build
                    ${CMAKE_CURRENT_BINARY_DIR}/H3DUtil )
endif()

if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../UI/build )
  message( STATUS "Including UI" )
  add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../UI/build
                    ${CMAKE_CURRENT_BINARY_DIR}/UI )
  include( ${H3DWithToolkitsWin_SOURCE_DIR}/../UI/build/UICPack.cmake )
endif()


if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../MedX3D/build )
  message( STATUS "Including MedX3D" )
  add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../MedX3D/build
                    ${CMAKE_CURRENT_BINARY_DIR}/MedX3D )
  include( ${H3DWithToolkitsWin_SOURCE_DIR}/../MedX3D/build/MedX3DCPack.cmake )
endif()

if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DPhysics/build )
  message( STATUS "Including H3DPhysics" )
  add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DPhysics/build
                     ${CMAKE_CURRENT_BINARY_DIR}/H3DPhysics )
  include( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DPhysics/build/H3DPhysicsCPack.cmake )
endif()

if( GENERATE_NodeRoutesToDotFile_BUILD )
  if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/Util/NodeRoutesToDotFile )
    # This project is added here since H3DPhysics/UI should be defined before this is done.
    message( STATUS "Including NodeRoutesToDotFile" )
    add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/Util/NodeRoutesToDotFile
                      ${CMAKE_CURRENT_BINARY_DIR}/NodeRoutesToDotFile )
  endif()
endif()


if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/Util/CreateH3DNodesFieldsList )
  handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES GENERATE_CreateH3DNodesFieldsList_PROJECT
                                                OLD_VARIABLE_NAMES CreateH3DNodesFieldsList_PROJECT
                                                DOC_STRINGS "If set to YES the CreateH3DNodesFieldsList project will be included in this build." )
  if( NOT DEFINED GENERATE_CreateH3DNodesFieldsList_PROJECT )
    set( GENERATE_CreateH3DNodesFieldsList_PROJECT NO CACHE BOOL "If set to YES the CreateH3DNodesFieldsList project will be included in this build." )
  endif()
  if( GENERATE_CreateH3DNodesFieldsList_PROJECT )

    # This project is added here since H3DPhysics/UI/MedX3D should be defined before this is done.
    message( STATUS "Including CreateH3DNodesFieldsList" )
    add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/Util/CreateH3DNodesFieldsList
                      ${CMAKE_CURRENT_BINARY_DIR}/CreateH3DNodesFieldsList )
  endif()
endif()

if( NOT DEFINED H3D_USE_DEPENDENCIES_ONLY )
  set( H3D_USE_DEPENDENCIES_ONLY TRUE CACHE INTERNAL "VARIABLE IS DEPRECATED. This variable is created for backwards compatiblity reasons, please replace all checks with checking for existance of targets in your CMake code." )
endif()

#pthread is required for using the H3DUtil library
find_package( PTHREAD REQUIRED )

if( WIN32 )
  # PTHREAD_INCLUDE_DIRS is added here since it is normally found by FindH3DUtil.
  # for this type of top cmakelists build we do not want to use FindH3DUtil, ever. If that
  # is done then any of the included CMakeLists.txt is doing something wrong.
  set( EXTERNAL_INCLUDE_DIR "${PTHREAD_INCLUDE_DIRS}" "${H3DWithToolkitsWin_SOURCE_DIR}/../External/include" CACHE INTERNAL "Set to internal so it can be changed by sub directories." )
endif()
  
if( NOT DEFINED H3D_EXTRA_TOOLKITS )
  set( H3D_EXTRA_TOOLKITS "" CACHE STRING "Add names of directories of extra toolkits that you want to include. Separate each name by ;. The extra toolkit CMakeLists.txt must be located in ${H3DWithToolkitsWin_SOURCE_DIR}/../<name of toolkit>/build" FORCE )
endif()

foreach( extra_toolkit ${H3D_EXTRA_TOOLKITS} )
  if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../${extra_toolkit}/build )
    message( STATUS "Including ${extra_toolkit}" )
    add_subdirectory( ${H3DWithToolkitsWin_SOURCE_DIR}/../${extra_toolkit}/build
                       ${CMAKE_CURRENT_BINARY_DIR}/${extra_toolkit} )
    if( EXISTS ${H3DWithToolkitsWin_SOURCE_DIR}/../${extra_toolkit}/build/${extra_toolkit}CPack.cmake )
      include( ${H3DWithToolkitsWin_SOURCE_DIR}/../${extra_toolkit}/build/${extra_toolkit}CPack.cmake )
    endif()
  endif()
endforeach()

# If cpack should be configured.
if( GENERATE_H3D_PACKAGE_PROJECT AND TARGET H3DAPI )
  install( FILES ${H3DWithToolkitsWin_SOURCE_DIR}/CMakeLists.txt
           DESTINATION build
           COMPONENT H3DUtil_cpack_sources )
  # Make sure that the nsis template for H3DAPI is used for packaging.
  set( CMAKE_MODULE_PATH "${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/build/localModules" )
  set( CPACK_PACKAGE_VERSION_MAJOR ${H3DWithToolkitsWin_MAJOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_MINOR ${H3DWithToolkitsWin_MINOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_PATCH ${H3DWithToolkitsWin_BUILD_VERSION} )
  set( CPACK_PACKAGE_NAME "H3DApi-Full" )
  set( CPACK_PACKAGE_FILE_NAME "H3DApi-Full-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}" )
  set( CPACK_PACKAGE_INSTALL_REGISTRY_KEY "${CPACK_PACKAGE_NAME} ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}" )

  # Set information properties about the project to install.
  set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "H3DAPI. A cross platform, haptics device independent, X3D based API for 3D graphics and haptics." )
  set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
  set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  set( CPACK_PACKAGE_DESCRIPTION_FILE "${H3DAPI_SOURCE_DIR}/../README.md" )
  set( CPACK_RESOURCE_FILE_LICENSE "${H3DAPI_SOURCE_DIR}/../LICENSE" )

  add_custom_command( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DummyFile
                      COMMAND echo )
  add_custom_target( INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                     DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/DummyFile )

  add_custom_command( TARGET INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                      POST_BUILD
                      COMMAND ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD} )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES H3DAPI ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES UI ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES MedX3D ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES H3DPhysics ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
  add_dependencies( INSTALL_RUNTIME_AND_LIBRARIES_ONLY ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )

  set( CPACK_NSIS_EXECUTABLES_DIRECTORY bin32 )
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( CPACK_NSIS_EXECUTABLES_DIRECTORY bin64 )
  endif()

  if( MSVC )
    # Additional, temporary command, to get 64 bit library for windows.
    getMSVCPostFix( h3d_msvc_postfix )

    set( 32_or_64 "64" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( 32_or_64 "32" )
    endif()
    set( NAMES_TO_INSTALL "H3DUtil" "HAPI" "H3DAPI" "H3DPhysics" "MedX3D" "UI" "OpenHapticsRenderer" "Chai3DRenderer" )
    foreach( NAME_TO_INSTALL ${NAMES_TO_INSTALL} )
      set( component_prefix "${NAME_TO_INSTALL}_cpack_" )
      if( ${component_prefix} STREQUAL "OpenHapticsRenderer_cpack_" OR ${component_prefix} STREQUAL "Chai3DRenderer_cpack_" )
        set( component_prefix "HAPI_cpack_" )
      endif()
      set( file_name_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../lib${32_or_64}/${NAME_TO_INSTALL}${h3d_msvc_postfix}.lib )
      if( EXISTS ${file_name_to_install} )
        install( FILES ${file_name_to_install}
                 DESTINATION lib${32_or_64}
                 COMPONENT ${component_prefix}libraries )
      endif()

      set( file_name_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../bin${32_or_64}/${NAME_TO_INSTALL}${h3d_msvc_postfix}.dll )
      if( EXISTS ${file_name_to_install} )
        install( FILES ${file_name_to_install}
                 DESTINATION bin${32_or_64}
                 COMPONENT ${component_prefix}runtime )
      endif()

      foreach( base_dir ${H3DWithToolkitsWin_SOURCE_DIR}/../${NAME_TO_INSTALL}/ ${CMAKE_CURRENT_BINARY_DIR}/${NAME_TO_INSTALL}/ ${CMAKE_CURRENT_BINARY_DIR}/H3DAPI/${NAME_TO_INSTALL}/ ${CMAKE_CURRENT_BINARY_DIR}/H3DAPI/HAPI/${NAME_TO_INSTALL}/ )
        set( file_name_to_install ${base_dir}include/H3D/${NAME_TO_INSTALL}_${32_or_64}.h )
        if( ${NAME_TO_INSTALL} STREQUAL "H3DAPI" )
          set( file_name_to_install ${base_dir}include/H3D/H3DApi_${32_or_64}.h )
        endif()
        if( EXISTS ${file_name_to_install} )
          install( FILES ${file_name_to_install}
                   DESTINATION ${NAME_TO_INSTALL}/include/H3D
                   COMPONENT ${component_prefix}headers )
        endif()

        set( file_name_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../${NAME_TO_INSTALL}/include/${NAME_TO_INSTALL}/${NAME_TO_INSTALL}_${32_or_64}.h )
        if( EXISTS ${file_name_to_install} )
          install( FILES ${file_name_to_install}
                   DESTINATION ${NAME_TO_INSTALL}/include/${NAME_TO_INSTALL}
                   COMPONENT ${component_prefix}headers )
        endif()

        set( file_name_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../${NAME_TO_INSTALL}/include/H3D/${NAME_TO_INSTALL}/${NAME_TO_INSTALL}_${32_or_64}.h )
        if( EXISTS ${file_name_to_install} )
          install( FILES ${file_name_to_install}
                   DESTINATION ${NAME_TO_INSTALL}/include/H3D/${NAME_TO_INSTALL}
                   COMPONENT ${component_prefix}headers )
        endif()
      endforeach()
    endforeach()

    set( file_name_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../bin${32_or_64}/H3DLoad.exe )
    if( EXISTS ${file_name_to_install} )
      install( FILES ${file_name_to_install}
               DESTINATION bin${32_or_64}
               COMPONENT H3DAPI_cpack_runtime )
    endif()

    set( file_name_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../bin${32_or_64}/H3DViewer.exe )
    if( EXISTS ${file_name_to_install} )
      install( FILES ${file_name_to_install}
               DESTINATION bin${32_or_64}
               COMPONENT H3DAPI_cpack_runtime )
    endif()

    set( file_name_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../bin${32_or_64}/MedX3DDemo.exe )
    if( EXISTS ${file_name_to_install} )
      install( FILES ${file_name_to_install}
               DESTINATION bin${32_or_64}
               COMPONENT MedX3D_cpack_examples_runtime )
    endif()

    set( file_names_to_install ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/Util/H3DViewerPackageExtraFiles/ACKNOWLEDGEMENTS
                              ${H3DWithToolkitsWin_SOURCE_DIR}/../H3DAPI/Util/H3DViewerPackageExtraFiles/NOTICE )
    foreach( NAME_TO_INSTALL ${file_names_to_install} )
      if( EXISTS ${NAME_TO_INSTALL} )
        install( FILES ${NAME_TO_INSTALL}
                 DESTINATION .
                 COMPONENT H3DUtil_cpack_runtime )
      endif()
    endforeach()


  endif()
  include( CPack )
endif()