if( NOT TARGET H3DViewer )
  message( FATAL_ERROR "Include file H3DViewerCPack.cmake require the target H3DViewer to exist. Please add H3DAPI/H3DViewer/build/CMakeLists.txt as subdirectory first." )
endif()

# To allow other projects that use H3DVIEWER as a subproject to add extra include directories
# when packaging.
if( GENERATE_H3DViewer_PACKAGE_PROJECT )
  if( WIN32 )
    handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3D_EXTERNAL_ROOT
                                                  OLD_VARIABLE_NAMES H3DViewer_CPACK_EXTERNAL_ROOT
                                                  DOC_STRINGS "Set to the External directory used with H3DViewer, needed to pack properly. If not set FIND_modules will be used instead." )
    # Add a cache variable which indicates where the Externals directory used for packaging
    # H3DViewer is located. If not set then FIND modules will be used instead.
    if( NOT DEFINED H3D_EXTERNAL_ROOT )
      set( h3d_external_root_default "" )
      if( H3D_USE_DEPENDENCIES_ONLY )
        foreach( EXTERNAL_INCLUDE_DIR_TMP ${EXTERNAL_INCLUDE_DIR} )
          if( EXISTS ${EXTERNAL_INCLUDE_DIR_TMP}/../include/pthread )
            set( h3d_external_root_default "${EXTERNAL_INCLUDE_DIR_TMP}/.." )
          endif()
        endforeach()
      else()
        set( h3d_external_root_default "$ENV{H3D_EXTERNAL_ROOT}" )
      endif()
      set( H3D_EXTERNAL_ROOT "${h3d_external_root_default}" CACHE PATH "Set to the External directory used with H3DViewer, needed to pack properly. If not set FIND_modules will be used instead." )
      mark_as_advanced( H3D_EXTERNAL_ROOT )
    endif()
  endif()
  
  # Set information properties about the project to install.
  set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "H3DViewer. An X3D viewer with extensions for haptics." )
  set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
  set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  set( CPACK_RESOURCE_FILE_LICENSE "${H3DViewer_SOURCE_DIR}/../../LICENSE" )
  set( CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};H3DViewer;ALL;/" )
  
  # Installation directory for package.
  set( CPACK_PACKAGE_INSTALL_DIRECTORY "SenseGraphics" )
  
  set( CPACK_PACKAGE_VERSION_MAJOR ${H3DViewer_MAJOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_MINOR ${H3DViewer_MINOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_PATCH ${H3DViewer_BUILD_VERSION} )
  set( CPACK_PACKAGE_INSTALL_REGISTRY_KEY "H3DViewer ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}" )
  set( CPACK_NSIS_PACKAGE_NAME "H3DViewer ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}" )
  set( CPACK_NSIS_UNINSTALL_NAME "H3DViewer-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}" )
  
  if( APPLE )
    handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3DViewer_PACKAGE_include_libraries
                                                  OLD_VARIABLE_NAMES H3DVIEWER_CPACK_INCLUDE_LIBRARIES
                                                  DOC_STRINGS "Decides if all dependent shared libraries should be included in the bundle or not." )
    if( NOT DEFINED H3DViewer_PACKAGE_include_libraries )
      set( H3DViewer_PACKAGE_include_libraries "NO" CACHE BOOL "Decides if all dependent shared libraries should be included in the bundle or not." )
      mark_as_advanced( H3DViewer_PACKAGE_include_libraries )
    endif()

    set( CPACK_BUNDLE_NAME "H3DViewer" ) #- provides the bundle name (displayed in the finder underneath the bundle icon). 
    set( CPACK_BUNDLE_ICON "${H3DViewer_SOURCE_DIR}/H3DViewer.icns" ) # - provides the bundle icon (displayed in the /Applications folder, on the dock, etc). 
    set( CPACK_BUNDLE_PLIST "${H3DViewer_SOURCE_DIR}/info.plist" ) # - path to a file that will become the bundle plist. 
    set( CPACK_BUNDLE_STARTUP_COMMAND "${H3DViewer_SOURCE_DIR}/start.sh" ) #- path to a file that will be executed when the user opens the bundle. Could be a shell-script or a binary.

    if( H3DViewer_PACKAGE_include_libraries )
      #Include all shared libraries in bundle
      include( "${H3DViewer_SOURCE_DIR}/OSXCPackLibraries.txt" )

      install( FILES ${OSX_PLUGIN_LIBRARIES}
               DESTINATION Plugins )

      install( CODE "EXECUTE_PROCESS( COMMAND \"python\" ${H3DViewer_SOURCE_DIR}/osx_bundle.py   
                                      WORKING_DIRECTORY \${CMAKE_INSTALL_PREFIX} )" )
    endif()
  endif()
  
  if( WIN32 )
    set( CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL "ON" )
    set( CPACK_MONOLITHIC_INSTALL "TRUE" )
    set( CPACK_PACKAGE_START_MENU_NAME "H3DViewer ${H3DViewer_MAJOR_VERSION}.${H3DViewer_MINOR_VERSION}" )
                           
    set( external_bin_path "bin32" )
    set( CPACK_PACKAGE_NAME "H3DViewer" )
    # CPACK_NSIS_INSTALL_ROOT must be set properly because cmake does not set it correctly
    # for a 64 bit build.
    set( CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES32" )
    set( CPACK_NSIS_DISPLAY_NAME_POSTFIX "(x86)" )
    set( CPACK_H3D_64_BIT "FALSE" )
    set( CPACK_NSIS_EXECUTABLES_DIRECTORY bin32 )
    set( CPACK_H3DViewer_RegEntry "H3DViewer ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}(x86)" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # check if the system is 64 bit
      set( external_bin_path "bin64" )
      set( CPACK_NSIS_EXECUTABLES_DIRECTORY bin64 )
      set( CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64" )
      set( CPACK_H3D_64_BIT "TRUE" )
      set( CPACK_NSIS_DISPLAY_NAME_POSTFIX "" )
      set( CPACK_H3DViewer_RegEntry "H3DViewer ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}(x64)" )
    endif()
    set( CPACK_NSIS_DISPLAY_NAME "H3DViewer${CPACK_NSIS_DISPLAY_NAME_POSTFIX} ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}" )

    set( FEATURES_TO_INSTALL "bin" "H3DViewer/${external_bin_path}" )
    set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\\n" )
    set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "\\n" )

    include( InstallH3DAPIAndExternals )

    # Modify path since in the NSIS template.
    set( CPACK_NSIS_MODIFY_PATH "ON" )
  
    if( EXISTS ${H3D_EXTERNAL_ROOT} )
      set( external_binaries ${external_binaries}
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30u_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30u_xml_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_adv_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_core_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_gl_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_qa_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_html_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_propgrid_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_richtext_vc_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30u_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30u_xml_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_adv_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_core_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_gl_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_qa_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_html_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_propgrid_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_richtext_vc_x64_custom.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/ode_double.dll )
      install( FILES "${H3D_EXTERNAL_ROOT}/include/ACKNOWLEDGEMENTS"
               DESTINATION H3DViewer )
      install( DIRECTORY ${H3D_EXTERNAL_ROOT}/include/ExternalLicenses/
               DESTINATION H3DViewer/ExternalLicenses )
    endif()

    foreach( binary ${external_binaries} )
      if( EXISTS ${binary} )
        install( FILES ${binary}
                 DESTINATION H3DViewer/${default_bin_install} )
      endif()
    endforeach()
    
    getMSVCPostFix( h3d_msvc_version )

    if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "H3DViewer" )
      # these part are added separately so that these plug in can be automatically added to H3DViewer
      install( FILES ${H3DViewer_BINARY_DIR}/../../H3DPhysics/Release/H3DPhysics${h3d_msvc_version}.dll
                     ${H3DViewer_BINARY_DIR}/../../MedX3D/Release/MedX3D${h3d_msvc_version}.dll
                     ${H3DViewer_BINARY_DIR}/../../UI/Release/UI${h3d_msvc_version}.dll
                     CONFIGURATIONS Release
                     DESTINATION H3DViewer/plugins )
    else()
      # these part are added separately so that these plug in can be automatically added to H3DViewer
      install( FILES ${H3DViewer_SOURCE_DIR}/../../../${default_bin_install}/H3DPhysics${h3d_msvc_version}.dll
                     ${H3DViewer_SOURCE_DIR}/../../../${default_bin_install}/MedX3D${h3d_msvc_version}.dll
                     ${H3DViewer_SOURCE_DIR}/../../../${default_bin_install}/UI${h3d_msvc_version}.dll
                     CONFIGURATIONS Release
                     DESTINATION H3DViewer/plugins )
    endif()
    
    if( EXISTS ${H3DViewer_SOURCE_DIR}/../../Util/H3DViewerPackageExtraFiles )
      install( FILES ${H3DViewer_SOURCE_DIR}/../../Util/H3DViewerPackageExtraFiles/README.md
             DESTINATION H3DViewer )
    endif()
  endif()  
  
  include( CPack )

endif()