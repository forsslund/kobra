if( NOT TARGET MedX3D )
  message( FATAL_ERROR "Include file MedX3DCPack.cmake require the target MedX3D to exist. Please add MedX3D/build/CMakeLists.txt as subdirectory first." )
endif()

# Add all sources, they are added to a variable called MedX3D_SRCS defined
# in the included file. All header files are added to a variable called
# MedX3D_HEADERS.
include( ${MedX3D_SOURCE_DIR}/MedX3DSourceFiles.txt )

if( GENERATE_H3D_PACKAGE_PROJECT )
  if( WIN32 )
    handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3D_EXTERNAL_ROOT
                                                  OLD_VARIABLE_NAMES MedX3D_CPACK_EXTERNAL_ROOT
                                                  DOC_STRINGS "Set to the External directory used with MedX3D, needed to pack properly. If not set FIND_modules will be used instead." )

    # Add a cache variable which indicates where the Externals directory used for packaging
    # MedX3D is located. If not set then FIND modules will be used instead.
    if( NOT DEFINED H3D_EXTERNAL_ROOT )
      set( H3D_EXTERNAL_ROOT "$ENV{H3D_EXTERNAL_ROOT}" CACHE PATH "Set to the External directory used with MedX3D, needed to pack properly. If not set FIND_modules will be used instead." )
      mark_as_advanced( H3D_EXTERNAL_ROOT )
    endif()
  endif()

  # Set information properties about the project to install.
  set( CPACK_ALL_INSTALL_TYPES Full Developer )
  set( CMAKE_MODULE_PATH "${MedX3D_SOURCE_DIR}/localModules" "${MedX3D_SOURCE_DIR}/modules" )
  set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "MedX3D. A cross platform toolkit that extends MedX3D with the volume rendering component of X3D." )
  set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
  set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  set( CPACK_PACKAGE_DESCRIPTION_FILE "${MedX3D_SOURCE_DIR}/../README.md" )
  set( CPACK_RESOURCE_FILE_LICENSE "${MedX3D_SOURCE_DIR}/../LICENSE" )
  
  # Project to install.
  set( CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};MedX3D;ALL;/" )

  # Installation directory for package.
  set( CPACK_PACKAGE_INSTALL_DIRECTORY "H3D" )
  
  # File patterns to ignore, common for all operating systems.
  set( MedX3D_CPACK_IGNORE_PATTERNS /\\\\.svn/
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
                                    "/Xj3D Extensions - Volume Rendering\\\\.pdf$"
                                    /notes\\\\.txt$
                                    /volren_composite\\\\.x3d$
                                    /volren_opacity_test\\\\.x3d$
                                    /gaussian64\\\\.raw$
                                    /hydrogen64\\\\.raw$
                                    /skull128\\\\.raw$
                                    /iron\\\\.png$
                                    /medx3d\\\\.png$
                                    /BoundaryEnhancementVolumeStyle_FragmentShader\\\\.glsl$
                                    /CartoonVolumeStyle_FragmentShader\\\\.glsl$
                                    /EdgeEnhancementVolumeStyle_FragmentShader\\\\.glsl$
                                    /ISOSurfaceVolumeStyle_FragmentShader\\\\.glsl$
                                    /MIPVolumeStyle_FragmentShader\\\\.glsl$
                                    /OpacityMapVolumeStyle_FragmentShader\\\\.glsl$
                                    /SilhouetteEnhancementVolumeStyle_FragmentShader\\\\.glsl$
                                    /Template_FragmentShader\\\\.glsl$
                                    /ToneMappedVolumeStyle_FragmentShader\\\\.glsl$
                                    /X3DVolumeRenderStyleNode_FS\\\\.glsl$
                                    /X3DVolumeRenderStyleNode_VertexShader\\\\.glsl$ )

  set( CPACK_PACKAGE_VERSION_MAJOR ${MEDX3D_MAJOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_MINOR ${MEDX3D_MINOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_PATCH ${MEDX3D_BUILD_VERSION} )
  
  if( WIN32 AND NOT UNIX )
    set( CPACK_NSIS_INSTALL_ROOT "C:" )
    set( CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL "ON" )
    set( CPACK_PACKAGE_START_MENU_NAME "MedX3D for H3DAPI 2.2" )
    #Extra links to start menu if values are "ON"
    set( CPACK_ADD_MedX3DDOC_LINKS "ON" )
    set( CPACK_ADD_H3DLOAD_DEMOS_LINKS "ON" )
    set( CPACK_ADD_MedX3D_DEMOS_LINKS "ON" )
    #set( NSIS_OPTIONS_NUMFIELDS "4" )
  
    # External binary directory to add to path.
    set( CPACK_H3D_64_BIT "FALSE" )
    set( CPACK_EXTERNAL_BIN "bin32" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( CPACK_EXTERNAL_BIN "bin64" )
      set( CPACK_H3D_64_BIT "TRUE" )
    endif()
    
      
    #We dont set any lib/bin/header here because all of them are required and should be added by CMakeLists.txt by default
  
    # autogenerate NSIS.InstallOptions.ini depending on the version
    #configure_file( ${MedX3D_SOURCE_DIR}/localModules/NSIS.InstallOptions.ini.in.cmake ${MedX3D_SOURCE_DIR}/localModules/NSIS.InstallOptions.ini.in )

  else()
    set( MedX3D_CPACK_IGNORE_PATTERNS ${MedX3D_CPACK_IGNORE_PATTERNS}
                                      /plugin/
                                      /dcmtk/
                                      /test_plugin\\\\.htm$
                                      /x3d/volren_composed_test\\\\.html$
                                      /x3d/volren_opacity_test\\\\.html$
                                      /x3d/volren_seg_test\\\\.html$
                                      "~$" )
  endif()
  
  #install header directory
  install( FILES ${MedX3D_HEADERS}
           DESTINATION MedX3D/include/H3D/MedX3D
           COMPONENT MedX3D_cpack_headers )

  # MedX3D.cmake that goes to headers is not needed unless sources is required.
  install( FILES ${MedX3D_SOURCE_DIR}/../include/H3D/MedX3D/MedX3D.cmake
      DESTINATION MedX3D/include/H3D/MedX3D
      COMPONENT MedX3D_cpack_sources )
       
  #install build files
  install( FILES ${MedX3D_SOURCE_DIR}/CMakeLists.txt
         ${MedX3D_SOURCE_DIR}/MedX3D.rc.cmake
         ${MedX3D_SOURCE_DIR}/MedX3DCPack.cmake
         ${MedX3D_SOURCE_DIR}/MedX3DSourceFiles.txt
         ${MedX3D_SOURCE_DIR}/UpdateResourceFile.exe
           DESTINATION MedX3D/build
           COMPONENT MedX3D_cpack_sources )
       
  #install module directory
  install( DIRECTORY ${MedX3D_SOURCE_DIR}/modules
           DESTINATION MedX3D/build
           COMPONENT MedX3D_cpack_sources
           REGEX "(/.svn)|(/CVS)" EXCLUDE )
       
  #install data directory
  install( DIRECTORY ${MedX3D_SOURCE_DIR}/../data
           DESTINATION MedX3D
           COMPONENT MedX3D_cpack_sources )

  #install demo directory
  install( DIRECTORY ${MedX3D_SOURCE_DIR}/../demo
           DESTINATION MedX3D
           COMPONENT MedX3D_cpack_sources )

  #install source directory
  install( DIRECTORY ${MedX3D_SOURCE_DIR}/../src
           DESTINATION MedX3D
           COMPONENT MedX3D_cpack_sources )
       
  #install x3d directory
  install( DIRECTORY ${MedX3D_SOURCE_DIR}/../x3d
           DESTINATION MedX3D
           COMPONENT MedX3D_cpack_sources )
       
  #install change log, readme, etc files
  install( FILES ${MedX3D_SOURCE_DIR}/../changelog
         ${MedX3D_SOURCE_DIR}/../LICENSE
         ${MedX3D_SOURCE_DIR}/../medx3d.txt
         ${MedX3D_SOURCE_DIR}/../README.md
           DESTINATION MedX3D
           COMPONENT MedX3D_cpack_sources )
  
  if( ( EXISTS ${MedX3D_SOURCE_DIR}/../doc/MedX3D.tag ) AND ( EXISTS ${MedX3D_SOURCE_DIR}/../doc/html ) )
    # Install documentation
    install( FILES ${MedX3D_SOURCE_DIR}/../doc/MedX3D.tag
             DESTINATION MedX3D/doc
             COMPONENT MedX3D_cpack_headers )
    install( DIRECTORY ${MedX3D_SOURCE_DIR}/../doc/html
             DESTINATION MedX3D/doc
             COMPONENT MedX3D_cpack_headers )
  endif()

  # setting names and dependencies between components and also grouping them.
  set( CPACK_COMPONENT_MEDX3D_CPACK_RUNTIME_DISPLAY_NAME "Runtime" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_RUNTIME_DESCRIPTION "The runtime libraries ( dlls ) for MedX3D." )
  set( CPACK_COMPONENT_MEDX3D_CPACK_RUNTIME_DEPENDS H3DAPI_cpack_runtime )
  set( CPACK_COMPONENT_MEDX3D_CPACK_RUNTIME_GROUP "MedX3D_cpack_group" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_RUNTIME_INSTALL_TYPES Developer Full )
  
  set( CPACK_COMPONENT_MEDX3D_CPACK_LIBRARIES_DISPLAY_NAME "Libraries" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_LIBRARIES_DESCRIPTION "MedX3D libraries, needed for building against MedX3D." )
  set( CPACK_COMPONENT_MEDX3D_CPACK_LIBRARIES_DEPENDS H3DAPI_cpack_libraries MedX3D_cpack_headers )
  set( CPACK_COMPONENT_MEDX3D_CPACK_LIBRARIES_GROUP "MedX3D_cpack_group" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_LIBRARIES_INSTALL_TYPES Developer Full )
    
  set( CPACK_COMPONENT_MEDX3D_CPACK_HEADERS_DISPLAY_NAME "C++ Headers" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_HEADERS_DESCRIPTION "MedX3D C++ headers, needed for building against MedX3D." )
  set( CPACK_COMPONENT_MEDX3D_CPACK_HEADERS_DEPENDS H3DAPI_cpack_headers MedX3D_cpack_libraries )
  set( CPACK_COMPONENT_MEDX3D_CPACK_HEADERS_GROUP "MedX3D_cpack_group" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_HEADERS_INSTALL_TYPES Developer Full )
    
  set( CPACK_COMPONENT_MEDX3D_CPACK_SOURCES_DISPLAY_NAME "C++ Source" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_SOURCES_DESCRIPTION "Everything needed to build MedX3D." )
  set( CPACK_COMPONENT_MEDX3D_CPACK_SOURCES_DEPENDS H3DAPI_cpack_sources MedX3D_cpack_headers )
  set( CPACK_COMPONENT_MEDX3D_CPACK_SOURCES_GROUP "MedX3D_cpack_group" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_SOURCES_INSTALL_TYPES Full )
  
  set( CPACK_COMPONENT_MEDX3D_CPACK_EXAMPLES_RUNTIME_DISPLAY_NAME "Example applications" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_EXAMPLES_RUNTIME_DESCRIPTION "The example applications for MedX3D." )
  set( CPACK_COMPONENT_MEDX3D_CPACK_EXAMPLES_RUNTIME_DEPENDS MedX3D_cpack_runtime )
  set( CPACK_COMPONENT_MEDX3D_CPACK_EXAMPLES_RUNTIME_GROUP "MedX3D_cpack_group" )
  set( CPACK_COMPONENT_MEDX3D_CPACK_EXAMPLES_RUNTIME_INSTALL_TYPES Developer Full )
  
  set( CPACK_COMPONENT_GROUP_MEDX3D_CPACK_GROUP_DISPLAY_NAME "MedX3D" )
  set( CPACK_COMPONENT_GROUP_MEDX3D_CPACK_GROUP_DESCRIPTION "MedX3D implements the Volume Rendering component of the X3D specification." )

  handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3D_CMake_runtime_path
                                                OLD_VARIABLE_NAMES H3D_cmake_runtime_path
                                                DOC_STRINGS "The path to the cmake runtime." )

  # Add a cache variable H3D_CMake_runtime_path to point to cmake binary.
  set( h3d_cmake_runtime_path_default "" )
  if( NOT DEFINED H3D_CMake_runtime_path )
    if( WIN32 AND NOT UNIX )
      set( VERSION_CMAKES "4.0" "3.9" "3.8" "3.7" "3.6" "3.5" "3.4" "3.3" "3.2" "3.1" "3.0" "2.9" "2.8" "2.7" "2.6" )
      foreach( version_cmake ${VERSION_CMAKES} )
        if( EXISTS "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe" )
          set( h3d_cmake_runtime_path_default "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()
        
        if( EXISTS "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe" )
          set( h3d_cmake_runtime_path_default "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()
        
        if( EXISTS "C:/Program/CMake ${version_cmake}/bin/cmake.exe" )
          set( h3d_cmake_runtime_path_default "C:/Program/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()
      endforeach()
    else()
      set( h3d_cmake_runtime_path_default "cmake" )
    endif()
    set( H3D_CMake_runtime_path ${h3d_cmake_runtime_path_default} CACHE FILEPATH "The path to the cmake runtime." )
    mark_as_advanced( H3D_CMake_runtime_path )
  endif()
  
  if( UNIX )
    set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${MedX3D_SOURCE_DIR}/..;/" )  
    set( CPACK_SOURCE_GENERATOR TGZ ZIP ) 
    set( CPACK_SOURCE_PACKAGE_FILE_NAME "h3dmedx3d-${MEDX3D_MAJOR_VERSION}.${MEDX3D_MINOR_VERSION}.${MEDX3D_BUILD_VERSION}" ) 


    set( MedX3D_CPACK_IGNORE_PATTERNS ${MedX3D_CPACK_IGNORE_PATTERNS}
            "/CVS/;/.svn/;/.bzr/;/.hg/;/.git.*/;.swp$;.#;/#;~$" )
    set( CPACK_SOURCE_IGNORE_FILES ${MedX3D_CPACK_IGNORE_PATTERNS} )
  endif()

  if( H3D_CMake_runtime_path )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD} 
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=MedX3D_cpack_runtime -P cmake_install.cmake 
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=MedX3D_cpack_libraries -P cmake_install.cmake
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=MedX3D_cpack_examples_runtime -P cmake_install.cmake )
    
    if( ${CMAKE_PROJECT_NAME} STREQUAL "MedX3D" )
      add_custom_command( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DummyFile
                          COMMAND echo )
      add_custom_target( INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/DummyFile )

      add_custom_command( TARGET INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                          POST_BUILD
                          ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD} )
      add_dependencies( INSTALL_RUNTIME_AND_LIBRARIES_ONLY MedX3D ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
    endif()
  else()
    message( STATUS "H3D_CMake_runtime_path is not set, please set it to continue" )
  endif()
  
  if( ${CMAKE_PROJECT_NAME} STREQUAL "MedX3D" )
    include( CPack )
  endif()
  #CPack said: could not find load file Debian
  # include( UseDebian )
  # if( DEBIAN_FOUND )
    # ADD_DEBIAN_TARGETS( MedX3D )
  # endif()
endif()
