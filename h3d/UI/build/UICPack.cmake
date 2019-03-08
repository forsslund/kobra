if( NOT TARGET UI )
  message( FATAL_ERROR "Include file UICPack.cmake require the target UI to exist. Please add UI/build/CMakeLists.txt as subdirectory first." )
endif()

# Add all sources, they are added to a variable called UI_SRCS defined
# in the included file. All header files are added to a variable called
# UI_HEADERS.
include( ${UI_SOURCE_DIR}/UISourceFiles.txt )
list( APPEND UI_HEADERS ${UI_SOURCE_DIR}/../include/H3D/UI/StdAfx.h )
list( APPEND UI_SRCS ${UI_SOURCE_DIR}/../src/StdAfx.cpp )

# If cpack should be configured.
if( GENERATE_H3D_PACKAGE_PROJECT )
    set( CPACK_ALL_INSTALL_TYPES Full Developer )
    set( CMAKE_MODULE_PATH "${UI_SOURCE_DIR}/localModules" "${UI_SOURCE_DIR}/modules" )
    set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "UI. A cross platform toolkit that extends H3DAPI with haptic user interface widgets." )
    set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
    set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
    set( CPACK_PACKAGE_DESCRIPTION_FILE "${UI_SOURCE_DIR}/../README.md" )
    set( CPACK_RESOURCE_FILE_LICENSE "${UI_SOURCE_DIR}/../LICENSE" )
    set( CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};UI;ALL;/" )
    set( CPACK_PACKAGE_INSTALL_DIRECTORY "H3D" )
    
    # File patterns to ignore, common for all operating systems.
  set( UI_CPACK_IGNORE_PATTERNS /\\\\.svn/
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
                  /build/vc9
                  /build/vc8
                  /build/vc7
                  /osx
                  /doxygen_log\\\\.txt$
                  "~$" )
                  
    set( CPACK_PACKAGE_VERSION_MAJOR ${UI_MAJOR_VERSION} )
    set( CPACK_PACKAGE_VERSION_MINOR ${UI_MINOR_VERSION} )
    set( CPACK_PACKAGE_VERSION_PATCH ${UI_BUILD_VERSION} )
    
    if( WIN32 AND NOT UNIX )
    set( CPACK_NSIS_INSTALL_ROOT "C:" )
    set( CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL "ON" )  
    set( CPACK_PACKAGE_START_MENU_NAME "UI" )
      
    # Extra links to start menu if values are "ON"
    set( CPACK_ADD_UIDOC_LINKS "ON" )
    set( CPACK_ADD_H3DLOAD_DEMOS_LINKS "ON" )
    set( CPACK_ADD_UI_DEMOS_LINKS "ON" )
  
    # External binary directory to add to path.
    set( CPACK_EXTERNAL_BIN "bin32" )
    set( CPACK_H3D_64_BIT "FALSE" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( CPACK_EXTERNAL_BIN "bin64" )
      set( CPACK_H3D_64_BIT "TRUE" )
    endif()
    
    # Do not modify path.
    set( CPACK_NSIS_MODIFY_PATH "ON" )

    set( CPACK_IGNORE_FILES ${UI_CPACK_IGNORE_PATTERNS} )
    #set( CPACK_INSTALLED_DIRECTORIES "${UI_SOURCE_DIR}/../../UI;." )
  else()
    set( CPACK_SOURCE_IGNORE_FILES ${UI_CPACK_IGNORE_PATTERNS} )
    set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${UI_SOURCE_DIR}/../../UI;UI" )
  endif()
  
  install( FILES ${UI_SOURCE_DIR}/../changelog
                 ${UI_SOURCE_DIR}/../LICENSE
                 ${UI_SOURCE_DIR}/../README.md
      DESTINATION UI
      COMPONENT UI_cpack_sources )
  
    # Install src files.
  install( FILES ${UI_SRCS}
      DESTINATION UI/src
      COMPONENT UI_cpack_sources )
  
  # Install header files
  install( FILES ${UI_HEADERS}
      DESTINATION UI/include/H3D/UI
      COMPONENT UI_cpack_headers )

  # UI.cmake that goes to headers is not needed unless sources is required.
  install( FILES ${UI_SOURCE_DIR}/../include/H3D/UI/UI.cmake
      DESTINATION UI/include/H3D/UI
      COMPONENT UI_cpack_sources )
  
  install( DIRECTORY ${UI_SOURCE_DIR}/../x3d
      DESTINATION UI
      COMPONENT UI_cpack_sources )
  
  install( FILES  ${UI_SOURCE_DIR}/CMakeLists.txt
          ${UI_SOURCE_DIR}/UI.rc.cmake
          ${UI_SOURCE_DIR}/UICPack.cmake
          ${UI_SOURCE_DIR}/UISourceFiles.txt
          ${UI_SOURCE_DIR}/UpdateResourceFile.exe
      DESTINATION UI/build
      COMPONENT UI_cpack_sources )
  
  install( DIRECTORY ${UI_SOURCE_DIR}/modules
           DESTINATION UI/build
           COMPONENT UI_cpack_sources
           REGEX "(/.svn)|(/CVS)" EXCLUDE )
  
  if( ( EXISTS ${UI_SOURCE_DIR}/../doc/UI.tag ) AND ( EXISTS ${UI_SOURCE_DIR}/../doc/html ) )
    # Install documentation
    install( FILES ${UI_SOURCE_DIR}/../doc/UI.tag
             DESTINATION UI/doc
             COMPONENT UI_cpack_headers )
    install( DIRECTORY ${UI_SOURCE_DIR}/../doc/html
             DESTINATION UI/doc
             COMPONENT UI_cpack_headers )
  endif()
  
  # setting names and dependencies between components and also grouping them.
  set( CPACK_COMPONENT_UI_CPACK_RUNTIME_DISPLAY_NAME "Runtime" )
  set( CPACK_COMPONENT_UI_CPACK_RUNTIME_DESCRIPTION "The runtime libraries ( dlls ) for UI." )
    set( CPACK_COMPONENT_UI_CPACK_RUNTIME_DEPENDS H3DAPI_cpack_runtime )
  set( CPACK_COMPONENT_UI_CPACK_RUNTIME_GROUP "UI_cpack_group" )
  set( CPACK_COMPONENT_UI_CPACK_RUNTIME_INSTALL_TYPES Developer Full )
  
  set( CPACK_COMPONENT_UI_CPACK_LIBRARIES_DISPLAY_NAME "Libraries" )
  set( CPACK_COMPONENT_UI_CPACK_LIBRARIES_DESCRIPTION "UI libraries" )
  set( CPACK_COMPONENT_UI_CPACK_LIBRARIES_DEPENDS H3DAPI_cpack_libraries UI_cpack_headers )
  set( CPACK_COMPONENT_UI_CPACK_LIBRARIES_GROUP "UI_cpack_group" )
  set( CPACK_COMPONENT_UI_CPACK_LIBRARIES_INSTALL_TYPES Developer Full )
  
  set( CPACK_COMPONENT_UI_CPACK_HEADERS_DISPLAY_NAME "C++ Headers" )
  set( CPACK_COMPONENT_UI_CPACK_HEADERS_DESCRIPTION "UI C++ Header, need to build against UI" )
  set( CPACK_COMPONENT_UI_CPACK_HEADERS_DEPENDS H3DAPI_cpack_headers UI_cpack_libraries )
  set( CPACK_COMPONENT_UI_CPACK_HEADERS_GROUP "UI_cpack_group" )
  set( CPACK_COMPONENT_UI_CPACK_HEADERS_INSTALL_TYPES Developer Full )
  
  set( CPACK_COMPONENT_UI_CPACK_SOURCES_DISPLAY_NAME "C++ Source" )
  set( CPACK_COMPONENT_UI_CPACK_SOURCES_DESCRIPTION "Everything needed to build UI" )
  set( CPACK_COMPONENT_UI_CPACK_SOURCES_DEPENDS H3DAPI_cpack_sources UI_cpack_headers )
  set( CPACK_COMPONENT_UI_CPACK_SOURCES_GROUP "UI_cpack_group" )
  set( CPACK_COMPONENT_UI_CPACK_SOURCES_INSTALL_TYPES Full )
  
  set( CPACK_COMPONENT_GROUP_UI_CPACK_GROUP_DISPLAY_NAME "UI" )
  set( CPACK_COMPONENT_GROUP_UI_CPACK_GROUP_DESCRIPTION "UI contains haptic user interface nodes. E.g. 3D buttons and sliders which can be touched by a haptics device." )

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
    set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${UI_SOURCE_DIR}/..;/" )  
    set( CPACK_SOURCE_GENERATOR TGZ ZIP ) 
    set( CPACK_SOURCE_PACKAGE_FILE_NAME "h3dui-${UI_MAJOR_VERSION}.${UI_MINOR_VERSION}.${UI_BUILD_VERSION}" ) 


    set( UI_CPACK_IGNORE_PATTERNS ${UI_CPACK_IGNORE_PATTERNS}
            "/CVS/;/.svn/;/.bzr/;/.hg/;/.git.*/;.swp$;.#;/#;~$" )
    set( CPACK_SOURCE_IGNORE_FILES ${UI_CPACK_IGNORE_PATTERNS} )
  endif()


  if( H3D_CMake_runtime_path )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD}
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=UI_cpack_runtime -P cmake_install.cmake 
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=UI_cpack_libraries -P cmake_install.cmake)
      
    if( ${CMAKE_PROJECT_NAME} STREQUAL "UI" )
      add_custom_command( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DummyFile
                          COMMAND echo )
      add_custom_target( INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/DummyFile )
      
      add_custom_command( TARGET INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                          POST_BUILD
                          ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD} )
      add_dependencies( INSTALL_RUNTIME_AND_LIBRARIES_ONLY UI ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
    endif()
  else()
    message( STATUS "H3D_CMake_runtime_path is not set, please set it to continue" )
  endif()
  
  if( ${CMAKE_PROJECT_NAME} STREQUAL "UI" )
    include( CPack )
  endif()
endif()