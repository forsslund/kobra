if( NOT TARGET H3DPhysics )
  message( FATAL_ERROR "Include file H3DPhysicsCPack.cmake require the target H3DPhysics to exist. Please add H3DPhysics/build/CMakeLists.txt as subdirectory first." )
endif()
# Add all sources, they are added to a variable called H3DPhysics_SRCS defined
# in the included file. All header files are added to a variable called
# H3DPhysics_HEADERS.
include( ${H3DPhysics_SOURCE_DIR}/H3DPhysicsSourceFiles.txt )
list( APPEND H3DPhysics_HEADERS_NO_COOKING ${H3DPhysics_SOURCE_DIR}/../include/H3D/H3DPhysics/StdAfx.h )
list( APPEND H3DPhysics_SRCS_NO_COOKING ${H3DPhysics_SOURCE_DIR}/../src/StdAfx.cpp )

# If cpack should be configured.
if( GENERATE_H3D_PACKAGE_PROJECT )
  if( WIN32 )
    handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3D_EXTERNAL_ROOT
                                                  OLD_VARIABLE_NAMES H3DPhysics_CPACK_EXTERNAL_ROOT
                                                  DOC_STRINGS "Set to the External directory used with H3DPhysics, needed to pack properly. If not set FIND_modules will be used instead." )
    # Add a cache variable which indicates where the Externals directory used for packaging
    # RigigBodyPhysics is located. If not set then FIND modules will be used instead.
    if( NOT DEFINED H3D_EXTERNAL_ROOT )
      set( h3d_external_root_default "" )
      if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "H3DPhysics" )
        foreach( external_include_dir_tmp ${H3DPhysics_INCLUDE_DIRS} )
          if( EXISTS ${external_include_dir_tmp}/../include/pthread )
            set( h3d_external_root_default "${external_include_dir_tmp}/.." )
          endif()
        endforeach()
      else()
        set( h3d_external_root_default "$ENV{H3D_EXTERNAL_ROOT}" )
      endif()
      set( H3D_EXTERNAL_ROOT "${h3d_external_root_default}" CACHE PATH "Set to the External directory used with H3DPhysics, needed to pack properly. If not set FIND_modules will be used instead." )
      mark_as_advanced( H3D_EXTERNAL_ROOT )
    endif()
  endif()
  
  # Set information properties about the project to install.
  set( CPACK_ALL_INSTALL_TYPES Full Developer )
  set( CMAKE_MODULE_PATH "${H3DPhysics_SOURCE_DIR}/localModules" "${H3DPhysics_SOURCE_DIR}/modules" )
  set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "H3DPhysics, an implementation for H3DPhysics of the H3DPhysics component of X3D." )
  set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
  set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  set( CPACK_PACKAGE_DESCRIPTION_FILE "${H3DPhysics_SOURCE_DIR}/../README.md" )
  set( CPACK_RESOURCE_FILE_LICENSE "${H3DPhysics_SOURCE_DIR}/../LICENSE" )
  
  # Project to install.
  set( CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};H3DPhysics;ALL;/" )

  # Installation directory for package.
  set( CPACK_PACKAGE_INSTALL_DIRECTORY "H3D" )

  # Our project depends on these debian packages for Linux.
  set(DEBIAN_PACKAGE_DEPENDS "libcurl3, libfreetype6-dev, ftgl-dev, python2.4-dev, libaudiofile0, libvorbis-dev, libopenal-dev, zlib1g-dev, libxerces27-dev, libfreeimage-dev, hapi(>= 1.0.0), h3dutil(>=1.0.0)" )
  
  # File patterns to ignore, common for all operating systems.
  set( H3DPhysics_CPACK_IGNORE_PATTERNS /\\\\.svn/
                                    \\\\.obj$
                                    \\\\.ncb$
                                    \\\\.log$
                                    \\\\.suo$
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

  set( CPACK_PACKAGE_VERSION_MAJOR ${H3DPHYSICS_MAJOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_MINOR ${H3DPHYSICS_MINOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_PATCH ${H3DPHYSICS_BUILD_VERSION} )
  
  if( WIN32 AND NOT UNIX )
    set( CPACK_NSIS_INSTALL_ROOT "C:" )
    set( CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL "ON" )
    set( CPACK_PACKAGE_START_MENU_NAME "H3DPhysics" )
    set( CPACK_ADD_H3DPhysics_DEMOS_LINKS "ON" )
    
    # external_includes and external_include_install_paths must be of equal lengths.
    # The reason for defining these variables here is in case we want to add functionality
    # to configure installation in some other way (using FIND-modules for example).
    set( external_includes "" )
    set( external_include_install_paths "" )
    # The external_include_files are installed directly in External/include
    set( external_include_files "" )
    set( external_libraries "" )
    set( external_static_libraries "" )
    set( external_binaries "" )
    
    # External binary directory to add to path.
    set( CPACK_EXTERNAL_BIN "bin32" )
    set( external_bin_replace_path "bin64" )
    set( CPACK_H3D_64_BIT "FALSE" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( CPACK_EXTERNAL_BIN "bin64" )
      set( external_bin_replace_path "bin32" )
      set( CPACK_H3D_64_BIT "TRUE" )
    endif()
    set( external_bin_path "${CPACK_EXTERNAL_BIN}" )

    if( EXISTS ${H3D_EXTERNAL_ROOT} )
      set( external_includes ${H3D_EXTERNAL_ROOT}/include/ode/
                             ${H3D_EXTERNAL_ROOT}/include/bullet/)

      set( external_include_install_paths External/include/ode
                                          External/include/bullet )

      set( external_libraries ${H3D_EXTERNAL_ROOT}/lib32/ode_double.lib )

      set( external_static_libraries ${external_static_libraries}
                   ${H3D_EXTERNAL_ROOT}/lib32/static/BulletCollision.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/BulletCollision_Debug.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/BulletDynamics.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/BulletDynamics_Debug.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/BulletSoftBody.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/BulletSoftBody_Debug.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/gtest.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/gtest_Debug.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/HACD_LIB.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/HACD_LIB_DEBUG.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/LinearMath.lib
                   ${H3D_EXTERNAL_ROOT}/lib32/static/LinearMath_Debug.lib )

      set( external_binaries ${external_binaries}
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/ode_double.dll )

    else()
      message( WARNING "H3D_EXTERNAL_ROOT must be set to the External directory used by H3DPhysics in order to package properly." )
    endif()

    if( external_includes )
      list( LENGTH external_includes external_includes_length )
      math( EXPR external_includes_length "${external_includes_length} - 1" )
      foreach( val RANGE ${external_includes_length} )
        list( GET external_includes ${val} val1 )
        list( GET external_include_install_paths ${val} val2 )
        install( DIRECTORY ${val1}
                 DESTINATION ${val2}
                 COMPONENT H3DPhysics_cpack_external_source
                 REGEX "(/.svn)|(/CVS)" EXCLUDE )
      endforeach()
    endif()

    foreach( include_file ${external_include_files} )
      if( EXISTS ${include_file} )
        install( FILES ${include_file}
                 DESTINATION External/include
                 COMPONENT H3DPhysics_cpack_external_source )
      endif()
    endforeach()

    #install all the libraries
    foreach( library ${external_libraries} )
      if( EXISTS ${library} )
        install( FILES ${library}
                 DESTINATION External/lib32
                 COMPONENT H3DPhysics_cpack_external_source )
      endif()
      # Add the other library path as well
      string( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      if( EXISTS ${other_library} )
        install( FILES ${other_library}
                 DESTINATION External/lib64
                 COMPONENT H3DPhysics_cpack_external_source )
      endif()
    endforeach()

    foreach( library ${external_static_libraries} )
      if( EXISTS ${library} )
        install( FILES ${library}
                 DESTINATION External/lib32/static
                 COMPONENT H3DPhysics_cpack_external_source )
      endif()
      # Add the other library path as well
      string( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      if( EXISTS ${other_library} )
        install( FILES ${other_library}
                 DESTINATION External/lib64/static
                 COMPONENT H3DPhysics_cpack_external_source )
      endif()
    endforeach()

    #install all the binaries
    foreach( binary ${external_binaries} )
      if( EXISTS ${binary} )
        install( FILES ${binary}
                 DESTINATION External/${external_bin_path}
                 COMPONENT H3DPhysics_cpack_external_runtime )
      endif()

      string( REGEX REPLACE "(/${external_bin_path}/)" "/${external_bin_replace_path}/" other_binary ${binary} )
      if( EXISTS ${other_binary} )
        install( FILES ${other_binary}
                 DESTINATION External/${external_bin_replace_path}
                 COMPONENT H3DPhysics_cpack_external_runtime )
      endif()
    endforeach()

    # setting names and dependencies between components and also grouping them.
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_RUNTIME_DISPLAY_NAME "External runtime" )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_RUNTIME_DESCRIPTION "External runtime binaries needed by H3DPhysics." )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_RUNTIME_DEPENDS H3DAPI_cpack_external_runtime )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_RUNTIME_GROUP "H3DPhysics_cpack_group" )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_RUNTIME_INSTALL_TYPES Developer Full )
    
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_SOURCE_DISPLAY_NAME "External header/libraries" )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_SOURCE_DESCRIPTION "External headers and libraries needed by H3DPhysics." )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_SOURCE_DEPENDS H3DAPI_cpack_external_source H3DPhysics_cpack_external_runtime )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_SOURCE_GROUP "H3DPhysics_cpack_group" )
    set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXTERNAL_SOURCE_INSTALL_TYPES Developer Full )

    #Modify path in the the NSIS template.
    set( CPACK_NSIS_MODIFY_PATH "ON" )
  endif()
  
  #install changlog, license and readme
  install( FILES ${H3DPhysics_SOURCE_DIR}/../changelog
                 ${H3DPhysics_SOURCE_DIR}/../LICENSE
                 ${H3DPhysics_SOURCE_DIR}/../README.md
      DESTINATION H3DPhysics
      COMPONENT H3DPhysics_cpack_sources )

  #install header files
  install( FILES ${H3DPhysics_HEADERS_NO_COOKING}
      DESTINATION H3DPhysics/include/H3D/H3DPhysics
      COMPONENT H3DPhysics_cpack_headers )
      
  install( FILES ${H3DPhysics_HEADERS_COOKING}
      DESTINATION H3DPhysics/include/H3D/H3DPhysics/PhysX
      COMPONENT H3DPhysics_cpack_headers )

  # H3DPhysics.cmake that goes to headers is not needed unless sources is required.
  install( FILES ${H3DPhysics_SOURCE_DIR}/../include/H3D/H3DPhysics/H3DPhysics.cmake
      DESTINATION H3DPhysics/include/H3D/H3DPhysics
      COMPONENT H3DPhysics_cpack_sources )

  #install source files
  install( FILES ${H3DPhysics_SRCS_NO_COOKING}
      DESTINATION H3DPhysics/src
      COMPONENT H3DPhysics_cpack_sources )

  install( FILES ${H3DPhysics_SRCS_COOKING}
      DESTINATION H3DPhysics/src/PhysX
      COMPONENT H3DPhysics_cpack_sources )

  #install build directory
  install( FILES ${H3DPhysics_SOURCE_DIR}/CMakeLists.txt
         ${H3DPhysics_SOURCE_DIR}/H3DPhysics.rc.cmake
         ${H3DPhysics_SOURCE_DIR}/H3DPhysicsCpack.cmake
         ${H3DPhysics_SOURCE_DIR}/H3DPhysicsSourceFiles.txt
         ${H3DPhysics_SOURCE_DIR}/UpdateResourceFile.exe
         DESTINATION H3DPhysics/build
         COMPONENT H3DPhysics_cpack_sources )
  
  #install module directory
  install( DIRECTORY ${H3DPhysics_SOURCE_DIR}/modules
           DESTINATION H3DPhysics/build
           COMPONENT H3DPhysics_cpack_sources
           REGEX "(/.svn)|(/CVS)" EXCLUDE )

  #install example directory
  install( DIRECTORY ${H3DPhysics_SOURCE_DIR}/../examples
           DESTINATION H3DPhysics
           COMPONENT H3DPhysics_cpack_sources )

  if( ( EXISTS ${H3DPhysics_SOURCE_DIR}/../doc/H3DPhysics.tag ) AND ( EXISTS ${H3DPhysics_SOURCE_DIR}/../doc/html ) )
    # Install documentation
    install( FILES ${H3DPhysics_SOURCE_DIR}/../doc/H3DPhysics.tag
             DESTINATION H3DPhysics/doc
             COMPONENT H3DPhysics_cpack_headers )
    install( DIRECTORY ${H3DPhysics_SOURCE_DIR}/../doc/html
             DESTINATION H3DPhysics/doc
             COMPONENT H3DPhysics_cpack_headers )
  endif()
  
  if( EXISTS ${H3DPhysics_SOURCE_DIR}/../doc/python/doc )
    # Install documentation
    install( DIRECTORY ${H3DPhysics_SOURCE_DIR}/../doc/python/doc
             DESTINATION H3DPhysics/doc/python
             COMPONENT H3DPhysics_cpack_headers )
  endif()
  
  # setting names and dependencies between components and also grouping them.
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_RUNTIME_DISPLAY_NAME "Runtime" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_RUNTIME_DESCRIPTION "The runtime libraries ( dlls ) for H3DPhysics." )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_RUNTIME_DEPENDS  H3DAPI_cpack_runtime H3DPhysics_cpack_external_runtime )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_RUNTIME_GROUP "H3DPhysics_cpack_group" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_RUNTIME_INSTALL_TYPES Developer Full )
    
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_LIBRARIES_DISPLAY_NAME "Libraries" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_LIBRARIES_DESCRIPTION "H3DPhysics libraries, needed for building against H3DPhysics." )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_LIBRARIES_DEPENDS H3DAPI_cpack_libraries H3DPhysics_cpack_external_source H3DPhysics_cpack_headers )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_LIBRARIES_GROUP "H3DPhysics_cpack_group" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_LIBRARIES_INSTALL_TYPES Developer Full )
    
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_HEADERS_DISPLAY_NAME "C++ Headers" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_HEADERS_DESCRIPTION "H3DPhysics C++ headers, needed for building against H3DPhysics." )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_HEADERS_DEPENDS H3DAPI_cpack_headers H3DPhysics_cpack_external_source H3DPhysics_cpack_libraries )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_HEADERS_GROUP "H3DPhysics_cpack_group" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_HEADERS_INSTALL_TYPES Developer Full )
    
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_SOURCES_DISPLAY_NAME "C++ Source" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_SOURCES_DESCRIPTION "Everything needed to build H3DPhysics." )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_SOURCES_DEPENDS H3DAPI_cpack_sources H3DPhysics_cpack_headers )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_SOURCES_GROUP "H3DPhysics_cpack_group" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_SOURCES_INSTALL_TYPES Full )
  
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXAMPLES_RUNTIME_DISPLAY_NAME "Example applications" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXAMPLES_RUNTIME_DESCRIPTION "The example applications for H3DPhysics." )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXAMPLES_RUNTIME_DEPENDS H3DPhysics_cpack_runtime )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXAMPLES_RUNTIME_GROUP "H3DPhysics_cpack_group" )
  set( CPACK_COMPONENT_H3DPHYSICS_CPACK_EXAMPLES_RUNTIME_INSTALL_TYPES Developer Full )
  
  set( CPACK_COMPONENT_GROUP_H3DPHYSICS_CPACK_GROUP_DISPLAY_NAME "H3DPhysics" )
  set( CPACK_COMPONENT_GROUP_H3DPHYSICS_CPACK_GROUP_DESCRIPTION "H3DPhysics implements the RigidBodyPhysics component of the X3D specification with additional nodes for soft body physics." )

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
        
        if(EXISTS "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe")
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
    set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${H3DPhysics_SOURCE_DIR}/..;/" )  
    set( CPACK_SOURCE_GENERATOR TGZ ZIP ) 
    set( CPACK_SOURCE_PACKAGE_FILE_NAME "h3dphysics-${H3DPHYSICS_MAJOR_VERSION}.${H3DPHYSICS_MINOR_VERSION}.${H3DPHYSICS_BUILD_VERSION}" ) 


    set( H3DPhysics_CPACK_IGNORE_PATTERNS ${H3DPhysics_CPACK_IGNORE_PATTERNS}
            "/CVS/;/.svn/;/.bzr/;/.hg/;/.git.*/;.swp$;.#;/#;~$" )
    set( CPACK_SOURCE_IGNORE_FILES ${H3DPhysics_CPACK_IGNORE_PATTERNS} )
  endif()


  if( H3D_CMake_runtime_path )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD}
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=H3DPhysics_cpack_runtime -P cmake_install.cmake 
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=H3DPhysics_cpack_libraries -P cmake_install.cmake
                                                       COMMAND ${H3D_CMake_runtime_path} 
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=MedX3D_cpack_examples_runtime -P cmake_install.cmake )
    
    if( ${CMAKE_PROJECT_NAME} STREQUAL "H3DPhysics" )
      add_custom_command( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DummyFile
                          COMMAND echo )
      add_custom_target( INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/DummyFile )
      
      add_custom_command( TARGET INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                          POST_BUILD
                          ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD} )
      add_dependencies( INSTALL_RUNTIME_AND_LIBRARIES_ONLY H3DPhysics ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
    endif()
  else()
    message( STATUS "H3D_CMake_runtime_path is not set, please set it to continue" )
  endif()
  
  if( ${CMAKE_PROJECT_NAME} STREQUAL "H3DPhysics" )
    include( CPack )
  endif()
  # CPack said: could not find load file Debian
  # include( UseDebian )
  # if( DEBIAN_FOUND )
    # ADD_DEBIAN_TARGETS( H3DPhysics )
  # endif()
endif()
