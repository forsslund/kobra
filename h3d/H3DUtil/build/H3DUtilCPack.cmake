if( NOT TARGET H3DUtil )
  message( FATAL_ERROR "Include file H3DUtilCPack.cmake require the target H3DUtil to exist. Please add H3DUtil/build/CMakeLists.txt as subdirectory first." )
endif()

# Add all sources, they are added to a variable called H3DUTIL_SRCS defined
# in the included file. All header files are added to a variable called
# H3DUTIL_HEADERS.
include( ${H3DUtil_SOURCE_DIR}/H3DUtilSourceFiles.txt )

# To allow other projects that use H3DUtil as a subproject to add extra include directories
# when packaging.
if( GENERATE_H3D_PACKAGE_PROJECT )
  if( WIN32 )
    # external_includes and external_include_install_paths must be of equal lengths.
    # The reason for defining these variables here is in case we want to add functionality
    # to configure installation in some other way (using FIND-modules for example).
    set( external_includes "" )
    set( external_include_install_paths "" )
    set( external_libraries "" )
    set( external_static_libraries "" )
    set( external_binaries "" )

    handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3D_EXTERNAL_ROOT
                                                  OLD_VARIABLE_NAMES H3DUtil_CPACK_EXTERNAL_ROOT
                                                  DOC_STRINGS "Set to the External directory used with H3DUtil, needed to pack properly. If not set FIND_modules will be used instead." )

    # Add a cache variable which indicates where the Externals directory used for packaging
    # H3DUtil is located. If not set then FIND modules will be used instead.
    if( NOT DEFINED H3D_EXTERNAL_ROOT )
      if( NOT DEFINED H3D_EXTERNAL_ROOT )
        set( h3d_external_root_default "" )
        if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "H3DUtil" )
          foreach( external_include_dir_tmp ${H3DUtil_INCLUDE_DIRS} )
            if( EXISTS ${external_include_dir_tmp}/../include/pthread )
              set( h3d_external_root_default "${external_include_dir_tmp}/.." )
            endif()
          endforeach()
        else()
          set( h3d_external_root_default "$ENV{H3D_EXTERNAL_ROOT}" )
        endif()
        set( H3D_EXTERNAL_ROOT "${h3d_external_root_default}" CACHE PATH "Set to the External directory used with H3DUtil, needed to pack properly. If not set FIND_modules will be used instead." )
        mark_as_advanced( H3D_EXTERNAL_ROOT )
      endif()
    else()
      set( H3D_EXTERNAL_ROOT ${H3D_EXTERNAL_ROOT} )
    endif()

    set( external_bin_path "bin32" )
    set( external_bin_replace_path "bin64" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # check if the system is 64 bit
      set( external_bin_path "bin64" )
      set( external_bin_replace_path "bin32" )
    endif()

    if( EXISTS ${H3D_EXTERNAL_ROOT} )
      set( external_includes ${H3D_EXTERNAL_ROOT}/include/pthread/
                             ${H3D_EXTERNAL_ROOT}/include/FreeImage/
                             ${H3D_EXTERNAL_ROOT}/include/zlib/
                             ${H3D_EXTERNAL_ROOT}/include/dcmtk/
                             ${H3D_EXTERNAL_ROOT}/include/teem/
                             ${H3D_EXTERNAL_ROOT}/include/Bzip2/
                             ${H3D_EXTERNAL_ROOT}/include/sofahelper/
                             ${H3D_EXTERNAL_ROOT}/include/vld/
                             ${H3D_EXTERNAL_ROOT}/include/OpenEXR/ )
      set( external_include_install_paths External/include/pthread
                                          External/include/FreeImage
                                          External/include/zlib
                                          External/include/dcmtk
                                          External/include/teem
                                          External/include/Bzip2
                                          External/include/sofahelper
                                          External/include/vld
                                          External/include/OpenEXR )

      set( external_libraries ${H3D_EXTERNAL_ROOT}/lib32/pthreadVC2.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/FreeImage.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/zlib.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/teem.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/libbz2.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/vld.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/SofaHelper_1_02.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/SofaHelper_1_0d2.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/Iex.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/Iex_d.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IexMath.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IexMath_d.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IlmImf.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IlmImf_d.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IlmImfUtil.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IlmImfUtil_d.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IlmThread.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/IlmThread_d.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/Imath.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/Imath_d.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/Half.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/Half_d.lib )
      set( dcm_name_list dcmjpeg ofstd oflog dcmimage dcmdata dcmimgle ijg8 ijg12 ijg16 )
      foreach( library_name ${dcm_name_list} )
        set( external_static_libraries ${external_static_libraries}
                                       #${H3D_EXTERNAL_ROOT}/lib32/static/${library_name}_vc7.lib
                                       #${H3D_EXTERNAL_ROOT}/lib32/static/${library_name}_vc8.lib
                                       #${H3D_EXTERNAL_ROOT}/lib32/static/${library_name}_vc9.lib
                                       ${H3D_EXTERNAL_ROOT}/lib32/static/${library_name}.lib
                                       ${H3D_EXTERNAL_ROOT}/lib32/static/${library_name}_d.lib )
      endforeach()
      set( external_static_libraries ${external_static_libraries}
                                     ${H3D_EXTERNAL_ROOT}/lib32/static/teem.lib )

      set( external_binaries ${H3D_EXTERNAL_ROOT}/${external_bin_path}/pthreadVC2.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/FreeImage.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/zlib.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/teem.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/libbz2.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/SofaHelper_1_0d2.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/SofaHelper_1_02.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/vld_x86.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/dbghelp.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/Iex.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/Iex_d.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IexMath.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IexMath_d.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IlmImf.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IlmImf_d.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IlmImfUtil.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IlmImfUtil_d.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IlmThread.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/IlmThread_d.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/Imath.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/Imath_d.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/Half.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/Half_d.dll )

    elseif( NOT DEFINED H3D_EXTERNAL_ROOT )
      message( WARNING "H3D_EXTERNAL_ROOT must be set to the External directory used by H3DUtil in order to package properly." )
    endif()

    if( external_includes )
      list( LENGTH external_includes external_includes_length )
      math( EXPR external_includes_length "${external_includes_length} - 1" )
      foreach( val RANGE ${external_includes_length} )
        list( GET external_includes ${val} val1 )
        list( GET external_include_install_paths ${val} val2 )
        install( DIRECTORY ${val1}
                 DESTINATION ${val2}
                 COMPONENT H3DUtil_cpack_external_source
                 REGEX "(/.svn)|(/CVS)" EXCLUDE )
      endforeach()
    endif()

    install( DIRECTORY ${H3D_EXTERNAL_ROOT}/include/ExternalLicenses/
                 DESTINATION External/include/ExternalLicenses
                 COMPONENT H3DUtil_cpack_external_runtime
                 REGEX "(/.svn)|(/CVS)" EXCLUDE )

    if( EXISTS "${H3D_EXTERNAL_ROOT}/include/ACKNOWLEDGEMENTS" )
      install( FILES "${H3D_EXTERNAL_ROOT}/include/ACKNOWLEDGEMENTS"
               DESTINATION External/include
               COMPONENT H3DUtil_cpack_external_runtime )
    endif()

    foreach( library ${external_libraries} )
      if( EXISTS ${library} )
        install( FILES ${library}
                 DESTINATION External/lib32
                 COMPONENT H3DUtil_cpack_external_source )
      endif()
      # Add the other library path as well
      string( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      if( EXISTS ${other_library} )
        install( FILES ${other_library}
                 DESTINATION External/lib64
                 COMPONENT H3DUtil_cpack_external_source )
      endif()
    endforeach()

    foreach( library ${external_static_libraries} )
      if( EXISTS ${library} )
        install( FILES ${library}
                 DESTINATION External/lib32/static
                 COMPONENT H3DUtil_cpack_external_source )
      endif()
      # Add the other library path as well
      string( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      if( EXISTS ${other_library} )
        install( FILES ${other_library}
                 DESTINATION External/lib64/static
                 COMPONENT H3DUtil_cpack_external_source )
      endif()
    endforeach()

    foreach( binary ${external_binaries} )
      if( EXISTS ${binary} )
        install( FILES ${binary}
                 DESTINATION External/${external_bin_path}
                 COMPONENT H3DUtil_cpack_external_runtime )
      endif()

      string( REGEX REPLACE "(/${external_bin_path}/)" "/${external_bin_replace_path}/" other_binary ${binary} )
      if( EXISTS ${other_binary} )
        install( FILES ${other_binary}
                 DESTINATION External/${external_bin_replace_path}
                 COMPONENT H3DUtil_cpack_external_runtime )
      endif()
    endforeach()


    # setting names and dependencies between components and also grouping them.
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_RUNTIME_DISPLAY_NAME "External runtime" )
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_RUNTIME_DESCRIPTION "External runtime binaries needed by H3DUtil." )
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_RUNTIME_GROUP "H3DUtil_cpack_group" )
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_RUNTIME_INSTALL_TYPES Developer Full )

    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_SOURCE_DISPLAY_NAME "External header/libraries" )
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_SOURCE_DESCRIPTION "External headers and libraries needed by H3DUtil." )
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_SOURCE_DEPENDS H3DUtil_cpack_external_runtime )
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_SOURCE_GROUP "H3DUtil_cpack_group" )
    set( CPACK_COMPONENT_H3DUTIL_CPACK_EXTERNAL_SOURCE_INSTALL_TYPES Developer Full )
  endif()

  if( UNIX )
    set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${H3DUtil_SOURCE_DIR}/..;/" )
    set( CPACK_SOURCE_GENERATOR TGZ ZIP )
    set( CPACK_SOURCE_PACKAGE_FILE_NAME "h3dutil-${H3DUTIL_MAJOR_VERSION}.${H3DUTIL_MINOR_VERSION}.${H3DUTIL_BUILD_VERSION}" )


    set( H3DUTIL_CPACK_IGNORE_PATTERNS ${H3DUTIL_CPACK_IGNORE_PATTERNS}
            "/CVS/;/.svn/;/.bzr/;/.hg/;/.git.*/;.swp$;.#;/#;~$" )
    set( CPACK_SOURCE_IGNORE_FILES ${H3DUTIL_CPACK_IGNORE_PATTERNS} )
  endif()

  handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3DUtil_documentation_directory H3D_CMake_runtime_path
                                                OLD_VARIABLE_NAMES H3DUtil_DOCS_DIRECTORY H3D_cmake_runtime_path
                                                DOC_STRINGS "Set this to the directory containing the manual and generated doxygen documentation of H3DUtil."
                                                            "The path to the cmake runtime." )

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

  if( H3D_CMake_runtime_path )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} H3DUtil )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD}
                                                       COMMAND ${H3D_CMake_runtime_path}
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=H3DUtil_cpack_runtime -P cmake_install.cmake
                                                       COMMAND ${H3D_CMake_runtime_path}
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=H3DUtil_cpack_libraries -P cmake_install.cmake )
  else()
    message( STATUS "H3D_CMake_runtime_path is not set, please set it to continue" )
  endif()

  set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "H3DUtil. Help functions and utility functions for H3D API and HAPI." )
  set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
  set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  set( CPACK_RESOURCE_FILE_LICENSE "${H3DUtil_SOURCE_DIR}/../LICENSE" )
  set( CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};H3DUtil;ALL;/" )
  set( CPACK_PACKAGE_INSTALL_DIRECTORY "H3DUtil" )
  set( CPACK_PACKAGE_VERSION_MAJOR ${H3DUTIL_MAJOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_MINOR ${H3DUTIL_MINOR_VERSION} )
  set( CPACK_PACKAGE_VERSION_PATCH ${H3DUTIL_BUILD_VERSION} )
  set( DEBIAN_PACKAGE_DEPENDS "zlib1g-dev, libfreeimage-dev" )

  # Install header files
  install( FILES ${H3DUTIL_HEADERS}
           DESTINATION H3DUtil/include/H3DUtil
           COMPONENT H3DUtil_cpack_headers )

  # H3DUtil.cmake that goes to headers is not needed unless sources is required.
  install( FILES ${H3DUtil_SOURCE_DIR}/../include/H3DUtil/H3DUtil.cmake
      DESTINATION H3DUtil/include/H3DUtil
      COMPONENT H3DUtil_cpack_sources )

  # Install src files.
  install( FILES ${H3DUTIL_SRCS}
           DESTINATION H3DUtil/src
           COMPONENT H3DUtil_cpack_sources )

  install( FILES ${H3DUtil_SOURCE_DIR}/../changelog
                 ${H3DUtil_SOURCE_DIR}/../LICENSE
                 ${H3DUtil_SOURCE_DIR}/../README.md
           DESTINATION H3DUtil
           COMPONENT H3DUtil_cpack_sources )

  install( FILES ${H3DUtil_SOURCE_DIR}/CMakeLists.txt
                 ${H3DUtil_SOURCE_DIR}/H3DUtil.rc.cmake
                 ${H3DUtil_SOURCE_DIR}/H3DUtilSourceFiles.txt
                 ${H3DUtil_SOURCE_DIR}/UpdateResourceFile.exe
                 ${H3DUtil_SOURCE_DIR}/H3DUtilCPack.cmake
           DESTINATION H3DUtil/build
           COMPONENT H3DUtil_cpack_sources )

  install( FILES ${H3DUtil_SOURCE_DIR}/modules/FindBZip2.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindDCMTK.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindFreeImage.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindH3DBZip2.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindH3DTeem.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindH3DZLIB.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindMd5sum.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindOpenEXR.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindPTHREAD.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindSofaHelper.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindTeem.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/H3DCommonFindModuleFunctions.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/H3DCommonFunctions.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/H3DExternalSearchPath.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/H3DUtilityFunctions.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/InstallH3DUtilAndExternals.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/StripAndAddLibraryDirectories.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/UseDebian.cmake
                 ${H3DUtil_SOURCE_DIR}/modules/FindZLIB.cmake
           DESTINATION H3DUtil/build/modules
           COMPONENT H3DUtil_cpack_sources )

  # Add a cache variable H3DUtil_documentation_directory used to indicate where the H3DUtil docs are.
  if( NOT DEFINED H3DUtil_documentation_directory )
    set( H3DUtil_DOCS_DIRECTORY_DEFAULT "" )
    if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "H3DUtil" )
      set( H3DUtil_DOCS_DIRECTORY_DEFAULT "${H3DUtil_SOURCE_DIR}/../../doc" )
    elseif( TARGET HAPI )
      set( H3DUtil_DOCS_DIRECTORY_DEFAULT "${HAPI_documentation_directory}" )
    endif()
    set( H3DUtil_documentation_directory "${H3DUtil_DOCS_DIRECTORY_DEFAULT}" CACHE PATH "Set this to the directory containing the generated doxygen documentation of H3DUtil." )
    mark_as_advanced( H3DUtil_documentation_directory )
  endif()

  if( EXISTS ${H3DUtil_documentation_directory} )
    install( DIRECTORY ${H3DUtil_documentation_directory}/H3DUtil
             DESTINATION doc
             COMPONENT H3DUtil_cpack_headers
             REGEX "(/.svn)|(/CVS)" EXCLUDE )
  endif()

  # setting names and dependencies between components and also grouping them.
  set( CPACK_COMPONENT_H3DUTIL_CPACK_RUNTIME_DISPLAY_NAME "Runtime" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_RUNTIME_DESCRIPTION "The runtime libraries ( dlls ) for H3DUtil." )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_RUNTIME_DEPENDS H3DUtil_cpack_external_runtime )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_RUNTIME_GROUP "H3DUtil_cpack_group" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_RUNTIME_INSTALL_TYPES Developer Full )

  # Apparently circular dependencies are no problem to handle, so libraries depends on headers, and headers depends on libraries.
  set( CPACK_COMPONENT_H3DUTIL_CPACK_LIBRARIES_DISPLAY_NAME "Libraries" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_LIBRARIES_DESCRIPTION "H3DUtil libraries, needed for building against H3DUtil." )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_LIBRARIES_DEPENDS H3DUtil_cpack_external_source H3DUtil_cpack_headers )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_LIBRARIES_GROUP "H3DUtil_cpack_group" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_LIBRARIES_INSTALL_TYPES Developer Full )

  set( CPACK_COMPONENT_H3DUTIL_CPACK_HEADERS_DISPLAY_NAME "C++ Headers" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_HEADERS_DESCRIPTION "H3DUtil C++ headers, needed for building against H3DUtil." )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_HEADERS_DEPENDS H3DUtil_cpack_external_source H3DUtil_cpack_libraries )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_HEADERS_GROUP "H3DUtil_cpack_group" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_HEADERS_INSTALL_TYPES Developer Full )

  set( CPACK_COMPONENT_H3DUTIL_CPACK_SOURCES_DISPLAY_NAME "C++ Source" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_SOURCES_DESCRIPTION "Everything needed to build H3DUtil." )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_SOURCES_DEPENDS H3DUtil_cpack_headers )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_SOURCES_GROUP "H3DUtil_cpack_group" )
  set( CPACK_COMPONENT_H3DUTIL_CPACK_SOURCES_INSTALL_TYPES Full )

  set( CPACK_COMPONENT_GROUP_H3DUTIL_CPACK_GROUP_DISPLAY_NAME "H3DUtil" )
  set( CPACK_COMPONENT_GROUP_H3DUTIL_CPACK_GROUP_DESCRIPTION "Utility C++ library used by HAPI and H3DAPI." )

  if( ${CMAKE_PROJECT_NAME} STREQUAL "H3DUtil" )
    if( NOT TARGET HAPI )
      include( CPack )
      if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
        include( UseDebian )
        if( DEBIAN_FOUND )
          ADD_DEBIAN_TARGETS( H3DUtil )
        endif()
      endif()
    endif()
  endif()

endif()
