cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( H3DAPI )

# Set here already in case a special NSIS template needs to be used by cpack.
set( CMAKE_MODULE_PATH "${H3DAPI_SOURCE_DIR}/localModules" "${H3DAPI_SOURCE_DIR}/modules" )

include( H3DCommonFunctions )
setCommonH3DCMakePolicies()
setupRPathForLib()
enableCpp11( FAIL_MESSAGE "Enabling C++11 support. This compiler lacks such support." )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${H3DAPI_SOURCE_DIR}/.. GENERATE_H3D_PACKAGE_PROJECT H3D_PREFER_STATIC_LIBRARIES GENERATE_NodeRoutesToDotFile_BUILD )

# Make include files in paths like /usr/local/include have preference before
# files in OSX frameworks.
set( CMAKE_FIND_FRAMEWORK "LAST" )

# make sure symbols are exported.
set( H3DAPI_COMPILE_FLAGS "-DH3DAPI_EXPORTS" )
set( H3DAPI_LINK_FLAGS )
set( H3DAPI_LINK_FLAGS_RELEASE )
set( H3DAPI_LINK_FLAGS_DEBUG )
if( WIN32 AND NOT DEFINED PREFER_FREEGLUT_STATIC_LIBRARIES )
  # Add a cache variable PREFER_FREEGLUT_STATIC_LIBRARIES to have the choice of generating a project
  # linking statically against glut. Default is NO since most people will not use this.
  set( PREFER_FREEGLUT_STATIC_LIBRARIES "NO" CACHE BOOL "Decides if CMake should prefer static libraries to dynamic libraries for FreeGLUT when both exist." )
  mark_as_advanced( PREFER_FREEGLUT_STATIC_LIBRARIES )
endif()

# Include HAPI in the build.
if( WIN32 AND EXISTS ${H3DAPI_SOURCE_DIR}/../../HAPI/build )
  message( STATUS "Including HAPI" )
  add_subdirectory( ${H3DAPI_SOURCE_DIR}/../../HAPI/build
                    ${CMAKE_CURRENT_BINARY_DIR}/HAPI )

  # Check if Haptik was found.
  if( Haptik_INCLUDE_DIR AND Haptik_LIBRARY )
    set( Haptik_FOUND 1 )
  endif()
endif()

# Add the directory to include_directories before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include ${H3DAPI_SOURCE_DIR}/../include )

# Version of H3D API.
set( H3DAPI_MAJOR_VERSION 2 )
set( H3DAPI_MINOR_VERSION 4 )
set( H3DAPI_BUILD_VERSION 1 )


# Set here already in case a special NSIS template needs to be used by cpack.
set( CMAKE_MODULE_PATH "${H3DAPI_SOURCE_DIR}/localModules" "${H3DAPI_SOURCE_DIR}/modules" )
include( StripAndAddLibraryDirectories )
include( H3DCommonFunctions ) # Include again to get the correct version of the included file.

# Version of project.
set( H3DAPI_FULL_VERSION
${H3DAPI_MAJOR_VERSION}.${H3DAPI_MINOR_VERSION}.${H3DAPI_BUILD_VERSION} )

# Add all sources, they are added to a variable called H3DAPI_SRCS
# defined in the included file. All header files are added to a variable called
# H3DAPI_HEADERS.
include( ${H3DAPI_SOURCE_DIR}/H3DAPISourceFiles.txt )
handleUnityBuild( PROJECT_NAME ${PROJECT_NAME}
                  SOURCE_FILES ${NON_UNITYBUILD_FILES}
                  OUTPUT_VARIABLE H3DAPI_SRCS
                  SOURCE_PREFIX_PATH ${H3DAPI_SOURCE_DIR}/../src/
                  COMPILE_FLAGS_VARIABLE H3DAPI_COMPILE_FLAGS
                  UNITY_BUILD_SOURCES ${H3DAPI_SRCS_UNITYBUILD} )
handlePrecompiledHeaders( PROJECT_NAME ${PROJECT_NAME}
                          HEADERS_VARIABLE H3DAPI_HEADERS
                          SRCS_VARIABLE H3DAPI_SRCS
                          STDAFX_HEADER_LOCATION ${H3DAPI_SOURCE_DIR}/../include/H3D/
                          STDAFX_SOURCE_LOCATION ${H3DAPI_SOURCE_DIR}/../src/ )

# Add the H3DAPI.rc resource file if Visual Studio
if( MSVC )
  set( H3DAPI_SRCS ${H3DAPI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/H3DAPI.rc )
endif()

# Add all optional libraries to this variable
set( optional_libs )

# Add all required libraries to this variable
set( required_libs )

set( vars_to_negate_after_creation )
foreach( tmp_var USE_DisplayList USE_field_access_type_check USE_Prototype_node USE_DisplayList_event_collect )
  if( NOT DEFINED ${tmp_var} )
    list( APPEND vars_to_negate_after_creation ${tmp_var} )
  endif()
endforeach()
handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES USE_DisplayList         USE_field_access_type_check     USE_Prototype_node    USE_DisplayList_event_collect       USE_H3D_application_manage_active_texture_unit
                                              OLD_VARIABLE_NAMES DISABLE_H3D_DISPLAYLIST DISABLE_FIELD_ACCESS_TYPE_CHECK DISABLE_PROTOTYPENODE DEPRECATE_DISPLAYLIST_EVENT_COLLECT USE_APPLICATION_MANAGED_ACTIVE_TEXTURE_UNIT
                                              DOC_STRINGS "Build with DisplayLists feature in H3D."
                                                          "Check field access types. Can be disabled in production releases to get better performance."
                                                          "Build with X3D proto node features. Disable for optimization if an application does not use this feature."
                                                          "Use event collection for H3DDisplayListObjects. Disable to save CPU time. Note: displayList fields hasCausedEvent will no longer report correctly if disabled."
                                                          "Let H3DAPI manage active texture value, instead of relying on opengl get function. H3DAPI rely on setTextureUnit and getTextureUnit to simply manage it." )
if( vars_to_negate_after_creation )
  foreach( var_to_negate ${vars_to_negate_after_creation} )
    if( DEFINED ${var_to_negate} ) # The function handleRenamingVariablesBackwardCompatibility initialized it based on the old variable name, that one should be negated.
      get_property( var_type CACHE ${var_to_negate} PROPERTY TYPE )
      get_property( var_doc CACHE ${var_to_negate} PROPERTY HELPSTRING )
      if( ${var_to_negate} )
        set( ${var_to_negate} OFF CACHE ${var_type} ${var_doc} FORCE )
      else()
        set( ${var_to_negate} ON CACHE ${var_type} ${var_doc} FORCE )
      endif()
    endif()
  endforeach()
endif()

if( NOT DEFINED USE_DisplayList )
  set( USE_DisplayList ON CACHE BOOL "Build with DisplayLists feature in H3D." )
  mark_as_advanced( USE_DisplayList )
endif()
set( DISABLE_H3D_DISPLAYLIST ON )  # Have to set this for H3DApi.h to be generated properly
if( USE_DisplayList )
  set( DISABLE_H3D_DISPLAYLIST OFF )
endif()

if( NOT DEFINED USE_field_access_type_check )
  set( USE_field_access_type_check ON CACHE BOOL "Check field access types. Can be disabled in production releases to get better performance." )
  mark_as_advanced( USE_field_access_type_check )
endif()
set( DISABLE_FIELD_ACCESS_TYPE_CHECK ON )  # Have to set this for H3DApi.h to be generated properly
if( USE_field_access_type_check )
  set( DISABLE_FIELD_ACCESS_TYPE_CHECK OFF )
endif()

if( NOT DEFINED USE_field_thread_check_debug )
  set( USE_field_thread_check_debug OFF CACHE BOOL "Cause an assertion if fields are accessed outside the main thread (i.e. in debug builds only)." )
  mark_as_advanced( USE_field_thread_check_debug )
endif()
set( H3D_FIELD_THREAD_CHECK_DEBUG ${USE_field_thread_check_debug} )

if( NOT DEFINED USE_field_thread_check_release )
  set( USE_field_thread_check_release OFF CACHE BOOL "Raise an exception if fields are accessed outside the main thread (including non-debug builds)." )
  mark_as_advanced( USE_field_thread_check_release )
endif()
set( H3D_FIELD_THREAD_CHECK_RELEASE ${USE_field_thread_check_release} )

if( NOT DEFINED USE_Prototype_node )
  set( USE_Prototype_node ON CACHE BOOL "Build with X3D proto node features. Disable for optimization if an application does not use this feature." )
  mark_as_advanced( USE_Prototype_node )
endif()
set( DISABLE_PROTOTYPENODE ON )  # Have to set this for H3DApi.h to be generated properly
if( USE_Prototype_node )
  set( DISABLE_PROTOTYPENODE OFF )
endif()

if( NOT DEFINED USE_DisplayList_event_collect )
  set( USE_DisplayList_event_collect ON CACHE BOOL "Use event collection for H3DDisplayListObjects. Disable to save CPU time. Note: displayList fields hasCausedEvent will no longer report correctly if disabled." )
  mark_as_advanced( USE_DisplayList_event_collect )
endif()
set( DEPRECATE_DISPLAYLIST_EVENT_COLLECT ON ) # Have to set this for H3DApi.h to be generated properly
if( USE_DisplayList_event_collect )
  set( DEPRECATE_DISPLAYLIST_EVENT_COLLECT OFF )
endif()

if( NOT DEFINED USE_H3D_application_manage_active_texture_unit )
  set( USE_H3D_application_manage_active_texture_unit "OFF" CACHE BOOL "Let H3DAPI manage active texture value, instead of relying on opengl get function. H3DAPI rely on setTextureUnit and getTextureUnit to simply manage it." )
  mark_as_advanced( USE_H3D_application_manage_active_texture_unit )
endif()
set( USE_APPLICATION_MANAGED_ACTIVE_TEXTURE_UNIT ${USE_H3D_application_manage_active_texture_unit} )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects( PROJECT_NAMES HAPI
                                           INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                           LIBRARIES_OUTPUT_VAR required_libs
                                           REQUIRED_PROJECTS HAPI
                                           REQUIRED_PROJECTS_COMPONENTS HAPI_SameComponentsAsInstalledLibrary )
include_directories( ${tmp_h3d_include_dirs} )

if( TARGET Chai3DRenderer )
  set( required_libs ${required_libs} Chai3DRenderer )
  if( MSVC80 )
    set( required_libs ${required_libs} optimized atls.lib debug atlsd.lib )
  endif()
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} Chai3DRenderer )
endif()

if( TARGET OpenHapticsRenderer )
  set( required_libs ${required_libs} OpenHapticsRenderer )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} OpenHapticsRenderer )
endif()

# Extra libs to link against.
if( WIN32 )
  set( required_libs ${required_libs} winmm.lib DelayImp.lib )
endif()

# Find OpenGL library. Required to compile.
find_package( OpenGL REQUIRED )
if( OPENGL_FOUND )
  set( HAVE_OPENGL 1 )
  include_directories( ${OPENGL_INCLUDE_DIR} )
  set( required_libs ${required_libs} ${OPENGL_LIBRARIES} )
endif()

# Find Glew library. Required to compile.
find_package( GLEW REQUIRED )
if( GLEW_FOUND )
  include_directories( ${GLEW_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${GLEW_LIBRARIES} )
endif()

# Optional extra library. Makes is possible to use GLUT windows handling with H3D API.
find_package( GLUT )

if( GLUT_FOUND )
  set( HAVE_GLUT 1 )
  include_directories( ${GLUT_INCLUDE_DIR} )
  if( WIN32 )
    if( MINGW )
      set( required_libs ${required_libs} ${GLUT_LIBRARIES} )
    else()
      # This is used in order for the compiler to find freeglut.lib.
      # If freeglut.lib is in any other place than glut32 on your system
      # either this CMakeLists.txt or FindGLUT needs to be changed
      # to allow for choice of extra library directory.
      stripAndAddLibraryDirectories( ${GLUT_LIBRARIES} )
    endif()
  else()
    set( required_libs ${required_libs} ${GLUT_LIBRARIES} )
  endif()
endif()

# Optional extra library. Without this H3D API can not load scenes defined with X3D syntax.
find_package( XercesC )
if( XercesC_FOUND )
  set( HAVE_XERCES 1 )
  include_directories( ${XercesC_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${XercesC_LIBRARIES} )
endif()

# Optional extra library. Used in H3D API to resolved urls.
find_package( CURL )
if( CURL_FOUND )
  set( HAVE_LIBCURL 1 )
  include_directories( ${CURL_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${CURL_LIBRARIES} )
endif()

# Optional extra library. Needed to support shading using the CG shader language from NVidia.
find_package( NvidiaCG )
if( NvidiaCG_FOUND )
  set( HAVE_CG 1 )
  include_directories( ${NvidiaCG_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${NvidiaCG_LIBRARIES} )
  addDelayLoadFlags( NvidiaCG_LIBRARIES H3DAPI_LINK_FLAGS )
endif()

# Optional extra library. Used to read compressed files.
find_package( ZLIB )
if( ZLIB_FOUND )
  include_directories( ${ZLIB_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${ZLIB_LIBRARIES} )
endif()

# Optional extra library. Add python bindings to H3D API, allow for extended prototyping without using C++.
findPython2Or3( python_include_dirs python_libs have_python_debug_library have_python_os_framework python_definitions )
add_definitions( ${python_definitions} )
if( python_include_dirs AND python_libs )
  set( HAVE_PYTHON 1 )
  set( HAVE_PYTHON_OSX_FRAMEWORK ${have_python_os_framework} )
  set( HAVE_PYTHON_DEBUG_LIBRARY ${have_python_debug_library} )
  include_directories( ${python_include_dirs} )
  set( optional_libs ${optional_libs} ${python_libs} )
endif()

# Optional extra library. Used for sound in H3D API.
find_package( OpenAL )
if( OPENAL_FOUND )
  set( HAVE_OPENAL 1 )
  include_directories( ${OPENAL_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${OPENAL_LIBRARIES} )
endif()

# Optional extra library. Used to read sound files.
find_package( Vorbis )
if( Vorbis_FOUND )
  set( HAVE_LIBVORBIS 1 )
  include_directories( ${Vorbis_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${Vorbis_LIBRARIES} )
  addDelayLoadFlags( Vorbis_vorbisfile_LIBRARY H3DAPI_LINK_FLAGS )
endif()

# Optional extra library. Used for Oculus Rift support.
find_package( LibOVR )
if( LibOVR_FOUND )
  set( HAVE_LIBOVR 1 )
  include_directories( ${LibOVR_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${LibOVR_LIBRARIES} )
endif()

# Optional extra library. Used to read sound files.
find_package( Audiofile )
if( Audiofile_FOUND )
  set( HAVE_LIBAUDIOFILE 1 )
  include_directories( ${Audiofile_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${Audiofile_LIBRARIES} )
endif()

# Optional extra library. Used to handle fonts in H3D API.
find_package( Freetype )
if( FREETYPE_FOUND )
  set( HAVE_FREETYPE 1 )
  include_directories( ${FREETYPE_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${FREETYPE_LIBRARIES} )
endif()

# Optional extra library. Adds support to use devices by 3dxware. See www.3dconnexion.com.
find_package( 3DXWARE )
if( 3DXWARE_FOUND )
  set( HAVE_3DXWARE 1 )
  include_directories( ${3DXWARE_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${3DXWARE_LIBRARIES} )
endif()

# Optional extra library. Used for RazerHydraSensor
find_package( SixenseSDK )
if( SixenseSDK_FOUND )
  set( HAVE_SIXENSE 1 )
  include_directories( ${SixenseSDK_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${SixenseSDK_LIBRARIES} )
  addDelayLoadFlags( SixenseSDK_LIBRARIES H3DAPI_LINK_FLAGS )
endif()

# Optional extra library. Used to render fonts in OpenGL.
find_package( FTGL )
if( FTGL_FOUND )
  set( HAVE_FTGL 1 )
  set( FTGL_INCLUDE_IS_UPPER ${FTGL_INCLUDE_IS_UPPER} )
  include_directories( ${FTGL_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${FTGL_LIBRARIES} )
endif()

# Optional extra library. Used to read images.
find_package( FreeImage )
if( FreeImage_FOUND )
  include_directories( ${FreeImage_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${FreeImage_LIBRARIES} )
endif()

if( Haptik_FOUND )
  set( optional_libs ${optional_libs} ${Haptik_LIBRARY} )
  addDelayLoadFlags( Haptik_LIBRARY H3DAPI_LINK_FLAGS )
endif()

if( WIN32 )
  # Windows specific libraries

  # Optional extra library. Used to render movies as textures.
  find_package( DirectShow )
  if( DirectShow_FOUND )
    set( HAVE_DSHOW 1 )
    include_directories( ${DirectShow_INCLUDE_DIRS} )
    set( optional_libs ${optional_libs} ${DirectShow_LIBRARIES} )
  endif()
endif()

if( NOT WIN32 )
  # Linux specific libraries

  find_package( FontConfig )
  if( FontConfig_FOUND )
    set( HAVE_FONTCONFIG 1 )
    include_directories( ${FontConfig_INCLUDE_DIRS} )
    set( optional_libs ${optional_libs} ${FontConfig_LIBRARIES} )
  endif()

  if( NOT APPLE )
    # Optional extra library. Used to render movies as textures.
    find_package( FFmpeg )
    if( FFmpeg_FOUND )
      set( HAVE_FFMPEG 1 )
      include_directories( ${FFmpeg_INCLUDE_DIRS} )
      list( APPEND optional_libs ${FFmpeg_LIBRARIES} )
    endif()
  endif()
endif()

find_package( DCMTK )
if( DCMTK_FOUND )
  include_directories( ${DCMTK_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${DCMTK_LIBRARIES} )
endif()

find_package( Teem )
if( Teem_FOUND )
  include_directories( ${Teem_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${Teem_LIBRARIES} )
endif()

# Optional extra library. Used to handle OpenEXR HDR image formats.
find_package( OpenEXR )
if( OpenEXR_FOUND )
  set( HAVE_OPENEXR 1 )
  include_directories( ${OpenEXR_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${OpenEXR_LIBRARIES} )
endif()

# Optional extra library. Used to support NvidiaGraphicsOptions node
set( USE_NVAPI NO CACHE BOOL "Enable NVIDIA NVAPI." )
if( USE_NVAPI )
  find_package(NVAPI)
  if (NVAPI_FOUND)
    set(HAVE_NVAPI 1)
    include_directories( ${NVAPI_INCLUDE_DIRS} )
    set( optional_libs ${optional_libs} ${NVAPI_LIBRARIES} )
  endif()
endif()

# We do not use V8 anymore, only SpiderMonkey
#find_package( V8 )
#if( V8_FOUND )
#  set( HAVE_V8 1 )
#  include_directories( ${V8_INCLUDE_DIRS} )
#  set( required_libs ${required_libs} ${V8_LIBRARIES} )
#endif()

find_package( SpiderMonkey )
if( SpiderMonkey_FOUND )
  set( HAVE_SPIDERMONKEY 1 )
  include_directories( ${SpiderMonkey_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${SpiderMonkey_LIBRARIES} )
endif()

find_package( VirtualHand )
if( VirtualHand_FOUND )
  set( HAVE_VIRTUAL_HAND_SDK 1 )
  include_directories( ${VirtualHand_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${VirtualHand_LIBRARIES} )
  addDelayLoadFlags( VirtualHand_LIBRARIES H3DAPI_LINK_FLAGS )
endif()

populateProjectIncludeDirectoriesCacheVar( VARIABLE_NAME H3DAPI_INCLUDE_DIRS
                                           DEPRECATED_VARIABLE_NAMES H3DAPI_INCLUDE_DIR )

# H3DAPI shared library definition
add_library( H3DAPI SHARED ${H3DAPI_SRCS} ${H3DAPI_HEADERS} )

# make sure that the H3DAPI.rc contains the correct svn-version
setupResourceFile( H3DAPI
                   VERSION_PREFIX H3DAPI
                   SVN_DIR_CANDIDATE ${H3DAPI_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${H3DAPI_SOURCE_DIR}/H3DAPI.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/H3DAPI.rc
                   UPDATERESOURCEFILE_EXE ${H3DAPI_SOURCE_DIR}/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "H3DAPI_Output_Name" "$(TargetFileName)" )

# add the libraries needed for linking
target_link_libraries( H3DAPI ${required_libs} ${optional_libs} )

handleCommonCacheVar( USE_thread_lock_debug FALSE )

# make the name of debug libraries end in _d.
set_target_properties( H3DAPI PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
set_target_properties( H3DAPI PROPERTIES VERSION ${H3DAPI_FULL_VERSION} )

if( APPLE )
  set_target_properties( H3DAPI
                         PROPERTIES
#                        BUILD_WITH_INSTALL_RPATH 1
                         INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib" )
endif()

if( UNIX )
  set_target_properties( H3DAPI PROPERTIES OUTPUT_NAME h3dapi )
endif()

if( MSVC )
  set_target_properties( H3DAPI PROPERTIES FOLDER "H3D" )
endif()

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

if( WIN32 )
  # Temporary variables used later to set LINK_FLAGS.
  set( temp_output_postfix "" )

  # if we are using 64 bit compiler some object files get too big so we need
  # to add the /bigobj flag
  if( CMAKE_CL_64 )
    set( H3DAPI_COMPILE_FLAGS "${H3DAPI_COMPILE_FLAGS} /bigobj" )
  endif()

  if( MSVC )
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other.
    setH3DMSVCOutputName( H3DAPI "H3DAPI" temp_output_postfix )
    # Add common H3D compile flags for microsoft visual studio
    addCommonH3DMSVCCompileFlags( H3DAPI_COMPILE_FLAGS )

    # H3D API is now to big to link incrementally with vc8.
    set( H3DAPI_LINK_FLAGS "${H3DAPI_LINK_FLAGS} /INCREMENTAL:NO" )
    set( H3DAPI_LINK_FLAGS_DEBUG "${H3DAPI_LINK_FLAGS_DEBUG} /NODEFAULTLIB:msvcrt" )
  endif()

  if( TARGET Chai3DRenderer AND MSVC )
    set( H3DAPI_LINK_FLAGS_DEBUG "${H3DAPI_LINK_FLAGS_DEBUG} /DELAYLOAD:\"Chai3DRenderer${temp_output_postfix}_d.dll\"" )
    set( H3DAPI_LINK_FLAGS_RELEASE "${H3DAPI_LINK_FLAGS_RELEASE} /DELAYLOAD:\"Chai3DRenderer${temp_output_postfix}.dll\"" )
  endif()

  if( TARGET OpenHapticsRenderer AND MSVC )
    set( H3DAPI_LINK_FLAGS_DEBUG "${H3DAPI_LINK_FLAGS_DEBUG} /DELAYLOAD:\"OpenHapticsRenderer${temp_output_postfix}_d.dll\"" )
    set( H3DAPI_LINK_FLAGS_RELEASE "${H3DAPI_LINK_FLAGS_RELEASE} /DELAYLOAD:\"OpenHapticsRenderer${temp_output_postfix}.dll\"" )
  endif()


endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  addCommonH3DGNUCompileFlags( H3DAPI_COMPILE_FLAGS TRUE )
endif()
addCommonAppleClangCompileFlags( H3DAPI_COMPILE_FLAGS )

if( GENERATE_NodeRoutesToDotFile_BUILD )
  set( H3DAPI_COMPILE_FLAGS "${H3DAPI_COMPILE_FLAGS} -DH3D_GENERATE_DOTROUTE_FILES" )
endif()

# Set compile and linker flags for H3DAPI.
  set_target_properties( H3DAPI
                         PROPERTIES
                         COMPILE_FLAGS "${H3DAPI_COMPILE_FLAGS}"
                         LINK_FLAGS "${H3DAPI_LINK_FLAGS}"
                         LINK_FLAGS_DEBUG "${H3DAPI_LINK_FLAGS_DEBUG}"
                         LINK_FLAGS_RELEASE "${H3DAPI_LINK_FLAGS_RELEASE}"
                         LINK_FLAGS_RELWITHDEBINFO "${H3DAPI_LINK_FLAGS_RELEASE}"
                         LINK_FLAGS_MINSIZEREL "${H3DAPI_LINK_FLAGS_RELEASE}" )


# autogenerate H3DAPI.h depending on the libraries available.
if( EXISTS ${H3DAPI_SOURCE_DIR}/../include/H3D/H3DApi.h )
  file( REMOVE ${H3DAPI_SOURCE_DIR}/../include/H3D/H3DApi.h )
endif()
configure_file( ${H3DAPI_SOURCE_DIR}/../include/H3D/H3DApi.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/H3DApi.h )

if( NOT GENERATE_H3D_PACKAGE_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/H3DApi.h
           DESTINATION ${H3DAPI_SOURCE_DIR}/../include/H3D )
endif()

# Intall directories.
install( TARGETS H3DAPI
         LIBRARY DESTINATION ${default_lib_install} COMPONENT H3DAPI_cpack_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT H3DAPI_cpack_runtime
         ARCHIVE DESTINATION ${default_lib_install} COMPONENT H3DAPI_cpack_libraries )

if( NOT ( WIN32 OR GENERATE_H3D_PACKAGE_PROJECT ) )
  # Install include files.
  install( FILES ${H3DAPI_HEADERS}
           DESTINATION include/H3D )
endif()

handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES GENERATE_H3DAPI_loader_PROJECTS GENERATE_H3DLoad_PROJECT GENERATE_H3DAPI_example_PROJECTS
                                              OLD_VARIABLE_NAMES H3DAPI_LOADER_PROJECTS H3DAPI_H3DLOAD_PROJECTS H3DAPI_EXAMPLE_PROJECTS
                                              DOC_STRINGS "If set to YES H3DLoad and H3DViewer might be included in the build."
                                                          "If set to YES H3DLoad will be included in the build."
                                                          "If set to YES various example projects build on H3DAPI will be included in the build." )

if( NOT DEFINED GENERATE_H3DAPI_loader_PROJECTS )
  set( GENERATE_H3DAPI_loader_PROJECTS YES CACHE BOOL "If set to YES H3DLoad and H3DViewer might be included in the build." )
endif()

if( NOT DEFINED GENERATE_H3DLoad_PROJECT )
  set( GENERATE_H3DLoad_PROJECT YES CACHE BOOL "If set to YES H3DLoad will be included in the build." )
endif()

if( NOT DEFINED GENERATE_H3DViewer_PROJECT )
  set( GENERATE_H3DViewer_PROJECT YES CACHE BOOL "If set to YES H3DViewer will be included in the build." )
endif()

if( GENERATE_H3DAPI_loader_PROJECTS )
  if( GENERATE_H3DLoad_PROJECT )
    if( GLUT_FOUND )
      message( STATUS "Including H3DLoad" )
      add_subdirectory( ${H3DAPI_SOURCE_DIR}/../H3DLoad/build
                        ${CMAKE_CURRENT_BINARY_DIR}/H3DLoad )
      set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} H3DLoad )
    else()
      message( STATUS "GLUT was not found. H3DLoad will not be generated." )
    endif()
  endif()

  if( GENERATE_H3DViewer_PROJECT )
    set( WXWINDOWS_USE_GL 1 )
    find_package( wxWidgets COMPONENTS base core )

    if( wxWidgets_FOUND )
      message( STATUS "Including H3DViewer" )
      add_subdirectory( ${H3DAPI_SOURCE_DIR}/../H3DViewer/build
                        ${CMAKE_CURRENT_BINARY_DIR}/H3DViewer )
      set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} H3DViewer )
    else()
      message( WARNING "wxWidgets was not found. H3DViewer will not be generated." )
    endif()
  endif()
endif()

if( NOT DEFINED GENERATE_H3DAPI_example_PROJECTS )
  set( GENERATE_H3DAPI_example_PROJECTS "NO" CACHE BOOL "If set to YES various example projects build on H3DAPI will be included in the build." )
endif()

if( GENERATE_H3DAPI_example_PROJECTS )
  if( GLUT_FOUND )
    message( STATUS "Including H3DAPI manual examples" )
    add_subdirectory( ${H3DAPI_SOURCE_DIR}/../examples/manualExamples/C++
                      ${CMAKE_CURRENT_BINARY_DIR}/examples/manualExamples )
  endif()

  message( STATUS "Including H3DAPI ThreadExample" )
  add_subdirectory( ${H3DAPI_SOURCE_DIR}/../examples/ThreadExample
                    ${CMAKE_CURRENT_BINARY_DIR}/examples/ThreadExample )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} ThreadExample Sphere_X3D Spheres_X3D )
endif()

if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "H3DAPI" )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PARENT_SCOPE )
else()
  include( ${H3DAPI_SOURCE_DIR}/H3DAPICPack.cmake )
endif()
