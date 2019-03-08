cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( H3DUtil )

set( CMAKE_MODULE_PATH "${H3DUtil_SOURCE_DIR}/modules" )

include( H3DCommonFunctions )
setCommonH3DCMakePolicies()
setupRPathForLib()

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include ${H3DUtil_SOURCE_DIR}/../include )

handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${H3DUtil_SOURCE_DIR}/.. GENERATE_H3D_PACKAGE_PROJECT H3D_PREFER_STATIC_LIBRARIES )

set( H3DUTIL_MAJOR_VERSION 1 )
set( H3DUTIL_MINOR_VERSION 4 )
set( H3DUTIL_BUILD_VERSION 1 )

set( H3DUTIL_FULL_VERSION
${H3DUTIL_MAJOR_VERSION}.${H3DUTIL_MINOR_VERSION}.${H3DUTIL_BUILD_VERSION} ) 

# Add all sources, they are added to a variable called H3DUTIL_SRCS defined
# in the included file. All header files are added to a variable called
# H3DUTIL_HEADERS.
include( ${H3DUtil_SOURCE_DIR}/H3DUtilSourceFiles.txt )

# add the H3DUtil.rc resource file if Visual Studio
if( MSVC )
  set( H3DUTIL_SRCS ${H3DUTIL_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/H3DUtil.rc )
endif()

# add all optional libraries to this variable
set( optional_libs )

# add all required libraries to this variable
set( required_libs )

handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES USE_profiler USE_memory_leak_detector USE_NvidiaToolsExt
                                              OLD_VARIABLE_NAMES ENABLE_PROFILER ENABLE_MEMORY_LEAK_DETECTOR USE_NVIDIA_TOOLSEXT
                                              DOC_STRINGS "When enabled parts of the code will be profiled and the info can be obtained at run time. Requires SofaHelper library."
                                                          "Enable memory leak detector. Currently Visual Leak Detector library is required for this to work."
                                                          "Enable NVIDIA Profiler Tools Extension." )

set( USE_profiler "OFF" CACHE BOOL "When enabled parts of the code will be profiled and the info can be obtained at run time. Requires SofaHelper library." )
if( USE_profiler )
  find_package( SofaHelper )
  if( SofaHelper_FOUND )
    set( HAVE_PROFILER 1 )
    include_directories( ${SofaHelper_INCLUDE_DIRS} )
    set( optional_libs ${optional_libs} ${SofaHelper_LIBRARIES} )
  elseif( SofaHelper_FOUND )
    message( WARNING "No sofa helper library is found, profiler can not be enabled" )
  endif()
endif()

# Optional extra library. Needed to support memory leak detector
set( USE_memory_leak_detector "OFF" CACHE BOOL "Enable memory leak detector. Currently Visual Leak Detector library is required for this to work." )
if( USE_memory_leak_detector )
    message( WARNING "Visual leak detector can occasionally crash when using OpenHapticsRenderer. It always crashes when using Visual Studio 2017. Using a different renderer will work fine.")
    find_package( VLD )
    if( VLD_FOUND )
        set( HAVE_LIBVLD 1 )
        include_directories( ${VLD_INCLUDE_DIRS} )
        set( optional_libs ${optional_libs} ${VLD_LIBRARIES} )
    elseif( VLD_FOUND )
        message( WARNING "No visual leak detector found, memory leak detection can not be enabled" )
    endif()
endif()

find_package( FreeImage )
if( FreeImage_FOUND )
  set( HAVE_FREEIMAGE 1 )
  include_directories( ${FreeImage_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${FreeImage_LIBRARIES} )
endif()

find_package( ZLIB )
if( ZLIB_FOUND )
  set( HAVE_ZLIB 1 )
  include_directories( ${ZLIB_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${ZLIB_LIBRARIES} )
endif()

find_package( DCMTK )
if( DCMTK_FOUND )
  set( HAVE_DCMTK 1 )
  include_directories( ${DCMTK_INCLUDE_DIRS} ) 
  set( optional_libs ${optional_libs} ${DCMTK_LIBRARIES} )
endif()

find_package( Teem )
if( Teem_FOUND )
  set( HAVE_TEEM 1 )
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

find_package( PTHREAD REQUIRED )
if( PTHREAD_FOUND )
  include_directories( ${PTHREAD_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${PTHREAD_LIBRARIES} )
endif()

set( USE_NvidiaToolsExt NO CACHE BOOL "Enable NVIDIA Profiler Tools Extension." )
if( USE_NvidiaToolsExt )
  find_package( NvidiaToolsExt )
  if( NvidiaToolsExt_FOUND )
    include_directories( ${NvidiaToolsExt_INCLUDE_DIRS} )
    set( optional_libs ${optional_libs} ${NvidiaToolsExt_LIBRARIES} )
    set( HAVE_NVIDIATX 1 )
  endif()
endif()

set( USE_H3DProfiler NO CACHE BOOL "Include the H3DProfiler library." )
if( USE_H3DProfiler )
  find_package( H3DProfiler )
  if( H3DProfiler_FOUND )
    set( HAVE_H3DPROFILER 1 )
    include_directories( ${H3DProfiler_INCLUDE_DIRS} )
  endif()
endif()

# If HAVE_H3DPROFILER is defined but we have not found H3DProfiler then assume
# that we have set it up in a metarepo setup
if( HAVE_H3DPROFILER )
  if( NOT H3DProfiler_FOUND )
    set( optional_libs ${optional_libs} H3DProfiler )
  else()
    set( optional_libs ${optional_libs} ${H3DProfiler_LIBRARIES} )
  endif()
endif()  

populateProjectIncludeDirectoriesCacheVar( VARIABLE_NAME H3DUtil_INCLUDE_DIRS
                                           DEPRECATED_VARIABLE_NAMES H3DUTIL_INCLUDE_DIR )

# H3DUtil shared library definition.
add_library( H3DUtil SHARED ${H3DUTIL_SRCS} ${H3DUTIL_HEADERS} )

# make sure that the H3DUtil.rc contains the correct svn-version
setupResourceFile( H3DUtil
                   VERSION_PREFIX H3DUTIL
                   SVN_DIR_CANDIDATE ${H3DUtil_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${H3DUtil_SOURCE_DIR}/H3DUtil.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/H3DUtil.rc
                   UPDATERESOURCEFILE_EXE ${H3DUtil_SOURCE_DIR}/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "H3DUTIL_Output_Name" "$(TargetFileName)" )

# check all available libraries

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

if( WIN32 )
  set( required_libs ${required_libs} winmm.lib )
endif()

# add the libraries needed for linking
target_link_libraries( H3DUtil ${required_libs} ${optional_libs} )

# make sure symbols are exported.
set( H3DUTIL_COMPILE_FLAGS "-DH3DUTIL_EXPORTS" )

handleCommonCacheVar( USE_thread_lock_debug TRUE ) # H3DUtil target have to exist, so wait until here.

# make the name of debug libraries end in _d.
set_target_properties( H3DUtil PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
set_target_properties( H3DUtil PROPERTIES VERSION ${H3DUTIL_FULL_VERSION} )


if( APPLE )
  set_target_properties( H3DUtil
               PROPERTIES 
#       BUILD_WITH_INSTALL_RPATH 1
                         INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib" )
endif()

if( UNIX )
  set_target_properties( H3DUtil PROPERTIES OUTPUT_NAME h3dutil )
endif()

if( MSVC )
  include( H3DCommonFunctions )
  # change the name depending on compiler to be able to tell them apart
  # since they are not compatible with each other. 
  setH3DMSVCOutputName( H3DUtil "H3DUtil" )
  # Add common H3D compile flags for microsoft visual studio
  addCommonH3DMSVCCompileFlags( H3DUTIL_COMPILE_FLAGS )
  
  # H3DUtil can not link incrementally on vc8 for some reason. We shut of incremental linking for
  # all visual studio versions.
  set_target_properties( H3DUtil PROPERTIES LINK_FLAGS_DEBUG " /NODEFAULTLIB:msvcrt /INCREMENTAL:NO" )

elseif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
  if( NOT MINGW )
    #used to link in dynamic library load functions. Needed by some examples in HAPI
    set_target_properties( H3DUtil PROPERTIES LINK_FLAGS "-ldl" )
  endif()
  
  if( CMAKE_BUILD_TYPE MATCHES Debug )
    set( H3D_DEBUG ON )
  endif()

  addCommonH3DGNUCompileFlags( H3DUTIL_COMPILE_FLAGS )
  
endif()

# set compile flags.
set_target_properties( H3DUtil PROPERTIES COMPILE_FLAGS "${H3DUTIL_COMPILE_FLAGS}" )

if( MSVC )
  set_target_properties( H3DUtil PROPERTIES FOLDER "H3D" )
endif()

# autogenerate H3DUtil.h depending on the libraries available.
if( EXISTS ${H3DUtil_SOURCE_DIR}/../include/H3DUtil/H3DUtil.h )
  file( REMOVE ${H3DUtil_SOURCE_DIR}/../include/H3DUtil/H3DUtil.h )
endif()
configure_file( ${H3DUtil_SOURCE_DIR}/../include/H3DUtil/H3DUtil.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/H3DUtil/H3DUtil.h )

if( NOT GENERATE_H3D_PACKAGE_PROJECT )
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/H3DUtil/H3DUtil.h
           DESTINATION ${H3DUtil_SOURCE_DIR}/../include/H3DUtil )
endif()

install( TARGETS H3DUtil 
         LIBRARY DESTINATION ${default_lib_install} COMPONENT H3DUtil_cpack_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT H3DUtil_cpack_runtime
         ARCHIVE DESTINATION ${default_lib_install} COMPONENT H3DUtil_cpack_libraries )

if( NOT ( WIN32 OR GENERATE_H3D_PACKAGE_PROJECT ) )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${H3DUTIL_HEADERS}
           DESTINATION include/H3DUtil )
endif()

if( NOT HAPI_SOURCE_DIR )
  include( ${H3DUtil_SOURCE_DIR}/H3DUtilCPack.cmake )
endif()