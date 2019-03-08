cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )
if( APPLE AND POLICY CMP0042 )
  cmake_policy( SET CMP0042 NEW )
endif()

project( H3DPhysics )

set( CMAKE_MODULE_PATH "${H3DPhysics_SOURCE_DIR}/localModules" "${H3DPhysics_SOURCE_DIR}/modules" )

include( H3DCommonFunctions )
setCommonH3DCMakePolicies()
setupRPathForLib()
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${H3DPhysics_SOURCE_DIR}/.. GENERATE_H3D_PACKAGE_PROJECT GENERATE_NodeRoutesToDotFile_BUILD )

include( "${H3DPhysics_SOURCE_DIR}/H3DPhysicsSourceFiles.txt" )

set( H3DPhysics_COMPILE_FLAGS "" )
handleUnityBuild( PROJECT_NAME ${PROJECT_NAME}
                  SOURCE_FILES ${H3DPhysics_SRCS}
                  OUTPUT_VARIABLE H3DPhysics_SRCS
                  COMPILE_FLAGS_VARIABLE H3DPhysics_COMPILE_FLAGS )
handlePrecompiledHeaders( PROJECT_NAME ${PROJECT_NAME}
                          HEADERS_VARIABLE H3DPhysics_HEADERS
                          SRCS_VARIABLE H3DPhysics_SRCS
                          STDAFX_HEADER_LOCATION ${H3DPhysics_SOURCE_DIR}/../include/H3D/H3DPhysics/
                          STDAFX_SOURCE_LOCATION ${H3DPhysics_SOURCE_DIR}/../src/ )

# Add the H3DPhysics.rc resource file if Visual Studio
if( MSVC )
  set( H3DPhysics_SRCS ${H3DPhysics_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/H3DPhysics.rc )
endif()



set( optional_libs )

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include ${H3DPhysics_SOURCE_DIR}/../include )

set( required_libs )

if( WIN32 AND MSVC )
  set( required_libs ${required_libs} DelayImp.lib )
endif()

# Version of H3DPhysics
set( H3DPHYSICS_MAJOR_VERSION 1 )
set( H3DPHYSICS_MINOR_VERSION 4 )
set( H3DPHYSICS_BUILD_VERSION 1 )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI H3DAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI H3DAPI )
include_directories( ${tmp_h3d_include_dirs} )

set( H3DPhysics_LINK_FLAGS )

# Optional extra library. Without this H3D API can not load scenes defined with X3D syntax.
find_package( XercesC )
if( XercesC_FOUND )
  set( HAVE_XERCES 1 )
  include_directories( ${XercesC_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${XercesC_LIBRARIES} )
endif()

#SOFA
find_package( SOFA COMPONENTS   core
                                helper
                                simulation
                                tree
                                base_visual
                                defaulttype
                                gui
                                guimain
                                guiglut
                                guiqt
                                component
                                component_base
                                component_common
                                component_general
                                component_misc
                                component_advanced
                                haptics
                                validation
                                exporter
                                engine
                                graph_component
                                topology_mapping
                                boundary_condition
                                user_interaction
                                constraint
                                dense_solver
                                sparse_solver
                                preconditioner
                                opengl_visual
                                base_collision
                                loader
                                mesh_collision
                                base_topology
                                deformable
                                simple_fem
                                explicit_ode_solver
                                implicit_ode_solver
                                base_linear_solver
                                base_mechanics
                                rigid
                                sph_fluid
                                object_interaction )

#find_package( Boost )
if( SOFA_FOUND )
  set( HAVE_SOFA 1 )
  include_directories( ${SOFA_INCLUDE_DIRS} )
#  include_directories( ${Boost_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${SOFA_LIBRARIES} )

  # only needed for some debugging, disabled for now
  # # Pull in external SOFA API source to compile with our project
  # file( TO_CMAKE_PATH "${SOFA_INSTALL_DIR}/applications/projects/SofaPhysicsAPI" SOFA_API_DIR )

  # include_directories( ${SOFA_API_DIR} )
  # set( H3DPhysics_SRCS ${H3DPhysics_SRCS}
                             # "${SOFA_API_DIR}/SofaPhysicsDataController.cpp"
                             # "${SOFA_API_DIR}/SofaPhysicsDataMonitor.cpp"
                             # "${SOFA_API_DIR}/SofaPhysicsOutputMesh.cpp"
                             # "${SOFA_API_DIR}/SofaPhysicsSimulation.cpp"
                             # "${SOFA_API_DIR}/fakegui.cpp" )
  # # Group all Sagentia code in IDE
  # source_group( "SofaAPI" "${SOFA_API_DIR}" )

endif()


#ODE
find_package( ODE )
if( ODE_FOUND )
  set( HAVE_ODE 1 )
  include_directories( ${ODE_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${ODE_LIBRARIES} )
  add_definitions( ${ODE_FLAGS} )
  addDelayLoadFlags( ODE_LIBRARIES H3DPhysics_LINK_FLAGS )
endif()

#PHYSX
find_package( PhysX )
if( PhysX_FOUND )
  set( HAVE_PHYSX 1 )
  include_directories( ${PhysX_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${PhysX_LIBRARIES} )
  addDelayLoadFlags( PhysX_LIBRARIES H3DPhysics_LINK_FLAGS )
endif()

#PHYSX3
find_package( PhysX3 )
if( PhysX3_FOUND )
  set( HAVE_PHYSX3 1 )
  include_directories( ${PhysX3_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${PhysX3_LIBRARIES} )
  addDelayLoadFlags( PhysX3_LIBRARIES H3DPhysics_LINK_FLAGS )

  #HACD
  find_package( HACD )
  if( HACD_FOUND )
    set( HAVE_HACD 1 )
    include_directories( ${HACD_INCLUDE_DIRS} )
    set( optional_libs ${optional_libs} ${HACD_LIBRARIES} )
  endif()
endif()

#Bullet
find_package( Bullet )
if( BULLET_FOUND )
  set( HAVE_BULLET 1 )
  include_directories( ${BULLET_INCLUDE_DIRS} )
  set( optional_libs ${optional_libs} ${BULLET_LIBRARIES} )
endif()

# H3DPhysics physics engine debugging option (pushing object names in the physics engine)
# Add a macro via H3DPhysics.h (auto generated from H3DPhysics.cmake) to include names in the physics engine parameters (better for debugging)
option( USE_H3DPhysics_name_in_phx_engine_parameters "This pushes down the X3DNode's name down to the physics engine" OFF )
if( USE_H3DPhysics_name_in_phx_engine_parameters )
  set( H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME 1 )
endif()

# Optional extra library. Add python bindings to H3D API, allow for extended prototyping without using C++.
if( APPLE )
  set( CMAKE_FIND_FRAMEWORK FIRST )
endif()

set( USE_Python3 "OFF" CACHE BOOL "Check if you want to use Python 3 instead of 2" )
set( PYTHON_VER "UNDEFINED" CACHE STRING "Located python version" )

# Check if the wrong version of python is located.
# If so, clear the cache variables FindPythonLibs-module uses otherwise it will complain and not find the right one
if( NOT (PYTHON_VER STREQUAL "UNDEFINED") )
  if( USE_Python3 AND PYTHON_VER VERSION_LESS "3.0.0" )
    unset(PYTHON_LIBRARY CACHE)
    unset(PYTHON_INCLUDE_DIR CACHE)
    unset(PYTHON_DEBUG_LIBRARY CACHE)
  elseif( (NOT USE_Python3 ) AND ( PYTHON_VER VERSION_GREATER "3.0.0" OR PYTHON_VER VERSION_EQUAL "3.0.0") )
    unset(PYTHON_LIBRARY CACHE)
    unset(PYTHON_INCLUDE_DIR CACHE)
    unset(PYTHON_DEBUG_LIBRARY CACHE)
  endif()
endif()

if( USE_Python3 )
  find_package( PythonLibs 3 )
  if( PYTHONLIBS_FOUND )
    set( PYTHON_VER ${PYTHONLIBS_VERSION_STRING} CACHE STRING "Located python version" FORCE )
  else ()
    if( WIN32 )
      # For all windows distributions: define which architectures can be used
      if (CMAKE_SIZEOF_VOID_P)
        # In this case, search only for 64bit or 32bit
        math (EXPR _${_PYTHON_PREFIX}_ARCH "${CMAKE_SIZEOF_VOID_P} * 8")
        set (_${_PYTHON_PREFIX}_ARCH2 ${_${_PYTHON_PREFIX}_ARCH})
      else()
        # architecture unknown, search for both 64bit and 32bit
        set (_${_PYTHON_PREFIX}_ARCH 64)
        set (_${_PYTHON_PREFIX}_ARCH2 32)
      endif()

      foreach(_CURRENT_VERSION ${_Python_VERSIONS})
        string(REPLACE "." "" _CURRENT_VERSION_NO_DOTS ${_CURRENT_VERSION})
        find_library(PYTHON_LIBRARY
          NAMES
            python${_CURRENT_VERSION_NO_DOTS}
            python${_CURRENT_VERSION}mu
            python${_CURRENT_VERSION}m
            python${_CURRENT_VERSION}u
            python${_CURRENT_VERSION}
          NAMES_PER_DIR
          PATHS
            [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}-${_${_PYTHON_PREFIX}_ARCH}\\InstallPath]/libs
            [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}-${_${_PYTHON_PREFIX}_ARCH2}\\InstallPath]/libs
            [HKEY_CURRENT_USER\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}-${_${_PYTHON_PREFIX}_ARCH}\\InstallPath]/libs
            [HKEY_CURRENT_USER\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}-${_${_PYTHON_PREFIX}_ARCH2}\\InstallPath]/libs
        )
      endforeach()
      find_package( PythonLibs 3 )
    endif()
    if( PYTHONLIBS_FOUND )
      set( PYTHON_VER ${PYTHONLIBS_VERSION_STRING} CACHE STRING "Located python version" FORCE )
    else()
      set( PYTHON_VER "UNDEFINED" CACHE STRING "Located python version" FORCE )
      message( WARNING "COULD NOT FIND PYTHON 3 - GENERATED PROJECT WILL HAVE ZERO PYTHON SUPPORT. UNCHECK USE_Python3 IF YOU WISH TO LOOK FOR PYTHON 2.")
    endif()
  endif()
else()
  find_package( PythonLibs 2 )
  if( PYTHONLIBS_FOUND )
    set( PYTHON_VER ${PYTHONLIBS_VERSION_STRING} CACHE STRING "Located python version" FORCE )
  else ()
    set( PYTHON_VER "UNDEFINED" CACHE STRING "Located python version" FORCE )
    message( WARNING "COULD NOT FIND PYTHON 2 - GENERATED PROJECT WILL HAVE ZERO PYTHON SUPPORT.")
  endif()
endif()

if( PYTHONLIBS_FOUND )
  set( HAVE_PYTHON 1 )
  set( HAVE_PYTHON_OSX_FRAMEWORK )
  include_directories( ${PYTHON_INCLUDE_DIRS} )
  if( APPLE )
    foreach( dir ${PYTHON_INCLUDE_DIRS} )
      if( dir MATCHES "/System/Library/Frameworks/Python[.]framework" )
        set( HAVE_PYTHON_OSX_FRAMEWORK 1 )
        break()
      endif()
    endforeach()
  endif()
  if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    # pyconfig.h is put in different directory from Ubuntu 13.04 (raring)
    # and CMake FindPythonLibs module is not updated for this yet.
    # Adding it explicitly here in the mean time.
    include_directories( /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}/python2.7 )
  endif()
  set( optional_libs ${optional_libs} ${PYTHON_LIBRARIES} )

  if( PYTHONLIBS_VERSION_STRING VERSION_LESS "3.0.0" AND CMAKE_COMPILER_IS_GNUCXX )
    # Python 2.7 does not follow strict aliasing rules (pointers to objects
    # of different types will never refer to the same memory location). So we
    # must disable optimizations based on this assumption. Failing to do so
    # gives warnings (with -Wall) and could give undefined results.
    # This issue is fixed in Python 3. As far as I can see VS does not make
    # such optimizations, so no action is required there.
    # Also see: http://legacy.python.org/dev/peps/pep-3123/
    add_definitions(-fno-strict-aliasing)
  endif()
endif()

# autogenerate H3DPhysics.h depending on the libraries available.
if( EXISTS ${H3DPhysics_SOURCE_DIR}/../include/H3D/H3DPhysics/H3DPhysics.h )
  file( REMOVE ${H3DPhysics_SOURCE_DIR}/../include/H3D/H3DPhysics/H3DPhysics.h )
endif()
configure_file( ${H3DPhysics_SOURCE_DIR}/../include/H3D/H3DPhysics/H3DPhysics.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/H3DPhysics/H3DPhysics.h )

if( NOT GENERATE_H3D_PACKAGE_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/H3D/H3DPhysics/H3DPhysics.h
           DESTINATION ${H3DPhysics_SOURCE_DIR}/../include/H3D/H3DPhysics )
endif()

populateProjectIncludeDirectoriesCacheVar( VARIABLE_NAME H3DPhysics_INCLUDE_DIRS
                                           DEPRECATED_VARIABLE_NAMES H3DPHYSICS_INCLUDE_DIR )

add_library( H3DPhysics SHARED ${H3DPhysics_SRCS}
                                    ${H3DPhysics_HEADERS} )

# make sure that the H3DPhysics.rc contains the correct svn-version
setupResourceFile( H3DPhysics
                   VERSION_PREFIX H3DPHYSICS
                   SVN_DIR_CANDIDATE ${H3DPhysics_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${H3DPhysics_SOURCE_DIR}/H3DPhysics.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/H3DPhysics.rc
                   UPDATERESOURCEFILE_EXE ${H3DPhysics_SOURCE_DIR}/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "H3DPHYSICS_Output_Name" "$(TargetFileName)" )

target_link_libraries( H3DPhysics ${required_libs} ${optional_libs} )

handleCommonCacheVar( USE_thread_lock_debug FALSE )

# make sure symbols are exported.
add_definitions( -DH3DPHYSICS_EXPORTS )

set( H3DPHYSICS_FULL_VERSION ${H3DPHYSICS_MAJOR_VERSION}.${H3DPHYSICS_MINOR_VERSION}.${H3DPHYSICS_BUILD_VERSION} )
set_target_properties( H3DPhysics PROPERTIES VERSION ${H3DPHYSICS_FULL_VERSION} )

# Debug version should have _d postfix.
set_target_properties( H3DPhysics PROPERTIES DEBUG_POSTFIX "_d" )

if( SOFA_FOUND )
  set( H3DPhysics_COMPILE_FLAGS "${H3DPhysics_COMPILE_FLAGS} -DSOFA_HAVE_GLEW -DTIXML_USE_STL" )
endif()


if( UNIX )
  set_target_properties( H3DPhysics PROPERTIES OUTPUT_NAME h3dphysics )
endif()

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

if( WIN32 )
  if( MSVC )
    include( H3DCommonFunctions )
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other.
    setH3DMSVCOutputName( H3DPhysics "H3DPhysics" )
    # Add common H3D compile flags for microsoft visual studio
    addCommonH3DMSVCCompileFlags( H3DPhysics_COMPILE_FLAGS )
  endif()

  # set link flags for H3DPhysics project
  # Seems like H3DPhysics needs to have incremental linking shut off for vc8. Since this is
  # not the most important feature it is shut off for all visual studio versions.
  if( MSVC )
    set_target_properties( H3DPhysics PROPERTIES LINK_FLAGS_RELWITHDEBINFO "${H3DPhysics_LINK_FLAGS} /INCREMENTAL:NO /NODEFAULTLIB:LIBCMT"
                                                 LINK_FLAGS_RELEASE "${H3DPhysics_LINK_FLAGS} /INCREMENTAL:NO /NODEFAULTLIB:LIBCMT"
                                                 LINK_FLAGS_MINSIZEREL "${H3DPhysics_LINK_FLAGS} /INCREMENTAL:NO /NODEFAULTLIB:LIBCMT"
                                                 LINK_FLAGS_DEBUG "${H3DPhysics_LINK_FLAGS} /INCREMENTAL:NO /NODEFAULTLIB:LIBCMTD" )
  endif()
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
  addCommonH3DGNUCompileFlags( H3DPhysics_COMPILE_FLAGS )
endif()

if( GENERATE_NodeRoutesToDotFile_BUILD )
  set( H3DPhysics_COMPILE_FLAGS "${H3DPhysics_COMPILE_FLAGS} -DH3D_GENERATE_DOTROUTE_FILES" )
endif()

# set compile flags for H3DPhysics project
set_target_properties( H3DPhysics PROPERTIES COMPILE_FLAGS "${H3DPhysics_COMPILE_FLAGS}" )

if( UNIX )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${H3DPhysics_HEADERS}
           DESTINATION include/H3D/H3DPhysics )

endif()

if( MSVC )
  set_target_properties( H3DPhysics PROPERTIES FOLDER "H3D" )
endif()

# Intall directories.
install( TARGETS H3DPhysics
         LIBRARY DESTINATION ${default_lib_install} COMPONENT H3DPhysics_cpack_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT H3DPhysics_cpack_runtime
         ARCHIVE DESTINATION ${default_lib_install} COMPONENT H3DPhysics_cpack_libraries )

if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "H3DPhysics" )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PARENT_SCOPE )
else()
  include( ${H3DPhysics_SOURCE_DIR}/H3DPhysicsCPack.cmake )
endif()
