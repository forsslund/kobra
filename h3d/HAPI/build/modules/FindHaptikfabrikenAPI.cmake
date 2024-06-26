# - Find HaptikfabrikenAPI
# Find the native HaptikfabrikenAPI headers and libraries.
#
#  HaptikfabrikenAPI_INCLUDE_DIRS -  where to find HaptikfabrikenAPI headers
#  HaptikfabrikenAPI_LIBRARIES    - List of libraries when using HaptikfabrikenAPI.
#  HaptikfabrikenAPI_FOUND        - True if HaptikfabrikenAPI found.

include( H3DUtilityFunctions )
handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES HaptikfabrikenAPI_INCLUDE_DIR HaptikfabrikenAPI_LIBRARY
                                              OLD_VARIABLE_NAMES HAPTIKFABRIKEN_INCLUDE_DIR HAPTIKFABRIKEN_LIBRARY
                                              DOC_STRINGS "Path in which the file HaptikfabrikenAPI.h is located. Needed to support Woodenhaptics and Polhem."
                                                          "Path to haptikfabrikenDLL.lib library." )

get_filename_component( module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )
MESSAGE("${module_file_path}/../../../../haptikfabrikenapi/winbin")
set( haptikfabrikenapi_include_search_paths "${module_file_path}/../../../../haptikfabrikenapi/winbin" )
set( haptikfabrikenapi_lib_search_paths "${module_file_path}/../../../../haptikfabrikenapi/winbin" )
if( NOT MSVC14 )
  include( H3DCommonFindModuleFunctions )
  getExternalSearchPathsH3D( haptikfabrikenapi_include_search_paths haptikfabrikenapi_lib_search_paths ${module_file_path} )
endif()

# Look for the header file.
find_path( HaptikfabrikenAPI_INCLUDE_DIR NAMES haptikfabrikenapi.h 
                                PATHS ${haptikfabrikenapi_include_search_paths}
                                DOC "Path in which the file fshapticdevicethread.h is located. Needed to support Haptikfabriken devices." )
mark_as_advanced( HaptikfabrikenAPI_INCLUDE_DIR )


# Look for the library.
if( WIN32 )
  find_library( HaptikfabrikenAPI_LIBRARY NAMES haptikfabrikenapi
                           PATHS ${haptikfabrikenapi_lib_search_paths}
                           DOC "Path to haptikfabriken.lib library. Needed to support haptikfabriken devices." )
else()
  find_library( HaptikfabrikenAPI_LIBRARY NAMES haptikfabrikenapi
                           PATHS ${haptikfabrikenapi_lib_search_paths}
                           DOC "Path to haptikfabrikenapi library. Needed to support haptikfabriken devices." )

endif()
mark_as_advanced( HaptikfabrikenAPI_LIBRARY )

include( FindPackageHandleStandardArgs )
# handle the QUIETLY and REQUIRED arguments and set HaptikfabrikenAPI_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args( HaptikfabrikenAPI DEFAULT_MSG
                                   HaptikfabrikenAPI_LIBRARY HaptikfabrikenAPI_INCLUDE_DIR )

set( HaptikfabrikenAPI_LIBRARIES ${HaptikfabrikenAPI_LIBRARY} )
set( HaptikfabrikenAPI_INCLUDE_DIRS ${HaptikfabrikenAPI_INCLUDE_DIR} )



# Backwards compatibility values set here.
set( HAPTIKFABRIKEN_INCLUDE_DIR ${HaptikfabrikenAPI_INCLUDE_DIRS} )
set( HAPTIKFABRIKEN_LIBRARIES ${HaptikfabrikenAPI_LIBRARIES} )
#set( HaptikfabrikenAPI_FOUND ${HAPTIKFABRIKEN_FOUND} ) # find_package_handle_standard_args for CMake 2.8 only define the upper case variant. 
set( HAPTIKFABRIKEN_FOUND  ${HaptikfabrikenAPI_FOUND} )
