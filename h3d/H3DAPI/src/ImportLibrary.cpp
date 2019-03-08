
//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file ImportLibrary.cpp
/// \brief CPP file for ImportLibrary, H3D scene-graph node.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ImportLibrary.h>
#include <H3D/ResourceResolver.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase ImportLibrary::database( 
                                    "ImportLibrary", 
                                    &(newInstance<ImportLibrary>), 
                                    typeid( ImportLibrary ),
                                    &X3DChildNode::database );

namespace TransformInternals {
  FIELDDB_ELEMENT( ImportLibrary, url, INITIALIZE_ONLY )
  // for backwards compability
  FIELDDB_ELEMENT_EX( ImportLibrary, url, INITIALIZE_ONLY, library )
}

ImportLibrary::ImportLibrary( Inst< SFNode >  _metadata,
                              Inst< MFString > _url ):
  X3DChildNode( _metadata ),
  X3DUrlObject( _url ) {
  
  type_name = "ImportLibrary";
  database.initFields( this );
}


string ImportLibrary::GetVCVer() {
#if _MSC_VER < 1500
  return "vc8";
#elif _MSC_VER < 1600
  return "vc9";
#elif _MSC_VER < 1700
  return "vc10";
#elif _MSC_VER < 1800
  return "vc11";
#elif _MSC_VER < 1900
  return "vc12";
#elif _MSC_VER < 1910
  return "vc14";
#elif _MSC_VER < 2000
  return "vc15";
#endif
}


DynamicLibrary::LIBHANDLE ImportLibrary::TryLoadLibrary(string library_name, bool ends_in_dll) {
  DynamicLibrary::LIBHANDLE handle;
  if (!ends_in_dll) {
    // try relative path first
    handle = DynamicLibrary::load(url_base + library_name);
    if (handle) return handle;
  }
  // try absolute path.
  handle = DynamicLibrary::load(library_name);
  if (handle) return handle;
  return 0;
}


void ImportLibrary::initialize() {
  for( MFString::const_iterator i = url->begin();
       i != url->end();
       ++i ) {
    string urn_name = *i;
    URNResolver *urn_resolver = ResourceResolver::getURNResolver();
    if( urn_resolver ) {
      urn_name = urn_resolver->resolveURN( *i );
    }

    // If the urn specified contains a file-extension (.dll, .so, .dylib), extract the file-extension free part
    // so that debug suffix can be properly injected
    string extension_stripped_urn = urn_name;
    string extension = "";

    if( urn_name.find( ".dll" ) != string::npos ) {
      extension = ".dll";
      extension_stripped_urn = urn_name.substr( 0, urn_name.find( ".dll" ) );
    } else if( urn_name.find( ".so" ) != string::npos ) {
      extension = ".so";
      extension_stripped_urn = urn_name.substr( 0, urn_name.find( ".so" ) );
    } else if( urn_name.find( ".dylib" ) != string::npos ) {
      extension = ".dylib";
      extension_stripped_urn = urn_name.substr( 0, urn_name.find( ".dylib" ) );
    }

    bool ends_in_dll = false;
#ifdef H3D_WINDOWS
    ends_in_dll = urn_name.find(".dll") != string::npos;
#endif // H3D_WINDOWS

#ifdef H3D_DEBUG
    if(TryLoadLibrary( extension_stripped_urn + "_d" + extension, ends_in_dll))
      return;
#endif // H3D_DEBUG
    if (TryLoadLibrary( extension_stripped_urn + extension, ends_in_dll))
      return;

#ifdef H3D_WINDOWS
#ifdef H3D_DEBUG
    if (TryLoadLibrary( extension_stripped_urn + "_" + GetVCVer() + "_d" + extension, ends_in_dll))
      return;
#endif // H3D_DEBUG
    if (TryLoadLibrary( extension_stripped_urn + "_" + GetVCVer() + extension, ends_in_dll))
      return;
#endif // H3D_WINDOWS


    // test the given name directly.
    string filename = resolveURLAsFile( urn_name );
    if( filename == "" ) filename = urn_name;
    if(DynamicLibrary::load( filename ))
      return;
  }

  // no library found
  Console(LogLevel::Error) << "Warning: Could not load any of the dynamic libraries ";
  for( MFString::const_iterator i = url->begin();
       i != url->end();
       ++i ) {
    Console(LogLevel::Error) << "\"" << *i << "\" ";
  }

  Console(LogLevel::Error) << "specified in " << getName() 
             << " (" << DynamicLibrary::getLastError() << ")." << endl;
}
