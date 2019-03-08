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
/// \file H3DPhysics.cpp
/// \brief Source file for library initialization functions for the
/// RigidBodyPhysics library.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <sstream>

#ifdef HAVE_BULLET
 // #include <btBulletDynamicsCommon.h>
#endif

#ifdef HAVE_ODE
#include <ode/ode.h>
#endif

#ifdef HAVE_PHYSX
// include relevent headers
#endif

namespace RBP {
  /// Initialize RBP API(only needed if using RBP API as a static library). 
  void initializeRBP() {
#ifdef HAVE_BULLET

#endif
#ifdef HAVE_ODE
    dInitODE();
#endif 
#ifdef HAVE_PHYSX

#endif
  }

/// Deinitialize RBP API(only needed if using RBP API as a static library). 
  void deinitializeRBP() {
#ifdef HAVE_BULLET

#endif
#ifdef HAVE_ODE
    dCloseODE();
#endif

#ifdef HAVE_PHYSX
  
#endif
  }
}


#ifdef H3D_WINDOWS
#if defined(__MINGW32__) && defined(__cplusplus)
// MinGW uses C linkage for DllMain()
extern "C" {
#endif
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
                       ) {
  switch (ul_reason_for_call) {
  case DLL_PROCESS_ATTACH: {
    RBP::initializeRBP();
    break;
  }
  case DLL_THREAD_ATTACH:
    break;
  case DLL_THREAD_DETACH:
    break;
  case DLL_PROCESS_DETACH:
    RBP::deinitializeRBP();
    break;
  }
  return TRUE;
}
#if defined(__MINGW32__) && defined(__cplusplus)
}
#endif
#else 
#ifdef __cplusplus
extern "C" {
#endif
  void __attribute__((constructor)) initRBPAPI( void ) {
    RBP::initializeRBP();
  }
  void __attribute__((destructor)) finiRBPAPI( void ) {
    RBP::deinitializeRBP();
  }
#ifdef __cplusplus
}
#endif

#endif // H3D_WINDOWS

#include <cstring>
using namespace std;
H3D::LibraryInfo getLibraryInfo() {
  H3D::LibraryInfo r = H3D::LibraryInfo::createEmptyInfo();
  std::stringstream s;
  s << H3DPHYSICS_MAJOR_VERSION << "."
    << H3DPHYSICS_MINOR_VERSION << " (build "
    << H3DPHYSICS_BUILD_VERSION << ")";

  strcpy(r.name, "H3DPhysics" );
  strcpy( r.version, s.str().c_str() );
  //strcpy( r.web, "http://www.h3dapi.org/modules/mediawiki/index.php/H3DPhysics" );
  strcpy( r.developer, "SenseGraphics AB" );
  strcpy( r.developer_web, "http://www.sensegraphics.com" );
  strcpy( r.info, "Adds the nodes from the H3DPhysics component of X3D allowing physics simulation. Soft and rigid bodies may be defined and manipulated with haptic devices." );
  return r;

}
