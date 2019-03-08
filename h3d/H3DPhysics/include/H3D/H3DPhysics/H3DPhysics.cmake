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
//
/// \file H3DPhysics/H3DPhysics.h
/// \brief Base header file for RBP
///
//////////////////////////////////////////////////////////////////////////////

/// \mainpage H3DPhysics documentation
/// Copyright 2004 - 2019, <a href="http://www.sensegraphics.com">SenseGraphics AB</a>

#ifndef __H3DPHYSICS_H__
#define __H3DPHYSICS_H__

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler. All files within this DLL
// are compiled with the RBP_EXPORTS symbol defined on the command
// line. this symbol should not be defined on any project that uses
// this DLL. This way any other project whose source files include
// this file see H3DPHYS_API functions as being imported from a DLL,
// whereas this DLL sees symbols defined with this macro as being
// exported.
#ifdef WIN32
#include <windows.h>

#ifdef H3DPHYSICS_EXPORTS
#define H3DPHYS_API __declspec(dllexport)
#else
#define H3DPHYS_API __declspec(dllimport)
#endif
#ifdef _MSC_VER
// disable dll-interface warnings for stl-exports
#pragma warning( disable: 4251 )
// disable warning C4273: 'getLibraryInfo' : inconsistent dll linkage
#pragma warning( disable: 4273 )
#endif


#endif

#if defined(__APPLE__) && defined(__MACH__)
#define H3DPHYS_API
#endif

#if defined(__linux)
#define H3DPHYS_API
#endif


#define H3DPHYSICS_MAJOR_VERSION ${H3DPHYSICS_MAJOR_VERSION}
#define H3DPHYSICS_MINOR_VERSION ${H3DPHYSICS_MINOR_VERSION}
#define H3DPHYSICS_BUILD_VERSION ${H3DPHYSICS_BUILD_VERSION}

#include <H3D/LibraryInfo.h>

extern "C" H3DPHYS_API H3D::LibraryInfo getLibraryInfo();
#ifdef _MSC_VER
// restore warning C4273: 'getLibraryInfo' : inconsistent dll linkage
#pragma warning( default: 4273 )
#endif

#cmakedefine HAVE_ODE
#cmakedefine ODE_VERSION_013_OR_EARLIER

#cmakedefine HAVE_PHYSX
#cmakedefine HAVE_PHYSX3
#cmakedefine HAVE_HACD

#cmakedefine HAVE_BULLET
#cmakedefine BULLET_HAVE_COLLISION_OBJECT_WRAPPER

#cmakedefine HAVE_SOFA

#cmakedefine HAVE_PYTHON
#cmakedefine HAVE_PYTHON_OSX_FRAMEWORK

/// H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME (if defined) helps for debugging
/// the physics engine (where objects have appropriate names instead of default ones).
#cmakedefine H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME

/// \defgroup SoftBody Soft body physics
/// Nodes specific to the soft body physics implementation

/// \defgroup Bullet Bullet physics engine
/// Nodes specific to the Bullet physics implementation

// Shiny profiler: http://sourceforge.net/projects/shinyprofiler/
//#define USE_PROFILER

#ifdef USE_PROFILER
#include <Shiny.h>
#else
#define PROFILE_BEGIN(name)
#define PROFILE_END()
#endif

#ifdef HAVE_PHYSX3

// the PhysX3 include files requires a define with LINUX set to 1 when
// compiling in Linux.
#ifdef H3D_LINUX
// check that exactly one of NDEBUG and _DEBUG is defined
#if !(defined NDEBUG ^ defined _DEBUG)
  #define _DEBUG 1
#endif
#endif

#endif

// Output debug information to file to debug
// lag of rigid body positions
//#define DEBUG_RB_LAG

// needed for Sofa
#ifdef min
#undef min
#endif
#ifdef max
#undef max

#endif

#endif

