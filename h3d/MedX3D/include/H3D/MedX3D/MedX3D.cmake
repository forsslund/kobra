//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
//
/// \file MedX3D.h
/// \brief Base header file for MedX3D
///
//////////////////////////////////////////////////////////////////////////////

/// \mainpage MedX3D documentation
/// Copyright 2004 - 2019, <a href="http://www.sensegraphics.com">SenseGraphics AB</a>

#ifndef __MEDX3D_H__
#define __MEDX3D_H__

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler. All files within this DLL
// are compiled with the MEDX3D_EXPORTS symbol defined on the command
// line. this symbol should not be defined on any project that uses
// this DLL. This way any other project whose source files include
// this file see MEDX3D_API functions as being imported from a DLL,
// whereas this DLL sees symbols defined with this macro as being
// exported.
#ifdef WIN32
#include <windows.h>
#ifdef MEDX3D_EXPORTS
#define MEDX3D_API __declspec(dllexport)
#else
#define MEDX3D_API __declspec(dllimport)
#endif
#ifdef _MSC_VER
// disable dll-interface warnings for stl-exports 
#pragma warning( disable: 4251 )
// disable warning C4273: 'getLibraryInfo' : inconsistent dll linkage
#pragma warning( disable: 4273 )
#endif


#endif

#if defined(__APPLE__) && defined(__MACH__)
#define MEDX3D_API
#endif

#if defined(__linux)
#define MEDX3D_API
#endif

#define MEDX3D_DEBUG() std::cerr << __FUNCTION__ << " --- Line " << __LINE__ << " in " << __FILE__ << std::endl;

#define MEDX3D_MAJOR_VERSION ${MEDX3D_MAJOR_VERSION}
#define MEDX3D_MINOR_VERSION ${MEDX3D_MINOR_VERSION}
#define MEDX3D_BUILD_VERSION ${MEDX3D_BUILD_VERSION}

#include <H3D/LibraryInfo.h>
#ifdef _MSC_VER
// restore warning C4273: 'getLibraryInfo' : inconsistent dll linkage
#pragma warning( default: 4273 )
#endif

extern "C" MEDX3D_API H3D::LibraryInfo getLibraryInfo();

/// \defgroup MedX3DClasses MedX3D Classes
/// Extra classes in MedX3D used for various things.

/// \ingroup Nodes
/// \defgroup MedX3DNodes MedX3D Node Classes.
/// These are the Node classes belonging to MedX3D.

#endif

