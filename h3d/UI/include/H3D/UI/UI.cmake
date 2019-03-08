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
/// \file UI.h
/// \brief Base header file that handles all configuration related settings
///
//////////////////////////////////////////////////////////////////////////////

/// \mainpage UI documentation
/// Copyright 2004 - 2019, <a href="http://www.sensegraphics.com">SenseGraphics AB</a>

#ifndef __UI_H__
#define __UI_H__

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler. All files within this DLL
// are compiled with the UI_EXPORTS symbol defined on the command
// line. this symbol should not be defined on any project that uses
// this DLL. This way any other project whose source files include
// this file see UI_API functions as being imported from a DLL,
// whereas this DLL sees symbols defined with this macro as being
// exported.
#ifdef WIN32
#include <windows.h>
#ifdef UI_EXPORTS
#define UI_API __declspec(dllexport)
#else
#define UI_API __declspec(dllimport)
#endif
#ifdef _MSC_VER
// disable dll-interface warnings for stl-exports 
#pragma warning( disable: 4251 )
// disable warning C4273: 'getLibraryInfo' : inconsistent dll linkage
#pragma warning( disable: 4273 )
#endif


#endif

#if defined(__APPLE__) && defined(__MACH__)
#define MACOSX
#define UI_API
#define HAVE_SYS_TIME_H 1
#endif

#if defined(__linux)
#define LINUX
#define UI_API 
#define HAVE_SYS_TIME_H 1
#endif

#define UI_MAJOR_VERSION ${UI_MAJOR_VERSION}
#define UI_MINOR_VERSION ${UI_MINOR_VERSION}
#define UI_BUILD_VERSION ${UI_BUILD_VERSION}

#include <H3D/LibraryInfo.h>

// Function to extract information about the library.
extern "C" UI_API H3D::LibraryInfo getLibraryInfo();
#ifdef _MSC_VER
// restore warning C4273: 'getLibraryInfo' : inconsistent dll linkage
#pragma warning( default: 4273 )
#endif

#endif

/// \defgroup AbstractNodes Abstract nodes.
/// \defgroup UINodes UI node classes.
