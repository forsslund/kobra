//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3DUtil.
//
//    H3DUtil is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3DUtil is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3DUtil; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
//
/// \file H3DUtil.h
/// \brief Base header file that handles all configuration related settings
///
//////////////////////////////////////////////////////////////////////////////

/// \mainpage H3DUtil Documentation
/// Copyright 2004 - 2019, <a href="http://www.sensegraphics.com">SenseGraphics AB</a>

#ifndef __H3DUTIL_H__
#define __H3DUTIL_H__

/// Undef if you do not have sofa helper component(http://www.sofa-framework.org/) installed
/// even if you set ENABLE_PROFILER to true
/// Required for support for time profiling.
#cmakedefine HAVE_PROFILER

#cmakedefine THREAD_LOCK_DEBUG

#cmakedefine PTHREAD_W32_LEGACY_VERSION

/// Undef if you do not have visual leak detector(https://vld.codeplex.com/) installed
#cmakedefine HAVE_LIBVLD
#ifdef HAVE_LIBVLD
/// If enabled VLD will be used in all configurations. If disabled VLD will only be used in Debug.
#cmakedefine VLD_FORCE_ENABLE
#include <vld/vld.h>
#endif

/// Undef if you do not have zlib(http://www.zlib.net/) installed. 
/// Required for support for parsing zipped files.
#cmakedefine HAVE_ZLIB

/// Undef if you do not have FreeImage(freeimage.sourceforge.net) installed.
/// Image files will not be possible to read (see ImageLoaderFunctions.h).
#cmakedefine HAVE_FREEIMAGE

/// Undef if you do not have teem(http://teem.sourceforge.net/) installed.
/// Nrrd files will not be possible to read otherwise (see
/// ImageLoaderFunctions.h).
#cmakedefine HAVE_TEEM

/// Undef if you do not have dcmtk(http://dicom.offis.de/) installed.
/// Dicom files will not be possible to read otherwise (see
/// ImageLoaderFunctions.h).
#cmakedefine HAVE_DCMTK
#cmakedefine DCMTK_IS_VERSION360

/// Defines if you do have the NVidia Tools Extension Library available
/// and have chosen to use it with CMake.
#cmakedefine HAVE_NVIDIATX

/// Defines if you have the H3D Profiler library available
/// and have chosen to use it with CMake
#cmakedefine HAVE_H3DPROFILER

/// Undef if you do not have OpenEXR
#cmakedefine HAVE_OPENEXR

// note that _WIN32 is always defined when _WIN64 is defined.
#if( defined( _WIN64 ) || defined(WIN64) )
// set when on 64 bit Windows
#define H3D_WIN64
#define H3D_ARCH64
#elif( defined( _WIN32 ) || defined(WIN32) )
// set when on 32 bit Windows
#define H3D_WIN32
#define H3D_ARCH32
#endif

#if __GNUC__
#if __x86_64__ || __ppc64__
#define H3D_ARCH64
#else
#define H3D_ARCH32
#endif
#endif

#if( defined( H3D_WIN32 ) || defined( H3D_WIN64 ) )
// set when on 32 or 64 bit Windows
#define H3D_WINDOWS
#endif

#ifdef H3D_WINDOWS
// Define this if you are linking Freeimage as a static library
#cmakedefine FREEIMAGE_LIB
#endif

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler. All files within this DLL
// are compiled with the H3DUTIL_EXPORTS symbol defined on the command
// line. this symbol should not be defined on any project that uses
// this DLL. This way any other project whose source files include
// this file see H3DUTIL_API functions as being imported from a DLL,
// whereas this DLL sees symbols defined with this macro as being
// exported.
#ifdef H3D_WINDOWS
#include <windows.h>

#ifdef H3DUTIL_LIB
#define H3DUTIL_API
#else

#ifdef H3DUTIL_EXPORTS
#define H3DUTIL_API __declspec(dllexport)
#else
#define H3DUTIL_API __declspec(dllimport)
#endif
#ifdef _MSC_VER
// disable dll-interface warnings for stl-exports 
#pragma warning( disable: 4251 )
#endif

#endif

#endif

#ifdef H3D_WINDOWS

#ifdef _DEBUG
#define H3D_DEBUG
#endif

#if defined(__MINGW32__) || defined(HAVE_LONG_LONG)
#define _int64 long long
#endif
#define H3DUTIL_INT64 _int64
#else
#cmakedefine H3D_DEBUG
#if defined(__GNUC__) || defined(HAVE_LONG_LONG)
#define H3DUTIL_INT64 long long
#endif
#endif

#if defined(__APPLE__) && defined(__MACH__)
#define MACOSX
#define H3D_OSX
#define H3DUTIL_API
#ifndef HAVE_SYS_TIME_H
#define HAVE_SYS_TIME_H 1
#endif
#endif

#if defined(__linux)
#define LINUX
#define H3D_LINUX
#define H3DUTIL_API 
#ifndef HAVE_SYS_TIME_H
#define HAVE_SYS_TIME_H 1
#endif
#endif

#define H3DUTIL_MAJOR_VERSION ${H3DUTIL_MAJOR_VERSION}
#define H3DUTIL_MINOR_VERSION ${H3DUTIL_MINOR_VERSION}
#define H3DUTIL_BUILD_VERSION ${H3DUTIL_BUILD_VERSION}

#if defined(_DEBUG) || defined(HAVE_PROFILER)
// if defined additional information about reference counted objects will be available.
// Also adds reference count to tree view in H3DViewer as well as possibility to see 
// where those references are held.
#define H3D_REFERENCE_COUNT_DEBUG
#endif

/// \defgroup H3DUtilClasses H3DUtil classes
/// All grouped classes in H3DUtil should be in this group.

/// H3DUtil namespace
namespace H3DUtil {
  /// Initialize H3DUtil(only needed if using H3DUtil as a static library)?
  void initializeH3DUtil();

  /// Deinitialize H3DUtil(only needed if using H3DUtil as a static library)?
  void deinitializeH3DUtil();

  /// Will return the version of H3DUtil as a double on the form
  /// H3DUTIL_MAJOR_VERSION.H3DUTIL_MINOR_VERSION
  double H3DUTIL_API getH3DUtilVersion();
}

#ifndef DEPRECATION_ENABLED
#define DEPRECATION_ENABLED
#endif

#ifndef DEPRECATION_ENABLED
#define DEPRECATED(alternative, entity) entity
#define DEPRECATED_NOALT
#endif

#ifndef DEPRECATED
#ifdef DEPRECATION_ENABLED
#ifdef __GNUC__
#define DEPRECATED(alternative, entity) __attribute__((deprecated)) entity
#define DEPRECATED_NOALT __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED_MESSAGE(alternative) "Entity has been deprecated. Use " alternative " instead."
#define DEPRECATED(alternative, entity) __declspec(deprecated(DEPRECATED_MESSAGE(alternative))) entity
#define DEPRECATED_NOALT __declspec(deprecated)
#else
#pragma message("Deprecation has not been implemented for this compiler.")
#define DEPRECATED(alternative, entity) entity
#define DEPRECATED_NOALT
#endif
#endif
#endif

#ifdef __GNUC__
#define H3D_DISABLE_UNUSED_PARAMETER_WARNING() _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")
#define H3D_PUSH_WARNINGS() _Pragma("GCC diagnostic push")
#define H3D_POP_WARNINGS() _Pragma("GCC diagnostic pop")
#elif defined(_MSC_VER)
#define H3D_DISABLE_UNUSED_PARAMETER_WARNING() __pragma(warning(disable:4100))
#define H3D_PUSH_WARNINGS() __pragma(warning(push))
#define H3D_POP_WARNINGS() __pragma(warning(pop))
#else
#define H3D_DISABLE_UNUSED_PARAMETER_WARNING()
#define H3D_PUSH_WARNINGS()
#define H3D_POP_WARNINGS()
#endif

#endif

