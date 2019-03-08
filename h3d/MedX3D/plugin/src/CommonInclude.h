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
/// \file CommonInclude.h
/// \brief Header file for properties used in several places in the code.
/// node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DPLUGIN_COMMON_INCLUDE_H
#define __H3DPLUGIN_COMMON_INCLUDE_H

#include <H3D/H3DApi.h>

#ifdef H3D_WINDOWS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX 1
#include <windows.h>

#include <cassert>

#endif

// Uncomment to process rendering on idle time instead
// Do note that idle time processing is not implemented yet! 
#define USE_TIMER

#ifdef USE_TIMER
#define PlayerFPS 100
#endif


//#pragma warning(disable:4267)



#endif
