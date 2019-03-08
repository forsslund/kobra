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
/// \file H3DPluginActiveX.h
/// \brief Header file for CH3DPluginActiveXApp.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DPLUGIN_ACTIVEX_H
#define __H3DPLUGIN_ACTIVEX_H

#if !defined( __AFXCTL_H__ )
#error include 'afxctl.h' before including this file
#endif

#include "Resource.h"

class CH3DPluginActiveXApp : public COleControlModule
{
public:
  BOOL InitInstance();
  int ExitInstance();
};


extern CH3DPluginActiveXApp theApp;


extern const GUID CDECL _tlid;
extern const WORD _wVerMajor;
extern const WORD _wVerMinor;

#endif
