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
/// \file X3DPluginActiveX.cpp
/// \brief CPP file for CH3DPluginActiveXApp and register/unregister functions
/// needed to make this an ActiveX plugin.
///
//
//////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "H3DPluginActiveX.h"
#include <algorithm>
#include <cassert>
#include <map>
#include <H3DPluginInstance.h>
#include <H3D/ProfilesAndComponents.h>
#ifdef HAVE_MEDX3D
#include <H3D/MedX3D/VolumeData.h>
#endif


CH3DPluginActiveXApp NEAR theApp;
#ifdef PRINT_ERR_FILE
H3DPluginInstance::PrintingCerrInfo printing_cerr_info;
#endif

/**
 * Declare the typelib and version numbers
 */
// {7ED2603F-68FD-479e-B9E3-DD7CFA8F16F3}
const GUID CDECL BASED_CODE _tlid = { 0x7ed2603f, 0x68fd, 0x479e,
  { 0xb9, 0xe3, 0xdd, 0x7c, 0xfa, 0x8f, 0x16, 0xf3 } };
const WORD _wVerMajor = 1;
const WORD _wVerMinor = 0;

// DLL initialization
BOOL CH3DPluginActiveXApp::InitInstance()
{
#ifdef HAVE_MEDX3D
  {
    auto_ptr< VolumeData > blaj( new VolumeData );
  }
#endif
  ProfilesAndComponents::check_profiles_components = false;
#ifdef PRINT_ERR_FILE
  cerr << "CH3DPluginActiveXApp::InitInstance" << endl;
#endif
  BOOL bInit = COleControlModule::InitInstance();
  return bInit;
}


// CH3DPluginActiveXApp::ExitInstance - DLL termination
int CH3DPluginActiveXApp::ExitInstance()
{
#ifdef PRINT_ERR_FILE
  cerr << "CH3DPluginActiveXApp::ExitInstance" << endl;
#endif
  return COleControlModule::ExitInstance();
}

std::map<const char *, const char *> getMimeTypes() {
  // Init the mimetypes we handle
  std::map<const char *, const char *> mimetypes;
  mimetypes["model/x3d+vrml"] = ".x3dv";
  mimetypes["model/x3d+xml"]  = ".x3d";
  mimetypes["model/vrml"]     = ".wrl";

  return mimetypes;
}

/**
 * See http://support.microsoft.com/kb/165072 for information on the registry
 entries used here
 */
HRESULT CreateRegistryEntries()
{
  // ActiveX control uuid
  const char *clsid = "{51EEAA22-604D-4b28-9FD3-98EF05FDDEA0}";
  std::map<const char *, const char *> mimetypes = getMimeTypes();  
  
  // Loop over the mimetypes and perform necessary registration for each type
  std::map<const char *, const char *>::iterator it;
  for( it = mimetypes.begin(); it != mimetypes.end(); ++it ) {
    const char *mime = it->first;
    const char *ext  = it->second;
    char subkey[256];
    HKEY hKey;
    // disposition & keyCreated are not really used right now but required to
    // find out if a key existed or not when we attempted to create it
    DWORD disposition;
    bool keyCreated = false;

    sprintf( subkey, "%s%s", "Mime\\Database\\Content Type\\", mime );
    // Register mimetype settings for this control
    DWORD status = ERROR_SUCCESS;

    status = RegCreateKeyEx( HKEY_CLASSES_ROOT, subkey, 0, NULL, 0, KEY_WRITE,
                             NULL, &hKey, (LPDWORD) &disposition);
    keyCreated = (disposition == REG_CREATED_NEW_KEY);
    if( ERROR_SUCCESS != status ) {
      // Failed to create/open key
      return E_UNEXPECTED;
    }

    status = RegSetValueEx( hKey, "Extension", 0, REG_SZ, (const BYTE *) ext,
                            strlen( ext ) );
    if( ERROR_SUCCESS !=  status ) {
      // Failed to create Extension value
      return E_UNEXPECTED;
    }

    status = RegSetValueEx( hKey, "CLSID", 0, REG_SZ, (const BYTE *) clsid,
                            strlen( clsid ) );
    if( ERROR_SUCCESS !=  status ) {
      // Failed to create CLSID value
      return E_UNEXPECTED;
    }

    RegCloseKey(hKey);
    
    status = RegCreateKeyEx( HKEY_CLASSES_ROOT, ext, 0, NULL, 0, KEY_WRITE,
                             NULL, &hKey, (LPDWORD) &disposition);
    keyCreated = (disposition == REG_CREATED_NEW_KEY);
    if( ERROR_SUCCESS != status ) {
      // Failed to create / open key
      return E_UNEXPECTED;
    }

    status = RegSetValueEx( hKey, "Content Type", 0, REG_SZ,
                            (const BYTE *) mime, strlen( mime ) );
    if( ERROR_SUCCESS !=  status ) {
      // Failed to create Content Type value
      char buffer[256];
      FormatMessage( FORMAT_MESSAGE_FROM_SYSTEM, NULL, status, 0,
                     buffer, 256, NULL );
      return E_UNEXPECTED;
    }

    RegCloseKey( hKey );

    sprintf( subkey, "CLSID\\%s\\EnableFullPage\\%s", clsid, ext );
    status = RegCreateKeyEx( HKEY_CLASSES_ROOT, subkey, 0, NULL, 0,
                             KEY_WRITE, NULL, &hKey, (LPDWORD) &disposition );
    keyCreated = (disposition == REG_CREATED_NEW_KEY);
    if( ERROR_SUCCESS != status ) {
      // Failed to create / open key
      return E_UNEXPECTED;
    }
    RegCloseKey( hKey );
  }

  return ERROR_SUCCESS;
}

HRESULT DeleteRegistryEntries() 
{
  std::map<const char *, const char *> mimetypes = getMimeTypes();
  // Loop over the mimetypes and perform necessary registration for each type
  std::map<const char *, const char *>::iterator it;
  for( it = mimetypes.begin(); it != mimetypes.end(); ++it ) {
    const char *mime = it->first;
    const char *ext  = it->second;
    char subkey[256];

    sprintf( subkey, "%s%s", "Mime\\Database\\Content Type\\", mime );
    if( ERROR_SUCCESS != RegDeleteKey( HKEY_CLASSES_ROOT, subkey ) ) {
      // Failed to clean the mime type registry entries
      return E_UNEXPECTED;
    }
    if( ERROR_SUCCESS != RegDeleteKey( HKEY_CLASSES_ROOT, ext ) ) {
      // Failed to clean the extension registry entries
      return E_UNEXPECTED;
    }
  }

  return ERROR_SUCCESS;
}

// DllRegisterServer - Adds entries to the system registry
STDAPI DllRegisterServer(void)
{
  AFX_MANAGE_STATE(_afxModuleAddrThis);
  if (!AfxOleRegisterTypeLib(AfxGetInstanceHandle(), _tlid))
    return ResultFromScode(SELFREG_E_TYPELIB);
  if (!COleObjectFactoryEx::UpdateRegistryAll(TRUE))
    return ResultFromScode(SELFREG_E_CLASS);
  
  if( ERROR_SUCCESS != CreateRegistryEntries() ) {
    return E_UNEXPECTED;
  }

  return NOERROR;
}


// DllUnregisterServer - Removes entries from the system registry
STDAPI DllUnregisterServer(void)
{
  AFX_MANAGE_STATE(_afxModuleAddrThis);
  if (!AfxOleUnregisterTypeLib(_tlid, _wVerMajor, _wVerMinor))
    return ResultFromScode(SELFREG_E_TYPELIB);
  if (!COleObjectFactoryEx::UpdateRegistryAll(FALSE))
    return ResultFromScode(SELFREG_E_CLASS);

  if( ERROR_SUCCESS != DeleteRegistryEntries() ) {
    return E_UNEXPECTED;
  }

  return NOERROR;
}
