//////////////////////////////////////////////////////////////////////////////
//    Copyright 2018-2019, SenseGraphics AB
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
/// \file NvidiaGraphicsOptions.cpp
/// \brief CPP file for NvidiaGraphicsOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/NvidiaGraphicsOptions.h>
#ifdef HAVE_NVAPI



using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase NvidiaGraphicsOptions::database(
                                      "NvidiaGraphicsOptions", 
                                      &(newInstance<NvidiaGraphicsOptions>),
                                      typeid( NvidiaGraphicsOptions ),
                                      &X3DChildNode::database );

namespace NvidiaGraphicsOptionsInternals {
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, settingsApplyingStatus, OUTPUT_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, threadedOptimization, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, threadedOptimizationCurrent, OUTPUT_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, verticalSync, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, verticalSyncCurrent, OUTPUT_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, fxaa, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, fxaaCurrent, OUTPUT_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, powerMode, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, powerModeCurrent, OUTPUT_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, shaderCache, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, shaderCacheCurrent, OUTPUT_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, tripleBuffering, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( NvidiaGraphicsOptions, tripleBufferingCurrent, OUTPUT_ONLY )
}

NvidiaGraphicsOptions::NvidiaGraphicsOptions( 
  Inst< SFNode>  _metadata,
  Inst< SFString > _settingsApplyingStatus,
  Inst< SFString > _threadedOptimization,
  Inst< SFString > _threadedOptimizationCurrent,
  Inst< SFString > _verticalSync,
  Inst< SFString > _verticalSyncCurrent,
  Inst< SFString > _fxaa,
  Inst< SFString > _fxaaCurrent,
  Inst< SFString > _powerMode,
  Inst< SFString > _powerModeCurrent,
  Inst< SFString > _shaderCache,
  Inst< SFString > _shaderCacheCurrent,
  Inst< SFString > _tripleBuffering,
  Inst< SFString > _tripleBufferingCurrent ) :
  X3DChildNode( _metadata ),
  settingsApplyingStatus( _settingsApplyingStatus ),
  threadedOptimization( _threadedOptimization ),
  threadedOptimizationCurrent( _threadedOptimizationCurrent ),
  verticalSync( _verticalSync ),
  verticalSyncCurrent( _verticalSyncCurrent ),
  fxaa( _fxaa ),
  fxaaCurrent( _fxaaCurrent ),
  powerMode( _powerMode ),
  powerModeCurrent( _powerModeCurrent ),
  shaderCache( _shaderCache ),
  shaderCacheCurrent( _shaderCacheCurrent ),
  tripleBuffering( _tripleBuffering ),
  tripleBufferingCurrent( _tripleBufferingCurrent ),
  setting_applied(false),
  options_applying_result(""){
  
  type_name = "NvidiaGraphicsOptions";
  database.initFields( this );

  settingsApplyingStatus->setValue( "UNINITIALIZED", id );

  threadedOptimization->addValidValue("ON");
  threadedOptimization->addValidValue("OFF");
  threadedOptimization->addValidValue("AUTO");
  threadedOptimization->addValidValue("GLOBAL");
  threadedOptimization->setValue( "GLOBAL", id );

  threadedOptimizationCurrent->addValidValue("ON");
  threadedOptimizationCurrent->addValidValue("OFF");
  threadedOptimizationCurrent->addValidValue("AUTO");
  threadedOptimizationCurrent->addValidValue("UNKNOWN");
  threadedOptimizationCurrent->setValue( "UNKNOWN", id );

  verticalSync->addValidValue("ON");
  verticalSync->addValidValue("OFF");
  verticalSync->addValidValue("ADAPTIVE");
  verticalSync->addValidValue("ADAPTIVE_HALF");
  verticalSync->addValidValue("GLOBAL");
  verticalSync->setValue( "GLOBAL", id );

  verticalSyncCurrent->addValidValue("ON");
  verticalSyncCurrent->addValidValue("OFF");
  verticalSyncCurrent->addValidValue("ADAPTIVE");
  verticalSyncCurrent->addValidValue("ADAPTIVE_HALF");
  verticalSyncCurrent->addValidValue("UNKNOWN");
  verticalSyncCurrent->setValue( "UNKNOWN", id );

  fxaa->addValidValue("ON");
  fxaa->addValidValue("OFF");
  fxaa->addValidValue("GLOBAL");
  fxaa->setValue( "GLOBAL", id );

  fxaaCurrent->addValidValue("ON");
  fxaaCurrent->addValidValue("OFF");
  fxaaCurrent->addValidValue("UNKNOWN");
  fxaaCurrent->setValue( "UNKNOWN", id );

  powerMode->addValidValue("OPTIMAL");
  powerMode->addValidValue("ADAPTIVE");
  powerMode->addValidValue("PERFORMANCE");
  powerMode->addValidValue("GLOBAL");
  powerMode->setValue( "GLOBAL", id );

  powerModeCurrent->addValidValue("OPTIMAL");
  powerModeCurrent->addValidValue("ADAPTIVE");
  powerModeCurrent->addValidValue("PERFORMANCE");
  powerModeCurrent->addValidValue("UNKNOWN");
  powerModeCurrent->setValue( "UNKNOWN", id );

  shaderCache->addValidValue("ON");
  shaderCache->addValidValue("OFF");
  shaderCache->addValidValue("GLOBAL");
  shaderCache->setValue( "GLOBAL", id );

  shaderCacheCurrent->addValidValue("ON");
  shaderCacheCurrent->addValidValue("OFF");
  shaderCacheCurrent->addValidValue("UNKNOWN");
  shaderCacheCurrent->setValue( "UNKNOWN", id );

  tripleBuffering->addValidValue("ON");
  tripleBuffering->addValidValue("OFF");
  tripleBuffering->addValidValue("GLOBAL");
  tripleBuffering->setValue( "GLOBAL" );

  tripleBufferingCurrent->addValidValue("ON");
  tripleBufferingCurrent->addValidValue("OFF");
  tripleBufferingCurrent->addValidValue("UNKNOWN");
  tripleBufferingCurrent->setValue( "UNKNOWN", id );
}

H3D::NvidiaGraphicsOptions::~NvidiaGraphicsOptions() {
  // delete the temporary profile created
  // This solution can not really guarantee that it can always
  // reliable delete the temporary profile created
  // INIT NVAPI
  NvAPI_Status status = NVAPI_OK;
  status = NvAPI_Initialize();
  printNVAPIError( status, __LINE__ );
  // create NVAPI session handle
  NvDRSSessionHandle hSession = 0;
  status = NvAPI_DRS_CreateSession( &hSession );
  printNVAPIError( status, __LINE__ );

  // load all existing settings into the session
  status = NvAPI_DRS_LoadSettings( hSession );
  printNVAPIError( status, __LINE__ );

  // if H3D_Temp_profile exist, then delete it
  NvDRSProfileHandle hProfile = 0;
  NvAPI_UnicodeString profile_name;
  setNVUstring( profile_name, L"H3D_Temp_profile" );
  status = NvAPI_DRS_FindProfileByName( hSession, profile_name, &hProfile );
  if( status == NVAPI_OK ) {
    // delete if exist
    status = NvAPI_DRS_DeleteProfile( hSession, hProfile );
    printNVAPIError( status, __LINE__ );
    // save after delete
    status = NvAPI_DRS_SaveSettings( hSession );
    printNVAPIError( status, __LINE__ );
  }
  // destroy the session
  NvAPI_DRS_DestroySession( hSession );
  hSession = 0;

}

void H3D::NvidiaGraphicsOptions::initialize() {
  X3DChildNode::initialize();
  applySettings();
}

void H3D::NvidiaGraphicsOptions::applySettings( ) {
  if( setting_applied ) {
    return;
  }



  // INIT NVAPI
  NvAPI_Status status = NVAPI_OK;
  status = NvAPI_Initialize();
  options_applying_result += printNVAPIError( status, __LINE__ );

  // create NVAPI session handle
  NvDRSSessionHandle hSession = 0;
  status = NvAPI_DRS_CreateSession( &hSession );
  options_applying_result += printNVAPIError( status, __LINE__ );

  // load all existing settings into the session
  status = NvAPI_DRS_LoadSettings( hSession );
  options_applying_result += printNVAPIError( status, __LINE__ );

  // if H3D_Temp_profile somehow already exist, then delete it
  NvDRSProfileHandle hProfile = 0;

  NvAPI_UnicodeString profile_name;
  setNVUstring( profile_name, L"H3D_Temp_profile" );
  status = NvAPI_DRS_FindProfileByName( hSession, profile_name, &hProfile );
  if( status == NVAPI_OK ) {
    // delete if exist
    status = NvAPI_DRS_DeleteProfile( hSession, hProfile );
    options_applying_result += printNVAPIError( status, __LINE__ );
    // save all settings to apply the deletion
    status = NvAPI_DRS_SaveSettings( hSession );
    options_applying_result += printNVAPIError( status, __LINE__ );
    // reload all settings
    status = NvAPI_DRS_LoadSettings( hSession );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }

  // create a new profile 
  NVDRS_PROFILE drsProfile = { 0 };
  drsProfile.version = NVDRS_PROFILE_VER;
  drsProfile.isPredefined = 0;
  setNVUstring( drsProfile.profileName, L"H3D_Temp_profile" );
  status = NvAPI_DRS_CreateProfile( hSession, &drsProfile, &hProfile );
  options_applying_result += printNVAPIError( status, __LINE__ );

  // create application H3DViewer and H3DLoad and connect to the profile H3D_Temp_profile
  NVDRS_APPLICATION app;
  app.version = NVDRS_APPLICATION_VER;
  app.isPredefined = 0;

  
  wchar_t current_app[MAX_PATH];

  if( GetModuleFileNameW( 0, current_app, MAX_PATH ) ) {
    setNVUstring( app.appName, current_app );
    setNVUstring( app.userFriendlyName, current_app );
    status = NvAPI_DRS_CreateApplication( hSession, hProfile, &app );
    options_applying_result += printNVAPIError( status, __LINE__ );
  } else {
    setNVUstring( app.appName, L"H3DViewer.exe" );
    setNVUstring( app.userFriendlyName, L"H3DViewer" );
    status = NvAPI_DRS_CreateApplication( hSession, hProfile, &app );
    options_applying_result += printNVAPIError( status, __LINE__ );

    setNVUstring( app.appName, L"H3DViewer_d.exe" );
    setNVUstring( app.userFriendlyName, L"H3DViewer debug" );
    status = NvAPI_DRS_CreateApplication( hSession, hProfile, &app );
    options_applying_result += printNVAPIError( status, __LINE__ );

    setNVUstring( app.appName, L"H3DLoad.exe" );
    setNVUstring( app.userFriendlyName, L"H3DLoad" );
    status = NvAPI_DRS_CreateApplication( hSession, hProfile, &app );
    options_applying_result += printNVAPIError( status, __LINE__ );

    setNVUstring( app.appName, L"H3DLoad_d.exe" );
    setNVUstring( app.userFriendlyName, L"H3DLoad debug" );
    status = NvAPI_DRS_CreateApplication( hSession, hProfile, &app );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }



  NVDRS_SETTING drsSetting = { 0 };
  drsSetting.version = NVDRS_SETTING_VER;



  // setting for threadedOptimization
  drsSetting = NVDRS_SETTING();
  drsSetting.version = NVDRS_SETTING_VER;
  drsSetting.settingId = OGL_THREAD_CONTROL_ID;
  drsSetting.settingType = NVDRS_DWORD_TYPE;
  const string threaded_optimization = threadedOptimization->getValue();
  if( "GLOBAL"!= threaded_optimization ) {
    if( "ON" == threaded_optimization ) {
      drsSetting.u32CurrentValue = OGL_THREAD_CONTROL_ENABLE;
      drsSetting.u32PredefinedValue = OGL_THREAD_CONTROL_ENABLE;
    } else if( "OFF" == threaded_optimization ) {
      drsSetting.u32CurrentValue = OGL_THREAD_CONTROL_DISABLE;
      drsSetting.u32PredefinedValue = OGL_THREAD_CONTROL_DISABLE;
    } else {
      drsSetting.u32CurrentValue = OGL_THREAD_CONTROL_DEFAULT;
      drsSetting.u32PredefinedValue = OGL_THREAD_CONTROL_DEFAULT;
    }

    status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }

  // read current setting
  NvAPI_DRS_GetSetting( hSession, hProfile, OGL_THREAD_CONTROL_ID, &drsSetting );
  if( drsSetting.u32CurrentValue == OGL_THREAD_CONTROL_ENABLE ) {
    threadedOptimizationCurrent->setValue( "ON", id );
  } else if( drsSetting.u32CurrentValue == OGL_THREAD_CONTROL_DISABLE ) {
    threadedOptimizationCurrent->setValue( "OFF", id );
  } else if( drsSetting.u32CurrentValue == OGL_THREAD_CONTROL_DEFAULT ) {
    threadedOptimizationCurrent->setValue( "AUTO", id );
  }

  const string vertical_sync = verticalSync->getValue();
  // setting for verticalSync
  if( "GLOBAL" != vertical_sync ) {
    drsSetting = NVDRS_SETTING();
    drsSetting.version = NVDRS_SETTING_VER;
    if( "ON" == vertical_sync ) {
      drsSetting.settingId = VSYNCTEARCONTROL_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCTEARCONTROL_DISABLE;
      status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
      options_applying_result += printNVAPIError( status, __LINE__ );

      drsSetting.settingId = VSYNCMODE_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCMODE_FORCEON;
    } else if( "OFF" == vertical_sync ) {
      drsSetting.settingId = VSYNCTEARCONTROL_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCTEARCONTROL_DISABLE;
      status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
      options_applying_result += printNVAPIError( status, __LINE__ );

      drsSetting.settingId = VSYNCMODE_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCMODE_FORCEOFF;

    } else if( "ADAPTIVE" == vertical_sync ) {
      drsSetting.settingId = VSYNCTEARCONTROL_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCTEARCONTROL_ENABLE;
      status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
      options_applying_result += printNVAPIError( status, __LINE__ );

      drsSetting.settingId = VSYNCMODE_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCMODE_FORCEON;

    } else if( "ADAPTIVE_HALF" == vertical_sync ) {
      drsSetting.settingId = VSYNCTEARCONTROL_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCTEARCONTROL_ENABLE;

      status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
      options_applying_result += printNVAPIError( status, __LINE__ );

      drsSetting.settingId = VSYNCMODE_ID;
      drsSetting.settingType = NVDRS_DWORD_TYPE;
      drsSetting.u32CurrentValue = VSYNCMODE_FLIPINTERVAL2;
    }
    status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }

  // read current setting
  NVDRS_SETTING vsync_tear_ctrl = { 0 };
  vsync_tear_ctrl.version = NVDRS_SETTING_VER;
  vsync_tear_ctrl.settingId = VSYNCTEARCONTROL_ID;
  vsync_tear_ctrl.settingType = NVDRS_DWORD_TYPE;
  NvAPI_DRS_GetSetting( hSession, hProfile, VSYNCTEARCONTROL_ID, &vsync_tear_ctrl );

  NVDRS_SETTING vsync_mode = { 0 };
  vsync_mode.version = NVDRS_SETTING_VER;
  vsync_mode.settingId = VSYNCMODE_ID;
  vsync_mode.settingType = NVDRS_DWORD_TYPE;
  NvAPI_DRS_GetSetting( hSession, hProfile, VSYNCMODE_ID, &vsync_mode );

  if( vsync_tear_ctrl.u32CurrentValue == VSYNCTEARCONTROL_DISABLE && 
      vsync_mode.u32CurrentValue == VSYNCMODE_FORCEON ) {
    verticalSyncCurrent->setValue( "ON", id );
  } else if( vsync_tear_ctrl.u32CurrentValue == VSYNCTEARCONTROL_DISABLE && 
             vsync_mode.u32CurrentValue == VSYNCMODE_FORCEOFF ) {
    verticalSyncCurrent->setValue( "OFF", id );
  } else if( vsync_tear_ctrl.u32CurrentValue == VSYNCTEARCONTROL_ENABLE &&
             vsync_mode.u32CurrentValue == VSYNCMODE_FORCEON ) {
    verticalSyncCurrent->setValue( "ADAPTIVE", id );
  } else if( vsync_tear_ctrl.u32CurrentValue == VSYNCTEARCONTROL_ENABLE &&
             vsync_mode.u32CurrentValue == VSYNCMODE_FLIPINTERVAL2 ) {
    verticalSyncCurrent->setValue( "ADAPTIVE_HALF", id );
  }
  
  // setting for fxaa
  drsSetting = NVDRS_SETTING();
  drsSetting.version = NVDRS_SETTING_VER;
  drsSetting.settingId = FXAA_ENABLE_ID;
  drsSetting.settingType = NVDRS_DWORD_TYPE;
  const string fxaa_setting = fxaa->getValue();
  if( "GLOBAL"!=fxaa_setting ) {
    if( "ON" == fxaa_setting ) {
      drsSetting.u32CurrentValue = FXAA_ENABLE_ON;
    } else if( "OFF" == fxaa_setting ) {
      drsSetting.u32CurrentValue = FXAA_ENABLE_OFF;
    }
    status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }
  
  // read current setting
  NvAPI_DRS_GetSetting( hSession, hProfile, FXAA_ENABLE_ID, &drsSetting );
  if( drsSetting.u32CurrentValue == FXAA_ENABLE_ON ) {
    fxaaCurrent->setValue( "ON", id );
  } else if( drsSetting.u32CurrentValue == FXAA_ENABLE_OFF ) {
    fxaaCurrent->setValue( "OFF", id );
  }

  // setting for power mode
  drsSetting = NVDRS_SETTING();
  drsSetting.version = NVDRS_SETTING_VER;
  drsSetting.settingId = PREFERRED_PSTATE_ID;
  drsSetting.settingType = NVDRS_DWORD_TYPE;
  const string power_mode = powerMode->getValue();
  if( "GLOBAL"!=power_mode ) {
    if( "OPTIMAL" == power_mode ) {
      drsSetting.u32CurrentValue = PREFERRED_PSTATE_OPTIMAL_POWER;
    } else if( "ADAPTIVE" == power_mode ) {
      drsSetting.u32CurrentValue = PREFERRED_PSTATE_ADAPTIVE;
    } else if( "PERFORMANCE" == power_mode ) {
      drsSetting.u32CurrentValue = PREFERRED_PSTATE_PREFER_MAX;
    }
    status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }

  // read current setting
  NvAPI_DRS_GetSetting( hSession, hProfile, PREFERRED_PSTATE_ID, &drsSetting );
  if( drsSetting.u32CurrentValue == PREFERRED_PSTATE_OPTIMAL_POWER ) {
    powerModeCurrent->setValue( "OPTIMAL", id );
  } else if( drsSetting.u32CurrentValue == PREFERRED_PSTATE_ADAPTIVE ) {
    powerModeCurrent->setValue( "ADAPTIVE", id );
  } else if( drsSetting.u32CurrentValue == PREFERRED_PSTATE_PREFER_MAX ) {
    powerModeCurrent->setValue( "PERFORMANCE", id );
  }

  // setting for shaderCache
  drsSetting = NVDRS_SETTING();
  drsSetting.version = NVDRS_SETTING_VER;
  drsSetting.settingId = PS_SHADERDISKCACHE_ID;
  drsSetting.settingType = NVDRS_DWORD_TYPE;
  const string shader_cache = shaderCache->getValue();
  if( "GLOBAL"!=shader_cache ) {
    if( "ON" == shader_cache ) {
      drsSetting.u32CurrentValue = PS_SHADERDISKCACHE_ON;
    } else if( "OFF" == shader_cache ) {
      drsSetting.u32CurrentValue = PS_SHADERDISKCACHE_OFF;
    }
    status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }
  
  // read current setting
  NvAPI_DRS_GetSetting( hSession, hProfile, PS_SHADERDISKCACHE_ID, &drsSetting );
  if( drsSetting.u32CurrentValue == PS_SHADERDISKCACHE_ON ) {
    shaderCacheCurrent->setValue( "ON", id );
  } else if( drsSetting.u32CurrentValue == PS_SHADERDISKCACHE_OFF ) {
    shaderCacheCurrent->setValue( "OFF", id );
  }

  // setting for tripleBuffering
  drsSetting = NVDRS_SETTING();
  drsSetting.version = NVDRS_SETTING_VER;
  drsSetting.settingId = OGL_TRIPLE_BUFFER_ID;
  drsSetting.settingType = NVDRS_DWORD_TYPE;
  const string triple_buffering = tripleBuffering->getValue();
  if( "GLOBAL"!=triple_buffering ) {
    if( "ON" == triple_buffering ) {
      drsSetting.u32CurrentValue = OGL_TRIPLE_BUFFER_ENABLED;
    } else if( "OFF" == triple_buffering ) {
      drsSetting.u32CurrentValue = OGL_TRIPLE_BUFFER_DISABLED;
    }
    status = NvAPI_DRS_SetSetting( hSession, hProfile, &drsSetting );
    options_applying_result += printNVAPIError( status, __LINE__ );
  }
  
  // read current setting
  NvAPI_DRS_GetSetting( hSession, hProfile, OGL_TRIPLE_BUFFER_ID, &drsSetting );
  if( drsSetting.u32CurrentValue == OGL_TRIPLE_BUFFER_ENABLED ) {
    tripleBufferingCurrent->setValue( "ON", id );
  } else if( drsSetting.u32CurrentValue == OGL_TRIPLE_BUFFER_DISABLED ) {
    tripleBufferingCurrent->setValue( "OFF", id );
  }



  status = NvAPI_DRS_SaveSettings( hSession );
  options_applying_result += printNVAPIError( status, __LINE__ );

  NvAPI_DRS_DestroySession( hSession );
  hSession = 0;

  if( options_applying_result.empty() ) {
    settingsApplyingStatus->setValue( "SUCCEEDED", id );
  } else {
    settingsApplyingStatus->setValue( "FAILED:" + options_applying_result, id );
  }
  setting_applied = true;

}

string H3D::NvidiaGraphicsOptions::printNVAPIError( const NvAPI_Status& err, const int& line ) {
  NvAPI_ShortString err_msg;
  if( err !=NVAPI_OK ) {
    NvAPI_GetErrorMessage( err, err_msg );
    stringstream ss;
    ss << line << " : "<< err_msg << std::endl;
    return  ss.str();
  }
  return "";
}


void H3D::NvidiaGraphicsOptions::setNVUstring( NvAPI_UnicodeString& nvStr, const wchar_t* wcStr ) {
  for( int i = 0; i < NVAPI_UNICODE_STRING_MAX; i++ )
    nvStr[i] = 0;

  int i = 0;
  while( wcStr[i] != 0 && i <NVAPI_UNICODE_STRING_MAX ) {
    nvStr[i] = wcStr[i];
    i++;
  }
  if( NVAPI_UNICODE_STRING_MAX==i ) {
    Console( LogLevel::Error ) << "Specified NVUstring is exceeding its length capacity!" << std::endl;
  }
}

#endif // HAVE_NVAPI
