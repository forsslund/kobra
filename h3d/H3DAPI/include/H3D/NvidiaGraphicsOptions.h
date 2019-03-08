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
/// \file NvidiaGraphicsOptions.h
/// \brief Header file for NvidiaGraphicsOptions.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __NVIDIAGRAPHICSOPTIONS_H__
#define __NVIDIAGRAPHICSOPTIONS_H__

#include <H3D/H3DApi.h>
#ifdef HAVE_NVAPI

#include <H3D/X3DChildNode.h>
#include <H3D/SFString.h>
#include <H3D/SFInt32.h>
#include <nvapi.h>
#include <NvApiDriverSettings.h>

namespace H3D {

  /// \ingroup H3DNodes
  /// \class NvidiaGraphicsOptions
  /// \brief Specifies parameters to use for the NVIDIA graphic settings
  /// which normally can also be changed from NVIDIA control panel. 
  /// For all other fields which is not specified in this node, it will use the
  /// global NVIDIA 3d settings.
  /// This node only works under windows system as NVAPI is windows only
  /// NVAPI is an optional library to H3DAPI, please make sure you have it on your system
  /// and it is selected during compiling.
  /// This node will create a temporary profile named H3D_Temp_profile and
  /// associate it with the current running application, or with H3DLoad, H3DViewer if current
  /// application name can not be retrieved . This profile will be deleted
  /// after application is closed if it is closed normally.
  /// This node can only affect the OpenGL context if it is initialized before the
  /// OpenGL context is initialized. Therefore it is expected to exist in the path
  /// specified by the static variable Scene::nvidia_graphics_options_file_url
  /// By default this path is set to NVSettings.x3d
  class H3DAPI_API NvidiaGraphicsOptions : public X3DChildNode {
  public:

    /// Constructor.
    NvidiaGraphicsOptions( Inst< SFNode   > _metadata = 0,
      Inst< SFString > _settingsApplyingStatus = 0,
      Inst< SFString > _threadedOptimization = 0,
      Inst< SFString > _threadedOptimizationCurrent = 0,
      Inst< SFString > _verticalSync = 0,
      Inst< SFString > _verticalSyncCurrent = 0,
      Inst< SFString > _fxaa = 0,
      Inst< SFString > _fxaaCurrent = 0,
      Inst< SFString > _powerMode = 0,
      Inst< SFString > _powerModeCurrent = 0,
      Inst< SFString > _shaderCache = 0,
      Inst< SFString > _shaderCacheCurrent = 0,
      Inst< SFString > _tripleBuffering = 0,
      Inst< SFString > _tripleBufferingCurrent = 0 ) ;

    ~NvidiaGraphicsOptions();
    
    /// Settings are applied when the node is initialized
    virtual void initialize();

    /// Output only field which indicate whether the settings are applied and whether 
    /// if they are applied successfully
    /// <b>Access type:</b> outputOnly
    auto_ptr<SFString> settingsApplyingStatus;


    /// Field controls the thread optimization field of NVIDIA 3d setting
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> "GLOBAL"
    /// <b>Valid values:</b> "ON", "OFF", "AUTO", "GLOBAL"
    auto_ptr <SFString> threadedOptimization;

    /// Reports the current value for the thread optimization field of NVIDIA 3d setting
    /// <b>Access type:</b> outputOnly
    /// <b>Default value:</b> "UNKNOWN"
    /// <b>Valid values:</b> "ON", "OFF", "AUTO", "UNKNOWN"
    auto_ptr <SFString> threadedOptimizationCurrent;

    /// Field controls the vertical sync field of NVIDIA 3d setting
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> "GLOBAL"
    /// <b>Valid values:</b> "ON","OFF","ADAPTIVE","ADAPTIVE_HALF", "GLOBAL"
    auto_ptr < SFString> verticalSync;

    /// Reports the current value for the vertical sync field of NVIDIA 3d setting
    /// <b>Access type:</b> outputOnly
    /// <b>Default value:</b> "UNKNOWN"
    /// <b>Valid values:</b> "ON", "OFF", "ADAPTIVE", "ADAPTIVE_HALF", "UNKNOWN"
    auto_ptr < SFString> verticalSyncCurrent;

    /// Field controls the fxaa field of NVIDIA 3d setting
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> "GLOBAL"
    /// <b>Valid values:</b> "ON","OFF", "GLOBAL"
    auto_ptr<SFString> fxaa;

    /// Reports the current value for the fxaa field of NVIDIA 3d setting
    /// <b>Access type:</b> outputOnly
    /// <b>Default value:</b> "UNKNOWN"
    /// <b>Valid values:</b> "ON", "OFF", "UNKNOWN"
    auto_ptr<SFString> fxaaCurrent;

    /// Field controls the power management mode of NVIDIA 3d setting
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> "GLOBAL"
    /// <b>Valid values:</b> "OPTIMAL", "ADAPTIVE", "PERFORMANCE", "GLOBAL"
    auto_ptr<SFString> powerMode;

    /// Reports the current value for the power management mode of NVIDIA 3d setting
    /// <b>Access type:</b> outputOnly
    /// <b>Default value:</b> "UNKNOWN"
    /// <b>Valid values:</b> "OPTIMAL", "ADAPTIVE", "PERFORMANCE", "UNKNOWN"
    auto_ptr<SFString> powerModeCurrent;

    /// Field controls the shader cache field of NVIDIA 3d setting
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> "GLOBAL"
    /// <b>Valid values:</b> "ON","OFF", "GLOBAL"
    auto_ptr<SFString> shaderCache;

    /// Reports the current value for the shader cache field of NVIDIA 3d setting
    /// <b>Access type:</b> outputOnly
    /// <b>Default value:</b> "UNKNOWN"
    /// <b>Valid values:</b> "ON", "OFF", "UNKNOWN"
    auto_ptr<SFString> shaderCacheCurrent;

    /// Field controls the triple buffering field of NVIDIA 3d setting
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> "GLOBAL"
    /// <b>Valid values:</b> "ON","OFF", "GLOBAL"
    auto_ptr<SFString> tripleBuffering;

    /// Reports the current value for the triple buffering field of NVIDIA 3d setting
    /// <b>Access type:</b> outputOnly
    /// <b>Default value:</b> "UNKNOWN"
    /// <b>Valid values:</b> "ON", "OFF", "UNKNOWN" 
    auto_ptr<SFString> tripleBufferingCurrent;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:
    void applySettings();

    string printNVAPIError( const NvAPI_Status& err, const int& line );
    void setNVUstring( NvAPI_UnicodeString& nvStr, const wchar_t* wcStr );
    bool setting_applied;
    string options_applying_result;
  };
}

#endif // HAVE_NVAPI

#endif
