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
/// \file VirtuoseDevice.h
/// \brief Header file for VirtuoseDevice.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __VIRTUOSEDEVICE_H__
#define __VIRTUOSEDEVICE_H__

#include <H3D/H3DHapticsDevice.h>
#include <H3D/SFString.h>

namespace H3D {

  /// \ingroup H3DNodes
  /// \class VirtuoseDevice
  /// \brief A VirtuoseDevice is a node for handling communication
  /// with a haptics device from Haption, such as the Virtuose haptics device.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../../H3DAPI/examples/All/VirtuoseDevice.x3d">VirtuoseDevice.x3d</a>
  ///     ( <a href="examples/VirtuoseDevice.x3d.html">Source</a> )
  class H3DAPI_API VirtuoseDevice: public H3DHapticsDevice {
  public:

    /// Constructor.
    VirtuoseDevice( 
            Inst< SFVec3f            > _devicePosition         = 0,
            Inst< SFRotation         > _deviceOrientation      = 0,
            Inst< TrackerPosition    > _trackerPosition        = 0,
            Inst< TrackerOrientation > _trackerOrientation     = 0,
            Inst< SFMatrix4f         > _positionCalibration    = 0,
            Inst< SFRotation         > _orientationCalibration = 0,
            Inst< SFVec3f            > _proxyPosition          = 0,
            Inst< WeightedProxy      > _weightedProxyPosition  = 0,
            Inst< SFFloat            > _proxyWeighting         = 0,
            Inst< SFBool             > _mainButton             = 0,
            Inst< SFBool             > _secondaryButton        = 0,
            Inst< SFInt32            > _buttons                = 0,
            Inst< SFVec3f            > _force                  = 0,
            Inst< SFVec3f            > _torque                 = 0,
            Inst< SFInt32            > _inputDOF               = 0,
            Inst< SFInt32            > _outputDOF              = 0,
            Inst< SFInt32            > _hapticsRate            = 0,
            Inst< SFInt32            > _desiredHapticsRate     = 0,
            Inst< SFNode             > _stylus                 = 0,
            Inst< SFHapticsRendererNode > _hapticsRenderer     = 0,
            Inst< MFVec3f            > _proxyPositions         = 0,
            Inst< SFBool             > _followViewpoint        = 0,
            Inst< SFString           > _ipAddress              = 0 );
    
    /// Creates a VirtuoseHapticsDevice in the hapi_device
    /// with name deviceName
    virtual void initialize();

    /// The ip address of the Virtuose device.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> "192.168.1.1" \n
    auto_ptr< SFString > ipAddress;

    /// Node database entry
    static H3DNodeDatabase database;
  };
}

#endif
