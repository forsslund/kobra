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
/// \file ForceDimensionDevice.h
/// \brief Header file for ForceDimensionDevice.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FORCEDIMENSIONDEVICE_H__
#define __FORCEDIMENSIONDEVICE_H__

#include <H3D/H3DHapticsDevice.h>
#include <H3D/MFString.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/SFVec3d.h>

namespace H3D {

  /// \ingroup H3DNodes
  /// \class ForceDimensionDevice
  /// \brief A ForceDimensionDevice is a node for handling communication
  /// with a haptics device from ForceDimension, such as the Omega and
  /// Delta haptics devices.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../../H3DAPI/examples/All/ForceDimensionDevice.x3d">ForceDimensionDevice.x3d</a>
  ///     ( <a href="examples/ForceDimensionDevice.x3d.html">Source</a> )
  class H3DAPI_API ForceDimensionDevice: public H3DHapticsDevice {
  public:

    /// Reset specializes SFBool to go into reset mode when a true event
    /// is received.
    class H3DAPI_API Reset: public OnNewValueSField< SFBool > {
      virtual void onNewValue( const bool &v );
    };

    /// WaitReset specializes SFBool to go into wait on reset mode when
    /// a true event is received.
    class H3DAPI_API WaitReset: public OnNewValueSField< SFBool > {
      virtual void onNewValue( const bool &v );
    };

    /// GravityComp specializes SFBool to toggle gravity compensation 
    /// on and off.
    class H3DAPI_API GravityComp: public OnValueChangeSField< SFBool > {
      virtual void onValueChange( const bool &v );
    };

    /// EffectorMass specializes SFFloat to change the end effector 
    /// mass at change of value.
    class H3DAPI_API EffectorMass: public OnValueChangeSField< SFFloat > {
      virtual void onValueChange( const H3DFloat &v );
    };

    /// Brakes specializes SFBool to toggle brakes
    /// on and off.
    class H3DAPI_API Brakes: public OnValueChangeSField< SFBool > {
      virtual void onValueChange( const bool &v );
    };

    /// EnableForce specializes SFBool to toggle forces
    /// on and off.
    class H3DAPI_API EnableForce: public OnValueChangeSField< SFBool > {
    public:
      EnableForce(): OnValueChangeSField< SFBool >(), call_hapi_function( true ) {}
    
      bool call_hapi_function;
    protected:
      virtual void onValueChange( const bool &v );
    };

    /// SFAutoCalibrate specializes SFBool to go into auto calibrate mode
    /// when a true event is received.
    class H3DAPI_API SFAutoCalibrate: public OnNewValueSField< SFBool > {
      virtual void onNewValue( const bool &v );
    };

    /// SFReleasePosition specializes SFVec3d to go set the release
    /// position of the contained haptics device.
    class H3DAPI_API SFReleasePosition: public OnNewValueSField< SFVec3d > {
    public:
      /// Constructor
      SFReleasePosition() : value_is_set( false ) {}

      /// Just a flag used to check if the value was set before the
      /// hapi_device pointer was set.
      bool value_is_set;

    protected:
      virtual void onNewValue( const Vec3d &v );
    };

    /// Constructor.
    ForceDimensionDevice( 
            Inst< SFVec3f            > _devicePosition         = 0,
            Inst< SFRotation         > _deviceOrientation      = 0,
            Inst< TrackerPosition    > _trackerPosition        = 0,
            Inst< TrackerOrientation > _trackerOrientation     = 0,
            Inst< SFMatrix4f         > _positionCalibration    = 0,
            Inst< SFRotation         > _orientationCalibration = 0,
            Inst< SFVec3f            > _proxyPosition          = 0,
            Inst< WeightedProxy      > _weightedProxyPosition  = 0,     
            Inst< SFFloat            > _proxyWeighting         = 0,
            Inst< SFBool             > _mainButton            = 0,
            Inst< SFBool             > _secondaryButton       = 0,
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
            Inst< GravityComp        > _useGravityCompensation = 0,
            Inst< Reset              > _reset                  = 0,
            Inst< WaitReset          > _waitForReset           = 0,
            Inst< EffectorMass       > _endEffectorMass        = 0,
            Inst< Brakes             > _useBrakes              = 0,
            Inst< SFInt32            > _deviceType             = 0,
            Inst< EnableForce        > _enableForce            = 0,
            Inst< SFFloat            > _vibrationFrequency     = 0,
            Inst< SFFloat            > _vibrationAmplitude     = 0,
            Inst< SFFloat            > _gripperAngle           = 0,
            Inst< SFAutoCalibrate    > _autoCalibrate          = 0,
            Inst< SFBool             > _isAutoCalibrated       = 0,
            Inst< SFInt32            > _desiredComThreadRate   = 0,
            Inst< SFFloat            > _gripperForce           = 0,
            Inst< SFBool             > _flipGripperValues      = 0,
            Inst< SFReleasePosition  > _releaseDevicePosition  = 0 );

    /// Does all the initialization needed for the device before starting to
    /// use it.
    virtual ErrorCode initDevice();

    /// Perform cleanup and let go of all device resources that are allocated.
    /// After a call to this function no haptic rendering can be performed on
    /// the device until the initDevice() function has been called again.
    virtual ErrorCode releaseDevice();

    /// Enable/disable gravity compensation. A value of true enables it.
    /// When gravity compensation is enabled, the weights of the arms and of
    /// the end-effector are taken into account and a vertical force is 
    /// dynamically applied to the end-effector on top of the user command.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> true \n
    auto_ptr< SFBool > useGravityCompensation;

    /// When a true event is received, the device is put into RESET mode.
    /// In this mode, the user is expected to put the device end-effector
    /// at its rest position. This is how the device performs its calibration. 
    /// 
    /// <b>Access type:</b> inputOnly \n
    auto_ptr< Reset > reset;

    /// When a true event is received, the device is put into RESET mode 
    /// and wait for the user to calibrate the device.
    /// 
    /// <b>Access type:</b> inputOnly \n
    auto_ptr< WaitReset > waitForReset;

    /// Set the end effector mass used in gravity compensation in order
    /// to provide accurate gravity compensation when custom-made or 
    /// modified end-effectors are used.
    /// 
    /// <b>Access type:</b> inputOnly \n
    auto_ptr< SFFloat > endEffectorMass;

    /// Enable/disable the device electromagnetic brakes. If enabled
    /// the device motor circuits are shortcut to produce electromagnetic
    /// viscosity. The viscosity is sufficient to prevent the device from
    /// falling too hard onto if forces are disabled abruptly, either by
    /// pressing the force button or by action of a safety feature.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> true \n
    auto_ptr< SFBool > useBrakes;

    /// Contains the device type of the Force Dimension device.
    /// Possible values are:
    /// - DHD_DEVICE_3DOF - the Delta Haptic Device 3DOF
    /// - DHD_DEVICE_6DOF - the Delta Haptic Device 6DOF
    /// - DHD_DEVICE_OMEGA - the OMEGA Haptic Device
    /// - DHD_DEVICE_OMEGA3, DHD_DEVICE_OMEGA33, DHD_DEVICE_OMEGA331 - the 
    /// second generation OMEGA.X Haptic Devices
    /// - DHD_DEVICE_CONTROLLER - the Force Dimension stand-alone USB 2.0
    ///  controller (DHD_DEVICE_CONTROLLER) 
    /// - DHD_DEVICE_SIMULATOR - the Force Dimension haptic device
    /// simulator
    /// - DHD_DEVICE_CUSTOM - Unknown devices.
    /// - -1, device not initialized
    /// Any other number listed in the documentation for the installed version
    /// of dhd.dll on the system.
    /// This field is treated as a mix of an initializedOnly field and
    /// an outputOnly field. If it is set at X3D level it will be used to
    /// request a device of a specific type. After initialization of the
    /// device it is only used to report the device and any further changes
    /// to the field from outside this node will be ignored.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default type:</b> -1 \n
    auto_ptr< SFInt32 > deviceType;

    /// On some (custom) devices from force dimension there is no
    /// button to enable/disable forces on the device. Therefore it might
    /// be useful to be able to do this programatically to be able
    /// to use the device. This is the field to use in that case. Note that
    /// this field will not be kept accurate if there is a button on the
    /// device to enable/disable forces and might in such cases be
    /// out of sync with the actual state.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> false \n
    auto_ptr< EnableForce > enableForce;

    /// Node database entry
    static H3DNodeDatabase database;

    /// On some (custom) devices from force dimension there are vibration
    /// support. This field is used to control the frequency of the vibration.
    /// Only used if vibrationAmplitude is above 0.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 100 \n
    /// <b>Valid values:</b> [0, inf]
    auto_ptr< SFFloat > vibrationFrequency;

    /// On some (custom) devices from force dimension there are vibration
    /// support. This field is used to control the amplitude of the vibration.
    /// Negative values are clamped to 0 which means that vibrations is turned off.
    /// A value of 1 means that vibration is at full amplitude of the device.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 \n
    /// <b>Valid values:</b> [0, 1]
    auto_ptr< SFFloat > vibrationAmplitude;

    /// Output field for the gripper angle. Angle is in radians.
    ///
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFFloat > gripperAngle;

    /// This function is used to transfer device values, such as position, 
    /// button status etc from the realtime loop to the fields of H3DHapticsDevice,
    /// and possible vice versa. Overridden to transfer gripper angle.
    virtual void updateDeviceValues();

    protected:
      /// EnableForce specializes SFFloat to handle vibration.
    class H3DAPI_API ChangeVibration: public PeriodicUpdate< TypedField< SFBool, Types< SFFloat, SFFloat > > > {
    protected:
      virtual void update();
    };

    auto_ptr< ChangeVibration > changeVibration;
    
    public:
    /// When a true event is received, the device is auto calibrated.
    /// This will not do anything unless H3DAPI is compiled with DRD api support
    /// (HAVE_DRDAPI defined).
    /// After the device is calibrated forces will be enabled.
    ///
    /// <b>Access type:</b> inputOnly \n
    /// <b>Default value:</b> false \n
    auto_ptr< SFAutoCalibrate > autoCalibrate;

    /// Indicates whether the device is calibrated.
    ///
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFBool > isAutoCalibrated;

    /// The desired frequency for the thread that communicates with the haptics device.
    /// If value is -1 then the communication is done directly in the haptics thread
    /// otherwise the communication is done in a separate thread at the given rate
    /// and locks are used to transfer values to the haptics thread.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value type:</b> 1000 \n
    auto_ptr< SFInt32 > desiredComThreadRate;

    /// Output field for the gripper force. Force is in Newton
    ///
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFFloat > gripperForce;

    /// If true then the gripper angle output is negated with respect to what
    /// the underlying dhd api reports. Same thing happens for the gripper force.
    /// Might be useful in some cases when there are two
    /// devices of a different type that should control a similar feature.
    /// with the gripper angles.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> false \n
    auto_ptr< SFBool > flipGripperValues;

    /// If this value is set, and HAPI was compiled with DRD support then
    /// when the device is released the device will be moved to this position.
    /// Note that the position is given in device space. If this field is not
    /// set explicitly by the user then the device will not move to any position
    /// when released.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> Vec3d( 0, 0, 0 ) \n
    auto_ptr< SFReleasePosition > releaseDevicePosition;

    /// Creates a ForceDimensionHapticsDevice in the hapi_device
    /// with deviceType
    virtual void initialize();
  };
}

#endif
