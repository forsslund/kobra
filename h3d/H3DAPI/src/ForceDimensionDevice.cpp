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
/// \file ForceDimensionDevice.cpp
/// \brief cpp file for ForceDimensionDevice
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/ForceDimensionDevice.h>
#ifdef HAVE_DHDAPI
#include <HAPI/ForceDimensionHapticsDevice.h>
#endif

using namespace H3D;

H3DNodeDatabase ForceDimensionDevice::database( "ForceDimensionDevice", 
                                            &(newInstance<ForceDimensionDevice>),
                                            typeid( ForceDimensionDevice ),
                                            &H3DHapticsDevice::database ); 
namespace ForceDimensionDeviceInternals {
  FIELDDB_ELEMENT( ForceDimensionDevice, useGravityCompensation, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ForceDimensionDevice, reset, INPUT_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, waitForReset, INPUT_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, endEffectorMass, INPUT_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, useBrakes, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ForceDimensionDevice, deviceType, OUTPUT_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, enableForce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ForceDimensionDevice, vibrationFrequency, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ForceDimensionDevice, vibrationAmplitude, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ForceDimensionDevice, gripperAngle, OUTPUT_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, autoCalibrate, INPUT_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, isAutoCalibrated, OUTPUT_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, desiredComThreadRate, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( ForceDimensionDevice, gripperForce, OUTPUT_ONLY )
}

/// Constructor.
ForceDimensionDevice::ForceDimensionDevice( 
               Inst< SFVec3f         > _devicePosition,
               Inst< SFRotation      > _deviceOrientation,
               Inst< TrackerPosition > _trackerPosition,
               Inst< TrackerOrientation > _trackerOrientation,
               Inst< SFMatrix4f      > _positionCalibration,
               Inst< SFRotation      > _orientationCalibration,
               Inst< SFVec3f         > _proxyPosition,
               Inst< WeightedProxy   > _weightedProxyPosition,
               Inst< SFFloat         > _proxyWeighting,
               Inst< SFBool          > _mainButton,
               Inst< SFBool          > _secondaryButton,
               Inst< SFInt32         > _buttons,
               Inst< SFVec3f         > _force,
               Inst< SFVec3f         > _torque,
               Inst< SFInt32         > _inputDOF,
               Inst< SFInt32         > _outputDOF,
               Inst< SFInt32         > _hapticsRate,
               Inst< SFInt32         > _desiredHapticsRate,
               Inst< SFNode          > _stylus,
               Inst< SFHapticsRendererNode > _hapticsRenderer,
               Inst< MFVec3f         > _proxyPositions,
               Inst< SFBool          > _followViewpoint,
               Inst< GravityComp     > _useGravityCompensation,
               Inst< Reset           > _reset,
               Inst< WaitReset       > _waitForReset,
               Inst< EffectorMass    > _endEffectorMass,
               Inst< Brakes          > _useBrakes,
               Inst< SFInt32         > _deviceType,
               Inst< EnableForce        > _enableForce,
               Inst< SFFloat            > _vibrationFrequency,
               Inst< SFFloat            > _vibrationAmplitude,
               Inst< SFFloat            > _gripperAngle,
               Inst< SFAutoCalibrate    > _autoCalibrate,
               Inst< SFBool             > _isAutoCalibrated,
               Inst< SFInt32            > _desiredComThreadRate,
               Inst< SFFloat            > _gripperForce ) :
  H3DHapticsDevice( _devicePosition, _deviceOrientation, _trackerPosition,
              _trackerOrientation, _positionCalibration, 
              _orientationCalibration, _proxyPosition,
              _weightedProxyPosition, _proxyWeighting, _mainButton,
                    _secondaryButton, _buttons,
              _force, _torque, _inputDOF, _outputDOF, _hapticsRate,
              _desiredHapticsRate, _stylus,_hapticsRenderer, _proxyPositions,
              _followViewpoint ),
  useGravityCompensation( _useGravityCompensation ),
  reset( _reset ),
  waitForReset( _waitForReset ),
  endEffectorMass( _endEffectorMass ),
  useBrakes( _useBrakes ),
  deviceType( _deviceType ),
  enableForce( _enableForce ),
  vibrationFrequency( _vibrationFrequency ),
  vibrationAmplitude( _vibrationAmplitude ),
  changeVibration( new ChangeVibration ),
  gripperAngle( _gripperAngle ),
  autoCalibrate( _autoCalibrate ),
  isAutoCalibrated( _isAutoCalibrated ),
  desiredComThreadRate( _desiredComThreadRate ),
  gripperForce( _gripperForce ) {

  type_name = "ForceDimensionDevice";
  database.initFields( this );

#ifdef HAVE_DHDAPI
  hapi_device.reset( new HAPI::ForceDimensionHapticsDevice );
#else
  Console(LogLevel::Error) << "Cannot use ForceDimensionDevice. HAPI compiled without"
       << " DHDAPI support. Recompile HAPI with HAVE_DHDAPI defined"
       << " in order to use it." << endl;
#endif

  gripperAngle->setValue( 0, id );
  gripperForce->setValue( 0, id );

  useGravityCompensation->setValue( true );
  useBrakes->setValue( false );
  deviceType->setValue( -1, id );
  enableForce->setValue( false );
  vibrationFrequency->setValue( 100 );
  vibrationAmplitude->setValue( 0 );
  changeVibration->setOwner( this );
  changeVibration->setName( "changeVibration" );
  vibrationFrequency->routeNoEvent( changeVibration );
  vibrationAmplitude->routeNoEvent( changeVibration );
  autoCalibrate->setValue( false );
  isAutoCalibrated->setValue( false, id );
  desiredComThreadRate->setValue( 1000 );
}


void ForceDimensionDevice::Reset::onNewValue( const bool &v ) {
#ifdef HAVE_DHDAPI
  ForceDimensionDevice *fd = 
    static_cast< ForceDimensionDevice * >( getOwner() );
  HAPI::ForceDimensionHapticsDevice * dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fd->hapi_device.get() );
  if( v ) dhd->reset();
#endif
}

void ForceDimensionDevice::WaitReset::onNewValue( const bool &v ) {
#ifdef HAVE_DHDAPI
  ForceDimensionDevice *fd = 
    static_cast< ForceDimensionDevice * >( getOwner() );
  HAPI::ForceDimensionHapticsDevice * dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fd->hapi_device.get() );
  if( v ) dhd->waitForReset();
#endif
}

void ForceDimensionDevice::GravityComp::onValueChange( const bool &v ) {
#ifdef HAVE_DHDAPI
  ForceDimensionDevice *fd = 
    static_cast< ForceDimensionDevice * >( getOwner() );
  HAPI::ForceDimensionHapticsDevice * dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fd->hapi_device.get() );
  dhd->useGravityCompensation( v );
#endif
}

void ForceDimensionDevice::EffectorMass::onValueChange( const H3DFloat &v ) {
#ifdef HAVE_DHDAPI
  ForceDimensionDevice *fd = 
    static_cast< ForceDimensionDevice * >( getOwner() );
  HAPI::ForceDimensionHapticsDevice * dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fd->hapi_device.get() );
  dhd->setEffectorMass( v );
#endif
}

void ForceDimensionDevice::Brakes::onValueChange( const bool &v ) {
#ifdef HAVE_DHDAPI
  ForceDimensionDevice *fd = 
    static_cast< ForceDimensionDevice * >( getOwner() );
  HAPI::ForceDimensionHapticsDevice * dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fd->hapi_device.get() );
  dhd->useBrakes( v );
#endif
}

void ForceDimensionDevice::EnableForce::onValueChange( const bool &v ) {
#ifdef HAVE_DHDAPI
  ForceDimensionDevice *fd = 
    static_cast< ForceDimensionDevice * >( getOwner() );
  HAPI::ForceDimensionHapticsDevice * dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fd->hapi_device.get() );
  if( call_hapi_function ) {
    dhd->enableForce( v );
  }
#endif
}

H3DHapticsDevice::ErrorCode ForceDimensionDevice::initDevice() {
#ifdef HAVE_DHDAPI
  HAPI::ForceDimensionHapticsDevice *dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( hapi_device.get() );
  if( dhd ) {
    dhd->setComThreadFrequency( desiredComThreadRate->getValue() );
  }
#endif
  HAPI::HAPIHapticsDevice::ErrorCode e = H3DHapticsDevice::initDevice();
#ifdef HAVE_DHDAPI
  if( dhd ) {
    deviceType->setValue( dhd->getDeviceType(), id ); 
    dhd->enableForce( enableForce->getValue() );
    if( e == HAPI::HAPIHapticsDevice::SUCCESS && autoCalibrate->getValue(id) && !dhd->isAutoCalibrated() ) {
      dhd->autoCalibrate();
    }
  }
#endif
  return e;
}

H3DHapticsDevice::ErrorCode ForceDimensionDevice::releaseDevice() {
  HAPI::HAPIHapticsDevice::ErrorCode e = H3DHapticsDevice::releaseDevice();
  deviceType->setValue( -1, id );
  return e;
}

void ForceDimensionDevice::ChangeVibration::update() {
#ifdef HAVE_DHDAPI
  H3DFloat frequency = static_cast<SFFloat *>(routes_in[0])->getValue();
  H3DFloat amplitude = static_cast<SFFloat *>(routes_in[1])->getValue();
  ForceDimensionDevice * fdd = static_cast<ForceDimensionDevice *>(getOwner());
  HAPI::ForceDimensionHapticsDevice *dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fdd->hapi_device.get() );
  if( dhd ) {
    if( frequency < 0 ) {
      frequency = 0;
    }
    if( amplitude < 0 ) {
      amplitude = 0;
    } else if( amplitude > 1 ) {
      amplitude = 1;
    }
    dhd->setVibration( frequency, amplitude );
  }
#endif
}

void ForceDimensionDevice::updateDeviceValues() {
  H3DHapticsDevice::updateDeviceValues();
#ifdef HAVE_DHDAPI
  HAPI::ForceDimensionHapticsDevice *dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( hapi_device.get() );
  if( dhd ) {
    HAPI::HAPIHapticsDevice::DeviceValues dv = hapi_device->getDeviceValues();
    gripperAngle->setValue( (H3DFloat)(dv.dof7_angle), id );
    gripperForce->setValue( (H3DFloat)(dv.dof7_force), id );
    bool is_auto_calibrated = dhd->isAutoCalibrated();
    if( isAutoCalibrated->getValue() != is_auto_calibrated ) {
      isAutoCalibrated->setValue( is_auto_calibrated, id );
    }
  }
#endif
}

void ForceDimensionDevice::SFAutoCalibrate::onNewValue( const bool &v ) {
#ifdef HAVE_DHDAPI
  ForceDimensionDevice *fd = 
    static_cast< ForceDimensionDevice * >( getOwner() );
  HAPI::ForceDimensionHapticsDevice * dhd = 
    static_cast< HAPI::ForceDimensionHapticsDevice * >( fd->hapi_device.get() );
  if( v ) {
    dhd->autoCalibrate();
    fd->enableForce->call_hapi_function = false;
    fd->enableForce->setValue(true);
    fd->enableForce->call_hapi_function = true;
  }
#endif
}
