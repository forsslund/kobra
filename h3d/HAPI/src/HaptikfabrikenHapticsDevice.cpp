//////////////////////////////////////////////////////////////////////////////
//    Copyright 2019, SenseGraphics AB
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file HaptikfabrikenHapticsDevice.cpp
/// \brief Cpp file for HaptikfabrikenHapticsDevice.
///
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HaptikfabrikenHapticsDevice.h>
#ifdef HAVE_HAPTIKFABRIKENAPI
#ifdef WIN32
#define WINDOWS
#endif
#include "uhaptikfabriken2.h"
#include <sstream>

using namespace HAPI;
using namespace haptikfabriken;

HAPIHapticsDevice::HapticsDeviceRegistration 
HaptikfabrikenHapticsDevice::device_registration
( "HaptikfabrikenHapticsDevice",
  &(newInstance< HaptikfabrikenHapticsDevice >));

/// Constructor. device_index is the index of falcon device
/// connected. Should not be larger than getNrConnectedDevices() - 1.
HaptikfabrikenHapticsDevice::HaptikfabrikenHapticsDevice(unsigned int device_index ,
                                                         bool wait_for_next_message,
                                                         std::string configuration):
  index( device_index ),
  hfab( 0 ),
  wait_for_next_message(wait_for_next_message),
  configuration(configuration){ // maybe new
  
  max_stiffness = 1500;

  std::cout << "HaptikfabrikenHapticsDevice::HaptikfabrikenHapticsDevice()\n";
  std::cout << haptikfabriken::version << '\n';
}

HaptikfabrikenHapticsDevice::~HaptikfabrikenHapticsDevice() {
    //hfab->close();
}

bool HaptikfabrikenHapticsDevice::initHapticsDevice( int _thread_frequency ) {
    std::cout << "HaptikfabrikenHapticsDevice::initHapticsDevice (" << configuration << ")\n";
  hfab.reset(new HaptikfabrikenInterface());
  HaptikfabrikenInterface::findUSBSerialDevices();
  if(hfab->open()) {
    std::stringstream s;
    s << "Cannot open Haptikfabriken device (index " << index << ") - Error: "
      << hfab->getErrorCode()
      << ". Make sure you have the device connected properly "
      << "and have the permissions to communicate over the USB "
      << "port. " << std::endl;
    setErrorMsg( s.str() );
    return false;
  }

  return true;
}

bool HaptikfabrikenHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  hfab->close();
  hfab.reset(0);
    
  return true;
}

void HaptikfabrikenHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                                HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );


  fsVec3d p = hfab->getPos();

  // Adjust offset to match Taiwan Kobra standard
  p = p + fsVec3d(0.3415, -0.182, 0.0542);
  
  // Adjust offset for desktop use with Kobra simulation cases ("chai" units)
  //p = p + fsVec3d(0.29, -0.03, 0.17);
  

  fsRot r = hfab->getRot();

  Matrix3 ChaiToH3d(0,1,0,  // h3d x is chai y (second column)
                    0,0,1,  // h3d y is chai z (third column)
                    1,0,0); // h3d z is chai x (first column)
  //Where is chai:  x y z

  dv.position = ChaiToH3d * Vec3(p.x(),p.y(),p.z());
  //std::cout << "Calculated position (H3D): " << dv.position.x << "," << dv.position.y << "," << dv.position.z << "\n";

  Matrix3 chaiRot(r.m[0][0], r.m[0][1], r.m[0][2],
                  r.m[1][0], r.m[1][1], r.m[1][2],
                  r.m[2][0], r.m[2][1], r.m[2][2]);


  // rotate about x 180
  /*
  Matrix3 rotx180(1,0,0,
                  0,-1,0,
                  0,0,-1);
                  */

  // rotate about x 90
  Matrix3 rotx90(1,0,0,
                  0,0,-1,
                  0,1,0);

  // rotate about x 90
  Matrix3 roty90(0,0,1,
                  0,1,0,
                  -1,0,0);

  // rotate about z 90
  /*
  Matrix3 rotz90(0,-1,0,
                 1,0,0,
                 0,0,1);
  */


  Matrix3 h3dRot = ChaiToH3d * chaiRot * rotx90 * roty90;

  // In Kobra the orientationCalibration is "1 0 0 1.0472" = 60 degrees abt x
  // theta = -1.0472; cos(t)=0.5, sin(t)=-0.8.., -cos(t)=-0.5, -sin(t)=0.8
  /*
  Matrix3 rotx_minus_60(1,0,0,
                        0,0.5,0.866026628,
                        0,-0.866026628,0.5);                  
  h3dRot = rotx_minus_60*h3dRot;
  */


  dv.orientation = Rotation(h3dRot);





  calculateVelocity(dv, dt);
  // TODO: Implement switches
  //dv.button_status = hfab->getSwitchesState().to_ulong(); // bitmask to be filled
}

void HaptikfabrikenHapticsDevice::sendOutput( DeviceOutput &dv,
                                        HAPITime dt ) {

  Matrix3 H3DtoChai(0,0,1,
                    1,0,0,
                    0,1,0);
  Vec3 f = H3DtoChai * dv.force;

  hfab->setForce(fsVec3d(f.x, f.y, f.z));
  //current_values.torque = dv.torque;
}

#endif
