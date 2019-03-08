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
/// \file keys.cpp
/// \brief CPP file with function which handles keys for the plugin.
/// NOTE: Only implemented for Windows.
///
//
//////////////////////////////////////////////////////////////////////////////
#include "keys.h"
#include "H3D/KeySensor.h"

#ifdef H3D_WINDOWS

using namespace H3D;
/* Returns -1 if we didn't find 'special' keycode */
/* Ugly to use keysensor codes here but for now it should work */
int getKeyFromKeycode( UINT keycode ) {
  int key = -1;
  switch( keycode ) {
    case VK_F1:    key = KeySensor::F1; break;
    case VK_F2:    key = KeySensor::F2; break;
    case VK_F3:    key = KeySensor::F3; break;
    case VK_F4:    key = KeySensor::F4; break;
    case VK_F5:    key = KeySensor::F5; break;
    case VK_F6:    key = KeySensor::F6; break;
    case VK_F7:    key = KeySensor::F7; break;
    case VK_F8:    key = KeySensor::F8; break;
    case VK_F9:    key = KeySensor::F9; break;
    case VK_F10:  key = KeySensor::F10; break;
    case VK_F11:  key = KeySensor::F11; break;
    case VK_F12:  key = KeySensor::F12; break;
    case VK_HOME:  key = KeySensor::HOME; break;
    case VK_END:  key = KeySensor::END; break;
    case VK_PRIOR:  key = KeySensor::PGUP; break;
    case VK_NEXT:  key = KeySensor::PGDN; break;
    case VK_UP:    key = KeySensor::UP; break;
    case VK_DOWN:  key = KeySensor::DOWN; break;
    case VK_LEFT:  key = KeySensor::LEFT; break;
    case VK_RIGHT:  key = KeySensor::RIGHT; break;
    case VK_SHIFT:  key = KeySensor::SHIFT; break;
    case VK_CONTROL: key = KeySensor::CONTROL; break;
    case VK_MENU:  key = KeySensor::ALT; break;
  }

  return key;
}

#endif
