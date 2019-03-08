//////////////////////////////////////////////////////////////////////////////
//    Copyright 2015-2019, SenseGraphics AB
//
//    This file is part of H3DUtil.
//
//    H3DUtil is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3DUtil is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3DUtil; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file DualQuaternion.cpp
/// \brief .cpp file for DualQuaternion.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3DUtil/DualQuaternion.h>
//#include <H3DUtil/DualQuaterniond.h>
#include <H3DUtil/Matrix4f.h>
#include <H3DUtil/Matrix4d.h>

using namespace H3DUtil;
using namespace ArithmeticTypes;

      
DualQuaternion::DualQuaternion( const Matrix4d &m ) {
  Matrix4f matrix = (Matrix4f)m;
  *this = DualQuaternion( matrix );
}

// Constructor. From Matrix3f that is a rotation matrix. 
DualQuaternion::DualQuaternion( const Matrix4f &m ) {
  q0 = Quaternion( m.getRotationPart() );
  Vec3f t ( m.getTranslationPart() );
  qe = 0.5 * Quaternion( t.x, t.y, t.z, 0 ) * q0;
}

