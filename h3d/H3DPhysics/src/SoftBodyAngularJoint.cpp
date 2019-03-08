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
/// \file SoftBodyAngularJoint.cpp
/// \brief Source file for SoftBodyAngularJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/SoftBodyAngularJoint.h>

using namespace H3D;

H3DNodeDatabase SoftBodyAngularJoint::database( "SoftBodyAngularJoint", 
                                               &(newInstance<SoftBodyAngularJoint>), 
                                               typeid( SoftBodyAngularJoint ),
                                               &H3DSoftBodyJointNode::database);

namespace SoftBodyAngularJointInternals {
  FIELDDB_ELEMENT( SoftBodyAngularJoint, axis, INPUT_OUTPUT )
}

SoftBodyAngularJoint::SoftBodyAngularJoint(
  Inst< SFNode            > _metadata,
  Inst< ValueUpdater      > _value_updater,
  Inst< SFH3DSoftBodyNode > _body1,
  Inst< SFH3DBodyNode     > _body2,
  Inst< MFString          > _forceOutput,
  Inst< MFEngineOptions   > _engineOptions,
  Inst< SFVec3f           > _axis,
  Inst< SFTransformNode > _transform ):
H3DSoftBodyJointNode ( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
axis ( _axis )
{
  type_name = "SoftBodyAngularJoint";
  database.initFields( this );

  axis->setValue ( Vec3f ( 1, 0, 0 ) );
  axis->route ( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* SoftBodyAngularJoint::createConstraintParameters() {
  return new SoftBodyAngularJointParameters();
}

PhysicsEngineParameters::ConstraintParameters* SoftBodyAngularJoint::getConstraintParameters( bool all_params ) {

  SoftBodyAngularJointParameters* params= 
    static_cast<SoftBodyAngularJointParameters*>(H3DSoftBodyJointNode::getConstraintParameters(all_params));
  if( !params )
    return NULL;

  if ( all_params || valueUpdater->hasCausedEvent ( axis ) ) {
    params->setAxis ( axis->getValue() );
  }

  return params;
}

void SoftBodyAngularJoint::applyTransform ( const Matrix4f& _transform ) { 
  axis->setValue( _transform.getRotationPart()*axis->getValue() );
}
