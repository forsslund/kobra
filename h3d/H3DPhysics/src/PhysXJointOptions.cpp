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
/// \file PhysXJointOptions.cpp
/// \brief Source file for PhysXJointOptions, implementing possible joint options nodes
/// handled by PhysX.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysXJointOptions.h>
#include <H3D/H3DPhysics/PhysXJointParameters.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PhysXJointOptions::database( "PhysXJointOptions",
                                             "PhysX3JointOptions",
                                              &newInstance<PhysXJointOptions>,
                                              typeid(PhysXJointOptions),
                                              &H3DEngineOptions::database );

namespace PhysXJointOptionsInternals {
  FIELDDB_ELEMENT( PhysXJointOptions, projectionTolerance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJointOptions, projectionFlag, INPUT_OUTPUT )
  FIELDDB_ELEMENT_EX( PhysXJointOptions, projectionFlag, INPUT_OUTPUT, constraintFlag )
  FIELDDB_ELEMENT( PhysXJointOptions, enabledProjection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJointOptions, enabledCollision, INPUT_OUTPUT )
}

PhysXJointOptions::PhysXJointOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFVec2f > _projectionTolerance,
  Inst< SFString > _projectionFlag,
  Inst< SFBool > _enabledProjection,
  Inst< SFBool > _enabledCollision ) :
  H3DEngineOptions( _metadata, _valueUpdater ),
  projectionTolerance( _projectionTolerance ),
  projectionFlag( _projectionFlag ),
  enabledProjection( _enabledProjection ),
  enabledCollision( _enabledCollision ) {

  type_name = "PhysXJointOptions";
  database.initFields( this );

  projectionTolerance->setValue( Vec2f( 1e10f, (H3DFloat)Constants::pi ) );
  projectionFlag->addValidValue( "ePROJECTION" );
  projectionFlag->addValidValue( "ePROJECT_TO_ACTOR0" );
  projectionFlag->addValidValue( "ePROJECT_TO_ACTOR1" );
  projectionFlag->setValue( "ePROJECTION" );
  enabledProjection->setValue( false );

  enabledCollision->setValue( false );

  projectionTolerance->route( valueUpdater );
  projectionFlag->route( valueUpdater );
  enabledProjection->route( valueUpdater );
  enabledCollision->route( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* PhysXJointOptions::getParameters( bool all_params ) {
  PhysXJointParameters* params = new PhysXJointParameters;

  if( !params ) return NULL;

  if( all_params || valueUpdater->hasCausedEvent( projectionTolerance ) ) {
    params->setProjectionTolerance( projectionTolerance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( enabledProjection ) || valueUpdater->hasCausedEvent( projectionFlag ) ) {
    params->setProjectionFlag( projectionFlag->getValue() );
    params->setEnabledProjection( enabledProjection->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( enabledCollision ) ) {
    params->setEnabledCollision( enabledCollision->getValue() );
  }

  return params;
}

H3DNodeDatabase PhysXSliderJointOptions::database( "PhysXSliderJointOptions",
                                                    &newInstance<PhysXSliderJointOptions>,
                                                    typeid(PhysXSliderJointOptions),
                                                    &PhysXJointOptions::database );

namespace PhysXSliderJointOptionsInternals {
  FIELDDB_ELEMENT( PhysXSliderJointOptions, explicitAnchorPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXSliderJointOptions, body1Offset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXSliderJointOptions, body2Offset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXSliderJointOptions, body2ForceScale, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXSliderJointOptions, forceType, INPUT_OUTPUT )
}

PhysXSliderJointOptions::PhysXSliderJointOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFVec3f > _explicitAnchorPoint,
  Inst< SFVec3f > _body1Offset,
  Inst< SFVec3f > _body2Offset,
  Inst< SFFloat > _body2ForceScale,
  Inst< SFString > _forceType ) :
  PhysXJointOptions( _metadata, _valueUpdater ),
  explicitAnchorPoint( _explicitAnchorPoint ),
  body1Offset( _body1Offset ),
  body2Offset( _body2Offset ),
  body2ForceScale( _body2ForceScale ),
  forceType( _forceType ) {

  type_name = "PhysXSliderJointOptions";
  database.initFields( this );

  explicitAnchorPoint->setValue( Vec3f() );
  body1Offset->setValue( Vec3f() );
  body2Offset->setValue( Vec3f() );
  body2ForceScale->setValue( 1.0 );
  forceType->setValue( "eForce" );

  explicitAnchorPoint->route( valueUpdater );
  body1Offset->route( valueUpdater );
  body2Offset->route( valueUpdater );
  body2ForceScale->route( valueUpdater );
  forceType->route( valueUpdater );

}

PhysicsEngineParameters::EngineOptionParameters* PhysXSliderJointOptions::getParameters( bool all_params ) {
  PhysXSliderJointParameters* params = new PhysXSliderJointParameters;

  if( all_params || valueUpdater->hasCausedEvent( explicitAnchorPoint ) ) {
    params->setExplicitAnchorPoint( explicitAnchorPoint->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( body1Offset ) ) {
    params->setBody1Offset( body1Offset->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( body2Offset ) ) {
    params->setBody2Offset( body2Offset->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( body2ForceScale ) ) {
    params->setBody2ForceScale( body2ForceScale->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( forceType ) ) {
    params->setForceType( forceType->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( projectionTolerance ) ) {
    params->setProjectionTolerance( projectionTolerance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( enabledProjection ) || valueUpdater->hasCausedEvent( projectionFlag ) ) {
    params->setProjectionFlag( projectionFlag->getValue() );
    params->setEnabledProjection( enabledProjection->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( enabledCollision ) ) {
    params->setEnabledCollision( enabledCollision->getValue() );
  }

  return params;
}

H3DNodeDatabase PhysXJoint6DOFLimitOptions::database( "PhysXJoint6DOFLimitOptions",
                                                      "PhysX3Joint6DOFLimitOptions",
                                                       &newInstance<PhysXJoint6DOFLimitOptions>,
                                                       typeid(PhysXJoint6DOFLimitOptions),
                                                       &PhysXJointOptions::database );

namespace PhysXJoint6DOFLimitOptionsInternals {
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, linear_x, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, linear_y, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, linear_z, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, angular_x, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, angular_y, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, angular_z, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, linearSpring, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, angularSpring, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, breakForce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXJoint6DOFLimitOptions, contactDistance, INPUT_OUTPUT )
}

PhysXJoint6DOFLimitOptions::PhysXJoint6DOFLimitOptions(
  Inst< SFNode  > _metadata,
  Inst< ValueUpdater   > _valueUpdater,
  Inst< SFVec2f > _linear_x,
  Inst< SFVec2f > _linear_y,
  Inst< SFVec2f > _linear_z,
  Inst< SFVec2f > _angular_x,
  Inst< SFVec2f > _angular_y,
  Inst< SFVec2f > _angular_z,
  Inst< SFVec2f > _linearSpring,
  Inst< SFVec2f > _angularSpring,
  Inst< SFVec2f > _breakForce,
  Inst< SFVec2f > _contactDistance ) :
  PhysXJointOptions( _metadata, _valueUpdater ),
  linear_x( _linear_x ),
  linear_y( _linear_y ),
  linear_z( _linear_z ),
  angular_x( _angular_x ),
  angular_y( _angular_y ),
  angular_z( _angular_z ),
  linearSpring( _linearSpring ),
  angularSpring( _angularSpring ),
  breakForce( _breakForce ),
  contactDistance( _contactDistance ) {

  type_name = "PhysXJoint6DOFLimitOptions";
  database.initFields( this );

  linear_x->setValue( Vec2f() );
  linear_y->setValue( Vec2f() );
  linear_z->setValue( Vec2f() );
  angular_x->setValue( Vec2f() );
  angular_y->setValue( Vec2f() );
  angular_z->setValue( Vec2f() );

  linearSpring->setValue( Vec2f() );
  angularSpring->setValue( Vec2f() );
  breakForce->setValue( Vec2f( -1.0f, -1.0f ) );
  contactDistance->setValue( Vec2f( 0.01f, 5.0f ) );

  linear_x->route( valueUpdater );
  linear_y->route( valueUpdater );
  linear_z->route( valueUpdater );
  angular_x->route( valueUpdater );
  angular_y->route( valueUpdater );
  angular_z->route( valueUpdater );
  linearSpring->route( valueUpdater );
  angularSpring->route( valueUpdater );
  breakForce->route( valueUpdater );
  contactDistance->route( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* PhysXJoint6DOFLimitOptions::getParameters( bool all_params ) {

  PhysXJoint6DOFLimitParameters* params = new PhysXJoint6DOFLimitParameters;

  if( all_params || valueUpdater->hasCausedEvent( projectionTolerance ) ) {
    params->setProjectionTolerance( projectionTolerance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( enabledProjection ) || valueUpdater->hasCausedEvent( projectionFlag ) ) {
    params->setProjectionFlag( projectionFlag->getValue() );
    params->setEnabledProjection( enabledProjection->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( enabledCollision ) ) {
    params->setEnabledCollision( enabledCollision->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( linear_x ) ) {
    params->setLinearX( linear_x->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( linear_y ) ) {
    params->setLinearY( linear_y->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( linear_z ) ) {
    params->setLinearZ( linear_z->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( angular_x ) ) {
    params->setAngularX( angular_x->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( angular_y ) ) {
    params->setAngularY( angular_y->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( angular_z ) ) {
    params->setAngularZ( angular_z->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( linearSpring ) ) {
    params->setLinearSpring( linearSpring->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( angularSpring ) ) {
    params->setAngularSpring( angularSpring->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( breakForce ) ) {
    params->setBreakForce( breakForce->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( contactDistance ) ) {
    params->setContactDistance( contactDistance->getValue() );
  }

  return params;
}
