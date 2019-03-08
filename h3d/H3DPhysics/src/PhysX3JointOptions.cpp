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
/// \file PhysX3JointOptions.cpp
/// \brief Source file for PhysX3JointOptions, implementing possible joint options nodes
/// handled by PhysX3.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysX3JointOptions.h>
#include <H3D/H3DPhysics/PhysX3JointParameters.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PhysX3JointOptions::database( "PhysX3JointOptions",
                                              &newInstance<PhysX3JointOptions>,
                                              typeid(PhysX3JointOptions),
                                              &H3DEngineOptions::database );

namespace PhysX3JointOptionsInternals {
  FIELDDB_ELEMENT( PhysX3JointOptions, projectionTolerance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3JointOptions, constraintFlag, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3JointOptions, enabledProjection, INPUT_OUTPUT )
}

PhysX3JointOptions::PhysX3JointOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFVec2f > _projectionTolerance,
  Inst< SFString > _constraintFlag,
  Inst< SFBool > _enabledProjection ) :
  H3DEngineOptions( _metadata, _valueUpdater ),
  projectionTolerance( _projectionTolerance ),
  constraintFlag( _constraintFlag ),
  enabledProjection( _enabledProjection ) {

  type_name = "PhysX3JointOptions";
  database.initFields( this );

  projectionTolerance->setValue( Vec2f( 1e10f, (H3DFloat)Constants::pi ) );
  constraintFlag->addValidValue( "ePROJECTION" );
  constraintFlag->addValidValue( "ePROJECT_TO_ACTOR0" );
  constraintFlag->addValidValue( "ePROJECT_TO_ACTOR1" );
  constraintFlag->setValue( "ePROJECTION" );
  enabledProjection->setValue( false );

  projectionTolerance->route( valueUpdater );
  constraintFlag->route( valueUpdater );
  enabledProjection->route( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* PhysX3JointOptions::getParameters( bool all_params ) {
  PhysX3JointParameters* params = new PhysX3JointParameters;

  if( !params ) return NULL;

  if( all_params || valueUpdater->hasCausedEvent( projectionTolerance ) ) {
    params->setProjectionTolerance( projectionTolerance->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( enabledProjection ) || valueUpdater->hasCausedEvent( constraintFlag ) ) {
    params->setConstraintFlag( constraintFlag->getValue() );
    params->setEnabledProjection( enabledProjection->getValue() );
  }

  return params;
}

H3DNodeDatabase PhysX3SliderJointOptions::database( "PhysX3SliderJointOptions",
                                                    &newInstance<PhysX3SliderJointOptions>,
                                                    typeid(PhysX3SliderJointOptions),
                                                    &PhysX3JointOptions::database );

namespace PhysX3SliderJointOptionsInternals {
  FIELDDB_ELEMENT( PhysX3SliderJointOptions, explicitAnchorPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3SliderJointOptions, body1Offset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3SliderJointOptions, body2Offset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3SliderJointOptions, body2ForceScale, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3SliderJointOptions, forceType, INPUT_OUTPUT )
}

PhysX3SliderJointOptions::PhysX3SliderJointOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFVec3f > _explicitAnchorPoint,
  Inst< SFVec3f > _body1Offset,
  Inst< SFVec3f > _body2Offset,
  Inst< SFFloat > _body2ForceScale,
  Inst< SFString > _forceType ) :
  PhysX3JointOptions( _metadata, _valueUpdater ),
  explicitAnchorPoint( _explicitAnchorPoint ),
  body1Offset( _body1Offset ),
  body2Offset( _body2Offset ),
  body2ForceScale( _body2ForceScale ),
  forceType( _forceType ) {

  type_name = "PhysX3SliderJointOptions";
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

PhysicsEngineParameters::EngineOptionParameters* PhysX3SliderJointOptions::getParameters( bool all_params ) {
  PhysX3SliderJointParameters* params = new PhysX3SliderJointParameters;

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
  if( all_params || valueUpdater->hasCausedEvent( enabledProjection ) || valueUpdater->hasCausedEvent( constraintFlag ) ) {
    params->setConstraintFlag( constraintFlag->getValue() );
    params->setEnabledProjection( enabledProjection->getValue() );
  }

  return params;
}

H3DNodeDatabase PhysX3Joint6DOFLimitOptions::database( "PhysX3Joint6DOFLimitOptions",
                                                       &newInstance<PhysX3Joint6DOFLimitOptions>,
                                                       typeid(PhysX3Joint6DOFLimitOptions),
                                                       &PhysX3JointOptions::database );

namespace PhysX3Joint6DOFLimitOptionsInternals {
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, linear_x, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, linear_y, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, linear_z, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, angular_x, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, angular_y, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, angular_z, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, linearSpring, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, angularSpring, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, breakForce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3Joint6DOFLimitOptions, contactDistance, INPUT_OUTPUT )
}

PhysX3Joint6DOFLimitOptions::PhysX3Joint6DOFLimitOptions(
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
  PhysX3JointOptions( _metadata, _valueUpdater ),
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

  type_name = "PhysX3Joint6DOFLimitOptions";
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

PhysicsEngineParameters::EngineOptionParameters* PhysX3Joint6DOFLimitOptions::getParameters( bool all_params ) {

  PhysX3Joint6DOFLimitParameters* params = new PhysX3Joint6DOFLimitParameters;

  if( all_params || valueUpdater->hasCausedEvent( projectionTolerance ) ) {
    params->setProjectionTolerance( projectionTolerance->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( enabledProjection ) || valueUpdater->hasCausedEvent( constraintFlag ) ) {
    params->setConstraintFlag( constraintFlag->getValue() );
    params->setEnabledProjection( enabledProjection->getValue() );
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
