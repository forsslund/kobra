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
/// \file SingleAxisHingeJoint.cpp
/// \brief Source file for SingleAxisHingeJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/SingleAxisHingeJoint.h>

using namespace H3D;

H3DNodeDatabase SingleAxisHingeJoint::database( "SingleAxisHingeJoint", 
                                               &(newInstance< SingleAxisHingeJoint >), 
                                               typeid( SingleAxisHingeJoint ),
                                               &H3DRigidBodyJointNode::database);

namespace SingleAxisHingeJointInternals {
  FIELDDB_ELEMENT( SingleAxisHingeJoint, anchorPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, axis, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, maxAngle, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, minAngle, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, stopBounce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, softness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, bias, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, stopErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, angle, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, angleRate, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, body1AnchorPoint, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SingleAxisHingeJoint, body2AnchorPoint, OUTPUT_ONLY )
}

SingleAxisHingeJoint::SingleAxisHingeJoint(
  Inst< SFNode  > _metadata,
  Inst< ValueUpdater > _value_updater,
  Inst< SFRigidBody  > _body1,
  Inst< SFRigidBody  > _body2,
  Inst< MFString  > _forceOutput,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFTransformNode   > _transform,
  Inst< SFVec3f  > _anchorPoint,
  Inst< SFVec3f  > _axis,
  Inst< SFFloat  > _maxAngle,
  Inst< SFFloat   > _minAngle,
  Inst< SFFloat   > _stopBounce,
  Inst< SFFloat   > _stopErrorCorrection,
  Inst< SFFloat   > _angle,
  Inst< SFFloat   > _angleRate,
  Inst< SFVec3f   > _body1AnchorPoint,
  Inst< SFVec3f   > _body2AnchorPoint,
  Inst< SFFloat   > _bias,
  Inst< SFFloat   > _softness):
H3DRigidBodyJointNode( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
anchorPoint( _anchorPoint ),
axis( _axis ),
maxAngle( _maxAngle ),
minAngle( _minAngle ),
stopBounce( _stopBounce ),
stopErrorCorrection( _stopErrorCorrection ),
angle( _angle ),
angleRate( _angleRate ),
body1AnchorPoint( _body1AnchorPoint ),
body2AnchorPoint( _body2AnchorPoint ),
bias( _bias),
softness( _softness ){

  // initialize fields.
  type_name = "SingleAxisHingeJoint";
  database.initFields( this );

  // set default values
  anchorPoint->setValue( Vec3f( 0, 0, 0 ) );
  axis->setValue( Vec3f( 0, 0, 0 ) );
  maxAngle->setValue( (H3DFloat) Constants::pi );
  minAngle->setValue( (H3DFloat) -Constants::pi );
  stopBounce->setValue( 0 );
  softness->setValue((H3DFloat)0.9);
  bias->setValue((H3DFloat)0.3);
  stopErrorCorrection->setValue( (H3DFloat)0.8 );
  angle->setValue( 0, id );
  angleRate->setValue( 0, id );

  // setup routes
  anchorPoint->route( valueUpdater );
  axis->route( valueUpdater );
  maxAngle->route( valueUpdater );
  minAngle->route( valueUpdater );
  stopBounce->route( valueUpdater );
  softness->route( valueUpdater );
  bias->route( valueUpdater );
  stopErrorCorrection->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* SingleAxisHingeJoint::createConstraintParameters () {
  return new PhysicsEngineParameters::SingleAxisHingeJointParameters();
}

PhysicsEngineParameters::ConstraintParameters * SingleAxisHingeJoint::getConstraintParameters( bool all_params ) {
  PhysicsEngineParameters::SingleAxisHingeJointParameters* params= 
    static_cast<PhysicsEngineParameters::SingleAxisHingeJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));
  if( !params )
    return NULL;

  if( all_params || valueUpdater->hasCausedEvent( anchorPoint ) ) {
    params->setAnchorPoint( anchorPoint->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( axis ) ) {
    params->setAxis( axis->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( minAngle ) ) {
    params->setMinAngle( minAngle->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( maxAngle ) ) {
    params->setMaxAngle( maxAngle->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( stopBounce ) ) {
    params->setStopBounce( stopBounce->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( softness ) ) {
    params->setSoftness( softness->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( bias ) ) {
    params->setBias( bias->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( stopErrorCorrection ) ) {
    params->setStopErrorCorrection( stopErrorCorrection->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( forceOutput ) ) {
    const vector<string> &output = forceOutput->getValue();
    bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
    if( !has_none ) {
      bool has_all = std::find( output.begin(), output.end(), "ALL" ) != output.end();

      if( has_all ) {
        params->enableAngle();
        params->enableAngleRate();
        params->enableBody1AnchorPoint();
        params->enableBody2AnchorPoint();
      } else {
        for( vector<string>::const_iterator i = output.begin(); i != output.end(); ++i ) {
          if( (*i) == "NONE" || (*i) == "ALL" ) continue;
          if( (*i) == "angle"  ) {
            params->enableAngle();
          } else if( (*i) == "angleRate" ) {
            params->enableAngleRate();
          } else if( (*i) == "body1AnchorPoint" ) {
            params->enableBody1AnchorPoint();
          } else if( (*i) == "body2AnchorPoint" ) {
            params->enableBody2AnchorPoint();
          } else {
            Console(4) << "Invalid forceOutput value in SingleAxisHingeJoint: " << (*i) << endl;
          }
        }
      }
    }

    // Save requested output fields in bit mask for next time
    output_bit_mask= params->getUpdateBitMask();
  }

  // To avoid recalculating the bit mask from the forceOutput field each time,
  // copy the output_bit_mask saved last time forceOutput was updated
  params->copyOutputFlags ( output_bit_mask );

  return params;
}

void SingleAxisHingeJoint::updateOutputFields() {
  H3DRigidBodyJointNode::updateOutputFields();

  PhysicsEngineParameters::SingleAxisHingeJointParameters params;

  // look at the forceOutput field to determine which fields to update
  unsigned int bitmask = 0;
  const vector<string> &output = forceOutput->getValue();
  bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();

  if( has_none ) return;

  engine_thread->getConstraintParameters( getConstraintId(), params );


  // set the fields with the updated data
  if( params.haveAngle() && params.getAngle() != angle->getValue() ) {
    angle->setValue( params.getAngle(), id );
  }

  if( params.haveAngleRate() && params.getAngleRate() != angleRate->getValue() ) {
    angleRate->setValue( params.getAngleRate(), id );
  }

  if( params.haveBody1AnchorPoint() && 
    params.getBody1AnchorPoint() != body1AnchorPoint->getValue() ) {
      body1AnchorPoint->setValue( params.getBody1AnchorPoint(), id );
  }

  if( params.haveBody2AnchorPoint() && 
    params.getBody2AnchorPoint() != body2AnchorPoint->getValue() ) {
      body2AnchorPoint->setValue( params.getBody2AnchorPoint(), id );
  }
}

void SingleAxisHingeJoint::applyTransform ( const Matrix4f& _transform ) { 
  anchorPoint->setValue( _transform*anchorPoint->getValue() );
  axis->setValue( _transform.getRotationPart()*axis->getValue() );
}