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
/// \file UniversalJoint.cpp
/// \brief Source file for UniversalJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/UniversalJoint.h>

using namespace H3D;

H3DNodeDatabase UniversalJoint::database( "UniversalJoint", 
                                         &(newInstance< UniversalJoint >), 
                                         typeid( UniversalJoint ),
                                         &H3DRigidBodyJointNode::database);

namespace UniversalJointInternals {
  FIELDDB_ELEMENT( UniversalJoint, anchorPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( UniversalJoint, axis1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( UniversalJoint, axis2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( UniversalJoint, stop1Bounce, INPUT_OUTPUT )
  FieldDBInsert string( INPUT_OUTPUT( &UniversalJoint::database, "stopBounce1", &UniversalJoint::stop1Bounce ) );
  FIELDDB_ELEMENT( UniversalJoint, stop1ErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( UniversalJoint, stop2Bounce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( UniversalJoint, stop2ErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( UniversalJoint, body1AnchorPoint, OUTPUT_ONLY )
  FIELDDB_ELEMENT( UniversalJoint, body1Axis, OUTPUT_ONLY )
  FIELDDB_ELEMENT( UniversalJoint, body2AnchorPoint, OUTPUT_ONLY )
  FIELDDB_ELEMENT( UniversalJoint, body2Axis, OUTPUT_ONLY )
}

UniversalJoint::UniversalJoint(Inst< SFNode   >  _metadata,
                               Inst< ValueUpdater > _value_updater,
                               Inst< SFRigidBody   >  _body1,
                               Inst< SFRigidBody   >  _body2,
                               Inst< MFString >  _forceOutput,
                               Inst< MFEngineOptions > _engineOptions,
                               Inst< SFTransformNode > _transform,
                               Inst< SFVec3f  >  _anchorPoint,
                               Inst< SFVec3f  >  _axis1,
                               Inst< SFVec3f  >  _axis2,
                               Inst< SFFloat  >  _stop1Bounce,
                               Inst< SFFloat  >  _stop1ErrorCorrection,
                               Inst< SFFloat  >  _stop2Bounce,
                               Inst< SFFloat  >  _stop2ErrorCorrection,
                               Inst< SFVec3f  >  _body1AnchorPoint,
                               Inst< SFVec3f  >  _body1Axis,
                               Inst< SFVec3f  >  _body2AnchorPoint,
                               Inst< SFVec3f  >  _body2Axis ): 
H3DRigidBodyJointNode( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
anchorPoint( _anchorPoint ),
axis1( _axis1 ),
axis2( _axis2 ),
stop1Bounce( _stop1Bounce ),
stop2Bounce( _stop2Bounce ),
stop1ErrorCorrection( _stop1ErrorCorrection ),
stop2ErrorCorrection( _stop2ErrorCorrection ),
body1AnchorPoint( _body1AnchorPoint ),
body2AnchorPoint( _body2AnchorPoint ),
body1Axis( _body1Axis ),
body2Axis( _body2Axis ) {

  type_name = "UniversalJoint";
  database.initFields( this );

  anchorPoint->setValue( Vec3f( 0, 0, 0 ) );
  axis1->setValue( Vec3f( 0, 0, 0 ) );
  axis2->setValue( Vec3f( 0, 0, 0 ) );
  stop1Bounce->setValue( 0 );
  stop1ErrorCorrection->setValue( (H3DFloat)0.8 );
  stop2Bounce->setValue( 0 );
  stop2ErrorCorrection->setValue( (H3DFloat)0.8 );

  anchorPoint->route( valueUpdater );
  axis1->route( valueUpdater );
  axis2->route( valueUpdater );
  stop1Bounce->route( valueUpdater );
  stop1ErrorCorrection->route( valueUpdater );
  stop2Bounce->route( valueUpdater );
  stop2ErrorCorrection->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* UniversalJoint::createConstraintParameters () {
  return new PhysicsEngineParameters::UniversalJointParameters();
}

PhysicsEngineParameters::ConstraintParameters * UniversalJoint::getConstraintParameters( bool all_params ) {

  PhysicsEngineParameters::UniversalJointParameters* params= 
    static_cast<PhysicsEngineParameters::UniversalJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));
  if( !params ) return NULL;

  if( all_params || valueUpdater->hasCausedEvent( anchorPoint ) ) {
    params->setAnchorPoint( anchorPoint->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis1 ) ) {
    params->setAxis1( axis1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis2 ) ) {
    params->setAxis2( axis2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop1Bounce ) ) {
    params->setStop1Bounce( stop1Bounce->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop1ErrorCorrection ) ) {
    params->setStop1ErrorCorrection( stop1ErrorCorrection->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop2Bounce ) ) {
    params->setStop2Bounce( stop2Bounce->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop2ErrorCorrection ) ) {
    params->setStop2ErrorCorrection( stop2ErrorCorrection->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( forceOutput ) ) {
    const vector<string> &output = forceOutput->getValue();
    bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
    if( !has_none ) {
      bool has_all = std::find( output.begin(), output.end(), "ALL" ) != output.end();
      if( has_all ) {
        params->enableBody1AnchorPoint();
        params->enableBody1Axis();
        params->enableBody2AnchorPoint();
        params->enableBody2Axis();
      } else {
        for( vector<string>::const_iterator i = output.begin(); i != output.end(); ++i ) {
          if( (*i) == "NONE" || (*i) == "ALL" ) continue;
          if( (*i) == "body1AnchorPoint"  ) {
            params->enableBody1AnchorPoint();
          } else if( (*i) == "body1Axis" ) {
            params->enableBody1Axis();
          } else if( (*i) == "body2AnchorPoint" ) {
            params->enableBody2AnchorPoint();
          } else if( (*i) == "body2Axis" ) {
            params->enableBody2Axis();
          } else {
            Console(4) << "Invalid forceOutput value in UniversalJoint: " << (*i) << endl;
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


void UniversalJoint::updateOutputFields() {
  H3DRigidBodyJointNode::updateOutputFields();
  PhysicsEngineParameters::UniversalJointParameters params;

  // look at the forceOutput field to determine which fields to update
  const vector<string> &output = forceOutput->getValue();
  bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
  if( has_none ) return;

  engine_thread->getConstraintParameters( getConstraintId(), params );

  // set the fields with the updated data
  if( params.haveBody1AnchorPoint() && params.getBody1AnchorPoint() != body1AnchorPoint->getValue() ) {
    body1AnchorPoint->setValue( params.getBody1AnchorPoint(), id );
  }
  if( params.haveBody1Axis() && params.getBody1Axis() != body1Axis->getValue() ) {
    body1Axis->setValue( params.getBody1Axis(), id );
  }
  if( params.haveBody2AnchorPoint() && params.getBody2AnchorPoint() != body2AnchorPoint->getValue() ) {
    body2AnchorPoint->setValue( params.getBody2AnchorPoint(), id );
  }
  if( params.haveBody2Axis() && params.getBody2Axis() != body2Axis->getValue() ) {
    body2Axis->setValue( params.getBody2Axis(), id );
  }
}

void UniversalJoint::applyTransform ( const Matrix4f& _transform ) {
  anchorPoint->setValue( _transform*anchorPoint->getValue() );
  axis1->setValue( _transform.getRotationPart()*axis1->getValue() );
  axis2->setValue( _transform.getRotationPart()*axis2->getValue() );
}
