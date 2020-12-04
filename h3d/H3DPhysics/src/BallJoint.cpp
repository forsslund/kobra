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
/// \file BallJoint.cpp
/// \brief Source file for BallJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/BallJoint.h>

using namespace H3D;

H3DNodeDatabase BallJoint::database( "BallJoint", 
                                    &(newInstance< BallJoint >), 
                                    typeid( BallJoint ),
                                    &H3DRigidBodyJointNode::database);

namespace BallJointInternals {
  FIELDDB_ELEMENT( BallJoint, anchorPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BallJoint, body1AnchorPoint, OUTPUT_ONLY )
  FIELDDB_ELEMENT( BallJoint, body2AnchorPoint, OUTPUT_ONLY )
}

BallJoint::BallJoint(Inst< SFNode      > _metadata,
                     Inst< ValueUpdater > _value_updater,
                     Inst< SFRigidBody > _body1,
                     Inst< SFRigidBody > _body2,
                     Inst< MFString    > _forceOutput,
                     Inst< MFEngineOptions > _engineOptions,
                     Inst< SFTransformNode   > _transform,
                     Inst< SFVec3f     > _anchorPoint,
                     Inst< SFVec3f     > _body1AnchorPoint,
                     Inst< SFVec3f     > _body2AnchorPoint ):
H3DRigidBodyJointNode( _metadata, _value_updater, _body1, _body2,
                      _forceOutput, _engineOptions, _transform ),
                      anchorPoint( _anchorPoint ),
                      body1AnchorPoint( _body1AnchorPoint ),
                      body2AnchorPoint( _body2AnchorPoint )
{

  type_name = "BallJoint";
  database.initFields( this );

  anchorPoint->setValue( Vec3f( 0, 0, 0 ) );
  anchorPoint->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* BallJoint::createConstraintParameters() {
  return new PhysicsEngineParameters::BallJointParameters();
}

PhysicsEngineParameters::ConstraintParameters* BallJoint::getConstraintParameters( bool all_params ) {

  BallJointParameters* params= 
    static_cast<BallJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));

  if( !params ) return NULL;

  if( all_params || valueUpdater->hasCausedEvent( anchorPoint ) ) {
    params->setAnchorPoint( anchorPoint->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( forceOutput ) ) {
    const vector<string> &output = forceOutput->getValue();
    bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
    if( !has_none ) {
      bool has_all = std::find( output.begin(), output.end(), "ALL" ) != output.end();
      if( has_all ) {
        params->enableBody1AnchorPoint();
        params->enableBody2AnchorPoint();
      } else {
        for( vector<string>::const_iterator i = output.begin(); i != output.end(); ++i ) {
          if( (*i) == "NONE" || (*i) == "ALL" ) continue;
          if( (*i) == "body1AnchorPoint" ) {
            params->enableBody1AnchorPoint();
          } else if( (*i) == "body2AnchorPoint" ) {
            params->enableBody2AnchorPoint();
          } else {
            Console(4) << "Invalid forceOutput value in BallJoint: " << (*i) << endl;
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

void BallJoint::updateOutputFields() {
  H3DRigidBodyJointNode::updateOutputFields();
  PhysicsEngineParameters::BallJointParameters params;

  // look at the forceOutput field to determine which fields to update
  const vector<string> &output = forceOutput->getValue();
  bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
  if( has_none ) return;

  engine_thread->getConstraintParameters( getConstraintId(), params );

  // set the fields with the updated data
  if( params.haveBody1AnchorPoint() && 
    params.getBody1AnchorPoint() != body1AnchorPoint->getValue() ) {
      body1AnchorPoint->setValue( params.getBody1AnchorPoint(), id );
  }
  if( params.haveBody2AnchorPoint() && 
    params.getBody2AnchorPoint() != body2AnchorPoint->getValue() ) {
      body2AnchorPoint->setValue( params.getBody2AnchorPoint(), id );
  }
}

void BallJoint::applyTransform ( const Matrix4f& _transform ) { 
  anchorPoint->setValue( _transform*anchorPoint->getValue() );
}