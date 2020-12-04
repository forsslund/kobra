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
/// \file DistanceJoint.cpp
/// \brief Source file for DistanceJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/DistanceJoint.h>

using namespace H3D;

H3DNodeDatabase DistanceJoint::database( "DistanceJoint",
                                         &(newInstance< DistanceJoint >),
                                         typeid(DistanceJoint),
                                         &H3DRigidBodyJointNode::database );

namespace DistanceJointInternals {
  FIELDDB_ELEMENT( DistanceJoint, maxDistance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DistanceJoint, minDistance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DistanceJoint, stiffness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DistanceJoint, damping, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DistanceJoint, distance, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DistanceJoint, tolerance, INPUT_OUTPUT )
}

DistanceJoint::DistanceJoint( Inst< SFNode      > _metadata,
                              Inst< ValueUpdater > _value_updater,
                              Inst< SFRigidBody > _body1,
                              Inst< SFRigidBody > _body2,
                              Inst< MFString    > _forceOutput,
                              Inst< MFEngineOptions > _engineOptions,
                              Inst< SFTransformNode   > _transform,
                              Inst< SFFloat   > _maxDistance,
                              Inst< SFFloat  > _minDistance,
                              Inst< SFFloat  > _stiffness,
                              Inst< SFFloat  > _damping,
                              Inst< SFFloat  > _distance,
                              Inst< SFFloat  > _tolerance ) :
  H3DRigidBodyJointNode( _metadata, _value_updater, _body1, _body2,
                         _forceOutput, _engineOptions, _transform ),
  maxDistance( _maxDistance ),
  minDistance( _minDistance ),
  stiffness( _stiffness ),
  damping( _damping ),
  distance( _distance ),
  tolerance( _tolerance ) {

  type_name = "DistanceJoint";
  database.initFields( this );

  maxDistance->setValue( 0.1f );
  minDistance->setValue( 0.0f );
  stiffness->setValue( 100.f );
  damping->setValue( 0.1f );
  distance->setValue( 0.0f, id );
  tolerance->setValue( 0.0f, id );

  maxDistance->route( valueUpdater );
  minDistance->route( valueUpdater );
  stiffness->route( valueUpdater );
  damping->route( valueUpdater );
  tolerance->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* DistanceJoint::createConstraintParameters() {
  return new PhysicsEngineParameters::DistanceJointParameters();
}

PhysicsEngineParameters::ConstraintParameters* DistanceJoint::getConstraintParameters( bool all_params ) {

  DistanceJointParameters* params =
    static_cast<DistanceJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters( all_params ));

  if( !params ) return NULL;

  if( all_params || valueUpdater->hasCausedEvent( maxDistance ) ) {
    params->setMaxDistance( maxDistance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( minDistance ) ) {
    params->setMinDistance( minDistance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( stiffness ) ) {
    params->setStiffness( stiffness->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( damping ) ) {
    params->setDamping( damping->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( tolerance ) ) {
    params->setTolerance( tolerance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( forceOutput ) ) {
    const vector<string> &output = forceOutput->getValue();
    bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
    if( !has_none ) {
      bool has_all = std::find( output.begin(), output.end(), "ALL" ) != output.end();
      if( has_all ) {
        params->enableDistance();
      } else {
        for( vector<string>::const_iterator i = output.begin(); i != output.end(); ++i ) {
          if( (*i) == "NONE" || (*i) == "ALL" ) continue;
          if( (*i) == "distance" ) {
            params->enableDistance();
          } else {
            Console(4) << "Invalid forceOutput value in DistanceJoint: " << (*i) << endl;
          }
        }
      }
    }

    // Save requested output fields in bit mask for next time
    output_bit_mask = params->getUpdateBitMask();
  }

  // To avoid recalculating the bit mask from the forceOutput field each time,
  // copy the output_bit_mask saved last time forceOutput was updated
  params->copyOutputFlags( output_bit_mask );

  return params;
}

void DistanceJoint::updateOutputFields() {
  H3DRigidBodyJointNode::updateOutputFields();
  PhysicsEngineParameters::DistanceJointParameters params;

  // look at the forceOutput field to determine which fields to update
  const vector<string> &output = forceOutput->getValue();
  bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
  if( has_none ) return;

  engine_thread->getConstraintParameters( getConstraintId(), params );

  // set the fields with the updated data
  if( params.haveDistance() &&
      params.getDistance() != distance->getValue() ) {
    distance->setValue( params.getDistance(), id );
  }

}

