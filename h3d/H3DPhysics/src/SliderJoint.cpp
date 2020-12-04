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
/// \file SliderJoint.cpp
/// \brief Source file for SliderJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/SliderJoint.h>

using namespace H3D;

H3DNodeDatabase SliderJoint::database( "SliderJoint", 
                                      &(newInstance< SliderJoint >), 
                                      typeid( SliderJoint ),
                                      &H3DRigidBodyJointNode::database);

namespace SliderJointInternals {
  FIELDDB_ELEMENT( SliderJoint, axis, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderJoint, maxSeparation, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderJoint, minSeparation, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderJoint, stopBounce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderJoint, stopErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderJoint, separation, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SliderJoint, separationRate, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SliderJoint, sliderForce, INPUT_OUTPUT )
}

SliderJoint::SliderJoint(
                         Inst< SFNode  > _metadata,
                         Inst< ValueUpdater > _value_updater,
                         Inst< SFRigidBody  > _body1,
                         Inst< SFRigidBody  > _body2,
                         Inst< MFString  > _forceOutput,
                         Inst< MFEngineOptions > _engineOptions,
                         Inst< SFTransformNode   > _transform,
                         Inst< SFVec3f  > _axis,
                         Inst< SFFloat   > _maxSeparation,
                         Inst< SFFloat   > _minSeparation,
                         Inst< SFFloat   > _stopBounce,
                         Inst< SFFloat   > _stopErrorCorrection,
                         Inst< SFFloat   > _separation,
                         Inst< SFFloat   > _separationRate,
                         Inst< SFFloat      > _sliderForce ): 
H3DRigidBodyJointNode( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
axis( _axis ),
maxSeparation( _maxSeparation ),
minSeparation( _minSeparation ),
stopBounce( _stopBounce ),
stopErrorCorrection( _stopErrorCorrection ),
separation( _separation ),
separationRate( _separationRate ),
sliderForce( _sliderForce ) {

  type_name = "SliderJoint";
  database.initFields( this );

  axis->setValue( Vec3f( 0, 1, 0 ) );
  maxSeparation->setValue( 1 );
  minSeparation->setValue( 0 );
  stopBounce->setValue( 0 );
  stopErrorCorrection->setValue( 1 );
  sliderForce->setValue( 0 );

  axis->route( valueUpdater );
  maxSeparation->route( valueUpdater );
  minSeparation->route( valueUpdater );
  stopBounce->route( valueUpdater );
  stopErrorCorrection->route( valueUpdater );
  sliderForce->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* SliderJoint::createConstraintParameters () {
  return new PhysicsEngineParameters::SliderJointParameters();
}

PhysicsEngineParameters::ConstraintParameters * SliderJoint::getConstraintParameters( bool all_params ) {
  PhysicsEngineParameters::SliderJointParameters* params= 
    static_cast<PhysicsEngineParameters::SliderJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));

  if( all_params || valueUpdater->hasCausedEvent( axis ) ) {
    params->setAxis( axis->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxSeparation ) ) {
    params->setMaxSeparation( maxSeparation->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( minSeparation ) ) {
    params->setMinSeparation( minSeparation->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stopBounce ) ) {
    params->setStopBounce( stopBounce->getValue() );
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
        params->enableSeparation();
        params->enableSeparationRate();
      } else {
        for( vector<string>::const_iterator i = output.begin(); i != output.end(); ++i ) {
          if( (*i) == "NONE" || (*i) == "ALL" ) continue;
          if( (*i) == "separation"  ) {
            params->enableSeparation();
          } else if( (*i) == "separationRate" ) {
            params->enableSeparationRate();
          } else {
            Console(4) << "Invalid forceOutput value in SliderJoint: " << (*i) << endl;
          }
        }
      }
    }

    // Save requested output fields in bit mask for next time
    output_bit_mask= params->getUpdateBitMask();
  }

  if( all_params || valueUpdater->hasCausedEvent( sliderForce ) ) {
    params->setSliderForce( sliderForce->getValue() );
  }

  // To avoid recalculating the bit mask from the forceOutput field each time,
  // copy the output_bit_mask saved last time forceOutput was updated
  params->copyOutputFlags ( output_bit_mask );

  return params;
}


void SliderJoint::updateOutputFields() {
  H3DRigidBodyJointNode::updateOutputFields();
  PhysicsEngineParameters::SliderJointParameters params;

  // look at the forceOutput field to determine which fields to update
  const vector<string> &output = forceOutput->getValue();
  bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
  if( has_none ) return;

  engine_thread->getConstraintParameters( getConstraintId(), params );

  // set the fields with the updated data
  if( params.haveSeparation() && params.getSeparation() != separation->getValue() ) {
    separation->setValue( params.getSeparation(), id );
  }
  if( params.haveSeparationRate() && params.getSeparationRate() != separationRate->getValue() ) {
    separationRate->setValue( params.getSeparationRate(), id );
  }
}

void SliderJoint::applyTransform ( const Matrix4f& _transform ) { 
  axis->setValue( _transform.getRotationPart()*axis->getValue() );
}