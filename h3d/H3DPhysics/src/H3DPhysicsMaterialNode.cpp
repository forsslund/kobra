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
/// \file H3DPhysicsMaterialNode.cpp
/// \brief Source file for H3DPhysicsMaterialNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DPhysicsMaterialNode.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase H3DPhysicsMaterialNode::database( "H3DPhysicsMaterialNode", 
                                                 NULL, 
                                                 typeid( H3DPhysicsMaterialNode ),
                                                 &X3DNode::database);

namespace H3DPhysicsMaterialNodeInternals {
  FIELDDB_ELEMENT( H3DPhysicsMaterialNode, mass, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DPhysicsMaterialNode, damping, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DPhysicsMaterialNode, friction, INPUT_OUTPUT )
}

H3DPhysicsMaterialNode::H3DPhysicsMaterialNode(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFH3DPhysicsMassNode > _mass,
  Inst< SFH3DPhysicsDampingNode > _damping,
  Inst< SFH3DPhysicsFrictionNode > _friction ):
X3DNode( _metadata ),
valueUpdater( _valueUpdater ),
mass( _mass ),
damping( _damping ),
friction( _friction ),
materialChanged( new Field ) {

  type_name = "H3DPhysicsMaterialNode";
  database.initFields( this );

  materialChanged->setOwner( this );
  materialChanged->setName( "materialChanged" );
  mass->route( materialChanged );
  damping->route( materialChanged );
  friction->route( materialChanged );

  valueUpdater->setOwner( this );
  valueUpdater->setName( "valueUpdater" );
  mass->route( valueUpdater );
  damping->route( valueUpdater );
  friction->route( valueUpdater );

  previousMass = mass->getValue();
  previousDamping = damping->getValue();
  previousFriction = friction->getValue();

}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* H3DPhysicsMaterialNode::getH3DPhysicsMaterialParameters(bool all_params ) {

  PhysicsEngineParameters::H3DPhysicsMaterialParameters *params = createH3DPhysicsMaterialParameters ();

  if ( all_params || valueUpdater->hasCausedEvent ( mass ) ) {
    params->setMass ( mass->getValue() );

    // If the mass field is set to a different node
    // create all the mass parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousMass != mass->getValue() )
    {
      material_params = true;
      previousMass = mass->getValue();
    }
    if ( mass->getValue() ) {
      params->setMassParameters( dynamic_cast< MassParameters* >
        (mass->getValue()->valueUpdater->getMaterialPropertyParameters( material_params ) ) );
    }
  }
  if ( all_params || valueUpdater->hasCausedEvent ( damping ) ) {
    params->setDamping ( damping->getValue() );

    // If the damping field is set to a different node
    // create all the damping parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousDamping != damping->getValue() )
    {
      material_params = true;
      previousDamping = damping->getValue();
    }
    if ( damping->getValue() ) {
      params->setDampingParameters( dynamic_cast< DampingParameters* >
        (damping->getValue()->valueUpdater->getMaterialPropertyParameters( material_params ) ) );
    }
  }
  if ( all_params || valueUpdater->hasCausedEvent ( friction ) ) {
    params->setFriction ( friction->getValue() );

    // If the friction field is set to a different node
    // create all the friction parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousFriction != friction->getValue() )
    {
      material_params = true;
      previousFriction = friction->getValue();
    }
    if ( friction->getValue() ) {
      params->setFrictionParameters( dynamic_cast< FrictionParameters* >
        (friction->getValue()->valueUpdater->getMaterialPropertyParameters( material_params ) ) );
    }
  }

  return params;
}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* 
H3DPhysicsMaterialNode::ValueUpdater::getH3DPhysicsMaterialParameters( bool all_params ) {
  allParams= all_params;
  upToDate();
  return params.get();
}

void H3DPhysicsMaterialNode::ValueUpdater::update() {
  H3DPhysicsMaterialNode* node= static_cast < H3DPhysicsMaterialNode* > ( getOwner() );
  params.reset ( node->getH3DPhysicsMaterialParameters ( allParams ) );

  EventCollectingField < PeriodicUpdate < Field > >::update();
}

