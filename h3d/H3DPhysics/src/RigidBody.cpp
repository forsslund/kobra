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
/// \file RigidBody.cpp
/// \brief Source file for RigidBody, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/RigidBodyCollection.h>
#include <H3D/H3DPhysics/CollidableOffset.h>
#include <string.h>

using namespace H3D;

H3DNodeDatabase RigidBody::database( "RigidBody",
                                    &(newInstance< RigidBody >),
                                    typeid( RigidBody ),
                                    &H3DBodyNode::database);

namespace RigidBodyInternals {
  FIELDDB_ELEMENT( RigidBody, angularDampingFactor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, angularVelocity     , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, autoDamp            , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, autoDisable         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, centerOfMass        , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, disableAngularSpeed , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, disableLinearSpeed  , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, disableTime         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, enabled             , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, finiteRotationAxis  , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, fixed               , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, forces              , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, geometry            , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, inertia             , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, linearDampingFactor , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, linearVelocity      , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, mass                , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, massDensityModel    , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, orientation         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, position            , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, torques             , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, useFiniteRotation   , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, useGlobalGravity    , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, engineOptions       , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, kinematicControl    , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, debug               , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBody, transform           , INPUT_OUTPUT )
}

RigidBody::RigidBody(Inst< SFNode >  _metadata,
                     Inst< SFFloat >  _angularDampingFactor,
                     Inst< SFVec3f >  _angularVelocity,
                     Inst< SFBool >  _autoDamp,
                     Inst< SFBool >  _autoDisable,
                     Inst< SFVec3f >  _centerOfMass,
                     Inst< SFFloat >  _disableAngularSpeed,
                     Inst< SFFloat >  _disableLinearSpeed,
                     Inst< SFFloat >  _disableTime,
                     Inst< SFBool >  _enabled,
                     Inst< SFVec3f >  _finiteRotationAxis,
                     Inst< SFBool >  _fixed,
                     Inst< MFVec3f >  _forces,
                     Inst< MFCollidableNode >  _geometry,
                     Inst< SFMatrix3f >  _inertia,
                     Inst< SFFloat >  _linearDampingFactor,
                     Inst< SFVec3f >  _linearVelocity,
                     Inst< SFFloat >  _mass,
                     Inst< SFNode >  _massDensityModel,
                     Inst< SFRotation >  _orientation,
                     Inst< SFVec3f >  _position,
                     Inst< MFVec3f >  _torques,
                     Inst< SFBool >  _useFiniteRotation,
                     Inst< SFBool >  _useGlobalGravity,
                     Inst< SFBool > _kinematicControl,
                     Inst< ValueUpdater > _valueUpdater,
                     Inst< MFEngineOptions > _engineOptions,
                     Inst< SFTransformNode > _transform ):
H3DBodyNode( _metadata ),
angularDampingFactor( _angularDampingFactor ),
angularVelocity( _angularVelocity ),
autoDamp( _autoDamp ),
autoDisable( _autoDisable ),
centerOfMass( _centerOfMass ),
disableAngularSpeed( _disableAngularSpeed ),
disableLinearSpeed( _disableLinearSpeed ),
disableTime( _disableTime ),
enabled( _enabled ),
finiteRotationAxis( _finiteRotationAxis ),
fixed( _fixed ),
forces( _forces ),
geometry( _geometry ),
inertia( _inertia ),
linearDampingFactor( _linearDampingFactor ),
linearVelocity( _linearVelocity ),
mass( _mass ),
massDensityModel( _massDensityModel ),
orientation( _orientation ),
position( _position ),
torques( _torques ),
useFiniteRotation( _useFiniteRotation ),
useGlobalGravity( _useGlobalGravity ),
kinematicControl ( _kinematicControl ),
valueUpdater( _valueUpdater ),
engineOptions ( _engineOptions ),
transform( _transform ),
debug ( new SFBool ) {

  // init fields
  type_name = "RigidBody";
  database.initFields( this );

  valueUpdater->setName( "valueUpdater" );
  valueUpdater->setOwner( this );

  // set default values
  angularDampingFactor->setValue( (H3DFloat)0.001 );
  angularVelocity->setValue( Vec3f( 0, 0, 0 ) );
  autoDamp->setValue( false );
  autoDisable->setValue( false );
  centerOfMass->setValue( Vec3f( 0, 0, 0 ) );
  disableAngularSpeed->setValue( 0 );
  disableLinearSpeed->setValue( 0 );
  disableTime->setValue( 0 );
  enabled->setValue( true );
  finiteRotationAxis->setValue( Vec3f( 0, 0, 0 ) );
  fixed->setValue( false );
  inertia->setValue( Matrix3f() );
  linearDampingFactor->setValue( (H3DFloat)0.001 );
  linearVelocity->setValue( Vec3f( 0, 0, 0 ) );
  mass->setValue( 1 );
  orientation->setValue( Rotation( 0, 0, 1, 0 ) );
  position->setValue( Vec3f( 0, 0, 0 ) );
  useFiniteRotation->setValue( false );
  useGlobalGravity->setValue( true );
  kinematicControl->setValue ( false );
  debug->setValue ( false );

  // setup routes
  angularDampingFactor->route( valueUpdater );
  angularVelocity->route( valueUpdater );
  autoDamp->route( valueUpdater );
  autoDisable->route( valueUpdater );
  centerOfMass->route( valueUpdater );
  disableAngularSpeed->route( valueUpdater );
  disableLinearSpeed->route( valueUpdater );
  disableTime->route( valueUpdater );
  enabled->route( valueUpdater );
  finiteRotationAxis->route( valueUpdater );
  fixed->route( valueUpdater );
  geometry->route( valueUpdater );
  inertia->route( valueUpdater );
  linearDampingFactor->route( valueUpdater );
  linearVelocity->route( valueUpdater );
  mass->route( valueUpdater );
  massDensityModel->route( valueUpdater );
  orientation->route( valueUpdater );
  position->route( valueUpdater );
  useFiniteRotation->route( valueUpdater );
  useGlobalGravity->route( valueUpdater );
  kinematicControl->route ( valueUpdater );
  engineOptions->route ( valueUpdater );
}

void RigidBody::traverseSG(H3D::TraverseInfo &ti) {
  X3DNode::traverseSG( ti );
  PhysicsEngineThread *pt = NULL;
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt ) {
    // initialize the rigid body if not initialized
    if( !isInitialized() ) {
      // Applying the transformation on the initial values
      if ( transform->getValue() ) {
        Matrix4f t= transform->getValue()->matrix->getValue();

        position->setValue(t*position->getValue());
        orientation->setValue(Rotation(t.getRotationPart())*orientation->getValue());
        }
      initializeBody( *pt );
    }

    if( pt == engine_thread ) {
      // traverse the collidables
      for( MFCollidableNode::const_iterator i = geometry->begin();
        i != geometry->end(); ++i ) {
          X3DNBodyCollidableNode *col =
            static_cast< X3DNBodyCollidableNode * >(*i);
          if( col ) col->traverseSG( ti );
      }

      // apply external forces to rigid body.
      applyForces( pt );

      // NF. Because there are fields that are both input and output,
      // like position and orientation (to support kinematic objects in Bullet and similar),
      // we need to ensure that any user-defined values are set before they are overwritten
      // with current values retrieved from the simulation
      // NB. This is asyncronous, so the values may still be overwritten temporarily, but they will sync in the end.
      valueUpdater->upToDate();

      // temporarily unroute the output fields while setting the fields
      // in order to avoid triggering an event for the valueUpdater. The
      // valueUpdater should only get events when a user explicitly sets
      // the field himself.
      position->unroute( valueUpdater );
      orientation->unroute( valueUpdater );
      linearVelocity->unroute( valueUpdater );
      angularVelocity->unroute( valueUpdater );
      enabled->unroute( valueUpdater );

      // update output fields
      PhysicsEngineParameters::RigidBodyParameters params;
      pt->getRigidBodyParameters( getBodyId(), params );
      Vec3f pos = params.getPosition();
      Rotation orn = params.getOrientation();
      if( pos != position->getValue() )
          position->setValue(pos);
      if( orn != orientation->getValue() )
        orientation->setValue( orn );

      linearVelocity->setValue( params.getLinearVelocity() );
      angularVelocity->setValue( params.getAngularVelocity() );
      if( params.getEnabled() != enabled->getValue() ) enabled->setValue( params.getEnabled() );

      // restore routes
      position->routeNoEvent( valueUpdater );
      orientation->routeNoEvent( valueUpdater );
      linearVelocity->routeNoEvent( valueUpdater );
      angularVelocity->routeNoEvent( valueUpdater );

      if( !collidables_without_ids.empty() ) {
          // update collidables that are not yet updated
          geometry->touch();
          nbframes_collidables_without_ids++;

          if( nbframes_collidables_without_ids > 10 ) {
            //
            H3D::Console( 4 ) << " The following collidables did not get valid ids. Hence, not been added to the underlying physics body" << std::endl;
            for( unsigned int idx = 0; idx < collidables_without_ids.size(); idx++ ) {
              std::pair<unsigned int, std::string> val = collidables_without_ids[idx];
              H3D::Console( 4 ) << "Name: "<< val.second << "\t index: " << val.first << std::endl;
            }
            nbframes_collidables_without_ids = 0;
            collidables_without_ids.clear();
          }
      }
      else {
        nbframes_collidables_without_ids = 0;
      }

#ifdef DEBUG_RB_LAG
    if ( debug->getValue() ) {
      if ( !lagDebugFile.is_open() ) {
        stringstream ss;
        ss << "LagDebugGraphics" << getBodyId();
        lagDebugFile= ofstream ( ss.str() );
      }
      LARGE_INTEGER time;
      QueryPerformanceCounter ( &time );
      lagDebugFile << time.QuadPart << ", " << position->getValue() << endl;
    }
#endif
    }
  }
}

void RigidBody::applyForces( PhysicsEngineThread *pt) {
  const vector< Vec3f > &force_vector = forces->getValue();
  const vector< Vec3f > &torque_vector = torques->getValue();

  Vec3f total_force, total_torque;

  // add forces from force field
  if (!forces->empty()) {
    for (unsigned int i = 0; i < force_vector.size(); ++i) {
      total_force = total_force + force_vector[i];
    }
  }

  // add torques from torque field
  if (!torques->empty()) {
    for (unsigned int i = 0; i < torque_vector.size(); ++i) {
      total_torque = total_torque + torque_vector[i];
    }
  }

  // add forces from haptics interaction.
  /*X3DNBodyCollidableNode *cn;
  Shape *s;
  X3DGeometryNode *geom;

  const NodeVector &c = geometry->getValue();
  for( unsigned int i = 0; i < c.size(); ++i ) {
    if( c[i] ) {
      cn = static_cast< X3DNBodyCollidableNode* >( c[i] );
      if ( CollidableOffset *co = dynamic_cast< CollidableOffset* >(cn) ) {
        s = co->getShape();
      } else if ( CollidableShape *cs = dynamic_cast< CollidableShape* >(cn) ) {
        s = static_cast< Shape* >( cs->shape->getValue() );
      }
      geom= s->hapticGeometry->getValue();
      if ( !geom ) {
        geom= s->geometry->getValue();
      }
      if ( geom ) {
        const vector< bool > &touch = geom->isTouched->getValue();
        const vector< Vec3f > &point = geom->contactPoint->getValue();
        const vector< Vec3f > &force = geom->force->getValue();

        for (unsigned int i = 0; i < touch.size(); ++i) {
          if (touch[i]){
            const Rotation &orn = orientation->getValue();
            const Vec3f &pos = position->getValue();
            Vec3f global_point =  orn*point[i] + pos;
            Vec3f global_force = orn * (-force[i]);
            Vec3f lever_arm = global_point - pos;
            total_force = total_force +  global_force;
            total_torque = total_torque + lever_arm.crossProduct( global_force );
          }
        }
      }
    }
  }*/

  pt->setGlobalExternalForce( getBodyId(), total_force );
  pt->setGlobalExternalTorque( getBodyId(), total_torque );
}

bool RigidBody::initializeBody( PhysicsEngineThread& pt ) {
  if( isInitialized() ) return false;

  // Purge all pending updates to parameters since we will initialise all
  // parameters when the body is created. Otherwise the parameters will be
  // set again next frame
  valueUpdater->upToDate();

  engine_thread= &pt;
  PhysicsEngineParameters::RigidBodyParameters *p = getRigidBodyParameters( true );
  body_id= pt.addRigidBody( *p );

  return H3DBodyNode::initializeBody ( pt );
}

bool RigidBody::deleteBody() {
  if ( isInitialized() ) {
    engine_thread->removeRigidBody( body_id );
    return H3DBodyNode::deleteBody ();
  }
  return false;
}

void RigidBody::ValueUpdater::update() {
  RigidBody *body = static_cast< RigidBody * >( getOwner());
  if ( body->isInitialized() ) {
    RigidBodyParameters *params = body->getRigidBodyParameters();
    if( params )
      body->engine_thread->setRigidBodyParameters( body->body_id, *params );
  }
  EventCollectingField< PeriodicUpdate< Field > >::update();
}

PhysicsEngineParameters::RigidBodyParameters* RigidBody::createRigidBodyParameters () {
  return new PhysicsEngineParameters::RigidBodyParameters();
}

PhysicsEngineParameters::RigidBodyParameters *RigidBody::getRigidBodyParameters( bool all_params ) {
  //PhysicsEngineParameters::RigidBodyParameters *params = new PhysicsEngineParameters::RigidBodyParameters;
  PhysicsEngineParameters::RigidBodyParameters *params = createRigidBodyParameters();

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  params->setName( this->getName() );
#endif

  if( all_params || valueUpdater->hasCausedEvent( angularDampingFactor ) ) {
    params->setAngularDampingFactor( angularDampingFactor->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( angularVelocity ) ) {
    params->setStartAngularVelocity( angularVelocity->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( autoDamp ) ) {
    params->setAutoDamp( autoDamp->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( autoDisable ) ) {
    params->setAutoDisable( autoDisable->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( centerOfMass ) ) {
    params->setCenterOfMass( centerOfMass->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( disableAngularSpeed ) ) {
    params->setDisableAngularSpeed( disableAngularSpeed->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( disableLinearSpeed ) ) {
    params->setDisableLinearSpeed( disableLinearSpeed->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( disableTime ) ) {
    params->setDisableTime( disableTime->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( enabled ) ) {
    params->setEnabled( enabled->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( finiteRotationAxis ) ) {
    params->setFiniteRotationAxis( finiteRotationAxis->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( fixed ) ) {
    params->setFixed( fixed->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( geometry ) ) {
    vector< H3DCollidableId > geom_ids;
    collidables_without_ids.clear();
    for( unsigned int i = 0; i < geometry->size(); ++i ) {
        X3DNBodyCollidableNode *cs = static_cast< X3DNBodyCollidableNode * >( geometry->getValueByIndex(i) );
        PhysicsEngineParameters::H3DCollidableId collidable_id = cs->getCollidableId();
        if( collidable_id )
          geom_ids.push_back( collidable_id );
        else
          collidables_without_ids.push_back( std::pair<unsigned int, std::string>(i,cs->getName()) );
    }
    params->setGeometry( geom_ids );
  }
  if( all_params || valueUpdater->hasCausedEvent( inertia ) ) {
    params->setInertia( inertia->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( linearDampingFactor ) ) {
    params->setLinearDampingFactor( linearDampingFactor->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( linearVelocity ) ) {
    params->setStartLinearVelocity( linearVelocity->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( mass ) ) {
    params->setMass( mass->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( massDensityModel ) ) {
    params->setMassDensityModel( massDensityModel->getValue() );
  }
  if( all_params ) {
    params->setStartOrientation( orientation->getValue() );
  }
  // N.B. Only set orientation if orientation is changed after initialization as
  // this will cause the rigid body to enter kinematic control mode
  else if ( valueUpdater->hasCausedEvent( orientation ) ) {
    params->setOrientation ( orientation->getValue() );

    // Enable kinematic control
    params->setKinematicControl( true );
    kinematicControl->unroute ( valueUpdater );
    kinematicControl->setValue ( true );
    kinematicControl->routeNoEvent ( valueUpdater );
  }
  if( all_params ) {
    params->setStartPosition( position->getValue() );
  }
  // N.B. Only set position if position is changed after initialization as
  // this will cause the rigid body to enter kinematic control mode
  else if ( valueUpdater->hasCausedEvent( position ) ) {
    params->setPosition ( position->getValue() );

    // Enable kinematic control
    params->setKinematicControl( true );
    kinematicControl->unroute ( valueUpdater );
    kinematicControl->setValue ( true );
    kinematicControl->routeNoEvent ( valueUpdater );
  }
  if( all_params || valueUpdater->hasCausedEvent( useFiniteRotation ) ) {
    params->setUseFiniteRotation( useFiniteRotation->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( useGlobalGravity ) ) {
    params->setUseGlobalGravity( useGlobalGravity->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( kinematicControl ) ) {
    params->setKinematicControl( kinematicControl->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( debug ) ) {
    params->setDebug( debug->getValue() );
  }
  if ( H3DEngineOptions* options= engineOptions->getOptions ( engine_thread->getEngine() ) ) {
    if ( all_params || valueUpdater->hasCausedEvent ( options->valueUpdater ) ) {
      params->setEngineOptions ( options->valueUpdater->getParameters( all_params ) );
    }
  }
  return params;
}
void RigidBody::renderCollidable( bool render_only_enabled_collidables ){
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  const Rotation &body_orn = orientation->getValue();
  const Vec3f &body_pos = position->getValue();
  glTranslatef( body_pos.x, body_pos.y, body_pos.z );
  glRotatef( (GLfloat) (body_orn.angle*180/H3DUtil::Constants::pi), body_orn.axis.x, body_orn.axis.y, body_orn.axis.z );
  for( RigidBody::MFCollidableNode::const_iterator c = geometry->begin(); c != geometry->end(); c++ ) {
    X3DNBodyCollidableNode *collidable = static_cast< X3DNBodyCollidableNode * >( *c );
    if( !render_only_enabled_collidables || collidable->enabled->getValue() ) {
      collidable->displayList->callList();
    }
  }
  glMatrixMode( GL_MODELVIEW );
  glPopMatrix();
}
