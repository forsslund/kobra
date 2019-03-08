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
/// \file ArticulatedRigidBody.cpp
/// \brief Source file for ArticulatedRigidBody, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/ArticulatedRigidBody.h>
#include <H3D/H3DPhysics/RigidBodyCollection.h>
#include <H3D/H3DPhysics/CollidableOffset.h>
#include <string.h>

using namespace H3D;

H3DNodeDatabase ArticulatedRigidBody::database( "ArticulatedRigidBody",
                                                &(newInstance< ArticulatedRigidBody >),
                                                typeid(ArticulatedRigidBody),
                                                &RigidBody::database );

namespace ArticulatedRigidBodyInternals {
  FIELDDB_ELEMENT( ArticulatedRigidBody, orientations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ArticulatedRigidBody, positions, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ArticulatedRigidBody, maxProjectionIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ArticulatedRigidBody, separationTolerance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ArticulatedRigidBody, jointInternalCompliance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ArticulatedRigidBody, jointExternalCompliance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ArticulatedRigidBody, proxyBodies, OUTPUT_ONLY )
  FIELDDB_ELEMENT( ArticulatedRigidBody, swingLimit, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ArticulatedRigidBody, twistLimit, INPUT_OUTPUT )
}

ArticulatedRigidBody::ArticulatedRigidBody( Inst< SFNode >  _metadata,
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
                                            Inst< MFArticulatedCollidables >  _geometry,
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
                                            Inst< SFTransformNode > _transform,
                                            Inst< MFRotation > _orientations,
                                            Inst< MFVec3f > _positions,
                                            Inst< MFProxyRigidBody  > _proxyBodies,
                                            Inst< SFInt32  > _maxProjectionIterations,
                                            Inst< SFFloat  > _separationTolerance,
                                            Inst< SFFloat  > _jointInternalCompliance,
                                            Inst< SFFloat  > _jointExternalCompliance,
                                            Inst< MFVec2f > _swingLimit,
                                            Inst< MFVec2f > _twistLimit ):
RigidBody( _metadata, _angularDampingFactor, _angularVelocity, _autoDamp, _autoDisable,
           _centerOfMass, _disableAngularSpeed, _disableLinearSpeed, _disableTime, _enabled,
           _finiteRotationAxis, _fixed, _forces, _geometry, _inertia, _linearDampingFactor,
           _linearVelocity, _mass, _massDensityModel, _orientation, _position, _torques,
           _useFiniteRotation, _useGlobalGravity, _kinematicControl, _valueUpdater, _engineOptions,
           _transform ),
  orientations( _orientations ), positions( _positions ),
  proxyBodies( _proxyBodies ), updateProxyBodies( new UpdateProxyBody ),
  proxy_bodies_initialized( false ),
  maxProjectionIterations( _maxProjectionIterations ),
  separationTolerance( _separationTolerance ),
  jointInternalCompliance( _jointInternalCompliance ),
  jointExternalCompliance( _jointExternalCompliance ),
  swingLimit( _swingLimit ),
  twistLimit( _twistLimit ) {

  // init fields
  type_name = "ArticulatedRigidBody";
  database.initFields( this );

  maxProjectionIterations->setValue( 4 );
  separationTolerance->setValue( 0.1f );
  jointInternalCompliance->setValue( 0.0f );
  jointExternalCompliance->setValue( 0.0f );

  geometry->setOwner( this );
  updateProxyBodies->setOwner( this );
  updateProxyBodies->setName( "updateProxyBodies" );

  positions->route( updateProxyBodies );
  orientations->route( updateProxyBodies );

  maxProjectionIterations->route( valueUpdater );
  separationTolerance->route( valueUpdater );
  jointInternalCompliance->route( valueUpdater );
  jointExternalCompliance->route( valueUpdater );
  swingLimit->route( valueUpdater );
  twistLimit->route( valueUpdater );

}

void ArticulatedRigidBody::traverseSG( H3D::TraverseInfo &ti ) {
  X3DNode::traverseSG( ti );
  PhysicsEngineThread *pt = NULL;
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt ) {
    // initialize the rigid body if not initialized
    if( !isInitialized() ) {

      size_t bodies_size = 0;
      // Check the consistency between the sizes of the collidables and the positions and orientations.
      if( geometry->getValue().size() != positions->getValue().size() ||
          geometry->getValue().size() != orientations->getValue().size() ||
          geometry->getValue().size() != proxyBodies->getValue().size() ) {

        Console(4) << "ERROR:ArticulatedRigidBody: Inconsisteny between the sizes of geometry, positions, orientations and proxyBodies." << endl
          << "Size of geometry: " << geometry->getValue().size() << endl
          << "Size of positions: " << positions->getValue().size() << endl
          << "Size of orientations: " << orientations->getValue().size() << endl
          << "Size of proxyBodies: " << proxyBodies->getValue().size() << endl;
        return;
      }

      bodies_size = geometry->getValue().size();
      // Applying the transformation on the initial values
      if( transform->getValue() ) {
        Matrix4f t = transform->getValue()->matrix->getValue();

        position->setValue( t*position->getValue() );
        orientation->setValue( Rotation( t.getRotationPart() )*orientation->getValue() );

        std::vector<Vec3f> pos = positions->getValue();
        std::vector<Rotation> orn = orientations->getValue();
        for( size_t i = 0; i < bodies_size; ++i ) {
          pos[i] = t * pos[i];
          orn[i] = Rotation( t.getRotationPart() )*orn[i];
        }
        positions->setValue( pos );
        orientations->setValue( orn );
      }
      initializeBody( *pt );
    }

    if( pt == engine_thread ) {
      // traverse the collidables
      for( MFCollidableNode::const_iterator i = geometry->begin();
           i != geometry->end(); ++i ) {
        X3DNBodyCollidableNode *col =
          static_cast<X3DNBodyCollidableNode *>(*i);
        if( col ) col->traverseSG( ti );
      }

      // NF. Because there are fields that are both input and output,
      // like position and orientation (to support kinematic objects in Bullet and similar),
      // we need to ensure that any user-defined values are set before they are overwritten 
      // with current values retrieved from the simulation
      // NB. This is asyncronous, so the values may still be overwritten temporarily, but they will sync in the end.
      valueUpdater->upToDate();

      //// update output fields
      PhysicsEngineParameters::ArticulatedRigidBodyParameters params;
      pt->getRigidBodyParameters( getBodyId(), params );
      positions->setValue( params.getPositions() );
      orientations->setValue( params.getOrientations() );

      initializeProxyBodies( *pt );


    }
  }
}

PhysicsEngineParameters::RigidBodyParameters* ArticulatedRigidBody::createRigidBodyParameters() {
  return new PhysicsEngineParameters::ArticulatedRigidBodyParameters();
}

PhysicsEngineParameters::RigidBodyParameters *ArticulatedRigidBody::getRigidBodyParameters( bool all_params ) {
  PhysicsEngineParameters::ArticulatedRigidBodyParameters *params =
    static_cast<PhysicsEngineParameters::ArticulatedRigidBodyParameters*>(RigidBody::getRigidBodyParameters( all_params ));

  if( all_params || valueUpdater->hasCausedEvent( maxProjectionIterations ) ) {
    params->setMaxProjectionIterations( maxProjectionIterations->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( separationTolerance ) ) {
    params->setSeparationTolerance( separationTolerance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( jointInternalCompliance ) ) {
    params->setJointInternalCompliance( jointInternalCompliance->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( jointExternalCompliance ) ) {
    params->setJointExternalCompliance( jointExternalCompliance->getValue() );
  }

  if( all_params ) {
    params->setStartPositions( positions->getValue() );
  }
  if( all_params ) {
    params->setStartOrientations( orientations->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( swingLimit ) ) {
    params->setSwingLimit( swingLimit->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( twistLimit ) ) {
    params->setTwistLimit( twistLimit->getValue() );
  }

  return params;
}

void ArticulatedRigidBody::renderCollidable( bool render_only_enabled_collidables ) {
  const std::vector<Vec3f> &body_poss = positions->getValue();
  const std::vector<Rotation> &body_orns = orientations->getValue();
  if( body_poss.size() != body_orns.size() || body_poss.size() != geometry->size() )
    return;

  glMatrixMode( GL_MODELVIEW );
  size_t i = 0;
  for( RigidBody::MFCollidableNode::const_iterator c = geometry->begin(); c != geometry->end(); c++, i++ ) {
    glPushMatrix();
    glTranslatef( body_poss[i].x, body_poss[i].y, body_poss[i].z );
    glRotatef( (GLfloat)(body_orns[i].angle * 180 / H3DUtil::Constants::pi), body_orns[i].axis.x, body_orns[i].axis.y, body_orns[i].axis.z );
    X3DNBodyCollidableNode *collidable = static_cast<X3DNBodyCollidableNode *>(*c);
    if( !render_only_enabled_collidables || collidable->enabled->getValue() ) {
      collidable->displayList->callList();
    }
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
  }

}

void ArticulatedRigidBody::initializeProxyBodies( H3D::PhysicsEngineThread& pt ) {
  if( !proxy_bodies_initialized ) {
    PhysicsEngineParameters::ArticulatedRigidBodyParameters params;
    pt.getRigidBodyParameters( getBodyId(), params );
    PhysicsEngineParameters::ArticulatedRigidBodyParameters::BodiesList proxy_body_ids = params.getBodies();

    if( proxy_body_ids.size() == proxyBodies->size() ) {
      const NodeVector &proxy_bodies = proxyBodies->getValue();
      for( size_t i = 0; i < proxy_bodies.size(); ++i ) {
        ProxyRigidBody* frb = static_cast<ProxyRigidBody*>(proxy_bodies[i]);
        frb->body_id = proxy_body_ids[i];
        frb->initializeBody( pt );
      }
      proxy_bodies_initialized = true;
    }
  }
}

void ArticulatedRigidBody::UpdateProxyBody::update() {
  ArticulatedRigidBody *body = static_cast<ArticulatedRigidBody*>(getOwner());
  if( body && routes_in.size() > 1 ) {
    const std::vector< Vec3f > &poss = static_cast<MFVec3f*>(routes_in[0])->getValue();
    const std::vector< Rotation > &orns = static_cast<MFRotation*>(routes_in[1])->getValue();

    if( poss.size() != orns.size() || poss.size() != body->proxyBodies->size() ) {
      //Console(4) << "Inconsistency between the sizes of positions, orientations and proxy bodies in articulated rigid body." << endl;
      return;
    }
    const NodeVector &ar_bodies = body->proxyBodies->getValue();
    for( size_t i = 0; i < ar_bodies.size(); ++i ) {
      ProxyRigidBody* frb = static_cast<ProxyRigidBody*>(ar_bodies[i]);
      frb->position->setValue( poss[i] );
      frb->orientation->setValue( orns[i] );
    }

  }
}

H3DNodeDatabase ProxyRigidBody::database( "ProxyRigidBody",
                                          &(newInstance< ProxyRigidBody >),
                                          typeid(ProxyRigidBody),
                                          &RigidBody::database );

ProxyRigidBody::ProxyRigidBody() :
  RigidBody() {

  // init fields
  type_name = "ProxyRigidBody";
  database.initFields( this );

}

bool ProxyRigidBody::initializeBody( PhysicsEngineThread& pt ) {
  if( isInitialized() ) return false;
  engine_thread = &pt;
  // Needs body Ids before this
  return H3DBodyNode::initializeBody( pt );
}

bool ProxyRigidBody::deleteBody() {
  if( isInitialized() ) {
    return H3DBodyNode::deleteBody();
  }
  return false;
}
