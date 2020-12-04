//////////////////////////////////////////////////////////////////////////////
//    Copyright 2019, SenseGraphics AB
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
/// \file PhysX4Callbacks.cpp
/// \brief Source file for PhysX4Callbacks, callback functions for
/// the PhysX4 physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysX4Callbacks.h>

#ifdef HAVE_PHYSX4

#include <H3D/H3DPhysics/PhysX4Joints.h>
#include <H3D/H3DPhysics/PhysXRigidBodyOptions.h>
#include <H3D/H3DPhysics/PhysXCollidableOptions.h>

#include <H3D/Box.h>
#include <H3D/Cone.h>
#include <H3D/Sphere.h>
#include <H3D/Cylinder.h>
#include <H3D/IndexedTriangleSet.h>
#include <H3D/Coordinate.h>

#ifdef HAVE_HACD
#include <hacdHACD.h>
#endif

using namespace H3D;

PhysicsEngineThread::PhysicsEngineRegistration
PhysX4Callbacks::registration( "PhysX4", 
                              PhysicsEngineThread::createPhysicsEngineCallbacks< PhysX4Callbacks >() );

std::vector< PhysX4RigidBody* > PhysX4RigidBody::all_physX4_bodies;

PhysX4RigidBody::PhysX4RigidBody ( PhysicsEngineParameters::RigidBodyParameters& _params ) :
  mass ( 0 ), 
  massDensityModel ( NULL ), 
  fixed ( false ), 
  kinematic ( false ),
  autoDisable ( false ),
  disableLinearSpeed ( 0 ),
  disableAngularSpeed ( 0 ),
  isStatic( false ),
  rigid_body( NULL ) {

  PhysX4RigidBody::all_physX4_bodies.push_back( this );
}

PhysX4RigidBody::~PhysX4RigidBody () {
  // Remove geometry
  for ( ShapeVector::const_iterator i= shapes.begin(); i != shapes.end(); ++i ) {
    PhysX4CollidableShape* shape= *i;
    //Console( LogLevel::Warning ) << "Removing cs: " << shape << " from body id: " << this << endl;
    shape->removeFromBody();
  }
  shapes.clear();

  if( rigid_body ) {
    rigid_body->release();
    rigid_body= NULL;
  }

  std::vector< PhysX4RigidBody* >::iterator j = std::find( all_physX4_bodies.begin(), all_physX4_bodies.end(), this );
  if( j != all_physX4_bodies.end() ) {
    all_physX4_bodies.erase( j );
  }
}

void PhysX4RigidBody::removeCollidableShape( PhysX4CollidableShape* s ){
  
  // This is called by PhysX4CollidableShape::removeFromBody, therefore never call that function.
  for ( ShapeVector::const_iterator i = shapes.begin(); i != shapes.end(); ++i ) {
    PhysX4CollidableShape* shape= *i;
    if( shape == s ){
      shapes.erase( i );
      return;
    }
  }
}
  
void PhysX4RigidBody::createRigidBody( PhysX4Callbacks::PhysX4SpecificData *physx_data ) {
  rigid_body= physx_data->sdk->createRigidDynamic( PxTransform( PxIdentity ) );
  rigid_body->userData= this;
}

void PhysX4RigidBody::createRigidBodyStatic( PhysX4Callbacks::PhysX4SpecificData *physx_data ) {
  rigid_body = physx_data->sdk->createRigidStatic( PxTransform( PxIdentity ) );
  rigid_body->userData = this;
}


void PhysX4RigidBody::setParameters ( PhysicsEngineParameters::RigidBodyParameters& _params ) {

  // Auto disable
  if ( _params.haveAutoDisable() ) {
    autoDisable= _params.getAutoDisable();
  }

  // Auto disable linear speed
  if ( _params.haveDisableLinearSpeed() ) {
    disableLinearSpeed= _params.getDisableLinearSpeed();
  }

  // Auto disable angular speed
  if ( _params.haveDisableAngularSpeed() ) {
    disableAngularSpeed= _params.getDisableAngularSpeed();
  }

  if ( _params.haveAutoDisable() || 
       _params.haveDisableLinearSpeed() ||
       _params.haveDisableAngularSpeed() ) {
    setSleeping ( _params );
  }

  // start position
  if ( _params.haveStartPosition() ) {
    rigid_body->setGlobalPose ( 
      PxTransform(toPxVec3(_params.getStartPosition()),rigid_body->getGlobalPose().q) );
  }
  // start orientation
  if ( _params.haveStartOrientation() ) {
    rigid_body->setGlobalPose ( 
      PxTransform(rigid_body->getGlobalPose().p,toPxQuat(_params.getStartOrientation())) );
  }

  // Enable / disable kinematic control
  if ( _params.haveKinematicControl() ) {
    kinematic= _params.getKinematicControl();
  }

  // Fixed
  if ( _params.haveFixed() ) {
    fixed= _params.getFixed();
  }

  if( rigid_body->is<PxRigidDynamic>() ) {

    // Enable (sleeping)
    if( _params.haveEnabled() ) {
      if( _params.getEnabled() ) {
        ((PxRigidDynamic*)rigid_body)->wakeUp();
      } else {
        ((PxRigidDynamic*)rigid_body)->putToSleep();
      }
    }

    // Make body kinematic if fixed or kinematic is enabled
    if( _params.haveKinematicControl() || _params.haveFixed() ) {
      ((PxRigidDynamic*)rigid_body)->setRigidBodyFlag( PxRigidBodyFlag::eKINEMATIC, kinematic || fixed );
    }

    // Kinematic control
    if( kinematic ) {
      // Position
      if( _params.havePosition() ) {
        rigid_body->setGlobalPose( PxTransform( toPxVec3( _params.getPosition() ), rigid_body->getGlobalPose().q ) );
      }

      // Orientation
      if( _params.haveOrientation() ) {
        rigid_body->setGlobalPose( PxTransform( rigid_body->getGlobalPose().p, toPxQuat( _params.getOrientation() ) ) );
      }
    }
  }

  
  // geometry
  if( _params.haveGeometry() ) {

    // Remove old geometry
    for ( ShapeVector::const_iterator i= shapes.begin(); i != shapes.end(); ++i ) {
      PhysX4CollidableShape* shape= *i;
      //Console( LogLevel::Warning ) << "Removing cs: " << shape << " from body bid: " << this << endl;
      shape->removeFromBody();
    }
    shapes.clear();

    // Add new geometry
    const vector< H3DCollidableId > geom_ids = _params.getGeometry();
    for ( vector< H3DCollidableId >::const_iterator i= geom_ids.begin(); i != geom_ids.end(); ++i ) {
      PhysX4CollidableShape* shape= (PhysX4CollidableShape*)*i;
      if( shape != NULL ) {
        PhysX4Callbacks::PhysX4SpecificData *physx_data = 
          static_cast< PhysX4Callbacks::PhysX4SpecificData * >(_params.getEngine()->getEngineSpecificData());

        shape->addToBody ( *this, *physx_data );
        shapes.push_back ( shape );
      }
    }
  }

  // This, which is setting mass related parameters, needs to be
  // called after the collidable shapes are added to the rigid body. Since in
  // some cases the collidable shapes are used to calculate the inertia of mass.
  if( rigid_body->is<PxRigidBody>() ) {
    setPxRigidBodyParameters( (PxRigidBody*)rigid_body, _params );
  }

  // PhysX4 specific rigid body options
  if ( _params.haveEngineOptions() ) {
    if ( PhysXRigidBodyParameters* options = dynamic_cast<PhysXRigidBodyParameters*>(_params.getEngineOptions()) ) {

      isStatic = options->getCreateAsStatic();

      if( rigid_body->is<PxRigidDynamic>() ) {
        // contactReportThreshold
        if( options->haveContactReportThreshold() ) {
          ((PxRigidDynamic*)rigid_body)->setContactReportThreshold( options->getContactReportThreshold() );
        }

        // solverPositionIterations
        if( options->haveSolverPositionIterations() ) {
          PxU32 p, v;
          ((PxRigidDynamic*)rigid_body)->getSolverIterationCounts( p, v );
          ((PxRigidDynamic*)rigid_body)->setSolverIterationCounts( options->getSolverPositionIterations(), v );
        }

        // solverVelocityIterations
        if( options->haveSolverVelocityIterations() ) {
          PxU32 p, v;
          ((PxRigidDynamic*)rigid_body)->getSolverIterationCounts( p, v );
          ((PxRigidDynamic*)rigid_body)->setSolverIterationCounts( p, options->getSolverVelocityIterations() );
        }

        if( options->haveLinearVelocityDamping() ) {
          ((PxRigidDynamic*)rigid_body)->setLinearDamping( options->getLinearVelocityDamping() );
        }

        if( options->haveAngularVelocityDamping() ) {
          ((PxRigidDynamic*)rigid_body)->setAngularDamping( options->getAngularVelocityDamping() );
        }
      }

      if( rigid_body->is<PxRigidBody>() && options->haveMaxDepenetrationVelocity() ) {
        maxDepenetrationVelocity = options->getMaxDepenetrationVelocity();
        ((PxRigidBody*)rigid_body)->setMaxDepenetrationVelocity( (PxReal)maxDepenetrationVelocity );
      }
    }
  }
}

void PhysX4RigidBody::setPxRigidBodyParameters( PxRigidBody* _body, PhysicsEngineParameters::RigidBodyParameters& _params ){
  if( _body ){

    // useGlobalGravity
    if ( _params.haveUseGlobalGravity() ) {
      _body->setActorFlag ( PxActorFlag::eDISABLE_GRAVITY, !_params.getUseGlobalGravity() );
    }

  // Mass
  if ( _params.haveMass() ) {
    mass= _params.getMass();
      _body->setMass ( mass );
  }
    
  // Mass density model
  if ( _params.haveMassDensityModel() ) {
    massDensityModel= _params.getMassDensityModel();
  }
    
  // Center of mass
  if ( _params.haveCenterOfMass() ) {
    centerOfMass= _params.getCenterOfMass();
  }
    
  // Inertia
  if ( _params.haveInertia() ) {
    inertia= _params.getInertia();
  }
    
  // mass, massDensityModel, inertia and centerOfMass
  if( _params.haveMass() || _params.haveMassDensityModel() || 
      _params.haveCenterOfMass() || _params.haveInertia() ) {
    // Reference: http://en.wikipedia.org/wiki/List_of_moment_of_inertia_tensors
    Node *n = massDensityModel;
    if ( Box *box = dynamic_cast< Box* >( n ) ) {
      // Box moment of inertia
      Vec3f s= box->size->getValue();
        _body->setMassSpaceInertiaTensor ( 
        PxVec3 ( (mass*(s.y*s.y + s.z*s.z))/12.0f,
                 (mass*(s.x*s.x + s.z*s.z))/12.0f,
                 (mass*(s.x*s.x + s.y*s.y))/12.0f ) );
    } else if ( Sphere *sph = dynamic_cast< Sphere* >( n ) ) {
      // Sphere moment of inertia
      H3DFloat r= sph->radius->getValue();
      H3DFloat m= (2*mass*r*r)/5.0f;
        _body->setMassSpaceInertiaTensor ( PxVec3 ( m, m, m ) );
    } else if ( Cylinder *cyl = dynamic_cast< Cylinder* >( n ) ) {
      // Cylinder moment of inertia
      H3DFloat r= cyl->radius->getValue();
      H3DFloat h= cyl->height->getValue();
        H3DFloat m= mass;
        _body->setMassSpaceInertiaTensor ( 
        PxVec3 ( (m*(3*r*r+h*h))/12.0f,
                 (m*(3*r*r+h*h))/12.0f,
                 (m*r*r)/2.0f ) );
    } else {
      if ( n ) {
        Console( LogLevel::Warning ) << "Warning: Mass density model " << n->getTypeName() << " is not supported by PhysX4!" 
                   << " Inertia will be calculated based on collision geometry instead with mass " << mass << endl;
    
        // No mass density model supplied, calculate it automatically based on the
        // current collision shapes
          bool success = PxRigidBodyExt::setMassAndUpdateInertia  ( *_body, mass );
          if( !success ) {
            Console( LogLevel::Error ) << "Failed to compute inertia"<< endl;
          }
      } else {
        // Set user defined center of mass and inertia
          _body->setCMassLocalPose ( PxTransform ( toPxVec3 ( centerOfMass ) ) );
        Matrix3f m= inertia;
          _body->setMassSpaceInertiaTensor ( PxVec3 ( m[0][0], m[1][1], m[2][2] ) );
      }
    }
  }
      }
}

void PhysX4RigidBody::getParameters ( PhysicsEngineParameters::RigidBodyParameters& _params ) {
  // position
  _params.setPosition( toVec3f ( rigid_body->getGlobalPose().p ) );
  // orientation  
  _params.setOrientation( toRotation ( rigid_body->getGlobalPose().q ) );

  if( rigid_body->is<PxRigidDynamic>() ){
    // linear velocity
    _params.setLinearVelocity( toVec3f( ((PxRigidDynamic*)rigid_body)->getLinearVelocity() ) );
    // angular velocity
    _params.setAngularVelocity( toVec3f( ((PxRigidDynamic*)rigid_body)->getAngularVelocity() ) );
  
    _params.setEnabled( !((PxRigidDynamic*)rigid_body)->isSleeping() );
  }
}

void PhysX4RigidBody::setSleeping ( PhysicsEngineParameters::RigidBodyParameters& _params ) {
  if( rigid_body->is<PxRigidDynamic>() ) {
    if( _params.getAutoDisable() ) {
      H3DFloat v_l = disableLinearSpeed;
      H3DFloat v_r = disableAngularSpeed;
      H3DFloat massNormalizedK = (v_l*v_l + v_r*v_r) / 2;

      // Actors whose kinetic energy divided by their mass is above this threshold will not be put to sleep.
      ((PxRigidDynamic*)rigid_body)->setSleepThreshold( massNormalizedK );
    } else {
      WorldParameters worldParams = _params.getEngine()->getWorldParameters();

      if( worldParams.getAutoDisable() ) {
        H3DFloat v_l = worldParams.getDisableLinearSpeed();
        H3DFloat v_r = worldParams.getDisableAngularSpeed();
        H3DFloat massNormalizedK = (v_l*v_l + v_r*v_r) / 2;

        // Actors whose kinetic energy divided by their mass is above this threshold will not be put to sleep.
        ((PxRigidDynamic*)rigid_body)->setSleepThreshold( massNormalizedK );
      } else {
        ((PxRigidDynamic*)rigid_body)->setSleepThreshold( 0 );
      }
    }
  }
}

PhysX4ArticulationLink::PhysX4ArticulationLink ( PhysicsEngineParameters::RigidBodyParameters& _params,
  PxArticulation* _articulation, PxArticulationLink* _link ) :
  PhysX4RigidBody( _params ), articulation( _articulation ), link( _link ) {
  //
}

PhysX4ArticulationLink::~PhysX4ArticulationLink () {
  //
}

PhysX4ArticulatedRigidBody::PhysX4ArticulatedRigidBody ( PhysicsEngineParameters::RigidBodyParameters& _params ) :
  PhysX4RigidBody( _params ), articulation( NULL ) {
}

PhysX4ArticulatedRigidBody::~PhysX4ArticulatedRigidBody () {
  // Remove geometry
  for ( ShapeVector::const_iterator i= shapes.begin(); i != shapes.end(); ++i ) {
    PhysX4CollidableShape* shape= *i;
    shape->removeFromBody();
  }
  shapes.clear();

  for ( ArticulationLinkList::const_iterator i= articulation_links.begin(); i != articulation_links.end(); ++i ) {
    PhysX4ArticulationLink* link= *i;
    delete link;
  }

  // The only links that are allowed to be released are leaf links.
  // For this reason we need to go backwards to release the rigid bodies (links).
  for( PxRigidBodyList::const_reverse_iterator i = rigid_bodies.rbegin(); i != rigid_bodies.rend(); ++i ) {
    (*i)->release();
  }
  rigid_bodies.clear();

  articulation->release();
  articulation=NULL;
}

void PhysX4ArticulatedRigidBody::createRigidBody( PhysX4Callbacks::PhysX4SpecificData *physx_data ) {
  articulation = physx_data->sdk->createArticulation();
  articulation->userData = this;
}

void PhysX4ArticulatedRigidBody::setParameters ( PhysicsEngineParameters::RigidBodyParameters& _params ) {
  // NOTE: Articulation does not support, in PhysX 3.3.1, kinematic, damping and velocity clamping.

  ArticulatedRigidBodyParameters* a_params = static_cast<ArticulatedRigidBodyParameters*>( &_params );

  // Auto disable
  if ( _params.haveAutoDisable() ) {
    autoDisable= _params.getAutoDisable();
  }

  // Auto disable linear speed
  if ( _params.haveDisableLinearSpeed() ) {
    disableLinearSpeed= _params.getDisableLinearSpeed();
  }

  // Auto disable angular speed
  if ( _params.haveDisableAngularSpeed() ) {
    disableAngularSpeed= _params.getDisableAngularSpeed();
  }

  if ( _params.haveAutoDisable() || 
       _params.haveDisableLinearSpeed() ||
       _params.haveDisableAngularSpeed() ) {
    setSleeping ( _params );
  }
  
  // start position
  ArticulatedRigidBodyParameters::PositionList s_positions;
  if ( a_params->haveStartPositions() ) {
    s_positions = a_params->getStartPositions();
  }

  // start orientation
  ArticulatedRigidBodyParameters::OrientationList s_orientations;
  if ( a_params->haveStartOrientations() ) {
    s_orientations = a_params->getStartOrientations();
  }
  
  // geometry
  if( _params.haveGeometry() ) {

    const vector< H3DCollidableId > geom_ids = _params.getGeometry();
    if( geom_ids.size() != s_positions.size() || geom_ids.size() != s_orientations.size() ){
      Console( LogLevel::Warning ) << "ERROR: The position and orientation field must have same number of elements with the geometry of the ArticulatedRigidBody." << endl;
      return;
    }

    // Remove old geometry
    for ( ShapeVector::const_iterator i= shapes.begin(); i != shapes.end(); ++i ) {
      PhysX4CollidableShape* shape= *i;
      shape->removeFromBody();
    }
    shapes.clear();

    rigid_body = NULL;
    rigid_bodies.clear();
    articulation_links.clear();
    unsigned int body_index = 0;
    PxArticulationLink* parent = NULL;

    PhysX4Callbacks::PhysX4SpecificData *physx_data = 
      static_cast< PhysX4Callbacks::PhysX4SpecificData * >(_params.getEngine()->getEngineSpecificData());

    // Add new geometry
    for ( vector< H3DCollidableId >::const_iterator i= geom_ids.begin(); i != geom_ids.end(); ++i ) {

      PhysX4CollidableShape* shape= (PhysX4CollidableShape*)*i;
      if( shape != NULL ) {
        
        PxArticulationLink* link = articulation->createLink( parent, PxTransform(toPxVec3( s_positions[body_index] ), toPxQuat( s_orientations[body_index] ) ) );
        PhysX4ArticulationLink* articulation_link = new PhysX4ArticulationLink( _params, articulation, link );
        link->userData = articulation_link;
        articulation_links.push_back( articulation_link );
        shape->addToBody( *articulation_link, *physx_data );
        shapes.push_back ( shape );

        // Use 1.0f as fallback because setting a mass of 0.0 will cause an assertion
        PxRigidBodyExt::updateMassAndInertia(*link, _params.haveMass() ? _params.getMass() : 1.0f );

        // Set the position of the joint w.r.t bodies.
        PxArticulationJoint *joint = static_cast<PxArticulationJoint*>( link->getInboundJoint() );
        if( joint ){
          Vec3f pos_g = 0.5*(s_positions[body_index] + s_positions[body_index-1]);
          PxTransform trans_g( toPxVec3(pos_g) );

          joint->setParentPose( rigid_bodies[body_index-1]->getGlobalPose().getInverse()*trans_g );
          joint->setChildPose(  link->getGlobalPose().getInverse()*trans_g );
        }

        rigid_bodies.push_back( link );
        parent = link;
        body_index++;


      }
    }
  }

  // This, which is setting mass related parameters, needs to be
  // called after the collidable shapes are added to the rigid body. Since in
  // some cases the collidable shapes are used to calculate the inertia of mass.
  // Update parameters for each rigid_actor
  for( size_t i=0; i<rigid_bodies.size(); ++i ){
    setPxRigidBodyParameters( rigid_bodies[i], _params );
  }

  // Articulation parameters
  
  // Max projection interations
  if ( a_params->haveMaxProjectionIterations() ) {
      articulation->setMaxProjectionIterations( a_params->getMaxProjectionIterations() );
  }

  // Separation tolerance used in projection
  if ( a_params->haveSeparationTolerance() ) {
      articulation->setSeparationTolerance( a_params->getSeparationTolerance() );
  }

  // Internal compliance of joints affecting the response to internal forces, i.e from other joints
  if ( a_params->haveJointInternalCompliance() ) {
    H3DFloat comp = a_params->getJointInternalCompliance();
    for( size_t i=0; i<articulation_links.size(); ++i ){
      PxArticulationLink* lnk = static_cast<PxArticulationLink*>( articulation_links[i]->getActor() );
      PxArticulationJoint *joint = static_cast<PxArticulationJoint*>( lnk->getInboundJoint() );
      if( joint && comp > 0 ) {
        joint->setInternalCompliance( comp );
      }
    }
  }

  // External compliance of joints affecting the response to external forces, i.e gravity
  if ( a_params->haveJointExternalCompliance() ) {
    H3DFloat comp = a_params->getJointExternalCompliance();
    for( size_t i=0; i<articulation_links.size(); ++i ){
      PxArticulationLink* lnk = static_cast<PxArticulationLink*>( articulation_links[i]->getActor() );
      PxArticulationJoint *joint = static_cast<PxArticulationJoint*>( lnk->getInboundJoint() );
      if( joint && comp > 0 ) {
        joint->setExternalCompliance( comp );
      }
    }
  }

  // swing limit
  vector< Vec2f > swing_limits = a_params->getSwingLimit();
  bool enable_swing_limit = a_params->haveSwingLimit();
  for( size_t i=1; i<articulation_links.size(); ++i ){
    bool joint_swing_limit_enabled = false;
    PxArticulationLink* lnk = static_cast<PxArticulationLink*>( articulation_links[i]->getActor() );
    PxArticulationJoint *joint = static_cast<PxArticulationJoint*>( lnk->getInboundJoint() );
    if( joint ) {
      size_t swing_index = i - 1;
      if( enable_swing_limit && swing_index < swing_limits.size() ) {
        if( swing_limits[swing_index].x > 0 && swing_limits[swing_index].x < Constants::pi &&
            swing_limits[swing_index].y > 0 && swing_limits[swing_index].y < Constants::pi ) {
          joint->setSwingLimit( swing_limits[swing_index].x, swing_limits[swing_index].y );
          joint_swing_limit_enabled = true;
        }
      }
      if( joint->getSwingLimitEnabled() != joint_swing_limit_enabled ) {
        joint->setSwingLimitEnabled( joint_swing_limit_enabled );
      }
    }
  }


  vector< Vec2f > twist_limits = a_params->getTwistLimit();
  bool enable_twist_limit = a_params->haveTwistLimit();
  for( size_t i=1; i<articulation_links.size(); ++i ){
    bool joint_twist_limit_enabled = false;
    PxArticulationLink* lnk = static_cast<PxArticulationLink*>( articulation_links[i]->getActor() );
    PxArticulationJoint *joint = static_cast<PxArticulationJoint*>( lnk->getInboundJoint() );
    if( joint ) {
      size_t twist_index = i - 1;
      if( enable_twist_limit && twist_index < twist_limits.size() ) {
        if( twist_limits[twist_index].x > -Constants::pi && twist_limits[twist_index].x < Constants::pi &&
            twist_limits[twist_index].y > -Constants::pi && twist_limits[twist_index].y < Constants::pi ) {
          joint->setTwistLimit( twist_limits[twist_index].x, twist_limits[twist_index].y );
          joint_twist_limit_enabled = true;
        }
      }
      if( joint->getTwistLimitEnabled() != joint_twist_limit_enabled ) {
        joint->setTwistLimitEnabled( joint_twist_limit_enabled );
      }
    }
  }

  // PhysX4 specific rigid body options
  if ( _params.haveEngineOptions() ) {
    if ( PhysXRigidBodyParameters* options= 
      dynamic_cast<PhysXRigidBodyParameters*>(_params.getEngineOptions()) ) {

      // solverPositionIterations
      if ( options->haveSolverPositionIterations() ) {
        PxU32 p, v;
        articulation->getSolverIterationCounts ( p, v );
        articulation->setSolverIterationCounts ( options->getSolverPositionIterations(), v );
      }

      // solverVelocityIterations
      if ( options->haveSolverVelocityIterations() ) {
        PxU32 p, v;
        articulation->getSolverIterationCounts ( p, v );
        articulation->setSolverIterationCounts ( p, options->getSolverVelocityIterations() );
      }

    }
  }
  
}

void PhysX4ArticulatedRigidBody::getParameters ( PhysicsEngineParameters::RigidBodyParameters& _params ) {
  ArticulatedRigidBodyParameters* a_params = static_cast<ArticulatedRigidBodyParameters*>( &_params );
  
  // position
  ArticulatedRigidBodyParameters::PositionList c_positions;
  // orientation  
  ArticulatedRigidBodyParameters::OrientationList c_orientations;
  // BodyIds
  ArticulatedRigidBodyParameters::BodiesList body_Ids;
  

  for( size_t i=0; i<rigid_bodies.size(); ++i ){
    c_positions.push_back( toVec3f( rigid_bodies[i]->getGlobalPose().p ) );
    c_orientations.push_back( toRotation( rigid_bodies[i]->getGlobalPose().q ) );
    body_Ids.push_back( (H3DBodyId)( articulation_links[i] ) );
  }  

  a_params->setPositions( c_positions ); 
  a_params->setOrientations( c_orientations );
  a_params->setBodies( body_Ids );
}

void PhysX4ArticulatedRigidBody::setSleeping ( PhysicsEngineParameters::RigidBodyParameters& _params ) {
  if ( _params.getAutoDisable() ) {
    H3DFloat v_l= disableLinearSpeed;
    H3DFloat v_r= disableAngularSpeed;
    H3DFloat massNormalizedK= (v_l*v_l+v_r*v_r)/2;

    // Actors whose kinetic energy divided by their mass is above this threshold will not be put to sleep.
    articulation->setSleepThreshold( massNormalizedK );
  } else {
    WorldParameters worldParams= _params.getEngine()->getWorldParameters();

    if ( worldParams.getAutoDisable() ) {
      H3DFloat v_l= worldParams.getDisableLinearSpeed();
      H3DFloat v_r= worldParams.getDisableAngularSpeed();
      H3DFloat massNormalizedK= (v_l*v_l+v_r*v_r)/2;

      // Actors whose kinetic energy divided by their mass is above this threshold will not be put to sleep.
      articulation->setSleepThreshold( massNormalizedK );
    } else {
      articulation->setSleepThreshold( 0 );
    }
  }
}

struct lt_PxVec3 {
  bool operator()(const PxVec3& v1, const PxVec3& v2) const {
    return (v1.x<v2.x) || (v1.x==v2.x && v1.y<v2.y) || (v1.x==v2.x && v1.y==v2.y && v1.z<v2.z);
  }
};

namespace PhysX4_H3DInternal {
#ifdef HAVE_HACD
  // Helper function to debug HACD by loading input directly from file

  bool LoadOFF(const std::string & fileName, std::vector< HACD::Vec3<HACD::Real> > & points, std::vector< HACD::Vec3<long> > & triangles, bool invert)
  {
    FILE * fid = fopen(fileName.c_str(), "r");
    if (fid)
    {
      const std::string strOFF("OFF");
      char temp[1024];
      fscanf(fid, "%s", temp);
      if (std::string(temp) != strOFF)
      {
        printf("Loading error: format not recognized \n");
        fclose(fid);

        return false;
      }
      else
      {
        int nv = 0;
        int nf = 0;
        int ne = 0;
        fscanf(fid, "%i", &nv);
        fscanf(fid, "%i", &nf);
        fscanf(fid, "%i", &ne);
        points.resize(nv);
        triangles.resize(nf);
        HACD::Vec3<HACD::Real> coord;
        float x = 0;
        float y = 0;
        float z = 0;
        for (long p = 0; p < nv; ++p)
        {
          fscanf(fid, "%f", &x);
          fscanf(fid, "%f", &y);
          fscanf(fid, "%f", &z);
          points[p].X() = x;
          points[p].Y() = y;
          points[p].Z() = z;
        }
        int i = 0;
        int j = 0;
        int k = 0;
        int s = 0;
        for (long t = 0; t < nf; ++t) {
          fscanf(fid, "%i", &s);
          if (s == 3)
          {
            fscanf(fid, "%i", &i);
            fscanf(fid, "%i", &j);
            fscanf(fid, "%i", &k);
            triangles[t].X() = i;
            if (invert)
            {
              triangles[t].Y() = k;
              triangles[t].Z() = j;
            }
            else
            {
              triangles[t].Y() = j;
              triangles[t].Z() = k;
            }
          }
          else      // Fix me: support only triangular meshes
          {
            for (long h = 0; h < s; ++h) fscanf(fid, "%i", &s);
          }
        }
        fclose(fid);
      }
    }
    else
    {
      printf("Loading error: file not found \n");
      return false;
    }
    return true;
  }

  // Helper function to debug HACD by saving result to file
  bool SaveOFF(const std::string & fileName, size_t nV, size_t nT, const HACD::Vec3<HACD::Real> * const points, const HACD::Vec3<long> * const triangles)
  {
    Console( LogLevel::Info ) << "Saving " << fileName << std::endl;
    std::ofstream fout(fileName.c_str());
    if (fout.is_open())
    {
      fout << "OFF" << std::endl;
      fout << nV << " " << nT << " " << 0 << std::endl;
      for (size_t v = 0; v < nV; ++v)
      {
        fout << points[v].X() << " "
          << points[v].Y() << " "
          << points[v].Z() << std::endl;
      }
      for (size_t f = 0; f < nT; ++f)
      {
        fout << "3 " << triangles[f].X() << " "
          << triangles[f].Y() << " "
          << triangles[f].Z() << std::endl;
      }
      fout.close();
      return true;
    }
    return false;
  }
#endif // HAVE_HACD

  vector< void * > collidables_with_contact_mode_all;
}

PhysX4CollidableShape::PhysX4CollidableShape ( PhysicsEngineParameters::ShapeParameters& _params ) :
  body ( NULL ), collisionEnabled ( true ), exceptionGroupMask(0), selectionGroupMask(0), restOffset(0.0), contactOffset(0.02),
    suppressDisabledContacts( true ), setFlagsForAll( false ), contactFilterOptionsMask( 0 ) {
  PhysX4Callbacks::PhysX4SpecificData *physx_data = 
    static_cast< PhysX4Callbacks::PhysX4SpecificData * >(_params.getEngine()->getEngineSpecificData());

  // Default
  contactFilterOptionsMask |= PxPairFlag::eCONTACT_DEFAULT; // PxPairFlag::eDETECT_DISCRETE_CONTACT | PxPairFlag::eSOLVE_CONTACT
  contactFilterOptionsMask |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
  contactFilterOptionsMask |= PxPairFlag::eNOTIFY_TOUCH_PERSISTS;

  if ( Sphere* sphere= dynamic_cast<Sphere*>(_params.getShape()) ) {
    geometry.push_back ( new PxSphereGeometry ( sphere->radius->getValue() ) );
  } else if ( Box* box= dynamic_cast<Box*>(_params.getShape()) ) {
    Vec3f size= box->size->getValue();
    geometry.push_back (  new PxBoxGeometry ( size.x/2, size.y/2, size.z/2 ) );
  } else if ( Cylinder* cylinder= dynamic_cast<Cylinder*>(_params.getShape()) ) {
    geometry.push_back ( new PxCapsuleGeometry ( cylinder->radius->getValue(), cylinder->height->getValue()/2 ) );
    rotationOffset= Rotation(0,0,1,(H3DFloat)H3DUtil::Constants::pi/2);
  } else {
    // get triangles from geometry
    vector< HAPI::Collision::Triangle > triangles;
    
    if( _params.haveShape() ) {    
      if( _params.getShape()->boundTree->isUpToDate() ) {
        // Only do getValue() if the tree is already uptodate, else causes OpenGL error
        _params.getShape()->boundTree->getValue()->getAllTriangles( triangles );
      }
    }

    // if there are no triangles built then don't create a shape
    if( triangles.size() > 0 ) {
      PhysXCollidableParameters options;
      if( _params.haveEngineOptions() ) {
        if( PhysXCollidableParameters* p =
          dynamic_cast<PhysXCollidableParameters*>(_params.getEngineOptions()) ) {
          options = *p;
        }
      }

      // Create appropriate collision shape
      if( options.getConvex() ) {
        // Mesh is already convex

        // Load the pre-cooked mesh if it exists
        string filename = options.getCookedFilename();
        bool load_cooked = filename != "";
        PxConvexMesh* convexMesh = NULL;
        if( load_cooked ) {
          auto_ptr< PxDefaultFileInputData > input( new PxDefaultFileInputData( filename.c_str() ) );
          if( !input->isValid() && options.getBaseURL() != "" ) {
            string filename_tmp = options.getBaseURL() + filename;
            input.reset( new PxDefaultFileInputData( filename_tmp.c_str() ) );
          }
          if( input->isValid() ) {
            convexMesh = physx_data->sdk->createConvexMesh( *input.get() );
            if( convexMesh ) {
              geometry.push_back( new PxConvexMeshGeometry( convexMesh ) );
            }
          }
        }

        if( !convexMesh ) {
          // Remove duplicates
          set<PxVec3, lt_PxVec3> points;
          for( unsigned int i = 0; i < triangles.size(); ++i ) {
            points.insert( toPxVec3( Vec3f( triangles[i].a ) ) );
            points.insert( toPxVec3( Vec3f( triangles[i].b ) ) );
            points.insert( toPxVec3( Vec3f( triangles[i].c ) ) );
          }

          // Copy unique points
          PxVec3* verts = new PxVec3[points.size()];
          size_t j = 0;
          for( set<PxVec3, lt_PxVec3>::iterator i = points.begin(); i != points.end(); ++i ) {
            verts[j++] = *i;
          }

          // Create mesh description
          PxConvexMeshDesc convexDesc;
          assert( points.size() <= std::numeric_limits<PxU32>::max() );
          convexDesc.points.count = static_cast<PxU32>(points.size());
          convexDesc.points.stride = sizeof( PxVec3 );
          convexDesc.points.data = verts;
          convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

          // Cook the mesh
          if( load_cooked ) {
            bool success = false;
            {
              auto_ptr< PxDefaultFileOutputStream > buf( new PxDefaultFileOutputStream( filename.c_str() ) );
              if( !buf->isValid() && options.getBaseURL() != "" ) {
                filename = options.getBaseURL() + filename;
                buf.reset( new PxDefaultFileOutputStream( filename.c_str() ) );
              }
              if( buf->isValid() )
                success = physx_data->cooking->cookConvexMesh( convexDesc, *buf.get() );
            }
            if( !success ) {
              Console( LogLevel::Warning ) << "Error: PhysX4 error cooking convex mesh!" << endl;
            } else {
              // Load the cooked mesh
              PxDefaultFileInputData input( filename.c_str() );
              if( input.isValid() ) {
                convexMesh = physx_data->sdk->createConvexMesh( input );
                geometry.push_back( new PxConvexMeshGeometry( convexMesh ) );
              } else
                Console( LogLevel::Warning ) << "Error: PhysX4 error creating convex mesh!" << endl;
            }
          } else {
            PxDefaultMemoryOutputStream buf;
            if( !physx_data->cooking->cookConvexMesh( convexDesc, buf ) ) {
              Console( LogLevel::Warning ) << "Error: PhysX4 error cooking convex mesh!" << endl;
            } else {
              // Load the cooked mesh
              PxDefaultMemoryInputData input( buf.getData(), buf.getSize() );
              convexMesh = physx_data->sdk->createConvexMesh( input );
              geometry.push_back( new PxConvexMeshGeometry( convexMesh ) );
            }
          }

          delete[] verts;

        }
      } else {
#ifdef HAVE_HACD
        // If we have HACD and the convex decomposition is requested then
      // attempt to decompose the mesh into multiple convex hulls
        if( options.getConvexDecomposition() ) {

          string filename = options.getCookedFilename();
          bool load_cooked = filename != "";

          size_t c;
          if( load_cooked ) {
            for( c = 0; true; ++c ) {
              stringstream ss;
              ss << filename << c;

              auto_ptr< PxDefaultFileInputData > input( new PxDefaultFileInputData( ss.str().c_str() ) );
              if( !input->isValid() && options.getBaseURL() != "" ) {
                stringstream ss1;
                ss1 << options.getBaseURL() << filename << c;
                input.reset( new PxDefaultFileInputData( ss1.str().c_str() ) );
              }
              PxConvexMesh* mesh = NULL;
              if( input->isValid() ) {
                mesh = physx_data->sdk->createConvexMesh( *input.get() );
              }
              if( mesh ) {
                geometry.push_back( new PxConvexMeshGeometry( mesh ) );
              } else {
                break;
              }
            }
          }

          if( !load_cooked || c == 0 ) {
            // This can take a long time, ouput message to console
            // Use PhysX4CollidableOptions::cookedFilename to cache the result
            if( !load_cooked ) {
              Console( LogLevel::Warning ) << "Warning: Consider caching convex decomposition using PhysX4CollidableOptions::cookedFilename" << endl;
            }

            Console( LogLevel::Info ) << "Note: Doing convex decomposition..." << endl;

            std::vector< HACD::Vec3<HACD::Real> > points;
            std::vector< HACD::Vec3<long> > tris;

            IndexedTriangleSet* triangle_set = dynamic_cast<IndexedTriangleSet*>(_params.getShape());
            Coordinate* coord = NULL;
            if( triangle_set ) {
              coord = dynamic_cast<Coordinate*>(triangle_set->coord->getValue());
            }
            if( coord ) {
              const std::vector<Vec3f>& tri_points = coord->point->getValue();
              const std::vector<H3DInt32>& tri_index = triangle_set->index->getValue();

              for( std::vector<Vec3f>::const_iterator i = tri_points.begin(); i != tri_points.end(); ++i ) {
                const Vec3f& v = *i;
                points.push_back( HACD::Vec3<HACD::Real>( v.x, v.y, v.z ) );
              }

              for( std::vector<H3DInt32>::const_iterator i = tri_index.begin(); i != tri_index.end(); i += 3 ) {
                tris.push_back( HACD::Vec3<long>( *i, *(i + 1), *(i + 2) ) );
              }
            } else {
              long j = 0;
              for( size_t i = 0; i < triangles.size(); ++i ) {
                points.push_back( HACD::Vec3<HACD::Real>( triangles[i].a.x, triangles[i].a.y, triangles[i].a.z ) );
                points.push_back( HACD::Vec3<HACD::Real>( triangles[i].b.x, triangles[i].b.y, triangles[i].b.z ) );
                points.push_back( HACD::Vec3<HACD::Real>( triangles[i].c.x, triangles[i].c.y, triangles[i].c.z ) );
                tris.push_back( j++ );
                tris.push_back( j++ );
                tris.push_back( j++ );
              }
            }

            HACD::HACD * const myHACD = HACD::CreateHACD();
            myHACD->SetPoints( &points[0] );
            myHACD->SetNPoints( points.size() );
            myHACD->SetTriangles( &tris[0] );
            myHACD->SetNTriangles( tris.size() );

            myHACD->SetCompacityWeight( options.getCompacityWeight() );
            myHACD->SetVolumeWeight( options.getVolumeWeight() );
            myHACD->SetScaleFactor( options.getScaleFactor() );
            myHACD->SetNClusters( options.getNrClusters() );
            myHACD->SetNVerticesPerCH( options.getNrVerticesPerCH() );
            myHACD->SetConcavity( options.getConcavity() );
            myHACD->SetAddExtraDistPoints( options.getAddExtraDistPoints() );
            myHACD->SetNTargetTrianglesDecimatedMesh( options.getNrTargetTrianglesDecimatedMesh() );
            myHACD->SetAddFacesPoints( options.getAddFacesPoints() );
            myHACD->SetConnectDist( options.getConnectDist() );
            myHACD->SetSmallClusterThreshold( options.getSmallClusterThreshold() );

            myHACD->Compute();

            if( options.getSaveConvexDecomposition() != "" ) {
              string convex_decomposition_file = options.getSaveConvexDecomposition();
              bool saved_composition = myHACD->Save( convex_decomposition_file.c_str(), false );
              if( !saved_composition && options.getBaseURL() != "" ) {
                convex_decomposition_file = options.getBaseURL() + convex_decomposition_file;
                saved_composition = myHACD->Save( convex_decomposition_file.c_str(), false );
              }

              if( saved_composition )
                Console( LogLevel::Debug ) << "Debugging: Saved convex decomposition to " << convex_decomposition_file << endl;
              else
                Console( LogLevel::Debug ) << "Debugging: Error saving convex decomposition to " << options.getSaveConvexDecomposition() << endl;

              const HACD::Vec3<HACD::Real> * const decimatedPoints = myHACD->GetDecimatedPoints();
              const HACD::Vec3<long> * const decimatedTriangles = myHACD->GetDecimatedTriangles();
              if( decimatedPoints && decimatedTriangles ) {
                if( PhysX4_H3DInternal::SaveOFF( convex_decomposition_file + "-decimated.off",
                  myHACD->GetNDecimatedPoints(),
                  myHACD->GetNDecimatedTriangles(), decimatedPoints, decimatedTriangles ) ) {
                  Console( LogLevel::Debug ) << "Debugging: Saved decimated mesh to " << convex_decomposition_file << "-decimated.off" << endl;
                } else {
                  Console( LogLevel::Debug ) << "Debugging: Error saving decimated mesh to " << options.getSaveConvexDecomposition() << "-decimated.off" << endl;
                }
              }
            }

            for( size_t c = 0; c < myHACD->GetNClusters(); ++c ) {

              size_t nPoints = myHACD->GetNPointsCH( c );
              size_t nTriangles = myHACD->GetNTrianglesCH( c );
              HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
              HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
              myHACD->GetCH( c, pointsCH, trianglesCH );

              // Copy points
              PxVec3* verts = new PxVec3[nPoints];
              for( size_t i = 0; i < nPoints; ++i ) {
                verts[i] = PxVec3( (PxReal)pointsCH[i].X(), (PxReal)pointsCH[i].Y(), (PxReal)pointsCH[i].Z() );
              }
              delete[] pointsCH;
              delete[] trianglesCH;

              // Create mesh description
              PxConvexMeshDesc convexDesc;
              assert( points.size() <= std::numeric_limits<PxU32>::max() );
              convexDesc.points.count = static_cast<PxU32>(nPoints);
              convexDesc.points.stride = sizeof( PxVec3 );
              convexDesc.points.data = verts;
              convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

              // Cook the mesh
              stringstream ss;
              ss << options.getCookedFilename() << c;
              string filename = ss.str().c_str();

              if( load_cooked ) {
                bool success = false;
                {
                  // Allocate using new in order to
                  auto_ptr< PxDefaultFileOutputStream > buf( new PxDefaultFileOutputStream( filename.c_str() ) );
                  if( !buf->isValid() && options.getBaseURL() != "" ) {
                    filename = options.getBaseURL() + filename;
                    buf.reset( new PxDefaultFileOutputStream( filename.c_str() ) );
                  }
                  if( buf->isValid() )
                    success = physx_data->cooking->cookConvexMesh( convexDesc, *buf.get() );
                }
                if( !success ) {
                  Console( LogLevel::Error ) << "Error: PhysX4 error cooking convex mesh!" << endl;
                } else {
                  // Load the cooked mesh
                  auto_ptr< PxDefaultFileInputData > input( new PxDefaultFileInputData( filename.c_str() ) );
                  PxConvexMesh* convexMesh = nullptr;
                  if( input->isValid() ) {
                    convexMesh = physx_data->sdk->createConvexMesh( *input.get() );
                    if( convexMesh ) {
                      geometry.push_back( new PxConvexMeshGeometry( convexMesh ) );
                    }
                  }
                  if( !convexMesh ) {
                    Console( LogLevel::Error ) << "Error: PhysX4 error creating convex mesh!" << endl;
                  }
                }
              } else {
                PxDefaultMemoryOutputStream buf;
                if( !physx_data->cooking->cookConvexMesh( convexDesc, buf ) ) {
                  Console( LogLevel::Error ) << "Error: PhysX4 error cooking convex mesh!" << endl;
                } else {
                  // Load the cooked mesh
                  PxDefaultMemoryInputData input( buf.getData(), buf.getSize() );
                  PxConvexMesh* convexMesh = physx_data->sdk->createConvexMesh( input );
                  geometry.push_back( new PxConvexMeshGeometry( convexMesh ) );
                }
              }

              delete[] verts;
            }
            Console( LogLevel::Info ) << "Done." << endl;
            HACD::DestroyHACD( myHACD );
          }
        } else {
#endif
          if( options.getConvexDecomposition() ) {
            Console( LogLevel::Warning ) << "Warning: H3DPhysics was not compiled with convex decomposition capabilities (requires HACD)!" << endl;
          }

          string filename = options.getCookedFilename();
          bool load_cooked = filename != "";
          PxTriangleMesh* triMesh = NULL;
          if( load_cooked ) {
            auto_ptr< PxDefaultFileInputData > input( new PxDefaultFileInputData( filename.c_str() ) );
            if( !input->isValid() && options.getBaseURL() != "" ) {
              filename = options.getBaseURL() + filename;
              input.reset( new PxDefaultFileInputData( filename.c_str() ) );
            }
            if( input->isValid() ) {
              triMesh = physx_data->sdk->createTriangleMesh( *input.get() );
              if( triMesh ) {
                geometry.push_back( new PxTriangleMeshGeometry( triMesh ) );
              }
            }
          }

          if( !triMesh ) {
            PxVec3* verts = new PxVec3[triangles.size() * 3];
            PxU32* index = new PxU32[triangles.size() * 3];
            for( size_t i = 0; i < triangles.size(); ++i ) {
              assert( i * 3 + 2 <= std::numeric_limits<PxU32>::max() );
              verts[i * 3] = toPxVec3( Vec3f( triangles[i].a ) );
              index[i * 3] = static_cast<PxU32>(i * 3);
              verts[i * 3 + 1] = toPxVec3( Vec3f( triangles[i].b ) );
              index[i * 3 + 1] = static_cast<PxU32>(i * 3 + 1);
              verts[i * 3 + 2] = toPxVec3( Vec3f( triangles[i].c ) );
              index[i * 3 + 2] = static_cast<PxU32>(i * 3 + 2);
            }

            PxTriangleMeshDesc meshDesc;
            assert( triangles.size() * 3 <= std::numeric_limits<PxU32>::max() );
            meshDesc.points.count = static_cast<PxU32>(triangles.size() * 3);
            meshDesc.points.stride = sizeof( PxVec3 );
            meshDesc.points.data = verts;

            assert( triangles.size() <= std::numeric_limits<PxU32>::max() );
            meshDesc.triangles.count = static_cast<PxU32>(triangles.size());
            meshDesc.triangles.stride = 3 * sizeof( PxU32 );
            meshDesc.triangles.data = index;

            // Cook the mesh
            if( load_cooked ) {
              bool success = false;
              {
                PxDefaultFileOutputStream buf( filename.c_str() );
                if( buf.isValid() )
                  success = physx_data->cooking->cookTriangleMesh( meshDesc, buf );
              }
              if( !success ) {
                Console( LogLevel::Error ) << "Error: PhysX4 error cooking convex mesh!" << endl;
              } else {
                // Load the cooked mesh
                PxDefaultFileInputData input( filename.c_str() );
                if( input.isValid() ) {
                  triMesh = physx_data->sdk->createTriangleMesh( input );
                  if( triMesh )
                    geometry.push_back( new PxTriangleMeshGeometry( triMesh ) );
                }
              }
            } else {
              PxDefaultMemoryOutputStream buf;
              if( !physx_data->cooking->cookTriangleMesh( meshDesc, buf ) ) {
                Console( LogLevel::Error ) << "Error: PhysX4 error cooking convex mesh!" << endl;
              } else {
                // Load the cooked mesh
                PxDefaultMemoryInputData input( buf.getData(), buf.getSize() );
                triMesh = physx_data->sdk->createTriangleMesh( input );
                geometry.push_back( new PxTriangleMeshGeometry( triMesh ) );
              }
            }
            delete[] verts;
            delete[] index;
          }
#ifdef HAVE_HACD
        }
#endif
      }
    }
    else {
      Console( LogLevel::Error ) << "Error: PhysX4. error no triangles in the shape bound tree!" << endl;
    }
  }
  
  setParameters ( _params );
}

PhysX4CollidableShape::~PhysX4CollidableShape () {
  
  //Console( LogLevel::Warning ) << "Deleting cs: " << this << " with bid: " << body << endl;
  if ( body ) {
    Console( LogLevel::Warning ) << "WARNING: A collidable shape is being deleted before it is being removed from its rigid body." << endl;
  }
  removeFromBody( true );
  for( vector<PxGeometry*>::iterator i = geometry.begin(); i != geometry.end(); ++i )
    delete (*i);
  geometry.clear();
  for( vector<PxShape*>::iterator i = shape.begin(); i != shape.end(); ++i )
    (*i)->release();
  shape.clear();

  std::vector< void * >::iterator k = std::find( PhysX4_H3DInternal::collidables_with_contact_mode_all.begin(), PhysX4_H3DInternal::collidables_with_contact_mode_all.end(), this );
  if( k != PhysX4_H3DInternal::collidables_with_contact_mode_all.end() ) {
    PhysX4_H3DInternal::collidables_with_contact_mode_all.erase( k );
  }
}

void PhysX4CollidableShape::setParameters ( PhysicsEngineParameters::ShapeParameters& _params ) {
  PhysX4Callbacks::PhysX4SpecificData *physx_data = 
    static_cast< PhysX4Callbacks::PhysX4SpecificData * >(_params.getEngine()->getEngineSpecificData());

  if ( _params.haveTranslation() ) {
    translation= _params.getTranslation();
  }

  if ( _params.haveRotation() ) {
    rotation= _params.getRotation();
  }

  if ( _params.haveTranslation() || _params.haveRotation() ) {
    for ( vector<PxShape*>::iterator i= shape.begin(); i != shape.end(); ++i ) {
      (*i)->setLocalPose ( PxTransform ( toPxVec3 ( translation ), toPxQuat ( rotation*rotationOffset ) ) );
    }
  }

  if ( _params.haveEnabled() ) {
    collisionEnabled = _params.getEnabled();
  }

  if ( _params.haveCollidableExceptionGroupList() ) {
    exceptionGroupMask = 0;
    CollidableParameters::CollidableExceptionGroupList gList = _params.getCollidableExceptionGroupList();
    for( unsigned int i=0; i<gList.size(); i++ ){
      unsigned int groupid = 1<<gList[i];
      exceptionGroupMask |= groupid;
    }
  }

  if ( _params.haveCollidableSelectionGroupList() ) {
    selectionGroupMask = 0;
    CollidableParameters::CollidableSelectionGroupList gList = _params.getCollidableSelectionGroupList();
    for( unsigned int i=0; i<gList.size(); i++ ){
      unsigned int g_id = (unsigned int)(gList[i]);
      unsigned int groupid = 1<<g_id;
      // Distinguish between collidables and selectedCollidables:
      float frac = gList[i] - g_id;
      if( abs(frac - 0.4f) < H3DUtil::Constants::f_epsilon ) {
        groupid <<= 16;
      }

      selectionGroupMask |= groupid;
    }
  }

  if( _params.haveClipPlanes() ) {
    clipPlanes_mutex.lock();
    clipPlanes = _params.getClipPlanes();
    clipPlanes_mutex.unlock();
  }

  bool filterDataNeedsUpdate = false;
  if( _params.haveEnabled() || _params.haveCollidableExceptionGroupList() || _params.haveClipPlanes() || _params.haveCollidableSelectionGroupList() ){
    filterDataNeedsUpdate = true;
  }

  // PhysX4 specific collidable options
  if ( _params.haveEngineOptions() ) {
    if ( PhysXCollidableParameters* options= 
      dynamic_cast<PhysXCollidableParameters*>(_params.getEngineOptions()) ) {

      // restOffset
      if ( options->haveRestOffset() ) {
        restOffset = options->getRestOffset();
        for ( vector<PxShape*>::iterator i= shape.begin(); i != shape.end(); ++i ) {
          (*i)->setRestOffset( (PxReal)restOffset );
        }
      }

      // contactOffset
      if ( options->haveContactOffset() ) {
        contactOffset = options->getContactOffset();
        for ( vector<PxShape*>::iterator i= shape.begin(); i != shape.end(); ++i ) {
          (*i)->setContactOffset( (PxReal)contactOffset );
        }
        
      }

      // suppressDisabledContacts
      if ( options->haveSuppressDisabledContacts() ) {
        suppressDisabledContacts = options->getSuppressDisabledContacts();
      }

      // setFlagsForAll
      if ( options->haveSetFlagsForAll() ) {
        setFlagsForAll = options->getSetFlagsForAll();
      }
      
      // contactFilterFlag
      if ( options->haveContactPairFlags() ) {
        vector< void * >::iterator found_iterator = find( PhysX4_H3DInternal::collidables_with_contact_mode_all.begin(), PhysX4_H3DInternal::collidables_with_contact_mode_all.end(), this );
        if( found_iterator != PhysX4_H3DInternal::collidables_with_contact_mode_all.end() ) {
          PhysX4_H3DInternal::collidables_with_contact_mode_all.erase(found_iterator);
        }
        std::vector<std::string> &flags = options->getContactPairFlags();
        contactFilterOptionsMask = 0;
        
        for( size_t i=0; i<flags.size(); i++ ){
          

          if( flags[i] == "eSOLVE_CONTACT" ){
            contactFilterOptionsMask |= PxPairFlag::eSOLVE_CONTACT;
          }
          else if( flags[i] == "eMODIFY_CONTACTS" ){
            contactFilterOptionsMask |= PxPairFlag::eMODIFY_CONTACTS;
          }
          else if( flags[i] == "eNOTIFY_TOUCH_FOUND" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
          }
          else if( flags[i] == "eNOTIFY_TOUCH_PERSISTS" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
          }
          else if( flags[i] == "eNOTIFY_TOUCH_LOST" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_TOUCH_LOST;
          }
          else if( flags[i] == "eNOTIFY_TOUCH_CCD" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_TOUCH_CCD;
          }
          else if( flags[i] == "eNOTIFY_THRESHOLD_FORCE_FOUND" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND;
          }
          else if( flags[i] == "eNOTIFY_THRESHOLD_FORCE_PERSISTS" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS;
          }
          else if( flags[i] == "eNOTIFY_THRESHOLD_FORCE_LOST" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST;
          }
          else if( flags[i] == "eNOTIFY_CONTACT_POINTS" ){
            contactFilterOptionsMask |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
          }
          else if( flags[i] == "eDETECT_DISCRETE_CONTACT" ){
            contactFilterOptionsMask |= PxPairFlag::eDETECT_DISCRETE_CONTACT;
          }
          else if( flags[i] == "eDETECT_CCD_CONTACT" ){
            contactFilterOptionsMask |= PxPairFlag::eDETECT_CCD_CONTACT;
          }
          else if( flags[i] == "ePRE_SOLVER_VELOCITY" ){
            contactFilterOptionsMask |= PxPairFlag::ePRE_SOLVER_VELOCITY;
          }
          else if( flags[i] == "ePOST_SOLVER_VELOCITY" ){
            contactFilterOptionsMask |= PxPairFlag::ePOST_SOLVER_VELOCITY;
          }
          else if( flags[i] == "eCONTACT_EVENT_POSE" ){
            contactFilterOptionsMask |= PxPairFlag::eCONTACT_EVENT_POSE;
          }
          else if( flags[i] == "eCONTACT_DEFAULT" ){
            contactFilterOptionsMask |= PxPairFlag::eCONTACT_DEFAULT;
          }
          else if( flags[i] == "eTRIGGER_DEFAULT" ){
            contactFilterOptionsMask |= PxPairFlag::eTRIGGER_DEFAULT;
          } else if( flags[i] == "CONTACT_MODE_ALL" ) {
            if( find( PhysX4_H3DInternal::collidables_with_contact_mode_all.begin(), PhysX4_H3DInternal::collidables_with_contact_mode_all.end(), this ) == PhysX4_H3DInternal::collidables_with_contact_mode_all.end() ) {
              PhysX4_H3DInternal::collidables_with_contact_mode_all.push_back( this );
            }
          }

        }

      }

      if( options->haveSuppressDisabledContacts() || options->haveSetFlagsForAll() || options->haveContactPairFlags()  ){
        filterDataNeedsUpdate = true;
      }

    }
  }

  if( filterDataNeedsUpdate ){
    updateFilterData(physx_data);
  }

}

void PhysX4CollidableShape::getParameters ( PhysicsEngineParameters::ShapeParameters& _params ) {
}


void PhysX4CollidableShape::addToBody ( PhysX4RigidBody& _body, PhysX4Callbacks::PhysX4SpecificData& physx_data ) {
  if ( !body ) {
    body= &_body;
    addToBodyInternal ( _body, physx_data );
  }
}

void PhysX4CollidableShape::removeFromBody( bool removeFromBodyShapes ) {
  if ( body ) {
    removeFromBodyInternal();
    if( removeFromBodyShapes ) {
      body->removeCollidableShape( this );
    }
    body= NULL;
  }
}

void PhysX4CollidableShape::addToBodyInternal ( PhysX4RigidBody& _body, PhysX4Callbacks::PhysX4SpecificData& physx_data ) {
  if( body->getActor() ) {
    for ( vector<PxGeometry*>::iterator i= geometry.begin(); i != geometry.end(); ++i ) {
      PxShape* s = physx::PxRigidActorExt::createExclusiveShape( *body->getActor(), **i, *physx_data.default_material );
      if( s ) {
        s->setLocalPose( PxTransform( PxTransform( toPxVec3( translation ), toPxQuat( rotation * rotationOffset ) ) ) );
        s->userData= this;
        s->setRestOffset( (PxReal)restOffset );
        s->setContactOffset( (PxReal)contactOffset );
        shape.push_back ( s );
      }
      //Console( LogLevel::Warning )<< "Reset: "  << s->getRestOffset() << "  contact: " << s->getContactOffset() << endl; 
    }
    updateFilterData(&physx_data);
  }
}

void PhysX4CollidableShape::removeFromBodyInternal () {

  if( body && body->getActor() ) {
    for( vector<PxShape*>::iterator i = shape.begin(); i != shape.end(); ++i ) {
      body->getActor()->detachShape( **i );
    }
  }
  shape.clear();
}

void PhysX4CollidableShape::updateFilterData( PhysX4Callbacks::PhysX4SpecificData * physx_data ){

  PxFilterData d;
  // Using word0 for contact filtering options:
  d.word0 = contactFilterOptionsMask;
  // Using word2 for different options:
  // 1st bit: enabled
  // 2nd bit: Clipped or not.
  // 3rd bit: Supress disabled contacts or not.
  // 4th bit: Set flags even for disabled collidables
  d.word2 = 0;
  if( collisionEnabled )
    d.word2 |= (1);
  if( clipPlanes.size() > 0 )
    d.word2 |= (1<<1);
  if( suppressDisabledContacts )
    d.word2 |= (1<<2);
  if( setFlagsForAll )
    d.word2 |= (1<<3);
  

  // Mask for exception groups which should be ignored.
  d.word3 = exceptionGroupMask;
  // Mask for selection groups
  d.word1 = selectionGroupMask;

  
  for ( vector<PxShape*>::iterator i= shape.begin(); i != shape.end(); ++i ) {
    (*i)->setSimulationFilterData ( d );
#if PX_PHYSICS_VERSION_MAJOR < 4 && PX_PHYSICS_VERSION_MINOR < 3
    (*i)->resetFiltering();
#endif
  }
  //Console( LogLevel::Warning ) << "In UpdateFilterData: "<< d.word0 <<"  "<< d.word3 <<endl;
  if( body && body->getActor() && !dynamic_cast<PhysX4ArticulationLink*>(body) ) {
    // reset filtering with only the shapes
    if( shape.size() > 0 ) {
      assert( shape.size() <= std::numeric_limits<PxU32>::max() );
      physx_data->scene->resetFiltering( *body->getActor(), &shape.front(), static_cast<PxU32>(shape.size()) );
    }
  }
}

void PhysX4Callbacks::SimulationEventCallback::onContact (const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) {
  bool get_all = ( contact_reporting_mode == "ALL" );
  for(PxU32 i=0; i < nbPairs; ++i) {
    const PxContactPair& cp = pairs[i];

    if( cp.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS ) {
      PxContactPairPoint *userBuffer= new PxContactPairPoint[cp.contactCount];
      cp.extractContacts (userBuffer, cp.contactCount);
      bool local_get_all = get_all;
      if( find( PhysX4_H3DInternal::collidables_with_contact_mode_all.begin(), PhysX4_H3DInternal::collidables_with_contact_mode_all.end(), cp.shapes[0]->userData ) != PhysX4_H3DInternal::collidables_with_contact_mode_all.end() ||
          find( PhysX4_H3DInternal::collidables_with_contact_mode_all.begin(), PhysX4_H3DInternal::collidables_with_contact_mode_all.end(), cp.shapes[1]->userData ) != PhysX4_H3DInternal::collidables_with_contact_mode_all.end() ) {
        local_get_all = true;
      }
      for ( size_t c= 0; c < cp.contactCount; ++c ) {
        PxContactPairPoint ct= userBuffer[c];
        if ( local_get_all || ct.separation < 0 ) {
          PhysicsEngineParameters::ContactParameters contact;

          contact.body1_id= (H3DBodyId)pairHeader.actors[0]->userData;
          contact.body2_id= (H3DBodyId)pairHeader.actors[1]->userData;

          contact.geom1_id= (H3DCollidableId)cp.shapes[0]->userData;
          contact.geom2_id= (H3DCollidableId)cp.shapes[1]->userData;

          contact.contact_normal= toVec3f(ct.normal);
          contact.position= toVec3f(ct.position);
          contact.depth= -ct.separation;

          contacts.push_back ( contact );
        }
      }

      delete [] userBuffer;
    }
  }
}

void PhysX4Callbacks::SimulationEventCallback::clearContacts () {
  contacts.clear();
}

void PhysX4Callbacks::SimulationEventCallback::getContacts ( list<PhysicsEngineParameters::ContactParameters>& _contacts ) {
  _contacts.insert ( _contacts.begin(), contacts.begin(), contacts.end() );
}

void PhysX4Callbacks::ContactModifyCallback::onContactModify(PxContactModifyPair* const pairs, PxU32 count){
  for(PxU32 i=0; i<count; i++){
    PxContactModifyPair& cmp = pairs[i];

    // Add all the clip planes from both shapes
    std::vector<Vec4d> planes;

    PhysX4CollidableShape* shape= (PhysX4CollidableShape*)cmp.shape[0]->userData;
    shape->clipPlanes_mutex.lock();
    planes.insert( planes.end(), shape->clipPlanes.begin(), shape->clipPlanes.end());
    shape->clipPlanes_mutex.unlock();

    shape= (PhysX4CollidableShape*)cmp.shape[1]->userData;
    shape->clipPlanes_mutex.lock();
    planes.insert( planes.end(), shape->clipPlanes.begin(), shape->clipPlanes.end());
    shape->clipPlanes_mutex.unlock();

    for(PxU32 n=0; n<cmp.contacts.size(); n++){

      for( unsigned int p=0; p<planes.size(); p++ ){
        if( cmp.contacts.getPoint( n ).dot( toPxVec3( Vec3f( static_cast<H3DFloat>(planes[p].x), static_cast<H3DFloat>(planes[p].y), static_cast<H3DFloat>(planes[p].z) ) ) ) + static_cast<H3DFloat>(planes[p].w) < 0.0 ) {
          cmp.contacts.ignore(n);
          break;
        }
      }
    }
  }
}

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

// A 'shader' that is executed by the collision detection system to
// determine how to handle contacts

PxFilterFlags PhysX4FilterShader (
  PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
  PxFilterObjectAttributes attributes1, PxFilterData filterData1,
  PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize) {
    // let triggers through
    if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
      pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
      return PxFilterFlag::eDEFAULT;
    }

    // If they are in the same exception group return
 if( filterData0.word3 & filterData1.word3 )
    return PxFilterFlag::eKILL;
 
    // If one object is in collidables of a selection group
    PxU32 data0_sg = filterData0.word1 & 0xFFFF;
    PxU32 data1_sg = filterData1.word1 & 0xFFFF;
    if( data0_sg ) {
      // Allow collision only if the other object is in the same field or
      // in the same selectionGroups selectedCollidables field.
      if( ! ( ( data0_sg & filterData1.word1 ) || ( data0_sg & (filterData1.word1 >> 16) ) ) )
        return PxFilterFlag::eKILL;
    }
    else if( data1_sg ) {
      // Allow collision only if the other object is in the same field or
      // in the same selectionGroups selectedCollidables field.
      if( ! ( ( data1_sg & filterData0.word1 & 0xFF ) || ( data1_sg & (filterData0.word1 >> 16) ) ) )
        return PxFilterFlag::eKILL;
    }
    
    // generate contacts for all that were not filtered above
    // Reset pairFlags
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;
    //pairFlags &= ~pairFlags;
    
  // Are the collision shapes both enabled
  if( ( 1 & filterData0.word2 ) && ( 1 & filterData1.word2 ) ) {
    
    // Set flags: For the ones which are set in both collidables in the contact
    PxPairFlag::Enum combined_flags = (PxPairFlag::Enum) ( filterData0.word0 & filterData1.word0 ); 
    pairFlags = combined_flags;

  } else {
    // At least one of the collidables are disabled
    
    // Set the flags only if any of them have the option setForAll set to True
    if( ( ( ( 1<<3 ) & filterData0.word2 ) || ( ( 1<<3 ) & filterData1.word2 ) ) ) {
      // Set flags: For the ones which are set in both collidables in the contact
      PxPairFlag::Enum combined_flags = (PxPairFlag::Enum) ( filterData0.word0 & filterData1.word0 ); 
      pairFlags = combined_flags;
    }
    
    // Supress only if both of them have the option supressDisabled contacts
    if( ( ( ( 1 << 2 ) & filterData0.word2 ) && ( ( 1 << 2 ) & filterData1.word2 ) ) ) {
      return PxFilterFlag::eSUPPRESS;
    }
  }

  // Clip plane
  if( ( ( 1<<1 ) & filterData0.word2 ) || ( ( 1<<1 ) & filterData1.word2 ) ){
    pairFlags |= PxPairFlag::eMODIFY_CONTACTS;
  }

  return PxFilterFlag::eDEFAULT;
}

PeriodicThread::CallbackCode PhysX4Callbacks::initEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );
  PhysX4Callbacks::PhysX4SpecificData *physx_data = 
    new PhysX4Callbacks::PhysX4SpecificData;
  pt->setEngineSpecificData( physx_data );

  // Initialise the SDK
  physx_data->foundation= PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
  if ( !physx_data->foundation ) { 
    Console( LogLevel::Warning ) << "Error: Unable to initialize the PhysX4 foundation" << endl; 
    return PeriodicThread::CALLBACK_DONE;
  }

  physx_data->sdk = PxCreatePhysics(PX_PHYSICS_VERSION, *physx_data->foundation, PxTolerancesScale());
  if ( !physx_data->sdk ) { 
    Console( LogLevel::Warning ) << "Error: Unable to initialize the PhysX4 SDK" << endl; 
    return PeriodicThread::CALLBACK_DONE;
  }

  // Initialise cooking
  physx_data->cooking = PxCreateCooking(PX_PHYSICS_VERSION, *physx_data->foundation, PxCookingParams( PxTolerancesScale() ) );
  if ( !physx_data->cooking ) {
    Console( LogLevel::Warning ) << "Error: Unable to initialize the Cooking PhysX4 SDK" << endl; 
    return PeriodicThread::CALLBACK_DONE;
  }

  if (!PxInitExtensions(*physx_data->sdk, physx_data->pvd)) {
    Console( LogLevel::Warning ) << "Error: Unable to initialize PhysX4 extensions" << endl; 
    return PeriodicThread::CALLBACK_DONE;
  }

  // Create scene
  PxSceneDesc sceneDesc (physx_data->sdk->getTolerancesScale());
  if(!sceneDesc.cpuDispatcher) {
    physx_data->cpu_dispatcher= PxDefaultCpuDispatcherCreate(1);
    sceneDesc.cpuDispatcher= physx_data->cpu_dispatcher;
    if(!sceneDesc.cpuDispatcher) {
      Console( LogLevel::Warning ) << "PxDefaultCpuDispatcherCreate failed!" <<endl;
      return PeriodicThread::CALLBACK_DONE;
    }
  } 
  if(!sceneDesc.filterShader)
    sceneDesc.filterShader= gDefaultFilterShader;

  // Set solver type
  const H3DUInt32 solver = pt->getSolverType();
  if ( solver <= 1 ) {
    sceneDesc.solverType = static_cast<PxSolverType::Enum>( solver );
  }
  else {
    Console(4) << "Invalid solver value: " << solver << ". Setting default solver." << endl;
    // No need to explicitly set default solver, implicitly set.
  }


  physx_data->simulation_events= new SimulationEventCallback;
  sceneDesc.simulationEventCallback= physx_data->simulation_events;

  physx_data->contact_modifier= new ContactModifyCallback;
  sceneDesc.contactModifyCallback = physx_data->contact_modifier;

  sceneDesc.filterShader= PhysX4FilterShader;

  physx_data->scene = physx_data->sdk->createScene( sceneDesc );

  GlobalContactParameters p= pt->getGlobalContactParameters();
  physx_data->default_material= physx_data->sdk->createMaterial ( 
    p.friction_coefficients.x, p.friction_coefficients.y, p.bounce );

  physx_data->simulation_events->setContactReportingMode( p.contact_report_mode );
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::deInitEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread *>( data );
  PhysX4SpecificData *physx_data = static_cast< PhysX4SpecificData * >( pt->getEngineSpecificData() );
  if ( physx_data->sdk != NULL ){
    physx_data->default_material->release();
    physx_data->default_material = NULL;

    physx_data->scene->release();
    physx_data->scene = NULL;

    delete physx_data->simulation_events;
    physx_data->simulation_events= NULL;

    delete physx_data->contact_modifier;
    physx_data->contact_modifier= NULL;

    physx_data->cpu_dispatcher->release();
    physx_data->cpu_dispatcher= NULL;

    PxCloseExtensions();

    physx_data->cooking->release();
    physx_data->cooking = NULL;

    physx_data->sdk->release();
    physx_data->sdk = NULL;

    physx_data->foundation->release();
    physx_data->foundation = NULL;
  }
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::doSimulationSteps(void *data) {
  PhysicsEngineThread * physics_thread = 
    static_cast< PhysicsEngineThread * >( data );
  PhysX4SpecificData *physx_data = 
    static_cast< PhysX4SpecificData * >(physics_thread->getEngineSpecificData());
  if( !physx_data->sdk )
    return PeriodicThread::CALLBACK_CONTINUE;

  TimeStamp t;

  // Default rigid body material properties
  GlobalContactParameters p= physics_thread->getGlobalContactParameters();
  physx_data->default_material->setStaticFriction ( p.friction_coefficients.x );
  physx_data->default_material->setDynamicFriction ( p.friction_coefficients.y );
  physx_data->default_material->setRestitution ( p.bounce );
  physx_data->simulation_events->setContactReportingMode( p.contact_report_mode );

  physx_data->simulation_events->clearContacts ();

  H3DTime max_dt= physics_thread->getStepSize();

  PhysicsEngineParameters::WorldParameters world_params = physics_thread->getWorldParameters();

  // Step the simulation

  if( world_params.getUseStaticTimeStep() ) {

    physx_data->last_update_time = TimeStamp();

    physx_data->scene->simulate ( PxReal( max_dt ) );
    physx_data->scene->fetchResults ( true );

  } else {

    H3DTime dt= TimeStamp()-physx_data->last_update_time;
    physx_data->last_update_time= TimeStamp();

    dt= std::min ( dt, 2*max_dt );
    while ( dt >= max_dt ) {
      physx_data->scene->simulate ( PxReal(max_dt) );
      physx_data->scene->fetchResults ( true );
      dt-= max_dt;
    }
    physx_data->last_update_time -= dt;

  }

  physics_thread->setLastLoopTime( TimeStamp() - t );
  
  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::setWorldParameters( void *data ) {
  PhysicsEngineParameters::WorldParameters *params = 
    static_cast< PhysicsEngineParameters::WorldParameters *>( data );
  PhysX4SpecificData *physx_data = static_cast< PhysX4SpecificData * >( 
    params->getEngine()->getEngineSpecificData() );
  if( !physx_data->sdk )
    return PeriodicThread::CALLBACK_DONE;

  // gravity
  if( params->haveGravity() ) {
    physx_data->scene->setGravity( toPxVec3 ( params->getGravity() ) ); 
  }

  // disabling bodies
  if( params->haveAutoDisable() || 
      params->haveDisableLinearSpeed() || 
      params->haveDisableAngularSpeed() ){
    list< H3DBodyId > bodies;
    params->getEngine()->getCurrentRigidBodies( bodies );
    for( list< H3DBodyId >::iterator i = bodies.begin(); 
         i != bodies.end(); ++i ) {
      PhysicsEngineParameters::RigidBodyParameters p;
      params->getEngine()->getRigidBodyParameters( *i, p );
      ((PhysX4RigidBody*)(*i))->setSleeping ( p );
    }
  }

  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::synchroniseWithSceneGraph( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::addRigidBody( void *data ) {
  RigidBodyParameters *params = 
    static_cast< RigidBodyParameters *>( data );
  PhysX4SpecificData *physx_data = 
    static_cast< PhysX4SpecificData * >(params->getEngine()->getEngineSpecificData());

  // Handle Articulation.
  if( ArticulatedRigidBodyParameters* a_params = dynamic_cast< ArticulatedRigidBodyParameters* >(params) ){
    PhysX4ArticulatedRigidBody* body_t = new PhysX4ArticulatedRigidBody ( *a_params );
    body_t->createRigidBody( physx_data );
    body_t->setParameters( *params );
    
    if( !body_t->getArticulation() || !physx_data || !physx_data->scene )
      Console( LogLevel::Warning )<< "ERROR: while adding articulation " << endl;
    physx_data->scene->addArticulation( *body_t->getArticulation() );
    params->setBodyId ( (H3DBodyId)body_t );

  } else {
    PhysX4RigidBody* body= new PhysX4RigidBody ( *params );
    PhysXRigidBodyParameters* engine_options = static_cast<PhysXRigidBodyParameters*>(params->getEngineOptions());
    if( engine_options && engine_options->haveCreateAsStatic() && engine_options->getCreateAsStatic()) {
      body->createRigidBodyStatic( physx_data );
    }
    else {
      body->createRigidBody( physx_data );
    }

    physx_data->scene->addActor( *body->getActor() );
    body->setParameters( *params );
    params->setBodyId ( (H3DBodyId)body );
  }

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::removeRigidBody( void *data ) {
  RigidBodyParameters *params = static_cast< RigidBodyParameters * >( data );
  PhysX4Callbacks::PhysX4SpecificData *physx_data = 
    static_cast< PhysX4Callbacks::PhysX4SpecificData *>( 
      params->getEngine()->getEngineSpecificData() );

  // Handle Articulation.
  if( ArticulatedRigidBodyParameters* a_params = dynamic_cast< ArticulatedRigidBodyParameters* >(params) ){
    PhysX4ArticulatedRigidBody* body= (PhysX4ArticulatedRigidBody*)params->getBodyId();
    physx_data->scene->removeArticulation ( *body->getArticulation() );
    delete body;
  } else {
    PhysX4RigidBody* body= (PhysX4RigidBody*)params->getBodyId();
    physx_data->scene->removeActor ( *body->getActor() );
    delete body;
  }

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::setRigidBodyParameters( void *data ) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  PhysX4SpecificData *physx_data = 
    static_cast< PhysX4SpecificData * >(params->getEngine()->getEngineSpecificData());
  
  PhysX4RigidBody* body= (PhysX4RigidBody*)params->getBodyId();
  std::vector< PhysX4RigidBody* >::iterator j = std::find( PhysX4RigidBody::all_physX4_bodies.begin(), PhysX4RigidBody::all_physX4_bodies.end(), body );
  if( j != PhysX4RigidBody::all_physX4_bodies.end() )
    body->setParameters ( *params );
  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::getRigidBodyParameters( void *data ) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  PhysX4RigidBody* body= (PhysX4RigidBody*)params->getBodyId();
  body->getParameters ( *params );
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::addCollidable( void *data ) {
  PhysicsEngineParameters::CollidableParameters *data_params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );
  PhysicsEngineParameters::ShapeParameters *params =
    dynamic_cast< PhysicsEngineParameters::ShapeParameters *>( data_params );
  if( !params ) {
    Console( LogLevel::Warning ) << "Warning: Only CollidableShapes are supported by PhysX4!" << endl;
    return PeriodicThread::CALLBACK_DONE;
  }
  PhysX4SpecificData *physx_data = 
    static_cast< PhysX4SpecificData * >(params->getEngine()->getEngineSpecificData());

  PhysX4CollidableShape* shape= new PhysX4CollidableShape ( *params );
  params->setCollidableId ( (H3DCollidableId)shape );

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::removeCollidable( void *data ) {
  ShapeParameters *params = static_cast< ShapeParameters * >( data );
  PhysX4Callbacks::PhysX4SpecificData *physx_data = 
    static_cast< PhysX4SpecificData * >(params->getEngine()->getEngineSpecificData());
  
  PhysX4CollidableShape* shape= (PhysX4CollidableShape*)params->getCollidableId();
  delete shape;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::setCollidableParameters( void *data ) {
  PhysicsEngineParameters::ShapeParameters *params = 
    static_cast< PhysicsEngineParameters::ShapeParameters *>( data );

  PhysX4CollidableShape* shape= (PhysX4CollidableShape*)params->getCollidableId();
  shape->setParameters ( *params );
  
  params->getEngine()->addNodeToDeleteInSynch( params->getShape() );
  if( !params->haveShape() && params->getShape() ) {
    Console( LogLevel::Warning ) << "WARNING:POTENTIAL BUG in PhysX4Callbacks::setCollidableParameters" << endl;
  }

  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::getCollidableParameters( void *data ) {
  PhysicsEngineParameters::ShapeParameters *params = 
    static_cast< PhysicsEngineParameters::ShapeParameters *>( data );

  PhysX4CollidableShape* shape= (PhysX4CollidableShape*)params->getCollidableId();
  shape->getParameters ( *params );
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::addGlobalExternalForceAndTorque( void *data ) {
  PhysicsEngineParameters::ExternalForceTorqueParameters *params = 
    static_cast< PhysicsEngineParameters::ExternalForceTorqueParameters *>( data );
  PhysX4SpecificData *physx_data = 
    static_cast< PhysX4SpecificData * >(params->engine_thread->getEngineSpecificData());

  PhysX4RigidBody* body= (PhysX4RigidBody*)params->body_id;
  if( body && body->getActor() && body->getActor()->is<PxRigidBody>() ) {
      PxRigidBody* actor = NULL;
      if( PhysX4ArticulatedRigidBody * articulated_body = dynamic_cast< PhysX4ArticulatedRigidBody * >(body) ) {
        if( params->actor_index < articulated_body->getActors().size() ) {
          actor = articulated_body->getActors()[params->actor_index];
        }
      } else {
        actor = (PxRigidBody*)body->getActor();
      }
      // only add forces and torques on non-kinematic objects

      bool non_kinematic = !(actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC);

      if( non_kinematic ) {
        H3DFloat _epsilon = Constants::f_epsilon;
        if( H3DAbs( params->force.x ) > _epsilon ||
          H3DAbs( params->force.y ) > _epsilon ||
          H3DAbs( params->force.z ) > _epsilon ) {
          actor->addForce( toPxVec3( params->force ), physx::PxForceMode::eFORCE, false );
        }
        if( H3DAbs( params->torque.x ) > _epsilon ||
          H3DAbs( params->torque.y ) > _epsilon ||
          H3DAbs( params->torque.z ) > _epsilon ) {
          actor->addTorque( toPxVec3( params->torque ), physx::PxForceMode::eFORCE, false );
        }
      }
    }
   return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::addConstraint( void *data ) {
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );
  PhysX4SpecificData *physx_data = 
    static_cast< PhysX4SpecificData * >(params->getEngine()->getEngineSpecificData());
  
  PhysX4Joint* joint= PhysX4Joint::createJoint ( *params );

  if ( !joint ) {
    // Constraint creation failed
    Console( LogLevel::Warning ) << "Warning: Constraint type " << params->getType() << " not supported by PhysX4 in H3DPhysics!" << endl;
  }

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::removeConstraint( void *data ) {
  ConstraintParameters *params = static_cast< ConstraintParameters *>( data );
  PhysX4Joint* joint= (PhysX4Joint*)params->getConstraintId();
  delete joint;
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::setConstraintParameters( void *data ) {
  ConstraintParameters *params = static_cast< ConstraintParameters *>( data );
  PhysX4Joint* joint= (PhysX4Joint*)params->getConstraintId();
  joint->setParameters ( *params );
  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::getConstraintParameters( void *data ) {
  ConstraintParameters *params = static_cast< ConstraintParameters *>( data );
  PhysX4Joint* joint= (PhysX4Joint*)params->getConstraintId();
  joint->getParameters ( *params );
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::getCurrentContacts( void *data ) {
  typedef pair< list< PhysicsEngineParameters::ContactParameters  >*,
                PhysicsEngineThread * > InputType;
  InputType *params = 
    static_cast< InputType *>( data );
  PhysX4SpecificData *physX_data = 
    static_cast< PhysX4SpecificData * >(params->second->getEngineSpecificData());
  physX_data->simulation_events->getContacts ( *params->first );
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::addSpace( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::removeSpace( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::setSpaceParameters( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysX4Callbacks::getSpaceParameters( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}



// soft body callback functions
//

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::addSoftBody ( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::removeSoftBody ( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::setSoftBodyParameters ( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::getSoftBodyParameters ( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysX4Callbacks::applyExternalForces ( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

#endif // HAVE_PHYSX4


