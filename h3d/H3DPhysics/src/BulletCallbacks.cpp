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
/// \file BulletCallbacks.cpp
/// \brief Source file for BulletCallbacks, struct containing callbacks and
/// variables for connecting to the bullet physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/BulletCallbacks.h>
#include <H3D/H3DPhysics/BulletJoints.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/H3DPhysics/BulletWorldOptions.h>
#include <H3D/H3DPhysics/BulletCollidableOptions.h>
#include <H3D/H3DPhysics/BulletSoftBodyOptions.h>
#include <H3D/H3DPhysics/BulletAttachmentOptions.h>
#include <H3D/H3DPhysics/BulletRigidBodyOptions.h>
#include <H3D/H3DPhysics/IndexedTetraSet.h>
#include <H3D/H3DPhysics/IndexedHexaSet.h>
#include <H3D/Box.h>
#include <H3D/Sphere.h>
#include <H3D/Cone.h>
#include <H3D/Cylinder.h>
#include <H3D/Capsule.h>

using namespace H3D;

#ifdef HAVE_BULLET

#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

SoftBodyPhysicsEngineThread::SoftBodyPhysicsEngineRegistration 
  BulletCallbacks::registration( "Bullet", 
  SoftBodyPhysicsEngineThread::createSoftBodyPhysicsEngineCallbacks< BulletCallbacks >() );

// Construct a transform to a coordinate system with the specified axis as the local x-axis
btTransform H3D::getAxisTransformX ( const btVector3& xAxis )
{
  btVector3 other ( 0, 1, 0 );
  if ( xAxis == other || -xAxis == other )
    other= btVector3 ( 0, 0, 1 );

  btVector3 yAxis= xAxis.cross ( other );
  yAxis.normalize();

  btVector3 zAxis= xAxis.cross ( yAxis );
  zAxis.normalize();

  return btTransform ( btMatrix3x3 ( xAxis.x(), yAxis.x(), zAxis.x(),
    xAxis.y(), yAxis.y(), zAxis.y(),
    xAxis.z(), yAxis.z(), zAxis.z() ) );
}

// Construct a transform to a coordinate system with the specified axis as the local z-axis
btTransform H3D::getAxisTransformZ ( const btVector3& zAxis )
{
  btVector3 other ( 0, 1, 0 );
  if ( zAxis == other || -zAxis == other )
    other= btVector3 ( 0, 0, 1 );

  btVector3 xAxis= zAxis.cross ( other );
  xAxis.normalize();

  btVector3 yAxis= zAxis.cross ( xAxis );
  yAxis.normalize();

  return btTransform ( btMatrix3x3 ( xAxis.x(), yAxis.x(), zAxis.x(),
    xAxis.y(), yAxis.y(), zAxis.y(),
    xAxis.z(), yAxis.z(), zAxis.z() ) );
}

Matrix4f H3D::toMatrix4f ( const btTransform& t )
{
  H3DFloat m[16];
  t.getOpenGLMatrix ( m );
  return Matrix4f (  m[0], m[4], m[8], m[12], 
    m[1], m[5], m[9], m[13],
    m[2], m[6], m[10], m[14],
    m[3], m[7], m[11], m[15] );
}

// Create a 3x3 bullet matrix from a 3x3 H3D matrix
btMatrix3x3 H3D::tobtMatrix3x3 ( const Matrix3f& m )
{
  return btMatrix3x3( m[0][0], m[1][0], m[2][0], 
    m[0][1], m[1][1], m[2][1], 
    m[0][2], m[1][2], m[2][2] );
}

// Handles link between H3D and bullet representations of a rigid body
// This makes management and clean-up of additional data easier and avoids the need for a map
BulletRigidBody::BulletRigidBody ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters )
  :
physicsThread ( bodyParameters.getEngine() ),
  massDensityModel ( NULL ),
  collisionGroup ( 0 ),
  collidesWith ( 0 ),
  collisionSoft ( 0 )
{
  // just create an empty rigid body. Parameters will be set in 
  // setRigidBodyParameters
  rigidBody.reset ( new btRigidBody( 0, NULL, NULL) );

  // The friction parameter is never used by rigid-rigid collisions because we have our 
  // own custom material callback. However, for rigid-soft collisions the friction used is
  // the minimum of both rigid body and soft body friction parameters. Therefore we set this
  // to 1 so that soft-rigid friction is controlled entirely by the soft body friction parameter.
  // Otherwise it is impossible to achieve maximum friction for soft-rigid collisions.
  rigidBody->setFriction ( 1.0f );

  // make sure the gContactAddedCallback will be called.
  rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() |
    btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );

  // Set a back-pointer to this object from the rigid body implementation
  // Makes it possible to access this object whenever bullet returns our btRigidBody
  // e.g. In collision detection.
  rigidBody->setUserPointer( this );

  // Set rigid body parameters
  setParameters( bodyParameters );
}

/// Apply an external force and torque to the body
void BulletRigidBody::addGlobalExternalForceAndTorque ( PhysicsEngineParameters::ExternalForceTorqueParameters& forces )
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(forces.engine_thread->getEngineSpecificData());


  if ( (forces.force.length() > Constants::f_epsilon || forces.torque.length() > Constants::f_epsilon) &&
    !(rigidBody->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT) ) {
      // after testing it seems like force is in global space and position is in local space
      // in Bullet. Very strange..
      btVector3 a_vly = rigidBody->getAngularVelocity().absolute();
      btVector3 l_vly = rigidBody->getLinearVelocity().absolute();
      if( bullet_data->m_maxVelocityAngular > 0 &&
          ( (a_vly.getX() > bullet_data->m_maxVelocityAngular) ||
            (a_vly.getY() > bullet_data->m_maxVelocityAngular) ||
            (a_vly.getZ() > bullet_data->m_maxVelocityAngular) ) ) {
        rigidBody->setAngularVelocity( tobtVector3( Vec3f( 0, 0, 0 ) ) );
        return;
      }

      if( bullet_data->m_maxVelocityLinear > 0 &&
          ( (l_vly.getX()>bullet_data->m_maxVelocityLinear) || 
            (l_vly.getY()>bullet_data->m_maxVelocityLinear) || 
            (l_vly.getZ()>bullet_data->m_maxVelocityLinear) ) ) {
        rigidBody->setLinearVelocity(tobtVector3(Vec3f(0, 0, 0)));
        return;
      }
      if( !rigidBody->isActive() && rigidBody->getActivationState() == ISLAND_SLEEPING ) {
        // This is a fix to make sure that rigidbodies which are set to autodisable will be affected by external forces
        // even if they have started sleeping. Otherwise they will not react unless some other rigidbody is hitting them.
        rigidBody->activate();
      }
      rigidBody->applyForce( tobtVector3 ( forces.force*bullet_data->m_worldScale ), rigidBody->getWorldTransform().inverse()*tobtVector3 ( forces.position*bullet_data->m_worldScale ) );
      rigidBody->applyTorque( tobtVector3 ( forces.torque*bullet_data->m_worldScale*bullet_data->m_worldScale ) );
  }
}

// Update bullet representation of rigid body from specified parameters
void BulletRigidBody::setParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters )
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());

  // fixed and mass properties. We also let enabled behave the same way as
  // fixed, i.e. if a body is not enabled we think of it as fixed.
  if( bodyParameters.haveFixed() || 
    bodyParameters.haveEnabled() || 
    bodyParameters.haveMassDensityModel() ||
    bodyParameters.haveMass() ) {
      if( bodyParameters.getFixed() || !bodyParameters.getEnabled() ) {
        rigidBody->setMassProps( 0, btVector3( 0, 0, 0 ) );
      } else {
        // not fixed, set mass properties.
        if ( bodyParameters.haveMassDensityModel() ) {
          massDensityModel= bodyParameters.getMassDensityModel();
        }
        if( !massDensityModel ) {
          btBoxShape box( btVector3( (btScalar)0.1, 
            (btScalar)0.1, 
            (btScalar)0.1 ) );
          btVector3 local_inertia( 0, 0, 0 );
          box.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
          rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );
        } else {
          if (Box *b = dynamic_cast< Box* >( massDensityModel ) ){
            Vec3f half_size = bullet_data->m_worldScale*b->size->getValue()/2;
            btBoxShape geom( btVector3( half_size.x, half_size.y, half_size.z ) );
            btVector3 local_inertia( 0, 0, 0 );
            geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
            rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
          } else if (Sphere *sph = dynamic_cast< Sphere * >( massDensityModel ) ){
            H3DFloat radius = bullet_data->m_worldScale*sph->radius->getValue();
            btSphereShape geom( radius );
            btVector3 local_inertia( 0, 0, 0 );
            geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
            rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );
          } else if (Cone *cone = dynamic_cast< Cone * >( massDensityModel ) ){
            H3DFloat radius = bullet_data->m_worldScale*cone->bottomRadius->getValue();
            H3DFloat height = bullet_data->m_worldScale*cone->height->getValue();
            btConeShape geom( radius, height );
            btVector3 local_inertia( 0, 0, 0 );
            geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
            rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
          } else if (Cylinder *cyl = dynamic_cast< Cylinder * >( massDensityModel ) ){
            H3DFloat radius = bullet_data->m_worldScale*cyl->radius->getValue();
            H3DFloat height = bullet_data->m_worldScale*cyl->height->getValue();
            btCylinderShape geom( btVector3( radius, height/2, radius ) );
            btVector3 local_inertia( 0, 0, 0 );
            geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
            rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
          } else {
            // we have an unsupported mass density model, use the bounding box of it as
            // density model
            Console(4) << "Invalid mass density model \"" << massDensityModel->getTypeName() << "\". " 
              << "Using bound of node as density model instead." << endl;
            X3DGeometryNode *g = static_cast< X3DGeometryNode * >( massDensityModel );
            BoxBound *bb = dynamic_cast< BoxBound * >( g->bound->getValue() );
            Vec3f half_size( 0.1f, 0.1f, 0.1f );
            if( bb ) half_size = bullet_data->m_worldScale*bb->size->getValue();

            btBoxShape box( btVector3( half_size.x, half_size.y, half_size.z ) );
            btVector3 local_inertia( 0, 0, 0 );
            box.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
            rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
          }
        }
      }
  }

  // useGlobalGravity
  if( bodyParameters.haveUseGlobalGravity() ) {
    if( bodyParameters.getUseGlobalGravity() ) {
      PhysicsEngineParameters::WorldParameters world_params = 
        bodyParameters.getEngine()->getWorldParameters();
      Vec3f g = world_params.getGravity()*bullet_data->m_worldScale;
      rigidBody->setGravity( btVector3(g.x, g.y, g.z) );
      // Needs to be set to make sure that global gravity is used
      // with latest bullet trunk. In collisionShapeChanged we call
      // btDiscreteDynamicsWorld::addRigidBody which calls setGravity for
      // btRigidBodies that has this flag set to 0.
      // This call might make our call to setGravity above useless.
      rigidBody->setFlags( 0 );
    } else {
      rigidBody->setGravity( btVector3(0, 0, 0) );
      // Needs to be set to make sure that global gravity is not used
      // with latest bullet trunk. In collisionShapeChanged we call
      // btDiscreteDynamicsWorld::addRigidBody which calls setGravity for
      // btRigidBodies that has this flag set to 0.
      // This call might make our call to setGravity above useless.
      rigidBody->setFlags( BT_DISABLE_WORLD_GRAVITY );
    }
  }

  // disable values
  if( bodyParameters.haveAutoDisable() || 
    bodyParameters.haveDisableLinearSpeed() || 
    bodyParameters.haveDisableAngularSpeed() ) {
    if( bodyParameters.getAutoDisable() ) {
      rigidBody->setSleepingThresholds( bodyParameters.getDisableLinearSpeed(),
        bodyParameters.getDisableAngularSpeed() );
      rigidBody->setActivationState ( ACTIVE_TAG );
    } else {
      PhysicsEngineParameters::WorldParameters world_params = 
        bodyParameters.getEngine()->getWorldParameters();
      if( world_params.getAutoDisable() ) {
        rigidBody->setSleepingThresholds( world_params.getDisableLinearSpeed(),
          world_params.getDisableAngularSpeed() );
        rigidBody->setActivationState ( ACTIVE_TAG );
      } else {
        // auto disable false everywhere, set to 0.
        rigidBody->setSleepingThresholds( 0, 0 );
        rigidBody->setActivationState ( DISABLE_DEACTIVATION );
      }
    }
  }

  // startPosition and startOrientation
  if( bodyParameters.haveStartPosition() || bodyParameters.haveStartOrientation() ) {
    btTransform t = rigidBody->getCenterOfMassTransform();
    if( bodyParameters.haveStartOrientation() ) {
      Quaternion q(bodyParameters.getStartOrientation() );
      t.setRotation( btQuaternion( q.v.x, q.v.y, q.v.z, q.w ) );
    }

    if( bodyParameters.haveStartPosition() ) {
      Vec3f v = bullet_data->m_worldScale*bodyParameters.getStartPosition();
      t.setOrigin(  btVector3( v.x, v.y, v.z ) );
    }

    rigidBody->setCenterOfMassTransform( t );
  }

  // startLinearVelocity 
  if( bodyParameters.haveStartLinearVelocity() ) {
    Vec3f v= bullet_data->m_worldScale*bodyParameters.getStartLinearVelocity();
    rigidBody->setLinearVelocity( btVector3( v.x, v.y, v.z ) );
  }

  //  startAngularVelocity
  if( bodyParameters.haveStartAngularVelocity() ) {
    Vec3f v= bullet_data->m_worldScale*bodyParameters.getStartAngularVelocity();
    rigidBody->setAngularVelocity( btVector3( v.x, v.y, v.z ) );
  }

  // Enable / disable kinematic control
  if ( bodyParameters.haveKinematicControl() ) {
    if ( bodyParameters.getKinematicControl() ) {
      rigidBody->setCollisionFlags ( rigidBody->getCollisionFlags() | 
        btCollisionObject::CF_KINEMATIC_OBJECT );
    } else {
      rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() &
        ~btCollisionObject::CF_KINEMATIC_OBJECT );
    }
  }

  if ( bodyParameters.havePosition() || 
    bodyParameters.haveOrientation() ) {
      btTransform t= rigidBody->getCenterOfMassTransform();

      if ( bodyParameters.havePosition() ) {
        t.setOrigin ( tobtVector3 ( bullet_data->m_worldScale*bodyParameters.getPosition() ) );
      }

      if ( bodyParameters.haveOrientation() ) {
        Quaternion q(bodyParameters.getOrientation() );
        t.setRotation( btQuaternion( q.v.x, q.v.y, q.v.z, q.w ) );
      }

      // Only one call to setCenterOfMassTransform() per simulation frame
      // otherwise problems with collision detection
      rigidBody->setCenterOfMassTransform ( t );
  }

  if ( bodyParameters.haveEngineOptions() ) {
    if ( BulletRigidBodyParameters* options= 
      dynamic_cast<BulletRigidBodyParameters*>(bodyParameters.getEngineOptions()) ) {

        if ( options->haveCollisionGroup() ) {
          collisionGroup= options->getCollisionGroup();
        }
        if ( options->haveCollidesWith() ) {
          collidesWith= options->getCollidesWith();
        }
        if( options->haveSoftBodyCollisionOptions() ) {
          const BulletRigidBodyParameters::SoftBodyCollisionOptions::e& o=
            options->getSoftBodyCollisionOptions();
          if( o == BulletRigidBodyParameters::SoftBodyCollisionOptions::SDF_RIGIDSOFT ) {
            collisionSoft = btSoftBody::fCollision::SDF_RS;
          } else if( o == BulletRigidBodyParameters::SoftBodyCollisionOptions::CLUSTER_RIGIDSOFT ) {
            collisionSoft = btSoftBody::fCollision::CL_RS;
          } else {
            collisionSoft = 0;
          }
        }

        if ( options->haveCollisionGroup() || options->haveCollidesWith()  ) {
          collisionShapeChanged();
        }
    }
  }

  // geometry
  if( bodyParameters.haveGeometry() ) {
    const vector<H3DCollidableId> geom_ids= bodyParameters.getGeometry();

    for ( BulletCollidableList::iterator i= collidables.begin(); 
      i != collidables.end(); ++i ) {
        (*i)->removeBody ( *this );
    }

    collidables.clear();

    for ( vector<H3DCollidableId>::const_iterator i= geom_ids.begin(); 
      i != geom_ids.end(); ++i ) {
        BulletCollidable* collidable= (BulletCollidable*)*i;
        collidables.push_back ( collidable );
        collidable->addBody ( *this );
    }

    updateCollidables();
  }
}

void BulletRigidBody::updateCollidables () {

  // No geometry specified
  if( collidables.empty() ) 
  {
    // If we just specify NULL here then the shape is never added for collision detection, which seems like
    // a good idea. But if the shape is not added it will cause a crash when calculating simulation islands for
    // joints. Traced to btDiscreteDynamicsWorld::calculateSimulationIslands() call to unite(). May be bug in bullet.
    // To avoid this, we use btEmptyShape
    collisionShape.reset ( new btEmptyShape() );
    rigidBody->setCollisionShape( collisionShape.get() );
  } 
  else 
  {
    // More than one collision shape required, use composite
    btCompoundShape* compoundShape= new btCompoundShape ();

    // For each specified geometry
    for ( BulletCollidableList::const_iterator i= collidables.begin(); 
      i != collidables.end(); ++i ) {
        // Recurse down and add atomic collidable to the compound shape
        // The hierachy is flattened so that the index can be used to identify
        // the child uniquely during collision detection
        (*i)->addCollidableTo ( *compoundShape );
    }

    // Set the compound shape as the body's geometry
    rigidBody->setCollisionShape( compoundShape );

    // Ensure additional collision shape is deleted
    collisionShape.reset ( compoundShape );

    validateCollidables( *compoundShape );
  }

  collisionShapeChanged();
}

void BulletRigidBody::validateCollidables ( btCompoundShape& compoundShape ) {
  if ( compoundShape.getNumChildShapes() > 1 ) {
    for ( int i= 0; i < compoundShape.getNumChildShapes(); ++i ) {
      if ( dynamic_cast<btBvhTriangleMeshShape*>(compoundShape.getChildShape(i)) ) {
        Console(4) << "Warning: Bullet implementation does not fully support "
          "multiple geometries per rigid body, where at least one geometry is a "
          "triangle mesh. CollisionSensor may report incorrect geometry in contact." << endl;
        return;
      }
    }
  }
}

// Get the current parameters of the rigid body
void BulletRigidBody::getParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters )
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());

  // Get the transform of the body.
  if ( !(rigidBody->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT) ) { 

    btTransform transform= rigidBody->getWorldTransform();

    bodyParameters.setPosition( toVec3f ( transform.getOrigin() )/bullet_data->m_worldScale );

    btQuaternion orn = transform.getRotation();
    bodyParameters.setOrientation( Rotation( Quaternion( orn.x(), 
      orn.y(), 
      orn.z(), 
      orn.w() ) ) );
  }

  bodyParameters.setLinearVelocity( toVec3f ( rigidBody->getLinearVelocity() )/bullet_data->m_worldScale );
  bodyParameters.setAngularVelocity( toVec3f ( rigidBody->getAngularVelocity() ) );
}

void BulletRigidBody::collisionShapeChanged ()
{
  BulletCallbacks::BulletSpecificData *bullet_data = 
    static_cast< BulletCallbacks::BulletSpecificData * >(physicsThread->getEngineSpecificData());

  // Make sure that the rigid body is not already in the world
  const btCollisionObjectArray &objects = bullet_data->m_dynamicsWorld->getCollisionObjectArray ();
  if( objects.findLinearSearch(rigidBody.get())  != objects.size()) {
    bullet_data->m_dynamicsWorld->removeRigidBody(rigidBody.get());
  }
  if ( collisionGroup ) {
    bullet_data->m_dynamicsWorld->addRigidBody(rigidBody.get(),collisionGroup,collidesWith);
  } else {
    bullet_data->m_dynamicsWorld->addRigidBody(rigidBody.get());
  }
}

BulletCollidable* BulletRigidBody::getCollidable ( size_t index ) { 
  if ( index < collidables.size() ) {
    return collidables[index];
  } else {
    return NULL;
  }
}

BulletCollidable::BulletCollidable ( PhysicsEngineParameters::CollidableParameters& collidableParameters ) 
  : enabled ( true ){
    localTransform= btTransform::getIdentity();
}

// Update bullet representation of collidable using specified parameters
void BulletCollidable::setParameters ( PhysicsEngineParameters::CollidableParameters& collidableParameters )
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(collidableParameters.getEngine()->getEngineSpecificData());
  bool needUpdateCollidables= false;

  // If the transform has been modified
  if ( collidableParameters.haveRotation() || collidableParameters.haveTranslation() )
  {
    // Calculate the new transform based on the parameters and the old transform
    updateTransform ( collidableParameters, localTransform );
    needUpdateCollidables= true;
  }

  // enabled or disable the collidable
  if ( collidableParameters.haveEnabled() )
  {
    enabled= collidableParameters.getEnabled();
    needUpdateCollidables= true;
  }

  if ( collidableParameters.haveEngineOptions() ) {
    if ( BulletCollidableParameters* options= 
      dynamic_cast<BulletCollidableParameters*>(collidableParameters.getEngineOptions()) ) {

        if ( options->haveCollisionMargin() ) {
          collidableShape->setMargin ( options->getCollisionMargin ()*bullet_data->m_worldScale );
        }
    }
  }

  // If required, re-add the collidables hierachy to the rigid bodies
  if ( needUpdateCollidables ) {
    updateCollidables ();
  }
}

// Add a rigid body that uses this collidable
void BulletCollidable::addBody ( BulletRigidBody& body )
{
  bodies.push_back ( &body );
}

// Remove a rigid body that no longer uses this collidable
void BulletCollidable::removeBody ( BulletRigidBody& body )
{
  RigidBodyList::iterator i= find ( bodies.begin(), bodies.end(), &body );
  if ( i != bodies.end() )
    bodies.erase ( i );
}

void BulletCollidable::updateCollidables ()
{
  for ( RigidBodyList::iterator i= bodies.begin(); i != bodies.end(); ++i ) {
    (*i)->updateCollidables ();
  }
}

void BulletCollidable::addCollidableTo ( btCompoundShape& compoundShape ) {
  addCollidableToRecursive ( compoundShape, btTransform::getIdentity(), true );
}

void BulletCollidable::addCollidableToRecursive ( btCompoundShape& compoundShape,
  const btTransform& parentTransform,
  bool parentEnabled )
{
  if ( enabled && parentEnabled ) {
    emptyShape.reset ( NULL );
    if( collidableShape.get() ) {
      compoundShape.addChildShape( localTransform*parentTransform, collidableShape.get() );
    }
  } else {
  //  emptyShape.reset ( new btEmptyShape() );
  //  compoundShape.addChildShape ( btTransform::getIdentity(), emptyShape.get() );
  }
}

/// Updates the transform to match the parameters specified
void BulletCollidable::updateTransform ( PhysicsEngineParameters::CollidableParameters& collidableParameters,
  btTransform& transform )
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(collidableParameters.getEngine()->getEngineSpecificData());

  // Update rotation if required
  if ( collidableParameters.haveRotation() )
    transform.setBasis ( tobtMatrix3x3 ( Matrix3f ( collidableParameters.getRotation() ) ) );

  // Update translation if required
  if ( collidableParameters.haveTranslation() )
    transform.setOrigin ( tobtVector3 ( collidableParameters.getTranslation() )*bullet_data->m_worldScale );
}

BulletCollidableOffset::BulletCollidableOffset ( PhysicsEngineParameters::OffsetParameters& offsetParameters )
  : BulletCollidable ( offsetParameters )
{
  childCollidable= (BulletCollidable*)offsetParameters.collidable;
  setParameters ( offsetParameters );
}

void BulletCollidableOffset::addCollidableToRecursive ( btCompoundShape& compoundShape,
  const btTransform& parentTransform,
  bool parentEnabled )
{
  childCollidable->addCollidableToRecursive ( compoundShape, 
    parentTransform * localTransform,
    enabled && parentEnabled );
}

// Add a rigid body that uses this collidable
// Add recursively
void BulletCollidableOffset::addBody ( BulletRigidBody& body ) {
  BulletCollidable::addBody ( body );
  childCollidable->addBody ( body );
}

// Remove a rigid body that no longer uses this collidable
// Remove recursively
void BulletCollidableOffset::removeBody ( BulletRigidBody& body ) {
  BulletCollidable::removeBody ( body );
  childCollidable->removeBody ( body );
}

// Handles the link between H3D CollidableShape and bullet representation
BulletCollidableShape::BulletCollidableShape ( PhysicsEngineParameters::ShapeParameters& shapeParameters )
  : BulletCollidable ( shapeParameters )
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(shapeParameters.getEngine()->getEngineSpecificData());

  // Create geometry
  collidableShape.reset ( createCollisionShape ( shapeParameters, *bullet_data ) );

  setParameters ( shapeParameters );
}

struct lt_btVector3 {
  bool operator()(const btVector3& v1, const btVector3& v2) const {
    return (v1.x()<v2.x()) || (v1.x()==v2.x() && v1.y()<v2.y()) || (v1.x()==v2.x() && v1.y()==v2.y() && v1.z()<v2.z());
  }
};

// Return an appropriate bullet btCollisionShape to match the specified X3DGeometryNode
btCollisionShape* BulletCollidableShape::createCollisionShape ( PhysicsEngineParameters::ShapeParameters& shapeParameters,
  BulletCallbacks::BulletSpecificData& bullet_data )
{
  // Bullet have special primitives for Sphere, Box, Cylinder and Cone 
  // geometries. The rest is represented as a btBvhTriangleMeshShape.  
  if (Box *b = dynamic_cast< Box * >( shapeParameters.getShape() ) ){
    Vec3f half_size = bullet_data.m_worldScale*b->size->getValue()/2;
    return new btBoxShape( btVector3( half_size.x, 
      half_size.y, 
      half_size.z ) );

  } else if (Sphere *sph = dynamic_cast< Sphere * >( shapeParameters.getShape() ) ){
    H3DFloat radius = bullet_data.m_worldScale*sph->radius->getValue();
    return new btSphereShape( radius );

  } else if (Cone *cone = dynamic_cast< Cone * >( shapeParameters.getShape() ) ){
    H3DFloat radius = bullet_data.m_worldScale*cone->bottomRadius->getValue();
    H3DFloat height = bullet_data.m_worldScale*cone->height->getValue();
    return new btConeShape( radius, height );

  } else if (Cylinder *cyl = dynamic_cast< Cylinder * >( shapeParameters.getShape() ) ){  
    H3DFloat radius = bullet_data.m_worldScale*cyl->radius->getValue();
    H3DFloat height = bullet_data.m_worldScale*cyl->height->getValue();
    return new btCylinderShape( btVector3( radius, height/2, radius ) );
  } else if( Capsule *caps = dynamic_cast< Capsule * >(shapeParameters.getShape()) ) {
    H3DFloat radius = bullet_data.m_worldScale*caps->radius->getValue();
    H3DFloat height = bullet_data.m_worldScale*caps->height->getValue();
    return new btCapsuleShape ( radius, height );
  } else {
    // Decide whether or not to use convex or concave triangle mesh representation
    bool convex= false;
    if ( shapeParameters.haveEngineOptions() ) {
      if ( BulletCollidableParameters* options= 
        dynamic_cast<BulletCollidableParameters*>(shapeParameters.getEngineOptions()) ) {
          convex= options->getConvex();
      }
    }

    // get triangles from geometry
    vector< HAPI::Collision::Triangle > triangles;
    if( shapeParameters.haveShape() ) {
      if( shapeParameters.getShape()->boundTree->isUpToDate() ) {
        // Only do getValue() if the tree is already uptodate, else causes OpenGL error
        shapeParameters.getShape()->boundTree->getValue()->getAllTriangles( triangles );
      }
    }

    // if there are no triangles built then don't create a shape
    if( triangles.size() <= 0 ) {
      Console( 4 ) << "Error: Bullet. error no triangles in the shape bound tree!" << endl;
      return NULL;
    }
    // Create appropriate collision shape
    if ( convex ) {

      set<btVector3,lt_btVector3> points;
      for( unsigned int i = 0; i < triangles.size(); ++i ) {
        points.insert(btVector3( (btScalar)triangles[i].a.x,(btScalar)triangles[i].a.y,(btScalar)triangles[i].a.z )*bullet_data.m_worldScale);
        points.insert(btVector3( (btScalar)triangles[i].b.x,(btScalar)triangles[i].b.y,(btScalar)triangles[i].b.z )*bullet_data.m_worldScale);
        points.insert(btVector3( (btScalar)triangles[i].c.x,(btScalar)triangles[i].c.y,(btScalar)triangles[i].c.z )*bullet_data.m_worldScale);
      }

      vector<btScalar> points2;
      points2.resize(points.size() * 3);
      int j = 0;
      for(set<btVector3,lt_btVector3>::const_iterator i=points.begin();i!=points.end();++i) {
        points2[j++] = i->x();
        points2[j++] = i->y();
        points2[j++] = i->z();
      }

      return new btConvexHullShape(&points2[0],int( points.size() ),sizeof(btScalar)*3);
    } else {
      // using 32 bit indices and 3 component vertices
      btTriangleMesh* triMesh= new btTriangleMesh( true, false );
      mesh.reset ( triMesh );
      triMesh->preallocateVertices( (int)triangles.size() * 3 );
      for( unsigned int i = 0; i < triangles.size(); ++i ) {

        triMesh->addTriangle( btVector3( (btScalar)triangles[i].a.x,
          (btScalar)triangles[i].a.y,
          (btScalar)triangles[i].a.z )*bullet_data.m_worldScale,
          btVector3( (btScalar)triangles[i].b.x,
          (btScalar)triangles[i].b.y,
          (btScalar)triangles[i].b.z )*bullet_data.m_worldScale,
          btVector3( (btScalar)triangles[i].c.x,
          (btScalar)triangles[i].c.y,
          (btScalar)triangles[i].c.z )*bullet_data.m_worldScale );
      }

      //return new btBvhTriangleMeshShape ( mesh, true );
      // Tests showed that btGImpactMeshShape performs better than btBvhTriangleMeshShape
      // which documentation says should only be used for static objects.
      btGImpactCollisionAlgorithm::registerAlgorithm(bullet_data.m_dispatcher.get());
      btGImpactMeshShape * trimesh = new btGImpactMeshShape(triMesh);
      trimesh->setMargin(bullet_data.m_collisionMargin);
      trimesh->updateBound();
      return trimesh;
    }
  }
}

H3DSoftBody::~H3DSoftBody() {
  for( int i = 0; i < softBody->m_nodes.size(); ++i ) {
    if( softBody->m_nodes[i].m_tag != NULL ) {
      list< BulletSoftBodyTagData * > * tag_datas =
        static_cast< list< BulletSoftBodyTagData * > * >(
        softBody->m_nodes[i].m_tag);
      delete tag_datas;
      softBody->m_nodes[i].m_tag = NULL;
    }
  }
}

// Apply external forces to the soft body
void H3DSoftBody::applyExternalForces ( H3DSoftBodyNodeParameters& bodyParameters ) {
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());

  // Apply user defined external forces
  const H3DSoftBodyNodeParameters::VertexForceList& vertexForces= 
    bodyParameters.getVertexForces();
  for ( H3DSoftBodyNodeParameters::VertexForceList::const_iterator i= vertexForces.begin();
    i != vertexForces.end(); ++i ) {
      softBody->addForce ( tobtVector3(i->second.force*bullet_data->m_worldScale), (int)i->second.index );
  }

  // Apply manipulation external forces
  const H3DSoftBodyNodeParameters::VertexForceList& manipulationForces= 
    bodyParameters.getManipulationForces();
  for ( H3DSoftBodyNodeParameters::VertexForceList::const_iterator i= manipulationForces.begin();
    i != manipulationForces.end(); ++i ) {
      softBody->addForce ( tobtVector3(i->second.force*bullet_data->m_worldScale), (int)i->second.index );
  }
}

void H3DSoftBody::setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters, bool updateGeometry ) {
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());
  BulletSoftBodyParameters* options= 
    dynamic_cast<BulletSoftBodyParameters*>(bodyParameters.getEngineOptions());

  if ( bodyParameters.havePhysicsMaterial() &&
    bodyParameters.haveH3DPhysicsMaterialParameters()) {

      H3DPhysicsMaterialParameters * matPar = 
        bodyParameters.getH3DPhysicsMaterialParameters();

      if( matPar->haveDamping() &&
        matPar->haveDampingParameters() ) {
          applyDampingParameters ( *matPar->getDampingParameters() );
      }
      if( matPar->haveFriction() &&
        matPar->haveFrictionParameters() ) {
          applyFrictionParameters ( *matPar->getFrictionParameters() );
      }
      if( matPar->haveMass() &&
        matPar->haveMassParameters() ) {
          applyMassParameters ( *matPar->getMassParameters() );
      }
      if( matPar->haveStiffness() &&
        matPar->haveStiffnessParameters() ) {
          applyStiffnessParameters ( *matPar->getStiffnessParameters() );
      }
      if( matPar->haveStiffnessAngular() &&
        matPar->haveStiffnessAngularParameters() ) {
        applyStiffnessAngularParameters( *matPar->getStiffnessAngularParameters() );
      }
      if( matPar->haveStiffnessVolume() &&
        matPar->haveStiffnessVolumeParameters() ) {
        applyStiffnessVolumeParameters( *matPar->getStiffnessVolumeParameters() );
      }
  }

  if ( bodyParameters.haveIndices () ) {
    indices= bodyParameters.getIndices();
  }

  if ( updateGeometry && bodyParameters.haveCoords () ) {
    // update existing coords
    const SoftBodyParameters::CoordList& coords= bodyParameters.getCoords();
    for ( int i= 0; i < (int)coords.size() && i < softBody->m_nodes.size(); ++i ) {
      softBody->m_nodes[i].m_x= tobtVector3 ( coords[i]*bullet_data->m_worldScale );
    }

    // add new coords
    for ( size_t i= softBody->m_nodes.size(); i < coords.size(); ++i ) {
      softBody->appendNode ( tobtVector3 ( coords[i]*bullet_data->m_worldScale ), 1 );
    }

    // remove old coords
    softBody->m_nodes.resize ( (int)coords.size() );
  }

  if ( updateGeometry && (bodyParameters.haveIndices () || bodyParameters.haveCoords ()) ) {
    if ( options ) {
      if ( options->havePIterations() ) {
        softBody->m_cfg.piterations= options->getPIterations();
      }

      if ( options->haveBendingContraintDistance() ) {
        softBody->generateBendingConstraints ( options->getBendingContraintDistance() );
      }
    }

    applyFixedConstraints();
  }

  if ( bodyParameters.haveEngineOptions() && options ) {
    if ( options->haveCollisionMargin() ) {
      softBody->getCollisionShape()->setMargin ( options->getCollisionMargin ()*bullet_data->m_worldScale );
    }

    if ( options->havePIterations() ) {
      softBody->m_cfg.piterations= options->getPIterations();
    }

    if ( options->haveDIterations() ) {
      softBody->m_cfg.diterations= options->getDIterations();
    }

    if ( options->haveCIterations() ) {
      softBody->m_cfg.citerations= options->getCIterations();
    }

    if ( updateGeometry ) {
      if ( options->haveBendingContraintDistance() ) {
        softBody->generateBendingConstraints ( options->getBendingContraintDistance() );
      }

      if ( options->haveNrClusters() ) {
        softBody->generateClusters ( options->getNrClusters() );
        nrClusters = options->getNrClusters();
      }
    }

    if ( options->haveEnableFastEdgeBuilding() ) {
      enableFastEdgeBuilding= options->getEnableFastEdgeBuilding();
    }
    if ( options->haveEnablePerEdgeStiffness() ) {
      enablePerEdgeStiffness= options->getEnablePerEdgeStiffness();
    }

    if ( options->haveSoftRigidClusterHardness() ) {
      softBody->m_cfg.kSRHR_CL = options->getSoftRigidClusterHardness();
    }

    if ( options->haveSoftRigidClusterImpulseSplit() ) {
      softBody->m_cfg.kSR_SPLT_CL = options->getSoftRigidClusterImpulseSplit();
    }

    if ( options->haveSoftKineticClusterHardness() ) {
      softBody->m_cfg.kSKHR_CL = options->getSoftKineticClusterHardness();
    }

    if ( options->haveSoftKineticClusterImpulseSplit() ) {
      softBody->m_cfg.kSK_SPLT_CL= options->getSoftKineticClusterImpulseSplit();
    }

    if( options->haveSoftRigidHardness() ) {
      softBody->m_cfg.kCHR = options->getSoftRigidHardness();
    }

    if( options->haveSoftKineticHardness() ) {
      softBody->m_cfg.kKHR = options->getSoftKineticHardness();
    }

    if( options->haveCollisionGroup() ) {
      softBody->collisionGroup = options->getCollisionGroup();
    }

    if( options->haveCollidesWith() ) {
      softBody->collidesWith = options->getCollidesWith();
    }

    if( 
      options->havePoseMatchingVolume() ||
      options->havePoseMatchingFrame() ) {
      softBody->setPose( options->getPoseMatchingVolume(), options->getPoseMatchingFrame() );
    }

    if( options->havePoseMatchingCoefficient() ) {
      softBody->m_cfg.kMT = options->getPoseMatchingCoefficient();
    }
  }

  setCollisionOptions ( bodyParameters );
}

struct GetForceMag {
  int size ( btSoftBody& sb ) { return sb.m_nodes.size(); }
  H3DFloat operator () ( btSoftBody& sb, int i ) { return sb.m_nodes[i].m_f.length(); }
};

struct GetSpeed {
  int size ( btSoftBody& sb ) { return sb.m_nodes.size(); }
  H3DFloat operator () ( btSoftBody& sb, int i ) { return sb.m_nodes[i].m_v.length(); }
};

struct GetForce {
  int size ( btSoftBody& sb ) { return sb.m_nodes.size(); }
  Vec3f operator () ( btSoftBody& sb, int i ) { return toVec3f(sb.m_nodes[i].m_f); }
};

struct GetVelocity {
  int size ( btSoftBody& sb ) { return sb.m_nodes.size(); }
  Vec3f operator () ( btSoftBody& sb, int i ) { return toVec3f(sb.m_nodes[i].m_v); }
};

// Get the current parameters of the soft body
void H3DSoftBody::getParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters )
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());

  H3DSoftBodyNodeParameters::CoordList coords;
  for ( int i= 0; i < softBody->m_nodes.size(); ++i )
  {
    btSoftBody::Node n= softBody->m_nodes[i];
    coords.push_back ( toVec3f ( n.m_x )/bullet_data->m_worldScale );
  }
  bodyParameters.setCoords ( coords );
  bodyParameters.setIndices ( indices );

  // Get float attributes
  const H3DSoftBodyNodeParameters::FloatOutputList& outputsFloat= bodyParameters.getOutputsFloat();
  for ( H3DSoftBodyNodeParameters::FloatOutputList::const_iterator i= outputsFloat.begin(); i != outputsFloat.end(); ++i ) {
    SoftBodyFloatAttributeParameters* output= *i;
    switch ( output->getOutputType() ) {
      case SoftBodyFloatAttributeParameters::OUTPUT_FORCE_MAGNITUDE: {
        // Output force magnitude
        switch ( output->getUnitType() ) {
        case SoftBodyFloatAttributeParameters::UNIT_NODE: {
          // Per node/vertex
          setOutputAttribute ( *output, GetForceMag() );
          break;
        }
        default:
          break;
        }
        break;
      }
      case SoftBodyFloatAttributeParameters::OUTPUT_SPEED: {
        // Output force magnitude
        switch ( output->getUnitType() ) {
          case SoftBodyFloatAttributeParameters::UNIT_NODE: {
            // Per node/vertex
            setOutputAttribute ( *output, GetSpeed() );
            break;
          }
          default:
            break;
        }
        break;
      }
      default:
        break;
    }
  }

  // Get Vec3f attributes
  const H3DSoftBodyNodeParameters::Vec3fOutputList& outputsVec3f= bodyParameters.getOutputsVec3f();
  for ( H3DSoftBodyNodeParameters::Vec3fOutputList::const_iterator i= outputsVec3f.begin(); i != outputsVec3f.end(); ++i ) {
    SoftBodyVec3fAttributeParameters* output= *i;
    switch ( output->getOutputType() ) {
      case SoftBodyVec3fAttributeParameters::OUTPUT_FORCE: {
        // Output force
        switch ( output->getUnitType() ) {
          case SoftBodyVec3fAttributeParameters::UNIT_NODE: {
            // Per node/vertex
            setOutputAttribute ( *output, GetForce() );
            break;
          }
          default:
            break;
        }
        break;
      }
      case SoftBodyVec3fAttributeParameters::OUTPUT_VELOCITY: {
        // Ouput velocity
        switch ( output->getUnitType() ) {
          case SoftBodyVec3fAttributeParameters::UNIT_NODE: {
            // Per node/vertex
            setOutputAttribute ( *output, GetVelocity() );
            break;
          }
          default:
            break;
        }
        break;
      }
      default:
        break;
    }
  }
}

void H3DSoftBody::updateEdgeStiffness () {
  for ( int i= 0; i < softBody->m_links.size(); ++i ) {
    btSoftBody::Link& l= softBody->m_links[i];
    // update the stiffness of the link
    // Could use: softBody->updateConstants(), but then the rest length is recalculated
    // causing the whole body to 'sag'.
    l.m_c0= (l.m_n[0]->m_im+l.m_n[1]->m_im)/l.m_material->m_kLST;
  }
}

btSoftBody::Material* H3DSoftBody::getLinkMaterial ( size_t linkIndex ) {
  while ( linkIndex+1 >= (size_t)softBody->m_materials.size() ) {
    btSoftBody::Material* m= softBody->appendMaterial();
    m->m_kAST= 1.0f;
    m->m_kLST= 1.0f;
    m->m_kVST= 1.0f;
  }
  return softBody->m_materials[static_cast<int>(linkIndex+1)];
}

void H3DSoftBody::removeLinkAndMaterial ( size_t linkIndex ) {
  // Remove the link
  btSwap(softBody->m_links[static_cast<int>(linkIndex)],
    softBody->m_links[softBody->m_links.size()-1]);
  softBody->m_links.pop_back();

  // Remove the material
  size_t materialIndex= linkIndex+1;
  if ( materialIndex < (size_t)softBody->m_materials.size() ) {
    btSwap(softBody->m_materials[static_cast<int>(materialIndex)],
      softBody->m_materials[softBody->m_materials.size()-1]);
    btAlignedFree(softBody->m_materials[softBody->m_materials.size()-1]);
    softBody->m_materials.pop_back();
  }
}

void H3DSoftBody::addFixedConstraint ( BulletFixedConstraint& fixedConstraint ) {
  fixedConstraints.push_back ( &fixedConstraint );
}

void H3DSoftBody::removeFixedConstraint ( BulletFixedConstraint& fixedConstraint ) {
  FixedConstraintList::iterator i= find ( fixedConstraints.begin(), fixedConstraints.end(), &fixedConstraint );
  if ( i != fixedConstraints.end() ) {
    fixedConstraints.erase ( i );
  }
}

void H3DSoftBody::applyFixedConstraints () {
  // Set body mass to set none-zero masses for nodes, unfixing any previously fixed nodes
  setMass ( mass );

  for ( FixedConstraintList::iterator i= fixedConstraints.begin(); i != fixedConstraints.end(); ++i ) {
    BulletFixedConstraint& constraint= **i;

    // Set zero mass for current fixed nodes
    const FixedConstraintParameters::IndexList& indices= constraint.getIndices();
    for ( H3DSoftBodyNodeParameters::IndexList::const_iterator i= indices.begin(); 
      i != indices.end(); ++i ) {
      softBody->setMass( *i, 0 );
      // Reset node velocity and other properties now it is fixed
      softBody->m_nodes[*i].m_f = btVector3( 0, 0, 0 );
      softBody->m_nodes[*i].m_v = btVector3( 0, 0, 0 );
      softBody->m_nodes[*i].m_q = softBody->m_nodes[*i].m_x;

    }
  }

  // Must regenerate clusters after changing fixed vertices
  softBody->generateClusters( nrClusters );
}

void H3DSoftBody::applyDampingParameters ( DampingParameters& dampingParams ) {
  switch ( dampingParams.getUnitType() ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->m_cfg.kDP= dampingParams.getValuePerUnit(0);
    break;
  case MaterialPropertyParameters::UNIT_NODE:
    break;
  default:
    Console(4) << "Warning: Damping unit type is not supported!" << endl;
  };
}

void H3DSoftBody::applyFrictionParameters ( FrictionParameters& frictionParams ) {
  switch ( frictionParams.getUnitType() ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->m_cfg.kDF= frictionParams.getValuePerUnit(0);
    break;
  case MaterialPropertyParameters::UNIT_NODE:
    break;
  default:
    Console(4) << "Warning: Friction unit type is not supported!" << endl;
  };
}

void H3DSoftBody::applyMassParameters ( MassParameters& massParams ) {
  switch ( massParams.getUnitType() ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    mass= massParams.getValuePerUnit(0);
    setMass ( mass );
    break;
  case MaterialPropertyParameters::UNIT_NODE:
    break;
  default:
    Console(4) << "Warning: Mass unit type is not supported!" << endl;
  };
}

void H3DSoftBody::applyStiffnessParameters ( StiffnessParameters& stiffnessParams ) {

  if ( stiffnessParams.haveUnitType() ) {
    stiffnessUnitType= stiffnessParams.getUnitType();
  }

  switch ( stiffnessUnitType ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->m_materials[0]->m_kLST = stiffnessParams.getValuePerUnit(0);

    // Make all links use the same stiffness
    clearPerEdgeStiffness ();
    break;
  case MaterialPropertyParameters::UNIT_EDGE:
    // Apply different stiffness each edge
    setStiffnessPerEdge ( stiffnessParams );
    break;
  default:
    Console( LogLevel::Warning ) << "Warning: Stiffness unit type is not supported!" << endl;
  };
}

void H3DSoftBody::applyStiffnessAngularParameters( StiffnessParameters& stiffnessParams ) {

  if( stiffnessParams.haveUnitType() ) {
    stiffnessAngularUnitType = stiffnessParams.getUnitType();
  }

  switch( stiffnessAngularUnitType ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->m_materials[0]->m_kAST = stiffnessParams.getValuePerUnit( 0 );

    if( hasEdgeStiffness ) {
      Console( LogLevel::Warning ) << 
        "Warning: It is not supported to have a uniform stiffness and a non-uniform angular stiffness! " << 
        "The stiffness unit types must match." << endl;
    }

    // Make all links use the same stiffness
    clearPerEdgeStiffness();
    break;
  //case MaterialPropertyParameters::UNIT_EDGE:
    // NOTE: Not implemented
  default:
    Console( LogLevel::Warning ) << "Warning: Angular stiffness unit type is not supported!" << endl;
  };
}

void H3DSoftBody::applyStiffnessVolumeParameters( StiffnessParameters& stiffnessParams ) {

  if( stiffnessParams.haveUnitType() ) {
    stiffnessVolumeUnitType = stiffnessParams.getUnitType();
  }

  switch( stiffnessVolumeUnitType ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->m_materials[0]->m_kVST = stiffnessParams.getValuePerUnit( 0 );

    if( hasEdgeStiffness ) {
      Console( LogLevel::Warning ) <<
        "Warning: It is not supported to have a uniform stiffness and a non-uniform volume stiffness! " <<
        "The stiffness unit types must match." << endl;
    }

    // Make all links use the same stiffness
    clearPerEdgeStiffness();
    break;
  //case MaterialPropertyParameters::UNIT_EDGE:
    // NOTE: Not implemented
  default:
    Console( LogLevel::Warning ) << "Warning: Volume stiffness unit type is not supported!" << endl;
  };
}

void H3DSoftBody::setCollisionOptions ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters ) {
  BulletSoftBodyParameters* options= 
    dynamic_cast<BulletSoftBodyParameters*>(bodyParameters.getEngineOptions());
  // For now collision with rigidbodies will always be enabled.
  // Unless you use a BulletSoftBodyOptions node to specify collisionOptions
  if ( options && options->haveCollisionOptions() ) {
    collisionOptions= options->getCollisionOptions();

    softBody->m_cfg.collisions= 0;
    if ( collisionOptions & BulletSoftBodyParameters::SDF_RIGIDSOFT ) {
      softBody->m_cfg.collisions|= btSoftBody::fCollision::SDF_RS;
    }
    if ( collisionOptions & BulletSoftBodyParameters::CLUSTER_RIGIDSOFT ) {
      softBody->m_cfg.collisions|= btSoftBody::fCollision::CL_RS;
    }
    if ( collisionOptions & BulletSoftBodyParameters::VERTEXFACE_SOFTSOFT ) {
      softBody->m_cfg.collisions|= btSoftBody::fCollision::VF_SS;
    }
    if ( collisionOptions & BulletSoftBodyParameters::CLUSTER_SOFTSOFT ) {
      softBody->m_cfg.collisions|= btSoftBody::fCollision::CL_SS;
    }
    if ( collisionOptions & BulletSoftBodyParameters::CLUSTER_SELF ) {
      softBody->m_cfg.collisions|= btSoftBody::fCollision::CL_SELF;
    }
    // Must regenerate clusters after changing collision options
    softBody->generateClusters ( nrClusters );
  }
}

void H3DSoftBody::setEdgeOptions ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters ) {
  BulletSoftBodyParameters* options= 
    dynamic_cast<BulletSoftBodyParameters*>(bodyParameters.getEngineOptions());

  if ( bodyParameters.haveEngineOptions() && options ) {
    if ( options->haveEnableFastEdgeBuilding() ) {
      enableFastEdgeBuilding= options->getEnableFastEdgeBuilding();
    }
    if ( options->haveEnablePerEdgeStiffness() ) {
      enablePerEdgeStiffness= options->getEnablePerEdgeStiffness();
    }
  }
}

void H3DSoftBody::setStiffnessPerEdge ( StiffnessParameters& stiffnessParams ) {

  if ( enablePerEdgeStiffness ) {
    hasEdgeStiffness = true;

    // Use 1 material per edge to control stiffness per edge
    // Keep 1st material to control default stiffness
    const StiffnessParameters::EditVector& changes= stiffnessParams.getChanges();
    if ( changes.empty() ) {
      // Update all
      for ( unsigned int i= 0; i < edgeToLink.size(); ++i ) {
        int linkIndex= edgeToLink[i];

        if ( linkIndex >= 0 && linkIndex < softBody->m_links.size() ) {

          btSoftBody::Link& l= softBody->m_links[linkIndex];
          l.m_material= getLinkMaterial ( linkIndex );
          l.m_material->m_kLST = stiffnessParams.getValuePerUnit(i);
          l.m_material->m_kVST = stiffnessParams.getValuePerUnit(i);
          l.m_material->m_kAST = stiffnessParams.getValuePerUnit(i);

          // Could use: softBody->updateConstants(), but then the rest length is recalculated
          // causing the whole body to 'sag'. Instead just recalculate l.m_c0 for updated edges 
          l.m_c0= (l.m_n[0]->m_im+l.m_n[1]->m_im)/stiffnessParams.getValuePerUnit(i);
        } else {
          Console(4) << "setStiffnessPerEdge(): No link for index " << linkIndex << ". There are " << softBody->m_links.size() << " links." << endl;
        }
      }
    } else {
      // Update only links that have changed
      for ( StiffnessParameters::EditVector::const_iterator i= changes.begin(); i != changes.end(); ++i ) {
        StiffnessParameters::Edit edit= *i;
        if ( edit.type == StiffnessParameters::FieldType::Edit_Update ) {
          for ( size_t j= 0; j < edit.index.size(); ++j ) {
            int edgeIndex= edit.index[j];
            H3DFloat edgeStiffness= edit.value[j];
            if ( edgeIndex >= 0 && edgeIndex < (int)edgeToLink.size() ) {
              int linkIndex= edgeToLink[edgeIndex];
              if ( linkIndex >= 0 && linkIndex < softBody->m_links.size() ) {
                btSoftBody::Link& l= softBody->m_links[linkIndex];
                l.m_material= getLinkMaterial ( linkIndex );
                l.m_material->m_kLST = edgeStiffness;
                l.m_material->m_kVST = edgeStiffness;
                l.m_material->m_kAST = edgeStiffness;

                // Could use: softBody->updateConstants(), but then the rest length is recalculated
                // causing the whole body to 'sag'. Instead just recalculate l.m_c0 for updated edges 
                l.m_c0= (l.m_n[0]->m_im+l.m_n[1]->m_im)/edgeStiffness;
              } else {
                Console(4) << "setStiffnessPerEdge(): No link for index " << linkIndex << ". There are " << softBody->m_links.size() << " links." << endl;
              }
            } else {
              Console(4) << "Warning: Attempt to update edge index out of range (" << edgeIndex << " of " << edgeToLink.size() << ")!" << endl;
            }
          }
        } else {
          Console(4) << "Warning: Unsupported edit type for soft body stiffness. Only updates are supported, not inserts or deletes!" << endl;
        }
      }
    }
  } else {
    Console(4) << "Warning: setStiffnessPerEdge(): Trying to set per edge stiffness when "
      "enablePerEdgeStiffness is disabled! Please see the enablePerEdgeStiffness in BulletSoftBodyOptions." << endl;
  }
}

void H3DSoftBody::clearPerEdgeStiffness () {
  for ( int i= 0; i < softBody->m_links.size(); ++i ) {
    btSoftBody::Link& l= softBody->m_links[i];
    // Don't touch links that belong to SoftBodyAttachments
    if ( !l.m_tag ) {
      l.m_material= softBody->m_materials[0];
      // update the stiffness of the link
      // Could use: softBody->updateConstants(), but then the rest length is recalculated
      // causing the whole body to 'sag'.
      l.m_c0=  (l.m_n[0]->m_im+l.m_n[1]->m_im)/l.m_material->m_kLST;
    }
  }

  hasEdgeStiffness = false;
}

void H3DSoftBody::clearEdgeLinks () {
  curEdgeLinkIndex= -1;
  edgeToLink.clear();

  // Remove all links that are not owned soft attachments
  for ( int i= 0; i < softBody->m_links.size(); ++i ) {
    btSoftBody::Link& l= softBody->m_links[i];
    // Don't touch links that belong to SoftBodyAttachments
    if ( !l.m_tag ) {
      removeLinkAndMaterial ( i );
      --i;
    }
  }
}

void H3DSoftBody::initAddEdgeLinks ( size_t _nrNodes ) {
  if ( enableFastEdgeBuilding ) {
#ifdef FAST_EDGE_BUILDING_USE_STL_VECTOR
    edgeAdded.clear ();
    edgeAdded.resize ( _nrNodes, vector<int>( _nrNodes, false ) );
#else
    nrNodes= _nrNodes;
    edgeAdded= new bool* [nrNodes];
    for ( size_t i= 0; i < nrNodes; ++i ) {
      edgeAdded[i]= new bool [nrNodes];
      for ( size_t j= 0; j < nrNodes; ++j ) {
        edgeAdded[i][j]= false;
      }
    }
#endif
  }
}

void H3DSoftBody::addEdgeLink ( int node0, int node1 ) {
#ifndef FAST_EDGE_BUILDING_USE_STL_VECTOR
  if ( edgeAdded ) {
#endif
    if ( enableFastEdgeBuilding ) {
      if ( !edgeAdded[node0][node1] && !edgeAdded[node1][node0] ) {
        softBody->appendLink(node0,node1,0,false);
        edgeAdded[node0][node1]= true;
        edgeAdded[node1][node0]= true;
      }
    } else {
      softBody->appendLink(node0,node1,0,true);
    }
#ifndef FAST_EDGE_BUILDING_USE_STL_VECTOR
  }
#endif  

  // This is only required to allow setting
  // stiffness per edge. Otherwise this can be skipped otherwise
  // and will save time and memory.
  if ( enablePerEdgeStiffness ) {
    ++curEdgeLinkIndex;
    if ( edgeToLink.size() < curEdgeLinkIndex+1 ) {
      edgeToLink.resize ( curEdgeLinkIndex+1 );
    }

    btSoftBody::Node* n0= &softBody->m_nodes[node0];
    btSoftBody::Node* n1= &softBody->m_nodes[node1];
    int linkIndex= -1;
    for ( int i= 0; i < softBody->m_links.size(); ++i ) {
      btSoftBody::Link& l= softBody->m_links[i];
      if ( (l.m_n[0] == n0 && l.m_n[1] == n1) ||
        (l.m_n[0] == n1 && l.m_n[1] == n0) ) {
          linkIndex= i;
          break;
      }
    }

    edgeToLink[curEdgeLinkIndex]= linkIndex;
  }
}

void H3DSoftBody::deinitAddEdgeLinks () {
#ifdef FAST_EDGE_BUILDING_USE_STL_VECTOR
  edgeAdded.clear();
#else
  if ( edgeAdded ) {
    for ( size_t i= 0; i < nrNodes; ++i ) {
      delete [] edgeAdded[i];
    }
    delete [] edgeAdded;
    edgeAdded= NULL;
    nrNodes= 0;
  }
#endif
}

// Handles the link between H3D SoftBody and bullet representation
BulletSoftBody::BulletSoftBody ( SoftBodyParameters& bodyParameters,
  BulletCallbacks::BulletSpecificData& bulletData )
{
  coordPerElement= 4;

  BulletSoftBodyParameters* options= 
    dynamic_cast<BulletSoftBodyParameters*>(bodyParameters.getEngineOptions());

  const SoftBodyParameters::CoordList& coords= bodyParameters.getCoords();
  indices= bodyParameters.getIndices();

  btVector3* vertices= new btVector3 [ coords.size() ];
  for ( size_t i= 0; i < coords.size(); ++i ) {
    vertices[i]= tobtVector3 ( coords[i]*bulletData.m_worldScale );
  }

  softBody.reset ( 
    new btSoftBodyH3D (
    &bulletData.m_softBodyWorldInfo, 
    (int)coords.size(),
    vertices,
    NULL ) );
  delete [] vertices;

  // Add a 2nd default material which we will use
  // to implement soft body attachment links
  softBody->appendMaterial();

  // Link nodes to form tetra
  appendTetra( bodyParameters );

  if ( options ) {
    if ( options->haveBendingContraintDistance() ) {
      softBody->generateBendingConstraints ( options->getBendingContraintDistance() );
    }

    if ( options->haveNrClusters() ) {
      softBody->generateClusters ( options->getNrClusters() );
      nrClusters = options->getNrClusters();
    }
  } else {
    softBody->generateBendingConstraints( 2 );
    softBody->generateClusters( 64 );
  }
  
  softBody->getCollisionShape()->setMargin ( bulletData.m_collisionMargin * bulletData.m_worldScale );
  setParameters ( bodyParameters, false );
}

// Update bullet representation of soft body from specified parameters
void BulletSoftBody::setParameters ( H3DSoftBodyNodeParameters& bodyParameters, bool updateGeometry ) {
  SoftBodyParameters* params= dynamic_cast<SoftBodyParameters*> ( &bodyParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());

  if ( params ) {

    if ( updateGeometry && (params->haveIndices () || params->haveCoords ()) ) {
      softBody->m_tetras.clear();

      // Link nodes to form tetra
      appendTetra( bodyParameters );
    }
  }

  H3DSoftBody::setParameters ( bodyParameters, updateGeometry );
}

void BulletSoftBody::setMass ( H3DFloat _mass ) {
  softBody->setVolumeMass ( _mass );
}

void BulletSoftBody::appendTetra( H3DSoftBodyNodeParameters& bodyParameters ) {
  if( !bodyParameters.getGeometry() ) {
    Console( LogLevel::Warning ) << "Warning: No SoftBody geometry!" << endl;
    return;
  }

  IndexedTetraSet * its = dynamic_cast< IndexedTetraSet * >(bodyParameters.getGeometry());
  IndexedHexaSet * ihs = dynamic_cast< IndexedHexaSet * >(bodyParameters.getGeometry());
  if( !its && !ihs )
    Console(4) << "Warning: Softbody geometry of type " 
    << bodyParameters.getGeometry()->getTypeName()
    << " is not supported." << endl;
  setEdgeOptions ( bodyParameters );
  initAddEdgeLinks( softBody->m_nodes.size() );
  if( its ) {
    for ( int i= 0; static_cast<size_t>(i) < indices.size(); i+= 4 )
    {
      appendOneTetra( i, i+1, i+2, i+3 );
    }
  } else if( ihs ) {
    for ( int i= 0; static_cast<size_t>(i) < indices.size(); i+= 8 )
    {
      // Inscribe four tetras in each IndexedHexaSet.
      appendOneTetra( i+1, i+4, i+2, i );
      appendOneTetra( i+3, i+2, i+7, i );
      appendOneTetra( i+2, i+1, i+6, i+4 );
      appendOneTetra( i+5, i+4, i+6, i+1 );
      appendOneTetra( i+6, i+7, i+2, i+4 );
      appendOneTetra( i+4, i, i+7, i+2 );
    }
  }
  deinitAddEdgeLinks();
}

void BulletSoftBody::appendOneTetra( const H3DSoftBodyNodeParameters::IndexList::value_type &i,
  const H3DSoftBodyNodeParameters::IndexList::value_type &j,
  const H3DSoftBodyNodeParameters::IndexList::value_type &k,
  const H3DSoftBodyNodeParameters::IndexList::value_type &l ) {
    softBody->appendTetra ( indices[i], indices[j], indices[k], indices[l] );
    addEdgeLink ( indices[i], indices[j] );
    addEdgeLink ( indices[j], indices[k] );
    addEdgeLink ( indices[k], indices[i] );
    addEdgeLink ( indices[i], indices[l] );
    addEdgeLink ( indices[j], indices[l] );
    addEdgeLink ( indices[k], indices[l] );
}

BulletCloth::BulletCloth ( PhysicsEngineParameters::ClothParameters& bodyParameters,
  BulletCallbacks::BulletSpecificData& bulletData )
{
  coordPerElement= 3;

  BulletSoftBodyParameters* options= 
    dynamic_cast<BulletSoftBodyParameters*>(bodyParameters.getEngineOptions());

  const SoftBodyParameters::CoordList& coords= bodyParameters.getCoords();
  indices= bodyParameters.getIndices();

  btVector3* vertices= new btVector3 [ coords.size() ];
  for ( size_t i= 0; i < coords.size(); ++i ) {
    vertices[i]= tobtVector3 ( coords[i]*bulletData.m_worldScale );
  }

  softBody.reset ( 
    new btSoftBodyH3D (
    &bulletData.m_softBodyWorldInfo, 
    (int)coords.size(),
    vertices,
    NULL ) );
  delete [] vertices;

  // Add a 2nd default material which we will use
  // to implement soft body attachment links
  softBody->appendMaterial();

  // Link nodes to form faces
  setEdgeOptions ( bodyParameters );
  clearEdgeLinks();
  initAddEdgeLinks ( softBody->m_nodes.size() );
  for ( size_t i= 0; i < indices.size(); i+= 3 )
  {
    softBody->appendFace ( indices[i], indices[i+1], indices[i+2] );
    addEdgeLink ( indices[i], indices[i+1] );
    addEdgeLink ( indices[i+1], indices[i+2] );
    addEdgeLink ( indices[i+2], indices[i] );
  }
  deinitAddEdgeLinks();

  ////softBody->m_cfg.citerations = 15;
  ////softBody->m_cfg.diterations = 15;
  ////softBody->m_cfg.piterations = 15;

  //softBody->m_cfg.kSRHR_CL = 0.5;
  //softBody->m_cfg.kSR_SPLT_CL = 0;

  if ( options ) {
    if ( options->haveBendingContraintDistance() ) {
      softBody->generateBendingConstraints ( options->getBendingContraintDistance() );
    }

    if ( options->haveNrClusters() ) {
      softBody->generateClusters ( options->getNrClusters() );
      nrClusters = options->getNrClusters();
    }
  } else {
    softBody->generateBendingConstraints(2);
    softBody->generateClusters(64);
  }

  softBody->getCollisionShape()->setMargin ( bulletData.m_collisionMargin * bulletData.m_worldScale );

  setParameters ( bodyParameters, false );
}

// Update bullet representation of soft body from specified parameters
void BulletCloth::setParameters ( H3DSoftBodyNodeParameters& bodyParameters, bool updateGeometry ) {
  ClothParameters* params= dynamic_cast<ClothParameters*> ( &bodyParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());

  if ( params ) {

    if ( updateGeometry && (params->haveIndices () || params->haveCoords ()) ) {
      softBody->m_faces.clear();

      // Link nodes to form faces
      setEdgeOptions ( bodyParameters );
      clearEdgeLinks();
      initAddEdgeLinks ( softBody->m_nodes.size() );
      for ( size_t i= 0; i < indices.size(); i+= 3 )
      {
        softBody->appendFace ( indices[i], indices[i+1], indices[i+2] );
        addEdgeLink ( indices[i], indices[i+1] );
        addEdgeLink ( indices[i+1], indices[i+2] );
        addEdgeLink ( indices[i+2], indices[i] );
      }
      deinitAddEdgeLinks();
    }
  }

  H3DSoftBody::setParameters ( bodyParameters, updateGeometry );
}

void BulletCloth::setMass ( H3DFloat _mass ) {
  softBody->setTotalMass ( _mass );
}

void BulletCloth::geometryFromBullet() {
  indices.clear();
  const btSoftBody::Node* nodeBase= &softBody->m_nodes[0];
  for ( int f= 0; f < softBody->m_faces.size(); ++f ) {
    const btSoftBody::Face& face= softBody->m_faces[f];
    // This casting should be ok except for EXTREMELY big soft bodies
    // but maybe check the size and warn if it's too big
    indices.push_back ( static_cast<H3DInt32>(face.m_n[0]-nodeBase) );
    indices.push_back ( static_cast<H3DInt32>(face.m_n[1]-nodeBase) );
    indices.push_back ( static_cast<H3DInt32>(face.m_n[2]-nodeBase) );
  }
}

BulletRope::BulletRope ( PhysicsEngineParameters::RopeParameters& bodyParameters,
  BulletCallbacks::BulletSpecificData& bulletData )
{
  coordPerElement= 2;

  BulletSoftBodyParameters* options= 
    dynamic_cast<BulletSoftBodyParameters*>(bodyParameters.getEngineOptions());

  const SoftBodyParameters::CoordList& coords= bodyParameters.getCoords();
  indices= bodyParameters.getIndices();

  btVector3* vertices= new btVector3 [ coords.size() ];
  for ( size_t i= 0; i < coords.size(); ++i ) {
    vertices[i]= tobtVector3 ( coords[i]*bulletData.m_worldScale );
  }

  softBody.reset ( 
    new btSoftBodyH3D (
    &bulletData.m_softBodyWorldInfo, 
    (int)coords.size(),
    vertices,
    NULL ) );
  delete [] vertices;

  // Add a 2nd default material which we will use
  // to implement soft body attachment links
  softBody->appendMaterial();

  // Link nodes to form rope/chain
  setEdgeOptions ( bodyParameters );
  initAddEdgeLinks ( softBody->m_nodes.size() );
  for ( size_t i= 1; i < indices.size(); ++i ) {
    addEdgeLink ( indices[i-1],indices[i] );
  }
  deinitAddEdgeLinks();

  if ( options ) {
    if ( options->haveBendingContraintDistance() ) {
      softBody->generateBendingConstraints ( options->getBendingContraintDistance() );
    }

    if ( options->haveNrClusters() ) {
      softBody->generateClusters ( options->getNrClusters() );
      nrClusters = options->getNrClusters();
    }
  } else {
    softBody->generateBendingConstraints(2);
    softBody->generateClusters(64);
  }

  softBody->getCollisionShape()->setMargin ( bulletData.m_collisionMargin * bulletData.m_worldScale );

  setParameters ( bodyParameters, false );
}

// Update bullet representation of soft body from specified parameters
void BulletRope::setParameters ( H3DSoftBodyNodeParameters& bodyParameters, bool updateGeometry ) {
  RopeParameters* params= dynamic_cast<RopeParameters*> ( &bodyParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(bodyParameters.getEngine()->getEngineSpecificData());

  if ( params ) {

    if ( updateGeometry && (params->haveIndices () || params->haveCoords ()) ) {
      // Link nodes to form faces
      setEdgeOptions ( bodyParameters );
      clearEdgeLinks();
      initAddEdgeLinks ( softBody->m_nodes.size() );
      for ( size_t i= 1; i < indices.size(); i+= 3 ) {
        addEdgeLink ( indices[i-1],indices[i] );
      }
      deinitAddEdgeLinks();
    }
  }

  H3DSoftBody::setParameters ( bodyParameters, updateGeometry );
}

void BulletRope::setMass ( H3DFloat _mass ) {
  softBody->setTotalMass ( _mass );
}

BulletRigidBodyAttachment::BulletRigidBodyAttachment ( H3DRigidBodyAttachmentParameters& attachmentParameters )
  : softBody ( NULL ),
  rigidBody ( NULL ) {
    setParameters ( attachmentParameters );
}

BulletRigidBodyAttachment::~BulletRigidBodyAttachment () {
  removeAnchors();
}

void BulletRigidBodyAttachment::setParameters ( H3DAttachmentParameters& attachmentParameters ) {
  H3DRigidBodyAttachmentParameters* params= 
    static_cast<H3DRigidBodyAttachmentParameters*> ( &attachmentParameters );
  BulletAttachment::setParameters ( attachmentParameters );

  if ( params->haveIndex() ) {
    indices = params->getIndex();
  }
  if ( params->haveBody1() ) {
    softBody= &((H3DSoftBody*)params->getBody1())->getSoftBody();
  }
  if ( params->haveBody2() ) {
    rigidBody= &((BulletRigidBody*)params->getBody2())->getRigidBody();
  }

  bool update_collide = false;
  if ( params->haveEngineOptions() ) {
    if ( BulletAttachmentParameters* options= dynamic_cast<BulletAttachmentParameters*>(params->getEngineOptions()) ) {

      if ( options->haveCollide() ) {
        collide= options->getCollide();
        update_collide = true;
      }
    }
  }

  if ( params->haveIndex() ||
    params->haveBody1() ||
    params->haveBody2() ) {
      // Remove any existing anchors
      removeAnchors();

      if ( softBody ) {
        // Add bullet anchors for each index in the attachment node
        for ( H3DRigidBodyAttachmentParameters::IndexList::const_iterator i= indices.begin(); 
          i != indices.end(); ++i ) {
            if ( *i >= 0 && *i < softBody->m_nodes.size() ) {
              // Add the anchor for this vertex
              softBody->appendAnchor ( *i, rigidBody, !collide );

              // Save the anchor for later removal
              addedAnchors.push_back ( softBody->m_anchors[softBody->m_anchors.size()-1] );
            }
        }
      }
  } else if( softBody && rigidBody && update_collide ) {
    if( collide ) {
      softBody->m_collisionDisabledObjects.remove( rigidBody );
    } else {
      if (softBody->m_collisionDisabledObjects.findLinearSearch(rigidBody)==softBody->m_collisionDisabledObjects.size())
      {
        softBody->m_collisionDisabledObjects.push_back(rigidBody);
      }
    }
  }
}

void BulletRigidBodyAttachment::remove () {
  removeAnchors();
}

void BulletRigidBodyAttachment::removeAnchors() {
  if ( softBody ) {
    for ( int i= 0; i < addedAnchors.size(); ++i ) {
      for ( int j= 0; j < softBody->m_anchors.size(); ++j ) {
        if ( addedAnchors[i].m_node == softBody->m_anchors[j].m_node &&
          addedAnchors[i].m_body== softBody->m_anchors[j].m_body ) {
            btSwap(softBody->m_anchors[j],softBody->m_anchors[softBody->m_anchors.size()-1]);
            softBody->m_anchors.pop_back();
            --j;
        }
      }
    }
  }
}

BulletSoftBodyAttachment::BulletSoftBodyAttachment ( SoftBodyAttachmentParameters& attachmentParameters )
  : softBody ( NULL ),
  softBody2 ( NULL ),
  h3dSoftBody ( NULL ),
  stiffnessUnitType ( PhysicsEngineParameters::MaterialPropertyParameters::UNIT_UNIFORM ) {

    setParameters ( attachmentParameters );
}

BulletSoftBodyAttachment::~BulletSoftBodyAttachment () {
  remove();
}

void BulletSoftBodyAttachment::update () {
  //for ( size_t i= 0; i < indices.size() && i < indices2.size(); ++i ) {
  //  btSoftBody::Node* n1= &softBody->m_nodes[indices[i]];
  //  btSoftBody::Node* n2= &softBody2->m_nodes[indices2[i]];

  //softBody->setMass ( indices[i], 0 );
  //softBody2->setMass ( indices2[i], 0 );
  //*n1 = *n2;
  //n1->m_v= n2->m_v;
  //}
}

void BulletSoftBodyAttachment::setParameters ( H3DAttachmentParameters& attachmentParameters ) {
  SoftBodyAttachmentParameters* params= 
    static_cast<SoftBodyAttachmentParameters*> ( &attachmentParameters );
  BulletAttachment::setParameters ( attachmentParameters );

  if ( params->haveIndex() ) {
    indices= params->getIndex();
  }
  if ( params->haveIndex2() ) {
    indices2= params->getIndex2();
  }
  if ( params->haveBody1() ) {
    h3dSoftBody= (H3DSoftBody*)params->getBody1();
    softBody= &h3dSoftBody->getSoftBody();
  }
  if ( params->haveBody2() ) {
    softBody2= &((H3DSoftBody*)params->getBody2())->getSoftBody();
  }

  if ( (params->haveIndex() || params->haveIndex2()) && softBody ) {
    const SoftBodyAttachmentParameters::EditVector& indexChanges= params->getIndexChanges ();
    const SoftBodyAttachmentParameters::EditVector& index2Changes= params->getIndex2Changes ();
    // If link indices have changed but there are no tracked changes the rebuild all
    if ( indexChanges.empty() ) {
      // Remove any existing links
      remove();
      // Add new links
      for ( size_t i= 0; i < indices.size() && i < indices2.size(); ++i ) {
        addLink ( i, indices[i], indices2[i] );
      }
    } else {
      // Indices have changed, and we have tracked changes, only update those that have changed
      for ( size_t i= 0; i < indexChanges.size(); ++i ) {
        SoftBodyAttachmentParameters::Edit edit= indexChanges[i];

        // Handle different types of edit
        switch ( edit.type ) {
          // Edit: Delete
        case SoftBodyAttachmentParameters::FieldType::Edit_Erase: {
          std::vector<H3DInt32> sorted ( edit.index );
          std::sort ( sorted.begin(), sorted.end() );

          size_t nrRemoved= 0;
          for ( std::vector<H3DInt32>::iterator j= sorted.begin(); j != sorted.end(); ++j ) {
            if ( removeLink ( *j-nrRemoved ) ) {
              ++nrRemoved;
            }
          }
                                                                  }
                                                                  break;
                                                                  // Edit: Insert
        case SoftBodyAttachmentParameters::FieldType::Edit_Insert: {
          // Must have a corresponding change in index2 field
          if ( i < index2Changes.size() ) {
            SoftBodyAttachmentParameters::Edit edit2= index2Changes[i];
            if ( edit2.type == edit.type && edit2.index.size() == edit.index.size() ) {
              for ( size_t j= 0; j < edit.index.size(); ++j )  {
                insertLink ( edit.index[j], edit.value[i], edit2.value[i] );
              }
            }
          }
                                                                   }
                                                                   break;
                                                                   // Edit: Update
        case SoftBodyAttachmentParameters::FieldType::Edit_Update: {
          // Must have a corresponding change in index2 field
          if ( i < index2Changes.size() ) {
            SoftBodyAttachmentParameters::Edit edit2= index2Changes[i];
            if ( edit2.type == edit.type && edit2.index.size() == edit.index.size() ) {
              for ( size_t j= 0; j < edit.index.size(); ++j )  {
                updateLink ( edit.index[j], edit.value[i], edit2.value[i] );
              }
            }
          }
                                                                   }
                                                                   break;
        }
      }
    }
  }

  // Update the physics material properties
  if ( params->havePhysicsMaterial() &&
    params->haveH3DPhysicsMaterialParameters()) {

      H3DPhysicsMaterialParameters* matPar= 
        params->getH3DPhysicsMaterialParameters();

      if( matPar->haveStiffness() &&
        matPar->haveStiffnessParameters() ) {
          applyStiffnessParameters ( *matPar->getStiffnessParameters() );
      }
  }
}

void BulletSoftBodyAttachment::remove () {
  if ( softBody ) {
    for ( int i= 0; i < softBody->m_links.size(); ++i ) {
      btSoftBody::Link &l = softBody->m_links[i];
      if( l.m_tag ) {
        H3DSoftBody::BulletSoftBodyTagData * tag_data =
          static_cast< H3DSoftBody::BulletSoftBodyTagData * >(l.m_tag);
        if( H3DSoftBody::LinkAttachmentTagData * link_tag_data =
          dynamic_cast< H3DSoftBody::LinkAttachmentTagData * >(tag_data) ) {
            if( link_tag_data->bullet_attachment == this ) {
              // Remove link_tag_data from node.
              if( l.m_n[1]->m_tag ) {
                list< H3DSoftBody::BulletSoftBodyTagData * > * tag_datas =
                  static_cast< list< H3DSoftBody::BulletSoftBodyTagData * > * >(
                  l.m_n[1]->m_tag );
                list< H3DSoftBody::BulletSoftBodyTagData * >::iterator m =
                  find( tag_datas->begin(), tag_datas->end(), link_tag_data );
                if( m != tag_datas->end() )
                  tag_datas->erase( m );
              }
              h3dSoftBody->removeLinkAndMaterial ( i );
              --i;
              delete link_tag_data;
            }
        }
      }
    }
  }
}

void BulletSoftBodyAttachment::addLink ( size_t index, size_t index1, size_t index2 ) {
  btSoftBody::Node* n1= &softBody->m_nodes[static_cast<int>(index1)];
  btSoftBody::Node* n2= &softBody2->m_nodes[static_cast<int>(index2)];
  // The material for the link
  int link_size_before = softBody->m_links.size();
  btSoftBody::Material* m= h3dSoftBody->getLinkMaterial ( link_size_before );
  softBody->appendLink ( n1, n2, m, false );
  if( softBody->m_links.size() > link_size_before ) {
    H3DSoftBody::LinkAttachmentTagData * link_tag_data =
      new H3DSoftBody::LinkAttachmentTagData( softBody,
      softBody->m_links.size()-1,
      softBody2,
      this, static_cast<int>(index) );
    list< H3DSoftBody::BulletSoftBodyTagData * > *tag_datas = NULL;
    if( n2->m_tag ) {
      tag_datas =
        static_cast< list< H3DSoftBody::BulletSoftBodyTagData * > * >
        ( n2->m_tag );
    } else {
      // This list pointer will be deleted when the SoftBody is destroyed.
      tag_datas = new list< H3DSoftBody::BulletSoftBodyTagData * >();
      n2->m_tag = tag_datas;
    }
    tag_datas->push_back( link_tag_data );
  }
}

void BulletSoftBodyAttachment::insertLink ( size_t index, size_t index1, size_t index2 ) {
  if ( softBody ) {
    // Shift indices
    for ( int i= 0; i < softBody->m_links.size(); ++i ) {
      btSoftBody::Link &l = softBody->m_links[i];
      if( l.m_tag ) {
        H3DSoftBody::BulletSoftBodyTagData * tag_data =
          static_cast< H3DSoftBody::BulletSoftBodyTagData * >(l.m_tag);
        if( H3DSoftBody::LinkAttachmentTagData * link_tag_data =
          dynamic_cast< H3DSoftBody::LinkAttachmentTagData * >(tag_data) ) {
            if ( link_tag_data->bullet_attachment == this ) {
              // Shift indices greater than insertion index to make space for new link
              if ( link_tag_data->attachment_link_index >= (int)index ) {
                // Update indices of other links greater than the removed index
                ++(link_tag_data->attachment_link_index);
              }
            }
        }
      }
    }
    // Add new link
    addLink ( index, index1, index2 );
  }
}

bool BulletSoftBodyAttachment::removeLink ( size_t index ) {
  bool removed= false;
  if ( softBody ) {
    for ( int i= 0; i < softBody->m_links.size(); ++i ) {
      btSoftBody::Link &l = softBody->m_links[i];
      if( l.m_tag ) {
        H3DSoftBody::BulletSoftBodyTagData * tag_data =
          static_cast< H3DSoftBody::BulletSoftBodyTagData * >(l.m_tag);
        if( H3DSoftBody::LinkAttachmentTagData * link_tag_data =
          dynamic_cast< H3DSoftBody::LinkAttachmentTagData * >(tag_data) ) {
            if ( link_tag_data->bullet_attachment == this ) {
              // If this link matches our index then remove it
              if ( link_tag_data->attachment_link_index == index ) {
                // Remove link_tag_data from node.
                if( l.m_n[1]->m_tag ) {
                  list< H3DSoftBody::BulletSoftBodyTagData * > * tag_datas =
                    static_cast< list< H3DSoftBody::BulletSoftBodyTagData * > * >(
                    l.m_n[1]->m_tag );
                  list< H3DSoftBody::BulletSoftBodyTagData * >::iterator m =
                    find( tag_datas->begin(), tag_datas->end(), link_tag_data );
                  if( m != tag_datas->end() )
                    tag_datas->erase( m );
                }
                h3dSoftBody->removeLinkAndMaterial ( i );
                --i;
                delete link_tag_data;
                removed= true;
              } else if ( link_tag_data->attachment_link_index > (int)index ) {
                // Update indices of other links greater than the removed index
                --(link_tag_data->attachment_link_index);
              }
            }
        }
      }
    }
  }

  return removed;
}

bool BulletSoftBodyAttachment::updateLink ( size_t index, size_t index1, size_t index2 ) {
  if ( softBody ) {
    if ( index1 < (size_t)softBody->m_nodes.size() && index2 < (size_t)softBody2->m_nodes.size() ) {
      for ( int i= 0; i < softBody->m_links.size(); ++i ) {
        btSoftBody::Link &l = softBody->m_links[i];
        if( l.m_tag ) {
          H3DSoftBody::BulletSoftBodyTagData * tag_data =
            static_cast< H3DSoftBody::BulletSoftBodyTagData * >(l.m_tag);
          if( H3DSoftBody::LinkAttachmentTagData * link_tag_data =
            dynamic_cast< H3DSoftBody::LinkAttachmentTagData * >(tag_data) ) {
              if ( link_tag_data->bullet_attachment == this ) {
                // If this link matches our index then remove it
                if ( link_tag_data->attachment_link_index == index ) {

                  l.m_n[0]= &softBody->m_nodes[static_cast<int>(index1)];
                  l.m_n[1]= &softBody2->m_nodes[static_cast<int>(index2)];
                  l.m_rl      =  (l.m_n[0]->m_x-l.m_n[1]->m_x).length();
                  //softBody->m_bUpdateRtCst= true;

                  return true;
                }
              }
          }
        }
      }
    }
  }

  return false;
}

void BulletSoftBodyAttachment::applyStiffnessParameters ( StiffnessParameters& stiffnessParams ) {
  if ( stiffnessParams.haveUnitType() ) {
    stiffnessUnitType= stiffnessParams.getUnitType();
  }

  switch ( stiffnessUnitType ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    // Use same stiffness value for all links
    setUniformStiffness ( stiffnessParams.getValuePerUnit(0) );
    break;
  case MaterialPropertyParameters::UNIT_EDGE:
    // Apply different stiffness each edge
    setStiffnessPerLink ( stiffnessParams );
    break;
  default:
    Console(4) << "Warning: Stiffness unit type is not supported!" << endl;
  };
}

void BulletSoftBodyAttachment::setUniformStiffness ( H3DFloat stiffness ) {
  // A material to share between all links of the attachment
  btSoftBody::Material* sharedMaterial= NULL;
  if ( softBody ) {
    for ( int i= 0; i < softBody->m_links.size(); ++i ) {
      btSoftBody::Link &l = softBody->m_links[i];
      if( l.m_tag ) {
        H3DSoftBody::BulletSoftBodyTagData * tag_data =
          static_cast< H3DSoftBody::BulletSoftBodyTagData * >(l.m_tag);
        if( H3DSoftBody::LinkAttachmentTagData * link_tag_data =
          dynamic_cast< H3DSoftBody::LinkAttachmentTagData * >(tag_data) ) {
            if ( link_tag_data->bullet_attachment == this ) {
              if ( !sharedMaterial ) {
                sharedMaterial= h3dSoftBody->getLinkMaterial ( i );
                sharedMaterial->m_kLST= stiffness;
                sharedMaterial->m_kVST= stiffness;
                sharedMaterial->m_kAST= stiffness;
              }
              l.m_material= sharedMaterial;

              // update the stiffness of the link
              l.m_c0= (l.m_n[0]->m_im+l.m_n[1]->m_im)/l.m_material->m_kLST;
            }
        }
      }
    }
  }
}

void BulletSoftBodyAttachment::setStiffnessPerLink ( StiffnessParameters& stiffnessParams ) {
  const StiffnessParameters::EditVector& changes= stiffnessParams.getChanges();
  if ( changes.empty() ) {
    // Update all
    for ( size_t i= 0; i < indices.size() && i < indices2.size(); ++i ) {
      setLinkStiffness ( i, stiffnessParams.getValuePerUnit(i) );
    }
  } else {
    // Update only selected links
    for ( StiffnessParameters::EditVector::const_iterator i= changes.begin(); i != changes.end(); ++i ) {
      StiffnessParameters::Edit edit= *i;
      if ( edit.type == StiffnessParameters::FieldType::Edit_Update ) {
        for ( size_t j= 0; j < edit.index.size(); ++j ) {
          if ( !setLinkStiffness ( edit.index[j], edit.value[j] ) ) {
            Console(4) << "Warning: Failed to set link stiffness index=" << edit.index[j] << endl;
          }
        }
      }
    }
  }
}

bool BulletSoftBodyAttachment::setLinkStiffness ( size_t index, H3DFloat stiffness ) {
  if ( softBody ) {
    for ( int i= 0; i < softBody->m_links.size(); ++i ) {
      btSoftBody::Link &l = softBody->m_links[i];
      if( l.m_tag ) {
        H3DSoftBody::BulletSoftBodyTagData * tag_data =
          static_cast< H3DSoftBody::BulletSoftBodyTagData * >(l.m_tag);
        if( H3DSoftBody::LinkAttachmentTagData * link_tag_data =
          dynamic_cast< H3DSoftBody::LinkAttachmentTagData * >(tag_data) ) {
            if ( link_tag_data->bullet_attachment == this ) {
              // If this link matches our index then update its stiffness
              if ( link_tag_data->attachment_link_index == index ) {
                btSoftBody::Material* m= h3dSoftBody->getLinkMaterial ( i );

                m->m_kLST= stiffness;
                m->m_kVST= stiffness;
                m->m_kAST= stiffness;
                l.m_material= m;

                // update the stiffness of the link
                l.m_c0= (l.m_n[0]->m_im+l.m_n[1]->m_im)/l.m_material->m_kLST;

                return true;
              }
            }
        }
      }
    }
  }
  return false;
}

BulletFixedConstraint::BulletFixedConstraint ( PhysicsEngineParameters::FixedConstraintParameters& fixedConstraintParameters ) :
softBody ( NULL ) {
  setParameters ( fixedConstraintParameters );
}

void BulletFixedConstraint::setParameters ( PhysicsEngineParameters::FixedConstraintParameters& fixedConstraintParameters ) {
  if ( fixedConstraintParameters.haveIndex() ) {
    indices= fixedConstraintParameters.getIndex();
  }
  if ( fixedConstraintParameters.haveBody1() ) {
    softBody= (BulletSoftBody*)fixedConstraintParameters.getBody1();
  }
}

const PhysicsEngineParameters::FixedConstraintParameters::IndexList& BulletFixedConstraint::getIndices () {
  return indices;
}

BulletSoftBody* BulletFixedConstraint::getSoftBody () {
  return softBody;
}

static bool customContactCallback(btManifoldPoint& cp,  
  const btCollisionObject* colObj0,
  int partId0,
  int index0,
  const btCollisionObject* colObj1,
  int partId1,
  int index1)
{
  PhysicsEngineThread& physics_thread= 
    static_cast<BulletRigidBody*>( colObj0->getUserPointer() )->getPhysicsThread();

  bool calculating_friction = false;

  // Apply global parameters for contacts
  PhysicsEngineParameters::GlobalContactParameters p= physics_thread.getGlobalContactParameters(); 
  for( vector< string >::iterator i = p.applied_parameters.begin();
    i != p.applied_parameters.end(); ++i )
  {
    string a = (*i);
    if ( a == "BOUNCE" )
    {
      // The bounce field value is used.
      cp.m_combinedRestitution = p.bounce;
    }
    //else if ( a == "USER_FRICTION" )
    //{
    //  // The system will normally calculate the friction direction vector that is perpendicular to the contact normal. 
    //  // This setting indicates that the user-supplied value in this contact should be used.
    //  //cp.m_lateralFrictionDir1= tobtVector3 ( p.friction_direction1 );
    //  //cp.m_lateralFrictionDir2= tobtVector3 ( p.friction_direction2 );
    //  //cp.m_lateralFrictionInitialized= true;
    //}
    //else if ( a == "FRICTION_COEFFICIENT?2" )
    //{

    //}
    //else if ( a == "ERROR_REDUCTION" )
    //{
    //  // The softnessErrorCorrection field value in the contact evaluation should be used.
    //}
    else if ( a == "CONSTANT_FORCE" )
    {
      // The softnessConstantForceMix field value in the contact evaluation should be used.
      //cp.m_contactCFM1= p.softness_constant_force_mix;
      //cp.m_contactCFM2= p.softness_constant_force_mix;
    }
    else if ( a == "SPEED-1" )
    {
      // The surfaceSpeed field value first component is used.
      cp.m_contactMotion1= p.surface_speed.x;
    }
    else if ( a == "SPEED-2" )
    {
      // The surfaceSpeed field value second component is used.
      cp.m_contactMotion2= p.surface_speed.y;
    }
    //else if ( a == "SLIP-1" )
    //{
    //  // The slipFactors field value first component is used.
    //  
    //}
    //else if ( a == "SLIP-2" )
    //{
    //  // The slipFactors field value second component is used.

    //}
  }

  // Friction parameter 1 is always applied, in keeping with ODE implementation?
  cp.m_combinedFriction= p.friction_coefficients.x;

  return true; // calculating friction
}

// Recent versions of Bullet have changed the interface for the contact callback
// This wraps the old version of the callback so that it's easy to switch back and forth
#ifdef BULLET_HAVE_COLLISION_OBJECT_WRAPPER
static bool customContactCallbackWrapper ( 
  btManifoldPoint& cp,
  const btCollisionObjectWrapper* colObj0,
  int partId0,
  int index0,
  const btCollisionObjectWrapper* colObj1,
  int partId1,int index1 ) {
    return customContactCallback ( cp, colObj0->m_collisionObject, partId0, index0, colObj1->m_collisionObject, partId1, index1 );
}
#define customContactCallback customContactCallbackWrapper
#endif

void customInternalTickCallback (btDynamicsWorld *world, btScalar timeStep) {
  BulletCallbacks::BulletSpecificData* bullet_data = 
    static_cast <BulletCallbacks::BulletSpecificData *> (world->getWorldUserInfo());

  for( size_t i = 0; i < bullet_data->m_softBodies.size(); ++i ) {
    H3DSoftBody* sb = bullet_data->m_softBodies[i];
    btSoftBodyH3D& btSb = sb->getSoftBody();
    if( btSb.isActive() ) {

      // First save contact info, then solve as usual
      btSb.cluster_contacts.clear();
      btSb.cluster_contacts.reserve( btSb.m_joints.size() );

      for( int i = 0; i < btSb.m_joints.size(); ++i ) {
        const btSoftBody::Joint* j = btSb.m_joints[i];
        if( j->Type() == btSoftBody::Joint::eType::Contact ) {
          const btSoftBody::CJoint* c = static_cast <const btSoftBody::CJoint*> (j);
          if( !c->m_bodies[0].m_soft || !c->m_bodies[1].m_soft ) {

            PhysicsEngineParameters::ContactParameters p;

            // Get info from the rigid body
            const btSoftBody::Body& r = c->m_bodies[0].m_soft ? c->m_bodies[1] : c->m_bodies[0];
            BulletRigidBody* rb = static_cast<BulletRigidBody*>(r.m_collisionObject->getUserPointer());
            BulletCallbacks::BulletSpecificData* bullet_data =
              static_cast<BulletCallbacks::BulletSpecificData*>(rb->getPhysicsThread().getEngineSpecificData());

            p.body1_id = (H3DBodyId)sb;
            p.body2_id = (H3DBodyId)rb;

            p.geom2_id = (H3DCollidableId)rb->getCollidable( 0 );

            p.contact_normal = toVec3f( c->m_normal );
            p.position = toVec3f( c->m_rpos[0] / bullet_data->m_worldScale );

            btSb.cluster_contacts.push_back( p );
          }
        }
      }
    }
  }

}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::initEngine(void *data) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );

  BulletCallbacks::BulletSpecificData *bullet_data = 
    new BulletCallbacks::BulletSpecificData;
  pt->setEngineSpecificData( bullet_data );

  // collision configuration contains default setup for memory, 
  // collision setup
  bullet_data->m_collisionConfiguration.reset( new btSoftBodyRigidBodyCollisionConfiguration() );

  // use the default collision dispatcher. For parallel processing you
  // can use a different dispatcher (see Extras/BulletMultiThreaded)
  bullet_data->m_dispatcher.reset( new  btCollisionDispatcher(bullet_data->m_collisionConfiguration.get()));

  bullet_data->m_broadphase.reset( new btDbvtBroadphase() );

  // the default constraint solver. For parallel processing you can 
  // use a different solver (see Extras/BulletMultiThreaded)
  bullet_data->m_solver.reset( new btSequentialImpulseConstraintSolver );

  bullet_data->m_softBodySolver.reset( new btSoftBodySolverH3D );

  bullet_data->m_dynamicsWorld.reset(
    new btSoftRigidDynamicsWorld( bullet_data->m_dispatcher.get(),
      bullet_data->m_broadphase.get(),
      bullet_data->m_solver.get(),
      bullet_data->m_collisionConfiguration.get(),
      bullet_data->m_softBodySolver.get() ) );
  bullet_data->m_dynamicsWorld->setGravity( btVector3( 0, 0, 0 ) );

  bullet_data->m_dynamicsWorld->setInternalTickCallback( customInternalTickCallback, bullet_data );

  gContactAddedCallback = customContactCallback;

  // Soft body physics initialisation
  bullet_data->m_softBodyWorldInfo.m_dispatcher= bullet_data->m_dispatcher.get();
  bullet_data->m_softBodyWorldInfo.m_broadphase= bullet_data->m_broadphase.get();

  /// Set default soft body world parameters
  bullet_data->m_softBodyWorldInfo.air_density= (btScalar)1.2;
  bullet_data->m_softBodyWorldInfo.water_density=  0;
  bullet_data->m_softBodyWorldInfo.water_offset= 0;
  bullet_data->m_softBodyWorldInfo.water_normal= btVector3(0,0,0);
  bullet_data->m_softBodyWorldInfo.m_gravity.setValue(0,0,0);
  bullet_data->m_softBodyWorldInfo.water_density=  0;
  bullet_data->m_softBodyWorldInfo.water_offset= 0;
  bullet_data->m_softBodyWorldInfo.water_normal= btVector3(0,0,0);
  bullet_data->m_softBodyWorldInfo.m_gravity.setValue(0,0,0);
  bullet_data->m_softBodyWorldInfo.m_sparsesdf.Initialize();
  bullet_data->m_softBodyWorldInfo.m_sparsesdf.Reset();

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode BulletCallbacks::doSimulationSteps(void *data) {
  PhysicsEngineThread * physics_thread = 
    static_cast< PhysicsEngineThread * >( data );

  TimeStamp t;

  BulletSpecificData *bullet_data = 
    static_cast< BulletSpecificData * >(physics_thread->getEngineSpecificData());
  PhysicsEngineParameters::WorldParameters world_params = 
    physics_thread->getWorldParameters();

  if( world_params.getEnabled() ) {
    btScalar timeStep= physics_thread->getStepSize();
    if ( !world_params.getUseStaticTimeStep() && physics_thread->getUpdateRate() > 0 )
      timeStep= (btScalar)1.0/physics_thread->getUpdateRate();

    //Stepping the world:

    // To make sure timeStep < maxSubSteps * fixedTimeStep, otherwise you will be losing time:
    //btScalar maxSubsSteps = (btScalar)timeStep/(bullet_data->m_fixedTimeStep );
    //maxSubsSteps *= 1.3f;
    //bullet_data->m_dynamicsWorld->stepSimulation( timeStep, maxSubsSteps, bullet_data->m_fixedTimeStep );

    // To disable Bullet's subdivision of timestep and let you pass in a fixedTimestep (see default value or set fixedTimeStep with BulletWorldOptions) :
    //bullet_data->m_dynamicsWorld->stepSimulation(timeStep, 0);

    bullet_data->m_dynamicsWorld->stepSimulation(timeStep, world_params.getIterations(), bullet_data->m_fixedTimeStep); 

    for ( BulletSpecificData::BulletAttachmentVector::iterator i= bullet_data->m_attachments.begin();
      i != bullet_data->m_attachments.end(); ++i ) {
        BulletAttachment* a= *i;
        a->update ();
    }
  }

  physics_thread->setLastLoopTime( TimeStamp() - t );

  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode BulletCallbacks::synchroniseWithSceneGraph(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}
PeriodicThread::CallbackCode BulletCallbacks::setWorldParameters(void *data) {
  PhysicsEngineParameters::WorldParameters *params = 
    static_cast< PhysicsEngineParameters::WorldParameters *>( data );
  BulletSpecificData *bullet_data = 
    static_cast< BulletSpecificData * >(params->getEngine()->getEngineSpecificData());

  if( params->haveAutoDisable() || 
    params->haveDisableLinearSpeed() || 
    params->haveDisableAngularSpeed() ){
      list< H3DBodyId > bodies;
      params->getEngine()->getCurrentRigidBodies( bodies );
      for( list< H3DBodyId >::iterator i = bodies.begin(); 
        i != bodies.end(); ++i ) {
          PhysicsEngineParameters::RigidBodyParameters p;
          params->getEngine()->getRigidBodyParameters( *i, p );
          if( !p.getAutoDisable() ) {
            // set the thresholds for all rigid bodies that do not have auto_disable
            // set in its own local parameters.
            btRigidBody& rb= ((BulletRigidBody*)(*i))->getRigidBody();
            if( params->getAutoDisable() ) {
              rb.setSleepingThresholds( params->getDisableLinearSpeed(),
                params->getDisableAngularSpeed() );
              rb.setActivationState ( ACTIVE_TAG );
            } else {
              // auto disable is false both locally and globally. In Bullet we use threshold of 0
              // for that.
              rb.setSleepingThresholds( 0, 0 );
              rb.setActivationState ( DISABLE_DEACTIVATION );
            }
          }
      }
  }

  if( params->haveGravity() ) {
    bullet_data->m_gravity= tobtVector3 ( params->getGravity() );

    btVector3 g ( bullet_data->m_gravity*bullet_data->m_worldScale );
    bullet_data->m_dynamicsWorld->setGravity(g);
    bullet_data->m_softBodyWorldInfo.m_gravity= g;
  }

  /*
  if( params->haveConstantForceMix() )
  dWorldSetCFM(bullet_data->world_id, params->getConstantForceMix() );


  if( params->haveContactSurfaceThickness() )
  dWorldSetContactSurfaceLayer(bullet_data->world_id, 
  params->getContactSurfaceThickness() );
  }

  if( params->haveErrorCorrection() ) {
  dWorldSetERP(bullet_data->world_id, params->getErrorCorrection() );
  }

  if( params->haveIterations() ) {
  dWorldSetQuickStepNumIterations( bullet_data->world_id, 
  params->getIterations() );
  }

  if( params->haveMaxCorrectionSpeed() ) {
  dWorldSetContactMaxCorrectingVel( bullet_data->world_id,
  params->getMaxCorrectionSpeed() );
  }
  */

  if ( params->haveEngineOptions() ) {
    if ( BulletWorldParameters* options= dynamic_cast<BulletWorldParameters*>(params->getEngineOptions()) ) {

      if ( options->haveWorldScale() ) {
        bullet_data->m_worldScale= options->getWorldScale();

        btVector3 g ( bullet_data->m_gravity*bullet_data->m_worldScale );
        bullet_data->m_dynamicsWorld->setGravity(g);
        bullet_data->m_softBodyWorldInfo.m_gravity= g;
      }

      if ( options->haveFixedTimeStep() ) {
        bullet_data->m_fixedTimeStep= options->getFixedTimeStep();
      }

      if ( options->haveCollisionMargin() ) {
        bullet_data->m_collisionMargin= options->getCollisionMargin();
      }

      if( options->haveMaxVelocityLinear() ) {
        bullet_data->m_maxVelocityLinear = options->getMaxVelocityLinear();
      }

      if( options->haveMaxVelocityAngular() ) {
        bullet_data->m_maxVelocityAngular = options->getMaxVelocityAngular();
      }
    }
  }

  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::setRigidBodyParameters(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );

  // Update rigid body from parameters
  ((BulletRigidBody*)params->getBodyId())->setParameters ( *params );

  delete params;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::setCollidableParameters(void *data) {
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );

  // Apply parameters to bullet collidable
  ((BulletCollidable*)params->getCollidableId())->setParameters ( *params );

  delete params;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::addCollidable(void *data)
{
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );

  // Create collidable shape
  if ( ShapeParameters *p = dynamic_cast< ShapeParameters* >( params ) )
    p->setCollidableId( (H3DCollidableId)new BulletCollidableShape ( *p ) ) ;

  // Create collidable offset
  else if ( OffsetParameters *p = dynamic_cast< OffsetParameters* >( params ) )
    p->setCollidableId( (H3DCollidableId)new BulletCollidableOffset ( *p ) );

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::addRigidBody(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  BulletSpecificData *bullet_data = 
    static_cast< BulletSpecificData * >(params->getEngine()->getEngineSpecificData());

  // Create rigid body
  params->setBodyId( (H3DBodyId)new BulletRigidBody ( *params ) );

  // Body is added later in setRigidBodyParameters(), no need to add twice

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::getCollidableParameters(void *data) {
  PhysicsEngineParameters::ShapeParameters *params = 
    static_cast< PhysicsEngineParameters::ShapeParameters *>( data );
  BulletSpecificData *bullet_data = 
    static_cast< BulletSpecificData * >(params->getEngine()->getEngineSpecificData());

  /*
  const dReal *pos = dGeomGetPosition( ( dGeomID )params->shape_id );
  const dReal *R   = dGeomGetRotation( ( dGeomID )params->shape_id );
  Rotation rot = Rotation(Matrix3f((H3DFloat)R[0], (H3DFloat)R[4], (H3DFloat)R[8], 
  (H3DFloat)R[1], (H3DFloat)R[5], (H3DFloat)R[9], 
  (H3DFloat)R[2], (H3DFloat)R[6], (H3DFloat)R[10]));  
  params->translation = Vec3f((H3DFloat)pos[0], (H3DFloat)pos[1], (H3DFloat)pos[2]);
  params->rotation = -rot;
  */
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::addConstraint(void *data) {


  PhysicsEngineParameters::ConstraintParameters *params = 
    static_cast< PhysicsEngineParameters::ConstraintParameters *>( data );

  // Create joint
  if ( JointParameters *j = dynamic_cast< JointParameters* >( params ) )
  {
    // Create joint based on type name
    BulletJoint* joint= BulletJoint::createJoint ( *j );
    j->setConstraintId( (PhysicsEngineParameters::H3DConstraintId)joint ) ;

    if( joint )
    {
      // Set joint parameters and add joint to simulation
      joint->setParameters ( *j );
      joint->add ();  
    }
    else
      // Joint creation failed
      Console ( 3 ) << "Warning: Joint type " << j->getType() << " not supported by Bullet!" << endl;

  }
  // Create attachments
  else if (  H3DAttachmentParameters *a = dynamic_cast< H3DAttachmentParameters* >( params ) )
  {
    BulletSpecificData* bulletData= 
      static_cast<BulletSpecificData*>( a->getEngine()->getEngineSpecificData() );

    // Create attachment
    BulletAttachment* attachment= NULL;

    // If required alternative types of attachment would be handled here
    if ( H3DRigidBodyAttachmentParameters* attParams= dynamic_cast<H3DRigidBodyAttachmentParameters*>(a) ) {
      attachment= new BulletRigidBodyAttachment ( *attParams );
    } else if ( SoftBodyAttachmentParameters* attParams= dynamic_cast<SoftBodyAttachmentParameters*>(a) ) {
      attachment= new BulletSoftBodyAttachment ( *attParams );
    }
    a->setConstraintId ( (H3DConstraintId)attachment );
  }
  else if ( VertexBodyConstraintParameters *vbc = dynamic_cast< VertexBodyConstraintParameters* >( params ) )
  {
    // Other vertex body constraint types can be handled here.
    if ( FixedConstraintParameters *fc = dynamic_cast< FixedConstraintParameters* >( vbc ) ) {
      BulletFixedConstraint* fixedConstraint= new BulletFixedConstraint ( *fc );
      if( fc->getBody1() ) {
        H3DSoftBody* sb= (H3DSoftBody*)fc->getBody1();
        sb->addFixedConstraint ( *fixedConstraint );
        sb->applyFixedConstraints();
      }
      vbc->setConstraintId ( (H3DConstraintId)fixedConstraint );
    }
  }
  else
    Console ( 3 ) << "Warning: Constraint type " << params->getType() << " not supported by Bullet!" << endl;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::getCurrentContacts(void *data) {

  PROFILE_BEGIN ( getCurrentContacts );

  typedef pair< list< PhysicsEngineParameters::ContactParameters  >*,
    PhysicsEngineThread * > InputType;
  InputType *params = 
    static_cast< InputType *>( data );

  BulletSpecificData *bullet_data = 
    static_cast< BulletSpecificData * >(params->second->getEngineSpecificData());

  // Rigid-rigid contacts
  unsigned int numManifolds = bullet_data->m_dynamicsWorld->getDispatcher()->getNumManifolds();
  for (unsigned int i=0;i<numManifolds;++i) {
    btPersistentManifold* contactManifold = bullet_data->m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
    const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

    int numContacts = contactManifold->getNumContacts();
    for (int j=0;j<numContacts;++j) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      PhysicsEngineParameters::ContactParameters p;
      const btVector3& pt_b = pt.getPositionWorldOnB();
      const btVector3& normal_on_b = pt.m_normalWorldOnB;
      p.position = Vec3f( pt_b[0], pt_b[1], pt_b[2] )/bullet_data->m_worldScale;
      p.contact_normal = Vec3f( normal_on_b[0], normal_on_b[1], normal_on_b[2] );
      p.depth = pt.getDistance()/bullet_data->m_worldScale;

      // Set rigid bodies and geometries involved in collision
      BulletRigidBody* rigidBodyA= static_cast<BulletRigidBody*>(obA->getUserPointer());
      BulletRigidBody* rigidBodyB= static_cast<BulletRigidBody*>(obB->getUserPointer());

      // Get body IDs involved in collision
      p.body1_id= (H3DBodyId)rigidBodyA;
      p.body2_id= (H3DBodyId)rigidBodyB;

      // Get geometry IDs involved in collision

      // In the case of triangle meshes, the index refers to the colliding triangle
      // not the index of the child shape. So there is no way to identify the child shape
      // so assume only one geometry for this body, and return that.
      // http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=4796&view=next

      if ( pt.m_partId0 == -1 ) {
        p.geom1_id= (H3DCollidableId)rigidBodyA->getCollidable ( pt.m_index0 );
      } else {
        // This is a tri-mesh, just return first geometry
        p.geom1_id= (H3DCollidableId)rigidBodyA->getCollidable ( 0 );
      }

      if ( pt.m_partId1 == -1 ) {
        p.geom2_id= (H3DCollidableId)rigidBodyB->getCollidable ( pt.m_index1 );
      } else {
        // This is a tri-mesh, just return first geometry
        p.geom2_id= (H3DCollidableId)rigidBodyB->getCollidable ( 0 );
      }

      params->first->push_back( p );   
    }
  }




  // Soft body contacts
  for ( BulletSpecificData::H3DSoftBodyVector::iterator i= bullet_data->m_softBodies.begin(); 
    i != bullet_data->m_softBodies.end(); ++i ) {
      H3DSoftBody* sb= *i;
      btSoftBodyH3D& btSb= sb->getSoftBody();

      // Rigid-soft contacts
      for ( int i= 0; i < btSb.m_rcontacts.size(); ++i ) {
        btSoftBody::RContact c= btSb.m_rcontacts[i];
        PhysicsEngineParameters::ContactParameters p;

        p.contact_normal= toVec3f(c.m_cti.m_normal);
        p.position= toVec3f(c.m_node->m_x)/bullet_data->m_worldScale;

        BulletRigidBody* rb= static_cast<BulletRigidBody*>(c.m_cti.m_colObj->getUserPointer());

        p.body1_id= (H3DBodyId)sb;
        p.body2_id= (H3DBodyId)rb;

        // For now return the first collidable
        p.geom2_id= (H3DCollidableId)rb->getCollidable ( 0 );

        params->first->push_back( p ); 
      }
      // Soft-soft contacts
      for ( int i= 0; i < btSb.m_scontacts.size(); ++i ) {

        btSoftBody::SContact c= btSb.m_scontacts[i];
        PhysicsEngineParameters::ContactParameters p;

        p.contact_normal= toVec3f(c.m_face->m_normal);
        p.position= toVec3f(c.m_node->m_x)/bullet_data->m_worldScale;

        p.body1_id= (H3DBodyId)sb;

        params->first->push_back( p ); 
      }

      // Cluster collisions
      params->first->insert( params->first->end(), btSb.cluster_contacts.begin(), btSb.cluster_contacts.end() );
  }

  PROFILE_END ();

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::getRigidBodyParameters(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );

  ((BulletRigidBody*)params->getBodyId())->getParameters ( *params );

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::addGlobalExternalForceAndTorque(void *data) {
  PhysicsEngineParameters::ExternalForceTorqueParameters *params = 
    static_cast< PhysicsEngineParameters::ExternalForceTorqueParameters *>( data );

  // Update bullet rigid body with forces
  ((BulletRigidBody*)params->body_id)->addGlobalExternalForceAndTorque ( *params );

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::setConstraintParameters(void *data) 
{
  PhysicsEngineParameters::ConstraintParameters *params = 
    static_cast< PhysicsEngineParameters::ConstraintParameters *>( data );

  // Set joint parameters
  if ( JointParameters *j = dynamic_cast< JointParameters* >( params ) )
  {    
    BulletJoint* joint= (BulletJoint*)j->getConstraintId();
    if ( joint )
      joint->setParameters ( *j );
  }
  // Set attachment parameters
  else if ( H3DAttachmentParameters *a = dynamic_cast< H3DAttachmentParameters* >( params ) )
  {
    // Set attachment parameters
    ((BulletAttachment*)a->getConstraintId())->setParameters ( *a );    
  }
  else if ( VertexBodyConstraintParameters *vbc = dynamic_cast< VertexBodyConstraintParameters* >( params ) )
  {
    // Other vertex body constraint types can be handled here.
    if ( FixedConstraintParameters *fc = dynamic_cast< FixedConstraintParameters* >( vbc ) ) {
      BulletFixedConstraint* fixedConstraint= (BulletFixedConstraint*)fc->getConstraintId();
      if ( fixedConstraint ) {

        // The body has changed
        if ( fc->haveBody1() ) {
          H3DSoftBody* sb= (H3DSoftBody*)fc->getBody1();

          // Remove from old body
          fixedConstraint->getSoftBody()->removeFixedConstraint ( *fixedConstraint );
          fixedConstraint->getSoftBody()->applyFixedConstraints();

          // Update parameters (e.g. fixed indices)
          fixedConstraint->setParameters ( *fc );

          // Add to new body
          sb->addFixedConstraint ( *fixedConstraint );
          sb->applyFixedConstraints();
        } else {
          // Only update the indices, body remains the same
          fixedConstraint->setParameters ( *fc );

          // Reapply to soft body
          fixedConstraint->getSoftBody()->applyFixedConstraints();
        }
      }
    }
  }

  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::getConstraintParameters(void *data) 
{ 
  PhysicsEngineParameters::ConstraintParameters *params = 
    static_cast< PhysicsEngineParameters::ConstraintParameters *>( data );

  // Get joint parameters
  if ( JointParameters *j = dynamic_cast< JointParameters* >( params ) )
  {
    BulletJoint* joint= (BulletJoint*)j->getConstraintId();
    if ( joint )
      joint->getParameters ( *j );
  }
  // Get attachment parameters
  else if ( H3DAttachmentParameters *a = dynamic_cast< H3DAttachmentParameters* >( params ) )
  {
    // Get attachment parameters
    ((BulletAttachment*)a->getConstraintId())->getParameters ( *a );
  }
  else if ( VertexBodyConstraintParameters *vbc = dynamic_cast< VertexBodyConstraintParameters* >( params ) )
  {
    // Other vertex body constraint types can be handled here.
    if ( FixedConstraintParameters *fc = dynamic_cast< FixedConstraintParameters* >( vbc ) )
    {
      if( fc->haveBody1() )
        fc->setIndex( ((H3DSoftBody*)fc->getBody1())->getFixedVertices() );        
    }
  }


  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::deInitEngine(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::removeConstraint(void *data) {

  PhysicsEngineParameters::ConstraintParameters *params = 
    static_cast< PhysicsEngineParameters::ConstraintParameters *>( data );

  // Remove joint. 
  if ( JointParameters *j = dynamic_cast< JointParameters* >( params ) )
  {

    BulletSpecificData* bullet_data = 
      static_cast<BulletSpecificData*>(j->getEngine()->getEngineSpecificData());

    if ( j->getConstraintId() )
    {
      BulletJoint* joint= (BulletJoint*)j->getConstraintId();

      // Remove joint from simulation and delete
      joint->remove ();

      delete joint;
    }
  }
  // Remove attachment.
  else if ( H3DAttachmentParameters *a = dynamic_cast< H3DAttachmentParameters* >( params ) )
  {
    // remove attachment

    delete (BulletAttachment*)params->getConstraintId();
  }
  else if ( VertexBodyConstraintParameters *vbc = dynamic_cast< VertexBodyConstraintParameters* >( params ) )
  {
    // Other vertex body constraint types can be handled here.
    if ( FixedConstraintParameters *fc = dynamic_cast< FixedConstraintParameters* >( vbc ) ) {
      BulletFixedConstraint* fixedConstraint= (BulletFixedConstraint*)fc->getConstraintId();

      if( fixedConstraint->getSoftBody() ) {
        fixedConstraint->getSoftBody()->removeFixedConstraint ( *fixedConstraint );
        fixedConstraint->getSoftBody()->applyFixedConstraints ();
      }

      delete fixedConstraint;
    }
  }

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::removeCollidable(void *data) {
  ShapeParameters *params = static_cast< ShapeParameters * >( data ); 
  BulletCollidable* bulletCollidable= (BulletCollidable*)params->getCollidableId();

  delete bulletCollidable;
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::removeRigidBody(void *data) {
  RigidBodyParameters *params = static_cast< RigidBodyParameters * >( data ); 

  BulletSpecificData *bullet_data = 
    static_cast< BulletSpecificData * >(params->getEngine()->getEngineSpecificData());

  // Remove body from world and delete
  BulletRigidBody* body = (BulletRigidBody*)params->getBodyId();
  bullet_data->m_dynamicsWorld->removeRigidBody( &body->getRigidBody() );

  delete body;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::addSpace(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::removeSpace(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::setSpaceParameters(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode BulletCallbacks::getSpaceParameters(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode BulletCallbacks::addSoftBody ( void *data ) {
  PhysicsEngineParameters::H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );
  BulletSpecificData* bulletData= 
    static_cast<BulletSpecificData*>( params->getEngine()->getEngineSpecificData() );

  // Create soft body
  H3DSoftBody* softBody= NULL;
  if ( SoftBodyParameters* bodyParams= dynamic_cast<SoftBodyParameters*>(params) ) {
    softBody= new BulletSoftBody ( *bodyParams, *bulletData );
  } else 
    if ( ClothParameters* bodyParams= dynamic_cast<ClothParameters*>(params) ) {
      softBody= new BulletCloth ( *bodyParams, *bulletData );
    } else
      if  ( RopeParameters* bodyParams= dynamic_cast<RopeParameters*>(params) ) {
        softBody= new BulletRope ( *bodyParams, *bulletData );
      }

      if ( softBody ) {
        params->setBodyId ( (H3DBodyId)softBody );

        // Add body to simulation
        bulletData->m_dynamicsWorld->addSoftBody( &softBody->getSoftBody() );
        bulletData->m_softBodies.push_back ( softBody );
      }

      return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode BulletCallbacks::removeSoftBody ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );
  BulletSpecificData* bulletData= 
    static_cast<BulletSpecificData*>( params->getEngine()->getEngineSpecificData() );

  H3DSoftBody* softBody= (H3DSoftBody*)params->getBodyId();

  // Remove body from simulation
  bulletData->m_softBodies.erase ( find ( bulletData->m_softBodies.begin(),
    bulletData->m_softBodies.end(), softBody ) );
  bulletData->m_dynamicsWorld->removeSoftBody ( &softBody->getSoftBody() );

  // Delete the soft body implementation
  delete softBody;

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode BulletCallbacks::setSoftBodyParameters ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );

  // Set soft body parameters
  ((H3DSoftBody*)params->getBodyId())->setParameters ( *params );

  delete params;

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode BulletCallbacks::getSoftBodyParameters ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );

  // Get soft body parameters
  ((H3DSoftBody*)params->getBodyId())->getParameters ( *params );

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode BulletCallbacks::applyExternalForces ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );

  // Get soft body parameters
  ((H3DSoftBody*)params->getBodyId())->applyExternalForces ( *params );

  return PeriodicThread::CALLBACK_DONE;
}

void btSoftBodySolverH3D::processCollision( btSoftBody* softBody, const btCollisionObjectWrapper* collisionObjectWrap ) {
  static_cast<btSoftBodyH3D*>(softBody)->filteredCollisionHandler( collisionObjectWrap );
}

void btSoftBodyH3D::filteredCollisionHandler( const btCollisionObjectWrapper * pcoWrap ) {

  BulletRigidBody* rb = static_cast<BulletRigidBody*>(pcoWrap->getCollisionObject()->getUserPointer());
  
  if(
    // If both objects are in the default collision group (0), then they will collide
    (!rb->collisionGroup && !collisionGroup) ||

    // For other collision groups, check the collision masks
    ((rb->collidesWith & collisionGroup) && (collidesWith & rb->collisionGroup)) ) {

    // Check the soft body collision flags
    int collisions = m_cfg.collisions & fCollision::RVSmask;

    // Check the rigid body soft collision flags, these override if present

    if( rb->collisionSoft ) {
      collisions = rb->collisionSoft;
    }

    switch( collisions ) {
      case fCollision::SDF_RS: {
        btSoftColliders::CollideSDF_RS docollide;
        btRigidBody* prb1 = (btRigidBody*)btRigidBody::upcast( pcoWrap->getCollisionObject() );
        btTransform wtr = pcoWrap->getWorldTransform();

        const btTransform ctr = pcoWrap->getWorldTransform();
        const btScalar timemargin = (wtr.getOrigin() - ctr.getOrigin()).length();
        const btScalar basemargin = getCollisionShape()->getMargin();
        btVector3 mins;
        btVector3 maxs;
        ATTRIBUTE_ALIGNED16( btDbvtVolume ) volume;
        pcoWrap->getCollisionShape()->getAabb( pcoWrap->getWorldTransform(),
                                               mins,
                                               maxs );
        volume = btDbvtVolume::FromMM( mins, maxs );
        volume.Expand( btVector3( basemargin, basemargin, basemargin ) );
        docollide.psb = this;
        docollide.m_colObj1Wrap = pcoWrap;
        docollide.m_rigidBody = prb1;

        docollide.dynmargin = basemargin + timemargin;
        docollide.stamargin = basemargin;
        m_ndbvt.collideTV( m_ndbvt.m_root, volume, docollide );
      }
      break;
      case fCollision::CL_RS: {
        btSoftColliders::CollideCL_RS collider;
        collider.ProcessColObj( this, pcoWrap );
      }
      break;
    }
  }
}

BulletCallbacks::BulletSpecificData::~BulletSpecificData() {
  m_softBodyWorldInfo.m_sparsesdf.Reset();
}

#endif // HAVE_BULLET
