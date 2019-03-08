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
/// \file SOFACallbacks.cpp
/// \brief Source file for SOFACallbacks, struct containing callbacks and
/// variables for connecting to the Open Dynamics Engine.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/SOFACallbacks.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>

#include <H3D/Box.h>
#include <H3D/Sphere.h>
#include <H3D/Shape.h>
#include <H3D/Cylinder.h>

#ifdef HAVE_SOFA

//#define SOFA_HAVE_GLEW

#include <sofa/simulation/tree/GNode.h>
#include <sofa/component/collision/cubemodel.h>
#include <sofa/helper/Quater.h>
//#include <sofa/component/typedef/Sofa_typedef.h>
#include <sofa/component/container/MechanicalObject.h>
#include <sofa/component/topology/MeshTopology.h>
#include "sofa/component/collision/TriangleModel.h"
#include "sofa/component/collision/LineModel.h"
#include "sofa/component/collision/PointModel.h"
#include "sofa/component/collision/SphereModel.h"

#define SOFA_DEBUG_PRINTOUTS

using namespace H3D;
using namespace sofa::simulation::tree;
using sofa::component::odesolver::EulerSolver;
using sofa::core::objectmodel::Data;
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::core::VecId;

PhysicsEngineThread::PhysicsEngineRegistration 
  SOFACallbacks::registration( "SOFA", 
  PhysicsEngineThread::createPhysicsEngineCallbacks< SOFACallbacks >() );

PeriodicThread::CallbackCode SOFACallbacks::initEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );
  // dInitSOFA(); Engine initialization is done in RigidBodyPhysics.cpp
  SOFACallbacks::SOFASpecificData *sofa_data = 
    new SOFACallbacks::SOFASpecificData;
  pt->setEngineSpecificData( sofa_data );

  sofa::simulation::setSimulation(new sofa::simulation::tree::TreeSimulation());

  sofa::component::init();
  sofa::simulation::xml::initXml();  

  sofa_data->simulationPtr = sofa::simulation::getSimulation();

  string fileName = "";

  sofa_data->scene_root = NULL;
  //    dynamic_cast<sofa::simulation::Node*>( sofa::simulation::getSimulation()->load(fileName.c_str()));

  if (sofa_data->scene_root==NULL) {
    sofa_data->scene_root = sofa_data->simulationPtr->createNewGraph("SOFA_scene_root");
    sofa_data->simulationPtr->init(sofa_data->scene_root.get());
  } 
  //  scene_root->setGravity(sofa::core::objectmodel::BaseContext::Vec3( 0, -10, 0 ) );
  //scene_root->setGravityInWorld( Coord3(0,-10,0) );
  //sofa_data->scene_root->setAnimate(true);


  // One solver for all the graph
  EulerSolver::SPtr solver = sofa::core::objectmodel::New<EulerSolver>();
  solver->setName("solver");
  solver->f_printLog.setValue(false);
  sofa_data->scene_root->addObject(solver);

#ifdef SOFA_DEBUG_PRINTOUTS
  Console(4) << "SOFA: init engine "  << endl;
#endif
  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode SOFACallbacks::deInitEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );
  SOFACallbacks::SOFASpecificData *sofa_data = 
    static_cast< SOFACallbacks::SOFASpecificData *>( pt->getEngineSpecificData() );

  if (sofa_data->scene_root!=NULL)
    sofa::simulation::getSimulation()->unload(sofa_data->scene_root);
#ifdef SOFA_DEBUG_PRINTOUTS
  Console(4) << "SOFA: deinit engine "  << endl;
#endif

  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode SOFACallbacks::doSimulationSteps(void *data) {
  PhysicsEngineThread * physics_thread = 
    static_cast< PhysicsEngineThread * >( data );

  TimeStamp t;

  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(physics_thread->getEngineSpecificData());

  PhysicsEngineParameters::WorldParameters world_params = 
    physics_thread->getWorldParameters();

  PhysicsEngineParameters::GlobalContactParameters contact_params = 
    physics_thread->getGlobalContactParameters(); 

  // only step if RigidBodyCollection is enabled.
  if( world_params.getEnabled() ) {
    void *a = (void *)  sofa::simulation::tree::getSimulation();
    if (sofa_data->simulationPtr && sofa_data->scene_root->getAnimate())
    {
      //Console(4) << a << " " << sofa_data->scene_root << " " << physics_thread->getStepSize()  << endl;
      sofa_data->simulationPtr->animate ( sofa_data->scene_root.get(), 0.01 );
    }
    else
      Console(4) << "Hmm, could not get hold of SOFA simulation " << physics_thread->getStepSize()  << endl;
  }

  physics_thread->setLastLoopTime( TimeStamp() - t );

  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode SOFACallbacks::synchroniseWithSceneGraph(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode SOFACallbacks::setWorldParameters(void *data) {
  PhysicsEngineParameters::WorldParameters *params = static_cast< PhysicsEngineParameters::WorldParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  if( params->haveAutoDisable() || 
    params->haveDisableLinearSpeed() || 
    params->haveDisableAngularSpeed() ){
      /// Adapt to SOFA
      //list< H3DBodyId > bodies;
      //params->engine_thread->getCurrentRigidBodies( bodies );
      //for( list< H3DBodyId >::iterator i = bodies.begin(); 
      //  i != bodies.end(); ++i ) {
      //    PhysicsEngineParameters::RigidBodyParameters p;
      //    params->engine_thread->getRigidBodyParameters( *i, &p );
      //    if( !p.getAutoDisable() ) {
      //      // set the thresholds for all rigid bodies that do not have auto_disable
      //      // set in its own local parameters.
      //      if( params->getAutoDisable() ) {
      //        ((BulletRigidBody*)(*i))->getRigidBody().setSleepingThresholds( params->getDisableLinearSpeed(),
      //          params->getDisableAngularSpeed() );
      //      } else {
      //        // auto disable is false both locally and globally. In Bullet we use threshold of 0
      //        // for that.
      //        ((BulletRigidBody*)(*i))->getRigidBody().setSleepingThresholds( 0, 0 );
      //      }
      //    }
      //}
  }

  // haveGravity
  if( params->haveGravity() ) {
    H3D::Vec3f g = params->getGravity();
    sofa_data->scene_root->setGravity(sofa::core::objectmodel::BaseContext::Vec3( g.x, g.y, g.z ) );
  }

  delete params;
  return PeriodicThread::CALLBACK_DONE;
}




H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::setRigidBodyParameters(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  //// Create rigid body
  sofa_data->scene_root->setAnimate(false);
  sofa_data->simulationPtr->updateContext(sofa_data->scene_root.get());
  // Update rigid body from parameters
  ((SOFARigidBody*)params->getBodyId())->setParameters ( *params );
  sofa_data->scene_root->setAnimate(true);
  sofa_data->simulationPtr->updateContext(sofa_data->scene_root.get());
  sofa_data->simulationPtr->init(sofa_data->scene_root.get());

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::getRigidBodyParameters(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );

  ((SOFARigidBody*)params->getBodyId())->getParameters ( *params );

  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::setCollidableParameters(void *data) {
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );

  // Apply parameters to SOFA collidable
  ((SOFACollidable*)params->getCollidableId())->setParameters ( *params );

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::addCollidable(void *data)
{
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );

  // Create collidable shape
  if ( ShapeParameters *p = dynamic_cast< ShapeParameters* >( params ) )
    p->setCollidableId((H3DCollidableId)new SOFACollidableShape ( *p ));

  // Create collidable offset
  else if ( OffsetParameters *p = dynamic_cast< OffsetParameters* >( params ) )
    p->setCollidableId((H3DCollidableId)new SOFACollidableOffset ( *p ));

  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::addRigidBody(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  // Create rigid body
  sofa_data->scene_root->setAnimate(false);
  sofa_data->simulationPtr->updateContext(sofa_data->scene_root.get());

  SOFARigidBody* rb= new SOFARigidBody ( *params );
  params->setBodyId((H3DBodyId)rb);
  sofa_data->scene_root->setAnimate(true);
  sofa_data->simulationPtr->updateContext(sofa_data->scene_root.get());

  // Add rigid body to simulation
  //sofa_data->scene_root->addChild ( rb->getNode() );

  sofa_data->simulationPtr->init(sofa_data->scene_root.get());

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::getCollidableParameters(void *data) {
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::addGlobalExternalForceAndTorque(void *data) {
  PhysicsEngineParameters::ExternalForceTorqueParameters *params = 
    static_cast< PhysicsEngineParameters::ExternalForceTorqueParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->engine_thread->getEngineSpecificData());
  /*

  // Check if autodisable disabled the body, if it did then enable it again.
  RigidBodyParameters rbp;  // RB object keyed from X3D file
  params->engine_thread->getRigidBodyParameters(params->body_id, &rbp);
  if( !dBodyIsEnabled( (dBodyID)params->body_id ) &&
  params->force.length() > Constants::f_epsilon &&
  !rbp.fixed && rbp.enabled ) {   // make sure it's not 'fixed' and 'enabled'
  dBodyEnable( (dBodyID)params->body_id );
  }
  dBodyAddForceAtPos( ( dBodyID ) params->body_id, 
  params->force.x,
  params->force.y,
  params->force.z,
  params->position.x,
  params->position.y,
  params->position.z );
  dBodyAddTorque( ( dBodyID )params->body_id, 
  params->torque.x,
  params->torque.y,
  params->torque.z );

  */
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::addJoint(void *data) {
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  Console ( 3 ) << "Warning: Joint type " << params->getType() << " not implement in SOFACallback!" << endl;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::getCurrentContacts(void *data) {
  typedef pair< list< PhysicsEngineParameters::ContactParameters  >*,
    PhysicsEngineThread * > InputType;
  InputType *params = 
    static_cast< InputType *>( data );

  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->second->getEngineSpecificData());
  /*
  // Loop over collected SOFA contacts and create ContactParameters
  for( vector<dContact>::iterator i= sofa_data->allContacts.begin(); 
  i != sofa_data->allContacts.end(); ++i ) {
  const dContact &c = *i;

  PhysicsEngineParameters::ContactParameters p;

  p.position = Vec3f((H3DFloat) c.geom.pos[0],(H3DFloat)c.geom.pos[1],(H3DFloat)c.geom.pos[2] );
  p.contact_normal = Vec3f( (H3DFloat)c.geom.normal[0],(H3DFloat)c.geom.normal[1],(H3DFloat)c.geom.normal[2] );
  p.depth = (H3DFloat)c.geom.depth;
  p.geom1_id = ( H3DUInt64 )c.geom.g1;
  p.geom2_id = ( H3DUInt64 )c.geom.g2;
  p.body1_id = ( H3DUInt64 )dGeomGetBody( c.geom.g1 );
  p.body2_id = ( H3DUInt64 )dGeomGetBody( c.geom.g2 );

  p.bounce = (H3DFloat)c.surface.bounce;
  p.min_bounce_speed =(H3DFloat) c.surface.bounce_vel;
  p.softness_error_correction = (H3DFloat)c.surface.soft_erp;
  p.softness_constant_force_mix = (H3DFloat)c.surface.soft_cfm;
  p.friction_coefficients = Vec2f( (H3DFloat)c.surface.mu, (H3DFloat)c.surface.mu2 );
  p.slip_coefficients = Vec2f( (H3DFloat)c.surface.slip1, (H3DFloat)c.surface.slip2 );
  p.surface_speed = Vec2f( (H3DFloat)c.surface.motion1, (H3DFloat)c.surface.motion2 );

  p.applied_parameters.clear();

  if( c.surface.mode & dContactBounce ) p.applied_parameters.push_back( "BOUNCE" );
  if( c.surface.mode & dContactFDir1 ) p.applied_parameters.push_back( "USER_FRICTION" );
  if( c.surface.mode & dContactMu2 ) p.applied_parameters.push_back( "FRICTION_COEFFICIENT-2");
  if( c.surface.mode & dContactSoftERP ) p.applied_parameters.push_back("ERROR_CORRECTION" );
  if( c.surface.mode & dContactSoftCFM ) p.applied_parameters.push_back( "CONSTANT_FORCE" );
  if( c.surface.mode & dContactMotion1 ) p.applied_parameters.push_back( "SPEED-1" );
  if( c.surface.mode & dContactMotion2 ) p.applied_parameters.push_back( "SPEED-2" );
  if( c.surface.mode & dContactSlip1 ) p.applied_parameters.push_back( "SLIP-1" );
  if( c.surface.mode & dContactSlip2 ) p.applied_parameters.push_back( "SLIP-2" );

  p.friction_direction = Vec3f( (H3DFloat) c.fdir1[0], (H3DFloat)c.fdir1[1],(H3DFloat) c.fdir1[2] );

  params->first->push_back( p );
  }
  */
  //cerr << "Contacts: "  << sofa_data->current_nr_contacts << endl;
  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::getJointParameters(void *data) {  
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );

  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::setJointParameters(void *data) {  
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );

  return PeriodicThread::CALLBACK_DONE;
}


PeriodicThread::CallbackCode SOFACallbacks::removeCollidable( void *data ) {
  CollidableParameters *params = static_cast< CollidableParameters * >( data );
  SOFACollidable* sofaCollidable= (SOFACollidable*)params->getCollidableId();
  delete sofaCollidable;
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode SOFACallbacks::removeRigidBody( void *data ) {
  RigidBodyParameters *params = static_cast< RigidBodyParameters * >( data ); 

  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  // Remove body from world and delete
  SOFARigidBody* body = (SOFARigidBody*)params->getBodyId();
  //sofa_data->m_dynamicsWorld->removeRigidBody( &body->getRigidBody() );
  delete body;

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode SOFACallbacks::removeJoint( void *data ) {
  JointParameters *params = static_cast< JointParameters * >( data ); 

  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode SOFACallbacks::addSpace(void *data) {
  PhysicsEngineParameters::SpaceParameters *params = 
    static_cast< PhysicsEngineParameters::SpaceParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode SOFACallbacks::removeSpace(void *data) {
  H3DSpaceId *id = static_cast< H3DSpaceId * >( data );
  //  dSpaceDestroy( ( dSpaceID )*id );
  //cerr << "SOFA: dSpaceDestroy called for SpaceID " << ( dSpaceID )*id << endl;
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode SOFACallbacks::setSpaceParameters(void *data) {
  PhysicsEngineParameters::SpaceParameters *params = 
    static_cast< PhysicsEngineParameters::SpaceParameters *>( data );
  SOFASpecificData *sofa_data = 
    static_cast< SOFASpecificData * >(params->getEngine()->getEngineSpecificData());

  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode SOFACallbacks::getSpaceParameters(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

// SOFA Rigid Body Implementation

SOFARigidBody::SOFARigidBody ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters ) :
physicsThread ( bodyParameters.getEngine() ),
  massDensityModel ( NULL )
{
  SOFACallbacks::SOFASpecificData *sofa_data = 
    static_cast< SOFACallbacks::SOFASpecificData * >(physicsThread->getEngineSpecificData());

  // just create an empty rigid body. Parameters will be set in 
  // setRigidBodyParameters

  // rigid body
  rigidBody = sofa::core::objectmodel::New<GNode>();
  sofa_data->scene_root->addChild ( rigidBody );
  rigidBody->setName("rigidBodyNode");

  rigidDOF= sofa::core::objectmodel::New<MechanicalObjectRigid3>();
  rigidDOF->setName ( "MechanicalObject" );
  rigidDOF->setTranslation(0.0,0.0,0.0);
  rigidDOF->setRotation(0.0,0.0,0.0);
  rigidBody->addObject ( rigidDOF );

  rigidMass= sofa::core::objectmodel::New<UniformMassRigid3>();
  rigidMass->setTotalMass(0.5);
  //rigidMass->setMass(inertiaMatrix);
  rigidBody->addObject(rigidMass);

  // make sure the gContactAddedCallback will be called.
  //rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() |
  //  btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );

  // Set a back-pointer to this object from the rigid body implementation
  // Makes it possible to access this object whenever bullet returns our btRigidBody
  // e.g. In collision detection.
  //rigidBody->setUserPointer( this );

  // Set rigid body parameters
  setParameters( bodyParameters );





}

void SOFARigidBody::addGlobalExternalForceAndTorque ( PhysicsEngineParameters::ExternalForceTorqueParameters& forces ) {
  Console(4) << "SOFARigidBody::addGlobalExternalForceAndTorque" << endl;
}

void SOFARigidBody::setParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters ) {
  Console(4) << "SOFARigidBody::setParameters" << endl;

  //SOFACallbacks::SOFASpecificData* sofaData= static_cast<SOFACallbacks::SOFASpecificData*> ( bodyParameters.engine_thread->getEngineSpecificData() );
  // degrees of freedom
  //rigidDOF= sofa::core::objectmodel::New<MechanicalObjectRigid3>();
  //rigidDOF->setName ( "MechanicalObject" );
  //rigidDOF->setTranslation(0.0,0.0,0.0);
  //rigidDOF->setRotation(0.0,0.0,0.0);
  //rigidBody->addObject ( rigidDOF );

  //  rigidDOF->resize(1);

  //const VecCoordRigid3& rigid_x = *rigidDOF->getX ();
  //rigid_x[0]= CoordRigid3 ( Coord3 ( 0, 0, 0 ),
  //                          Quat3::identity() );


  //rigidMass= sofa::core::objectmodel::New<UniformMassRigid3>();
  //rigidMass->setTotalMass(0.5);
  ////rigidMass->setMass(inertiaMatrix);
  //rigidBody->addObject(rigidMass);

  // mass
  /*rigidMass= new UniformMassRigid3;
  rigidBody->addObject ( rigidMass );
  rigidMass->setName ( "M2" );
  UniformMassRigid3::MassType* m= rigidMass->mass.beginEdit();
  m->mass= 0.3;
  UniformMassRigid3::MassType::Mat3x3 inertia;
  inertia.fill(0.0);
  float in= 0.1;
  inertia[0][0]= in;
  inertia[1][1]= in;
  inertia[2][2]= in ;
  m->inertiaMatrix= inertia;
  m->recalc();
  rigidMass->mass.endEdit ();*/

  // fixed and mass properties. We also let enabled behave the same way as
  // fixed, i.e. if a body is not enabled we think of it as fixed.
  if( bodyParameters.haveFixed() || 
    bodyParameters.haveEnabled() || 
    bodyParameters.haveMassDensityModel() ||
    bodyParameters.haveMass() ) {
      // enabled
      rigidBody->setActive( bodyParameters.getEnabled() && !bodyParameters.getFixed() );
      
      if( bodyParameters.getFixed() || !bodyParameters.getEnabled() ) {
        rigidMass->setTotalMass(0);
        //rigidBody->setMassProps( 0, btVector3( 0, 0, 0 ) );
      } else {
        // not fixed, set mass properties.
        if ( bodyParameters.haveMassDensityModel() ) {
          massDensityModel= bodyParameters.getMassDensityModel();
        }
        if( !massDensityModel ) {
          //btBoxShape box( btVector3( (btScalar)0.1, 
          //  (btScalar)0.1, 
          //  (btScalar)0.1 ) );
          //btVector3 local_inertia( 0, 0, 0 );
          //box.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
          //rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );
        } else {
          rigidMass->setTotalMass(bodyParameters.getMass());
     //     if (Box *b = dynamic_cast< Box* >( massDensityModel ) ){
     //       Vec3f half_size = bullet_data->m_worldScale*b->size->getValue()/2;
     //       btBoxShape geom( btVector3( half_size.x, half_size.y, half_size.z ) );
     //       btVector3 local_inertia( 0, 0, 0 );
     //       geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
     //       rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
     //     } else if (Sphere *sph = dynamic_cast< Sphere * >( massDensityModel ) ){
     //       H3DFloat radius = bullet_data->m_worldScale*sph->radius->getValue();
     //       btSphereShape geom( radius );
     //       btVector3 local_inertia( 0, 0, 0 );
     //       geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
     //       rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );
     //     } else if (Cone *cone = dynamic_cast< Cone * >( massDensityModel ) ){
     //       H3DFloat radius = bullet_data->m_worldScale*cone->bottomRadius->getValue();
     //       H3DFloat height = bullet_data->m_worldScale*cone->height->getValue();
     //       btConeShape geom( radius, height );
     //       btVector3 local_inertia( 0, 0, 0 );
     //       geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
     //       rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
     //     } else if (Cylinder *cyl = dynamic_cast< Cylinder * >( massDensityModel ) ){
     //       H3DFloat radius = bullet_data->m_worldScale*cyl->radius->getValue();
     //       H3DFloat height = bullet_data->m_worldScale*cyl->height->getValue();
     //       btCylinderShape geom( btVector3( radius, height/2, radius ) );
     //       btVector3 local_inertia( 0, 0, 0 );
     //       geom.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
     //       rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
          //} else {
     //       // we have an unsupported mass density model, use the bounding box of it as
     //       // density model
     //       Console(4) << "Invalid mass density model \"" << massDensityModel->getTypeName() << "\". " 
     //         << "Using bound of node as density model instead." << endl;
     //       X3DGeometryNode *g = static_cast< X3DGeometryNode * >( massDensityModel );
     //       BoxBound *bb = dynamic_cast< BoxBound * >( g->bound->getValue() );
     //       Vec3f half_size( 0.1f, 0.1f, 0.1f );
     //       if( bb ) half_size = bullet_data->m_worldScale*bb->size->getValue();

     //       btBoxShape box( btVector3( half_size.x, half_size.y, half_size.z ) );
     //       btVector3 local_inertia( 0, 0, 0 );
     //       box.calculateLocalInertia( bodyParameters.getMass(), local_inertia);
     //       rigidBody->setMassProps( bodyParameters.getMass(), local_inertia );  
     //     }
        }
      }
  }

  // useGlobalGravity
  if( bodyParameters.haveUseGlobalGravity() ) {
    if( bodyParameters.getUseGlobalGravity() ) {
      PhysicsEngineParameters::WorldParameters world_params = 
        bodyParameters.getEngine()->getWorldParameters();
      const Vec3f &g = world_params.getGravity();
      rigidBody->setGravity( sofa::core::objectmodel::BaseContext::Vec3( g.x, g.y, g.z ));
    } else {
      rigidBody->setGravity( sofa::core::objectmodel::BaseContext::Vec3( 0, 0, 0 ) );
    }
  }

  // disable values
  if( bodyParameters.haveAutoDisable() || 
    bodyParameters.haveDisableLinearSpeed() || 
    bodyParameters.haveDisableAngularSpeed() ){
  //  if( bodyParameters.getAutoDisable() ) {
  //    rigidBody->setSleepingThresholds( bodyParameters.getDisableLinearSpeed(),
  //                                      bodyParameters.getDisableAngularSpeed() );
  //    rigidBody->setActivationState ( ACTIVE_TAG );
  //  } else {
  //    PhysicsEngineParameters::WorldParameters world_params = 
  //      bodyParameters.getEngine()->getWorldParameters();
  //    if( world_params.getAutoDisable() ) {
  //      rigidBody->setSleepingThresholds( world_params.getDisableLinearSpeed(),
  //                                   world_params.getDisableAngularSpeed() );
  //      rigidBody->setActivationState ( ACTIVE_TAG );
  //    } else {
  //      PhysicsEngineParameters::WorldParameters world_params = 
  //        bodyParameters.getEngine()->getWorldParameters();
  //      if( world_params.getAutoDisable() ) {
  //        rigidBody->setSleepingThresholds( world_params.getDisableLinearSpeed(),
  //          world_params.getDisableAngularSpeed() );
  //          rigidBody->setActivationState ( ACTIVE_TAG );
  //      } else {
  //        // auto disable false everywhere, set to 0.
  //        rigidBody->setSleepingThresholds( 0, 0 );
  //        rigidBody->setActivationState ( DISABLE_DEACTIVATION );
  //      }
  //    }
  }

  // startPosition and startOrientation
  if( bodyParameters.haveStartPosition() || bodyParameters.haveStartOrientation() ) {
    //btTransform t = rigidBody->getCenterOfMassTransform();
    if( bodyParameters.haveStartOrientation() ) {
      //Quaternion q(bodyParameters.getStartOrientation() );
      //t.setRotation( btQuaternion( q.v.x, q.v.y, q.v.z, q.w ) );

      Rotation rot = bodyParameters.getStartOrientation();
      Vec3f euler = rot.toEulerAngles();
      rigidDOF->setRotation(euler.x ,euler.y, euler.z);
     Console(4) << "SOFARigidBody::setParameters Orientation" << endl;
    }

    if( bodyParameters.haveStartPosition() ) {
      Vec3f v = bodyParameters.getStartPosition();
      rigidDOF->setTranslation(v.x, v.y, v.z);
     Console(4) << "SOFARigidBody::setParameters Position" << endl;
    }
    //else
    //  rigidDOF->setTranslation(0.0,0.0,0.0);

    //Vec3f start_position = bodyParameters.getStartPosition(); 
    //Quaternion start_rotation ( bodyParameters.getStartOrientation() );

    //WriteAccessor< Data<MechanicalObjectRigid3::VecCoord> > positions = *(rigidDOF)->write( VecId::position() );
    //positions[0] = sofa::defaulttype::RigidCoord< 3, float>(sofa::defaulttype::Vec3f( start_position.x, start_position.y, start_position.z ),
    //  sofa::defaulttype::Quat( start_rotation.v.x, start_rotation.v.y, start_rotation.v.z, start_rotation.w ) );    


    //rigidBody->setCenterOfMassTransform( t );
  }

  // startLinearVelocity 
  if( bodyParameters.haveStartLinearVelocity() ) {
    const Vec3f &v = bodyParameters.getStartLinearVelocity();
    //rigidBody->setLinearVelocity( btVector3( v.x, v.y, v.z ) );
  }

  //  startAngularVelocity
  if( bodyParameters.haveStartAngularVelocity() ) {
    const Vec3f &v = bodyParameters.getStartAngularVelocity();
    //rigidBody->setAngularVelocity( btVector3( v.x, v.y, v.z ) );
  }

  // geometry
  if( bodyParameters.haveGeometry() ) {
    const vector<H3DCollidableId> geom_ids= bodyParameters.getGeometry();

    for ( SOFACollidableList::iterator i= collidables.begin(); 
      i != collidables.end(); ++i ) {
        (*i)->removeBody ( *this );
    }

    collidables.clear();

    for ( vector<H3DCollidableId>::const_iterator i= geom_ids.begin(); 
      i != geom_ids.end(); ++i ) {
        SOFACollidable* collidable= (SOFACollidable*)*i;
        collidables.push_back ( collidable );
        //collidable->addBody ( *this );
    }

    updateCollidables();
  }

}

void SOFARigidBody::getParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters ) {
  //const sofa::defaulttype::Vector3 &p= rigidDOF->getTranslation();

  const MechanicalObjectRigid3::VecCoord *fff = rigidDOF->getX();
  //sofa::defaulttype::Vector3 *fff = rigidDOF->getX();
  const sofa::core::objectmodel::Data<MechanicalObjectRigid3::VecCoord> *a = rigidDOF->read(  sofa::core::ConstVecCoordId::position() );
  const MechanicalObjectRigid3::VecCoord &p = a->getValue();
  //p[0].
  bodyParameters.setPosition ( H3DUtil::Vec3f ( p[0][0], p[0][1], p[0][2] ) );

  //const VecCoordRigid3& p= *rigidDOF->getX();
  //bodyParameters.setPosition ( H3DUtil::Vec3f ( p[0][0], p[0][1], p[0][2] ) );
}


void SOFARigidBody::updateCollidables()
{
  // No geometry specified
  if( collidables.empty() ) 
  {
    //collisionShape.reset ( NULL );
    //rigidBody->setCollisionShape( collisionShape.get() );
  } 
  else 
  {
    //// More than one collision shape required, use composite
    //btCompoundShape* compoundShape= new btCompoundShape ();

    //// For each specified geometry
    //for ( BulletCollidableList::const_iterator i= collidables.begin(); 
    //  i != collidables.end(); ++i ) {
    //    // Recurse down and add atomic collidable to the compound shape
    //    // The hierachy is flattened so that the index can be used to identify
    //    // the child uniquely during collision detection
    //    (*i)->addCollidableTo ( *compoundShape );
    //}

    //// Set the compound shape as the body's geometry
    //rigidBody->setCollisionShape( compoundShape );

    //// Ensure additional collision shape is deleted
    //collisionShape.reset ( compoundShape );

    //validateCollidables( *compoundShape );
  }

  collisionShapeChanged();
}

void SOFARigidBody::collisionShapeChanged ()
{
  SOFACallbacks::SOFASpecificData *sofa_data = 
    static_cast< SOFACallbacks::SOFASpecificData * >(physicsThread->getEngineSpecificData());

  //// Make sure that the rigid body is not already in the world
  //const btCollisionObjectArray &objects = bullet_data->m_dynamicsWorld->getCollisionObjectArray ();
  //if( objects.findLinearSearch(rigidBody.get())  != objects.size()) {
  //  bullet_data->m_dynamicsWorld->removeRigidBody(rigidBody.get());
  //}
  //bullet_data->m_dynamicsWorld->addRigidBody(rigidBody.get());
}

SOFACollidable::SOFACollidable ( PhysicsEngineParameters::CollidableParameters& collidableParameters ) 
  : enabled ( true ){
    SOFACallbacks::SOFASpecificData* sofa_data= static_cast<SOFACallbacks::SOFASpecificData*> ( collidableParameters.getEngine()->getEngineSpecificData() );

    collidableNode = sofa::core::objectmodel::New<GNode>();
    sofa_data->scene_root->addChild ( collidableNode );
    collidableNode->setName("collidableNode");

    localTransform= sofa::defaulttype::Mat4x4f::Mat();
    setParameters ( collidableParameters );
}

void SOFACollidable::addBody( SOFARigidBody& body )
{

}

void SOFACollidable::removeBody( SOFARigidBody& body )
{

}

void SOFACollidable::updateCollidables()
{

}


// Update sofa representation of collidable using specified parameters
void SOFACollidable::setParameters ( PhysicsEngineParameters::CollidableParameters& collidableParameters )
{
  /*  bool needUpdateCollidables= false;

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

  // If required, re-add the collidables hierachy to the rigid bodies
  if ( needUpdateCollidables ) {
  updateCollidables ();
  }*/
}


SOFACollidableOffset::SOFACollidableOffset( PhysicsEngineParameters::OffsetParameters& offsetParameters )
  : SOFACollidable ( offsetParameters )
{
}

void SOFACollidableOffset::addBody( SOFARigidBody& body )
{
}

void SOFACollidableOffset::removeBody( SOFARigidBody& body )
{
}

SOFACollidableShape::SOFACollidableShape( PhysicsEngineParameters::ShapeParameters& shapeParameters )
  : SOFACollidable(shapeParameters)
{

  // Create geometry
  if( shapeParameters.haveShape() )
    collidableShape = createCollisionShape ( shapeParameters.getShape() );
}

// Return an appropriate sofa::core::collisionModel to match the specified X3DGeometryNode
sofa::core::CollisionModel * SOFACollidableShape::createCollisionShape ( X3DGeometryNode * geometry )
{

  // SOFA has no special primitives for Box, Cylinder and Cone geometries.
  // There is one CollisionModel called CubeModel which does not work(!).
  // So the options are either an hierarchy of SphereModels to represent
  // shapes or to use a combination of PointModel, LineModel and
  // TriangleModel to capture collisions.
  if (const Box *b = dynamic_cast< const Box * >( geometry ) ){
    Vec3f half_size = b->size->getValue()/2;

    //sofa::simulation::Node::SPtr CollisionNode = parentNode->createChild("Collision");
    typedef sofa::component::container::MechanicalObject<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > MechanicalObject3;
    MechanicalObject3::SPtr cubeBox;
    cubeBox = sofa::core::objectmodel::New<MechanicalObject3>();
    collidableNode->addObject(cubeBox);
    cubeBox->resize(8);
    cubeBox->setName("MechanicalObjCube");
    //get write access to the position vector of mechanical object DOF 
    sofa::helper::WriteAccessor<Data<sofa::defaulttype::Vec3dTypes::VecCoord> > x = *cubeBox->write(VecId::position());

    x[0] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,-half_size.y,-half_size.z);
    x[1] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,-half_size.y,-half_size.z);
    x[2] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,half_size.y,-half_size.z);
    x[3] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,half_size.y,-half_size.z);
    x[4] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,-half_size.y,half_size.z);
    x[5] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,-half_size.y,half_size.z);
    x[6] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,half_size.y,half_size.z);
    x[7] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,half_size.y,half_size.z);

    // Add uniform mass, do we really need this here?
    typedef sofa::component::mass::UniformMass<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, double> UniformMass3;
    UniformMass3::SPtr mass = sofa::core::objectmodel::New<UniformMass3>();
    collidableNode->addObject(mass);
    mass->setMass(2);
    mass->setName("mass");

    // Tetrahedron topology
    sofa::component::topology::MeshTopology::SPtr topology = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
    topology->setName("mesh topology");
    collidableNode->addObject( topology );
    topology->addHexa(0,1,2,3,4,5,6,7);

    sofa::component::collision::TriangleModel::SPtr triMod = sofa::core::objectmodel::New<sofa::component::collision::TriangleModel>();
    triMod->setName("triangle CollisionShape");
    collidableNode->addObject(triMod);

    sofa::component::collision::LineModel::SPtr lineMod = sofa::core::objectmodel::New<sofa::component::collision::LineModel>();
    lineMod->setName("line CollisionShape");
    collidableNode->addObject(lineMod);

    sofa::component::collision::PointModel::SPtr pointMod = sofa::core::objectmodel::New<sofa::component::collision::PointModel>();
    pointMod->setName("point CollisionShape");
    collidableNode->addObject(pointMod);

    return NULL;
  }
  else if (const Sphere *sph = dynamic_cast< const Sphere * >( geometry ) ){
    H3DFloat radius = sph->radius->getValue();

    sofa::component::collision::SphereModel::SPtr sphere = sofa::core::objectmodel::New<sofa::component::collision::SphereModel>();

    return NULL;

    //} else if (const Cone *cone = dynamic_cast< const Cone * >( geometry ) ){
    //  H3DFloat radius = cone->bottomRadius->getValue();
    //  H3DFloat height = cone->height->getValue();


    //  return NULL;

    //} else if (const Cylinder *cyl = dynamic_cast< const Cylinder * >( geometry ) ){  
    //  H3DFloat radius = cyl->radius->getValue();
    //  H3DFloat height = cyl->height->getValue();

    //  return NULL;
  } else {
    // concave tri mesh
    vector< HAPI::Collision::Triangle > triangles;
    vector< HAPI::Collision::LineSegment > lines;
    vector< HAPI::Collision::Point > points;
    geometry->boundTree->getValue()->getAllPrimitives( triangles, lines, points);

    typedef sofa::component::container::MechanicalObject<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > MechanicalObject3;
    MechanicalObject3::SPtr cubeBox;
    cubeBox = sofa::core::objectmodel::New<MechanicalObject3>();
    collidableNode->addObject(cubeBox);
    cubeBox->resize(8);
    cubeBox->setName("MechanicalObjCube");
    //get write access to the position vector of mechanical object DOF 
    sofa::helper::WriteAccessor<Data<sofa::defaulttype::Vec3dTypes::VecCoord> > x = *cubeBox->write(VecId::position());

    //x[0] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,-half_size.y,-half_size.z);
    //x[1] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,-half_size.y,-half_size.z);
    //x[2] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,half_size.y,-half_size.z);
    //x[3] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,half_size.y,-half_size.z);
    //x[4] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,-half_size.y,half_size.z);
    //x[5] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,-half_size.y,half_size.z);
    //x[6] = sofa::defaulttype::Vec3dTypes::Coord(half_size.x,half_size.y,half_size.z);
    //x[7] = sofa::defaulttype::Vec3dTypes::Coord(-half_size.x,half_size.y,half_size.z);

    // Add uniform mass, do we really need this here?
    typedef sofa::component::mass::UniformMass<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, double> UniformMass3;
    UniformMass3::SPtr mass = sofa::core::objectmodel::New<UniformMass3>();
    collidableNode->addObject(mass);
    mass->setMass(2);
    mass->setName("mass");

    // Tetrahedron topology
    sofa::component::topology::MeshTopology::SPtr topology = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
    topology->setName("mesh topology");
    collidableNode->addObject( topology );
    topology->addHexa(0,1,2,3,4,5,6,7);

    // using 32 bit indices and 3 component vertices
    //btTriangleMesh *mesh = new btTriangleMesh( true, false );
    //mesh->preallocateVertices( triangles.size() * 3 );
    for( unsigned int i = 0; i < triangles.size(); ++i ) {

      //mesh->addTriangle( btVector3( (btScalar)triangles[i].a.x,
      //                              (btScalar)triangles[i].a.y,
      //                              (btScalar)triangles[i].a.z ),
      //                   btVector3( (btScalar)triangles[i].b.x,
      //                              (btScalar)triangles[i].b.y,
      //                              (btScalar)triangles[i].b.z ),
      //                   btVector3( (btScalar)triangles[i].c.x,
      //                              (btScalar)triangles[i].c.y,
      //                              (btScalar)triangles[i].c.z ) );
    }

    return NULL;
  }


}


#endif // HAVE_SOFA






