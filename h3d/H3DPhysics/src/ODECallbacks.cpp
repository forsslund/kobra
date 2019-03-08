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
/// \file ODECallbacks.cpp
/// \brief Source file for ODECallbacks, struct containing callbacks and
/// variables for connecting to the Open Dynamics Engine.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/ODECallbacks.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>

#include <H3D/Box.h>
#include <H3D/Sphere.h>
#include <H3D/Shape.h>
#include <H3D/Cylinder.h>

#ifdef HAVE_ODE

using namespace H3D;

PhysicsEngineThread::PhysicsEngineRegistration 
ODECallbacks::registration( "ODE", 
                            PhysicsEngineThread::createPhysicsEngineCallbacks< ODECallbacks >() );

PeriodicThread::CallbackCode ODECallbacks::initEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );
  // dInitODE(); Engine initialization is done in H3DPhysics.cpp
  ODECallbacks::ODESpecificData *ode_data = 
    new ODECallbacks::ODESpecificData;
  pt->setEngineSpecificData( ode_data );
  ode_data->world_id = dWorldCreate();
  ode_data->space_id = dHashSpaceCreate(0);
  ode_data->contact_group = dJointGroupCreate(0);
  //cerr << "ODE: dWorldCreate called, world created with ID " << ode_data->world_id << endl;
  //cerr << "ODE: dHashSpaceCreate called, space created with ID " << (H3DSpaceId) ode_data->space_id << endl;
  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode ODECallbacks::deInitEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );
  ODECallbacks::ODESpecificData *ode_data = 
      static_cast< ODECallbacks::ODESpecificData *>( pt->getEngineSpecificData() );
  dJointGroupDestroy( ode_data->contact_group );
  dSpaceDestroy( ode_data->space_id );
  //cerr << "ODE: dSpaceDestroy with ID " << (H3DSpaceId) ode_data->space_id << endl;
  dWorldDestroy( ode_data->world_id );
  //cerr << "ODE: dWorldDestroy called with ID " << ode_data->world_id << endl;
  // dCloseODE(); Engine clean-up is called in RigidBodyPhysics.cpp  
  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode ODECallbacks::doSimulationSteps(void *data) {
  PhysicsEngineThread * physics_thread = 
    static_cast< PhysicsEngineThread * >( data );

  TimeStamp t;

  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(physics_thread->getEngineSpecificData());
  
  PhysicsEngineParameters::WorldParameters world_params = 
    physics_thread->getWorldParameters();

  PhysicsEngineParameters::GlobalContactParameters contact_params = 
    physics_thread->getGlobalContactParameters(); 

  // only step if RigidBodyCollection is enabled.
  if( world_params.getEnabled() ) {

    // save the transforms of each triangle mesh for use later.
    vector< dReal > transforms(ode_data->tri_meshes.size() * 16);
  
    unsigned int index = 0;
    for( map< dGeomID, dGeomID >::iterator i = ode_data->tri_meshes.begin();
         i != ode_data->tri_meshes.end(); ++i, ++index ) {
      const dReal* pos = dGeomGetPosition( i->second );
      const dReal* rot = dGeomGetRotation( i->second );
      transforms[index*16+0] = rot[0];
      transforms[index*16+1] = rot[1];
      transforms[index*16+2] = rot[2];
      transforms[index*16+3] = 0;
      transforms[index*16+4] = rot[4];
      transforms[index*16+5] = rot[5];
      transforms[index*16+6] = rot[6];
      transforms[index*16+7] = 0;
      transforms[index*16+8] = rot[8];
      transforms[index*16+9] = rot[9];
      transforms[index*16+10] = rot[10];
      transforms[index*16+11] = 0;
      transforms[index*16+12] = pos[0];
      transforms[index*16+13] = pos[1];
      transforms[index+14] = pos[2];
      transforms[index+15] = 1;

      if( index == 0 ) {
        const dReal* real_t = dGeomTriMeshGetLastTransform( i->second );
      }
    }

    // perform collision detection
    ode_data->allContacts.clear();
    if( contact_params.collision_enabled ) {
      dSpaceCollide(ode_data->space_id, 
                    physics_thread, 
                    &ODECallbacks::nearCallback);
    }

    for( map< dJointID, dReal >::iterator i = ode_data->slider_joint_forces.begin();
        i != ode_data->slider_joint_forces.end(); ++i ) {
      dJointAddSliderForce( (*i).first, (*i).second );
    }

    // step the world
    if( world_params.getPreferAccuracy() ) 
      dWorldStep( ode_data->world_id, physics_thread->getStepSize() );
    else
      dWorldQuickStep( ode_data->world_id, physics_thread->getStepSize() );
  
    dJointGroupEmpty(ode_data->contact_group);

    // set the last transform for all tri meshes as per ODE recommendations.
    index = 0;
    for( map< dGeomID, dGeomID >::iterator i = ode_data->tri_meshes.begin();
         i != ode_data->tri_meshes.end(); ++i, ++index ) {
      dGeomTriMeshSetLastTransform( i->second, &transforms[index*16] );
    }
  }

  physics_thread->setLastLoopTime( TimeStamp() - t );

  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode ODECallbacks::synchroniseWithSceneGraph(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode ODECallbacks::setWorldParameters(void *data) {
  PhysicsEngineParameters::WorldParameters *params = static_cast< PhysicsEngineParameters::WorldParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  // autoDisable
  if( params->haveAutoDisable() ) 
    dWorldSetAutoDisableFlag(ode_data->world_id, params->getAutoDisable() );
  
  // constantForceMix
  if( params->haveConstantForceMix() )
    dWorldSetCFM(ode_data->world_id, params->getConstantForceMix() );

  // contactSurfaceThickness
  if( params->haveContactSurfaceThickness() ) {
    dWorldSetContactSurfaceLayer(ode_data->world_id, 
                                 params->getContactSurfaceThickness() );
  }

  // disableAngularSpeed
  if( params->haveDisableAngularSpeed() ) {
    dWorldSetAutoDisableAngularThreshold( ode_data->world_id,
                                          params->getDisableAngularSpeed() );
  }

  // diableLinearSpeed
  if( params->haveDisableLinearSpeed() ) {
    dWorldSetAutoDisableLinearThreshold( ode_data->world_id,
                                         params->getDisableLinearSpeed() );
  }

  // disableTime
  if( params->haveDisableTime() ) {
    dWorldSetAutoDisableTime( ode_data->world_id,
                              params->getDisableTime() );
  }

  // haveErrorCorrection
  if( params->haveErrorCorrection() ) {
    dWorldSetERP(ode_data->world_id, params->getErrorCorrection() );
  }

  // haveGravity
  if( params->haveGravity() ) {
    Vec3f g = params->getGravity();
    dWorldSetGravity(ode_data->world_id, g.x, g.y, g.z ); 
  }

  // iterations
  if( params->haveIterations() ) {
    dWorldSetQuickStepNumIterations( ode_data->world_id, 
                                     params->getIterations() );
  }
  
  // massCorrectionSpeed
  if( params->haveMaxCorrectionSpeed() ) {
    dWorldSetContactMaxCorrectingVel( ode_data->world_id,
                                      params->getMaxCorrectionSpeed() );
  }

  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

void ODECallbacks::nearCallback(void *data, dGeomID o1, dGeomID o2){

  PhysicsEngineThread * physics_thread = 
    static_cast< PhysicsEngineThread * >( data );  
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(physics_thread->getEngineSpecificData());

  if ( dGeomIsSpace(o1) || dGeomIsSpace(o2) ) {
      // check possible colliding space and geom and 
      // call nearCallback on each pair
      dSpaceCollide2 ( o1, o2, data, &nearCallback );
      // collide all geoms within a space
      if ( dGeomIsSpace(o1) ) dSpaceCollide ( (dSpaceID)o1, data, &nearCallback );
      if ( dGeomIsSpace(o2) ) dSpaceCollide ( (dSpaceID)o2, data, &nearCallback );
  } else {
      // colliding two non-space geoms
      // generate contacts between these two geometries and store
      // in ode_data->contacts
      int n = dCollide(o1, o2, 
                   ode_data->max_nr_contacts, 
                   &ode_data->contacts[0].geom,
                   sizeof(dContact));

      // add contacts to simulation
      PhysicsEngineParameters::GlobalContactParameters p = 
        physics_thread->getGlobalContactParameters(); 

      unsigned int mode = 0;
      
      if( n > 0 ) {
        for( vector< string >::iterator i = p.applied_parameters.begin();
            i != p.applied_parameters.end(); ++i ) {
          if( (*i) == "BOUNCE" ) mode |= dContactBounce;
          else if( (*i) == "FRICTION_COEFFICIENT-2" ) mode |= dContactMu2;
          else if( (*i) == "ERROR_CORRECTION" ) mode |= dContactSoftERP;
          else if( (*i) == "CONSTANT_FORCE" ) mode |= dContactSoftCFM;
          else if( (*i) == "SPEED-1" ) mode |= dContactMotion1;
          else if( (*i) == "SPEED-2" ) mode |= dContactMotion2;
          else if( (*i) == "SLIP-1" ) mode |= dContactSlip1;
          else if( (*i) == "SLIP-2" ) mode |= dContactSlip2;
        }
      }

      for (int i=0; i < n; ++i){
        ode_data->contacts[i].surface.mode = mode;
        ode_data->contacts[i].surface.bounce = p.bounce;
        ode_data->contacts[i].surface.bounce_vel = p.min_bounce_speed;
        ode_data->contacts[i].surface.soft_erp = p.softness_error_correction;
        ode_data->contacts[i].surface.soft_cfm = p.softness_constant_force_mix;
        ode_data->contacts[i].surface.mu = p.friction_coefficients.x;
        ode_data->contacts[i].surface.mu2 = p.friction_coefficients.y;
        ode_data->contacts[i].surface.slip1 = p.slip_factors.x;
        ode_data->contacts[i].surface.slip2 = p.slip_factors.y;
        ode_data->contacts[i].surface.motion1 = p.surface_speed.x;
        ode_data->contacts[i].surface.motion2 = p.surface_speed.y;

        dJointID c = dJointCreateContact(ode_data->world_id, 
                                        ode_data->contact_group, 
                                        &ode_data->contacts[i]);
        dJointAttach(c,
                    dGeomGetBody(ode_data->contacts[i].geom.g1), 
                    dGeomGetBody(ode_data->contacts[i].geom.g2));
        // copy the contact to the list of all contacts in this frame
        ode_data->allContacts.push_back ( ode_data->contacts[i] );
      }
  }
  
/*
  PhysicsEngineThread * physics_thread = 
    static_cast< PhysicsEngineThread * >( data );
  
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(physics_thread->getEngineSpecificData());

  int n = dCollide(o1, o2, 
                   ode_data->max_nr_contacts, &ode_data->contacts[0].geom, 
                   sizeof(dContact));

  ode_data->current_nr_contacts = n;

  PhysicsEngineParameters::GlobalContactParameters p = 
    physics_thread->getGlobalContactParameters(); 

  unsigned int mode = 0;
  
  if( n > 0 ) {
    for( vector< string >::iterator i = p.applied_parameters.begin();
         i != p.applied_parameters.end(); ++i ) {
      if( (*i) == "BOUNCE" ) mode |= dContactBounce;
      else if( (*i) == "FRICTION_COEFFICIENT-2" ) mode |= dContactMu2;
      else if( (*i) == "ERROR_CORRECTION" ) mode |= dContactSoftERP;
      else if( (*i) == "CONSTANT_FORCE" ) mode |= dContactSoftCFM;
      else if( (*i) == "SPEED-1" ) mode |= dContactMotion1;
      else if( (*i) == "SPEED-2" ) mode |= dContactMotion2;
      else if( (*i) == "SLIP-1" ) mode |= dContactSlip1;
      else if( (*i) == "SLIP-2" ) mode |= dContactSlip2;
    }
  }

  for (int i=0; i < n; ++i){
    ode_data->contacts[i].surface.mode = mode;
    ode_data->contacts[i].surface.bounce = p.bounce;
    ode_data->contacts[i].surface.bounce_vel = p.min_bounce_speed;
    ode_data->contacts[i].surface.soft_erp = p.softness_error_correction;
    ode_data->contacts[i].surface.soft_cfm = p.softness_constant_force_mix;
    ode_data->contacts[i].surface.mu = p.friction_coefficients.x;
    ode_data->contacts[i].surface.mu2 = p.friction_coefficients.y;
    ode_data->contacts[i].surface.slip1 = p.slip_factors.x;
    ode_data->contacts[i].surface.slip2 = p.slip_factors.y;
    ode_data->contacts[i].surface.motion1 = p.surface_speed.x;
    ode_data->contacts[i].surface.motion2 = p.surface_speed.y;

    dJointID c = dJointCreateContact(ode_data->world_id, 
                                     ode_data->contact_group, 
                                     &ode_data->contacts[i]);
    dJointAttach(c,
                 dGeomGetBody(ode_data->contacts[i].geom.g1), 
                 dGeomGetBody(ode_data->contacts[i].geom.g2));
  }
  */
}

void ODECallbacks::setRigidBodyParameters( PhysicsEngineParameters::RigidBodyParameters *params,
                                           ODESpecificData *ode_data ) {
  dBodyID body = ( dBodyID )params->getBodyId();

   // autoDisable
  if( params->haveAutoDisable() ) {
    dBodySetAutoDisableFlag( body, params->getAutoDisable() );
  }

  // disableAngularSpeed
  if( params->haveDisableAngularSpeed() ) {
    dBodySetAutoDisableAngularThreshold( body,
                                         params->getDisableAngularSpeed() );
  }

  // disableLinearSpeed
  if( params->haveDisableLinearSpeed() ) {
    dBodySetAutoDisableLinearThreshold( body,
                                        params->getDisableLinearSpeed() );
  }

  // disableTime
  if( params->haveDisableTime() ) {
    dBodySetAutoDisableTime( body,
                             params->getDisableTime() );
  }

  // startLinearVelocity 
  if( params->haveStartLinearVelocity() ) {
    const Vec3f &v = params->getStartLinearVelocity();
    dBodySetLinearVel( body, v.x, v.y, v.z );
  }
  

  //  startAngularVelocity
  if( params->haveStartAngularVelocity() ) {
    const Vec3f &v = params->getStartAngularVelocity();
    dBodySetAngularVel( body, v.x, v.y, v.z );
  }

  // mass, massDensityModel, inertia and centerOfMass
  if( params->haveMass() || params->haveMassDensityModel() ||
      params->haveInertia() || params->haveCenterOfMass() ) {
  
    dMass ode_mass;

    Node *n = params->getMassDensityModel();
    if( !n ) {
      const Vec3f &center = params->getCenterOfMass();
      const Matrix3f &inertia = params->getInertia();
      // no mass density model, use ineria and center of mass directly.
      dMassSetParameters( &ode_mass,
                          params->getMass(),
                          center.x, center.y, center.z,
                          inertia[0][0], inertia[1][1], inertia[2][2],
                          inertia[0][1], inertia[0][2], inertia[1][2] );
    } else {
      if (Box *b = dynamic_cast< Box* >( n ) ) {
        Vec3f size = b->size->getValue();
        dMassSetBoxTotal( &ode_mass, params->getMass(), size.x, size.y, size.z);
      } else if (Sphere *sph = dynamic_cast< Sphere * >( n ) ){
        dReal radius = sph->radius->getValue();
        dMassSetSphereTotal(&ode_mass, params->getMass(), radius);
      } else if (Cylinder *cyl = dynamic_cast< Cylinder * >( n ) ){
        dReal radius = cyl->radius->getValue();
        dReal height = cyl->height->getValue();
        dMassSetCylinderTotal( &ode_mass, params->getMass(), 2, radius, height );
      } else if (X3DGeometryNode *g = dynamic_cast< X3DGeometryNode * >( n )){
        // we have an unsupported mass density model, use the bounding box of it as
        // density model
        // Console(4) << "Invalid mass density model \" " << n->getTypeName() << "\". " 
        //            << "Using bound of node as density model instead." << endl;
        BoxBound *bb = dynamic_cast< BoxBound * >( g->bound->getValue() );
        Vec3f size( 0.1f, 0.1f, 0.1f );
        if( bb ) size = bb->size->getValue();
        dMassSetBoxTotal( &ode_mass, params->getMass(), size.x, size.y, size.z);
      } else {
        Console(4) << "Invalid node used in mass density model \" " << n->getTypeName() << "\". " 
                   << "Mass density model must be a geometry." << endl;
      }
    }
    dBodySetMass( ( dBodyID )params->getBodyId(), &ode_mass);
  }
 

  // enabled
  if( params->haveEnabled() ) {
    if( params->getEnabled() ) {
      dBodyEnable( body );
    } else {
      dBodyDisable( body );
    }
  }

  // startPosition 
  if( params->haveStartPosition() ) {
    const Vec3f &start_position = params->getStartPosition();
    dBodySetPosition( ( dBodyID )params->getBodyId(), 
                      start_position.x, 
                      start_position.y, 
                      start_position.z);
  }

  // startOrientation 
  if( params->haveStartOrientation() ) {
    const Rotation &start_orientation = params->getStartOrientation();
    dMatrix3 rotation;
    dRFromAxisAndAngle( rotation, 
                        start_orientation.axis.x, 
                        start_orientation.axis.y, 
                        start_orientation.axis.z, 
                        start_orientation.angle );
    dBodySetRotation( ( dBodyID )params->getBodyId(), rotation );
  }

  // useGlobalGravity
  if( params->haveUseGlobalGravity() ) {
    dBodySetGravityMode( body, params->getUseGlobalGravity() );
  }
  
  // useFiniteRotation
  if( params->haveUseFiniteRotation() ) {
    dBodySetFiniteRotationMode( body, params->getUseFiniteRotation() );
  }
  
  // finiteRotationAxis
  if( params->haveFiniteRotationAxis() ) {
    const Vec3f &axis = params->getFiniteRotationAxis();
    dBodySetFiniteRotationAxis( body, axis.x, axis.y, axis.z );
  }

  // fixed and geometry
  if( params->haveFixed() || params->haveGeometry() ) {
    bool fixed = params->getFixed();

    if ( fixed ) {
      // Disable the body so it does not move. 
      dBodyDisable( ( dBodyID )params->getBodyId() );
    } else {
      if( params->getEnabled() ) {
        dBodyEnable( ( dBodyID )params->getBodyId() ); 
      }
    }

    const vector< H3DCollidableId > &geom_ids = params->getGeometry();
    for( vector< H3DCollidableId >::const_iterator i = geom_ids.begin();
         i != geom_ids.end(); ++i ) {
      dGeomID collidable_id = (dGeomID) *i;
      if ( fixed ) {
        // Attach the shape to 0 instead of a rigid body. ODE way of saying fixed
        // object.
        dGeomSetBody( collidable_id, 0 );
    
        // Set geom position and rotation = body position and rotation
        const dReal *pos = dBodyGetPosition( ( dBodyID )params->getBodyId() );
        dGeomSetPosition( collidable_id, pos[0], pos[1], pos[2] );    
        
        const dReal *R   = dBodyGetRotation( ( dBodyID )params->getBodyId() );
        dMatrix3 rotation;    
        for ( int j=0; j<12;++j ) {
          rotation[j] = R[j];
        }
        dGeomSetRotation( collidable_id, rotation );      
      } else {
        // connect body and shape.
        dGeomSetBody( collidable_id, ( dBodyID )params->getBodyId() );
      }
    }
  }

  // Once a body is connected / updated the offsets can be applied relative to the body.
  ODECallbacks::handlePendingOffsets( params );
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::setRigidBodyParameters(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());

  ODECallbacks::setRigidBodyParameters( params, ode_data );
  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::getRigidBodyParameters(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());

  // Get position of body in global coordinates
  dBodyID bodyID = ( dBodyID )params->getBodyId();
  
  const dReal *pos = dBodyGetPosition( bodyID );
  const dReal *R   = dBodyGetRotation( bodyID );
  const dReal *lin_vel = dBodyGetLinearVel( bodyID );
  const dReal *ang_vel = dBodyGetAngularVel( bodyID );
  Rotation rot = Rotation(Matrix3f((H3DFloat)R[0], (H3DFloat)R[4], (H3DFloat)R[8], 
                                   (H3DFloat)R[1], (H3DFloat)R[5], (H3DFloat)R[9], 
                                   (H3DFloat)R[2], (H3DFloat)R[6], (H3DFloat)R[10]));  
  
  params->setPosition( Vec3f((H3DFloat)pos[0], (H3DFloat)pos[1], (H3DFloat)pos[2]) );
  params->setOrientation( -rot );
  params->setLinearVelocity( Vec3f((H3DFloat)lin_vel[0], 
                                   (H3DFloat)lin_vel[1], 
                                   (H3DFloat)lin_vel[2]) );
  params->setAngularVelocity( Vec3f((H3DFloat)ang_vel[0], 
                                    (H3DFloat)ang_vel[1], 
                                    (H3DFloat)ang_vel[2]) );
  return PeriodicThread::CALLBACK_DONE;
}

void updateCollidableParameters( PhysicsEngineParameters::CollidableParameters* params ) {

  dGeomID geom_id = ( dGeomID )params->getCollidableId();

  ODECallbacks::ODESpecificData *ode_data = 
    static_cast< ODECallbacks::ODESpecificData * >(params->getEngine()->getEngineSpecificData());

  ODECallbacks::ODESpecificData::CollidableOffset offset;
  offset.rotation = params->getRotation();
  offset.translation = params->getTranslation();

  ode_data->pending_offsets[ geom_id ].push_back( offset );
  
  if( params->getEnabled() )
    dGeomEnable( geom_id );
  else
    dGeomDisable( geom_id );
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::setCollidableParameters(void *data) {
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );
  updateCollidableParameters( params );

  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::addCollidable(void *data) {
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );
  if ( ShapeParameters *p = dynamic_cast< ShapeParameters* >( params ) ) {
    addShape( p );
    //cerr << "ODE: collidableShape created with collidableID " << params->collidable_id << endl;
  } else if ( OffsetParameters *p = dynamic_cast< OffsetParameters* >( params ) ) {
    addOffset( p );
    //cerr << "ODE: collidableOffet created with collidableID " << params->collidable_id << endl;
  }
  return PeriodicThread::CALLBACK_DONE;
}

void ODECallbacks::addOffset( OffsetParameters * params ) {
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  if ( params->getParentSpaceId() == CollidableParameters::WORLD_SPACE ) {
    params->setParentSpaceId( (H3DSpaceId) ode_data->space_id );
  } else if ( params->getParentSpaceId() == CollidableParameters::NO_SPACE ) {
    params->setParentSpaceId( (H3DSpaceId) 0 );
  }

  params->setCollidableId( params->collidable );

  ODESpecificData::CollidableOffset offset;
  offset.translation = params->getTranslation();
  offset.rotation = params->getRotation();

  dGeomID geom_id = (dGeomID) params->collidable;

  ode_data->pending_offsets[ geom_id ].push_back( offset );

  if( params->getEnabled() )
    dGeomEnable( geom_id );
  else
    dGeomDisable( geom_id );
}

void ODECallbacks::handlePendingOffsets( RigidBodyParameters* params ) {

  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());

  std::vector< H3DCollidableId > geometries = params->getGeometry();

  for( std::vector< H3DCollidableId >::iterator i = geometries.begin(); i != geometries.end(); ++i ) {

    dGeomID geom_id = (dGeomID) *i;
    dBodyID body_id = dGeomGetBody( geom_id );

    if( body_id != 0 && ode_data->pending_offsets.count( geom_id ) != 0 ) {

      dGeomClearOffset( geom_id );

      const std::vector< ODESpecificData::CollidableOffset >& offsets = ode_data->pending_offsets[ geom_id ];

      Rotation total_rotation = Rotation();
      Vec3f total_translation = Vec3f();

      for( size_t j = 0; j < offsets.size(); ++j ) {
        total_translation += offsets[j].translation;
        total_rotation *= offsets[j].rotation;
      }

      dMatrix3 R;
      dRFromAxisAndAngle(R, 
                          total_rotation.axis.x,  
                          total_rotation.axis.y,  
                          total_rotation.axis.z, 
                          total_rotation.angle );

      dGeomSetOffsetPosition( geom_id, 
                              total_translation.x, 
                              total_translation.y, 
                              total_translation.z );
      dGeomSetOffsetRotation( geom_id, R);

      ode_data->pending_offsets.erase( geom_id );

    }

  }

}

void ODECallbacks::addShape( ShapeParameters * params ) {
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  if ( params->getParentSpaceId() == CollidableParameters::WORLD_SPACE ) {
    params->setParentSpaceId( (H3DSpaceId) ode_data->space_id );
  } else if ( params->getParentSpaceId() == CollidableParameters::NO_SPACE ) {
    params->setParentSpaceId( (H3DSpaceId) 0 );
  }

  dGeomID temp_geom;

  if ( Box *b = dynamic_cast< Box * >( params->getShape() )) {
    Vec3f size = b->size->getValue();
    temp_geom = dCreateBox(ode_data->space_id, size.x, size.y, size.z);
  } else if ( Sphere *sph = dynamic_cast< Sphere * >( params->getShape() )) {
    dReal radius = sph->radius->getValue();
    temp_geom = dCreateSphere(ode_data->space_id, radius);
  } else if ( Cylinder *cyl = dynamic_cast< Cylinder * >( params->getShape() )) {  
    dReal radius = cyl->radius->getValue();
    dReal height = cyl->height->getValue();

    // the ODE cylinder is specified along z-axis, so we need to rotate
    // it to be along y-axis.
    temp_geom = dCreateCylinder(ode_data->space_id, radius, height);

    ODESpecificData::CollidableOffset offset;
    offset.rotation = Rotation( Vec3f( 1.0f, 0.0f, 0.0f ), H3DFloat( Constants::pi ) / 2.0f );
    offset.translation = Vec3f( 0.0f, 0.0f, 0.0f );

    ode_data->pending_offsets[ temp_geom ].push_back( offset );

  } else {
    vector< HAPI::Collision::Triangle > triangles;
    if( params->haveShape() ) {    
      if( params->getShape()->boundTree->isUpToDate() ) {
        // Only do getValue() if the tree is already uptodate, else causes OpenGL error
        params->getShape()->boundTree->getValue()->getAllTriangles( triangles );
      }
    }
    // if there are no triangles built then don't create a shape
    if( triangles.size() > 0 ) {
      Console( 4 ) << "Error: ODE. error no triangles in the shape bound tree!" << endl;
    }

    dVector3 *ode_tris = new dVector3[ triangles.size()*3];
    dTriIndex *ode_indices = new dTriIndex[ triangles.size() * 3];
    for( unsigned int i = 0; i < triangles.size(); ++i ) {
      ode_tris[i*3][0] = triangles[i].a.x;
      ode_tris[i*3][1] = triangles[i].a.y;
      ode_tris[i*3][2] = triangles[i].a.z;
      ode_indices[i*3] = i*3;

      ode_tris[i*3+1][0] = triangles[i].b.x;
      ode_tris[i*3+1][1] = triangles[i].b.y;
      ode_tris[i*3+1][2] = triangles[i].b.z;
      ode_indices[i*3+1] = i*3+1;

      ode_tris[i*3+2][0] = triangles[i].c.x;
      ode_tris[i*3+2][1] = triangles[i].c.y;
      ode_tris[i*3+2][2] = triangles[i].c.z;
      ode_indices[i*3+2] = i*3+2;
    }

    dTriMeshDataID tri_mesh_id = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSimple( tri_mesh_id,
                                 (dReal*)ode_tris, static_cast<int>(triangles.size() * 3),
                                 ode_indices, static_cast<int>(triangles.size() * 3 ));
    temp_geom = dCreateTriMesh( ode_data->space_id,
                                tri_mesh_id, NULL, NULL, NULL );
    ode_data->tri_meshes[temp_geom] = temp_geom;
  }

  params->setCollidableId( ( H3DCollidableId )temp_geom );

  // Add offset defined by the collidable's initial transformation matrix.
  ODESpecificData::CollidableOffset offset;
  offset.rotation = params->getRotation();
  offset.translation = params->getTranslation();
  ode_data->pending_offsets[ temp_geom ].push_back( offset );

}


H3DUtil::PeriodicThread::CallbackCode ODECallbacks::addRigidBody(void *data) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());

  dBodyID body_temp = dBodyCreate( ode_data->world_id );
  params->setBodyId( (H3DUInt64) body_temp );
  //cerr << "ODE: dBodyCreate called, body created with bodyID " << params->body_id << endl;
  ODECallbacks::setRigidBodyParameters( params, ode_data ); 
  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::getCollidableParameters(void *data) {
  PhysicsEngineParameters::CollidableParameters *params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  
  dGeomID geomID = ( dGeomID )params->getCollidableId();
  const dReal *pos = dGeomGetPosition( geomID );
  const dReal *R   = dGeomGetRotation( geomID );
  Rotation rot = Rotation(Matrix3f((H3DFloat)R[0], (H3DFloat)R[4], (H3DFloat)R[8], 
                                   (H3DFloat)R[1], (H3DFloat)R[5], (H3DFloat)R[9], 
                                   (H3DFloat)R[2], (H3DFloat)R[6], (H3DFloat)R[10]));  
  params->setTranslation( Vec3f((H3DFloat)pos[0], (H3DFloat)pos[1], (H3DFloat)pos[2]) );
  params->setRotation( -rot );

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::addGlobalExternalForceAndTorque(void *data) {
  PhysicsEngineParameters::ExternalForceTorqueParameters *params = 
    static_cast< PhysicsEngineParameters::ExternalForceTorqueParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->engine_thread->getEngineSpecificData());
  // Check if autodisable disabled the body, if it did then enable it again.
  RigidBodyParameters rbp;  // RB object keyed from X3D file
  params->engine_thread->getRigidBodyParameters(params->body_id, rbp);
  if( !dBodyIsEnabled( (dBodyID)params->body_id ) &&
      params->force.length() > Constants::f_epsilon &&
      !rbp.getFixed() && rbp.getEnabled() ) {   // make sure it's not 'fixed' and 'enabled'
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
  return PeriodicThread::CALLBACK_DONE;
}

void ODECallbacks::attachJoint(void *data) {
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );
  dJointAttach( ( dJointID )params->getConstraintId(), ( dBodyID )params->getBody1(), ( dBodyID )params->getBody2() );
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::addConstraint(void *data) {
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  
  if( params->getType() == "BallJoint" ) {
    dJointID temp_joint = dJointCreateBall( ode_data->world_id, 0 );
    params->setConstraintId( ( H3DUInt64 )temp_joint );
    //std::cout << "ODE: Created ballJoint: " << params->joint_id << "\n";
  }
  if( params->getType() == "DoubleAxisHingeJoint" ) {
    dJointID temp_joint = dJointCreateHinge2( ode_data->world_id, 0 );
    params->setConstraintId( ( H3DUInt64 )temp_joint );
    //std::cout << "ODE: Created doubleAxisHingeJoint: " << params->joint_id << "\n";
  }
  if( params->getType() == "MotorJoint" ) {
    dJointID temp_joint = dJointCreateAMotor ( ode_data->world_id, 0 );
    params->setConstraintId( ( H3DUInt64 )temp_joint );
    //std::cout << "ODE: Created motorJoint: " << params->joint_id << "\n";
  }
  if( params->getType() == "SingleAxisHingeJoint" ) {
    dJointID temp_joint = dJointCreateHinge( ode_data->world_id, 0 );
    params->setConstraintId( ( H3DUInt64 )temp_joint );
    //std::cout << "ODE: Created singleAxisHingeJoint: " << params->joint_id << "\n";
  }  
  if( params->getType() == "SliderJoint" ) {
    dJointID temp_joint = dJointCreateSlider( ode_data->world_id, 0 );
    params->setConstraintId( ( H3DUInt64 )temp_joint );
    //std::cout << "ODE: Created sliderJoint: " << params->joint_id << "\n";
  }
  if( params->getType() == "UniversalJoint" ) {
    dJointID temp_joint = dJointCreateUniversal( ode_data->world_id, 0 );
    params->setConstraintId( ( H3DUInt64 )temp_joint );
    //std::cout << "ODE: Created universalJoint: " << params->joint_id << "\n";
  }
  // note: attach must be done before setJointParameters
  if ( params->getConstraintId() ) {
    ODECallbacks::attachJoint( params );
    ODECallbacks::setConstraintParameters( params );
  } else {
    // Joint creation failed
    Console ( 3 ) << "Warning: Joint type " << params->getType() << " not supported by ODE!" << endl;
  }

  return PeriodicThread::CALLBACK_DONE;
}


// Balljoint
void ODECallbacks::setBallJointParameters(void *data) {
  PhysicsEngineParameters::BallJointParameters *params = 
    static_cast< PhysicsEngineParameters::BallJointParameters *>( data );

  if( params->haveAnchorPoint() ) {
    const Vec3f &ap = params->getAnchorPoint();
    dJointSetBallAnchor( ( dJointID )params->getConstraintId(),
                          ap.x, ap.y, ap.z);
  }
}


void ODECallbacks::getBallJointParameters(void *data) {
  PhysicsEngineParameters::BallJointParameters *params = 
    static_cast< PhysicsEngineParameters::BallJointParameters *>( data );
 
  if( params->haveBody1AnchorPoint() ) {
    dVector3 v;
    dJointGetBallAnchor( ( dJointID )params->getConstraintId(), v );
    params->setBody1AnchorPoint( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }
  if( params->haveBody2AnchorPoint() ) {
    dVector3 v;
    dJointGetBallAnchor2( ( dJointID )params->getConstraintId(), v );
    params->setBody2AnchorPoint( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }
}


// DoubleAxisHingeJoint
void ODECallbacks::setDoubleAxisHingeJointParameters(void *data) {
  PhysicsEngineParameters::DoubleAxisHingeJointParameters *params = 
    static_cast< PhysicsEngineParameters::DoubleAxisHingeJointParameters *>( data );
 
  if( params->haveAnchorPoint() ) {
    const Vec3f &ap = params->getAnchorPoint();
    dJointSetHinge2Anchor( ( dJointID )params->getConstraintId(), 
                           ap.x, ap.y, ap.z);
  }
#ifdef ODE_VERSION_013_OR_EARLIER
  if( params->haveAxis1() ) {
    const Vec3f &axis1 = params->getAxis1();
    dJointSetHinge2Axis1( ( dJointID )params->getConstraintId(), 
                          axis1.x, axis1.y, axis1.z);
  }
  if( params->haveAxis2() ) {
    const Vec3f &axis2 = params->getAxis2();
    dJointSetHinge2Axis2( ( dJointID )params->getConstraintId(), 
                          axis2.x, axis2.y, axis2.z);
  }
#else
  dReal axis1_ode[3] = {0,0,0};
  if( params->haveAxis1() ) {
    const Vec3f &axis1 = params->getAxis1();
    axis1_ode[0] = axis1.x;
    axis1_ode[1] = axis1.y;
    axis1_ode[2] = axis1.z;
  }
  
  dReal axis2_ode[3] = {0,0,0};
  if( params->haveAxis2() ) {
    const Vec3f &axis2 = params->getAxis2();
    axis2_ode[0] = axis2.x;
    axis2_ode[1] = axis2.y;
    axis2_ode[2] = axis2.z;
  }
  
  if( params->haveAxis1() || params->haveAxis2() ) {
    dJointSetHinge2Axes( ( dJointID )params->getConstraintId(), params->haveAxis1() ? axis1_ode : NULL, params->haveAxis2() ? axis2_ode : NULL );
  }
#endif
  if( params->haveDesiredAngularVelocity1() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(), dParamVel, params->getDesiredAngularVelocity1() );
  }
  if( params->haveDesiredAngularVelocity2() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(), dParamVel2, params->getDesiredAngularVelocity2() );
  }    
  if( params->haveMinAngle1() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamLoStop,
                          params->getMinAngle1() );
  }
  if( params->haveMaxAngle1() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamHiStop,
                          params->getMaxAngle1() );
  }
  if( params->haveMaxTorque1() ) {
    if( params->getMaxTorque1() < 0 ) { 
      // ODE specifies dParamFMax >= 0. X3D specifies value can be any real number
    }
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamFMax,
                          params->getMaxTorque1() );
  }
  if( params->haveMaxTorque2() ) {
    if( params->getMaxTorque2() < 0 ) { 
      // ODE specifies dParamFMax >= 0. X3D specifies value can be any real number
    }
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamFMax2,
                          params->getMaxTorque2() );
  }
  if( params->haveStopBounce1() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamBounce,
                          params->getStopBounce1() );
  }
  if( params->haveStopConstantForceMix1() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamStopCFM,
                          params->getStopConstantForceMix1() );
  }
  if( params->haveStopErrorCorrection1() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamStopERP,
                          params->getStopErrorCorrection1() );
  }
  if( params->haveSuspensionErrorCorrection() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamSuspensionERP,
                          params->getSuspensionErrorCorrection() );
  }
  if( params->haveSuspensionForce() ) {
    dJointSetHinge2Param( ( dJointID )params->getConstraintId(),
                          dParamSuspensionCFM,
                          params->getSuspensionForce() );
  }
}


void ODECallbacks::getDoubleAxisHingeJointParameters(void *data) {
  PhysicsEngineParameters::DoubleAxisHingeJointParameters *params = 
    dynamic_cast< PhysicsEngineParameters::DoubleAxisHingeJointParameters *>( (PhysicsEngineParameters::JointParameters *)data );
 
  if( params->haveHinge1Angle() ) {
    params->setHinge1Angle( (H3DFloat)dJointGetHinge2Angle1( ( dJointID )params->getConstraintId() ) );
  }
  if( params->haveHinge2Angle() ) {
    //params->setHinge2Angle( (H3DFloat)dJointGetHinge2Angle2( ( dJointID )params->joint_id ) );
  }
  if( params->haveHinge1AngleRate() ) {
    params->setHinge1AngleRate( (H3DFloat)dJointGetHinge2Angle1Rate( ( dJointID )params->getConstraintId() ) );
  }
  if( params->haveHinge2AngleRate() ) {
    params->setHinge2AngleRate( (H3DFloat)dJointGetHinge2Angle2Rate( ( dJointID )params->getConstraintId() ) );
  }
  if( params->haveBody1AnchorPoint() ) {
    dVector3 v;
    dJointGetHinge2Anchor( ( dJointID )params->getConstraintId(), v );
    params->setBody1AnchorPoint( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }
  if( params->haveBody2AnchorPoint() ) {
    dVector3 v;
    dJointGetHinge2Anchor2( ( dJointID )params->getConstraintId(), v );
    params->setBody2AnchorPoint( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }
  if( params->haveBody1Axis() ) {
    dVector3 v;
    dJointGetHinge2Axis1( ( dJointID )params->getConstraintId(), v );
    params->setBody1Axis( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }
  if( params->haveBody2Axis() ) {
    dVector3 v;
    dJointGetHinge2Axis2( ( dJointID )params->getConstraintId(), v );
    params->setBody2Axis( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }
}


// MotorJoint
void ODECallbacks::setMotorJointParameters(void *data) {
  PhysicsEngineParameters::MotorJointParameters *params = 
    static_cast< PhysicsEngineParameters::MotorJointParameters *>( data );
  // autoCalc == TRUE
  if( params->haveAutoCalc() && params->getAutoCalc() ) {
    dJointSetAMotorMode( ( dJointID )params->getConstraintId(), dAMotorEuler );
  } else {
    // autoCalc == FALSE
    dJointSetAMotorMode( ( dJointID )params->getConstraintId(), dAMotorUser );
    if( params->haveEnabledAxes() ) {
      dJointSetAMotorNumAxes ( ( dJointID )params->getConstraintId(), params->getEnabledAxes() );
    }
    // set axisAngles. axisAngles only need to be set in manual mode
    if( params->haveAxis1Angle() ) {
      dJointSetAMotorAngle ( ( dJointID )params->getConstraintId(), 0, params->getAxis1Angle() );
    }
    if( params->haveAxis2Angle() ) {
      dJointSetAMotorAngle ( ( dJointID )params->getConstraintId(), 0, params->getAxis2Angle() );
    }
    if( params->haveAxis3Angle() ) {
      dJointSetAMotorAngle ( ( dJointID )params->getConstraintId(), 0, params->getAxis3Angle() );
    }
  }
  // motor1Axis
  if( params->haveMotor1Axis() ) {
    const Vec3f &axis = params->getMotor1Axis();
    dJointSetAMotorAxis( ( dJointID )params->getConstraintId(), 0, 0, axis.x, axis.y, axis.z );
  }
  // motor2Axis
  if( params->haveMotor2Axis() ) {
    const Vec3f &axis = params->getMotor2Axis();
    dJointSetAMotorAxis( ( dJointID )params->getConstraintId(), 1, 1, axis.x, axis.y, axis.z );
  }
  // motor3Axis
  if( params->haveMotor3Axis() ) {
    const Vec3f &axis = params->getMotor3Axis();
    dJointSetAMotorAxis( ( dJointID )params->getConstraintId(), 2, 2, axis.x, axis.y, axis.z );
  }
  // axisTorques. if motor has fewer than 3 axes, the extra args are automatically ignored by ODE
  H3DFloat torque1 = 0, torque2 = 0, torque3 = 0;
  if ( params->haveAxis1Torque() ) torque1 = params->getAxis1Torque();
  if ( params->haveAxis2Torque() ) torque2 = params->getAxis2Torque();
  if ( params->haveAxis3Torque() ) torque3 = params->getAxis3Torque();
  if ( torque1 || torque2 || torque3 ) {
    dJointAddAMotorTorques( ( dJointID )params->getConstraintId(), torque1, torque2, torque3 );
  }
  // stop1Bounce
  if( params->haveStop1Bounce() ) {
    dJointSetAMotorParam( ( dJointID )params->getConstraintId(),
                         dParamBounce,
                         params->getStop1Bounce() );
  }
  // stop1ErrorCorrection
  if( params->haveStop1ErrorCorrection() ) {
    dJointSetAMotorParam( ( dJointID )params->getConstraintId(),
                         dParamStopERP,
                         params->getStop1ErrorCorrection() );
  }
  // stop1Bounce
  if( params->haveStop2Bounce() ) {
    dJointSetAMotorParam( ( dJointID )params->getConstraintId(),
                         dParamBounce2,
                         params->getStop2Bounce() );
  }
  // stop2ErrorCorrection
  if( params->haveStop2ErrorCorrection() ) {
    dJointSetAMotorParam( ( dJointID )params->getConstraintId(),
                         dParamStopERP2,
                         params->getStop2ErrorCorrection() );
  }
  // stop3Bounce
  if( params->haveStop3Bounce() ) {
    dJointSetAMotorParam( ( dJointID )params->getConstraintId(),
                         dParamBounce3,
                         params->getStop3Bounce() );
  }
  // stop3ErrorCorrection
  if( params->haveStop2ErrorCorrection() ) {
    dJointSetAMotorParam( ( dJointID )params->getConstraintId(),
                         dParamStopERP3,
                         params->getStop3ErrorCorrection() );
  }
}

void ODECallbacks::getMotorJointParameters(void *data) {
  PhysicsEngineParameters::MotorJointParameters *params = 
    dynamic_cast< PhysicsEngineParameters::MotorJointParameters *>( (PhysicsEngineParameters::JointParameters *)data );
 
  // angle output
  if( params->haveMotor1Angle() ) {
    params->setMotor1Angle( (H3DFloat)dJointGetAMotorAngle( ( dJointID )params->getConstraintId(), 0 ) );
  }
  if( params->haveMotor2Angle() ) {
    params->setMotor2Angle( (H3DFloat)dJointGetAMotorAngle( ( dJointID )params->getConstraintId(), 1 ) );
  }
  if( params->haveMotor3Angle() ) {
    params->setMotor3Angle( (H3DFloat)dJointGetAMotorAngle( ( dJointID )params->getConstraintId(), 2 ) );
  }
  // angleRate output
  if( params->haveMotor1AngleRate() ) {
    params->setMotor1AngleRate( (H3DFloat)dJointGetAMotorAngleRate( ( dJointID )params->getConstraintId(), 0 ) );
  }
  if( params->haveMotor2AngleRate() ) {
    params->setMotor2AngleRate( (H3DFloat)dJointGetAMotorAngleRate( ( dJointID )params->getConstraintId(), 1 ) );
  }
  if( params->haveMotor3AngleRate() ) {
    params->setMotor3AngleRate( (H3DFloat)dJointGetAMotorAngleRate( ( dJointID )params->getConstraintId(), 2 ) );
  }
}

// SingleAxisHingeJoint
void ODECallbacks::setSingleAxisHingeJointParameters(void *data) {
  PhysicsEngineParameters::SingleAxisHingeJointParameters *params = 
    static_cast< PhysicsEngineParameters::SingleAxisHingeJointParameters *>( data );
 
  // anchorPoint
  if( params->haveAnchorPoint() ) {
    const Vec3f &ap = params->getAnchorPoint();
    dJointSetHingeAnchor( ( dJointID )params->getConstraintId(), 
                          ap.x, ap.y, ap.z);
  }
  // axis
  if( params->haveAxis() ) {
    const Vec3f &axis = params->getAxis();
    dJointSetHingeAxis( ( dJointID )params->getConstraintId(), 
                        axis.x, axis.y, axis.z);
  }
  // minAngle
  if( params->haveMinAngle() ) {
    dJointSetHingeParam( ( dJointID )params->getConstraintId(),
                         dParamLoStop,
                         params->getMinAngle() );
  }
  // maxAngle
  if( params->haveMaxAngle() ) {
    dJointSetHingeParam( ( dJointID )params->getConstraintId(),
                         dParamHiStop,
                         params->getMaxAngle() );
  }
  // stopBounce
  if( params->haveStopBounce() ) {
    dJointSetHingeParam( ( dJointID )params->getConstraintId(),
                         dParamBounce,
                         params->getStopBounce() );
  }
  // stopErrorCorrection
  if( params->haveStopErrorCorrection() ) {
    dJointSetHingeParam( ( dJointID )params->getConstraintId(),
                         dParamStopERP,
                         params->getStopErrorCorrection() );
  }
}


void ODECallbacks::getSingleAxisHingeJointParameters(void *data) {
  PhysicsEngineParameters::SingleAxisHingeJointParameters *params = 
    dynamic_cast< PhysicsEngineParameters::SingleAxisHingeJointParameters *>( (PhysicsEngineParameters::JointParameters *)data );
 
  // angle output
  if( params->haveAngle() ) {
    params->setAngle( (H3DFloat)dJointGetHingeAngle( ( dJointID )params->getConstraintId() ) );
  }

  // angleRate output
  if( params->haveAngleRate() ) {
    params->setAngleRate( (H3DFloat)dJointGetHingeAngleRate( ( dJointID )params->getConstraintId() ) );
  }

  // body1AnchorPoint output
  if( params->haveBody1AnchorPoint() ) {
    dVector3 v;
    dJointGetHingeAnchor( ( dJointID )params->getConstraintId(), v );
    params->setBody1AnchorPoint( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }

  // body2AnchorPoint
  if( params->haveBody2AnchorPoint() ) {
    dVector3 v;
    dJointGetHingeAnchor2( ( dJointID )params->getConstraintId(), v );
    params->setBody2AnchorPoint( Vec3f( (H3DFloat)v[0], (H3DFloat)v[1], (H3DFloat)v[2] ) );
  }
}


// SliderJoint
void ODECallbacks::setSliderJointParameters(void *data) {
  PhysicsEngineParameters::SliderJointParameters *params = 
    static_cast< PhysicsEngineParameters::SliderJointParameters *>( data );
 
  // axis
  if( params->haveAxis() ) {
    const Vec3f &axis = params->getAxis();
    dJointSetSliderAxis( ( dJointID )params->getConstraintId(), 
                         axis.x, axis.y, axis.z);
  }
  // minSeparation
  if( params->haveMinSeparation() ) {
    dJointSetSliderParam( ( dJointID )params->getConstraintId(),
                           dParamLoStop,
                           params->getMinSeparation() );
  }
  // maxSepartion
  if( params->haveMaxSeparation() ) {
    dJointSetSliderParam( ( dJointID )params->getConstraintId(),
                           dParamHiStop,
                           params->getMaxSeparation() );
  }
  // stopBounce
  if( params->haveStopBounce() ) {
    dJointSetSliderParam( ( dJointID )params->getConstraintId(),
                           dParamBounce,
                           params->getStopBounce() );
  }
  // stopErrorCorrection
  if( params->haveStopErrorCorrection() ) {
    dJointSetSliderParam( ( dJointID )params->getConstraintId(),
                           dParamStopERP,
                           params->getStopErrorCorrection() );
  }

  // sliderForce
  ODESpecificData *ode_data = 
        static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  map< dJointID, dReal >::iterator joint_force_iterator =
    ode_data->slider_joint_forces.find( ( dJointID )params->getConstraintId() );
  if( params->haveSliderForce() ) {
    H3DFloat slider_force = params->getSliderForce();
    if( slider_force != 0 ) {
      if( joint_force_iterator == ode_data->slider_joint_forces.end() ) {
        ode_data->slider_joint_forces[( dJointID )params->getConstraintId()] = dReal( slider_force );
      } else {
        (*joint_force_iterator).second = dReal( slider_force );
      }
    } else if( joint_force_iterator != ode_data->slider_joint_forces.end() ) {
      ode_data->slider_joint_forces.erase( joint_force_iterator );
    }
  } else if( joint_force_iterator != ode_data->slider_joint_forces.end() ) {
    ode_data->slider_joint_forces.erase( joint_force_iterator );
  }
}

void ODECallbacks::getSliderJointParameters(void *data) {
  PhysicsEngineParameters::SliderJointParameters *params = 
    dynamic_cast< PhysicsEngineParameters::SliderJointParameters *>( (PhysicsEngineParameters::JointParameters *)data );
 
  // separation output
  if( params->haveSeparation() ) {
    params->setSeparation( (H3DFloat)dJointGetSliderPosition( ( dJointID )params->getConstraintId() ) );
  }
  // separationRate output
  if( params->haveSeparationRate() ) {
    params->setSeparationRate( (H3DFloat)dJointGetSliderPositionRate( ( dJointID )params->getConstraintId() ) );
  }
}

// UniversalJoint
void ODECallbacks::setUniversalJointParameters(void *data) {
  PhysicsEngineParameters::UniversalJointParameters *params = 
    static_cast< PhysicsEngineParameters::UniversalJointParameters *>( data );

  // anchorPoint
  if( params->haveAnchorPoint() ) {
    const Vec3f &point = params->getAnchorPoint();
    dJointSetUniversalAnchor( ( dJointID )params->getConstraintId(), 
                               point.x, point.y, point.z);
  }
  // axis1
  if( params->haveAxis1() ) {
    const Vec3f &axis = params->getAxis1();
    dJointSetUniversalAxis1( ( dJointID )params->getConstraintId(), 
                             axis.x, axis.y, axis.z);
  }
  // axis2
  if( params->haveAxis2() ) {
    const Vec3f &axis = params->getAxis2();
    dJointSetUniversalAxis2( ( dJointID )params->getConstraintId(), 
                             axis.x, axis.y, axis.z);
  }
  // stop1Bounce
  if( params->haveStop1Bounce() ) {
    dJointSetUniversalParam( ( dJointID )params->getConstraintId(),
                             dParamBounce,
                             params->getStop1Bounce() );
  }
  // stop2Bounce
  if( params->haveStop2Bounce() ) {
    dJointSetUniversalParam( ( dJointID )params->getConstraintId(),
                             dParamBounce2,
                             params->getStop2Bounce() );
  }
  // stop1ErrorCorrection
  if( params->haveStop1ErrorCorrection() ) {
    dJointSetUniversalParam( ( dJointID )params->getConstraintId(),
                             dParamStopERP,
                             params->getStop1ErrorCorrection() );
  }
  // stop2ErrorCorrection
  if( params->haveStop2ErrorCorrection() ) {
    dJointSetUniversalParam( ( dJointID )params->getConstraintId(),
                             dParamStopERP2,
                             params->getStop2ErrorCorrection() );
  }
}

void ODECallbacks::getUniversalJointParameters(void *data) {
  PhysicsEngineParameters::UniversalJointParameters *params = 
    dynamic_cast< PhysicsEngineParameters::UniversalJointParameters *>( (PhysicsEngineParameters::JointParameters *)data );
 
  // body1AnchorPoint output
  if( params->haveBody1AnchorPoint() ) {
    dReal point[3];
    dJointGetUniversalAnchor( ( dJointID )params->getConstraintId(), point );
    params->setBody1AnchorPoint( Vec3f((H3DFloat)point[0], (H3DFloat)point[1], (H3DFloat)point[2]) );
  }
  // body2AnchorPoint output
  if( params->haveBody2AnchorPoint() ) {
    dReal point[3];
    dJointGetUniversalAnchor2( ( dJointID )params->getConstraintId(), point );
    params->setBody2AnchorPoint( Vec3f((H3DFloat)point[0], (H3DFloat)point[1], (H3DFloat)point[2]) );
  }
  // body1Axis output
  if( params->haveBody1Axis() ) {
    dReal point[3];
    dJointGetUniversalAxis1( ( dJointID )params->getConstraintId(), point );
    params->setBody1Axis( Vec3f((H3DFloat)point[0], (H3DFloat)point[1], (H3DFloat)point[2]) );
  }
  // body2Axis output
  if( params->haveBody2Axis() ) {
    dReal point[3];
    dJointGetUniversalAxis2( ( dJointID )params->getConstraintId(), point );
    params->setBody2Axis( Vec3f((H3DFloat)point[0], (H3DFloat)point[1], (H3DFloat)point[2]) );
  }
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::getCurrentContacts(void *data) {
  typedef pair< list< PhysicsEngineParameters::ContactParameters  >*,
                PhysicsEngineThread * > InputType;
  InputType *params = 
    static_cast< InputType *>( data );

  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->second->getEngineSpecificData());

  // Loop over collected ODE contacts and create ContactParameters
  for( vector<dContact>::iterator i= ode_data->allContacts.begin(); 
       i != ode_data->allContacts.end(); ++i ) {
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
//cerr << "Contacts: "  << ode_data->current_nr_contacts << endl;
  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode ODECallbacks::getConstraintParameters(void *data) {  
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );

  if( params->getType() == "BallJoint" ) {
    getBallJointParameters( data );
  }
  if( params->getType() == "DoubleAxisHingeJoint" ) {
    getDoubleAxisHingeJointParameters( data );
  }
  if( params->getType() == "MotorJoint" ) {
    getMotorJointParameters( data );
  }
  if( params->getType() == "SingleAxisHingeJoint" ) {
    getSingleAxisHingeJointParameters( data );
  }
  if( params->getType() == "SliderJoint" ) {
    getSliderJointParameters( data );
  }
  if( params->getType() == "UniversalJoint" ) {
    getUniversalJointParameters( data );
  }  
  return PeriodicThread::CALLBACK_DONE;
}

void ODECallbacks::setConstraintParameters( PhysicsEngineParameters::JointParameters * params ) {

  if( params->getType() == "BallJoint" ) {
    setBallJointParameters( params );
  }
  if( params->getType() == "DoubleAxisHingeJoint" ) {
    setDoubleAxisHingeJointParameters( params );
  }
  if( params->getType() == "MotorJoint" ) {
    setMotorJointParameters( params );
  }
  if( params->getType() == "SingleAxisHingeJoint" ) {
    setSingleAxisHingeJointParameters( params );
  }
  if( params->getType() == "SliderJoint" ) {
    setSliderJointParameters( params );
  }
  if( params->getType() == "UniversalJoint" ) {
    setUniversalJointParameters( params );
  }
}

H3DUtil::PeriodicThread::CallbackCode ODECallbacks::setConstraintParameters(void *data) {
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );
  setConstraintParameters( params );
  delete params;
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode ODECallbacks::removeCollidable( void *data ) {
  CollidableParameters *params = static_cast< CollidableParameters * >( data );
  dGeomDestroy( ( dGeomID )params->getCollidableId());
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  ode_data->tri_meshes.erase( (dGeomID)params->getCollidableId() );
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode ODECallbacks::removeRigidBody( void *data ) {
  RigidBodyParameters *params = static_cast< RigidBodyParameters * >( data ); 
  dBodyDestroy( ( dBodyID )params->getBodyId());
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode ODECallbacks::removeConstraint( void *data ) {
  JointParameters *params = static_cast< JointParameters * >( data ); 
  ODESpecificData *ode_data = 
        static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  map< dJointID, dReal >::iterator joint_force_iterator =
    ode_data->slider_joint_forces.find( ( dJointID )params->getConstraintId() );
  if( joint_force_iterator != ode_data->slider_joint_forces.end() ) {
    ode_data->slider_joint_forces.erase( joint_force_iterator );
  }
  dJointDestroy( ( dJointID ) params->getConstraintId());
  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode ODECallbacks::addSpace(void *data) {
  PhysicsEngineParameters::SpaceParameters *params = 
    static_cast< PhysicsEngineParameters::SpaceParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());
  if ( params->getParentSpaceId() == CollidableParameters::WORLD_SPACE ) {
    params->setParentSpaceId( (H3DSpaceId) ode_data->space_id );
  } else if ( params->getParentSpaceId() == CollidableParameters::NO_SPACE ) {
    params->setParentSpaceId( (H3DSpaceId) 0 );
  }
  params->setSpaceId( (H3DSpaceId) dHashSpaceCreate( (dSpaceID) params->getParentSpaceId() ) );
  ODECallbacks::setSpaceParameters( params );
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode ODECallbacks::removeSpace(void *data) {
  PhysicsEngineParameters::SpaceParameters *params = 
    static_cast< PhysicsEngineParameters::SpaceParameters *>( data );
  dSpaceDestroy( ( dSpaceID )params->getSpaceId() );
  //cerr << "ODE: dSpaceDestroy called for SpaceID " << ( dSpaceID )*id << endl;
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode ODECallbacks::setSpaceParameters(void *data) {
  PhysicsEngineParameters::SpaceParameters *params = 
    static_cast< PhysicsEngineParameters::SpaceParameters *>( data );
  ODESpecificData *ode_data = 
    static_cast< ODESpecificData * >(params->getEngine()->getEngineSpecificData());

  if( params->getEnabled() )
    dGeomEnable( ( dGeomID )params->getSpaceId() );
  else
    dGeomDisable( ( dGeomID )params->getSpaceId() );
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode ODECallbacks::getSpaceParameters(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}


#endif // HAVE_ODE
