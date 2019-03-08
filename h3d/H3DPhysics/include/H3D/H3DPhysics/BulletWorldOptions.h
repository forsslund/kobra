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
/// \file BulletWorldOptions.h
/// \brief Header file for BulletWorldOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __BULLETWORLDOPTIONS__
#define __BULLETWORLDOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>

namespace H3D{

  namespace PhysicsEngineParameters {
    /// Contains values for parameters specifying the behaviour of the
    /// bullet physics simulation on a grand scale.
    struct BulletWorldParameters : public EngineOptionParameters {

      /// Constructor.
      BulletWorldParameters() :
        worldScale( 1.0f ),
        fixedTimeStep( 1 / 1000.0f ),
        collisionMargin( 0.01f ),
        maxVelocityLinear( -1.0f ),
        maxVelocityAngular( -1.0f ) {}

      /// Set the bullet world scale.
      void setWorldScale( H3DFloat _worldScale ) {
        update_bit_mask |= WORLD_SCALE;
        worldScale = _worldScale;
      }

      /// Set a fixed time step for bullet.
      void setFixedTimeStep( H3DFloat _fixedTimeStep ) {
        update_bit_mask |= FIXED_TIME_STEP;
        fixedTimeStep = _fixedTimeStep;
      }

      /// Set the bullet collision shape margin
      void setCollisionMargin ( H3DFloat _collisionMargin ) {
        update_bit_mask|= COLLISION_MARGIN;
        collisionMargin= _collisionMargin;
      }

      /// Set the maximum allowed linear velocity for a physics object.
      void setMaxVelocityLinear( H3DFloat _maxVelocityLinear ) {
        update_bit_mask |= MAX_VELOCITY_LINEAR;
        maxVelocityLinear = _maxVelocityLinear;
      }

      /// Set the maximum allowed angular velocity for a physics object.
      void setMaxVelocityAngular( H3DFloat _maxVelocityAngular ) {
        update_bit_mask |= MAX_VELOCITY_ANGULAR;
        maxVelocityAngular = _maxVelocityAngular;
      }

      /// Get the world scale.
      H3DFloat getWorldScale () {
        return worldScale;
      }

      /// Get the current value of the fixed time step.
      H3DFloat getFixedTimeStep () {
        return fixedTimeStep;
      }

      /// Get the bullet collision shape margin
      H3DFloat getCollisionMargin () {
        return collisionMargin;
      }

      /// Get the value for the maximum allowed linear velocity.
      H3DFloat getMaxVelocityLinear() {
        return maxVelocityLinear;
      }

      /// Get the value for the maximum allowed angular velocity.
      H3DFloat getMaxVelocityAngular() {
        return maxVelocityAngular;
      }

      /// Returns true if the world scale has been specified.
      bool haveWorldScale () {
        return (update_bit_mask & WORLD_SCALE) != 0;
      }

      /// Returns true if a fixed time step has been specified.
      bool haveFixedTimeStep () {
        return (update_bit_mask & FIXED_TIME_STEP) != 0;
      }

      /// Returns true if the bullet collision shape margin has been specified
      bool haveCollisionMargin () {
        return (update_bit_mask & COLLISION_MARGIN) != 0;
      }

      /// Returns true if the maximum linear velocity has been specified
      bool haveMaxVelocityLinear() {
        return (update_bit_mask & MAX_VELOCITY_LINEAR) != 0;
      }

      /// Returns true if the maximum angular velocity has been specified
      bool haveMaxVelocityAngular() {
        return (update_bit_mask & MAX_VELOCITY_ANGULAR) != 0;
      }

    protected:
      static const unsigned int WORLD_SCALE = 0x0001;
      static const unsigned int FIXED_TIME_STEP = 0x0002;
      static const unsigned int COLLISION_MARGIN = 0x0004;
      static const unsigned int MAX_VELOCITY_LINEAR = 0x0008;
      static const unsigned int MAX_VELOCITY_ANGULAR = 0x0010;

      /// A scale for the entire bullet world.
      H3DFloat worldScale;
      /// The fixed time step that bullet should use, if any.
      H3DFloat fixedTimeStep;

      /// The bullet collision shape margin
      H3DFloat collisionMargin;

      /// The maximum linear velocity allowed by physics objects in bullet.
      H3DFloat maxVelocityLinear;
      /// The maximum angular velocity allowed by physics objects in bullet.
      H3DFloat maxVelocityAngular;
    };
  }

  /// \ingroup Bullet
  /// Node used to specify options relating to the simulation world that are specific to
  /// the Bullet physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an RigidBodyCollection node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/BulletOptions.x3d">BulletOptions.x3d</a>
  ///     ( <a href="examples/BulletOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile BulletWorldOptions.dot
  class H3DPHYS_API BulletWorldOptions : public H3DEngineOptions {
  public:
    /// Constructor.
    BulletWorldOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFFloat > _worldScale = 0,
      Inst< SFFloat > _fixedTimeStep = 0,
      Inst< SFFloat > _collisionMargin = 0,
      Inst< SFFloat > _maxVelocityLinear = 0,
      Inst< SFFloat > _maxVelocityAngular = 0 );

    /// Returns the string identifier of the physics engine that these options relate to.
    ///
    /// In the case of this node, this function returns "Bullet"
    virtual string getEngine () {
      return "Bullet";
    }

    /// A scaling factor to apply internally to the physics simulation
    ///
    /// The simulation world will be scaled by this factor before being passed
    /// to the Bullet physics engine for processing. The output parameters will
    /// then be scaled by the inverse of this factor as they are returned from 
    /// the physics engine.
    ///
    /// The purpose of this parameter is to provide a mechanism for controling
    /// the accuracy of the simulation irrespective of the desired scale of the
    /// simulation. Bullet works best within specific ranges of sizes and distances.
    /// Using the worldScale parameter you can bring the simulation world back within 
    /// these ranges, while externally the scale of the rendering of the simulation 
    /// appears unchanged.
    ///
    /// Gravity and external forces and torques are also scaled to compensate for
    /// the changes in distance.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1.0 \n
    /// 
    /// \dotfile BulletWorldOptions_worldScale.dot
    auto_ptr < SFFloat > worldScale;

    /// The fixed time step used when stepping the simulation
    ///
    /// A smaller value makes the simulation more accurate and helps improve collision detection.
    /// However a smaller value may reduce the physics loop rate and cause instabilities.
    ///
    /// fixedTimeStep * iterations (RigidBodyCollection field) must be greater or equal to the
    /// time duration between physics thread loops, otherwise the simulation will lose time.
    ///
    /// If no BulletWorldOptions node is specified, then the default fixed time step of 0.001 is used.
    ///
    /// The parameter also affects the characteristics of soft bodies and joints. Generally both
    /// are 'stiffer' when using a lower fixed time step.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.001 \n
    /// 
    /// \dotfile BulletWorldOptions_fixedTimeStep.dot
    auto_ptr < SFFloat > fixedTimeStep;

    /// The margin of the underlying bullet collision shape
    ///
    /// A larger value will help prevent interpenetration, but may result in
    /// unrealistically large separation of resting objects. The value is
    /// scaled by the worldScale field.
    ///
    /// If a collidable has a BulletCollidableOptions node, then it will override the
    /// value given here.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.01 \n
    /// 
    /// \dotfile BulletWorldOptions_collisionMargin.dot
    auto_ptr < SFFloat > collisionMargin;

    /// The maximum linear velocity of a rigid body
    ///
    /// Setting a maximum limit for velocity helps to maintain stability
    /// -1 means that there is no maximum to consider.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile BulletWorldOptions_maxVelocityLinear.dot
    auto_ptr < SFFloat > maxVelocityLinear;

    /// The maximum angular velocity of a rigid body
    ///
    /// Setting a maximum limit for velocity helps to maintain stability
    /// -1 means that there is no maximum to consider.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile BulletWorldOptions_maxVelocityAngular.dot
    auto_ptr < SFFloat > maxVelocityAngular;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
