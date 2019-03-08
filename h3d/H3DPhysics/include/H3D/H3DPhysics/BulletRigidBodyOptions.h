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
/// \file BulletRigidBodyOptions.h
/// \brief Header file for BulletRigidBodyOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __BULLETRIGIDBODYOPTIONS__
#define __BULLETRIGIDBODYOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/SFInt32.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup Bullet
    /// Structure describing the state of a BulletRigidBodyOptions node
    /// to be passed to the physics simulation thread
    struct BulletRigidBodyParameters : public EngineOptionParameters {

      struct SoftBodyCollisionOptions {
        enum e {
          DEFAULT,
          SDF_RIGIDSOFT,
          CLUSTER_RIGIDSOFT
        };
      };

      /// Constructor
      BulletRigidBodyParameters () : 
        collisionGroup ( 0 ),
        collidesWith ( 0 ),
        softBodyCollisionOptions ( SoftBodyCollisionOptions ::DEFAULT ) {}

      // 'set' functions

      void setCollisionGroup ( H3DInt32 _collisionGroup ) {
        update_bit_mask|= COLLISION_GROUP;
        collisionGroup= _collisionGroup;
      }

      void setCollidesWith ( H3DInt32 _collidesWith ) {
        update_bit_mask|= COLLIDES_WITH;
        collidesWith= _collidesWith;
      }

      void setSoftBodyCollisionOptions( SoftBodyCollisionOptions::e options ) {
        update_bit_mask |= SOFT_BODY_COLLISION_OPTIONS;
        softBodyCollisionOptions = options;
      }

      // 'get' functions

      H3DInt32 getCollisionGroup() {
        return collisionGroup;
      }

      H3DInt32 getCollidesWith () {
        return collidesWith;
      }

      SoftBodyCollisionOptions::e getSoftBodyCollisionOptions() {
        return softBodyCollisionOptions;
      }

      // 'have' functions

      bool haveCollisionGroup () {
        return (update_bit_mask & COLLISION_GROUP) != 0;
      }

      bool haveCollidesWith () {
        return (update_bit_mask & COLLIDES_WITH) != 0;
      }

      bool haveSoftBodyCollisionOptions() {
        return (update_bit_mask & SOFT_BODY_COLLISION_OPTIONS) != 0;
      }

    protected:
      // update bit mask flags
      static const unsigned int COLLISION_GROUP = 0x0001;
      static const unsigned int COLLIDES_WITH = 0x0002;
      static const unsigned int SOFT_BODY_COLLISION_OPTIONS = 0x0004;

      H3DInt32 collisionGroup;
      H3DInt32 collidesWith;

      SoftBodyCollisionOptions::e softBodyCollisionOptions;
    };
  }

  /// \ingroup Bullet
  /// Node used to specify options relating to a RigidBody that are specific to
  /// the Bullet physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an RigidBody node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/BulletRigidBodyOptions.x3d">BulletRigidBodyOptions.x3d</a>
  ///     ( <a href="examples/BulletRigidBodyOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile BulletRigidBodyOptions.dot
  class H3DPHYS_API BulletRigidBodyOptions :  public H3DEngineOptions {
  public:

    /// Constructor.
    BulletRigidBodyOptions (
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFInt32 > _collisionGroup = 0,
      Inst< SFInt32 > _collidesWith = 0,
      Inst< SFString > _softBodyCollisionOptions = 0 );

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "Bullet"
    virtual string getEngine () {
      return "Bullet";
    }


    /// The collisionGroup field is treated as a bit mask which specify the
    /// collision group for a RigidBody. All RigidBodies that belong to the
    /// same group should use the same number here. 0 Means that 
    /// the RigidBody is not part of a group and in that case collidesWith
    /// will be ignored and the RigidBody will behave as if no
    /// BulletRigidBodyOptions was provided.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile BulletRigidBodyOptions_collisionGroup.dot
    auto_ptr < SFInt32 > collisionGroup;

    /// The collidesWith field is treated as a bit mask in which each
    /// bit with a value of 1 represents a collisionGroup that a
    /// RigidBody using this BulletRigidBodyOptions node will collide with.
    /// If for examples collisionGroup = 1 and collidesWith = 1
    /// then all RigidBodies that use this BulletRigidBodyOptions node will
    /// collide with eachother.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile BulletRigidBodyOptions_collidesWith.dot
    auto_ptr < SFInt32 > collidesWith;

    /// A string used to identify the collision algorithm to use for collision between
    /// this body and any soft body.
    ///
    /// If specified this option overrides the option specified by the soft body.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "DEFAULT" \n
    /// <b>Valid values:</b> "CLUSTER_RIGIDSOFT", "SDF_RIGIDSOFT", "DEFAULT"
    /// 
    /// \dotfile BulletRigidBodyOptions_softBodyCollisionOptions.dot
    auto_ptr < SFString > softBodyCollisionOptions;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
