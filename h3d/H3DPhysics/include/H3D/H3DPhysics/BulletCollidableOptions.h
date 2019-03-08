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
/// \file BulletCollidableOptions.h
/// \brief Header file for BulletCollidableOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __BULLETCOLLIDABLEOPTIONS__
#define __BULLETCOLLIDABLEOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup Bullet
    /// Structure describing the state of a BulletCollidableOptions node
    /// to be passed to the physics simulation thread
    struct BulletCollidableParameters : public EngineOptionParameters {

      /// Constructor
      BulletCollidableParameters () : 
    collisionMargin ( 0.0f ),
      convex ( false ) {}

    // 'set' functions

    /// Set the bullet collision shape margin
    void setCollisionMargin ( H3DFloat _collisionMargin ) {
      update_bit_mask|= COLLISION_MARGIN;
      collisionMargin= _collisionMargin;
    }

    /// Set whether the shape is convex
    void setConvex ( bool _convex) {
      update_bit_mask|= CONVEX;
      convex = _convex;
    }

    // 'get' functions

    /// Get the bullet collision shape margin
    H3DFloat getCollisionMargin () {
      return collisionMargin;
    }

    /// Returns true if the shape is convex
    bool getConvex () {
      return convex;
    }

    // 'have' functions

    /// Returns true if the bullet collision shape margin has been specified
    bool haveCollisionMargin () {
      return (update_bit_mask & COLLISION_MARGIN) != 0;
    }

    /// Returns true if the convex property has been specified
    bool haveConvex () {
      return (update_bit_mask & CONVEX) != 0;
    }

    protected:
      // update bit mask flags
      static const unsigned int COLLISION_MARGIN = 0x0001;
      static const unsigned int CONVEX          = 0x0002;

      /// The bullet collision shape margin
      H3DFloat collisionMargin;

      /// Is the geometry convex?
      bool convex;
    };
  }

  /// \ingroup Bullet
  /// Node used to specify options relating to a collidable that are specific to
  /// the Bullet physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an X3DNBodyCollidable
  /// node. These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/BulletOptions.x3d">BulletOptions.x3d</a>
  ///     ( <a href="examples/BulletOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile BulletCollidableOptions.dot
  class H3DPHYS_API BulletCollidableOptions : public H3DEngineOptions {
  public:
    /// Constructor.
    BulletCollidableOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFFloat > _collisionMargin = 0,
      Inst< SFBool > _convex = 0
      );

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "Bullet"
    virtual string getEngine () {
      return "Bullet";
    }

    /// The margin of the underlying bullet collision shape
    ///
    /// A larger value will help prevent interpenetration, but may result in
    /// unrealistically large separation of resting objects. The value is
    /// scaled by the worldScale field of the BulletWorldOptions node if specified.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.01 \n
    /// 
    /// \dotfile BulletCollidableOptions_collisionMargin.dot
    auto_ptr < SFFloat > collisionMargin;

    /// Specifies whether the collidable is convex or concave.
    ///
    /// If convex, then optimizations can be made during collision detection.
    /// Only applicable if the geometry type is handled as a triangle mesh. e.g.
    /// Not a Cube, Sphere or Cylinder.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> false \n
    /// 
    /// \dotfile BulletCollidableOptions_collisionMargin.dot
    auto_ptr < SFBool > convex;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
