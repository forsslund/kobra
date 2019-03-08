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
/// \file BulletAttachmentOptions.h
/// \brief Header file for BulletAttachmentOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __BULLETATTACHMENTOPTIONS__
#define __BULLETATTACHMENTOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/SFInt32.h>
#include <H3D/MFString.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup Bullet SoftBody
    /// Structure describing the state of a BulletAttachmentOptions node
    /// to be passed to the physics simulation thread
    struct BulletAttachmentParameters : public EngineOptionParameters {

      /// Constructor
      BulletAttachmentParameters () : 
    collide ( true ) {}

    // 'set' functions

    /// Set the distance used when generating contraints
    void setCollide( bool _collide ) {
      update_bit_mask|= COLLIDE;
      collide= _collide;
    }

    // 'get' functions

    /// Get the distance used when generating contraints
    bool getCollide() {
      return collide;
    }

    // 'have' functions

    /// Returns true if the distance used when generating contraints has been specified
    bool haveCollide () {
      return (update_bit_mask & COLLIDE) != 0;
    }

    protected:
      // update bit mask flags
      static const unsigned int COLLIDE = 0x0001;

      /// If true, then bodies in the attachment should inter-collide
      bool collide;
    };
  }

  /// \ingroup Bullet SoftBody
  /// Node used to specify options relating to an H3DSoftBodyNode that are
  /// specific to the Bullet physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an H3DSoftBodyNode
  /// node. These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/BulletAttachmentOptions.x3d">BulletAttachmentOptions.x3d</a>
  ///     ( <a href="examples/BulletAttachmentOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile BulletAttachmentOptions.dot
  class H3DPHYS_API BulletAttachmentOptions : public H3DEngineOptions {
  public:
    typedef EnumMField < MFString > MFCollisionOptions;

    /// Constructor.
    BulletAttachmentOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFBool > _collide = 0
      );

    /// Returns the string identifier of the physics engine that these options
    /// relate to. In the case of this node, this function returns "Bullet"
    virtual string getEngine () {
      return "Bullet";
    }

    /// A value of true indicates that the bodies involved in the attachment
    /// may collide with each other, otherwise the bodies will not collide.
    /// \note There seems to be no way to update this value in the bullet engine
    /// through the use of functions. Therefore we have to modify some bullet
    /// variable to update collision after initialization. If there are ever
    /// problems with this the problem could be caused by newer versions of bullet.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> TRUE \n
    /// 
    /// \dotfile BulletAttachmentOptions_collide.dot
    auto_ptr < SFBool > collide;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields.
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
