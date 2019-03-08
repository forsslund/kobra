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
/// \file PhysicsContainer.h
/// \brief Header file for PhysicsContainer, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PHYSICSCONTAINER__
#define __PHYSICSCONTAINER__

#include <H3D/X3DChildNode.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>
#include <H3D/H3DPhysics/H3DBodyConstraintNode.h>
#include <H3D/H3DPhysics/H3DBodyModifierNode.h>

namespace H3D{
  /// The purpose of the PhysicsContainer node is simply to allow a physics entity, e.g.
  /// a RigidBody or CollidableShape, to exist in an X3D scene without being active in a
  /// simulation. 
  ///
  /// This allows an X3D document to be defined that could, for example, be used
  /// to group graphics and physics elements that relate to a single entity in the application.
  /// That document could be used as a template to instatiate multiple instances of that entity.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/PhysicsContainer.x3d">PhysicsContainer.x3d</a>
  ///     ( <a href="examples/PhysicsContainer.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysicsContainer.dot
  class H3DPHYS_API PhysicsContainer : public X3DChildNode {
  public:
    typedef TypedMFNode < RigidBody > MFRigidBody;
    typedef TypedMFNode < H3DBodyConstraintNode > MFBodyConstraint;
    typedef TypedMFNode < X3DNBodyCollidableNode > MFCollidable;
    typedef TypedMFNode < H3DSoftBodyNode > MFSoftBody;
    typedef TypedMFNode < H3DBodyModifierNode > MFBodyModifier;

    /// Constructor.
    PhysicsContainer(
      Inst< SFNode  > _metadata = 0,
      Inst< MFRigidBody > _bodies = 0,
      Inst< MFBodyConstraint > _joints = 0,
      Inst< MFCollidable > _collidables= 0,
      Inst< MFSoftBody > _softBodies= 0,
      Inst< MFBodyModifier > _modifiers= 0 );

    /// The contained RigidBody nodes
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile PhysicsContainer_bodies.dot
    auto_ptr< MFRigidBody > bodies;

    /// The contained H3DBodyConstraintNode nodes
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile PhysicsContainer_joints.dot
    auto_ptr< MFBodyConstraint > joints;

    /// The contained X3DNBodyCollidableNode nodes
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile PhysicsContainer_collidables.dot
    auto_ptr< MFCollidable > collidables;

    /// The contained H3DSoftBodyNode nodes
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile PhysicsContainer_softBodies.dot
    auto_ptr< MFSoftBody > softBodies;

    /// The contained H3DBodyModifierNode nodes
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile PhysicsContainer_modifiers.dot
    auto_ptr< MFBodyModifier > modifiers;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
