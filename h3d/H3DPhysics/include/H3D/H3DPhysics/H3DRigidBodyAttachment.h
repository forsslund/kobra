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
/// \file H3DRigidBodyAttachment.h
/// \brief Header file for H3DRigidBodyAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DRIGIDBODYATTACHMENT__
#define __H3DRIGIDBODYATTACHMENT__

#include <H3D/H3DPhysics/H3DAttachmentNode.h>

namespace H3D{  

  /// A H3DRigidBodyAttachment allows vertices of an H3DSoftBodyNode to be attached to
  /// a RigidBody.
  ///
  /// \note This node is named RigidBodyAttachment in the node database (X3D level).
  /// Using H3DRigidBodyAttachment at X3D level is deprecated and will be removed in
  /// the future.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/RigidBodyAttachment.x3d">RigidBodyAttachment.x3d</a>
  ///     ( <a href="examples/RigidBodyAttachment.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile H3DRigidBodyAttachment.dot
  class H3DPHYS_API H3DRigidBodyAttachment : public H3DAttachmentNode {
  public:

    /// Constructor.
    H3DRigidBodyAttachment ( 
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DSoftBodyNode > _body1 = 0,      
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< TrackedMFInt32 > _index = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of 
    /// PhysicsEngineParameters::H3DRigidBodyAttachmentParameters with the values updated with
    /// corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

  };
}

#endif
