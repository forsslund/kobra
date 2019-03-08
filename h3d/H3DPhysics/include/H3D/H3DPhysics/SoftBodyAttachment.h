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
/// \file SoftBodyAttachment.h
/// \brief Header file for SoftBodyAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SOFTBODYATTACHMENT__
#define __SOFTBODYATTACHMENT__

#include <H3D/H3DPhysics/H3DSoftBodyAttachment.h>
#include <H3D/H3DPhysics/FieldTemplates.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// A SoftBodyAttachment allows vertices of an H3DSoftBodyNode to be attached to
  /// a vertices of another H3DSoftBodyNode.
  ///
  /// To efficiently add and remove links between the two bodies the *Tracked() function
  /// of the index and index2 fields should be used. The physics engine may be able to 
  /// optimize the changes if these functions are used instead of setValue(). If not then
  /// the changes will be equivalent to having called setValue().
  ///
  /// Physics engine notes:
  /// <ul>
  ///   <li><b>Bullet</b>: Tracked insertions, updates and deletions are handled.</li>
  /// </ul>
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/SoftBodyAttachment.x3d">SoftBodyAttachment.x3d</a>
  ///     ( <a href="examples/SoftBodyAttachment.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SoftBodyAttachment.dot
  class H3DPHYS_API SoftBodyAttachment : public H3DSoftBodyAttachment {
    public:

    /// The SFH3DPhysicsMaterialNode is dependent on the
    /// materialChanged field of the contained H3DPhysicsMaterialNode.
    typedef DependentSFNode< FieldRef<H3DPhysicsMaterialNode,
      Field,
      &H3DPhysicsMaterialNode::materialChanged > > 
      SFH3DPhysicsMaterialNode;

    /// Constructor.
    SoftBodyAttachment (
              Inst< SFNode > _metadata = 0,
              Inst< ValueUpdater > _valueUpdater = 0,
              Inst< SFH3DSoftBodyNode > _body1 = 0,
              Inst< MFString     > _forceOutput = 0,
              Inst< MFEngineOptions > _engineOptions = 0,
              Inst< SFH3DSoftBodyNode > _body2 = 0,
              Inst< TrackedMFInt32 > _index = 0,
              Inst< TrackedMFInt32 > _index2 = 0,
              Inst< SFH3DPhysicsMaterialNode > _physicsMaterial = 0
              );

    /// Defines the properties of the material used to connect the bodies.
    ///
    /// The H3DPhysicsMaterialNode including different types of
    /// H3DPhysicsMaterialPropertyNodes like mass, damping, elasticity
    /// etc.
    ///
    /// Physics engine notes:
    /// <ul>
    ///   <li><b>Bullet</b>: A MassSpringPhysicsMaterial should be used and 
    ///                      only the stiffness property of the material is used.</li>
    /// </ul>
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile SoftBodyAttachment_physicsMaterial.dot
    auto_ptr < SFH3DPhysicsMaterialNode > physicsMaterial;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:

    /// Returns a new concrete instance of H3DAttachmentParameters appropriate for this subtype of H3DAttachmentNode
    ///
    /// For this node it returns a new instance of SoftBodyAttachmentParameters
    virtual PhysicsEngineParameters::SoftBodyAttachmentParameters* createConstraintParameters ();

    /// Returns a new concrete instance of H3DAttachmentParameters appropriate for this subtype of H3DAttachmentNode
    /// and populated with values that reflect the current state of this node
    ///
    /// For this node it returns a new instance of SoftBodyAttachmentParameters
    virtual PhysicsEngineParameters::SoftBodyAttachmentParameters* getConstraintParameters( bool all_params = false );

    /// Variables used to create all parameters of the node incase the whole
    /// node is changed.
    H3DPhysicsMaterialNode *previousPhysicsMaterial;
  };
}

#endif
