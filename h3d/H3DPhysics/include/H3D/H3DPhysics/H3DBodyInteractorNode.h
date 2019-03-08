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
/// \file H3DBodyInteractorNode.h
/// \brief Header file for H3DBodyInteractorNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DBODYINTERACTORNODE__
#define __H3DBODYINTERACTORNODE__

#include <H3D/H3DPhysics/H3DBodyNode.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/H3DSoftBodyNode.h>
#include <H3D/SFNode.h>

namespace H3D{  

  /// \ingroup AbstractNodes H3DBodyInteractorNode
  /// Abstract base class for nodes used to interact with body such as
  /// constraints, attachments, joints as well as vertex forces,
  /// device modifiers etc.
  ///
  /// \par Internal routes:
  /// \dotfile H3DBodyInteractorNode.dot
  class H3DPHYS_API H3DBodyInteractorNode : public X3DNode {
  public:

    /// The SFRigidBody field type is an SFNode type that is
    /// constrained to only taking RigidBody nodes as field value.
    class H3DPHYS_API SFH3DBodyNode: public TypedSFNode< H3DBodyNode > {
    protected:
      /// When a RigidBody is added to field, this function checks
      /// if it has been initialized. If it is not, then it is
      /// initialized.
      virtual void onAdd( Node *n );
    };

    typedef SpecializedSFNode < RigidBody, SFH3DBodyNode > SFRigidBody;
    typedef SpecializedSFNode < H3DSoftBodyNode, SFH3DBodyNode > SFH3DSoftBodyNode;

    /// Constructor.
    H3DBodyInteractorNode(
      Inst< SFNode > _metadata = 0,
      Inst< SFH3DBodyNode > _body1 = 0 );

    /// Destructor
    virtual ~H3DBodyInteractorNode() {};

    /// Returns the default xml containerField attribute value.
    /// For this node it is "interactors".
    ///
    virtual string defaultXMLContainerField() {
      return "interactors";
    }

    /// Returns true if this interactor node has been initialized for the
    /// given PhysicsEngineThread.
    bool isInitialized();

    /// One of the bodies of the interaction.
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile H3DBodyInteractorNode_body1.dot
    auto_ptr< SFH3DBodyNode > body1;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// The PhysicsEngineThread with which this jiont is created
    H3D::PhysicsEngineThread * engine_thread;

    /// Bit mask representing the output fields requested for this joint
    unsigned int output_bit_mask;

    /// Virtual function that updates the output fields of the 
    /// H3DBodyInteractorNode, i.e. fields that give information about
    /// the current state of joints in the physics simulation.
    /// This should be overridden by all subclasses of H3DBodyInteractorNode
    /// that have output fields.
    virtual void updateOutputFields() {}

    /// Print warning that specified body is not initialized
    void uninitializedBodyWarning ( H3DBodyNode& body );

  };
}
#endif
