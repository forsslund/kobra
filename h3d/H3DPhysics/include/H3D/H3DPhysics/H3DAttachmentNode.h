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
/// \file H3DAttachmentNode.h
/// \brief Header file for H3DAttachmentNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DATTACHMENTNODE__
#define __H3DATTACHMENTNODE__

#include <H3D/H3DPhysics/H3DBodyConstraintNode.h>

namespace H3D{  

  /// \ingroup AbstractNodes SoftBody
  /// An H3DAttachmentNode allows vertices of an H3DSoftBodyNode to be attached to
  /// another body.
  ///
  /// \par Internal routes:
  /// \dotfile H3DAttachmentNode.dot
  class H3DPHYS_API H3DAttachmentNode : public H3DBodyConstraintNode {
  public:
    typedef TrackedMField < MFInt32 > TrackedMFInt32;

    /// Constructor.
    H3DAttachmentNode ( 
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DSoftBodyNode > _body1 = 0,      
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFH3DBodyNode > _body2 = 0,
      Inst< TrackedMFInt32 > _index = 0 );

    /// The second body to which body1 will be attached
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DAttachmentNode_body2.dot
    auto_ptr < SFH3DBodyNode > body2;

    /// Indices of coordinates in body1 which will be attached
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile H3DAttachmentNode_index.dot
    auto_ptr < TrackedMFInt32 > index;

    /// Initialize the constraint for the given PhysicsEngineThread. I.e. 
    /// create a new constraint in the physics engine with the parameters
    /// of the constraint fields. Returns 0 on success. Overrides the
    /// initializeConstraint function of the H3DBodyConstraint in order
    /// to check whether the engineThread is softBodyPhysicsEngineThread
    /// or not.
    virtual bool initializeConstraint( H3D::PhysicsEngineThread& pt );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Virtual function that returns a new instance of a subclass of
    /// PhysicsEngineParameters::ConstraintParameters that describes the 
    /// constraint. This should be overridden in each subclass of H3DBodyConstraintNode
    /// so that it returns the parameters for the constraint that is implemented.
    /// If all_params is true then it returns all field values regardless of whether
    /// the values have changed
    virtual PhysicsEngineParameters::ConstraintParameters *getConstraintParameters( bool all_params = false );
  };
}

#endif
