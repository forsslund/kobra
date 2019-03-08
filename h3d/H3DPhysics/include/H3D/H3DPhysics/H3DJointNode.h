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
/// \file H3DJointNode.h
/// \brief Header file for H3DJointNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DJOINTNODE__
#define __H3DJOINTNODE__

#include <H3D/H3DPhysics/H3DBodyConstraintNode.h>
#include <H3D/MatrixTransform.h>

namespace H3D{

  /// \ingroup AbstractNodes
  /// \class H3DJointNode
  /// \brief  The H3DJointNode abstract node type is the base type for
  /// all joint types
  ///
  /// Because computers are not guaranteed to be accurate in their mathematical
  /// calculations and because of the nature of the discrete time steps 
  /// in the evaluation mechanisms, the behaviour of the system will not be
  /// 100% accurate. 
  ///
  /// Every joint type will have a set of joint-specific fields that define
  /// a set of error correction conditions. This error correction conditions
  /// provide guidance as to how to automatically correct for internally
  /// calculated errors including such errors as object interpenetration. 
  /// In addition, these error correction conditions can be used to control
  /// how quickly the errors should be corrected. Fast corrections may not 
  /// always be desirable for the appropriate visual output required.
  ///
  /// \par Internal routes:
  /// \dotfile H3DJointNode.dot
  class H3DPHYS_API H3DJointNode : public H3DBodyConstraintNode {
  public:

    typedef TypedSFNode < MatrixTransform > SFTransformNode;

    /// Constructor.
    H3DJointNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< SFH3DBodyNode  > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform = 0 );

    /// Initialize the constraint for the given PhysicsEngineThread. I.e. 
    /// create a new constraint in the physics engine with the parameters
    /// of the constraint fields. Returns 0 on success.
    virtual bool initializeConstraint( H3D::PhysicsEngineThread& pt );

    /// The second body the joint is connected to.
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile H3DJointNode_body2.dot
    auto_ptr< SFH3DBodyNode > body2;

    /// An initial transformation applied to all joint parameters
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile H3DJointNode_transform.dot
    auto_ptr< SFTransformNode > transform;

    /// Returns the default xml containerField attribute value.
    /// For this node it is "joints".
    ///
    virtual string defaultXMLContainerField() {
      return "joints";
    }

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

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform ) {};
  };
}
#endif
