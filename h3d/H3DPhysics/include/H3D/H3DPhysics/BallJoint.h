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
/// \file BallJoint.h
/// \brief Header file for BallJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __BALLJOINT__
#define __BALLJOINT__

#include <H3D/X3DNode.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFFloat.h>
#include <H3D/MFString.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>

// H3DUtil includes
#include <H3DUtil/AutoPtrVector.h>

namespace H3D{

  /// \ingroup X3DNodes RigidBody
  /// \class BallJoint
  /// \brief The BallJoint node represents an unconstrained joint between two 
  /// bodies that pivot about a common anchor point.
  ///
  /// body1AnchorPoint and body2AnchorPoint represent the output that describes 
  /// where the anchorPoint is relative to the two bodies local coordinate 
  /// reference frame. This can be used to detect if the joint has caused a 
  /// separation if the two values are not the same for a given frame.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/BallJoint.x3d">BallJoint.x3d</a>
  ///     ( <a href="examples/BallJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile BallJoint.dot
  class H3DPHYS_API BallJoint :  public H3DRigidBodyJointNode {
  public:

    /// Constructor.
    BallJoint( Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode   > _transform = 0,
      Inst< SFVec3f  > _anchorPoint = 0,
      Inst< SFVec3f   > _body1AnchorPoint = 0,
      Inst< SFVec3f   > _body2AnchorPoint = 0 );


    /// The point where the joint is centered in world coordinates.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile BallJoint_anchorPoint.dot
    auto_ptr< SFVec3f > anchorPoint;

    /// The body1AnchorPoint output field reports the current location of the 
    /// anchor point relative to body1.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile BallJoint_body1AnchorPoint.dot
    auto_ptr< SFVec3f > body1AnchorPoint;

    /// The body2AnchorPoint output field reports the current location of the 
    /// anchor point relative to body2.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile BallJoint_body2AnchorPoint.dot
    auto_ptr< SFVec3f > body2AnchorPoint;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of 
    /// PhysicsEngineParameters::BallJointParameters with the values updated
    /// with corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless
    /// of whether the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters *getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate
    /// for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

    /// Updates the body1AnchorPoint and body2AnchorPoint
    /// fields if they are enabled by the forceOutput field.
    void updateOutputFields();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );
  };
}
#endif
