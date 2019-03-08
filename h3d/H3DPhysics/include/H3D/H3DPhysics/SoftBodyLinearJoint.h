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
/// \file SoftBodyLinearJoint.h
/// \brief Header file for SoftBodyLinearJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SOFTBODYLINEARJOINT__
#define __SOFTBODYLINEARJOINT__

#include <H3D/H3DPhysics/H3DSoftBodyJointNode.h>

namespace H3D{

  /// \ingroup SoftBody
  /// A SoftBodyLinearJoint is a joint that allows an H3DSoftBodyNode to be
  /// linked by a common anchor point to an H3DBodyNode (i.e., either a RigidBody or another
  /// H3DSoftBodyNode).
  ///
  /// The SoftBodyLinearJoint is the equivalent of a BallJoint for soft bodies.
  ///
  /// \par Limitations:
  /// It is unlikely that this joint can be supported in physics engines other than Bullet.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/SoftBodyLinearJoint.x3d">SoftBodyLinearJoint.x3d</a>
  ///     ( <a href="examples/SoftBodyLinearJoint.x3d.html">Source</a> )
  ///   - <a href="../../examples/softbody/SoftBodyLinearJoint_Rigid.x3d">SoftBodyLinearJoint_Rigid.x3d</a>
  ///     ( <a href="examples/SoftBodyLinearJoint_Rigid.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SoftBodyLinearJoint.dot
  class H3DPHYS_API SoftBodyLinearJoint : public H3DSoftBodyJointNode {
  public:

    /// Constructor.
    SoftBodyLinearJoint(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DSoftBodyNode > _body1 = 0,
      Inst< SFH3DBodyNode > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFVec3f           > _anchorPoint = 0,
      Inst< SFTransformNode > _transform = 0 );

    /// The anchor point of the ball joint
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> Vec3f () \n
    /// 
    /// \dotfile SoftBodyLinearJoint_anchorPoint.dot
    auto_ptr < SFVec3f > anchorPoint;

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

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );

  };
}
#endif
