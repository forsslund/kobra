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
/// \file DistanceJoint.h
/// \brief Header file for DistanceJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __DISTANCEJOINT__
#define __DISTANCEJOINT__

#include <H3D/X3DNode.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFFloat.h>
#include <H3D/MFString.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>

// H3DUtil includes
#include <H3DUtil/AutoPtrVector.h>

namespace H3D {

  /// \ingroup X3DNodes RigidBody
  /// \class DistanceJoint
  /// \brief The DistanceJoint node represents joint keeping the distance between
  /// two bodies within a specified range.
  ///
  /// It is practically a spring between the two bodies. It, however, affects solution
  /// of constraints in the physics engine rather than externally applying forces to the
  /// bodies.
  /// 
  /// Implemented in PhysX3 only.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/DistanceJoint/DistanceJoint.x3d">DistanceJoint.x3d</a>
  ///     ( <a href="examples/DistanceJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile DistanceJoint.dot
  class H3DPHYS_API DistanceJoint : public H3DRigidBodyJointNode {
  public:

    /// Constructor.
    DistanceJoint( Inst< SFNode > _metadata = 0,
                   Inst< ValueUpdater > _valueUpdater = 0,
                   Inst< SFRigidBody > _body1 = 0,
                   Inst< SFRigidBody > _body2 = 0,
                   Inst< MFString     > _forceOutput = 0,
                   Inst< MFEngineOptions > _engineOptions = 0,
                   Inst< SFTransformNode   > _transform = 0,
                   Inst< SFFloat  > _maxDistance = 0,
                   Inst< SFFloat  > _minDistance = 0,
                   Inst< SFFloat  > _stiffness = 0,
                   Inst< SFFloat  > _damping = 0,
                   Inst< SFFloat  > _distance = 0,
                   Inst< SFFloat  > _tolerance = 0 );

    /// Maximum distance allowed between the two bodies.
    /// Set to a negative value to disable maximum distance limit.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.1 \n
    /// 
    /// \dotfile DistanceJoint_maxDistance.dot
    auto_ptr< SFFloat > maxDistance;

    /// Minimum distance allowed between the two bodies.
    /// Set to a negative value to disable minimum distance limit.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.0 \n
    /// 
    /// \dotfile DistanceJoint_minDistance.dot
    auto_ptr< SFFloat > minDistance;

    /// Stiffness of the spring between the two bodies.
    /// Set to a negative value to disable spring.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 100 \n
    /// <b>Valid range:</b> [0-inf]
    /// 
    /// \dotfile DistanceJoint_stiffness.dot
    auto_ptr< SFFloat > stiffness;

    /// Damping of the spring between the two bodies.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.1 \n
    /// <b>Valid range:</b> [0-inf]
    /// 
    /// \dotfile DistanceJoint_damping.dot
    auto_ptr< SFFloat > damping;

    /// Distance between the two bodies.
    /// Only updated if forceOutput is "ALL".
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DistanceJoint_distance.dot
    auto_ptr< SFFloat > distance;

    /// The error tolerance of the joint. If the
    /// distance between the bodies exceeds the limits
    /// plus this tolerance then the joint activates.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile DistanceJoint_tolerance.dot
    auto_ptr< SFFloat > tolerance;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of 
    /// PhysicsEngineParameters::DistanceJointParameters with the values updated
    /// with corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless
    /// of whether the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters *getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate
    /// for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters();

    /// Updates the distance
    /// fields if they are enabled by the forceOutput field.
    void updateOutputFields();

  };
}
#endif
