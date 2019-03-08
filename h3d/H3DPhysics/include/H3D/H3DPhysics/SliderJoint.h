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
/// \file SliderJoint.h
/// \brief Header file for SliderJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SLIDERJOINT__
#define __SLIDERJOINT__

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

  /// \ingroup X3DNodes
  /// \class SliderJoint
  /// \brief The SliderJoint node represents a joint where all movement 
  /// between the bodies specified by the body1 and body2 fields is 
  /// constrained to a single dimension along a user-defined axis.
  /// 
  /// The axis field indicates which axis along which the two bodies 
  /// will act. The value should represent a normalized vector.
  ///
  /// The sliderForce field value is used to apply a force along the axis
  /// of the slider in equal and opposite directions to the two bodies.
  ///
  /// If minSeparation is greater than maxSeparation, the stops become 
  /// ineffective as if the object has no stops at all.
  ///
  /// The separation output field is used to indicate the final separation
  /// of the two bodies.
  ///
  /// The separationRate output field is used to indicate the change in 
  /// separation over time since the last update.
  ///
  /// The stopBounce field describes how much the joint should bounce 
  /// the body back if the joint limit has been reached or exceeded. A 
  /// value of zero indicates no bounce at all, and a value of one indicates 
  /// that it should bounce with velocity equal and opposite to the collision
  /// velocity of the contact.
  ///
  /// The stopErrorCorrection field describes the amount of error correction
  /// to be performed in a time step when the joint reaches the limit. A 
  /// value of zero means no error correction is to be performed and a value
  /// of one means all error should be corrected in a single step.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/SliderJoint.x3d">SliderJoint.x3d</a>
  ///     ( <a href="examples/SliderJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SliderJoint.dot
  class H3DPHYS_API SliderJoint :  public H3DRigidBodyJointNode {
  public:
    /// Constructor.
    SliderJoint(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode   > _transform = 0,
      Inst< SFVec3f      > _axis = 0,
      Inst< SFFloat      > _maxSeparation = 0,
      Inst< SFFloat      > _minSeparation = 0,
      Inst< SFFloat      > _stopBounce = 0,
      Inst< SFFloat      > _stopErrorCorrection = 0,
      Inst< SFFloat      > _separation = 0,
      Inst< SFFloat      > _separationRate = 0,
      Inst< SFFloat      > _sliderForce = 0 );

    /// The vector along which the two bodies will act. 
    /// The value should represent a normalized vector.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0, 1, 0) \n
    /// 
    /// \dotfile SliderJoint_axis.dot
    auto_ptr< SFVec3f > axis;

    /// The maximum separation between the two bodies in the joint.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile SliderJoint_maxSeparation.dot
    auto_ptr< SFFloat > maxSeparation;

    /// The minimum separation between the two bodies in the joint.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile SliderJoint_minSeparation.dot
    auto_ptr< SFFloat > minSeparation;

    /// The amount at which the joint should bounce the body back if 
    /// the joint limit has been reached or exceeded. Zero value indicates
    /// no bounce at all, 1 indicates that it should bounce with velocity 
    /// equal and opposite to the collision velocity of the contact.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile SliderJoint_stopBounce.dot
    auto_ptr< SFFloat > stopBounce;

    /// The amount of error correction to be performed in a time step when
    /// the joint reaches the limit. Zero value indicates no error correction
    /// and 1 indicates that all error should be corrected in a single step.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile SliderJoint_stopErrorCorrection.dot
    auto_ptr< SFFloat > stopErrorCorrection;

    /// The final separation of the two bodies.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile SliderJoint_separation.dot
    auto_ptr< SFFloat > separation;

    /// The change in separation over time since the last update.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile SliderJoint_separationRate.dot
    auto_ptr< SFFloat > separationRate;

    /// Used to apply a force (specified in force base units) along the axis of
    /// the slider in equal and opposite directions to the two bodies. A
    /// positive value applies a force such that the two bodies accelerate
    /// away from each other while a negative value applies a force such that
    /// the two bodies accelerate toward each other.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile SliderJoint_sliderForce.dot
    auto_ptr< SFFloat > sliderForce;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns a new instance of 
    /// PhysicsEngineParameters::SliderJointParameters with the values updated with
    /// corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

    /// Updates the separation and separationRate fields if they are enabled 
    /// by the forceOutput field.
    void updateOutputFields();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );
  };
}
#endif
