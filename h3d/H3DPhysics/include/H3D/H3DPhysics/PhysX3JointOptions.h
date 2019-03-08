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
/// \file PhysX3JointOptions.h
/// \brief Header file for PhysX3JointOptions, implementing possible joint options nodes
/// handled by PhysX3.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSX3JOINTOPTIONS_H__
#define __PHYSX3JOINTOPTIONS_H__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/SFVec2f.h>

namespace H3D {

  /// \ingroup PhysX3
  /// Node used to specify options relating to a joint that are specific to
  /// the PhysX3 physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of a joint node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/DistanceJoint/PhysX3JointOptions.x3d">PhysX3JointOptions.x3d</a>
  ///     ( <a href="examples/PhysX3JointOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysX3JointOptions.dot
  class H3DPHYS_API PhysX3JointOptions : public H3DEngineOptions {
  public:
    /// Constructor.
    PhysX3JointOptions(
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFVec2f > _projectionTolerance = 0,
      Inst< SFString > _constraintFlag = 0,
      Inst< SFBool > _enabledProjection = 0 );


    /// ProjectionTolerance is similar to limits, however, applied to
    /// axis which are not limited but locked by the physices engine itself.
    ///
    /// For each locked degree of freedom, NOT limited, If the joint separates
    /// by more than this distance(or angle) along its locked degrees of freedom,
    /// the solver will move the bodies to close the distance (or angle).
    /// It applies to only if its values are greater than 0.
    ///
    /// Setting a very small tolerance may result in simulation jitter or other artifacts.
    /// Sometimes it is not possible to project, for example when there is a cyclic chain of rigid
    /// bodies linked by constraints.
    ///
    /// Use with caution! It is best avoided if practical, as setting the projection in cases of multiple
    /// complex chains of bodies might introduce bugs and unexpected behaviours.
    /// 
    /// x - linear tolerance, y - angular tolerance in degrees
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(1e10f , Constants::pi) \n
    /// <b>Valid range :</b>[0,3.4028234663852885981170418348452e+38F] for x, [0,pi] for y\n
    /// 
    /// \dotfile PhysX3JointOptions_projectionTolerance.dot
    auto_ptr < SFVec2f > projectionTolerance;

    /// The constraint flag value to set the joint to.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> "ePROJECTION" \n
    /// 
    /// \dotfile PhysX3JointOptions_constraintFlag.dot
    auto_ptr < SFString > constraintFlag;

    /// enabledProjection is True if the projection is enabled for the joint.
    /// 
    /// Use with caution! It is best avoided if practical, as setting the projection in cases of multiple
    /// complex chains of bodies might introduce bugs and unexpected behaviours.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> false \n
    /// 
    /// \dotfile PhysX3JointOptions_enabledProjection.dot
    auto_ptr < SFBool > enabledProjection;

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "PhysX3"
    virtual string getEngine() {
      return "PhysX3";
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );

  };

  /// \ingroup PhysX3
  /// Node used to specify options relating to a slider joint that are specific to
  /// the PhysX3 physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an SliderJoint node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/DistanceJoint/PhysX3JointOptions.x3d">PhysX3JointOptions.x3d</a>
  ///     ( <a href="examples/PhysX3JointOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysX3SliderJointOptions.dot
  class H3DPHYS_API PhysX3SliderJointOptions : public PhysX3JointOptions {
  public:

    /// Constructor.
    PhysX3SliderJointOptions(
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFVec3f > _explicitAnchorPoint = 0,
      Inst< SFVec3f > _body1Offset = 0,
      Inst< SFVec3f > _body2Offset = 0,
      Inst< SFFloat > _body2ForceScale = 0,
      Inst< SFString > _forceType = 0 );

    /// It is used only when the axis is being updated for a slider joint which
    /// is already in the simulation. When the slider joint is first added, it has
    /// no effect.
    /// 
    /// By default the anchor point for the slider joint is set to the mid
    /// point between the two bodies. One can overwrite the anchor point explicitly.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0.0f, 0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_explicitAnchorPoint.dot
    auto_ptr < SFVec3f > explicitAnchorPoint;

    /// It is used only when the axis is being updated for a slider joint which
    /// is already in the simulation. When the slider joint is first added, it has
    /// no effect.
    ///
    /// In case the axis of slider joint is changed, position of one of the bodies
    /// can be off than desired position. This parameter is the offset which needs
    /// to be added to the current body position s.t it would end up at the desired
    /// position.
    ///
    /// The slider joint forces this movement, for high offset values jumps and
    /// instabilities might occur.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0.0f, 0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_body1Offset.dot
    auto_ptr < SFVec3f > body1Offset;

    /// It is used only when the axis is being updated for a slider joint which
    /// is already in the simulation. When the slider joint is first added, it has
    /// no effect.
    ///
    /// In case the axis of slider joint is changed, position of one of the bodies
    /// can be off than desired position. This parameter is the offset which needs
    /// to be added to the current body position s.t it would end up at the desired
    /// position.
    ///
    /// The slider joint forces this movement, for high offset values jumps and
    /// instabilities might occur.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0.0f, 0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_body2Offset.dot
    auto_ptr < SFVec3f > body2Offset;

    /// It is used only when to apply different forces, by using sliderForce, to
    /// the bodies of the slider joint. sliderForce is applied to body1, and it is
    /// scaled before being applied to body2.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> H3DFloat(1.0f) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_body2ForceScale.dot
    auto_ptr < SFFloat > body2ForceScale;

    /// Determines how the sliderForce is interpreted while being applied by the engine.
    ///
    /// eFORCE,           //!< parameter has unit of mass * distance/ time^2, i.e. a force
    /// eIMPULSE,         //!< parameter has unit of mass * distance /time
    /// eVELOCITY_CHANGE, //!< parameter has unit of distance / time, i.e. the effect is mass independent: a velocity change.
    /// eACCELERATION     //!< parameter has unit of distance/ time^2, i.e. an acceleration. It gets treated just like a force except the mass is not divided out before integration.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> string(eFORCE) \n
    /// 
    /// \dotfile PhysX3SliderJointOptions_forceType.dot
    auto_ptr < SFString > forceType;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };

  /// \ingroup PhysX3
  /// Node used to specify options relating to a 6Dof joint that are specific to
  /// the PhysX3 physics engine implementation.
  ///
  /// Used to set limits to the joints in different axis of 6DOF.
  /// It could be either hard limits or soft limits, controlled by a spring.
  /// The axis for which the limit parameters are not set are not affected.
  ///
  /// Mostly usefeul in BallJoint, UniversalJoint and Generic6DofJoint.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/DistanceJoint/PhysX3JointOptions.x3d">PhysX3JointOptions.x3d</a>
  ///     ( <a href="examples/PhysX3JointOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysX3Joint6DOFLimitOptions.dot
  class H3DPHYS_API PhysX3Joint6DOFLimitOptions : public PhysX3JointOptions {
  public:

    /// Constructor.
    PhysX3Joint6DOFLimitOptions(
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFVec2f > _linear_x = 0,
      Inst< SFVec2f > _linear_y = 0,
      Inst< SFVec2f > _linear_z = 0,
      Inst< SFVec2f > _angular_x = 0,
      Inst< SFVec2f > _angular_y = 0,
      Inst< SFVec2f > _angular_z = 0,
      Inst< SFVec2f > _linearSpring = 0,
      Inst< SFVec2f > _angularSpring = 0,
      Inst< SFVec2f > _breakForce = 0,
      Inst< SFVec2f > _contactDistance = 0 );

    /// Limit on the translation along x-axis, (min,max)
    /// \note NOT IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linear_x.dot
    auto_ptr < SFVec2f > linear_x;

    /// Limit on the translation along y-axis, (min,max)
    /// \note NOT IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linear_y.dot
    auto_ptr < SFVec2f > linear_y;

    /// Limit on the translation along z-axis, (min,max)
    /// \note NOT IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linear_z.dot
    auto_ptr < SFVec2f > linear_z;

    /// Limit on the rotation around x-axis, (min,max)-degrees
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angular_x.dot
    auto_ptr < SFVec2f > angular_x;

    /// Limit on the rotation around y-axis, (min,max)-degrees
    /// \note NOT IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angular_y.dot
    auto_ptr < SFVec2f > angular_y;

    /// Limit on the rotation around z-axis, (min,max)-degrees
    /// \note NOT IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angular_z.dot
    auto_ptr < SFVec2f > angular_z;

    /// Instead of applying a hard limit one can apply a soft limit controlled
    /// by a spring. It applies to linear limits if its value is different
    /// than 0.
    /// 
    /// x - stiffness, y - daming of the spring
    ///
    /// \note NOT FULLY IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_linearSpring.dot
    auto_ptr < SFVec2f > linearSpring;

    /// Instead of applying a hard limit one can apply a soft limit controlled
    /// by a spring. It applies to angular limits if its value is different
    /// than 0.
    /// 
    /// x - stiffness, y - daming of the spring
    ///
    /// \note NOT FULLY IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.0f, 0.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_angularSpring.dot
    auto_ptr < SFVec2f > angularSpring;

    /// If the force or torque are greater than this value the joint is broken.
    ///
    /// x - max force, y - max torque
    ///
    /// \note NOT IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(-1.0f, -1.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_breakForce.dot
    auto_ptr < SFVec2f > breakForce;

    /// The distance from the upper or lower limit at which the limit constraint becomes
    /// active.
    ///
    /// x - linear distance, y - angular distance - degrees
    ///
    /// \note NOT FULLY IMPLEMENTED YET ON THE PHYSX3 SIDE
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f(0.01f, 5.0f) \n
    /// 
    /// \dotfile PhysX3Joint6DOFLimitOptions_contactDistance.dot
    auto_ptr < SFVec2f > contactDistance;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };

}
#endif
