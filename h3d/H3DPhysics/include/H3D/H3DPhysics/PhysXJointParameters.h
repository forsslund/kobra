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
/// \file PhysXJointParameters.h
/// \brief Header file for PhysX implementation of joints parameters.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSXJOINTPARAMETERS_H__
#define __PHYSXJOINTPARAMETERS_H__

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup PhysX
    /// Structure describing the state of a PhysXJointParameters node
    /// to be passed to the physics simulation thread
    struct PhysXJointParameters : public EngineOptionParameters {
      /// Constructor
      PhysXJointParameters() :
        projection_tolerance( Vec2f() ), projection_flag( "ePROJECTION" ),
        enabled_projection( false ), enabled_collision( false ), EngineOptionParameters() {
      }

      // 'set' functions

      void setProjectionTolerance( Vec2f _projection_tolerance ) {
        update_bit_mask |= PROJECTION_TOLERANCE;
        projection_tolerance = _projection_tolerance;
      }

      void setProjectionFlag( string _projection_flag ) {
        update_bit_mask |= PROJECTION_FLAG;
        projection_flag = _projection_flag;
      }

      void setEnabledProjection( bool _enabled_projection ) {
        update_bit_mask |= ENABLED_PROJECTION;
        enabled_projection = _enabled_projection;
      }

      void setEnabledCollision( bool _enabled_collision ) {
        update_bit_mask |= ENABLED_COLLISION;
        enabled_collision = _enabled_collision;
      }

      // 'get' functions

      Vec2f getProjectionTolerance() {
        return projection_tolerance;
      }

      string getProjectionFlag() {
        return projection_flag;
      }

      bool getEnabledProjection() {
        return enabled_projection;
      }

      bool getEnabledCollision() {
        return enabled_collision;
      }

      // 'have' functions

      bool haveProjectionTolerance() {
        return (update_bit_mask & PROJECTION_TOLERANCE) != 0;
      }

      bool haveProjectionFlag() {
        return (update_bit_mask & PROJECTION_FLAG) != 0;
      }

      bool haveEnabledProjection() {
        return (update_bit_mask & ENABLED_PROJECTION) != 0;
      }

      bool haveEnabledCollision() {
        return (update_bit_mask & ENABLED_COLLISION) != 0;
      }

    protected:
      // update bit mask flags
      static const unsigned int PROJECTION_TOLERANCE = 0x0001;
      static const unsigned int PROJECTION_FLAG = 0x0002;
      static const unsigned int ENABLED_PROJECTION = 0x0004;
      static const unsigned int ENABLED_COLLISION = 0x0008;

      Vec2f projection_tolerance;
      string projection_flag;
      bool enabled_projection;
      bool enabled_collision;

    public:
      /// \cond MAKE_SURE_DOXYGEN_IS_NOT_FOOLED_BY_MACRO
      DEPRECATED( "setProjectionFlag", )
      /// \endcond
      void setConstraintFlag( string _constraint_flag ) {
        setProjectionFlag( _constraint_flag );
      }

      /// \cond MAKE_SURE_DOXYGEN_IS_NOT_FOOLED_BY_MACRO
      DEPRECATED( "getProjectionFlag", )
      /// \endcond
      string getConstraintFlag() {
        return getProjectionFlag();
      }

      /// \cond MAKE_SURE_DOXYGEN_IS_NOT_FOOLED_BY_MACRO
      DEPRECATED( "haveProjectionFlag", )
      /// \endcond
      bool haveConstraintFlag() {
        return haveProjectionFlag();
      }
    };

    /// \ingroup PhysX
    /// Structure describing the state of a PhysXJoint6DOFLimitParameters node
    /// to be passed to the physics simulation thread
    struct PhysXJoint6DOFLimitParameters : public PhysXJointParameters {
      /// Constructor
      PhysXJoint6DOFLimitParameters() :
        linear_x( Vec2f() ), linear_y( Vec2f() ), linear_z( Vec2f() ),
        angular_x( Vec2f() ), angular_y( Vec2f() ), angular_z( Vec2f() ),
        linear_spring( Vec2f() ), angular_spring( Vec2f() ), PhysXJointParameters() {}

      // 'set' functions

      void setLinearX ( Vec2f _linear_x ) {
        update_bit_mask|= LINEAR_X;
        linear_x = _linear_x;
      }

      void setLinearY ( Vec2f _linear_y ) {
        update_bit_mask|= LINEAR_Y;
        linear_y = _linear_y;
      }

      void setLinearZ ( Vec2f _linear_z ) {
        update_bit_mask|= LINEAR_Z;
        linear_z = _linear_z;
      }

      void setAngularX ( Vec2f _angular_x ) {
        update_bit_mask|= ANGULAR_X;
        angular_x = _angular_x;
      }

      void setAngularY ( Vec2f _angular_y ) {
        update_bit_mask|= ANGULAR_Y;
        angular_y = _angular_y;
      }

      void setAngularZ ( Vec2f _angular_z ) {
        update_bit_mask|= ANGULAR_Z;
        angular_z = _angular_z;
      }

      void setLinearSpring ( Vec2f _linear_spring ) {
        update_bit_mask|= LINEAR_SPRING;
        linear_spring = _linear_spring;
      }

      void setAngularSpring ( Vec2f _angular_spring ) {
        update_bit_mask|= ANGULAR_SPRING;
        angular_spring = _angular_spring;
      }

      void setBreakForce ( Vec2f _break_force ) {
        update_bit_mask|= BREAK_FORCE;
        break_force = _break_force;
      }

      void setContactDistance ( Vec2f _contact_distance ) {
        update_bit_mask|= CONTACT_DISTANCE;
        contact_distance = _contact_distance;
      }

      // 'get' functions

      Vec2f getLinearX () {
        return linear_x;
      }

      Vec2f getLinearY () {
        return linear_y;
      }

      Vec2f getLinearZ () {
        return linear_z;
      }

      Vec2f getAngularX () {
        return angular_x;
      }

      Vec2f getAngularY () {
        return angular_y;
      }

      Vec2f getAngularZ () {
        return angular_z;
      }

      Vec2f getLinearSpring () {
        return linear_spring;
      }

      Vec2f getAngularSpring () {
        return angular_spring;
      }

      Vec2f getBreakForce() {
        return break_force;
      }
      
      Vec2f getContactDistance() {
        return contact_distance;
      }

      // 'have' functions

      bool haveLinearX() {
        return (update_bit_mask & LINEAR_X) != 0;
      }

      bool haveLinearY() {
        return (update_bit_mask & LINEAR_Y) != 0;
      }

      bool haveLinearZ() {
        return (update_bit_mask & LINEAR_Z) != 0;
      }

      bool haveAngularX() {
        return (update_bit_mask & ANGULAR_X) != 0;
      }

      bool haveAngularY() {
        return (update_bit_mask & ANGULAR_Y) != 0;
      }

      bool haveAngularZ() {
        return (update_bit_mask & ANGULAR_Z) != 0;
      }

      bool haveLinearSpring() {
        return (update_bit_mask & LINEAR_SPRING) != 0;
      }

      bool haveAngularSpring() {
        return (update_bit_mask & ANGULAR_SPRING) != 0;
      }

      bool haveBreakForce() {
        return (update_bit_mask & BREAK_FORCE) != 0;
      }

      bool haveContactDistance() {
        return (update_bit_mask & CONTACT_DISTANCE) != 0;
      }

      protected:
        // update bit mask flags
        static const unsigned int LINEAR_X            = 0x0010;
        static const unsigned int LINEAR_Y            = 0x0020;
        static const unsigned int LINEAR_Z            = 0x0040;
        static const unsigned int ANGULAR_X           = 0x0080;
        static const unsigned int ANGULAR_Y           = 0x0100;
        static const unsigned int ANGULAR_Z           = 0x0200;
        static const unsigned int LINEAR_SPRING       = 0x0400;
        static const unsigned int ANGULAR_SPRING      = 0x0800;
        static const unsigned int BREAK_FORCE         = 0x1000;
        static const unsigned int CONTACT_DISTANCE    = 0x2000;
        
        Vec2f linear_x;
        Vec2f linear_y;
        Vec2f linear_z;
        Vec2f angular_x;
        Vec2f angular_y;
        Vec2f angular_z;

        Vec2f linear_spring;
        Vec2f angular_spring;
        Vec2f break_force;
        Vec2f contact_distance;

    };

    /// \ingroup PhysX
    /// Structure describing the state of a PhysXSliderJointParameters node
    /// to be passed to the physics simulation thread
    struct PhysXSliderJointParameters : public PhysXJointParameters {
      /// Constructor
      PhysXSliderJointParameters() : explicitAnchorPoint( Vec3f() ),
        body1Offset( Vec3f() ), body2Offset( Vec3f() ), body2ForceScale( 1.0 ),
        forceType("eForce"),  PhysXJointParameters() {}


      // 'set' functions

      void setExplicitAnchorPoint ( Vec3f _explicitAnchorPoint ) {
        update_bit_mask|= EXPLICIT_ANCHOR_POINT;
        explicitAnchorPoint = _explicitAnchorPoint;
      }

      void setBody1Offset ( Vec3f _body1Offset ) {
        update_bit_mask|= BODY1_OFFSET;
        body1Offset = _body1Offset;
      }

      void setBody2Offset ( Vec3f _body2Offset ) {
        update_bit_mask|= BODY2_OFFSET;
        body2Offset = _body2Offset;
      }

      void setBody2ForceScale ( float _forceScale ) {
        update_bit_mask|= BODY2_FORCESCALE;
        body2ForceScale = _forceScale;
      }

      void setForceType ( string _forceType ) {
        update_bit_mask|= FORCETYPE;
        forceType = _forceType;
      }

      // 'get' functions

      Vec3f getExplicitAnchorPoint () {
        return explicitAnchorPoint;
      }

      Vec3f getBody1Offset () {
        return body1Offset;
      }

      Vec3f getBody2Offset () {
        return body2Offset;
      }

      H3DFloat getBody2ForceScale () {
        return body2ForceScale;
      }

      string getForceType () {
        return forceType;
      }

      // 'have' functions

      bool haveExplicitAnchorPoint() {
        return (update_bit_mask & EXPLICIT_ANCHOR_POINT) != 0;
      }

      bool haveBody1Offset () {
        return (update_bit_mask & BODY1_OFFSET) != 0;
      }

      bool haveBody2Offset () {
        return (update_bit_mask & BODY2_OFFSET) != 0;
      }

      bool haveBody2ForceScale () {
        return (update_bit_mask & BODY2_FORCESCALE) != 0;
      }

      bool haveForceType () {
        return (update_bit_mask & FORCETYPE) != 0;
      }

      protected:
        // update bit mask flags
        static const unsigned int EXPLICIT_ANCHOR_POINT   = 0x0010;
        static const unsigned int BODY1_OFFSET            = 0x0020;
        static const unsigned int BODY2_OFFSET            = 0x0040;
        static const unsigned int BODY2_FORCESCALE        = 0x0080;
        static const unsigned int FORCETYPE               = 0x0100;

        Vec3f explicitAnchorPoint;
        Vec3f body1Offset;
        Vec3f body2Offset;
        H3DFloat body2ForceScale;
        string forceType;

    };

    /// \ingroup PhysX
    /// Structure describing the state of a PhysXSingleAxisHingeJointParameter node
    /// to be passed to the physics simulation thread.
    /// Can be used to set drive/motor functionality to the joint to make it reach a certain
    /// angular velocity.
    struct PhysXSingleAxisHingeJointParameter : public PhysXJointParameters {
      /// Constructor
      PhysXSingleAxisHingeJointParameter() : drive_velocity( 0.f ), enabled_drive( false ), drive_force_limit( -1 ) {
      }

      // 'set' functions

      void setDriveVelocity( H3DFloat _drive_velocity ) {
        update_bit_mask |= DRIVE_VELOCITY;
        drive_velocity = _drive_velocity;
      }

      void setEnabledDrive( bool _enabled_drive ) {
        update_bit_mask |= ENABLED_DRIVE;
        enabled_drive = _enabled_drive;
      }

      void setDriveForceLimit( H3DFloat _drive_force_limit ) {
        update_bit_mask |= DRIVE_FORCE_LIMIT;
        drive_force_limit = _drive_force_limit;
      }

      // 'get' functions

      H3DFloat getDriveVelocity() {
        return drive_velocity;
      }

      bool getEnabledDrive() {
        return enabled_drive;
      }

      H3DFloat getDriveForceLimit() {
        return drive_force_limit;
      }

      // 'have' functions

      bool haveDriveVelocity() {
        return (update_bit_mask & DRIVE_VELOCITY) != 0;
      }

      bool haveEnabledDrive() {
        return (update_bit_mask & ENABLED_DRIVE) != 0;
      }

      bool haveDriveForceLimit() {
        return (update_bit_mask & DRIVE_FORCE_LIMIT) != 0;
      }

    protected:
      // update bit mask flags
      static const unsigned int DRIVE_VELOCITY = 0x0010;
      static const unsigned int ENABLED_DRIVE = 0x0020;
      static const unsigned int DRIVE_FORCE_LIMIT = 0x0040;

      H3DFloat drive_velocity;
      bool enabled_drive;
      H3DFloat drive_force_limit;
    };

  }

}
#endif
