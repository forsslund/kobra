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
/// \file PhysX3JointParameters.h
/// \brief Header file for PhysX3 implementation of joints parameters.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSX3JOINTPARAMETERS_H__
#define __PHYSX3JOINTPARAMETERS_H__

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3JointParameters node
    /// to be passed to the physics simulation thread
    struct PhysX3JointParameters : public EngineOptionParameters {
      /// Constructor
      PhysX3JointParameters() :
        projection_tolerance( Vec2f() ), constraint_flag( "ePROJECTION" ),
        enabled_projection( false ), EngineOptionParameters() {
      }

      // 'set' functions

      void setProjectionTolerance( Vec2f _projection_tolerance ) {
        update_bit_mask |= PROJECTION_TOLERANCE;
        projection_tolerance = _projection_tolerance;
      }

      void setConstraintFlag( string _constraint_flag ) {
        update_bit_mask |= CONSTRAINT_FLAG;
        constraint_flag = _constraint_flag;
      }

      void setEnabledProjection( bool _enabled_projection ) {
        update_bit_mask |= ENABLED_PROJECTION;
        enabled_projection = _enabled_projection;
      }

      // 'get' functions

      Vec2f getProjectionTolerance() {
        return projection_tolerance;
      }

      string getConstraintFlag() {
        return constraint_flag;
      }

      bool getEnabledProjection() {
        return enabled_projection;
      }

      // 'have' functions

      bool haveProjectionTolerance() {
        return (update_bit_mask & PROJECTION_TOLERANCE) != 0;
      }

      bool haveConstraintFlag() {
        return (update_bit_mask & CONSTRAINT_FLAG) != 0;
      }

      bool haveEnabledProjection() {
        return (update_bit_mask & ENABLED_PROJECTION) != 0;
      }

    protected:
      // update bit mask flags
      static const unsigned int PROJECTION_TOLERANCE = 0x0001;
      static const unsigned int CONSTRAINT_FLAG = 0x0002;
      static const unsigned int ENABLED_PROJECTION = 0x0004;

      Vec2f projection_tolerance;
      string constraint_flag;
      bool enabled_projection;
    };

    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3Joint6DOFLimitParameters node
    /// to be passed to the physics simulation thread
    struct PhysX3Joint6DOFLimitParameters : public PhysX3JointParameters {
      /// Constructor
      PhysX3Joint6DOFLimitParameters() :
        linear_x( Vec2f() ), linear_y( Vec2f() ), linear_z( Vec2f() ),
        angular_x( Vec2f() ), angular_y( Vec2f() ), angular_z( Vec2f() ),
        linear_spring( Vec2f() ), angular_spring( Vec2f() ), PhysX3JointParameters() {}

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
        static const unsigned int LINEAR_X            = 0x0008;
        static const unsigned int LINEAR_Y            = 0x0010;
        static const unsigned int LINEAR_Z            = 0x0020;
        static const unsigned int ANGULAR_X           = 0x0040;
        static const unsigned int ANGULAR_Y           = 0x0080;
        static const unsigned int ANGULAR_Z           = 0x0100;
        static const unsigned int LINEAR_SPRING       = 0x0200;
        static const unsigned int ANGULAR_SPRING      = 0x0400;
        static const unsigned int BREAK_FORCE         = 0x0800;
        static const unsigned int CONTACT_DISTANCE    = 0x1000;
        
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

    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3SliderJointParameters node
    /// to be passed to the physics simulation thread
    struct PhysX3SliderJointParameters : public PhysX3JointParameters {
      /// Constructor
      PhysX3SliderJointParameters() : explicitAnchorPoint( Vec3f() ),
        body1Offset( Vec3f() ), body2Offset( Vec3f() ), body2ForceScale( 1.0 ),
        forceType("eForce"),  PhysX3JointParameters() {}


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
        static const unsigned int EXPLICIT_ANCHOR_POINT   = 0x0008;
        static const unsigned int BODY1_OFFSET            = 0x0010;
        static const unsigned int BODY2_OFFSET            = 0x0020;
        static const unsigned int BODY2_FORCESCALE        = 0x0040;
        static const unsigned int FORCETYPE               = 0x0080;

        Vec3f explicitAnchorPoint;
        Vec3f body1Offset;
        Vec3f body2Offset;
        H3DFloat body2ForceScale;
        string forceType;

    };

  }

}
#endif
