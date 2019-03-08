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
/// \file Contact.h
/// \brief Header file for Contact, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __CONTACT__
#define __CONTACT__

#include <H3D/X3DNode.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFVec2f.h>
#include <H3D/SFFloat.h>
#include <H3D/MFString.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>

#include <H3D/H3DPhysics/H3DBodyNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>

namespace H3D{

  /// \ingroup X3DNodes
  /// \class Contact
  /// \brief The Contact node specifies information concerning a contact 
  /// between collidable objects and/or spaces.
  ///
  /// The body1 and body2 fields specify two top-level nodes that should
  /// be evaluated in the physics model as a single set of interactions 
  /// with respect to each other.
  ///
  /// The geometry1 and geometry2 fields specify information about body1 and
  /// body2.
  ///
  /// The position field indicates the exact location of the contact that was
  /// made between the two objects.
  ///
  /// The contactNormal field is a unit vector describing the normal between
  /// the two colliding bodies.
  /// 
  /// The depth field indicates how deep the current intersection is along
  /// the normal vector.
  ///
  /// The frictionDirection field describes
  /// which way friction is to be applied to the contact location. If there is
  /// no friction, the direction should be set to 0, 0, 0.
  ///
  /// The bounce field indicates how bouncy the surface contact is. A value
  /// of 0 indicates no bounce at all while a value of 1 indicates maximum
  /// bounce.
  ///
  /// The minBounceSpeed field indicates the minimum speed, in metres per 
  /// second, that an object shall have before an object will bounce. If
  /// the object is below this speed, it will not bounce, effectively 
  /// having an equivalent value for the bounce field of zero.
  ///
  /// The surfaceSpeed field defines the speed in the two friction directions
  /// in metres per second. This is used to indicate whether the contact 
  /// surface is moving independently of the motion of the bodies.
  /// 
  /// EXAMPLE  A conveyor belt mechanism may be stationary while its belt is
  /// moving. The object being placed on the conveyor belt will not be affected
  /// by the motion of the belt until it is in contact with it.
  ///
  /// The softnessConstantForceMix value applies a constant force value to make
  /// the colliding surfaces appear to be somewhat soft.
  ///
  /// The softnessErrorCorrection determines how much of the collision error
  /// should be fixed in a set of evaluations. The value is limited to the
  /// range of [0,1] where 0 specifies no error correction while a value of 
  /// 1 specifies that all errors should be corrected in a single step.
  ///
  /// The appliedParameters indicates which of the parameters in the Contact 
  /// node are used by the rigid body physics system. 
  ///
  /// "BOUNCE" The bounce field value is used.
  /// "USER_FRICTION" The system will normally calculate the friction direction
  /// vector that is perpendicular to the contact normal. This setting 
  /// indicates that the user-supplied value in this contact should be used.
  /// "FRICTION_COEFFICIENT-2" The frictionCoefficients field values are used.
  /// "ERROR_REDUCTION" The softnessErrorCorrection field value in the contact
  /// evaluation should be used.
  /// "CONSTANT_FORCE" The softnessConstantForceMix field value in the contact
  /// evaluation should be used.
  /// "SPEED-1" The surfaceSpeed field value first component is used.
  /// "SPEED-2" The surfaceSpeed field value second component is used.
  /// "SLIP-1"  The slipCoefficients field value first component is used.
  /// "SLIP-2"  The slipCoefficients field value second component is used.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/CollisionSensor.x3d">CollisionSensor.x3d</a>
  ///     ( <a href="examples/CollisionSensor.x3d.html">Source</a> )
  class H3DPHYS_API Contact :  public X3DNode {
  public:

    typedef TypedSFNode< X3DNBodyCollidableNode > SFCollidableNode;
    typedef TypedSFNode< H3DBodyNode > SFH3DBodyNode;

    /// Constructor.
    Contact(
      Inst< SFNode           > _metadata = 0,
      Inst< MFString         > _appliedParameters = 0,
      Inst< SFH3DBodyNode      > _body1 = 0,
      Inst< SFH3DBodyNode      > _body2 = 0,
      Inst< SFFloat          > _bounce = 0,
      Inst< SFVec3f          > _contactNormal = 0,
      Inst< SFFloat          > _depth = 0,
      Inst< SFVec2f          > _frictionCoefficients = 0,
      Inst< SFVec3f          > _frictionDirection = 0,
      Inst< SFCollidableNode > _geometry1 = 0,
      Inst< SFCollidableNode > _geometry2 = 0,
      Inst< SFFloat          > _minBounceSpeed = 0,
      Inst< SFVec3f          > _position = 0,
      Inst< SFVec2f          > _slipCoefficients = 0,
      Inst< SFFloat          > _softnessConstantForceMix = 0,
      Inst< SFFloat          > _softnessErrorCorrection = 0,
      Inst< SFVec2f          > _surfaceSpeed = 0 );

    /// The appliedParameters indicates which of the parameters in the Contact 
    /// node are used by the rigid body physics system. 
    ///
    /// "BOUNCE" The bounce field value is used.
    /// "USER_FRICTION" The system will normally calculate the friction 
    /// direction
    /// vector that is perpendicular to the contact normal. This setting 
    /// indicates that the user-supplied value in this contact should be used.
    /// "FRICTION_COEFFICIENT-2" The frictionCoefficients field values are used.
    /// "ERROR_REDUCTION" The softnessErrorCorrection field value in the contact
    /// evaluation should be used.
    /// "CONSTANT_FORCE" The softnessConstantForceMix field value in the 
    /// contact evaluation should be used.
    /// "SPEED-1" The surfaceSpeed field value first component is used.
    /// "SPEED-2" The surfaceSpeed field value second component is used.
    /// "SLIP-1"  The slipCoefficients field value first component is used.
    /// "SLIP-2"  The slipCoefficients field value second component is used.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> "BOUNCE"
    auto_ptr< MFString > appliedParameters;

    /// One of the rigid bodies of the contact.
    /// 
    /// <b>Access type: </b> inputOutput
    auto_ptr< SFH3DBodyNode > body1;

    /// One of the rigid bodies of the contact.
    /// 
    /// <b>Access type: </b> inputOutput
    auto_ptr< SFH3DBodyNode > body2;

    /// The bounce field indicates how bouncy the surface contact is. A value
    /// of 0 indicates no bounce at all while a value of 1 indicates maximum
    /// bounce.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0
    auto_ptr< SFFloat > bounce;

    /// The contactNormal field is a unit vector describing the normal between
    /// the two colliding bodies.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 1, 0 )
    auto_ptr< SFVec3f > contactNormal;

    /// The depth field indicates how deep the current intersection is along
    /// the normal vector.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0
    auto_ptr< SFFloat > depth;

    /// Coulomb friction coefficients in the range 0 to infinity. 
    /// 0 results in a frictionless contact. The first value is the friction
    /// in friction direction 1, which is perpendicular to the contact normal.
    /// Second parameter is friction in friction direction 2(perpendicular 
    /// to normal and direction 1).
    /// 
    /// <b>Access type: </b>   inputOutput
    /// <b>Default value: </b> Vec2f( 0, 0 )
    auto_ptr< SFVec2f > frictionCoefficients;

    /// Friction direction 1. This is the direction along which frictional
    /// force is applied. It must be of unit length and perpendicular to
    /// the contact normal (so it is typically tangential to the contact
    /// surface).
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 1, 0 )
    auto_ptr< SFVec3f > frictionDirection;

    /// The collidable node for body 1.
    /// 
    /// <b>Access type: </b> inputOutput
    auto_ptr< SFCollidableNode > geometry1;

    /// The collidable node for body 2.
    /// 
    /// <b>Access type: </b> inputOutput
    auto_ptr< SFCollidableNode > geometry2;

    /// The minBounceSpeed field indicates the minimum speed, in metres per 
    /// second, that an object shall have before an object will bounce. If
    /// the object is below this speed, it will not bounce, effectively 
    /// having an equivalent value for the bounce field of zero.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0
    auto_ptr< SFFloat > minBounceSpeed;

    /// The position field indicates the exact location of the contact that 
    /// was made between the two objects.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 0, 0 )
    auto_ptr< SFVec3f > position;

    /// The coefficients of force-dependent-slip (FDS) for friction directions 
    /// 1 and 2. FDS is an effect that causes the contacting surfaces to side 
    /// past each other with a velocity that is proportional to the force that 
    /// is being applied tangentially to that surface. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec2f( 0, 0 )
    auto_ptr< SFVec2f > slipCoefficients;

    /// The softnessConstantForceMix value applies a constant force value
    /// to make the colliding surfaces appear to be somewhat soft.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.0001
    auto_ptr< SFFloat > softnessConstantForceMix;

    /// The softnessErrorCorrection determines how much of the collision error
    /// should be fixed in a set of evaluations. The value is limited to the
    /// range of [0,1] where 0 specifies no error correction while a value of 
    /// 1 specifies that all errors should be corrected in a single step.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.8
    auto_ptr< SFFloat > softnessErrorCorrection;

    /// The surfaceSpeed field defines the speed in the two friction directions
    /// in metres per second. This is used to indicate whether the contact 
    /// surface is moving independently of the motion of the bodies.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec2f( 0, 0 )
    auto_ptr< SFVec2f > surfaceSpeed;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  };
}
#endif
