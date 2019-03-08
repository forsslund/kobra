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
/// \file CollisionSensor.h
/// \brief Header file for CollisionSensor, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __COLLISIONSENSOR__
#define __COLLISIONSENSOR__

#include <H3D/X3DSensorNode.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFNode.h>
#include <H3D/MFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/CollisionCollection.h>
#include <H3D/H3DPhysics/Contact.h>


namespace H3D{

  /// \ingroup X3DNodes
  /// \class CollisionSensor
  /// \brief The CollisionSensor node is used to send collision detection 
  /// information into the scene graph for user processing. The collision
  /// detection system does not require an instance of this class to be in
  /// the scene in order for it to run or affect the physics model. This 
  /// class is used to report to the user contact information should the
  /// user require this information for other purposes.
  ///
  /// The collider field specifies the CollisionCollection node for which
  /// to detect contacts.
  ///
  /// The contacts field is used to report contacts that were generated
  /// as a result of the scene graph changes last frame. This field generates
  /// instances of the Contact node.
  ///
  /// The CollisionSensor is active (isActive is TRUE) when contacts were 
  /// located as a result of the movement of the watched objects from last
  /// frame.
  ///
  /// The intersections field is used to report the colliding geometry that
  /// was detected in this last frame.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/CollisionSensor.x3d">CollisionSensor.x3d</a>
  ///     ( <a href="examples/CollisionSensor.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile CollisionSensor.dot
  class H3DPHYS_API CollisionSensor :  public X3DSensorNode  {
  public:

    typedef TypedSFNode< CollisionCollection > SFCollisionCollection;
    typedef TypedMFNode< X3DNBodyCollidableNode > MFCollidableNode;
    typedef TypedMFNode< Contact > MFContact;

    /// Constructor.
    CollisionSensor( Inst< SFBool > _enabled = 0,
      Inst< SFNode > _metadata = 0,
      Inst< SFBool > _isActive = 0,
      Inst< SFCollisionCollection > _collider = 0,
      Inst< MFCollidableNode > _intersections = 0,
      Inst< MFContact > _contacts = 0 );

    /// Traverse the scene-graph. Update the contacts field with info
    /// from the last contacts.
    virtual void traverseSG( TraverseInfo &ti );

    /// The collider field specifies the CollisionCollection node for which
    /// to detect contacts.
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile CollisionSensor_collider.dot
    auto_ptr< SFCollisionCollection > collider;

    /// Not implemented in H3D.
    /// 
    /// <b>Access type: </b> outputOnly
    ///
    /// \dotfile CollisionSensor_intersections.dot
    auto_ptr< MFCollidableNode > intersections ;


    /// The contacts field is used to report contacts that were generated
    /// as a result of the scene graph changes last frame. This field generates
    /// instances of the Contact node.
    /// 
    /// <b>Access type: </b> outputOnly
    ///
    /// \dotfile CollisionSensor_contacts.dot
    auto_ptr< MFContact > contacts;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  };
}
#endif
