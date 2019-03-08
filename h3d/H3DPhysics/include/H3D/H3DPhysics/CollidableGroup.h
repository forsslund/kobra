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
/// \file CollidableGroup.h
/// \brief Header file for CollidableGroup, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __COLLIDABLEGROUP__ 
#define __COLLIDABLEGROUP__

#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/MFNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>

namespace H3D{
  
  // Forward declaration.
  class X3DNBodyCollisionSpaceNode;

  /// \ingroup X3DNodes
  /// \class CollidableGroup
  /// \brief The CollidableGroup node is used to group collidables which can
  /// collide with chosen collidables only.
  ///
  /// Each collidable will be checked for collision with collidables in the
  /// same CollidableGroup. More than one CollidableGroups can be added in
  /// the CollisionCollection. In case no CollidableGroup exists collision is
  /// possible between all collidables. The use of CollidableGroup has nothing
  /// to do with hierarchical representation for collision, as in the case of
  /// collisionSpace, instead the aim is to be able choose which objects can
  /// collide each other. Since the collidableGroup is not responsible for
  /// creating/deleting collidables, the collidables should be included directly
  /// to the collidables field of collisionCollection. In case separate groups
  /// including objects which collide only with objects in the same group want
  /// to be created collidableGroups can be used. The same collidable can be
  /// included in different collidableGroups.
  ///
  /// \note SUPPORT FOR THIS NODE IS NOT IMPLEMENTED BY ANY PHYSICS ENGINE YET.
  /// IF YOU TRY TO GET IT TO WORK YOU WILL HAVE TO IMPLEMENT IT FIRST.
  ///
  /// \par Internal routes:
  /// \dotfile CollidableGroup.dot
  class H3DPHYS_API CollidableGroup :
    public X3DChildNode {
  public:

    /// Node type of values in collidables field. The collidables field value
    /// should only be of type X3DNBodyCollisionSpaceNode or 
    /// X3DNBodyCollidableNode
    class H3DPHYS_API MFCollidable: public MFNode {
    public:
      /// Destructor. To make sure the correct onRemove function is called upon
      /// destruction.
      ~MFCollidable() {
        clear();
      }

    protected:
      /// On addition of a node to collidables field, add the collidableGroupId
      /// to the list of inCollidableGroups field of the node.
      virtual void onAdd( Node *n );

      /// On removal of a node from collidables field, remove the collidableGroupId
      /// from the list of inCollidableGroups field of the node. In case the node
      /// does not belong to any other collidableGroup assign the default value.
      virtual void onRemove( Node *n );
    };

    /// Constructor.
    CollidableGroup(  Inst< SFNode  >  _metadata = 0, 
      Inst< MFCollidable > _collidables = 0);

    ~CollidableGroup();

    /// traverseSG function of this node. 
    virtual void traverseSG( H3D::TraverseInfo &ti );

    /// The collidable field holds a reference to a single nested item of
    /// collidable scene graph. If there are multiple transformation paths to
    /// this reference, the results are undefined.
    /// 
    /// <b>Access type: </b> inputOutput
    auto_ptr< MFCollidable > collidables;

    /// Cleares collidableGroup_id from the inCollidableGroup field of
    /// the collidables it used to include.
    void deleteCollidableGroup();

    /// Get the H3DCollidableGroupId used by this node.
    /// This is only valid if initializeCollidable has been called before.
    inline H3DCollidableGroupId getCollidableGroupId() { 
      return collidableGroup_id;
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    H3DCollidableGroupId collidableGroup_id;

    std::vector<X3DNBodyCollidableNode*> newX3DNBodyCollidableNodes;
    std::vector<X3DNBodyCollisionSpaceNode*> newX3DNBodyCollisionSpaceNodes;

  };
}
#endif
