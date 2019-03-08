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
/// \file CollidableSelectionGroup.h
/// \brief Header file for CollidableSelectionGroup, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __COLLIDABLESELECTIONGROUP__ 
#define __COLLIDABLESELECTIONGROUP__

#include <H3D/SFNode.h>
#include <H3D/MFNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollisionSpaceNode.h>

namespace H3D {
  /// \ingroup X3DNodes
  /// \class CollidableSelectionGroup
  /// \brief The CollidableSelectionGroup node is used to group collidables for isolation
  /// purposes s.t they collide only with chosen collidables and do not affect( be not affected by)
  /// the others.
  ///
  /// A collidable object added to the collidables field will only collide with other collidables
  /// in the collidables field and in the selectedCollidables field. Collision relations of the
  /// objects in the selectedCollidables field with other objects in the scene will not be affected
  /// by the CollidableSelectionGroup.
  ///
  /// A collidable in any other CollidableSelectionGroup can not be added to the collidables field,
  /// neither a collidable in collidables field can be added to another CollidableSelectionGroup.
  /// A collidable can be added in selectedCollidables field of more than one CollidableSelectionGroup
  ///
  /// More than one CollidableSelectionGroups can be added in the
  /// CollisionCollection. In case no CollidableSelectionGroup exists collision is
  /// possible between all collidables. The use of CollidableSelectionGroup has nothing
  /// to do with hierarchical representation for collision, as in the case of
  /// collisionSpace, instead the aim is to be able choose which objects should not
  /// collide each other. Since the collidableGroup is not responsible for
  /// creating/deleting collidables, the collidables should be included directly
  /// to the collidables field of collisionCollection. The same collidable can be
  /// included in different collidableSelectionGroups.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/CollidableSelectionGroup.x3d">CollidableSelectionGroup.x3d</a>
  ///     ( <a href="examples/CollidableSelectionGroup.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile CollidableSelectionGroup.dot
  class H3DPHYS_API CollidableSelectionGroup :
    public X3DChildNode {
  public:

    /// Node type of values in collidables field. The collidables field value
    /// should only be of type X3DNBodyCollisionSpaceNode or 
    /// X3DNBodyCollidableNode
    class H3DPHYS_API MFCollidable : public MFNode {
    public:
      /// Destructor. To make sure the correct onRemove function is called upon
      /// destruction.
      ~MFCollidable() {
        clear();
      }

    protected:
      /// On addition of a node to collidables field, add the collidableSelectionGroupId
      /// to the list of inCollidableSelectionGroups field of the node.
      virtual void onAdd( Node *n );

      /// On removal of a node from collidables field, remove the collidableSelectionGroupId
      /// from the list of inCollidableSelectionGroups field of the node. In case the node
      /// does not belong to any other collidableGroup assign the default value.
      virtual void onRemove( Node *n );
    };

    /// Node type of values in collidables field. The collidables field value
    /// should only be of type X3DNBodyCollisionSpaceNode or 
    /// X3DNBodyCollidableNode
    class H3DPHYS_API MFSelectedCollidable : public MFNode {
    public:
      /// Destructor. To make sure the correct onRemove function is called upon
      /// destruction.
      ~MFSelectedCollidable() {
        clear();
      }

    protected:
      /// On addition of a node to collidables field, add the collidableSelectionGroupId
      /// to the list of inCollidableSelectionGroups field of the node.
      virtual void onAdd( Node *n );

      /// On removal of a node from collidables field, remove the collidableSelectionGroupId
      /// from the list of inCollidableSelectionGroups field of the node. In case the node
      /// does not belong to any other collidableGroup assign the default value.
      virtual void onRemove( Node *n );
    };

    /// Constructor.
    CollidableSelectionGroup( Inst< SFNode  >  _metadata = 0,
                              Inst< MFCollidable > _collidables = 0,
                              Inst< MFSelectedCollidable > _selectedCollidables = 0 );

    ~CollidableSelectionGroup();

    /// traverseSG function of this node. 
    virtual void traverseSG( H3D::TraverseInfo &ti );

    /// A collidable object added to the collidables field will only
    /// collide with other collidables in the collidables field and in
    /// the selectedCollidables field.
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile CollidableSelectionGroup_collidables.dot
    auto_ptr< MFCollidable > collidables;

    /// A collidable object added to the selectedCollidables field will only
    /// collide with other collidables in the collidables field and in
    /// the selectedCollidables field. The collidable objects added to
    /// this field can also be specified in the selectedCollidable field
    /// of other CollidableSelectionGroups.
    /// 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile CollidableSelectionGroup_selectedCollidables.dot
    auto_ptr< MFSelectedCollidable > selectedCollidables;

    /// Cleares collidableSelectionGroup_id from the inCollidableSelectionGroup field of
    /// the collidables it used to include.
    void deleteCollidableSelectionGroup();

    /// True if the node is already in the group.
    bool isNodeInGroup( Node *n );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    H3DCollidableSelectionGroupId collidableSelectionGroup_id;

    /// Assigns a group id to the CollidableSelectionGroup. This should be
    /// called only once by the CollidableSelectionGroup in its constructor.
    static H3DCollidableSelectionGroupId reserveCollidableSelectionGroupId();

    /// Makes the id available for other CollidableSelectionGroups. This should be
    /// called only once by the CollidableSelectionGroup in its destructor.
    static void freeCollidableSelectionGroupId( H3DCollidableSelectionGroupId id );

    /// Returns the list of available group ids which were used before.
    static CollidableParameters::CollidableSelectionGroupList &getFreeGroupIds();

    std::vector<X3DNBodyCollidableNode*> new_X3DNBodyCollidableNodes;
    std::vector<X3DNBodyCollisionSpaceNode*> new_X3DNBodyCollisionSpaceNodes;

    std::vector<Node*> previous_collidables;
    std::vector<Node*> previous_selected_collidables;

  };
}
#endif
