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
/// \file CollidableExceptionGroup.h
/// \brief Header file for CollidableExceptionGroup, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __COLLIDABLEEXCEPTIONGROUP__ 
#define __COLLIDABLEEXCEPTIONGROUP__ 

#include <H3D/SFNode.h>
#include <H3D/MFNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollisionSpaceNode.h>

namespace H3D {
  /// \ingroup X3DNodes
  /// \class CollidableExceptionGroup
  /// \brief The CollidableExceptionGroup node is used to group collidables which
  /// should not collide with each other.
  ///
  /// Collisions between the collidables in the same CollidableExceptionGroup will
  /// be ignored. More than one CollidableExceptionGroups can be added in the
  /// CollisionCollection. In case no CollidableExceptionGroup exists collision is
  /// possible between all collidables. The use of CollidableExceptionGroup has nothing
  /// to do with hierarchical representation for collision, as in the case of
  /// collisionSpace, instead the aim is to be able choose which objects should not
  /// collide each other. Since the collidableGroup is not responsible for
  /// creating/deleting collidables, the collidables should be included directly
  /// to the collidables field of collisionCollection. The same collidable can be
  /// included in different collidableExceptionGroups.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/CollidableExceptionGroup.x3d">CollidableExceptionGroup.x3d</a>
  ///     ( <a href="examples/CollidableExceptionGroup.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile CollidableExceptionGroup.dot
  class H3DPHYS_API CollidableExceptionGroup :
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
      /// On addition of a node to collidables field, add the collidableExceptionGroupId
      /// to the list of inCollidableExceptionGroups field of the node.
      virtual void onAdd( Node *n );

      /// On removal of a node from collidables field, remove the collidableExceptionGroupId
      /// from the list of inCollidableExceptionGroups field of the node. In case the node
      /// does not belong to any other collidableGroup assign the default value.
      virtual void onRemove( Node *n );
    };

    /// Constructor.
    CollidableExceptionGroup( Inst< SFNode  >  _metadata = 0,
                              Inst< MFCollidable > _collidables = 0 );

    ~CollidableExceptionGroup();

    /// traverseSG function of this node. 
    virtual void traverseSG( H3D::TraverseInfo &ti );

    /// The collidable field holds a reference to a single nested item of
    /// collidable scene graph. If there are multiple transformation paths to
    /// this reference, the results are undefined.
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile CollidableExceptionGroup_collidables.dot
    auto_ptr< MFCollidable > collidables;

    /// Cleares collidableExceptionGroup_id from the inCollidableExceptionGroup field of
    /// the collidables it used to include.
    void deleteCollidableExceptionGroup();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    H3DCollidableExceptionGroupId collidableExceptionGroup_id;

    /// Assigns a group id to the CollidableExceptionGroup. This should be
    /// called only once by the CollidableExceptionGroup in its constructor.
    static H3DCollidableExceptionGroupId reserveCollidableExceptionGroupId();

    /// Makes the id available for other CollidableExceptionGroups. This should be
    /// called only once by the CollidableExceptionGroup in its destructor.
    static void freeCollidableExceptionGroupId( H3DCollidableExceptionGroupId id );

    /// Returns the list of available group ids which were used before.
    static CollidableParameters::CollidableExceptionGroupList &getFreeGroupIds();

    std::vector<X3DNBodyCollidableNode*> newX3DNBodyCollidableNodes;
    std::vector<X3DNBodyCollisionSpaceNode*> newX3DNBodyCollisionSpaceNodes;

  };
}
#endif
