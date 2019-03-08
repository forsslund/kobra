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
/// \file X3DNBodyCollisionSpaceNode.h
/// \brief Header file for X3DNBodyCollisionSpaceNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __X3DNBODYCOLLISIONSPACENODE__
#define __X3DNBODYCOLLISIONSPACENODE__

#include <H3D/X3DChildNode.h>
#include <H3D/X3DBoundedObject.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFRotation.h>
#include <H3D/SFBool.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/CollidableGroup.h>

namespace H3D{

  /// \ingroup AbstractNodes
  /// \class X3DNBodyCollisionSpaceNode
  /// \brief The X3DNBodyCollisionSpaceNode abstract node type represents 
  /// objects that act as a self-contained spatial collection of objects 
  /// that can interact through collision detection routines. Different
  /// types of spaces may be defined depending on spatial organization or
  /// other optimization mechanisms.
  ///
  /// \par Internal routes:
  /// \dotfile X3DNBodyCollisionSpaceNode.dot
  class H3DPHYS_API X3DNBodyCollisionSpaceNode :
    public X3DNode, 
    public X3DBoundedObject {
  public:

    typedef vector< H3DCollidableGroupId > H3DCollidableGroupIdList;
    typedef vector< H3DCollidableExceptionGroupId > H3DCollidableExceptionGroupIdList;
    typedef vector< H3DCollidableSelectionGroupId > H3DCollidableSelectionGroupIdList;

    class H3DPHYS_API MFH3DCollidableGroupId: public MField< H3DCollidableGroupId > {
    public:
      MFH3DCollidableGroupId(){}
      MFH3DCollidableGroupId( size_type sz ): MField< H3DCollidableGroupId >( sz ){}      
    };

    class H3DPHYS_API MFH3DCollidableExceptionGroupId: public MField< H3DCollidableExceptionGroupId > {
    public:
      MFH3DCollidableExceptionGroupId(){}
      MFH3DCollidableExceptionGroupId( size_type sz ): MField< H3DCollidableExceptionGroupId >( sz ){}
    };

    class H3DPHYS_API MFH3DCollidableSelectionGroupId: public MField< H3DFloat > {
    public:
      MFH3DCollidableSelectionGroupId(){}
      MFH3DCollidableSelectionGroupId( size_type sz ): MField< H3DFloat >( sz ){}
    };

    /// Constructor.
    X3DNBodyCollisionSpaceNode( Inst< SFBool  > _enabled    = 0,
      Inst< SFNode  > _metadata   = 0,
      Inst< SFBound > _bound      = 0,
      Inst< SFVec3f > _bboxCenter = 0,
      Inst< SFVec3f > _bboxSize   = 0,
      Inst< MFH3DCollidableGroupId > _inCollidableGroup = 0,
      Inst< MFH3DCollidableExceptionGroupId > _inCollidableExceptionGroup = 0,
      Inst< MFH3DCollidableSelectionGroupId > _inCollidableSelectionGroup = 0);

    /// Specifies whether the collision space is to be considered during 
    /// collision processing.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> TRUE \n
    /// 
    /// \dotfile X3DNBodyCollisionSpaceNode_enabled.dot
    auto_ptr< SFBool > enabled;

    /// Adds the X3DNBodyCollisionSpaceNode to the collidableGroup.
    virtual void addToCollidableGroup( H3DCollidableGroupId groupId );

    /// Removes the X3DNBodyCollisionSpaceNode from the collidableGroup.
    virtual void removeFromCollidableGroup( H3DCollidableGroupId groupId );

    /// Adds the X3DNBodyCollidableNode to the collidableExceptionGroup.
    virtual void addToCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId );

    /// Removes the X3DNBodyCollidableNode from the collidableExceptionGroup.
    virtual void removeFromCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId );

    /// Adds the X3DNBodyCollidableNode to the collidableSelectionGroup.
    virtual void addToCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId, bool selected = false );

    /// Removes the X3DNBodyCollidableNode from the collidableSelectionGroup.
    virtual void removeFromCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId );

    /// Get the H3DSpaceId used by this node.
    /// This is only valid if initializeSpace has been called before.
    inline H3DSpaceId getSpaceId() { 
      return space_id;
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    struct EpsilonCompareFunctor {
      EpsilonCompareFunctor( H3DFloat _groupId ) : groupId( _groupId ) {};
      H3DFloat groupId;
      inline bool operator()( const H3DFloat& cmp ) const {
        return epsilonCompare( groupId, cmp );
      }
    };

  protected:

    /// The PhysicsEngineThread with which this space is created
    H3D::PhysicsEngineThread * engine_thread;

    /// A map from CollisionSpace ID to the CollisionShape node that is
    /// associated with that ID.
    static std::map< H3DSpaceId, X3DNBodyCollisionSpaceNode * > space_id_map;

    /// The ID of this CollisionSpace object
    H3DSpaceId space_id;

    /// The inCollidableGroup field keeps track of which collidableGroups
    /// this X3DNBodyCollisionSpaceNode is included. By default each
    /// X3DNBodyCollisionSpaceNode is assumed to be included in the default
    /// collidableGroup with a collidableGroupId of -1. The same collidable
    /// can be included in more than one collidableGroups. When the node
    /// is added to a collidableGroup it is removed from the default
    /// collidableGroup. When the node does not belong to any collidableGroup
    /// it is added back to the default collidableGroup.
    auto_ptr< MFH3DCollidableGroupId > inCollidableGroup;

    /// The inCollidableExceptionGroup field keeps track of which
    /// collidableExceptionGroups this X3DNBodyCollidableNode is included.
    /// By default X3DNBodyCollidableNode is not included in any
    /// collidableExceptionGroup. The same collidable can be included
    /// in more than one collidableExceptionGroups.
    auto_ptr< MFH3DCollidableExceptionGroupId > inCollidableExceptionGroup; 

    /// The inCollidableSelectionGroup field keeps track of which
    /// collidableSelectionGroups this X3DNBodyCollidableNode is included.
    /// By default X3DNBodyCollidableNode is not included in any
    /// collidableSelectionGroup. The same collidable can be included
    /// in more than one collidableSelectionGroups.
    auto_ptr< MFH3DCollidableSelectionGroupId > inCollidableSelectionGroup;

    friend class CollidableSelectionGroup;

  };
}
#endif
