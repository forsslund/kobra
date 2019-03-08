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
/// \file CollisionSpace.h
/// \brief Header file for CollisionSpace, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __COLLISIONSPACE__
#define __COLLISIONSPACE__

#include <H3D/SFVec3f.h>
#include <H3D/MFNode.h>
#include <H3D/SFBool.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/H3DPhysics/X3DNBodyCollisionSpaceNode.h>


namespace H3D{

  /// \ingroup X3DNodes
  /// \class CollisionSpace
  /// \brief The CollisionSpace node holds a collection of objects in 
  /// the collidables field that can be considered as a single entity 
  /// for resolution of inter-object collisions with other groups of 
  /// collidable objects. 
  ///
  /// A group consists of both collidable objects as well as nested 
  /// collections. This grouping allows creation of efficient collision 
  /// detection scenarios by grouping functional sets of objects together.
  /// Spaces may be collided against each other to determine if the larger
  /// group of objects are anywhere near each other. If there is some 
  /// intersection between two spaces, or between a collidable space and 
  /// a collidable object, the system will traverse into the contained 
  /// objects looking for finer resolution on exactly which objects 
  /// collided together.
  ///
  /// The useGeometry field indicates whether the collision detection 
  /// code should check for collisions down to the level of geometry or
  /// only make approximations using the bounds of the geometry. Using 
  /// the geometry will be more accurate but slower. In most cases, 
  /// just testing against the bounds of the object is sufficient.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/CollisionSpace.x3d">CollisionSpace.x3d</a>
  ///     ( <a href="examples/CollisionSpace.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile CollisionSpace.dot
  class H3DPHYS_API CollisionSpace : public X3DNBodyCollisionSpaceNode {
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
      /// On addition of a node to collidables field, add the node to the 
      /// nodes_changed list and route its bound to bound of CollisionSpace.
      virtual void onAdd( Node *n );

      /// On removal of a node from collidables field, add the node to the 
      /// nodes_changed list and unroute its bound to bound of CollisionSpace.
      virtual void onRemove( Node *n );
    };

    /// SFBound is specialized to update from the SFBound fields 
    /// routed to it. The resulting Bound object is the union of 
    /// the Bound objects routed to it. If the bboxSize of the 
    /// CollisionSpace node containing the SFBound field is ( -1, -1, -1 )
    /// bound fields of all Nodes in the children field of the containing
    /// Node that are instances of X3DBoundedObject are routed to it.
    /// Otherwise the bound will be a BoxBound with center and
    /// radius specified with the bboxCenter and bboxSize fields.
    class SFBound: 
      public TypedField< X3DBoundedObject::SFBound,
      void,
      AnyNumber< X3DBoundedObject::SFBound > > {
        /// The SFBound is updated to a bound that is the union of the 
        /// the Bound objects routed to it.
        virtual void update();
    };

    /// Constructor.
    CollisionSpace( Inst< MFCollidable > _collidables = 0,
      Inst< SFBool  > _enabled     = 0,
      Inst< SFNode  > _metadata    = 0,
      Inst< SFBool  > _useGeometry = 0,
      Inst< SFBound > _bound       = 0,
      Inst< SFVec3f > _bboxCenter  = 0,
      Inst< SFVec3f > _bboxSize    = 0 );

    /// Destructor. Deletes node values in the collidables field.
    ~CollisionSpace();

    /// initialize function of this node. Updates the bounding 
    /// box for this CollisionSpace from bboxCenter and bboxSize.
    virtual void initialize();

    /// traverseSG function of this node. 
    virtual void traverseSG( H3D::TraverseInfo &ti );

    /// Child collidable objects 
    /// i.e. X3DNBodyCollisionSpaceNodes or X3DNBodyCollidableNodes
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile CollisionSpace_collidables.dot
    auto_ptr< MFNode > collidables;

    /// Indicates whether the collision detection code should check 
    /// for collisions down to the level of geometry or only make 
    /// approximations using the bounds of the geometry.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> FALSE \n
    /// 
    /// \dotfile CollisionSpace_useGeometry.dot
    auto_ptr< SFBool > useGeometry;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// Initializes the CollisionSpace with the given PhysicsEngineThread i.e.
    /// create space in the physics engine. This function is called by the 
    /// containing CollisionSpace of this node or CollectionCollection. 
    /// Returns 0 on success. 
    int initializeSpace( H3D::PhysicsEngineThread *pt, X3DNode *n );

    /// Returns true if this CollisionSpace node has been initialized for 
    /// PhysicsEngineThread.
    bool isInitialized();

    /// Deletes this CollisionSpace from PhysicsEngineThread.
    int deleteSpace();

    /// Adds the CollisionSpace and its children to the collidableGroup.
    virtual void addToCollidableGroup( H3DCollidableGroupId groupId );

    /// Removes the CollisionSpace and its children from the collidableGroup.
    virtual void removeFromCollidableGroup( H3DCollidableGroupId groupId );

  protected:
    /// If true a route will be set up between the bound field of the
    /// nodes in collidables and the bound field of the CollisionSpace node. 
    bool use_union_bound;

    /// A list of node changes in collidables field value i.e. a list of
    /// nodes from the collidables field of which onRemove or onAdd was
    /// called in the traversal.
    typedef std::list< pair< bool, AutoRef< Node > > > NodesChangedList;
    NodesChangedList nodes_changed;

    /// Function to update changes on the nodes_changed list.
    void updateNodeChanges();
  };
}
#endif
