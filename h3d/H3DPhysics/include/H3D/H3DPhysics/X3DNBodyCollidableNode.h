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
/// \file X3DNBodyCollidableNode.h
/// \brief Header file for X3DNBodyCollidableNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __X3DNBODYCOLLIDABLENODE__
#define __X3DNBODYCOLLIDABLENODE__

#include <H3D/X3DChildNode.h>
#include <H3D/X3DBoundedObject.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFRotation.h>
#include <H3D/SFBool.h>
#include <H3D/SFNode.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/MField.h>

#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/H3DPhysics/H3DEngineOptions.h>

namespace H3D{

  class RigidBodyCollection;

  /// \ingroup AbstractNodes
  /// \class X3DNBodyCollidableNode
  /// \brief The X3DNBodyCollidableNode abstract node type represents objects
  /// that act as the interface between the rigid body physics, collision
  /// geometry proxy, and renderable objects in the scene graph hierarchy.
  ///
  /// The enabled field is used to specify whether a collidable object is
  /// eligible for collision detection interactions.
  ///
  /// The translation and rotation fields define an offset from, and rotation
  /// about, the body's center that the collidable node occupies. This can be
  /// used to place the collidable geometry in a different location relative to
  /// the actual rigid body that has the physics model being applied.
  ///
  /// \par Internal routes:
  /// \dotfile X3DNBodyCollidableNode.dot
  class H3DPHYS_API X3DNBodyCollidableNode :
    public X3DChildNode,
    public X3DBoundedObject,
    public H3DDisplayListObject {
  public:
    /// The ValueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// X3DNBodyCollidableNode. More specifically it calls 
    /// PhysicsEngineThread::setCollidableParameters with the new values.
    class H3DPHYS_API ValueUpdater: 
      public EventCollectingField< PeriodicUpdate< Field > > {
        virtual void update();
    };

    typedef vector< H3DCollidableGroupId > H3DCollidableGroupIdList;
    typedef vector< H3DCollidableExceptionGroupId > H3DCollidableExceptionGroupIdList;
    typedef vector< H3DCollidableSelectionGroupId > H3DCollidableSelectionGroupIdList;

    class H3DPHYS_API MFH3DCollidableGroupId: public MField< H3DCollidableGroupId > {
    public:
      MFH3DCollidableGroupId(){}
      MFH3DCollidableGroupId( size_type sz ): MField< H3DCollidableGroupId >( sz ){}
      //virtual X3DTypes::X3DType getX3DType() { return X3DTypes::UNKNOWN_X3D_TYPE; }      
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

    typedef MFH3DEngineOptions < X3DNBodyCollidableNode > MFEngineOptions;
    friend class MFH3DEngineOptions < X3DNBodyCollidableNode >;

    /// Constructor.
    X3DNBodyCollidableNode( Inst< SFBool > _enabled = 0,
                            Inst< SFNode  >  _metadata = 0, 
                            Inst< SFRotation >  _rotation = 0,
                            Inst< SFVec3f >  _translation = 0,
                            Inst< SFVec3f >  _scale = 0,
                            Inst< SFBound >_bound = 0,
                            Inst< SFVec3f >  _bboxCenter = 0,
                            Inst< SFVec3f  >  _bboxSize = 0,
                            Inst< ValueUpdater > _valueUpdater = 0,
                            Inst< MFEngineOptions > _engineOptions = 0,
                            Inst< MFH3DCollidableGroupId > _inCollidableGroup = 0,
                            Inst< MFH3DCollidableExceptionGroupId > _inCollidableExceptionGroup = 0,
                            Inst< MFH3DCollidableSelectionGroupId > _inCollidableSelectionGroup = 0,
                            Inst< DisplayList > _displayList = 0 );

    /// The enabled field is used to specify whether a collidable object is
    /// eligible for collision detection interactions.
    ///
    /// <b>Default value: </b> True
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile X3DNBodyCollidableNode_enabled.dot
    auto_ptr< SFBool > enabled;

    /// The rotation field define a rotation about, the body's center that the
    /// collidable node occupies.
    /// 
    /// <b>Default value: </b> 0 0 1 0
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile X3DNBodyCollidableNode_rotation.dot
    auto_ptr< SFRotation > rotation;

    /// The translation field define an offset from, the body's center that the
    /// collidable node occupies.
    /// 
    /// <b>Default value: </b> 0 0 0
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile X3DNBodyCollidableNode_translation.dot
    auto_ptr< SFVec3f > translation;

    /// The scale field defines scaling factor to apply to the geometry of the
    /// collidable shape.
    ///
    /// Not part of the X3D specification
    /// 
    /// <b>Default value: </b> 1 1 1
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile X3DNBodyCollidableNode_scale.dot
    auto_ptr< SFVec3f > scale;

    /// Adds the X3DNBodyCollidableNode to the collidableGroup.
    virtual void addToCollidableGroup( H3DCollidableGroupId groupId );

    /// Removes the X3DNBodyCollidableNode from the collidableGroup.
    virtual void removeFromCollidableGroup( H3DCollidableGroupId groupId );

    /// Adds the X3DNBodyCollidableNode to the collidableExceptionGroup.
    virtual void addToCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId );

    /// Removes the X3DNBodyCollidableNode from the collidableExceptionGroup.
    virtual void removeFromCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId );

    /// Adds the X3DNBodyCollidableNode to the collidableSelectionGroup.
    virtual void addToCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId, bool selected = false );

    /// Removes the X3DNBodyCollidableNode from the collidableSelectionGroup.
    virtual void removeFromCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId );

  protected:

    /// The valueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// X3DNBodyCollidableNode. More specifically it calls 
    /// PhysicsEngineThread::setCollidableParameters with the new values.
    /// Every field that has a corresponding value in 
    /// PhysicsEngineThreadParameters::CollidableParameters is routed 
    /// to this field.
    /// C++ only field.
    auto_ptr< ValueUpdater > valueUpdater; 

    /// The inCollidableGroup field keeps track of which collidableGroups
    /// this X3DNBodyCollidableNode is included. By default each
    /// X3DNBodyCollidableNode is assumed to be included in the default
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

    struct EpsilonCompareFunctor {
      EpsilonCompareFunctor( H3DFloat _groupId ) : groupId( _groupId ) {};
      H3DFloat groupId;
      inline bool operator()( const H3DFloat& cmp ) const {
        return epsilonCompare( groupId, cmp );
      }
    };

  public:

    /// Physics engine options specific for this X3DNBodyCollidableNode.
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile X3DNBodyCollidableNode_engineOptions.dot
    auto_ptr< MFEngineOptions > engineOptions;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// Given a H3DCollidableId returns the X3DNBodyCollidableNode that is 
    /// associated with it.
    static X3DNBodyCollidableNode *getCollidableFromId( H3DCollidableId );

    /// Initializes the X3DNBodyCollidableNode for the given PhysicsEngineThread
    virtual int initializeCollidable( H3D::PhysicsEngineThread *pt, X3DNode *parent )
#ifndef H3D_GENERATE_DOTROUTE_FILES
      = 0;
#else
    { return 0; }
#endif

    /// Deletes this collidable shape from PhysicsEngineThread
    virtual int deleteCollidable()
#ifndef H3D_GENERATE_DOTROUTE_FILES
      = 0;
#else
    { return 0; }
#endif

    /// Returns true if this CollidableShape node has been initialized for PhysicsEngineThread.
    virtual bool collidableInitialized();

    /// Get the H3DCollidableId used by this node.
    /// This is only valid if initializeCollidable has been called before.
    inline H3DCollidableId getCollidableId() { 
      return collidable_id;
    }

    void setRigidBodyCollection( RigidBodyCollection* _rbc = NULL );

  protected:
    /// The PhysicsEngineThread that this object uses.
    H3D::PhysicsEngineThread * engine_thread;

    /// The H3DCollidableId used by nodes derived from X3DNBodyCollidableNode
    /// in the PhysicsEngineThread of the RigidBodyCollection it resides in.
    H3DCollidableId collidable_id;

    /// A map from collidable shape id to the X3DNBodyCollidableNode subclass node
    /// that is associated with that id.
    static std::map< H3DCollidableId, X3DNBodyCollidableNode * > collidable_id_map;

    /// Creates a new instance of a subclass of CollidableParameters appropriate for the subclass of collidable
    virtual PhysicsEngineParameters::CollidableParameters* createCollidableParameters () {
      return new PhysicsEngineParameters::CollidableParameters;
    }

    /// Returns a CollidableParameter to describe the collidable. By default
    /// the function returns a CollidableParameter with values that have changed
    /// since the last loop.
    //// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::CollidableParameters *getCollidableParameters( bool all_params = false );

    RigidBodyCollection* rbc;
  };
}
#endif
