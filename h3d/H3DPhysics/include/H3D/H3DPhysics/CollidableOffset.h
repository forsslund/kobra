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
/// \file CollidableOffset.h
/// \brief Header file for CollidableOffset, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __COLLIDABLEOFFSET__ 
#define __COLLIDABLEOFFSET__

#include <H3D/SFVec3f.h>
#include <H3D/SFRotation.h>
#include <H3D/SFBool.h>
#include <H3D/SFNode.h>
#include <H3D/Shape.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>


namespace H3D{
  /// \ingroup X3DNodes
  /// \class CollidableOffset
  /// \brief The CollidableOffset node is used to reposition a piece of
  /// geometry relative to the center of the owning body while keeping it
  /// consistent within the geometry space.
  ///
  /// The collidable field holds a reference to a single nested item of
  /// collidable scene graph. If there are multiple transformation paths to
  /// this reference, the results are undefined.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/CollidableOffset.x3d">CollidableOffset.x3d</a>
  ///     ( <a href="examples/CollidableOffset.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile CollidableOffset.dot
  class H3DPHYS_API CollidableOffset :
    public X3DNBodyCollidableNode {
  public:

    class H3DPHYS_API SFCollidable : public TypedSFNode< X3DNBodyCollidableNode > {
    public:
      /// Destructor. On destruction, call deleteCollidable on value
      ~SFCollidable() {
        if ( value.get() )
          static_cast< X3DNBodyCollidableNode* >( value.get() )->deleteCollidable();
      }

    protected:
      /// On removal of a node from collidable field, delete the collidable
      virtual void onRemove( X3DNBodyCollidableNode *n ) {
        n->deleteCollidable();
        TypedSFNode< X3DNBodyCollidableNode >::onRemove( n );
      }
    };

    /// Constructor.
    CollidableOffset( Inst< SFBool > _enabled = 0,
      Inst< SFNode  >  _metadata = 0, 
      Inst< SFRotation >  _rotation = 0,
      Inst< SFVec3f >  _translation = 0,
      Inst< SFVec3f >  _scale = 0,
      Inst< SFBound     > _bound = 0,
      Inst< SFVec3f >  _bboxCenter = 0,
      Inst< SFVec3f  >  _bboxSize = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFCollidable > _collidable = 0);

    ~CollidableOffset();

    /// The collidable field holds a reference to a single nested item of
    /// collidable scene graph. If there are multiple transformation paths to
    /// this reference, the results are undefined.
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile CollidableOffset_collidable.dot
    auto_ptr< SFNode > collidable;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// Initializes the CollidableOffset for the given PhysicsEngineThread
    virtual int initializeCollidable( H3D::PhysicsEngineThread *pt, X3DNode *parent );

    /// Deletes this collidable shape from PhysicsEngineThread
    virtual int deleteCollidable();

    /// Adds the CollidableOffset and its collidable to the collidableGroup.
    virtual void addToCollidableGroup( H3DCollidableGroupId groupId );

    /// Removes the CollidableOffset and its collidable from the collidableGroup.
    virtual void removeFromCollidableGroup( H3DCollidableGroupId groupId );

    /// Gets the Shape node from the shape field of child CollidableShape, if any
    Shape * getShape();

  protected:

    /// Creates a new instance of a subclass of CollidableParameters appropriate for
    /// the subclass of collidable
    virtual PhysicsEngineParameters::OffsetParameters* createCollidableParameters () {
      return new PhysicsEngineParameters::OffsetParameters;
    }

    /// Returns a CollidableParameter to describe the collidable. By default
    /// the function returns a CollidableParameter with values that have changed
    /// since the last loop.
    //// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::OffsetParameters *getCollidableParameters( bool all_params = false );
  };
}
#endif
