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
/// \file CollidableShape.h
/// \brief Header file for CollidableShape, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __COLLIDABLESHAPE__
#define __COLLIDABLESHAPE__

#include <H3D/MFVec4f.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFRotation.h>
#include <H3D/SFBool.h>
#include <H3D/SFNode.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/Box.h>
#include <H3D/Sphere.h>
#include <H3D/Cylinder.h>
#include <H3D/X3DGeometryNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>
#include <H3D/ClipPlane.h>
#include <H3D/Inline.h>


namespace H3D {
  /// \ingroup X3DNodes
  /// \class CollidableShape
  /// \brief The CollidableShape node represents the glue between the collision
  /// detection system, the rigid body model, and the renderable scene graph.
  /// Its job is to take a single piece of geometry wrapped in a Shape node
  /// and provide a way for the physics model body to move the geometry. In
  /// addition, it allows the collision detection system to determine the
  /// location of the geometry primitives that it uses for collision
  /// management. When placed under a part of the transformation hierarchy, it
  /// can be used to visually represent the movement of the object.
  ///
  /// The shape field uses the geometry proxy for specifying which geometry
  /// best represents the collidable object.
  ///
  /// NOTE  Since the shape node is still writable, it is strongly recommended
  /// that the author not dynamically change the Shape's geometry field as it
  /// may have large performance impacts due to optimizations used by the
  /// collision system.
  ///
  /// Not all geometry types are mappable to the collision node type.
  ///
  /// EXAMPLE  PointSet
  ///
  /// If the containing shape node is given an explicit bounding box size, the
  /// geometry shall be approximated using that shape for the purposes of
  /// collision detection. If there is no bounding box, the results are
  /// implementation-dependent.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/CollidableShape.x3d">CollidableShape.x3d</a>
  ///     ( <a href="examples/CollidableShape.x3d.html">Source</a> )
  ///   - <a href="../../examples/RigidBody/CollidableShapeWithClipPlane.x3d">CollidableShapeWithClipPlane.x3d</a>
  ///     ( <a href="examples/CollidableShapeWithClipPlane.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile CollidableShape.dot
  class H3DPHYS_API CollidableShape :
    public X3DNBodyCollidableNode {
  public:

    /// Subclass SFBound since it will depend on two fields of the node and
    /// also the bound of the contained Shape node.
    ///
    /// routes_in[0] - The bboxCenter field of the owner CollidableShape.
    /// routes_in[1] - The transRotMatrix field of the owner CollidableShape.
    class H3DPHYS_API SFBound: 
      public TypedField< X3DNBodyCollidableNode::SFBound,
      Types< SFMatrix4f >,
      X3DNBodyCollidableNode::SFBound > {
    protected:
      /// Updates the bound.
      virtual void update();

      /// Set up the routes from the owner Node's bound
      /// and transRotMatrix field to the contained TransformedBoxBound
      /// instance's boxBound and transRotMatrix field.
      virtual void onAdd( Bound *b ) {
        X3DNBodyCollidableNode::SFBound::onAdd( b );
        TransformedBoxBound *tbb = dynamic_cast< TransformedBoxBound * >( b );
        if( b && tbb ) {
          if( owner ) {
            CollidableShape *cs = static_cast< CollidableShape * >(owner);
            cs->bound->route( tbb->boxBound );
            cs->transRotMatrix->route( tbb->matrix );
          }
        }
      }

      /// Remove the routes that were set up in onAdd().
      virtual void onRemove( Bound *b ) {
        TransformedBoxBound *tbb = dynamic_cast< TransformedBoxBound * >( b );
        if( b && tbb ) {
          // Check if owner exists since it might be deleted before this code
          // is called.
          if( owner ) {
            CollidableShape *cs = static_cast< CollidableShape * >(owner);
            cs->bound->unroute( tbb->boxBound );
            cs->transRotMatrix->unroute( tbb->matrix );
          }
        }
        X3DNBodyCollidableNode::SFBound::onRemove( b );
      }
    };

    /// SFShapeNode is dependent on the displayList field of its
    /// encapsulated Shape node, i.e. an event from that
    /// field will trigger an event from the SFShapeNode as
    /// well. Also we ovverride onAdd and onRemove in order to uphold
    /// a route between the encapsulated Shape's bound field
    /// to the bound field of the Shape the field resides in.
    class SFShapeNode: 
      public DependentSFNode< X3DShapeNode, 
      FieldRef< H3DDisplayListObject,
      H3DDisplayListObject::DisplayList,
      &H3DDisplayListObject::displayList >, true > {
        typedef DependentSFNode< X3DShapeNode, 
          FieldRef< H3DDisplayListObject,
          H3DDisplayListObject::DisplayList,
          &H3DDisplayListObject::displayList >, 
          true > BaseField;

    public:

      /// Destructor. Sets the value to NULL in order to get the correct
      /// onAdd and onRemove functions to be called. 
      virtual ~SFShapeNode() {
        value = NULL;
      }

      /// This function will be called when the value of RefCountField
      /// changes. As soon as the value changes onRemove will
      /// be called on the old value and onAdd will be called
      /// on the new value. 
      /// \param n The new value.
      ///
      virtual void onAdd( Node *n ) {
        BaseField::onAdd( n );
        CollidableShape *_shape = 
          static_cast< CollidableShape * >( getOwner() );
        if( _shape && n ) {
          X3DShapeNode *g = static_cast< X3DShapeNode * >( n );
          g->bound->route( _shape->bound );
        }
      }

      /// This function will be called when the value of RefCountField
      /// changes. As soon as the value changes onRemove will
      /// be called on the old value and onAdd will be called
      /// on the new value. 
      /// \param n The old value.
      ///
      virtual void onRemove( Node *n ) {
        BaseField::onRemove( n );
        CollidableShape *_shape =
          static_cast< CollidableShape * >( getOwner() );
        // Check if shape exists since it might be deleted before this code is
        // called.
        if( _shape && n ) {
          X3DShapeNode *g = static_cast< X3DShapeNode * >( n );
          g->bound->unroute( _shape->bound );
        }
      }
    };

    /// Field class that outputs a matrix created from the input fields.
    ///
    /// routes_in[0] - The translation field of the owner CollidableShape.
    /// routes_in[1] - The rotation field of the owner CollidableShape.
    /// routes_in[1] - The scale field of the owner CollidableShape.
    class TransformMatrix:
      public AutoUpdate< TypedField< SFMatrix4f,
      Types< SFVec3f, SFRotation, SFVec3f > > > {
    protected:
      virtual void update();
    };

    /// MFClipPlaneNode is dependent on the displayList field of its
    /// encapsulated ClipPlane node, i.e. an event from that
    /// field will trigger an event from the MFClipPlaneNode as well.
    typedef DependentMFNode< ClipPlane,
                             FieldRef<H3DDisplayListObject, 
                                      H3DDisplayListObject::DisplayList,
                                      &H3DDisplayListObject::displayList >, 
                             true > 
    MFClipPlaneNode;

    /// Constructor.
    CollidableShape( Inst< SFBool      > _enabled = 0,
      Inst< SFNode      > _metadata = 0, 
      Inst< SFRotation  > _rotation = 0,
      Inst< SFVec3f     > _translation = 0,
      Inst< SFVec3f     >  _scale = 0,
      Inst< SFBound     > _bound = 0,
      Inst< SFVec3f     > _bboxCenter = 0,
      Inst< SFVec3f     > _bboxSize = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFShapeNode > _shape = 0,
      Inst< SFBool      > _selfCollide = 0,
      Inst< MFClipPlaneNode > _clipPlanes = 0 ,
      Inst< SFBool  > _updateShapeBounds = 0);

    ~CollidableShape();

    /// Sets the default bounding box.
    virtual void initialize();

    /// Render the CollidableShape with OpenGL.
    virtual void render();

    /// The shape field uses the geometry proxy for specifying which geometry 
    /// best represents the collidable object.
    /// 
    /// <b>Access type: </b> initializeOnly
    /// <b>Default value: </b> NULL
    ///
    /// \dotfile CollidableShape_shape.dot
    auto_ptr< SFNode > shape;

    /// If true then the collidableShape can collide with itself.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> false
    ///
    /// \dotfile CollidableShape_selfCollide.dot
    auto_ptr< SFBool > selfCollide;

    /// The plane equation of the contained clip planes will be used
    /// to clip the collidable shape. The contact points considered
    /// for collision are the points that satisfy the
    /// inequation (as for the clip plane node):
    /// plane.x*x+plane.y*y+plane.z*z+plane.w >= 0
    /// 
    /// for all the planes. Defining several number of planes might
    /// have effects on performance. 
    /// 
    /// \note It is not a part of X3D Standard. Implemented only in PhysX3.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// 
    /// \dotfile CollidableShape_clipPlanes.dot
    auto_ptr< MFClipPlaneNode > clipPlanes;

    /// If false then the shapes bound tree is not updated. Memory Optimisation.
    /// Set this to false if it is known that the collidable will be handled as
    /// a primitive by the underlying physics engine and not as a triangle mesh.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> true
    ///
    /// \dotfile CollidableShape_updateShapeBounds.dot
    auto_ptr< SFBool > updateShapeBounds;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// Initializes the CollidableShape for the given PhysicsEngineThread i.e. 
    /// creates a new collidable object in the physics engine with the parameters
    /// of the CollidableShape fields. Returns 0 on success.
    virtual int initializeCollidable( H3D::PhysicsEngineThread *pt, X3DNode *parent );

    /// Deletes this collidable shape from PhysicsEngineThread.
    virtual int deleteCollidable();

  protected:

    /// Creates a new instance of a subclass of CollidableParameters appropriate for the subclass of collidable
    virtual PhysicsEngineParameters::ShapeParameters* createCollidableParameters () {
      return new PhysicsEngineParameters::ShapeParameters;
    }

    /// Returns a CollidableParameter to describe the collidable. By default
    /// the function returns a CollidableParameter with values that have changed
    /// since the last loop.
    //// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::ShapeParameters* getCollidableParameters( bool all_params = false );

    /// Used to create a matrix from the translation and rotation fields.
    /// C++ only field.
    auto_ptr< TransformMatrix > transRotMatrix;

    void updateNodeChanges( H3D::PhysicsEngineThread *pt );
    typedef std::list< pair< bool, AutoRef< Node > > > NodesChangedList;
    NodesChangedList nodes_changed;

    /// This Inline node is used to visualize "cooked" files for collidables
    /// using the PhysX3CollidableOptions in order to subdivide the CollisionShape
    /// into convex parts.
    auto_ptr< Inline > debug_collidable_inline; 
  };
}
#endif
