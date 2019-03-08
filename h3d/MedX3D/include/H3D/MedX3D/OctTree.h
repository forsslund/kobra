//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file OctTree.h
/// \brief Header file for OctTree, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __OCTTREE_H__
#define __OCTTREE_H__

#include <H3D/MedX3D/X3DVolumeNode.h>

#include <H3D/X3DChildNode.h>
#include <H3D/X3DBoundedObject.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/H3DDisplayListObject.h>
#include <H3D/SFFloat.h>
#include <H3D/X3DGroupingNode.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/MatrixTransform.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class OctTree
  /// \brief Allows for the definition of multiresolution data sets that 
  ///  resolve using octants of volume. This node is not restricted to only
  ///  having volume data as its children - all other geometry types are
  ///  also valid structures.
  ///
  ///  The level of detail is switched depending upon whether the user is
  ///  closer or further than range metres from the coordinate center.
  ///  
  ///  The lowRes field holds the low resolution object instance to be
  ///  rendered when the viewer is outside range metres. The highRes field
  ///  is used to hold the geometry to be viewed when the inside range metres.
  ///  An OctTree renders up to 8 children sub graphs as defined by the
  ///  highRes field. If this field contains more than 8 children, only the
  ///  first 8 shall be rendered. If less than 8 children are defined, all
  ///  shall be rendered. It is up to the user to spatially located the 
  ///  geometry for each of the children subgraphs.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/OctTreeTest.x3d">OctTreeTest.x3d</a>
  ///     ( <a href="x3d/OctTreeTest.x3d.html">Source</a> )
  class MEDX3D_API OctTree : public X3DChildNode, 
                             public X3DBoundedObject, 
                             public H3DDisplayListObject {
  public:
    class MEDX3D_API MFChild :
      public DependentMFNode< X3DChildNode,
                              FieldRef< H3DDisplayListObject,
                                        H3DDisplayListObject::DisplayList,
                                        &H3DDisplayListObject::displayList >,
                              true >  {
    public:
      /// The base class of MFChild.
      typedef DependentMFNode< X3DChildNode,
                               FieldRef< H3DDisplayListObject,
                                         H3DDisplayListObject::DisplayList,
                                         &H3DDisplayListObject::displayList >,
                               true >  MFChildBase;

      /// Destructor. Clears the MFNode in order to get the correct onRemove 
      /// function to be called upon destruction. Must be done in all
      /// subclasses that overrides onRemove ().
      ~MFChild() {
        clear();
      }
      /// Set up the routes we want to maintain.
      virtual void onAdd( Node *n );

      /// Remove the routes we want to maintain.
      virtual void onRemove( Node *n );

    private:
      friend class OctTree;
    };

    class SFLowResNode :
      public DependentSFNode< X3DNode,
                              FieldRef< H3DDisplayListObject,
                                        H3DDisplayListObject::DisplayList,
                                        &H3DDisplayListObject::displayList >,
                              true >  {
    public:
      typedef DependentSFNode< X3DNode,
                               FieldRef< H3DDisplayListObject,
                                         H3DDisplayListObject::DisplayList,
                                         &H3DDisplayListObject::displayList >,
                               true >  SFNodeBase;

      /// Destructor. Clears the SFLowResNode in order to get the correct
      /// onRemove function to be called upon destruction. Must be done in all
      /// subclasses that overrides onRemove ().
      ~SFLowResNode() {
        value = NULL;
      }


      /// Set up the routes we want to maintain.
      virtual void onAdd( Node *n );

      /// Remove the routes we want to maintain.
      virtual void onRemove( Node *n );

    private:
      friend class OctTree;
    };

    /// SFBound is specialized to update from the SFBound fields 
    /// routed to it. The resulting Bound object is the union of 
    /// the Bound objects routed to it. If the bboxSize of the 
    /// X3DGrouping node containing the SFBound field is ( -1, -1, -1 )
    /// bound fields of all Nodes in the children field of the containing
    /// Node that are instances of X3DBoundedObject are routed to it.
    /// Otherwise the bound will be a BoxBound with center and
    /// radius specified with the bboxCenter and bboxSize fields.
    ///
    class MEDX3D_API SFBound: 
      public TypedField< X3DBoundedObject::SFBound,
                         void,
                         AnyNumber< X3DBoundedObject::SFBound > > {
    protected:
      /// The SFBound is updated to a bound that is the union of the 
      /// the Bound objects routed to it.
        virtual void update();
    };

    /// Constructor.
    OctTree(  Inst< MFChild       > _highRes  = 0,
              Inst< SFLowResNode  > _lowRes           = 0, 
              Inst< SFNode        > _metadata         = 0, 
              Inst< SFBool        > _lowResActive     = 0,
              Inst< SFVec3f       > _center           = 0,
              Inst< SFFloat       > _range            = 0,
              Inst< SFBound       > _bound            = 0,
              Inst< SFVec3f       > _bboxCenter       = 0,
              Inst< SFVec3f       > _bboxSize         = 0,
              Inst< DisplayList   > _displayList      = 0);

    virtual void render();

    /// Traverse the scenegraph. Saves the accumulated inverse
    /// matrix for later use when transforming the X3DViewpointNode in
    /// GLWindow.
    virtual void traverseSG( TraverseInfo &ti );
    
    /// Detect intersection between a line segment and a Node.
    /// Calls lineIntersect for all highRes nodes or the lowRes Node.
    /// \param from The start of the line segment.
    /// \param to The end of the line segment.
    /// \param result Contains info about the closest intersection for every
    /// object that intersects the line.
    /// \returns true if intersected, false otherwise.
    virtual bool lineIntersect( const Vec3f &from, 
                                const Vec3f &to,    
                                LineIntersectResult &result );

    /// Sets up the bound field using the bboxCenter and bboxSize fields.
    /// If bboxSize is (-1, -1, -1) the bound will be the union of all the
    /// bound objects of the Nodes in the children field. Otherwise it will
    /// be a BoxBound with center and origin determined by the bboxCenter
    /// and bboxOrigin fields.
    virtual void initialize() {
      const Vec3f &size = bboxSize->getValue();
      if( size.x == -1 && size.y == -1 && size.z == -1 ) {
        //NodeVector children_nodes = children->getValue();
        use_union_bound = true;
        // set the values againg in order to run onNodeAdd again
        // with use_union_bound set to true to set the routes.
        //children->setValue( children_nodes );
      } else {
        use_union_bound = false;
        BoxBound *bb = new BoxBound();
        bb->center->setValue( bboxCenter->getValue() );
        bb->size->setValue( bboxSize->getValue() );
        bound->setValue( bb );
      }
      X3DChildNode::initialize();
    }

    /// Find closest point on Node to p. Calls closestPoint for
    /// all children
    /// \param p The point to find the closest point to.
    /// \param result A struct containing various results of closest
    /// points such as which geometries the closest points where
    /// detected on.
    virtual void closestPoint( const Vec3f &p,
                               NodeIntersectResult &result );

    /// Detect collision between a moving sphere and the Node.
    /// Calls movingSphereIntersect for all children
    /// \param radius The radius of the sphere
    /// \param from The start position of the sphere
    /// \param to The end position of the sphere.
    /// \param result A struct containing various results of intersections
    /// such as which geometries intersected the moving sphere.
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersect( H3DFloat radius,
                                        const Vec3f &from, 
                                        const Vec3f &to,
                                        NodeIntersectResult &result );

    /// if true a route will be set up between the bound field of the
    /// nodes in children and the bound field of the grouping node. 
    bool use_union_bound;

    /// The highRes field is used to hold the geometry to be viewed
    /// when the viewer is inside the range.
    /// 
    /// <b>Access type:</b> InputOutput \n
    /// <b>Default value:</b> NULL \n
    auto_ptr< MFChild > highRes;


    /// The lowRes field holds the low resolution object instance to be 
    /// rendered when the viewer is outside the range.
    ///
    /// <b>Access type:</b> InputOutput \n
    /// <b>Default value:</b> NULL \n
    auto_ptr <SFLowResNode> lowRes; 

    /// The lowResActive boolean declares if lowRes or highRes should
    /// be rendered.
    ///
    /// <b>Access type:</b> OutputOnly \n
    auto_ptr <SFBool> lowResActive; 

    ///  The coordinate center.
    ///
    /// <b>Access type:</b> InputOutput \n
    /// <b>Default value:</b> 0 0 0 \n
    auto_ptr< SFVec3f > center;


    /// The range field 
    ///
    /// <b>Access type:</b> InputOutput \n
    /// <b>Default value:</b> 20 \n
    auto_ptr< SFFloat > range;
   
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
