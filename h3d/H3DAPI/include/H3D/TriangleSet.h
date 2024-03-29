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
/// \file TriangleSet.h
/// \brief Header file for TriangleSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __TRIANGLESET_H__
#define __TRIANGLESET_H__

#include <H3D/X3DComposedGeometryNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/X3DCoordinateNode.h>
#include <H3D/X3DColorNode.h>
#include <H3D/CoordBoundField.h>

namespace H3D {

  /// \class TriangleSet
  /// \brief The TriangleSet node represents a 3D shape that represents a
  /// collection of individual triangles. 
  ///
  /// The coord field contains a Coordinate node that defines the 3D
  /// vertices that define the triangle. Each triangle is formed from a
  /// consecutive set of three vertices of the coordinate node. If the
  /// coordinate node does not contain a multiple of three coordinate
  /// values, the remaining vertices shall be ignored.
  ///
  /// The TriangleSet node is specified in the local coordinate system and
  /// is affected by the transformations of its ancestors. If values are 
  /// provided for the color, normal, and texCoord fields, there shall be
  /// at least as many values as are present in the coord field. The value
  /// of the colorPerVertex field is ignored and always treated as TRUE. 
  /// If the normal field is not supplied, the normal shall be generated
  /// as perpendicular to the face for either version of normalPerVertex. 
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../../H3DAPI/examples/All/TriangleSet.x3d">TriangleSet.x3d</a>
  ///     ( <a href="examples/TriangleSet.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile TriangleSet.dot
  class H3DAPI_API TriangleSet : public X3DComposedGeometryNode {
  public:
    /// Thrown if the number of colors in the color field is less than
    /// the number coordinates in the coord field.
    H3D_VALUE_EXCEPTION( int, NotEnoughColors );

    /// Thrown if the number of texture coordinates in the color field is less 
    /// than the number coordinates in the coord field.
     H3D_VALUE_EXCEPTION( int, NotEnoughTextureCoordinates );

    /// The bound field for IndexedFaceSet is a CoordBoundField.
    typedef CoordBoundField SFBound;

    /// Specialized field for automatically generating normals from
    /// coordinates.
    /// routes_in[0] is the normalPerVertex field.
    /// routes_in[1] is the coord field.
    /// routes_in[2] is the ccw field.
    
    class H3DAPI_API AutoNormal: 
      public TypedField< SFNormalNode,
                         Types< SFBool, SFCoordinateNode, SFBool > > {
      /// Calls generateNormalsPerFace().
      virtual void update();

      /// Create a new X3DNormalNode from the arguments given
      /// with one normal for each vertex specified.
      ///
      /// \param coord Node with the coordinates.
      /// \param ccw Defines the ordering of the vertex coordinates of the 
      /// geometry with respect to generated normal vectors used in the 
      /// lighting model equations. If ccw is TRUE, the normals shall 
      /// follow the right hand rule; the orientation of each normal with
      /// respect to the vertices (taken in order) shall be such that the
      /// vertices appear to be oriented in a counterclockwise order when 
      /// the vertices are viewed (in the local coordinate system of the Shape)
      /// from the opposite direction as the normal.
      /// \returns A new Normal node with a normal for each
      /// vertex.
      ///
      virtual X3DNormalNode *generateNormalsPerFace( 
                                                      X3DCoordinateNode *_coord,
                                                      bool _ccw );
    };

    /// Constructor.
    TriangleSet( Inst< SFNode           > _metadata        = 0,
                 Inst< SFBound          > _bound           = 0,
                 Inst< DisplayList      > _displayList     = 0,
                 Inst< SFColorNode      > _color           = 0,
                 Inst< SFCoordinateNode > _coord           = 0,
                 Inst< SFNormalNode     > _normal          = 0,
                 Inst< SFTextureCoordinateNode > _texCoord = 0,
                 Inst< SFBool           > _ccw             = 0,
                 Inst< SFBool           > _colorPerVertex  = 0,
                 Inst< SFBool           > _normalPerVertex = 0,
                 Inst< SFBool           > _solid           = 0,
                 Inst< MFVertexAttributeNode > _attrib     = 0,
                 Inst< AutoNormal       > _autoNormal      = 0,
                 Inst< SFFogCoordinate  > _fogCoord        = 0 );

    ///  Renders the TriangleSet with OpenGL.
    virtual void render();

    // Traverse the scenegraph. See X3DGeometryNode::traverseSG
    // for more info.
    virtual void traverseSG( TraverseInfo &ti );

    /// The number of triangles renderered in this geometry.
    virtual int nrTriangles() {
      X3DCoordinateNode *coord_node = coord->getValue();
      if( coord_node ) 
        return coord_node->nrAvailableCoords() / 3;
      else return 0;
    }

    /// Auto-generated normals that are used if the normal field is NULL.
    /// Only accessable in C++.
    ///
    /// \dotfile TriangleSet_autoNormal.dot 
    auto_ptr< AutoNormal  >  autoNormal;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
