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
/// \file IndexedTriangleFanSet.h
/// \brief Header file for IndexedTriangleFanSet, X3D scene-graph node
///
///
//////////////////////////////////////////////////////////////////////////////
#ifndef __INDEXEDTRIANGLEFANSET_H__
#define __INDEXEDTRIANGLEFANSET_H__

#include <H3D/X3DComposedGeometryNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/X3DCoordinateNode.h>
#include <H3D/X3DColorNode.h>
#include <H3D/CoordBoundField.h>
#include <H3D/MFInt32.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class IndexedTriangleFanSet
  /// \brief An IndexedTriangleFanSet represents a 3D shape composed of 
  /// triangles that form a fan shape around the first vertex declared
  /// in each fan.
  ///
  /// IndexedTriangleFanSet uses the indices in its index field to specify
  /// the triangle fans by connecting vertices from the coord field. An
  /// index of "-1" indicates that the current fan has ended and the next
  /// one begins. The last fan may be (but does not have to be) followed by
  /// a "-1".Each fan shall have at least three non-coincident vertices.
  ///
  /// The IndexedTriangleFanSet node is specified in the local coordinate
  /// system and is affected by the transformations of its
  /// ancestors. Descriptions of the color, coord, normal, and texCoord
  /// fields are provided in the Color, Coordinate, Normal, and
  /// TextureCoordinate nodes, respectively. If values are provided for the
  /// color, normal and texCoord fields, the values are applied in the same
  /// manner as the values from the coord field and there shall be at least
  /// as many values as are present in the coord field. The value of the
  /// colorPerVertex field is ignored and always treated as TRUE. If the
  ///normal field is not provided, normals shall be generated as follows:
  ///
  /// - If normalPerVertex is TRUE, the normal for each vertex shall be the
  ///   average of the normals for all triangles sharing that vertex. 
  ///
  /// - If normalPerVertex is FALSE, the normal shall be generated for the
  ///   current triangle based on the ccw field. 
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../../H3DAPI/examples/All/IndexedTriangleFanSet.x3d">IndexedTriangleFanSet.x3d</a>
  ///     ( <a href="examples/IndexedTriangleFanSet.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile IndexedTriangleFanSet.dot
  class H3DAPI_API IndexedTriangleFanSet : public X3DComposedGeometryNode {
  public:
        /// Thrown if the number of colors in the color field is less than
    /// the number coordinates in the coord field.
    H3D_VALUE_EXCEPTION( int, NotEnoughColors );

    /// Thrown if the number of texture coordinates in the color field is less 
    /// than the number coordinates in the coord field.
     H3D_VALUE_EXCEPTION( int, NotEnoughTextureCoordinates );

    /// The bound field for IndexedTriangleFanSet is a CoordBoundField.
    typedef CoordBoundField SFBound;

    /// Specialized field for automatically generating normals from
    /// coordinates.
    /// routes_in[0] is the normalPerVertex field.
    /// routes_in[1] is the coord field.
    /// routes_in[2] is the index field.
    /// routes_in[3] is the ccw field.
    
    class H3DAPI_API AutoNormal: 
      public TypedField< SFNormalNode,
                         Types< SFBool, SFCoordinateNode, MFInt32, SFBool > > {
      /// Calls generateNormalsPerVertex() if routes_in[0] is true, 
      /// otherwise generateNormalsPerFace() is called.
      virtual void update();

      /// Create a new X3DNormalNode from the arguments given
      /// with one normal for each vertex specified.
      ///
      /// \param coord Node with the coordinates.
      /// \param index The indices in coord for the vertices.
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
      virtual X3DNormalNode *generateNormalsPerVertex( 
                                                      X3DCoordinateNode *_coord,
                                                      const vector< int > &index,
                                                      bool _ccw );
    

      /// Create a new X3DNormalNode from the arguments given
      /// with one normal for each face specified.
      ///
      /// \param coord Node with the coordinates.
      /// \param index The indices in coord for the vertices.
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
                                                    const vector< int > &index,
                                                    bool _ccw );

    };

    /// Constructor.
    IndexedTriangleFanSet( Inst< SFNode           > _metadata        = 0,
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
                           Inst< MFInt32          > _set_index       = 0,
                           Inst< MFInt32          > _index           = 0,
                           Inst< SFFogCoordinate  > _fogCoord        = 0 );

    ~IndexedTriangleFanSet();

    // Traverse the scenegraph. See X3DGeometryNode::traverseSG
    // for more info.
    virtual void traverseSG( TraverseInfo &ti );  

    /// Renders the IndexedTriangleFanSet with OpenGL.
    virtual void render();

    /// An upper bound of the number of triangles renderered in this geometry.
    virtual int nrTriangles() {
      return index->size();
    }

    /// Auto-generated normals that are used if the normal field is NULL.
    /// Only accessable in C++.
    ///
    /// \dotfile IndexedTriangleFanSet_autoNormal.dot 
    auto_ptr< AutoNormal  >  autoNormal;

    /// Field for setting the value of the index field.
    ///
    /// <b>Access type:</b> inputOnly 
    ///
    /// \dotfile IndexedTriangleFanSet_set_index.dot 
    auto_ptr< MFInt32 > set_index;

    /// The index field is used to specify the triangle strips by connecting
    /// vertices from the coord field. An index of "-1" indicates that the
    /// current strip has ended and the next one begins. The last strip may
    /// be (but does not have to be) followed by a "-1". Each strip shall have
    /// at least three non-coincident vertices.  
    /// 
    /// <b>Access type:</b> inputOutput 
    ///
    /// \dotfile IndexedTriangleFanSet_index.dot   
    auto_ptr< MFInt32 > index;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:
    /// Internal field used to know if vertex buffer object can be created.
    /// C++ only field
    auto_ptr< Field > vboFieldsUpToDate;
    // The index for the vertex buffer object
    GLuint *vbo_id;
    // Vector needed to draw each fan using vbo.
    vector< GLsizei > nr_index_per_fan;
  };
}

#endif
