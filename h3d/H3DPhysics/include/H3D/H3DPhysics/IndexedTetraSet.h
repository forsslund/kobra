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
/// \file IndexedTetraSet.h
/// \brief Header file for IndexedTetraSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __INDEXEDTETRASET__
#define __INDEXEDTETRASET__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/MFInt32.h>
#include <H3D/IndexedTriangleSet.h>   // Note: Include required to avoid error LNK2005 (vc8)
#include <H3D/X3DComposedGeometryNode.h>

namespace H3D{

  /// \ingroup SoftBody
  /// A geometry node representing a volume defined by a set of tetrahedra
  ///
  /// IndexedTetraSet uses the indices in its index field to
  /// specify the vertices of each tetrahedron from the coord field. Each
  /// tetrahedron is formed from a set of four vertices of the Coordinate
  /// node identified by four consecutive indices from the index field. If
  /// the index field does not contain a multiple of four coordinate
  /// values, the remaining vertices shall be ignored.
  ///
  /// The IndexedTetraSet node is specified in the local coordinate
  /// system and is affected by the transformations of its
  /// ancestors.  If values are provided for the
  /// color, normal and texCoord fields, the values are applied in the same
  /// manner as the values from the coord field and there shall be at least
  /// as many values as are present in the coord field. The value of the
  /// colorPerVertex field is ignored and always treated as TRUE. If the
  /// normal field is not supplied, normals shall be generated as follows:
  /// 
  /// - If normalPerVertex is TRUE, the normal at each vertex shall be the
  /// average of the normals for all triangles that share that vertex. 
  /// 
  /// - If normalPerVertex is FALSE, the normal at each vertex shall be
  /// perpendicular to the face for that triangle. 
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/SoftBody.x3d">SoftBody.x3d</a>
  ///     ( <a href="examples/SoftBody.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile IndexedTetraSet.dot
  class H3DPHYS_API IndexedTetraSet : public X3DComposedGeometryNode {
    public:

    /// The bound field for IndexedTetraSet is a CoordBoundField.
    typedef CoordBoundField SFBound;

    /// Specialized field for automatically generating normals from
    /// coordinates.
    ///
    /// routes_in[0] is the normalPerVertex field.
    /// routes_in[1] is the coord field.
    /// routes_in[2] is the index field.
    /// routes_in[3] is the ccw field.
    class H3DPHYS_API AutoNormal: 
      public TypedField< SFNormalNode,
                         Types< SFBool, SFCoordinateNode, MFInt32, SFBool > > {
    protected:
      /// Calls generateNormalsPerVertex() if routes_in[0] is true, 
      /// otherwise generateNormalsPerFace() is called.
      virtual void update();

      /// Create a new X3DNormalNode from the arguments given
      /// with one normal for each vertex specified.
      ///
      /// \param _coord Node with the coordinates.
      /// \param index The indices in coord for the vertices.
      /// \param _ccw Defines the ordering of the vertex coordinates of the 
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
      /// \param _coord Node with the coordinates.
      /// \param index The indices in coord for the vertices.
      /// \param _ccw Defines the ordering of the vertex coordinates of the 
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

      /// Generate the normal for the triangle given by the indices specified
      Vec3f triangleNormal ( int a, int b, int c, X3DCoordinateNode* _coord, bool _ccw );
    };

    /// Specialized field for automatically generating two FloatVertexAttribute
    /// nodes representing the tangent and binormal of each vertex(or face if
    /// normalPerVertex is false). These can then be used in shader nodes
    /// such as PhongShader.
    ///
    /// routes_in[0] is the normalPerVertex field.
    /// routes_in[1] is the coord field.
    /// routes_in[2] is the index field.
    /// routes_in[3] is the texCoord field.
    
    class H3DPHYS_API AutoTangent: 
      public TypedField< MFVertexAttributeNode,
  Types< SFBool, SFCoordinateNode, MFInt32, SFTextureCoordinateNode > > {

      /// Calls generateTangentsPerVertex() if routes_in[0] is true, 
      /// otherwise generateTangentsPerFace() is called.
      virtual void update();

      /// Set the vales in the tangent and binormal arguments
      /// to the tangent and binormal for each vertex.
      ///
      /// \param coord Node with the coordinates.
      /// \param tex_coord Node with the texture coordinates.
      /// \param index The indices in coord for the vertices.
      virtual void generateTangentsPerVertex( 
                X3DCoordinateNode *_coord,
                X3DTextureCoordinateNode *tex_coord,
                const vector< int > &index,
                FloatVertexAttribute *tangent,
                FloatVertexAttribute *binormal
                );
    

      /// Set the vales in the tangent and binormal arguments
      /// to the tangent and binormal for each face.
      ///
      /// \param coord Node with the coordinates.
      /// \param tex_coord Node with the texture coordinates.
      /// \param index The indices in coord for the vertices.
      virtual void generateTangentsPerFace( 
             X3DCoordinateNode *_coord,
             X3DTextureCoordinateNode *tex_coord,
             const vector< int > &index,
             FloatVertexAttribute *tangent,
             FloatVertexAttribute *binormal );

      /// Returns the texture coordinate for a given index. If no
      /// texture coordinate node is specified it will use the
      /// default texture coordinate generation for an IndexedTetraSet.
      Vec3f getTexCoord( X3DCoordinateNode *_coord,
                         X3DTextureCoordinateNode *tex_coord,
                        int index );

      /// Calculate the tangent and binormal for a triangle.
      /// a,b and c are the coordinates of the triangle vertices.
      /// ta, tb and tc are the texture coordinates of the triangle vertices.
      /// tangent and binormal are output parameters that are set by the function.
      void calculateTangent( const Vec3f &a, const Vec3f &b, const Vec3f &c,
                             const Vec3f &ta, const Vec3f &tb, const Vec3f &tc,
                             Vec3f &tangent, Vec3f &binormal );

      /// Calculates the tangent and binormal for the triangle specified by
      /// index, adds the tangent and binormal to the tangents and binormals
      /// vectors for each vertex in the triangle.
      void calculateAndAddTangentPerVertex( X3DCoordinateNode *_coord,
                                           X3DTextureCoordinateNode *tex_coord,
                                            vector< H3DFloat > &tangents,
                                            vector< H3DFloat > &binormals,
                                            int i, int j, int k );

      /// Calculates the tangent and binormal for the triangle specified by
      /// index, adds the tangent and binormal to the tangents and binormals
      /// vectors.
      void calculateAndAddTangentPerFace( X3DCoordinateNode *_coord,
                                          X3DTextureCoordinateNode *tex_coord,
                                          vector< H3DFloat > &tangents,
                                          vector< H3DFloat > &binormals,
                                          int i, int j, int k );

    };

    /// Constructor.
    IndexedTetraSet(
                      Inst< SFNode           > _metadata        = 0,
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
                      Inst< SFFogCoordinate     > _fogCoord     = 0,
                      Inst< MFInt32          > _index           = 0,
                      Inst< AutoNormal       > _autoNormal      = 0,
                      Inst< SFString         > _renderMode      = 0 );

    ~IndexedTetraSet();

    // Traverse the scenegraph. See X3DGeometryNode::traverseSG
    // for more info.
    virtual void traverseSG( TraverseInfo &ti );

    /// Indices of the tetrahedrons.
    /// Each tetrahedron is defined by 4 consecutive indices, pointing to vertices in the coord field.
    /// The indexes should probably be given such that the following index order will form a counter clockwise
    /// orientation when viewed from outside (is the start index of each 4 consecutive indices )
    /// ( i, i+1, i+2 ), ( i, i+2, i+3 ), ( i, i+3, i+1 ), ( i+1, i+3, i+2 )
    /// If this is not the case then automatic normal calculations for normal per vertex
    /// might be incorrect. If ccw is false then that index order should form a clockwise
    /// orientation from the outside.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile IndexedTetraSet_index.dot
    auto_ptr < MFInt32 > index;

    /// Returns true if this geometry supports the automatic generation
    /// of tangents and binormals as FloatVertexAttribues(needed by
    /// e.g. PhongShader.
    /// IndexedTetraSet does support this.
    virtual bool supportsTangentAttributes() {
      return true;
    }

    /// Auto-generated normals that are used if the normal field is NULL.
    /// Only accessable in C++.
    ///
    /// \dotfile IndexedTetraSet_autoNormal.dot 
    auto_ptr< AutoNormal > autoNormal;

    /// Auto-generated vertex attributes for tangents and binormals.
    /// Only accessable in C++.
    ///
    /// \dotfile IndexedTetraSet_autoTangent.dot 
    auto_ptr< AutoTangent > autoTangent;

    /// The renderMode field decides with what OpenGL primitives the 
    /// IndexedTetraSet will be rendered.
    /// "TRIANGLES" - each side of each tetra will be rendered with a triangle primitive
    /// "LINES_ADJACENCY" - each tetra will be rendered as a GL_LINES_ADJACENCY
    /// primitive.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "TRIANGLES" \n
    /// 
    /// \dotfile IndexedTetraSet_renderMode.dot
    auto_ptr< SFString > renderMode;

    /// Render the tetrahedron set
    virtual void render ();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Renders the trianlge indexed by a, b and c
    void renderTriangle ( unsigned int a, unsigned int b, unsigned int c );

    /// Render the vertex indexed by index
    void renderVertex ( unsigned int _index );

    // Calculates the indices for triangles from the given indices of tetras.
    void getTriangleIndices( vector< int > &triangle_indices,
                             const vector< int > &tetra_indices );

    // Cache various paramaters during rendering to
    // avoid passing them between functions
    X3DCoordinateNode* coordinate_node;
    FogCoordinate* fog_coord_node;
    X3DColorNode* color_node;
    X3DNormalNode* normal_node;
    X3DTextureCoordinateNode *tex_coord_node;
    bool tex_coords_per_vertex;
    unsigned int triIndex;
    bool vertexNormals;

    /// This will be set to true in traverseSG if the render function
    /// is supposed to render tangent vertex attributes.
    bool render_tangents;

    /// Internal field used to know if vertex buffer object can be created.
    /// C++ only field
    auto_ptr< Field > vboFieldsUpToDate;
    // The index for the vertex buffer object
    GLuint *vbo_id;
  };
}
#endif
