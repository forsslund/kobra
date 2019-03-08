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
/// \file IndexedHexaSet.h
/// \brief Header file for IndexedHexaSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __INDEXEDHEXASET__
#define __INDEXEDHEXASET__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/MFInt32.h>
#include <H3D/IndexedTriangleSet.h>   // Note: Include required to avoid error LNK2005 (vc8)
#include <H3D/X3DBindableNode.h>   // Note: Include required to avoid error LNK2005 (vc10)
#include <H3D/X3DComposedGeometryNode.h>

namespace H3D{

  /// \ingroup SoftBody
  /// A geometry node representing a volume defined by a set of hexahedra
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/geometries/IndexedHexaSet.x3d">IndexedHexaSet.x3d</a>
  ///     ( <a href="examples/IndexedHexaSet.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile IndexedHexaSet.dot
  class H3DPHYS_API IndexedHexaSet : public X3DComposedGeometryNode {
  public:

    /// Checks if the faces provided by the coord and indices
    /// fields create valid quads when set to true.
    class H3DPHYS_API VerifyHexaSet: public H3D::AutoUpdate< SFBool > {

      /// Calls verifyHexaFace for all the hexahedrons in the
      /// hexaSet.
      virtual void update();

      /// Checks if the given vertices create a valid quad.
      bool verifyHexaFace ( int a, int b, int c, int d, X3DCoordinateNode* _coord );
    };

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

      /// Generate the normal for the hexa given by the indices specified
      Vec3f hexaNormal ( int a, int b, int c, int d, X3DCoordinateNode* _coord, bool _ccw );
    };

    /// The bound field for IndexedHexaSet is a CoordBoundField.
    typedef CoordBoundField SFBound;

    /// Constructor.
    IndexedHexaSet(
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
      Inst< AutoNormal       > _autoNormal      = 0 );



    //      7__________ 6
    //     /           /|
    //    /           / |
    //   /           /  |
    //  3___________2   |
    //  |           |   |
    //  |           |   |
    //  |           |   /5
    //  |           |  /
    //  |           | /
    //  |___________|/
    //  0           1
    /// Indices of the hexahedrons.
    /// Each hexahedron is defined by 8 consecutive indices, pointing
    /// to vertices in the coord field. If the first 4 indices of the
    /// hexahedron forms a polygon the last 4 indices should be given
    /// to form a polygon which does not intersect the first one. The
    /// order of first and last 4 indices should be given in a consistent
    /// way s.t, they would create the same path when observe from the same
    /// side. If the first four indices are specified in a clock wise way
    /// then ccw should be set to false.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile IndexedHexaSet_index.dot
    auto_ptr < MFInt32 > index;

    /// Checks if the coords and indices given create valid quads
    /// when set to True. In case of existence of invalid quad
    /// prints a message in the console.
    /// Only accessable in C++.
    auto_ptr< VerifyHexaSet >  verifyHexaSet;

    /// Auto-generated normals that are used if the normal field is NULL.
    /// Only accessable in C++.
    ///
    /// \dotfile IndexedHexaSet_autoNormal.dot 
    auto_ptr< AutoNormal  >  autoNormal;

    /// Render the hexahedron set
    virtual void render ();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Renders the hexa indexed by a, b, c and d.
    void renderHexa ( unsigned int a, unsigned int b, unsigned int c, unsigned int d );

    /// Render the vertex indexed by index
    void renderVertex ( unsigned int _index );

    // Cache various parameters during rendering to
    // avoid passing them between functions
    X3DCoordinateNode* coordinate_node;
    FogCoordinate* fog_coord_node;
    X3DColorNode* color_node;
    X3DNormalNode* normal_node;
    unsigned int hexaIndex;
    bool vertexNormals, tex_coords_per_vertex;
    X3DTextureCoordinateNode *tex_coord_node;
  public:
    // Traverse the scenegraph. See X3DGeometryNode::traverseSG
    // for more info.
    virtual void traverseSG( TraverseInfo &ti );
  };
}
#endif
