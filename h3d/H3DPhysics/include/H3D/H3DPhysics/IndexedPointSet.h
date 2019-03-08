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
/// \file IndexedPointSet.h
/// \brief Header file for IndexedPointSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __INDEXEDPOINTSET__
#define __INDEXEDPOINTSET__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/MFInt32.h>
#include <H3D/SFNode.h>
#include <H3D/IndexedTriangleSet.h>   // Note: Include required to avoid error LNK2005 (vc8)
#include <H3D/X3DComposedGeometryNode.h>

namespace H3D{

  /// \ingroup SoftBody
  /// A geometry node representing a volume defined by a set of points.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/geometries/IndexedPointSet.x3d">IndexedPointSet.x3d</a>
  ///     ( <a href="examples/IndexedPointSet.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile IndexedPointSet.dot
  class H3DPHYS_API IndexedPointSet : public X3DComposedGeometryNode {
  public:

    /// The bound field for IndexedHexaSet is a CoordBoundField.
    typedef CoordBoundField SFBound;

    /// Constructor.
    IndexedPointSet(Inst< SFNode           > _metadata        = 0,
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
      Inst< MFInt32          > _index           = 0 );

    /// Indices of the points.
    /// 
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile IndexedPointSet_index.dot
    auto_ptr < MFInt32 > index;

    /// Render the point set
    virtual void render ();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Renders the point indexed by a.
    void renderPoint ( unsigned int a );

    /// Render the vertex indexed by index
    void renderVertex ( unsigned int _index );

    // Cache various paramaters during rendering to
    // avoid passing them between functions
    X3DCoordinateNode* coordinate_node;
    FogCoordinate* fog_coord_node;
    X3DColorNode* color_node;
    X3DNormalNode* normal_node;
    bool tex_coords_per_vertex;
    X3DTextureCoordinateNode *tex_coord_node;
  };
}
#endif
