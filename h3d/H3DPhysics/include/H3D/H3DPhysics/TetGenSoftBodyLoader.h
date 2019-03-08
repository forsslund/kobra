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
/// \file TetGenSoftBodyLoader.h
/// \brief Header file for TetGenSoftBodyLoader, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __TETGENTETRALOADER__
#define __TETGENTETRALOADER__

#include <H3D/H3DPhysics/H3DSoftBodyLoader.h>
#include <H3D/H3DPhysics/IndexedTetraSet.h>
#include <H3D/H3DPhysics/FieldTemplates.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// Node used to load a SoftBody representation from
  /// files in Tetgen's (http://tetgen.berlios.de/) output format.
  /// The SoftBody may then be used in the scene graph or saved in X3D format.
  ///
  /// 3 input files are expected: filename.node, filename.ele and filename.face, which should
  /// be output from Tetgen in plain text format, where 'filename' is the string specified 
  /// in the filename field.
  ///
  /// Both the internal collision and surface geometry will be loaded, as an IndexedTetraSet,
  /// IndexedTriangleSet and IndexedTriangleSet respectively. The surface, collision
  /// and internal geometry will share the same Coordinate node.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/TetGen/TetGenConverter.x3d">TetGenConverter.x3d</a>
  ///     ( <a href="examples/TetGenConverter.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile TetGenSoftBodyLoader.dot
  class H3DPHYS_API TetGenSoftBodyLoader : public H3DSoftBodyLoader {
  public:

    typedef SpecializedSFNode < IndexedTetraSet, SFX3DGeometryNode > SFIndexedTetraSet;
    typedef SpecializedMFNode < X3DComposedGeometryNode, MFX3DGeometryNode > MFX3DComposedGeometryNode;

    /// Constructor.
    TetGenSoftBodyLoader(
      Inst< SFNode            > _metadata       = 0,
      Inst< SFString          > _output         = 0,
      Inst< SFIndexedTetraSet > _geometry       = 0,
      Inst< MFX3DComposedGeometryNode > _surfaceGeometry = 0,
      Inst< MFX3DNBodyCollidableNode > _collisionGeometry = 0,
      Inst< SFBool            > _success        = 0,
      Inst< MFString          >  _url           = 0 );

    /// Initialize the node
    ///
    /// Override to load the SoftBody geometry from file
    virtual void initialize();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns true if the specified line is a valid line of input
    /// and not a comment
    bool validLine ( const string& line );

    /// Return the next logical input line from the specified input stream
    /// skipping invalid lines and comments
    string getTetGenLine ( istream& input );

    /// Loads the *.node, *.ele and *.face Tetgen files with the specified base filename
    /// \return true iff successful
    bool load ( const vector< string > &urls );

    /// Loads the coordinates/nodes from the *.node file
    /// \return true iff successful
    bool loadNodes ( istream& input );

    /// Loads the tetrahedra from the *.ele file
    /// \return true iff successful
    bool loadTetra ( istream& input );

    /// Loads the triangles from the *.face file
    /// \return true iff successful
    bool loadTriangles ( istream& input );

    /// Creates the suitable type of X3DComposedGeometry node for the concrete of the loader. 
    virtual X3DGeometryNode* createGeometry();

    /// Creates the suitable type of X3DGeometry nodes for the concrete of the loader. 
    virtual H3DSoftBodyNode::X3DGeometryNodeList* createSurfaceGeometry();

    /// Creates the suitable type of X3DNBodyCollidableNode's for the concrete of the loader. 
    virtual H3DSoftBodyNode::X3DNBodyCollidableNodeList* createCollisionGeometry();

    /// The list of surface geometries.
    H3DSoftBodyNode::X3DGeometryNodeList sgList;

    /// The list of collidable nodes.
    H3DSoftBodyNode::X3DNBodyCollidableNodeList cnList;

  };
}
#endif
