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
/// \file H3DSoftBodyLoader.h
/// \brief Header file for H3DSoftBodyLoader, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DTETRALOADER__
#define __H3DTETRALOADER__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>
#include <H3D/H3DPhysics/H3DSoftBodyNode.h>
#include <H3D/X3DGeometryNode.h>
#include <H3D/X3DChildNode.h>
#include <H3D/SFString.h>
#include <H3D/SFBool.h>
#include <H3D/X3DUrlObject.h>

namespace H3D{

  /// \ingroup AbstractNodes SoftBody
  /// Abstract base for nodes that load a SoftBody representation from an external
  /// file. The SoftBody may then be used in the scene graph or saved in X3D format.
  ///
  /// Both the internal, collision and surface geometry should be loaded,
  /// as an X3DGeometry node. The subclasses decide which type of
  /// X3DGeometry(e.g., IndexedTetraSet, IndexedTriangleSet )will
  /// be used depending on the type of file loaded. Unless implemented
  /// otherwise the surface, collision and internal geometry share the
  /// same Coordinate node.
  ///
  /// \par Internal routes:
  /// \dotfile H3DSoftBodyLoader.dot
  class H3DPHYS_API H3DSoftBodyLoader : public X3DChildNode,
                                        public X3DUrlObject {
  public:

    typedef TypedSFNode < X3DGeometryNode > SFX3DGeometryNode;
    typedef TypedMFNode < X3DGeometryNode > MFX3DGeometryNode;
    typedef TypedMFNode < X3DNBodyCollidableNode > MFX3DNBodyCollidableNode;

    /// Constructor.
    H3DSoftBodyLoader(
      Inst< SFNode            > _metadata       = 0,
      Inst< SFString          > _output         = 0,
      Inst< SFX3DGeometryNode > _geometry       = 0,
      Inst< MFX3DGeometryNode > _surfaceGeometry = 0,
      Inst< MFX3DNBodyCollidableNode > _collisionGeometry = 0,
      Inst< SFBool            > _success        = 0,
      Inst< MFString          >  _url           = 0 );

    /// Initialize the node
    ///
    /// Override to load the SoftBody geometry from file
    virtual void initialize();

    /// The filename to save the SoftBodyGeometry to in X3D format
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> "" \n
    /// 
    /// \dotfile H3DSoftBodyLoader_output.dot
    auto_ptr < SFString > output;

    /// The loaded internal geometry node. The concrete type
    /// is decided by the sub-classes of H3DSoftBodyLoader.
    ///
    /// <b>Access type:</b> outputOnly \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyLoader_geometry.dot
    auto_ptr < SFX3DGeometryNode > geometry;

    /// The loaded surface geometry node.The concrete type
    /// is decided by the sub-classes of H3DSoftBodyLoader.
    ///
    /// <b>Access type:</b> outputOnly \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyLoader_surfaceGeometry.dot
    auto_ptr < MFX3DGeometryNode > surfaceGeometry;

    /// The loaded collision geometry node. The concrete type
    /// is decided by the sub-classes of H3DSoftBodyLoader.
    ///
    /// May be a composed geometry like IndexedTriangleSet, IndexedFaceSet
    /// or a collection of geometries such as spehere, box etc.
    ///
    /// <b>Access type:</b> outputOnly \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyLoader_collisionGeometry.dot
    auto_ptr < MFX3DNBodyCollidableNode > collisionGeometry;

    /// True if the SoftBody geometry was loaded successfully
    ///
    /// <b>Access type:</b> outputOnly \n
    /// <b>Default value:</b> false \n
    /// 
    /// \dotfile H3DSoftBodyLoader_success.dot
    auto_ptr < SFBool > success;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Creates all the geometries.
    void createGeometries();

    /// Creates the suitable type of X3DGeometry node for the concrete of the loader. 
    virtual X3DGeometryNode* createGeometry()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// Creates the suitable type of X3DGeometry nodes for the concrete of the loader. 
    virtual H3DSoftBodyNode::X3DGeometryNodeList* createSurfaceGeometry()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// Creates the suitable type of X3DNBodyCollidableNode's for the concrete of the loader. 
    virtual H3DSoftBodyNode::X3DNBodyCollidableNodeList* createCollisionGeometry()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

  };
}
#endif
