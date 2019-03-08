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
/// \file H3DAdjacencySoftBodyDeformer.h
/// \brief Header file for H3DAdjacencySoftBodyDeformer, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DADJACENCYSOFTBODYDEFORMER__
#define __H3DADJACENCYSOFTBODYDEFORMER__

#include <H3D/H3DFunctionNode.h>
#include <H3D/H3DPhysics/H3DDeviceSoftBodyModifierNode.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>
#include <HAPI/HAPIFunctionObject.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// \brief An abstract base class for soft body deformers that apply forces
  /// to vertices based on their distance from the deformation center measured
  /// in terms of number of edges.
  class H3DPHYS_API H3DAdjacencySoftBodyDeformer : public H3DDeviceSoftBodyModifierNode {
    public:

    /// Constructor.
    H3DAdjacencySoftBodyDeformer (
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFGeometryNode > _hapticGeometry = 0,
      Inst< SFInt32 > _deviceIndex = 0,
      Inst< SFInt32 > _maxAdjacencyDistance = 0 );

    /// Traverse the scene graph
    virtual void traverseSG ( TraverseInfo& ti );

    /// The maximum number of edges between the closest vertex to the deformer position and
    /// a vertex that can have a force applied to it as a result of the deformer.
    ///
    /// A value of -1 means that there is no such maximum, and potentially all vertices may
    /// have a force applied to them (depending on the distanceToForce function).
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> -1 \n
    auto_ptr < SFInt32 > maxAdjacencyDistance;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    typedef vector<size_t> IndexList;
    typedef vector<bool> BoolList;
    typedef set<size_t> IndexSet;
    typedef vector<IndexSet> AdjacencyList;

    /// Build/re-build adjacency information for soft body geometry if and only if required.
    void adjacencyUpToDate ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams );

    /// Build adjacency information for the current soft body geometry
    void buildAdjacency ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams );

    /// Returns the index of the closest vertex to the specified position
    size_t closestVertex ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams, Vec3f position );

    /// Returns a list of adjacent vertices within a given edge distance of a vertex.
    /// This should be called from the physics thread only.
    ///
    IndexList getAdjacentVertices ( size_t vertexIndex, size_t distance, PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams );

    /// An internal helper function used to implement getAdjacentVertices()
    IndexList getAdjacentVerticesInternal ( size_t vertexIndex, size_t distance, BoolList& visited, size_t currentDepth= 0 );

    /// Returns a list of adjacent vertices within a given edge distance of the closest vertex to the
    /// specified position.
    ///
    /// This function is provided as a helper for subclasses.
    /// This should be called from the physics thread only.
    ///
    IndexList getAdjacentVerticesFromPosition ( Vec3f position, size_t distance, PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams );

    /// The geometry node which we currently have built adjacency information for
    ///
    /// NULL if no adjacency information is built. This ptr is never accessed, only 
    /// used for comparisons to determine if adjacency info needs to be re-built.
    ///
    X3DGeometryNode* adjacencyGeometry;

    /// Information about which vertices are connected in the soft body geometry
    ///
    /// The list is indexed by the vertices index in the coord array. For each vertex, there is a 
    /// list of connected vertices.
    ///
    AdjacencyList adjacencyList;

    /// A copy of the field value maxAdjacencyDistance for the physics thread
    H3DInt32 max_adjacency_distance;
  };
}

#endif
