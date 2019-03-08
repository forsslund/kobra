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
/// \file H3DAdjacencySoftBodyDeformer.cpp
/// \brief Source file for H3DAdjacencySoftBodyDeformer, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DAdjacencySoftBodyDeformer.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase H3DAdjacencySoftBodyDeformer::database( "H3DAdjacencySoftBodyDeformer", 
                                                NULL, 
                                                typeid( H3DAdjacencySoftBodyDeformer ),
                                                &H3DDeviceSoftBodyModifierNode::database);

namespace H3DAdjacencySoftBodyDeformerInternals {
  FIELDDB_ELEMENT( H3DAdjacencySoftBodyDeformer, maxAdjacencyDistance, INPUT_OUTPUT )
}

H3DAdjacencySoftBodyDeformer::H3DAdjacencySoftBodyDeformer (
                                      Inst< SFNode > _metadata,
                                      Inst< ValueUpdater > _valueUpdater,
                                      Inst< SFH3DBodyNode > _body1,
                                      Inst< MFEngineOptions > _engineOptions,
                                      Inst< SFGeometryNode > _hapticGeometry,
                                      Inst< SFInt32 > _deviceIndex,
                                      Inst< SFInt32 > _maxAdjacencyDistance )
  : H3DDeviceSoftBodyModifierNode ( _metadata, _valueUpdater, _body1, _engineOptions, _hapticGeometry, _deviceIndex ),
    maxAdjacencyDistance ( _maxAdjacencyDistance ),
    adjacencyGeometry ( NULL ),
    max_adjacency_distance ( -1 )
{
  // init fields
  type_name = "H3DAdjacencySoftBodyDeformer";
  database.initFields( this );

  maxAdjacencyDistance->setValue ( -1 );
}

void H3DAdjacencySoftBodyDeformer::traverseSG ( TraverseInfo& ti ) {
  H3DDeviceSoftBodyModifierNode::traverseSG ( ti );

  max_adjacency_distance= maxAdjacencyDistance->getValue();
}

void H3DAdjacencySoftBodyDeformer::adjacencyUpToDate ( H3DSoftBodyNodeParameters& softBodyParams ) {
  if ( max_adjacency_distance != -1 ) {
    X3DGeometryNode* geometryNode= softBodyParams.getGeometry();
    if ( !adjacencyGeometry || adjacencyGeometry != geometryNode ) {
      buildAdjacency ( softBodyParams );
      adjacencyGeometry= geometryNode;
    }
  }
}

void H3DAdjacencySoftBodyDeformer::buildAdjacency ( H3DSoftBodyNodeParameters& softBodyParams ) {
  adjacencyList.clear ();
  adjacencyList.resize ( softBodyParams.getCoords().size() );

  const H3DSoftBodyNodeParameters::IndexList& tetra= softBodyParams.getIndices();

  for ( size_t i= 0; i < tetra.size(); i+= 4 ) {
    // In a tetra, every point is connected to every other...
    adjacencyList[tetra[i]].insert ( tetra[i+1] );
    adjacencyList[tetra[i]].insert ( tetra[i+2] );
    adjacencyList[tetra[i]].insert ( tetra[i+3] );

    adjacencyList[tetra[i+1]].insert ( tetra[i] );
    adjacencyList[tetra[i+1]].insert ( tetra[i+2] );
    adjacencyList[tetra[i+1]].insert ( tetra[i+3] );

    adjacencyList[tetra[i+2]].insert ( tetra[i+1] );
    adjacencyList[tetra[i+2]].insert ( tetra[i] );
    adjacencyList[tetra[i+2]].insert ( tetra[i+3] );

    adjacencyList[tetra[i+3]].insert ( tetra[i+1] );
    adjacencyList[tetra[i+3]].insert ( tetra[i+2] );
    adjacencyList[tetra[i+3]].insert ( tetra[i] );
  }
}

size_t H3DAdjacencySoftBodyDeformer::closestVertex ( H3DSoftBodyNodeParameters& softBodyParams, Vec3f position ) {
  // Find the vertex closest to the contact point
  H3DDouble closestDistance= 0.0f;
  size_t closestIndex= -1;
  const H3DSoftBodyNodeParameters::CoordList& coords= softBodyParams.getCoords();
  for ( size_t i= 0; i < coords.size(); ++i ) {
    Vec3f p= coords[i];
    H3DDouble distance= (position-p).length();
    if ( closestIndex == -1 || distance < closestDistance ) {
      closestDistance= distance;
      closestIndex= i;
    }
  }
  return closestIndex;
}

H3DAdjacencySoftBodyDeformer::IndexList H3DAdjacencySoftBodyDeformer::getAdjacentVertices ( size_t vertexIndex, size_t distance, H3DSoftBodyNodeParameters& softBodyParams ) {
  adjacencyUpToDate ( softBodyParams );
  BoolList visited ( adjacencyList.size(), false );
  return getAdjacentVerticesInternal ( vertexIndex, distance, visited );
}

H3DAdjacencySoftBodyDeformer::IndexList H3DAdjacencySoftBodyDeformer::getAdjacentVerticesInternal ( size_t vertexIndex, size_t distance, BoolList& visited, size_t currentDepth ) {
  IndexList v;
  if ( currentDepth < distance && !visited[vertexIndex] ) {
    IndexSet& a= adjacencyList[vertexIndex];
    v.insert ( v.end(), a.begin(), a.end() );
    visited[vertexIndex]= true;

    for ( IndexSet::iterator i= a.begin(); i != a.end(); ++i ) {
      IndexList l= getAdjacentVerticesInternal ( *i, distance, visited, currentDepth+1 );
      v.insert ( v.end(), l.begin(), l.end() );
    }
  }

  return v;
}

H3DAdjacencySoftBodyDeformer::IndexList H3DAdjacencySoftBodyDeformer::getAdjacentVerticesFromPosition ( Vec3f position, size_t distance, H3DSoftBodyNodeParameters& softBodyParams ) {
  return getAdjacentVertices ( closestVertex ( softBodyParams, position ), distance, softBodyParams );
}