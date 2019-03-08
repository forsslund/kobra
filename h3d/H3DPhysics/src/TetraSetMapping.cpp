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
/// \file TetraSetMapping.cpp
/// \brief Source file for TetraSetMapping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/TetraSetMapping.h>
#include <H3D/Coordinate.h>
#include <H3DUtil/H3DBasicTypes.h>
#include <HAPI/CollisionObjects.h>

#undef max
#undef min
#include <limits>

// Output additional statistics and profiling info about geometry linking process
//#define DEBUG_LINKING

using namespace H3D;

H3DNodeDatabase TetraSetMapping::database( "TetraSetMapping", 
                                          &newInstance<TetraSetMapping>, 
                                          typeid( TetraSetMapping ),
                                          &H3DGeometryMapping::database);

namespace TetraSetMappingInternals {
  FIELDDB_ELEMENT( TetraSetMapping, linkOutsidePoints, INPUT_OUTPUT )
}

// Notes of barycentric coordinates in tetrahedrons:
// http://people.sc.fsu.edu/~jburkardt/presentations/cg_lab_barycentric_tetrahedrons.pdf

TetraSetMapping::TetraSetMapping (
                                  Inst< SFNode > _metadata,
                                  Inst< SFVec3f > _position,
                                  Inst< SFVec3f > _scale,
                                  Inst< SFRotation > _orientation,
                                  Inst< SFBool > _linkOutsidePoints ) :
  H3DGeometryMapping ( _metadata, _position, _scale, _orientation ),
  linkOutsidePoints ( _linkOutsidePoints )
{
  // init fields
  type_name = "TetraSetMapping";
  database.initFields( this );

  linkOutsidePoints->setValue ( true );
}

void TetraSetMapping::linkGeometryInternal ( const CoordList& coords, const IndexList& indices,
                                            const CoordList& dependentCoords ) {

  // Build new vertex info list
  verticesInfo.clear();

  // Save starting coordinates
  origCoords= dependentCoords;

  if ( coords.size() > 0 && indices.size() > 0 && dependentCoords.size() > 0 ) {

  // For each point in coords:
  // * Work out which tetra the point should be linked to
  // * Store the tetra and the point's barycentric coordinates relative to that tetra

#ifdef DEBUG_LINKING
  Console(4) << "Debug: " << "Linking geometries..." << endl;
  H3DTime startTime= TimeStamp();
  int closestCount= 0;
#endif

  // Build bound tree containing tetras for fast intersection tests
  vector < HAPI::Collision::GeometryPrimitive* > primitives;

  // For each tetrahedron in the volume mesh
  for ( size_t i= 0; i < indices.size(); i+= 4 ) {

    // Build tetra bound from vertices
    vector<HAPI::Vec3> tetraPoints;
    tetraPoints.push_back ( coords[indices[i]] );
    tetraPoints.push_back ( coords[indices[i+1]] );
    tetraPoints.push_back ( coords[indices[i+2]] );
    tetraPoints.push_back ( coords[indices[i+3]] );

    HAPI::Collision::AABoxBound boundBox;
    boundBox.fitAroundPoints ( tetraPoints );

    primitives.push_back ( new VolumePrimitive ( boundBox, i ) );
  }

#ifdef DEBUG_LINKING
  Console(4) << "Debug: Creating tree with " << primitives.size() << " tetra." << endl;
#endif

  // Build the tree from the primitives
  VolumeTree tetraBounds ( primitives );

#ifdef DEBUG_LINKING
  // Sanity check
  primitives.clear();
  tetraBounds.getAllPrimitives ( primitives );
  Console(4) << "Debug: tetraBounds contains : " << primitives.size() << " tetra." << endl;

  H3DTime treeTime= TimeStamp()-startTime;
  Console(4) << "Debug: " << "Built tree in " << treeTime << endl;
  Console(4) << "Debug: " << "Searching..." << endl;
  startTime= TimeStamp();
#endif

  // For each point in the surface mesh
  size_t coordIndex= 0;
  for ( CoordList::const_iterator i= dependentCoords.begin(); i != dependentCoords.end(); ++i, ++coordIndex ) {
    Vec3f p= *i;

    // Look up tetra who are inside an AABB that contains the query point
    vector < HAPI::Collision::GeometryPrimitive* > _primitives;
    tetraBounds.insidePrimitives ( p, _primitives );
    if ( _primitives.size() > 0 ) {
      H3DFloat minDistance= std::numeric_limits<H3DFloat>::max();
      VertexInfo minVertexInfo;

      // Is the point p inside a tetra?
      bool insideTetra= false;

      for ( vector < HAPI::Collision::GeometryPrimitive* >::iterator j= _primitives.begin();
            j != _primitives.end(); ++j ) {
          VolumePrimitive* primitive= static_cast<VolumePrimitive*>(*j);
          Vec3f v1= coords[indices[primitive->index]];
          Vec3f v2= coords[indices[primitive->index+1]];
          Vec3f v3= coords[indices[primitive->index+2]];
          Vec3f v4= coords[indices[primitive->index+3]];

        VertexInfo vi ( (H3DInt32)primitive->index, (H3DInt32)coordIndex );
        barycentricCoordinates ( p, 
                                 v1, v2, v3, v4, 
                                 vi.a, vi.b, vi.c, vi.d );

        // If point inside tetra, choose that tetra
        const H3DFloat& eps= H3DUtil::Constants::f_epsilon;
        if ( vi.a >= -eps && vi.a <= 1 + eps &&
             vi.b >= -eps && vi.b <= 1 + eps &&
             vi.c >= -eps && vi.c <= 1 + eps &&
             vi.d >= -eps && vi.d <= 1 + eps ) {
          verticesInfo.push_back ( vi );
          insideTetra= true;
          break;
        }

        // Track closest tetra and if not inside any tetra use closest
        H3DFloat d= 0.0f;
        if (vi.a + vi.b + vi.c > 1.0f) d = vi.a + vi.b + vi.c - 1.0f;
        if (vi.a < 0.0f) d = (-vi.a < d) ? d : -vi.a;
        if (vi.b < 0.0f) d = (-vi.b < d) ? d : -vi.b;
        if (vi.c < 0.0f) d = (-vi.c < d) ? d : -vi.c;

        if ( d < minDistance ) {
          minDistance= d;
          minVertexInfo= vi;
        }
      }

      // If p is not inside a tetra then link to the
      // closest tetra instead
      if ( !insideTetra ) {
        verticesInfo.push_back ( minVertexInfo );
      }
    } else {
      // Point not in tetra
      // Look for closest tetra to link to (if that option is enabled)
      if ( linkOutsidePoints->getValue () ) {
        Vec3f closestPoint;
  #ifdef DEBUG_LINKING
        ++closestCount;
        Console(4) << "Debug: " << "Point outside: " << p << endl;
  #endif
        VolumePrimitive* primitive= tetraBounds.closestPrimitive ( p, closestPoint );
        Vec3f v1= coords[indices[primitive->index]];
        Vec3f v2= coords[indices[primitive->index+1]];
        Vec3f v3= coords[indices[primitive->index+2]];
        Vec3f v4= coords[indices[primitive->index+3]];

        VertexInfo vi ( (H3DInt32)primitive->index, (H3DInt32)coordIndex );
        barycentricCoordinates ( p, 
                                 v1, v2, v3, v4, 
                                 vi.a, vi.b, vi.c, vi.d );
        verticesInfo.push_back ( vi );
      }
    }
  }

#ifdef DEBUG_LINKING
  H3DTime totalTime= TimeStamp()-startTime;
  Console(4) << "Debug: " << "Link completed in " << totalTime << endl;
  Console(4) << "Debug: Had to search " << closestCount << " times for points outside tetra." << endl;
#endif

  }
}

void TetraSetMapping::updateDependentGeometry ( const CoordList& coords, const IndexList& indices,
                                               CoordList& dependentCoords )
{
  // Build list of new coordinates
  dependentCoords= origCoords;

  // For each vertex of the surface mesh:
  // * Re-calculate the vertex position using the linked tetra vertices and the stored
  //   barycentric coordinates calculated in linkGeometry()
  for ( size_t i= 0; i < verticesInfo.size(); ++i ) {
    const VertexInfo& vi= verticesInfo[i];

    // Vertices of tetra linked to this vertex
    Vec3f v1= coords[indices[vi.tetraIndex]];
    Vec3f v2= coords[indices[vi.tetraIndex+1]];
    Vec3f v3= coords[indices[vi.tetraIndex+2]];
    Vec3f v4= coords[indices[vi.tetraIndex+3]];

    // Calculate new vertex position based on barycentric coords and tetra vertices
    dependentCoords[vi.coordIndex]= v1*vi.a + v2*vi.b + v3*vi.c + v4*vi.d;
  }
}

void TetraSetMapping::updateDependentGeometryLocal ( const CoordList& coords, const IndexList& indices,
                                                    CoordList& dependentCoords, const IndexList& dependentIndices )
{
  // For each vertex of the surface mesh:
  // * Re-calculate the vertex position using the linked tetra vertices and the stored
  //   barycentric coordinates calculated in linkGeometry()
  for ( size_t i= 0; i < dependentIndices.size(); ++i ) {
    const VertexInfo& vi= verticesInfo[ dependentIndices[i] ];

    // Vertices of tetra linked to this vertex
    Vec3f v1= coords[indices[vi.tetraIndex]];
    Vec3f v2= coords[indices[vi.tetraIndex+1]];
    Vec3f v3= coords[indices[vi.tetraIndex+2]];
    Vec3f v4= coords[indices[vi.tetraIndex+3]];

    // Calculate new vertex position based on barycentric coords and tetra vertices
    dependentCoords.at( dependentIndices[i] ) = v1*vi.a + v2*vi.b + v3*vi.c + v4*vi.d ;
  }
}


Vec3f TetraSetMapping::triangleNormal ( Vec3f a, Vec3f b, Vec3f c ) {
  Vec3f n= (c-b).crossProduct ( b-a );
  n.normalizeSafe();
  return n;
}

H3DFloat TetraSetMapping::signedDistance ( Vec3f p, 
                                          Vec3f planeP, Vec3f planeN )
{
  return (p - planeP).dotProduct ( planeN );
}

void TetraSetMapping::barycentricCoordinates ( Vec3f p,
                                              Vec3f t1, Vec3f t2, Vec3f t3, Vec3f t4, 
                                              H3DFloat& b1, H3DFloat& b2, H3DFloat& b3, H3DFloat& b4 )
{
  Vec3f n= triangleNormal ( t2, t3, t4 );
  b1= signedDistance ( p, t2, n ) / 
      signedDistance ( t1, t2, n );

  n= triangleNormal ( t1, t3, t4 );
  b2= signedDistance ( p, t1, n ) /
      signedDistance ( t2, t1, n );

  n= triangleNormal ( t1, t2, t4 );
  b3= signedDistance ( p, t1, n ) /
      signedDistance ( t3, t1, n );

  n= triangleNormal ( t1, t2, t3 );
  b4= signedDistance ( p, t1, n ) /
      signedDistance ( t4, t1, n );
}
