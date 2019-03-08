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
/// \file HexaSetMapping.cpp
/// \brief Source file for HexaSetMapping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/HexaSetMapping.h>
#include <H3D/Coordinate.h>
#include <H3DUtil/H3DBasicTypes.h>
#include <HAPI/CollisionObjects.h>

#undef max
#undef min
#include <limits>

// Output additional statistics and profiling info about geometry linking process
//#define DEBUG_LINKING

using namespace H3D;

H3DNodeDatabase HexaSetMapping::database( "HexaSetMapping", 
                                         &newInstance<HexaSetMapping>, 
                                         typeid( HexaSetMapping ),
                                         &H3DGeometryMapping::database);

// Notes of barycentric coordinates in hexahedrons:
// http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch18.d/AFEM.Ch18.pdf

HexaSetMapping::HexaSetMapping (Inst< SFNode > _metadata,
                                Inst< SFVec3f > _position,
                                Inst< SFVec3f > _scale,
                                Inst< SFRotation > _orientation ) :
H3DGeometryMapping ( _metadata, _position, _scale, _orientation )
{
  // init fields
  type_name = "HexaSetMapping";
  database.initFields( this );
}

void HexaSetMapping::linkGeometryInternal ( const CoordList& coords, const IndexList& indices,
                                           const CoordList& dependentCoords )
{
  // For each point in coords:
  // * Work out which hexa the point should be linked to
  // * Store the hexa and the point's barycentric coordinates relative to that hexa

#ifdef DEBUG_LINKING
  Console(4) << "Debug: " << "Linking geometries..." << endl;
  H3DTime startTime= TimeStamp();
  int closestCount= 0;
#endif

  // Build bound tree containing hexas for fast intersection tests
  vector < HAPI::Collision::GeometryPrimitive* > primitives;

  // For each hexahedron in the volume mesh
  for ( size_t i= 0; i < indices.size(); i+= 8 ) {

    // Build hexa bound from vertices
    vector<HAPI::Vec3> hexaPoints;
    hexaPoints.push_back ( coords[indices[i]] );
    hexaPoints.push_back ( coords[indices[i+1]] );
    hexaPoints.push_back ( coords[indices[i+2]] );
    hexaPoints.push_back ( coords[indices[i+3]] );
    hexaPoints.push_back ( coords[indices[i+4]] );
    hexaPoints.push_back ( coords[indices[i+5]] );
    hexaPoints.push_back ( coords[indices[i+6]] );
    hexaPoints.push_back ( coords[indices[i+7]] );

    HAPI::Collision::AABoxBound boundBox;
    boundBox.fitAroundPoints ( hexaPoints );

    primitives.push_back ( new VolumePrimitive ( boundBox, i ) );
  }

#ifdef DEBUG_LINKING
  Console(4) << "Debug: Creating tree with " << primitives.size() << " hexa." << endl;
#endif

  // Build the tree from the primitives
  VolumeTree hexaBounds ( primitives );

#ifdef DEBUG_LINKING
  // Sanity check
  primitives.clear();
  hexaBounds.getAllPrimitives ( primitives );
  Console(4) << "Debug: hexaBounds contains : " << primitives.size() << " hexa." << endl;

  H3DTime treeTime= TimeStamp()-startTime;
  Console(4) << "Debug: " << "Built tree in " << treeTime << endl;
  Console(4) << "Debug: " << "Searching..." << endl;
  startTime= TimeStamp();
#endif

  // Build new vertex info list
  verticesInfo.clear();

  // For each point in the surface mesh
  for ( CoordList::const_iterator i= dependentCoords.begin(); i != dependentCoords.end(); ++i ) {
    Vec3f p= *i;

    // Look up hexa who are inside an AABB that contains the query point
    vector < HAPI::Collision::GeometryPrimitive* > _primitives;
    hexaBounds.insidePrimitives ( p, _primitives );
    if ( _primitives.size() > 0 ) {
      H3DFloat minDistance= std::numeric_limits<H3DFloat>::max();
      VertexInfo minVertexInfo;

      // Is the point p inside a hexa?
      bool insideHexa= false;

      for ( vector < HAPI::Collision::GeometryPrimitive* >::iterator j= _primitives.begin();
        j != _primitives.end(); ++j ) {
          VolumePrimitive* primitive= static_cast<VolumePrimitive*>(*j);
          Vec3f v1= coords[indices[primitive->index]];
          Vec3f v2= coords[indices[primitive->index+1]];
          Vec3f v3= coords[indices[primitive->index+2]];
          Vec3f v4= coords[indices[primitive->index+3]];
          Vec3f v5= coords[indices[primitive->index+4]];
          Vec3f v6= coords[indices[primitive->index+5]];
          Vec3f v7= coords[indices[primitive->index+6]];
          Vec3f v8= coords[indices[primitive->index+7]];

          VertexInfo vi ( (H3DInt32)primitive->index );
          barycentricCoordinates ( p, 
            v1, v2, v3, v4, v5, v6, v7, v8, 
            vi.a, vi.b, vi.c );

          // If point inside hexa, choose that hexa
          const H3DFloat& eps= H3DUtil::Constants::f_epsilon;
          if ( vi.a >= -1-eps && vi.a <= 1 + eps &&
            vi.b >= -1-eps && vi.b <= 1 + eps &&
            vi.c >= -1-eps && vi.c <= 1 + eps ) {
              verticesInfo.push_back ( vi );
              insideHexa= true;
              break;
          }

          // Track closest hexa and if not inside any hexa use closest
          H3DFloat d = vi.a*vi.a + vi.b*vi.b + vi.c*vi.c;

          if ( d < minDistance ) {
            minDistance= d;
            minVertexInfo= vi;
          }
      }

      // If p is not inside a hexa then link to the
      // closest hexa instead
      if ( !insideHexa ) {
        verticesInfo.push_back ( minVertexInfo );
      }
    } else {
      Vec3f closestPoint;
#ifdef DEBUG_LINKING
      ++closestCount;
#endif
      VolumePrimitive* primitive= hexaBounds.closestPrimitive ( p, closestPoint );
      Vec3f v1= coords[indices[primitive->index]];
      Vec3f v2= coords[indices[primitive->index+1]];
      Vec3f v3= coords[indices[primitive->index+2]];
      Vec3f v4= coords[indices[primitive->index+3]];
      Vec3f v5= coords[indices[primitive->index+4]];
      Vec3f v6= coords[indices[primitive->index+5]];
      Vec3f v7= coords[indices[primitive->index+6]];
      Vec3f v8= coords[indices[primitive->index+7]];

      VertexInfo vi ( (H3DInt32)primitive->index );
      barycentricCoordinates ( p, 
        v1, v2, v3, v4, v5, v6, v7, v8, 
        vi.a, vi.b, vi.c );
      verticesInfo.push_back ( vi );
    }
  }

#ifdef DEBUG_LINKING
  H3DTime totalTime= TimeStamp()-startTime;
  Console(4) << "Debug: " << "Link completed in " << totalTime << endl;
  Console(4) << "Debug: Had to search " << closestCount << " times for points outside hexa." << endl;
#endif
}

void HexaSetMapping::updateDependentGeometry ( const CoordList& coords, const IndexList& indices,
                                              CoordList& dependentCoords )
{
  // Build list of new coordinates
  dependentCoords.clear();

  // For each vertex of the surface mesh:
  // * Re-calculate the vertex position using the linked hexa vertices and the stored
  //   barycentric coordinates calculated in linkGeometry()
  for ( size_t i= 0; i < verticesInfo.size(); ++i ) {
    const VertexInfo& vi= verticesInfo[i];

    // Vertices of hexa linked to this vertex
    Vec3f v1= coords[indices[vi.hexaIndex]];
    Vec3f v2= coords[indices[vi.hexaIndex+1]];
    Vec3f v3= coords[indices[vi.hexaIndex+2]];
    Vec3f v4= coords[indices[vi.hexaIndex+3]];
    Vec3f v5= coords[indices[vi.hexaIndex+4]];
    Vec3f v6= coords[indices[vi.hexaIndex+5]];
    Vec3f v7= coords[indices[vi.hexaIndex+6]];
    Vec3f v8= coords[indices[vi.hexaIndex+7]];

    // NOTE: Right hand coord. system similar in openGL is followed.
    // The node v1 to v4 has higher z values than the nodes v5-v8.

    dependentCoords.push_back ( 0.125*( v1*(1-vi.a)*(1-vi.b)*(1+vi.c) +
      v2*(1+vi.a)*(1-vi.b)*(1+vi.c) +
      v3*(1+vi.a)*(1+vi.b)*(1+vi.c) + 
      v4*(1-vi.a)*(1+vi.b)*(1+vi.c) + 
      v5*(1-vi.a)*(1-vi.b)*(1-vi.c) + 
      v6*(1+vi.a)*(1-vi.b)*(1-vi.c) + 
      v7*(1+vi.a)*(1+vi.b)*(1-vi.c) + 
      v8*(1-vi.a)*(1+vi.b)*(1-vi.c)  ) );

  }
}

void HexaSetMapping::updateDependentGeometryLocal ( const CoordList& coords, const IndexList& indices,
                                                   CoordList& dependentCoords, const IndexList& dependentIndices )
{
  // For each vertex of the surface mesh:
  // * Re-calculate the vertex position using the linked hexa vertices and the stored
  //   barycentric coordinates calculated in linkGeometry()
  for ( size_t i= 0; i < dependentIndices.size(); ++i ) {
    const VertexInfo& vi= verticesInfo[ dependentIndices[i] ];

    // Vertices of hexa linked to this vertex
    Vec3f v1= coords[indices[vi.hexaIndex]];
    Vec3f v2= coords[indices[vi.hexaIndex+1]];
    Vec3f v3= coords[indices[vi.hexaIndex+2]];
    Vec3f v4= coords[indices[vi.hexaIndex+3]];
    Vec3f v5= coords[indices[vi.hexaIndex+4]];
    Vec3f v6= coords[indices[vi.hexaIndex+5]];
    Vec3f v7= coords[indices[vi.hexaIndex+6]];
    Vec3f v8= coords[indices[vi.hexaIndex+7]];

    // Calculate new vertex position based on barycentric coords and hexa vertices
    dependentCoords.at( dependentIndices[i] ) = 0.125*( v1*(1-vi.a)*(1-vi.b)*(1+vi.c) +
      v2*(1+vi.a)*(1-vi.b)*(1+vi.c) +
      v3*(1+vi.a)*(1+vi.b)*(1+vi.c) + 
      v4*(1-vi.a)*(1+vi.b)*(1+vi.c) + 
      v5*(1-vi.a)*(1-vi.b)*(1-vi.c) + 
      v6*(1+vi.a)*(1-vi.b)*(1-vi.c) + 
      v7*(1+vi.a)*(1+vi.b)*(1-vi.c) + 
      v8*(1-vi.a)*(1+vi.b)*(1-vi.c)  ) ;
  }
}

void HexaSetMapping::barycentricCoordinates ( Vec3f p,
                                             Vec3f t1, Vec3f t2, Vec3f t3, Vec3f t4, 
                                             Vec3f t5, Vec3f t6, Vec3f t7, Vec3f t8, 
                                             H3DFloat& b1, H3DFloat& b2, H3DFloat& b3 )
{

  Vec3f c1234 = findQuadCentroid( t1, t2, t3, t4 );
  Vec3f c2673 = findQuadCentroid( t2, t6, t7, t3 );
  Vec3f c6587 = findQuadCentroid( t6, t5, t8, t7 );
  Vec3f c5148 = findQuadCentroid( t5, t1, t4, t8 );
  Vec3f c5612 = findQuadCentroid( t5, t6, t1, t2 );
  Vec3f c4378 = findQuadCentroid( t4, t3, t7, t8 );

  Vec3f aAxis = c2673 - c5148;
  Vec3f bAxis = c4378 - c5612;
  Vec3f cAxis = c1234 - c6587;

  float aSize = 0.5f*aAxis.length();
  float bSize = 0.5f*bAxis.length();
  float cSize = 0.5f*cAxis.length();

  Vec3f origin = findIntersection( c5148, aAxis, c5612, bAxis );
  Vec3f pHexa = p-origin;

  aAxis.normalizeSafe();
  bAxis.normalizeSafe();
  cAxis.normalizeSafe();

  b1 = (pHexa.dotProduct( aAxis )) / aSize; 
  b2 = (pHexa.dotProduct( bAxis )) / bSize; 
  b3 = (pHexa.dotProduct( cAxis )) / cSize; 

}
Vec3f HexaSetMapping::findQuadCentroid( Vec3f t1, Vec3f t2, Vec3f t3, Vec3f t4 )
{
  // WARNINGUMUT: Check if it is always valid.
  return 0.25*(t1 + t2 + t3 + t4);
}
Vec3f HexaSetMapping::findIntersection( Vec3f pA, Vec3f rayA, Vec3f pB, Vec3f rayB )
{
  // This is a quick and dirty implementation. It might still be good enough.
  HAPI::Collision::LineSegment first_line( pA, pA + rayA );
  HAPI::HAPIFloat s, t;
  HAPI::Vec3 c0, c1;
  first_line.closestPointOnLine( pB, pB + rayB, s, t, c0, c1 );
  return Vec3f( c1 );
}

