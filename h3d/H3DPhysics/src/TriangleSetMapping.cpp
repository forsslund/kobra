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
/// \file TriangleSetMapping.cpp
/// \brief Source file for TriangleSetMapping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/TriangleSetMapping.h>
#include <H3D/Coordinate.h>
#include <H3DUtil/H3DBasicTypes.h>
#include <HAPI/CollisionObjects.h>

#undef max
#undef min
#include <limits>

// Output additional statistics and profiling info about geometry linking process
//#define DEBUG_LINKING

using namespace H3D;

#ifdef DEBUG_LINKING
namespace {
  static H3DTime linking_time = 0;
}
#endif

H3DNodeDatabase TriangleSetMapping::database( "TriangleSetMapping", 
                                          &newInstance<TriangleSetMapping>, 
                                          typeid( TriangleSetMapping ),
                                          &H3DGeometryMapping::database);

namespace TriangleSetMappingInternals {
  FIELDDB_ELEMENT( TriangleSetMapping, maxLinkedTriangles, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( TriangleSetMapping, triangleBlendWidth, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( TriangleSetMapping, radiusOfInfluence, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( TriangleSetMapping, enableDebugging, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( TriangleSetMapping, debugNodes, OUTPUT_ONLY )
}

TriangleSetMapping::TriangleSetMapping (
  Inst< SFNode > _metadata,
  Inst< SFVec3f > _position,
  Inst< SFVec3f > _scale,
  Inst< SFRotation > _orientation,
  Inst < SFInt32 > _maxLinkedTriangles,
  Inst < SFFloat > _triangleBlendWidth,
  Inst < SFVec2f > _radiusOfInfluence,
  Inst< SFBool > _enableDebugging,
  Inst< MFNode > _debugNodes ) :
  H3DGeometryMapping ( _metadata, _position, _scale, _orientation ),
  maxLinkedTriangles ( _maxLinkedTriangles ),
  triangleBlendWidth ( _triangleBlendWidth ),
  radiusOfInfluence ( _radiusOfInfluence ),
  enableDebugging( _enableDebugging ),
  debugNodes ( _debugNodes ) {
  // init fields
  type_name = "TriangleSetMapping";
  database.initFields( this );

  maxLinkedTriangles->setValue( 5 );
  triangleBlendWidth->setValue( H3DFloat ( 0.0025 ) );
  radiusOfInfluence->setValue( Vec2f ( H3DFloat ( 0.01 ), H3DFloat ( 0.1 ) ) );
  enableDebugging->setValue ( false );
}

void TriangleSetMapping::initialize() {
  H3DGeometryMapping::initialize();
  if( enableDebugging->getValue() ) {
    debugNodes->push_back(
      X3D::createX3DNodeFromString(
        "<Shape><Appearance><Material emissiveColor='1 1 0' /></Appearance><PointSet pointSize='4'><Coordinate DEF='C_unlinked_points' /></PointSet></Shape>",
        &debug_dn ).get(), id
    );
  }
}

void TriangleSetMapping::linkGeometryInternal ( const CoordList& coords, const IndexList& indices,
                                            const CoordList& dependentCoords ) {
  // Build new vertex info list
  verticesInfo.clear();

  // Save starting coordinates
  origCoords= dependentCoords;

  if( coords.size() > 0 && indices.size() > 0 && dependentCoords.size() > 0 ) {
    // For each point in dependentCoords:
    // * Work out which triangles the point should be linked to
    // * Store the triangles, weight and the point's barycentric coordinates relative to that triangle
    verticesInfo.reserve( dependentCoords.size() );

#ifdef DEBUG_LINKING
    H3DConsole( LogLevel::Info ) << "Linking geometries..." << std::endl;
    H3DTime startTime = TimeStamp();
#endif

    // Build a spacial structure to efficiently return triangles close to a query point
    std::vector < HAPI::Collision::GeometryPrimitive* > primitives;
    for( size_t j = 0; j + 2 < indices.size(); j += 3 ) {
      primitives.push_back( 
        new IndexedTriangle( 
          coords[indices[j]], 
          coords[indices[j + 1]], 
          coords[indices[j + 2]], j ) );
    }
    HAPI::Collision::BBPrimitiveTree primitiveTree ( 
      &HAPI::Collision::BBPrimitiveTree::newInstance<HAPI::Collision::AABoxBound>, primitives );

#ifdef DEBUG_LINKING
    TimeStamp buildTreeEnd;
    H3DTime buildTreeElapsed = buildTreeEnd - startTime;
    H3DConsole( LogLevel::Info ) << "Built tree in: " << buildTreeElapsed << " secs" << std::endl;
    startTime = TimeStamp();
    linking_time += buildTreeElapsed;
#endif

    // Parameters for linking
    unsigned int max_linked_triangles = maxLinkedTriangles->getValue();
    H3DFloat triangle_blend_width = triangleBlendWidth->getValue();
    const Vec2f& radius_of_influence = radiusOfInfluence->getValue();
    bool enable_debugging = enableDebugging->getValue();

    MFVec3f::vector_type unlinked_points;
    if( enable_debugging ) {
      unlinked_points.reserve( dependentCoords.size() );
    }

    // For each point in the surface mesh, find triangles to link to
    size_t coordIndex = 0;
    for( CoordList::const_iterator i = dependentCoords.begin(); i != dependentCoords.end(); ++i, ++coordIndex ) {
      Vec3f p = *i;
      bool point_linked = false;

      // Find the closest triangles
      std::vector < TriangleInfo > triangles;

      std::vector < HAPI::Collision::GeometryPrimitive* > primitivesWithinRadius;
      primitiveTree.getPrimitivesWithinRadius( p, radius_of_influence.y, primitivesWithinRadius );

      // For each nearby triangle, compute closest distance to triangle
      for( size_t j = 0; j < primitivesWithinRadius.size(); ++j ) {
        IndexedTriangle* t = static_cast < IndexedTriangle* > ( primitivesWithinRadius[j] );

        HAPI::Vec3 closest_point;
        HAPI::Vec3 closest_normal;
        HAPI::Vec3 closest_tc;
        t->closestPoint( HAPI::Vec3( p ), closest_point, closest_normal, closest_tc );

        H3DFloat d = (p - Vec3f( closest_point )).length();
        TriangleInfo ti;
        ti.weight = d;
        ti.index = static_cast<H3DInt32>(t->index);
        triangles.push_back( ti );
      }

      if( !triangles.empty() ) {
        // Sort triangles by distance and discard some if there are more than needed
        std::sort( triangles.begin(), triangles.end(), triangleWeightCompare );
        if( triangles.size() > max_linked_triangles ) {
          triangles.resize( max_linked_triangles );
        }

        // Closest distance to any triangle
        H3DFloat closest_d = triangles[0].weight;

        // Compute overall influence on this coordinate and skip further computation if it is zero
        VertexInfo vi;
        vi.influence = H3DMax( H3DFloat( 0 ), H3DMin( H3DFloat( 1 ) - (closest_d - radius_of_influence.x) / (radius_of_influence.y - radius_of_influence.x), H3DFloat( 1 ) ) );
        if( vi.influence > Constants::f_epsilon ) {
          // Convert distances to weights
          H3DFloat total_w = 1;
          for( size_t j = 1; j < triangles.size(); ++j ) {
            H3DFloat diff = triangles[j].weight - triangles[0].weight;
            triangles[j].weight = 1 - std::min( diff / triangle_blend_width, H3DFloat( 1 ) );
            total_w += triangles[j].weight;
          }
          triangles[0].weight = 1;

          for( size_t j = 0; j < triangles.size(); ++j ) {
            triangles[j].weight /= total_w;
          }

          // Compute barycentric coordinates for each triangle
          for( size_t j = 0; j < triangles.size(); ++j ) {
            TriangleInfo& ti = triangles[j];

            Vec3f a( coords[indices[ti.index]] );
            Vec3f b( coords[indices[ti.index + 1]] );
            Vec3f c( coords[indices[ti.index + 2]] );

            Vec3f n = triangleNormal( a, b, c );

            // Project point on to triangle plane, and get distance to plane
            ti.d = (p - coords[indices[ti.index]]).dotProduct( n );
            Vec3f p_on_plane = p - ti.d*n;

            // Compute triangle barycentric coordinates
            H3DFloat areaABC = n.dotProduct( (b - a).crossProduct( c - a ) );
            H3DFloat areaPBC = n.dotProduct( (b - p_on_plane).crossProduct( c - p_on_plane ) );
            H3DFloat areaPCA = n.dotProduct( (c - p_on_plane).crossProduct( a - p_on_plane ) );
            ti.a = areaPBC / areaABC;
            ti.b = areaPCA / areaABC;
            ti.c = 1.0f - ti.a - ti.b;

          }

          // Add the vertex info
          vi.coordIndex = static_cast<H3DInt32>(coordIndex);
          vi.triangleInfo = triangles;
          verticesInfo.push_back( vi );
          point_linked = true;
        }
      }

      if( enable_debugging && !point_linked ) {
        // Debug unlinked points
        unlinked_points.push_back( p );
      }
    }

    if( enable_debugging ) {
      Coordinate* debug_unlinked_points = NULL;
      debug_dn.getNode( "C_unlinked_points", debug_unlinked_points );
      debug_unlinked_points->point->setValue( unlinked_points );
    }

#ifdef DEBUG_LINKING
    H3DTime searchTreeElapsed = TimeStamp() - startTime;
    linking_time += searchTreeElapsed;
    H3DConsole( LogLevel::Info ) << "Searched tree in: " << searchTreeElapsed << " secs" << std::endl;
    H3DConsole( LogLevel::Info ) << "Total linking time: " << linking_time << " secs" << std::endl;
#endif
  }
}

void TriangleSetMapping::updateDependentGeometry ( const CoordList& coords, const IndexList& indices,
                                               CoordList& dependentCoords )
{
  // Build list of new coordinates
  dependentCoords= origCoords;

  // For each vertex of the surface mesh:
  // * Re-calculate the vertex position using the linked tetra vertices and the stored
  //   barycentric coordinates calculated in linkGeometry()
  for ( size_t i= 0; i < verticesInfo.size(); ++i ) {
    const VertexInfo& vi= verticesInfo[i];

    dependentCoords[vi.coordIndex] = Vec3f();
    for( size_t j = 0; j < vi.triangleInfo.size(); ++j ) {
      const TriangleInfo& ti = vi.triangleInfo[j];

      Vec3f n = triangleNormal(
        Vec3f( coords[indices[ti.index]] ),
        Vec3f( coords[indices[ti.index + 1]] ),
        Vec3f( coords[indices[ti.index + 2]] ) );
      
      dependentCoords[vi.coordIndex]+=
        ti.weight * (
          ti.a * coords[indices[ti.index]] +
          ti.b * coords[indices[ti.index + 1]] +
          ti.c * coords[indices[ti.index + 2]] +
          ti.d * n);
    }

    dependentCoords[vi.coordIndex] = 
      vi.influence*dependentCoords[vi.coordIndex] + (1 - vi.influence)*origCoords[vi.coordIndex];
  }
}

void TriangleSetMapping::updateDependentGeometryLocal ( const CoordList& coords, const IndexList& indices,
                                                    CoordList& dependentCoords, const IndexList& dependentIndices ) {
}

Vec3f TriangleSetMapping::triangleNormal ( Vec3f a, Vec3f b, Vec3f c ) {
  Vec3f n= (c-b).crossProduct ( b-a );
  n.normalizeSafe();
  return n;
}

bool TriangleSetMapping::triangleWeightCompare( const TriangleInfo& firstElem, const TriangleInfo& secondElem ) {
  return firstElem.weight < secondElem.weight;
}
