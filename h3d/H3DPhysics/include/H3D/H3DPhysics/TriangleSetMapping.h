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
/// \file TriangleSetMapping.h
/// \brief Header file for TriangleSetMapping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3D_PHYSICS_TRIANGLESETMAPPING__
#define __H3D_PHYSICS_TRIANGLESETMAPPING__

#include <H3D/H3DPhysics/H3DGeometryMapping.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/SFBool.h>
#include <H3D/SFInt32.h>
#include <H3D/SFFloat.h>
#include <H3D/SFVec2f.h>
#include <H3D/X3D.h>

#undef max
#undef min
#include <limits>

namespace H3D{  

  /// \ingroup SoftBody
  /// Implements a mapping from a IndexedTriangleSet mesh to an X3DComposedGeometryNode.
  ///
  /// Refer to H3DGeometryMapping for further details.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/TriangleSetMapping.x3d">TriangleSetMapping.x3d</a>
  ///     ( <a href="examples/TriangleSetMapping.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile TriangleSetMapping.dot
  class H3DPHYS_API TriangleSetMapping : public H3DGeometryMapping {
  public:

    /// Constructor.
    TriangleSetMapping(
      Inst< SFNode > _metadata = 0,
      Inst< SFVec3f > _position = 0,
      Inst< SFVec3f > _scale = 0,
      Inst< SFRotation > _orientation = 0,
      Inst< SFInt32 > _maxLinkedTriangles = 0,
      Inst< SFFloat > _triangleBlendWidth = 0,
      Inst< SFVec2f > _radiusOfInfluence = 0,
      Inst< SFBool > _enableDebugging = 0, 
      Inst< MFNode > _debugNodes = 0 );

    /// Initialize the node
    virtual void initialize();

    /// Move the dependentGeometry to match the current configuration of geometry using the mapping
    ///
    /// Subclasses should implement this function to update the dependant geometry based on the independant geometry.
    ///
    /// \param[in] coords A list of coordinates of the independant geometry
    /// \param[in] indices A list of indices of the independant geometry. How these are interpreted 
    ///                    (e.g. triangles or tetra) is up to the subclass.
    /// \param[out] dependentCoords The coordinates of the dependant geometry updated using the mapping.
    virtual void updateDependentGeometry ( const CoordList& coords, const IndexList& indices,
      CoordList& dependentCoords );

    /// Move only a local part of the dependentGeometry determined by dependentIndices to match the
    /// current configuration of geometry using the mapping
    ///
    /// Subclasses should implement this function to update the dependant geometry based on the independant geometry.
    ///
    /// \param[in] coords A list of coordinates of the independant geometry
    /// \param[in] indices A list of indices of the independant geometry. How these are interpreted 
    ///                    (e.g. triangles or tetra) is up to the subclass.
    /// \param[out] dependentCoords The coordinates of the dependant geometry updated using the mapping.
    /// \param[in]  dependentIndices A list of indices of the dependent geometry to used update the
    ///             dependent geometry. Only the coordinates included in the dependentIndices will be
    ///             updated.
    virtual void updateDependentGeometryLocal ( const CoordList& coords, const IndexList& indices,
      CoordList& dependentCoords, const IndexList& dependentIndices );

    /// Information about a link to a single triangle 
    struct TriangleInfo {
      H3DInt32 index;
      H3DFloat weight;
      H3DFloat a, b, c, d;
    };

    /// Information about a single node/vertex in the surface geometry generated during linking
    /// of surface and volume geometries, used to update the surface geometry based on the volume geometry
    struct VertexInfo {
      std::vector < TriangleInfo > triangleInfo;

      /// The index of the coordinate this vertex info refers to
      H3DInt32 coordIndex;

      /// The overall influence on this coordinate
      H3DFloat influence;
    };

    typedef vector < VertexInfo > VertexInfoList;

    /// Get information about each vertex and how it is linked to the volume
    ///
    /// See VertexInfo for more information
    ///
    const VertexInfoList& getVertexInfoList () {
      return verticesInfo;
    }

    /// The maximum number of triangles which can influence the movement of a point
    ///
    /// <b>Access type: </b> initializeOnly
    /// <b>Default value: </b> 5
    auto_ptr < SFInt32 > maxLinkedTriangles;
    
    /// The distance over which to blend the influence of multiple triangles
    ///
    /// This should probably be smaller than the average triangle width
    ///
    /// <b>Access type: </b> initializeOnly
    /// <b>Default value: </b> 0.0025
    auto_ptr < SFFloat > triangleBlendWidth;

    /// The overall radius of influence for a point
    ///
    /// The influence will fall-off linearly from 1 to 0 between 
    /// radiusOfInfluence.x and radiusOfInfluence.y
    ///
    /// <b>Access type: </b> initializeOnly
    /// <b>Default value: </b> Vec2f ( 0.01, 0.1 )
    auto_ptr < SFVec2f > radiusOfInfluence;

    /// If true then visual debugging nodes will be added to the field debugNodes
    ///
    /// <b>Access type: </b> initializeOnly
    /// <b>Default value: </b> false
    auto_ptr < SFBool > enableDebugging;

    /// Contains nodes to visualize/debug the mapping if enableDebugging is true
    ///
    /// <b>Access type: </b> outputOnly
    /// <b>Default value: </b> []
    auto_ptr < MFNode > debugNodes;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Using the current configuration of both geometries, link the two geometries to each other.
    /// 
    /// Subclasses should implement this function to perform any pre-processing on the two
    /// geometries required the implement the mapping.
    ///
    /// \param[in] coords A list of coordinates of the independant geometry
    /// \param[in] indices A list of indices of the independant geometry. How these are interpreted 
    ///                    (e.g. triangles or tetra) is up to the subclass.
    /// \param[in] dependentCoords A list of the coordinates of the dependant geometry. 
    ///                          Note: The surface coordinates will already have been transformed to 
    ///                          account for the position, orientation and scale fields.
    virtual void linkGeometryInternal ( const CoordList& coords, const IndexList& indices,
      const CoordList& dependentCoords );

    /// Returns the surface normal of the triangle defined by points a, b, c
    static Vec3f triangleNormal ( Vec3f a, Vec3f b, Vec3f c );

    /// Comparison function for sorting triangles by weight
    static bool triangleWeightCompare( const TriangleInfo& firstElem, const TriangleInfo& secondElem );

    /// Visual debugging nodes
    X3D::DEFNodes debug_dn;

    /// List of information about each vertex/node in the surface geometry and how it is linked to the
    /// volume geometry. This information is generated in linkGeometry() and later used in updateSurfaceGeometry()
    VertexInfoList verticesInfo;

    /// Original, unmodified coordinates
    CoordList origCoords;

    class IndexedTriangle : public HAPI::Collision::Triangle {
    public:
      IndexedTriangle( 
        const HAPI::Vec3& _a, 
        const HAPI::Vec3& _b, 
        const HAPI::Vec3& _c, 
        size_t _index ) :
        HAPI::Collision::Triangle( _a, _b, _c ),
        index( _index ) {
      }

      /// The index of the triangle
      size_t index;
    };

  };
}
#endif
