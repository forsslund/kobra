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
/// \file HexaSetMapping.h
/// \brief Header file for HexaSetMapping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HEXASETMAPPING__
#define __HEXASETMAPPING__

#include <H3D/H3DPhysics/H3DGeometryMapping.h>
#include <H3D/H3DPhysics/IndexedHexaSet.h>
#include <H3D/H3DPhysics/FieldTemplates.h>

#undef max
#undef min
#include <limits>

namespace H3D{  

  /// \ingroup SoftBody
  /// Implements a mapping from a IndexedHexaSet volume mesh to an X3DComposedGeometryNode.
  ///
  /// Refer to H3DGeometryMapping for further details.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/HexaSetMapping.x3d">HexaSetMapping.x3d</a>
  ///     ( <a href="examples/HexaSetMapping.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile HexaSetMapping.dot
  class H3DPHYS_API HexaSetMapping : public H3DGeometryMapping {
  public:

    /// Constructor.
    HexaSetMapping (
      Inst< SFNode > _metadata = 0,
      Inst< SFVec3f > _position = 0,
      Inst< SFVec3f > _scale = 0,
      Inst< SFRotation > _orientation = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

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


  protected:

    /// Information about a single node/vertex in the surface geometry generated during linking
    /// of surface and volume geometries, used to update the surface geometry based on the volume geometry
    struct VertexInfo {

      /// Constructor
      VertexInfo ( H3DInt32 _hexaIndex= -1, 
        H3DFloat _a= 0.0f, H3DFloat _b= 0.0f, H3DFloat _c= 0.0f ) : 
      hexaIndex ( _hexaIndex ), a ( _a ), b ( _b ), c ( _c ) {}

      /// The index of the hexa that this node is linked to
      H3DInt32 hexaIndex;

      /// The barycentric coordinates of this vertex w.r.t. the hexa the node is linked to
      H3DFloat a, b, c ;
    };

    typedef vector < VertexInfo > VertexInfoList;

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

    /// Calculates the barycentric coordinates (b1, b2, b3) for point p in
    /// hexahedron (t1, t2, t3, t4, t5, t6, t7, t8)
    static void barycentricCoordinates (  Vec3f p,
      Vec3f t1, Vec3f t2, Vec3f t3, Vec3f t4, Vec3f t5, Vec3f t6, Vec3f t7, Vec3f t8, 
      H3DFloat& b1, H3DFloat& b2, H3DFloat& b3 );

    /// Calculates the centroid of the quad specified by t1, t2, t3, t4.
    static Vec3f findQuadCentroid( Vec3f t1, Vec3f t2, Vec3f t3, Vec3f t4 );

    /// Calculate the intersection of ttwo lines. 
    static Vec3f findIntersection( Vec3f pA, Vec3f rayA, Vec3f pB, Vec3f rayB );  

    /// List of information about each vertex/node in the surface geometry and how it is linked to the
    /// volume geometry. This information is generated in linkGeometry() and later used in updateSurfaceGeometry()
    VertexInfoList verticesInfo;

    /// Information about a single element of the mesh that is stored in the bound tree
    ///
    /// Note: The AABB of the mesh element is represented as a line segment running
    /// from the AABB min to the AABB max. This is because AABox is not handled by 
    /// BBPrimitiveTree as a GeometryPrimitive.
    class VolumePrimitive : public HAPI::Collision::LineSegment {
    public:
      /// Construct the primitive from the bounding box of the mesh element
      ///
      /// \param[in] _bound The bounding box of the mesh element
      /// \param[in] _index The index of the mesh element
      VolumePrimitive ( const HAPI::Collision::AABoxBound& _bound, size_t _index ) :
          LineSegment ( _bound.min, _bound.max ),
            index ( _index ) {
          }

          /// The index of the mesh element, stored in the tree
          size_t index;
    };

    class VolumeTree;

    /// A specialized AABoxBound which is used by VolumeTree to provide the required lookup 
    /// of mesh elements.
    ///
    /// Note: These functions are implemented in the bound rather than the tree because new bound
    /// instances of the correct type can be created using the new_func member during construction.
    class VolumeBound : public HAPI::Collision::AABoxBound {
    public:

      /// Find all the primitives/mesh elements that are in a leaf whose bounding box contains the
      /// specified query point.
      ///
      /// \param[in] _point The query point
      /// \param[in,out] _primitives The primitives found that match the query
      /// \param[in] _tree The VolumeTree owning this bound
      void insidePrimitives ( Vec3f _point, vector< HAPI::Collision::GeometryPrimitive * > &_primitives,
        HAPI::Collision::BBPrimitiveTree& _tree ) {
        // If the first input is a leaf, that is, there was no subdivision when building
        // the tree, then check it.
        if( _tree.isLeaf() ) {
          _primitives.insert ( _primitives.end(), _tree.primitives.begin(), _tree.primitives.end() );
        }

        if ( _tree.left.get() ) {
          if ( _tree.left->isLeaf() ) {
            _primitives.insert ( _primitives.end(), _tree.left->primitives.begin(), _tree.left->primitives.end() );
          } else {
            if ( _tree.left->insideBound ( _point ) ) {
              VolumeBound* left= static_cast<VolumeBound*>(_tree.left->bound.get());
              left->insidePrimitives ( _point, _primitives, *_tree.left );
            }
          }
        }
        if ( _tree.right.get() ) {
          if ( _tree.right->isLeaf() ) {
            _primitives.insert ( _primitives.end(), _tree.right->primitives.begin(), _tree.right->primitives.end() );
          } else {
            if ( _tree.right->insideBound ( _point ) ) {
              VolumeBound* right= static_cast<VolumeBound*>(_tree.right->bound.get());
              right->insidePrimitives (_point, _primitives, *_tree.right );
            }
          }
        }
      }

      /// Finds the closest primitive/mesh element to the query point specified.
      ///
      /// Note: Currently uses the LineSegment representation.
      /// \param[in] _point The query point
      /// \param[in] _tree The VolumeTree that owns this bound
      /// \param[out] _closestPoint The closest point on the closest primitive (that which is returned)
      /// \return The primitive closest to the query point
      VolumePrimitive*
        closestPrimitive ( Vec3f _point,
        HAPI::Collision::BBPrimitiveTree& _tree,
        Vec3f& _closestPoint ) {
          if ( _tree.right.get() && _tree.right->isLeaf() ) {
            HAPI::HAPIFloat min_d= std::numeric_limits<HAPI::HAPIFloat>::max();
            HAPI::Vec3 cp, n, tex;
            HAPI::Collision::GeometryPrimitive* closestPrimitive= NULL;

            for( H3DUtil::AutoRefVector< HAPI::Collision::GeometryPrimitive >::const_iterator
              i = _tree.right->primitives.begin();
              i != _tree.right->primitives.end(); ++i ) {
                (*i)->closestPoint ( _point, cp, n, tex );
                HAPI::HAPIFloat d= (cp-_point).length();
                if ( d < min_d ) {
                  min_d= d;
                  closestPrimitive= *i;
                  _closestPoint= Vec3f(cp);
                }
            }

            return static_cast<VolumePrimitive*>(closestPrimitive);
          }

          if ( _tree.left.get() && _tree.left->isLeaf() ) {
            HAPI::HAPIFloat min_d= std::numeric_limits<HAPI::HAPIFloat>::max();
            HAPI::Vec3 cp, n, tex;
            HAPI::Collision::GeometryPrimitive* closestPrimitive= NULL;

            for( H3DUtil::AutoRefVector< HAPI::Collision::GeometryPrimitive >::const_iterator
              i = _tree.left->primitives.begin();
              i != _tree.left->primitives.end(); ++i ) {
                (*i)->closestPoint ( _point, cp, n, tex );
                HAPI::HAPIFloat d= (cp-_point).length();
                if ( d < min_d ) {
                  min_d= d;
                  closestPrimitive= *i;
                  _closestPoint= Vec3f(cp);
                }
            }

            return static_cast<VolumePrimitive*>(closestPrimitive);
          }

          if ( _tree.left.get() && _tree.right.get() ) {
            VolumeBound* left= static_cast<VolumeBound*>(_tree.left->bound.get());
            VolumeBound* right= static_cast<VolumeBound*>(_tree.left->bound.get());
            Vec3f cpLeft, cpRight;
            VolumePrimitive* closestLeft= left->closestPrimitive ( _point, *_tree.left, cpLeft );
            VolumePrimitive* closestRight= right->closestPrimitive ( _point, *_tree.right, cpRight );
            HAPI::HAPIFloat dLeft= (_point-cpLeft).length();
            HAPI::HAPIFloat dRight= (_point-cpRight).length();
            if ( dLeft < dRight ) {
              _closestPoint= cpLeft;
              return closestLeft;
            } else {
              _closestPoint= cpRight;
              return closestRight;
            }
          } else {
            if ( _tree.left.get() ) {
              VolumeBound* left= static_cast<VolumeBound*>(_tree.left->bound.get());
              return left->closestPrimitive ( _point, *_tree.left, _closestPoint );
            } else {
              VolumeBound* right= static_cast<VolumeBound*>(_tree.left->bound.get());
              return right->closestPrimitive ( _point, *_tree.right, _closestPoint );
            }
          }
      }
    };

    /// A tree structure used to store information about mesh elements to facilitate
    /// fast point inside tetra test that are used during the geometry->surfaceGeometry linking
    /// pre-processing step.
    class VolumeTree : public HAPI::Collision::BBPrimitiveTree {
    public:
      /// Build a new tree to contain the specified VolumePrimitive primitives
      VolumeTree ( const vector< HAPI::Collision::GeometryPrimitive * > &_primitives,
        unsigned int max_nr_primitives_in_leaf = 1 ) :
      BBPrimitiveTree ( &newInstance<VolumeBound>,
        _primitives, max_nr_primitives_in_leaf ) {
      }

      /// Find all the primitives/mesh elements that are in a leaf whose bounding box contains the
      /// specified query point.
      ///
      /// \param[in] _point The query point
      /// \param[out] _primitives The primitives found that match the query
      void insidePrimitives ( Vec3f _point,
        vector< HAPI::Collision::GeometryPrimitive * > &_primitives ) {
          static_cast<VolumeBound*>(bound.get())->insidePrimitives ( _point, _primitives, *this );
      }

      /// Finds the closest primitive/mesh element to the query point specified.
      ///
      /// Note: Currently uses the LineSegment representation. Much slower than insidePrimitives()
      /// used as last resort.
      /// \param[in] _point The query point
      /// \param[out] _closestPoint The closest point on the closest primitive (that which is returned)
      /// \return The primitive closest to the query point
      VolumePrimitive*
        closestPrimitive ( Vec3f _point, Vec3f& _closestPoint ) {
          return static_cast<VolumeBound*>(bound.get())->closestPrimitive ( _point, *this, _closestPoint );
      }
    };
  };
}
#endif
