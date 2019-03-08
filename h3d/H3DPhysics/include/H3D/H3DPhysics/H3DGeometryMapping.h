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
/// \file H3DGeometryMapping.h
/// \brief Header file for H3DGeometryMapping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DGEOMETRYMAPPING__
#define __H3DGEOMETRYMAPPING__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/X3DNode.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFRotation.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// \ingroup AbsractNodes
  /// An H3DGeometryMapping node provides a mapping from a soft body's geometry used
  /// for physics simulation to the soft body's surfaceGeometry used for rendering or
  /// collisionGeometry used for collision. When vertices of the geometry are moved,
  /// the appropriate vertices/coords of the surfaceGeometry or collisionGeometry
  /// will be moved to mimic the manipulation of the physics geometry.
  ///
  /// The implementation of this mapping does not assume that both meshes have the same
  /// number of  vertices, or that any of the vertices have the same location. Therefore
  /// an H3DGeometryMapping node can be used to link a high-resolution surface mesh to a
  /// low-resolution soft body simulation mesh, or vice-versa.
  ///
  /// \par Internal routes:
  /// \dotfile H3DGeometryMapping.dot
  class H3DPHYS_API H3DGeometryMapping : public X3DNode {
  public:
    typedef vector<Vec3f> CoordList;
    typedef vector<H3DInt32> IndexList;

    /// Constructor.
    H3DGeometryMapping(
      Inst< SFNode > _metadata = 0,
      Inst< SFVec3f > _position = 0,
      Inst< SFVec3f > _scale = 0,
      Inst< SFRotation > _orientation = 0 );

    /// Using the current configuration of both geometries, link the two geometries to each other.
    /// 
    /// The surface coordinates are transformed and passed on to linkGeometryInternal(), subclasses
    /// should implement linkGeometryInternal() to perform any pre-processing required to implement 
    /// the mapping.
    ///
    /// \param[in] coords A list of coordinates of the independant geometry
    /// \param[in] indices A list of indices of the independant geometry. How these are interpreted 
    ///                    (e.g. triangles or tetra) is up to the subclass.
    /// \param[in] dependentCoords A list of the coordinates of the dependant geometry
    /// \param[in] transform The initial transformation of the soft body
    virtual void linkGeometry ( const CoordList& coords, const IndexList& indices,
      const CoordList& dependentCoords,
      const Matrix4f& transform );

    /// Move the dependentGeometry to match the current configuration
    /// of geometry using the mapping
    ///
    /// Subclasses should implement this function to update the dependant geometry based on the
    /// independant geometry.
    ///
    /// \param[in] coords A list of coordinates of the independant geometry
    /// \param[in] indices A list of indices of the independant geometry. How these are interpreted 
    ///                    (e.g. triangles or tetra) is up to the subclass.
    /// \param[out] dependentCoords The coordinates of the dependant geometry updated using the mapping.
    virtual void updateDependentGeometry ( const CoordList& coords, const IndexList& indices,
      CoordList& dependentCoords )
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    {}
#endif

    /// Move only a local part of the dependentGeometry determined by dependentIndices to match the
    /// current configuration of geometry using the mapping
    ///
    /// Subclasses should implement this function to update the dependant geometry based on the
    /// independant geometry.
    ///
    /// \param[in] coords A list of coordinates of the independant geometry
    /// \param[in] indices A list of indices of the independant geometry. How these are interpreted 
    ///                    (e.g. triangles or tetra) is up to the subclass.
    /// \param[out] dependentCoords The coordinates of the dependant geometry updated using the mapping.
    /// \param[in]  dependentIndices A list of indices of the dependent geometry to used update the
    ///             dependent geometry. Only the coordinates included in the dependentIndices will be
    ///             updated.
    virtual void updateDependentGeometryLocal ( const CoordList& coords, const IndexList& indices,
      CoordList& dependentCoords, const IndexList& dependentIndices )
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    {}
#endif

    /// The position offset of the dependent geometry relative to the geometry
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> Vec3f() \n
    /// 
    /// \dotfile H3DGeometryMapping_position.dot
    auto_ptr < SFVec3f > position;

    /// The scale of the dependent geometry relative to the geometry
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> Vec3f(1, 1, 1) \n
    /// 
    /// \dotfile H3DGeometryMapping_scale.dot
    auto_ptr < SFVec3f > scale;

    /// The orientation offset of the dependent geometry relative to the geometry
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> Rotation() \n
    /// 
    /// \dotfile H3DGeometryMapping_orientation.dot
    auto_ptr < SFRotation > orientation;

    /// Returns the default XML containerField attribute value.
    ///
    /// For this node it is "surfaceMapping".
    virtual string defaultXMLContainerField() {
      return "surfaceMapping";
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Helper function to transform coordinates of dependent geometry to global space
    ///
    /// \param[in] coords The coordinates in surface geometry space
    /// \param[in] transform The initial transformation of the soft body
    /// \return The dependent coordinates transformed to global space
    CoordList transformCoords ( const CoordList& coords, const Matrix4f& transform );

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
      const CoordList& dependentCoords ) {};
  };
}
#endif
