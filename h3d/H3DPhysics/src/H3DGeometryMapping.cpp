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
/// \file H3DGeometryMapping.cpp
/// \brief Source file for H3DGeometryMapping, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DGeometryMapping.h>
#include <H3D/MFVec3f.h>
#include <H3D/X3DChildNode.h>

using namespace H3D;  

H3DNodeDatabase H3DGeometryMapping::database( "H3DGeometryMapping", 
                                             NULL, 
                                             typeid( H3DGeometryMapping ),
                                             &X3DChildNode::database);

namespace H3DGeometryMappingInternals {
  FIELDDB_ELEMENT( H3DGeometryMapping, position, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DGeometryMapping, scale, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DGeometryMapping, orientation, INPUT_OUTPUT )
}

H3DGeometryMapping::H3DGeometryMapping (Inst< SFNode > _metadata,
                                        Inst< SFVec3f > _position,
                                        Inst< SFVec3f > _scale,
                                        Inst< SFRotation > _orientation ) :
X3DNode ( _metadata ),
position ( _position ),
scale ( _scale ),
orientation ( _orientation )
{
  // init fields
  type_name = "H3DGeometryMapping";
  database.initFields( this );

  // default field values
  scale->setValue ( Vec3f ( 1, 1, 1 ) );
}

void H3DGeometryMapping::linkGeometry ( const CoordList& coords, const IndexList& indices,
                                       const CoordList& dependentCoords,
                                       const Matrix4f& transform )
{
  // Transform the dependent coordinates by the offset and differ to subclass
  linkGeometryInternal( coords, indices, transformCoords ( dependentCoords, transform ) );
}

H3DGeometryMapping::CoordList H3DGeometryMapping::transformCoords ( const CoordList& coords,
                                                                   const Matrix4f& transform )
{
  const Vec3f& pos= position->getValue();
  Matrix4f ori= Matrix4f(orientation->getValue());
  const Vec3f& s= scale->getValue();
  Matrix4f sca= Matrix4f (  s.x, 0, 0, 0,
    0, s.y, 0, 0,
    0, 0, s.z, 0,
    0, 0, 0, 1 );

  CoordList transformedCoords;
  transformedCoords.reserve ( coords.size() );
  for ( MFVec3f::const_iterator i= coords.begin(); i != coords.end(); ++i ) {
    transformedCoords.push_back ( transform*(ori*sca*(*i) + pos) );
  }

  return transformedCoords;
}
