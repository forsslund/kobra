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
/// \file Rope.cpp
/// \brief Source file for Rope, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/Rope.h>
#include <H3D/Coordinate.h>

using namespace H3D;  

H3DNodeDatabase Rope::database( "Rope", 
                               &(newInstance< Rope >), 
                               typeid( Rope ),
                               &H3DSoftBodyNode::database);


Rope::Rope( Inst< SFNode > _metadata,
           Inst< SFMatrix4f > _transform,
           Inst< SFH3DPhysicsMaterialNode > _material,
           Inst< SFIndexedLineSet > _geometry,
           Inst< MFX3DGeometryNode > _surfaceGeometry,
           Inst< MFX3DNBodyCollidableNode > _collisionGeometry,
           Inst< MFH3DGeometryMapping > _surfaceMapping,
           Inst< MFH3DGeometryMapping > _collisionMapping,
           Inst< SFH3DDeformationStrategyNode > _deformationStrategy,
           Inst< MFH3DSoftBodyOutputNode > _output,      
           Inst< LinkGeometry > _linkSurfaceGeometry,
           Inst< LinkGeometry > _linkCollisionGeometry,      
           Inst< ValueUpdater > _valueUpdater,
           Inst< MFEngineOptions > _engineOptions ) :
H3DSoftBodyNode ( _metadata, _transform, _material, _geometry,
                 _surfaceGeometry, _collisionGeometry, _surfaceMapping, _collisionMapping,
                 _deformationStrategy, _output, _linkSurfaceGeometry, _linkCollisionGeometry, 
                 _valueUpdater, _engineOptions )
{
  // init fields
  type_name = "Rope";
  database.initFields( this );
}

PhysicsEngineParameters::RopeParameters* Rope::createSoftBodyParameters () {
  return new PhysicsEngineParameters::RopeParameters();
}

PhysicsEngineParameters::RopeParameters* Rope::getSoftBodyParameters( bool all_params ) {
  PhysicsEngineParameters::RopeParameters* params= 
    static_cast<PhysicsEngineParameters::RopeParameters*>(H3DSoftBodyNode::getSoftBodyParameters ( all_params ));

  // If geometry node has changed, update the mesh in the simulation
  if ( all_params || valueUpdater->hasCausedEvent ( geometryChanged ) ) {
    IndexedLineSet* geom= static_cast<IndexedLineSet*>(geometry->getValue());
    Coordinate* coord= dynamic_cast<Coordinate*> ( geom->coord->getValue() );
    if ( coord ) {
      params->setIndices ( geom->coordIndex->getValue() );
      params->setCoords ( transformCoords ( coord->point->getValue() ) );
    } else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The geometry field must use a Coordinate node." << endl;
    }
  }

  return params;
}

void Rope::setSoftBodyParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params ) {
  // update the geometry node coords and indices
  IndexedLineSet* geom= static_cast<IndexedLineSet*>(geometry->getValue());
  Coordinate* coord= dynamic_cast<Coordinate*>(geom->coord->getValue ());
  if ( coord ) {
    geom->coordIndex->unroute ( geometryChanged );
    geom->coord->unroute ( geometryChanged );
    coord->point->unroute ( geometryChanged );

    geom->coordIndex->setValue ( params.getIndices() );
    coord->point->setValue ( params.getCoords() );

    geom->coordIndex->routeNoEvent ( geometryChanged );
    geom->coord->routeNoEvent ( geometryChanged );
    coord->point->routeNoEvent ( geometryChanged );
  } else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must use a Coordinate node." << endl;
  }
}

void Rope::linkGeometries( const X3DGeometryNode &sourceGeometry, X3DGeometryNodeList &linkingGeometries,
                          const H3DGeometryMappingVector &mappings ) {

}

void Rope::updateSurfaceGeometry () {

}
void Rope::updateCollisionGeometry () {

}