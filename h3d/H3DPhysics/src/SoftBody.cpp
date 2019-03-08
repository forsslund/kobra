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
/// \file SoftBody.cpp
/// \brief Source file for SoftBody, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/SoftBody.h>
#include <H3D/H3DPhysics/CollidableShape.h>

#include <H3D/Coordinate.h>
#include <H3D/Shape.h>

using namespace H3D;  

H3DNodeDatabase SoftBody::database( "SoftBody", 
                                   &(newInstance< SoftBody >), 
                                   typeid( SoftBody ),
                                   &H3DSoftBodyNode::database);

SoftBody::SoftBody(Inst< SFNode > _metadata,
                   Inst< SFMatrix4f > _transform,
                   Inst< SFH3DPhysicsMaterialNode > _material,
                   Inst< SFX3DComposedGeometryNode > _geometry,
                   Inst< MFX3DComposedGeometryNode > _surfaceGeometry, 
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
  type_name = "SoftBody";
  database.initFields( this );

}

PhysicsEngineParameters::SoftBodyParameters* SoftBody::createSoftBodyParameters () {
  return new PhysicsEngineParameters::SoftBodyParameters();
}

PhysicsEngineParameters::SoftBodyParameters* SoftBody::getSoftBodyParameters( bool all_params ) {

  PhysicsEngineParameters::SoftBodyParameters *params= 
    static_cast<PhysicsEngineParameters::SoftBodyParameters*>(H3DSoftBodyNode::getSoftBodyParameters ( all_params ));

  // If geometry node has changed, update the mesh in the simulation
  if ( all_params || valueUpdater->hasCausedEvent ( geometryChanged ) ) {
    X3DComposedGeometryNode* geom= dynamic_cast<X3DComposedGeometryNode*>(geometry->getValue());
    if ( geom ) {
      Coordinate* coord= dynamic_cast<Coordinate*> ( geom->coord->getValue() );
      if ( coord ) {
        Field *index = geom->getField( "index" );
        if( index ) {
          if( MFInt32 *mf_index = dynamic_cast< MFInt32 * >(index) ) {
            params->setIndices ( mf_index->getValue() );
            params->setCoords ( transformCoords ( coord->point->getValue() ) );
          }
        }
      } else {
        Console(4) << "Warning: " << getName() << ": " <<
          "The geometry field must use a Coordinate node." << endl;
      }
    }
  }

  return params;
}

void SoftBody::setSoftBodyParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params ) {
  H3DSoftBodyNode::setSoftBodyParameters ( params );

  // update the geometry node coords and indices
  X3DComposedGeometryNode* geom= dynamic_cast<X3DComposedGeometryNode*>(geometry->getValue());
  if ( geom ) {
    Coordinate* coord= dynamic_cast<Coordinate*>(geom->coord->getValue ());
    if ( coord ) {
      Field *index = geom->getField( "index" );
      MFInt32 *mf_index = NULL;
      if( index ) mf_index = dynamic_cast< MFInt32 * >(index);

      if( mf_index )
        mf_index->unroute ( geometryChanged );
      geom->coord->unroute ( geometryChanged );
      coord->point->unroute ( geometryChanged );

      if( mf_index )
        mf_index->setValue ( params.getIndices() );
      coord->point->setValue ( params.getCoords() );

      if( mf_index )
        mf_index->routeNoEvent ( geometryChanged );
      geom->coord->routeNoEvent ( geometryChanged );
      coord->point->routeNoEvent ( geometryChanged );
    } else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The geometry field must use a Coordinate node." << endl;
    }
  }
}

void SoftBody::linkGeometries (const X3DGeometryNode& sourceGeometry, X3DGeometryNodeList& linkingGeometries,
                               const H3DGeometryMappingVector& mappings )
{
  const X3DComposedGeometryNode* geom= dynamic_cast<const X3DComposedGeometryNode*>( &sourceGeometry );
  if( geom ){

    if ( linkingGeometries.size() == mappings.size()) {
      Matrix4f _transform= getTransform();

      std::vector<X3DGeometryNode* >::iterator i= linkingGeometries.begin();
      std::vector<H3DGeometryMapping* >::const_iterator j = mappings.begin(); 

      for ( ;(i != linkingGeometries.end() && j != mappings.end()); ++i, ++j ) {

        X3DComposedGeometryNode* linkingGeom= dynamic_cast<X3DComposedGeometryNode*>(*i);
        H3DGeometryMapping* mapping = (*j);

        if ( mapping && linkingGeom && geom ) {

          Coordinate* linkingCoord= dynamic_cast<Coordinate*>(linkingGeom->coord->getValue());
          Coordinate* coord= dynamic_cast<Coordinate*>(geom->coord->getValue());

          if ( linkingCoord && coord ) {
            Field *index = geom->getField( "index" );
            if( MFInt32 * mf_index = dynamic_cast< MFInt32 * >(index) )
              mapping->linkGeometry( coord->point->getValue(), mf_index->getValue(),
                linkingCoord->point->getValue(), _transform );
          }
        }
      }
    }
    else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The mappings and surface/collision geometries should have the same size." << endl;    
    }

  }
  else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must use a Coordinate node." << endl;
  }

}

void SoftBody::updateSurfaceGeometry () {

  X3DComposedGeometryNode* geom= 
    dynamic_cast<X3DComposedGeometryNode*>( geometry->getValue()  );
  Coordinate* coord= dynamic_cast<Coordinate*>(geom->coord->getValue());

  if( geom && coord ){

    const NodeVector &mappings = (surfaceMapping->getValue());
    const NodeVector &surfaceGeoms = (surfaceGeometry->getValue());

    if ( surfaceGeoms.size() == mappings.size()) {

      for( unsigned int i = 0; i < surfaceGeoms.size(); ++i ) {

        X3DGeometryNode* s_geom = static_cast< X3DGeometryNode* >( surfaceGeoms[i] );
        H3DGeometryMapping *mapping = static_cast< H3DGeometryMapping* >( mappings[i] );

        X3DComposedGeometryNode* surfaceGeom= dynamic_cast<X3DComposedGeometryNode*>(s_geom);          
        Coordinate* surfaceCoord= surfaceGeom ? dynamic_cast<Coordinate*>(surfaceGeom->coord->getValue()) : NULL;

        if ( mapping && surfaceGeom && surfaceCoord ) {
          Field *index = geom->getField( "index" );
          if( MFInt32 * mf_index = dynamic_cast< MFInt32 * >(index) ) {
            H3DGeometryMapping::CoordList surfaceCoordinates;
            mapping->updateDependentGeometry( coord->point->getValue(), mf_index->getValue(),
              surfaceCoordinates );
            surfaceCoord->point->setValue ( surfaceCoordinates );
          }
        }
      }

    }
    else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The mappings and surface geometries should have the same size." << endl;    
    }

  }
  else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must use a Coordinate node." << endl;
  }

}

void SoftBody::updateCollisionGeometry () {
  X3DComposedGeometryNode* geom = 
    dynamic_cast<X3DComposedGeometryNode*>( geometry->getValue() );
  Coordinate* coord= dynamic_cast<Coordinate*>(geom->coord->getValue());

  if( geom && coord ){

    const NodeVector &mappings = (surfaceMapping->getValue());
    const NodeVector &collisionGeoms = (collisionGeometry->getValue());

    if ( collisionGeoms.size() == mappings.size()) {

      for( unsigned int i = 0; i < collisionGeoms.size(); ++i ) {

        H3DGeometryMapping *mapping = static_cast< H3DGeometryMapping* >( mappings[i] );
        X3DNBodyCollidableNode* c_node = static_cast< X3DNBodyCollidableNode* >( collisionGeoms[i] );

        CollidableShape* c_shape_node = dynamic_cast<CollidableShape*>(c_node);
        Shape* c_shape = dynamic_cast<Shape*>(c_shape_node->shape->getValue());
        X3DComposedGeometryNode* collisionGeom= 
          dynamic_cast<X3DComposedGeometryNode*>(c_shape->geometry->getValue() );  
        Coordinate* collisionCoord= collisionGeom ? dynamic_cast<Coordinate*>(collisionGeom->coord->getValue()) : NULL;

        if ( mapping && collisionGeom && collisionCoord ) {

          Field *index = geom->getField( "index" );
          if( MFInt32 * mf_index = dynamic_cast< MFInt32 * >(index) ) {
            H3DGeometryMapping::CoordList collisionCoordinates;
            mapping->updateDependentGeometry( coord->point->getValue(), mf_index->getValue(),
              collisionCoordinates );
            collisionCoord->point->setValue ( collisionCoordinates );
          }
        }
      }
    }
    else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The mappings and surface geometries should have the same size." << endl;    
    }

  }
  else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must use a Coordinate node." << endl;
  }
}