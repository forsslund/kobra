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
/// \file H3DSoftBodyLoader.cpp
/// \brief Source file for H3DSoftBodyLoader, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DSoftBodyLoader.h>
#include <H3D/H3DPhysics/PhysicsBodyCollection.h>
#include <H3D/H3DPhysics/SoftBody.h>
#include <H3D/Coordinate.h>
#include <H3D/Shape.h>
#include <H3D/Material.h>
#include <H3D/FrictionalSurface.h>
#include <H3D/X3D.h>
#include <fstream>

using namespace H3D;  

H3DNodeDatabase H3DSoftBodyLoader::database( "H3DSoftBodyLoader", 
                                            NULL, 
                                            typeid( H3DSoftBodyLoader ),
                                            &X3DChildNode::database);

namespace H3DTetraLoaderInternals {
  FIELDDB_ELEMENT( H3DSoftBodyLoader, output, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( H3DSoftBodyLoader, geometry, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DSoftBodyLoader, surfaceGeometry, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( H3DSoftBodyLoader, collisionGeometry, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( H3DSoftBodyLoader, success, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DSoftBodyLoader, url, INITIALIZE_ONLY )
  // add an alias for the children field to work with old files.
  FieldDBInsert string( INPUT_OUTPUT( &H3DSoftBodyLoader::database, "filename", &H3DSoftBodyLoader::url ) );
}

H3DSoftBodyLoader::H3DSoftBodyLoader (Inst< SFNode            > _metadata,
                                      Inst< SFString          > _output,
                                      Inst< SFX3DGeometryNode > _geometry,
                                      Inst< MFX3DGeometryNode > _surfaceGeometry,
                                      Inst< MFX3DNBodyCollidableNode > _collisionGeometry,
                                      Inst< SFBool            > _success,
                                      Inst< MFString          >  _url ) :
X3DChildNode( _metadata ), X3DUrlObject( _url ),
geometry ( _geometry ), surfaceGeometry ( _surfaceGeometry ), 
collisionGeometry( _collisionGeometry ), success ( _success ), output ( _output )
{
  // init fields
  type_name = "H3DSoftBodyLoader";
  database.initFields( this );

  success->setValue ( false, id );
}

void H3DSoftBodyLoader::initialize () {
  X3DChildNode::initialize();

  if ( success->getValue() && output->getValue() != "" ) {
    ofstream outputFile ( output->getValue().c_str() );
    if ( outputFile.is_open() ) {
      auto_ptr< Group > group ( new Group );

      Shape* shape= new Shape;
      Appearance* app= new Appearance;
      app->material->setValue ( new Material );
      app->surface->setValue ( new FrictionalSurface );
      shape->appearance->setValue ( app );
      //WARNINGUMUT : Only one surface geometry assumed.
      shape->geometry->setValue ( surfaceGeometry->getValue()[0] );
      group->children->push_back ( shape );

      PhysicsBodyCollection* srbc= new PhysicsBodyCollection;
      SoftBody* sb= new SoftBody;
      sb->geometry->setValue ( geometry->getValue() );
      sb->surfaceGeometry->setValue ( surfaceGeometry->getValue() );
      sb->collisionGeometry->setValue( collisionGeometry->getValue() );
      srbc->softBodies->push_back ( sb );
      group->children->push_back ( srbc );

      X3D::writeNodeAsX3D ( outputFile, group.get() );

    } else {
      Console(4) << "Warning: Could not write to output file "
                 << output->getValue()
                 << " in node " << getName() << endl;
    }
  }
}
void H3DSoftBodyLoader::createGeometries() {
  geometry->setValue( createGeometry(), id );

  NodeVector surface_nodes;
  std::vector<X3DGeometryNode*>  surfaces = (*createSurfaceGeometry());
  for( unsigned int i = 0; i<surfaces.size(); ++i )
    surface_nodes.push_back( surfaces[i] );
  surfaceGeometry->setValue( surface_nodes );

  NodeVector col_nodes;
  std::vector<X3DNBodyCollidableNode*>  col_surfaces = (*createCollisionGeometry());
  for( unsigned int i = 0; i<col_surfaces.size(); ++i )
    col_nodes.push_back( col_surfaces[i] );
  collisionGeometry->setValue( col_nodes );
}
