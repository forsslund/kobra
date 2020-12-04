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
/// \file TetGenSoftBodyLoader.cpp
/// \brief Source file for TetGenSoftBodyLoader, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/TetGenSoftBodyLoader.h>
#include <H3D/H3DPhysics/CollidableShape.h>

#include <H3D/Coordinate.h>
#include <H3D/IndexedFaceSet.h>
#include <H3D/Material.h>
#include <H3D/FrictionalSurface.h>
#include <H3D/Shape.h>
#include <H3DUtil/ResourceResolver.h>
#include <fstream>

#undef max
#undef min
#include <limits>

using namespace H3D;  

H3DNodeDatabase TetGenSoftBodyLoader::database( "TetGenSoftBodyLoader", 
                                               &(newInstance<TetGenSoftBodyLoader>), 
                                               typeid( TetGenSoftBodyLoader ),
                                               &H3DSoftBodyLoader::database);

TetGenSoftBodyLoader::TetGenSoftBodyLoader (
  Inst< SFNode            > _metadata,
  Inst< SFString          > _output,
  Inst< SFIndexedTetraSet > _geometry,
  Inst< MFX3DComposedGeometryNode > _surfaceGeometry,
  Inst< MFX3DNBodyCollidableNode > _collisionGeometry,
  Inst< SFBool            > _success,
  Inst< MFString          >  _url
  ) :
H3DSoftBodyLoader (  _metadata, _output, _geometry, _surfaceGeometry,
                   _collisionGeometry, _success, _url )
{
  // init fields
  type_name = "TetGenSoftBodyLoader";
  database.initFields( this );
}

void TetGenSoftBodyLoader::initialize () {
  H3DSoftBodyLoader::createGeometries();
  success->setValue ( load ( url->getValue() ), id );
  H3DSoftBodyLoader::initialize();
}

bool TetGenSoftBodyLoader::validLine ( const string& line ) {

  for ( string::const_iterator i= line.begin(); i != line.end(); ++i ) {
    if ( !isspace(*i) ) {
      if ( *i == '#' ) {
        // Comment line
        return false;
      } else {
        return true;
      }
    }
  }
  // Empty line
  return false;
}

string TetGenSoftBodyLoader::getTetGenLine ( istream& input ) {
  string line;
  getline ( input, line );
  while ( !validLine ( line ) && !input.eof() ) {
    getline ( input, line );
  }

  if ( input.eof() ) {
    line= "";
  }

  return line;
}

bool TetGenSoftBodyLoader::load ( const vector< string > &urls ) {

  X3DComposedGeometryNode* sg = 
    dynamic_cast <X3DComposedGeometryNode*>( surfaceGeometry->getValue()[0]);

  IndexedTriangleSet* tris= dynamic_cast<IndexedTriangleSet*>(sg);
  IndexedFaceSet* faces= dynamic_cast<IndexedFaceSet*>(sg);
  if ( !tris && !faces ) {
    Console(4) << "Warning: " << getName() << ": " <<
      "Invalid type for surfaceGeometry (" << sg->getTypeName() << "). Field must contain one of: "
      "IndexedTriangleSet, IndexedFaceSet." << endl;
    return false;
  }

  if ( !geometry->getValue() ) {
    Console(4) << "Warning: " << getName() << ": " <<
      "Field geometry must not be NULL." << endl;
    return false;
  }

  if ( !collisionGeometry->getValue()[0] ) {
    Console(4) << "Warning: " << getName() << ": " <<
      "Field collisionGeometry must be set." << endl;
    return false;
  }

  bool opened_file = false;
  for( unsigned int i = 0; i < urls.size(); ++i ) {
    bool node_is_tmp_file = false;
    string filename = resolveURLAsFile( urls[i] + ".node", &node_is_tmp_file );
    if( filename == "" ) continue;
    ifstream nodesFile ( filename.c_str() );
    if( !nodesFile.is_open() ) continue;

    bool read_file = loadNodes ( nodesFile );
    nodesFile.close();
    if( node_is_tmp_file ) ResourceResolver::releaseTmpFileName( filename );
    if( !read_file ) {
      Console(4) << "Warning: Could not read .node file for node " <<
                    getName() << " tried url " << filename << endl;
      return false;
    }
    opened_file = true;

    string tetra_filename;
    bool is_tmp_file = false;
    if( node_is_tmp_file )
      tetra_filename = resolveURLAsFile( urls[i] + ".ele", &is_tmp_file );
    else {
      filename = filename.substr( 0, filename.size() - 5 );
      tetra_filename = filename + ".ele";
    }

    ifstream tetraFile ( tetra_filename.c_str() );
    if( !tetraFile.is_open() ) {
      Console(4) << "Warning: " << getName() << ": " <<
        "Could not open ele file: " << tetra_filename << endl;
      return false;
    }
    read_file = loadTetra ( tetraFile );
    tetraFile.close();
    if( is_tmp_file ) ResourceResolver::releaseTmpFileName( tetra_filename );
    if( !read_file ) {
      Console(4) << "Warning: Could not read .ele file for node " <<
                    getName() << " tried url " << tetra_filename << endl;
      return false;
    }

    string tris_filename;
    is_tmp_file = false;
    if( node_is_tmp_file )
      tris_filename = resolveURLAsFile( urls[i] + ".face", &is_tmp_file );
    else {
      tris_filename = filename + ".face";
    }

    ifstream trisFile ( tris_filename.c_str() );
    if( !trisFile.is_open() ) {
      Console(4) << "Warning: " << getName() << ": " <<
        "Could not open face file: " << tris_filename << endl;
      return false;
    }
    read_file = loadTriangles ( trisFile );
    trisFile.close();
    if( is_tmp_file ) ResourceResolver::releaseTmpFileName( tris_filename );
    if( !read_file ) {
      Console(4) << "Warning: Could not read .face file for node " <<
                    getName() << " tried url " << tris_filename << endl;
      return false;
    }
  }

  if( !opened_file ) {
    Console(4) << "Warning: Could not resolve any of " <<
                  "the urls in url field of node " << getName() << endl;
    return false;
  }

  return true;
}

bool TetGenSoftBodyLoader::loadNodes ( istream& input ) {
  IndexedTetraSet* tetra= static_cast<IndexedTetraSet*>(geometry->getValue());
  Coordinate* coord= static_cast<Coordinate*>(tetra->coord->getValue());

  vector<Vec3f> nodes;

  string line= getTetGenLine ( input ); // Skip first line
  while ( !input.eof() ) {
    line= getTetGenLine ( input );
    if ( line != "" ) {
      istringstream iss(line);
      int index;
      Vec3f point;
      iss >> index;
      iss >> point.x;
      iss >> point.y;
      iss >> point.z;
      nodes.push_back ( point );
    }
  }

  coord->point->setValue ( nodes );

  return true;
}

bool TetGenSoftBodyLoader::loadTetra ( istream& input ) {
  IndexedTetraSet* tetra= static_cast<IndexedTetraSet*>(geometry->getValue());

  vector<H3DInt32> indices;
  H3DInt32 min= numeric_limits<int>::max();

  string line= getTetGenLine ( input ); // Skip first line
  while ( !input.eof() ) {
    line= getTetGenLine ( input );
    if ( line != "" ) {
      istringstream iss(line);
      int index;
      H3DInt32 a, b, c, d;
      iss >> index;
      iss >> a;
      iss >> b;
      iss >> c;
      iss >> d;
      indices.push_back ( a-1 );
      indices.push_back ( b-1 );
      indices.push_back ( c-1 );
      indices.push_back ( d-1 );

      if ( a < min ) min= a;
      if ( b < min ) min= b;
      if ( c < min ) min= c;
      if ( d < min ) min= d;
    }
  }

  H3DInt32 delta= 1-min;
  for ( vector<H3DInt32>::iterator i= indices.begin(); i != indices.end(); ++i ) {
    (*i)+= delta;
  }

  tetra->index->setValue ( indices );

  return true;
}

bool TetGenSoftBodyLoader::loadTriangles ( istream& input ) {

  X3DComposedGeometryNode* sg = 
    dynamic_cast <X3DComposedGeometryNode*>( surfaceGeometry->getValue()[0]);

  IndexedTriangleSet* tris= dynamic_cast<IndexedTriangleSet*>(sg);
  IndexedFaceSet* faces= dynamic_cast<IndexedFaceSet*>(sg);

  CollidableShape* cs = dynamic_cast<CollidableShape*>
    (collisionGeometry->getValue()[0]);

  Shape* css = dynamic_cast<Shape*>
    (cs->shape->getValue());

  X3DComposedGeometryNode* cg= dynamic_cast<X3DComposedGeometryNode*>
    (css->geometry->getValue());

  IndexedTriangleSet* trisCg= dynamic_cast<IndexedTriangleSet*>(cg);
  IndexedFaceSet* facesCg= dynamic_cast<IndexedFaceSet*>(cg);

  vector<H3DInt32> indices;
  H3DInt32 min= numeric_limits<int>::max();

  string line= getTetGenLine ( input ); // Skip first line
  while ( !input.eof() ) {
    line= getTetGenLine ( input );
    if ( line != "" ) {
      istringstream iss(line);
      int index;
      H3DInt32 a, b, c;
      iss >> index;
      iss >> a;
      iss >> b;
      iss >> c;
      indices.push_back ( c-1 );
      indices.push_back ( b-1 );
      indices.push_back ( a-1 );
      if ( faces ) {
        indices.push_back ( -1 );
      }

      if ( a < min ) min= a;
      if ( b < min ) min= b;
      if ( c < min ) min= c;
    }
  }

  H3DInt32 delta= 1-min;
  size_t step= faces ? 4 : 3; // Step size (IndexedFaceSet includes terminating -1)
  for ( size_t i= 0; i < indices.size(); i+= step ) {
    indices[i]+= delta;
    indices[i+1]+= delta;
    indices[i+2]+= delta;
  }

  IndexedTetraSet* st_set = dynamic_cast<IndexedTetraSet*>
    (geometry->getValue());  
  sg->coord->setValue ( st_set->coord->getValue() );
  cg->coord->setValue ( st_set->coord->getValue() );

  if ( tris ) {
    tris->set_index->setValue ( indices );
    trisCg->set_index->setValue ( indices );
  } else if ( faces ) {
    faces->set_coordIndex->setValue ( indices );
    facesCg->set_coordIndex->setValue ( indices );
  }

  return true;
}

X3DGeometryNode* TetGenSoftBodyLoader::createGeometry() {
  IndexedTetraSet* tetraset= new IndexedTetraSet;
  tetraset->coord->setValue ( new Coordinate );
  return tetraset;
}

H3DSoftBodyNode::X3DGeometryNodeList* TetGenSoftBodyLoader::createSurfaceGeometry() {
  IndexedTriangleSet* triSet= new IndexedTriangleSet;

  IndexedTetraSet* tetraset= dynamic_cast<IndexedTetraSet*>
    (geometry->getValue());  

  triSet->coord->setValue ( tetraset->coord->getValue() );
  sgList.clear();
  sgList.push_back( triSet);
  return (&sgList);
}

H3DSoftBodyNode::X3DNBodyCollidableNodeList* TetGenSoftBodyLoader::createCollisionGeometry() {
  IndexedTriangleSet* triset= new IndexedTriangleSet;
  IndexedTetraSet* tetraset= dynamic_cast<IndexedTetraSet*>
    (geometry->getValue());  

  triset->coord->setValue ( tetraset->coord->getValue() );
  Shape* shape= new Shape;
  Appearance* app= new Appearance;
  app->material->setValue ( new Material );
  app->surface->setValue ( new FrictionalSurface );
  shape->appearance->setValue ( app );
  shape->geometry->setValue ( triset );
  CollidableShape *cs = new CollidableShape;
  cs->shape->setValue( shape );

  cnList.clear();
  cnList.push_back( cs );
  return (&cnList);
}
