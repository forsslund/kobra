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
/// \file GmshSoftBodyLoader.cpp
/// \brief Source file for GmshSoftBodyLoader, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/GmshSoftBodyLoader.h>
#include <H3D/H3DPhysics/CollidableShape.h>

#include <H3D/Coordinate.h>
#include <H3D/IndexedFaceSet.h>
#include <H3D/Material.h>
#include <H3D/FrictionalSurface.h>
#include <H3D/Shape.h>
#include <H3D/ResourceResolver.h>
#include <fstream>

#undef max
#undef min
#include <limits>

using namespace H3D;  

H3DNodeDatabase GmshSoftBodyLoader::database( "GmshSoftBodyLoader", 
                                            &(newInstance<GmshSoftBodyLoader>), 
                                            typeid( GmshSoftBodyLoader ),
                                            &H3DSoftBodyLoader::database);

GmshSoftBodyLoader::GmshSoftBodyLoader (
                                        Inst< SFNode            > _metadata,
                                        Inst< SFString          > _output,
                                        Inst< SFIndexedTetraSet > _geometry,
                                        Inst< MFX3DComposedGeometryNode > _surfaceGeometry,
                                        Inst< MFX3DNBodyCollidableNode > _collisionGeometry,
                                        Inst< SFBool            > _success,
                                        Inst< MFString          >  _url
                                      )
  : H3DSoftBodyLoader ( _metadata, _output, _geometry, _surfaceGeometry, _collisionGeometry, _success, _url )
{
  // init fields
  type_name = "GmshSoftBodyLoader";
  database.initFields( this );
}

void GmshSoftBodyLoader::initialize () {
  H3DSoftBodyLoader::createGeometries();
  success->setValue ( load ( url->getValue() ), id );
  H3DSoftBodyLoader::initialize();
}

bool GmshSoftBodyLoader::validLine ( const string& line ) {

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

string GmshSoftBodyLoader::getGmshLine ( istream& input ) {
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

bool GmshSoftBodyLoader::load ( const vector< string > &urls ) {
  X3DComposedGeometryNode* sg= 
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
    bool is_tmp_file = false;
    string filename = resolveURLAsFile( urls[i], &is_tmp_file );
    if( filename.empty() ) continue;
    // This should be rewritten to add the proper read element to the right
    // node. Currently we reread the file at least once.
    ifstream mshFile ( filename.c_str() );
    if ( mshFile.is_open() ) {
      opened_file = true;
      if ( !loadNodes ( mshFile ) ) {
        mshFile.close();
        if( is_tmp_file ) ResourceResolver::releaseTmpFileName( filename );
        return false;
      }
      streampos pos_before_tetra_load = mshFile.tellg();
      if ( !loadTetra ( mshFile ) ) {
        mshFile.close();
        if( is_tmp_file ) ResourceResolver::releaseTmpFileName( filename );
        return false;
      }
      mshFile.seekg( pos_before_tetra_load );
      if ( !loadTriangles ( mshFile ) ) {
        mshFile.close();
        if( is_tmp_file ) ResourceResolver::releaseTmpFileName( filename );
        return false;
      }
      mshFile.close();
      if( is_tmp_file ) ResourceResolver::releaseTmpFileName( filename );
    }
  }
  if( !opened_file ) {
    Console(4) << "Warning: " << getName() << ": " <<
      "Could not open msh file: " << url->getValueAsString() << endl;
    return false;
  }

  return true;
}

bool GmshSoftBodyLoader::loadNodes ( istream& input ) {
  IndexedTetraSet* tetra= static_cast<IndexedTetraSet*>(geometry->getValue());
  Coordinate* coord= static_cast<Coordinate*>(tetra->coord->getValue());

  vector<Vec3f> nodes;

  string line = "";

  // skip until nodes section
  while ( !input.eof() && line!= "$Nodes") {
   line= getGmshLine ( input );
  }

  line= getGmshLine ( input ); // Skip first line after nodes section
  // it contains the number of nodes...

  // read until end of nodes section
  while ( !input.eof() && line!= "$EndNodes") {
   line= getGmshLine ( input );
   if ( line != "" ) {
     istringstream iss(line);
     int index;
     Vec3f point;
     iss >> index; // not really used...
     iss >> point.x;
     iss >> point.y;
     iss >> point.z;
     nodes.push_back ( point );
   }
  }

  coord->point->setValue ( nodes );

  return true;
}

bool GmshSoftBodyLoader::loadTetra ( istream& input ) {
  IndexedTetraSet* tetra= static_cast<IndexedTetraSet*>(geometry->getValue());

  vector<H3DInt32> indices;
  H3DInt32 min= numeric_limits<int>::max();

  string line = "";

  // skip until elements section
  while ( !input.eof() && line!= "$Elements") {
   line= getGmshLine ( input );
  }

  line= getGmshLine ( input ); // Skip first line after elements section
  // it contains the number of elements...

  // read until end of elements section
  while ( !input.eof() && line!= "$EndElements") {
   line= getGmshLine ( input );
   if ( line != "" ) {
     istringstream iss(line);
     int index, elementtype, skipentries, dummy;
     H3DInt32 a, b, c, d;
     iss >> index;  // not really used...
   iss >> elementtype;
   iss >> skipentries; // some useless entries, that we can ignore

   // check if the element is a tetrahedra
   if (elementtype == 4)
   {
     for (int i=0; i < skipentries; ++i)
     {
        iss >> dummy;
     }
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
  }

  H3DInt32 delta= 1-min;
  for ( vector<H3DInt32>::iterator i= indices.begin(); i != indices.end(); ++i ) {
   (*i)+= delta;
  }

  tetra->index->setValue ( indices );

  return true;
}

bool GmshSoftBodyLoader::loadTriangles ( istream& input ) {
  
  X3DComposedGeometryNode* sg = 
    dynamic_cast <X3DComposedGeometryNode*>( surfaceGeometry->getValue()[0]);
  
  IndexedTriangleSet* tris= dynamic_cast<IndexedTriangleSet*>(sg);
  IndexedFaceSet* faces= dynamic_cast<IndexedFaceSet*>(sg);

  vector<H3DInt32> indices;
  H3DInt32 min= numeric_limits<int>::max();

  string line = "";

  // skip until elements section
  while ( !input.eof() && line!= "$Elements") {
   line= getGmshLine ( input );
  }

  line= getGmshLine ( input ); // Skip first line after elements section
  // it contains the number of elements...

  // read until end of elements section
  while ( !input.eof() && line!= "$EndElements") {
   line= getGmshLine ( input );
   if ( line != "" ) {
     istringstream iss(line);
     int index, elementtype, skipentries, dummy;
     H3DInt32 a, b, c;
     iss >> index;
   iss >> elementtype;
   iss >> skipentries; // some useless entries, that we can ignore

   // check if the element is a triangle
   if (elementtype == 2)
   {
     for (int i=0; i < skipentries; ++i)
     {
        iss >> dummy;
     }
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
  }

  H3DInt32 delta= 1-min;
  size_t step= faces ? 4 : 3; // Step size (IndexedFaceSet includes terminating -1)
  for ( size_t i= 0; i < indices.size(); i+= step ) {
   indices[i]+= delta;
   indices[i+1]+= delta;
   indices[i+2]+= delta;
  }

  sg->coord->setValue ( static_cast<IndexedTetraSet*>(geometry->getValue())->coord->getValue() );
  if ( tris ) {
   tris->set_index->setValue ( indices );
  } else if ( faces ) {
   faces->set_coordIndex->setValue ( indices );
  }

  return true;
}

X3DGeometryNode* GmshSoftBodyLoader::createGeometry() {
  IndexedTetraSet* tetraset= new IndexedTetraSet;
  tetraset->coord->setValue ( new Coordinate );
  return tetraset;
}

H3DSoftBodyNode::X3DGeometryNodeList* GmshSoftBodyLoader::createSurfaceGeometry() {
  IndexedTriangleSet* triSet= new IndexedTriangleSet;

  IndexedTetraSet* tetraset= dynamic_cast<IndexedTetraSet*>
    (geometry->getValue());  

  triSet->coord->setValue ( tetraset->coord->getValue() );
  sgList.clear();
  sgList.push_back( triSet);
  return (&sgList);
}

H3DSoftBodyNode::X3DNBodyCollidableNodeList* GmshSoftBodyLoader::createCollisionGeometry() {
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
