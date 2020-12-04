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
/// \file SOFAHexmshSoftBodyLoader.cpp
/// \brief Source file for SOFAHexmshSoftBodyLoader, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/SOFAHexmshSoftBodyLoader.h>
#include <H3D/H3DPhysics/IndexedTetraSet.h>
#include <H3D/Coordinate.h>
#include <H3D/IndexedFaceSet.h>
#include <H3D/IndexedTriangleSet.h>
#include <H3D/H3DPhysics/CollidableShape.h>
#include <H3D/Material.h>
#include <H3D/FrictionalSurface.h>
#include <H3D/Shape.h>
#include <H3DUtil/ResourceResolver.h>
#include <fstream>

#undef max
#undef min
#include <limits>

using namespace H3D;  

H3DNodeDatabase SOFAHexmshSoftBodyLoader::database( "SOFAHexmshSoftBodyLoader", 
                                            &(newInstance<SOFAHexmshSoftBodyLoader>), 
                                            typeid( SOFAHexmshSoftBodyLoader ),
                                            &H3DSoftBodyLoader::database);

SOFAHexmshSoftBodyLoader::SOFAHexmshSoftBodyLoader (
                                        Inst< SFNode            > _metadata,
                                        Inst< SFString          > _output,
                                        Inst< SFX3DGeometryNode > _geometry,
                                        Inst< MFX3DComposedGeometryNode > _surfaceGeometry,
                                        Inst< MFX3DNBodyCollidableNode > _collisionGeometry, 
                                        Inst< SFBool            > _success,
                                        Inst< MFString          > _url
                                      )
  : H3DSoftBodyLoader (  _metadata, _output, _geometry, _surfaceGeometry, _collisionGeometry, _success, _url )
{
  // init fields
  type_name = "SOFAHexmshSoftBodyLoader";
  database.initFields( this );
}

void SOFAHexmshSoftBodyLoader::initialize () {
  H3DSoftBodyLoader::createGeometries();
  success->setValue ( load ( url->getValue() ), id );
  H3DSoftBodyLoader::initialize();
}

bool SOFAHexmshSoftBodyLoader::validLine ( const string& line ) {

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

string SOFAHexmshSoftBodyLoader::getGmshLine ( istream& input ) {
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

bool SOFAHexmshSoftBodyLoader::load ( const vector< string > &urls ) {

  const NodeVector& sg= surfaceGeometry->getValue();
  IndexedTriangleSet* tris= NULL;
  IndexedFaceSet* faces= NULL;
  if ( sg.size() > 0 ) {
    tris= dynamic_cast<IndexedTriangleSet*>(sg[0]);
    faces= dynamic_cast<IndexedFaceSet*>(sg[0]);
    if ( !tris && !faces ) {
      Console(4) << "Warning: " << getName() << ": " <<
        "Invalid type for surfaceGeometry. Field must contain one of: "
        "IndexedTriangleSet, IndexedFaceSet." << endl;
      return false;
    }
  } else {
    Console(4) << "Warning: " << getName() << ": " <<
      "Field surfaceGeometry must not be empty." << endl;
    return false;
  }

  bool opened_file = false;
  for( unsigned int i = 0; i < urls.size(); ++i ) {
    bool node_is_tmp_file = false;
    string filename = resolveURLAsFile( urls[i] + ".node", &node_is_tmp_file );
    if( filename == "" ) continue;
    ifstream nodesFile ( filename.c_str() );
    if( !nodesFile.is_open() ) continue;

    string transform_filename;
    string filename_base = "";
    bool is_tmp_file = false;
    if( node_is_tmp_file )
      transform_filename = resolveURLAsFile( urls[i] + ".transform", &is_tmp_file );
    else {
      filename_base = filename.substr( 0, filename.size() - 5 );
      transform_filename = filename_base + ".transform";
    }
    ifstream transFile ( transform_filename.c_str() );
    if( !transFile.is_open() ) {
      Console(4) << "Warning: " << getName() << ": " <<
        "No transform matrix given, using identity: " << transform_filename << endl;
      transformMatrix= Matrix4f();
    } else {
      bool read_file = loadTransform( transFile );
      transFile.close();
      if( !read_file ) {
        Console( 4 ) << "Warning: " << getName() << ": " <<
          "Could not load transform from transform file, using identity: " << transform_filename << endl;
        transformMatrix = Matrix4f();
      }
    }
    if( is_tmp_file ) ResourceResolver::releaseTmpFileName( transform_filename );

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
    is_tmp_file = false;
    if( node_is_tmp_file )
      tetra_filename = resolveURLAsFile( urls[i] + ".topo", &is_tmp_file );
    else {
      tetra_filename = filename_base + ".topo";
    }

    ifstream tetraFile ( tetra_filename.c_str() );
    if( !tetraFile.is_open() ) {
      Console(4) << "Warning: " << getName() << ": " <<
        "Could not open .topo file: " << tetra_filename << endl;
      return false;
    }
    read_file = loadTetra ( tetraFile );
    tetraFile.close();
    if( is_tmp_file ) ResourceResolver::releaseTmpFileName( tetra_filename );
    if( !read_file ) {
      Console(4) << "Warning: Could not read .topo file for node " <<
                    getName() << " tried url " << tetra_filename << endl;
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

bool SOFAHexmshSoftBodyLoader::loadNodes ( istream& input ) {
  IndexedTetraSet* tetra= dynamic_cast<IndexedTetraSet*>(geometry->getValue());
  if ( tetra ) {
    Coordinate* coord= dynamic_cast<Coordinate*>(tetra->coord->getValue());
    if ( coord ) {
      vector<Vec3f> nodes;

      string line = "";

      // skip until first time step
      while ( !input.eof() && line!= "T= 0") {
        line= getGmshLine ( input );
      }

      // skip until Node list
      while ( !input.eof() && line.substr(0,4)!="  X=") {
        line= getGmshLine ( input );
      }
      if (line.substr(0,4)=="  X=") 
      {
        istringstream iss(line.substr(4,line.length()-4));
        while (iss.peek() && !iss.eof())
        {
          Vec3f point;
          iss >> point.x;
          iss >> point.y;
          iss >> point.z;
          point = transformMatrix * point;
          nodes.push_back ( point );
        }
      }

      coord->point->setValue ( nodes );

      return true;
    } else {
      Console(4) << "Warning: " << getName() << ": " <<
        "The IndexedTetraSet in the geometry field must contain an Coordinate node." << endl;
      return false;
    }
  } else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must contain an IndexedTetraSet node." << endl;
    return false;
  }
}

bool SOFAHexmshSoftBodyLoader::loadTetra ( istream& input ) {
  IndexedTetraSet* tetra= dynamic_cast<IndexedTetraSet*>(geometry->getValue());
  if ( tetra ) {
    vector<H3DInt32> indices;
    H3DInt32 min= numeric_limits<int>::max();

    string line = "";

    // skip until first time step
    while ( !input.eof() && line!= "T= 0") {
      line= getGmshLine ( input );
    }

    int hexaCount= 0;
    int tetraCount= 0;
    // skip until Node list
    while ( !input.eof() && line.substr(0,11)!="  Hexahedra") {
      line= getGmshLine ( input );
    }
    if (line.substr(0,11)=="  Hexahedra") 
    {
      line= getGmshLine ( input );
      istringstream iss(line);

      while (iss.peek() && !iss.eof())
      {
        ++hexaCount;
        tetraCount+=5;
        // read hexa
        H3DInt32 h0,h1,h2,h3,h4,h5,h6,h7;
        iss >> h0;
        iss >> h1;
        iss >> h2;
        iss >> h3;
        iss >> h4;
        iss >> h5;
        iss >> h6;
        iss >> h7;

        // convert to 5 tetra
        indices.push_back(h0);
        indices.push_back(h1);
        indices.push_back(h5);
        indices.push_back(h2);

        indices.push_back(h7);
        indices.push_back(h3);
        indices.push_back(h2);
        indices.push_back(h0);

        indices.push_back(h0);
        indices.push_back(h5);
        indices.push_back(h4);
        indices.push_back(h7);

        indices.push_back(h5);
        indices.push_back(h6);
        indices.push_back(h2);
        indices.push_back(h7);

        indices.push_back(h5);
        indices.push_back(h7);
        indices.push_back(h2);
        indices.push_back(h0);

        if ( h0 < min ) min= h0;
        if ( h1 < min ) min= h1;
        if ( h2 < min ) min= h2;
        if ( h3 < min ) min= h3;
        if ( h4 < min ) min= h4;
        if ( h5 < min ) min= h5;
        if ( h6 < min ) min= h6;
        if ( h7 < min ) min= h7;
      }
    }

    H3DInt32 delta= min;
    for ( vector<H3DInt32>::iterator i= indices.begin(); i != indices.end(); ++i ) {
      (*i)+= delta;
    }

    tetra->index->setValue ( indices );

    return true;
  } else {
    Console(4) << "Warning: " << getName() << ": " <<
      "The geometry field must contain an IndexedTetraSet node." << endl;
    return false;
  }
}

bool SOFAHexmshSoftBodyLoader::loadTransform ( istream& input ) {
  string line = "";

  for (int i=0; i<4 && !input.eof(); ++i)
  {
    line= getGmshLine ( input );
    istringstream iss(line);
    float value;
    for (int j=0; j<4; ++j)
    {
      iss >> value;
      transformMatrix.setElement(i,j,value);
    }
  }
  return true;
}

X3DGeometryNode* SOFAHexmshSoftBodyLoader::createGeometry() {
  IndexedTetraSet* tetraset= new IndexedTetraSet;
  tetraset->coord->setValue ( new Coordinate );
  return tetraset;
}

H3DSoftBodyNode::X3DGeometryNodeList* SOFAHexmshSoftBodyLoader::createSurfaceGeometry() {
  IndexedTriangleSet* triSet= new IndexedTriangleSet;

  IndexedTetraSet* tetraset= dynamic_cast<IndexedTetraSet*>
    (geometry->getValue());  

  triSet->coord->setValue ( tetraset->coord->getValue() );
  sgList.clear();
  sgList.push_back( triSet);
  return (&sgList);
}

H3DSoftBodyNode::X3DNBodyCollidableNodeList* SOFAHexmshSoftBodyLoader::createCollisionGeometry() {
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
