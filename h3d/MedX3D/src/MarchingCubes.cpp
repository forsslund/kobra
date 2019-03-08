//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file MarchingCubes.cpp
/// \brief CPP file for MarchingCubes, MedX3D scene graph node.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/MedX3D/MarchingCubes.h>
#include <H3D/MultiTexture.h>
#include <H3D/GlobalSettings.h>
#include <H3D/TextureCoordinateGenerator.h>
#include <assert.h>
#include <math.h>
#include <limits>
#include <fstream>
#include <H3D/FloatVertexAttribute.h>

using namespace H3D;

#define TREEDEPTH 3

// a2iVertexOffset lists the positions (int), relative to vertex0,
// of each of the 8 vertices of a cube
const int MarchingCubes::a2iVertexOffset[8][3] =  {
  {0, 0, 0},{1, 0, 0},{1, 1, 0},{0, 1, 0},
  {0, 0, 1},{1, 0, 1},{1, 1, 1},{0, 1, 1}
};

// a2fVertexOffset lists the positions (float), relative to vertex0,
// of each of the 8 vertices of a cube
const float MarchingCubes::a2fVertexOffset[8][3] = {
  {0.0, 0.0, 0.0},{1.0, 0.0, 0.0},{1.0, 1.0, 0.0},{0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0},{1.0, 0.0, 1.0},{1.0, 1.0, 1.0},{0.0, 1.0, 1.0}
};

// a2iEdgeConnection lists the index of the endpoint vertices for each of
// the 12 edges of the cube
const int MarchingCubes::a2iEdgeConnection[12][2] = {
  {0,1}, {1,2}, {2,3}, {3,0},
  {4,5}, {5,6}, {6,7}, {7,4},
  {0,4}, {1,5}, {2,6}, {3,7}
};

// a2fEdgeDirection lists the direction vector (vertex1-vertex0) for each
// edge in the cube
const float MarchingCubes::a2fEdgeDirection[12][3] = {
  {1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{-1.0, 0.0, 0.0},{0.0, -1.0, 0.0},
  {1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{-1.0, 0.0, 0.0},{0.0, -1.0, 0.0},
  {0.0, 0.0, 1.0},{0.0, 0.0, 1.0},{ 0.0, 0.0, 1.0},{0.0,  0.0, 1.0}
};

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase MarchingCubes::database( "MarchingCubes", 
                                         &(newInstance<MarchingCubes>),
                                         typeid( MarchingCubes ),
                                         &X3DGeometryNode::database );

namespace MarchingCubesInternals { 
  FIELDDB_ELEMENT( MarchingCubes, isovalue, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MarchingCubes, voxels, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MarchingCubes, normalRenderMode, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MarchingCubes, writeMarchingCubesAsITS, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MarchingCubes, texCoord, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MarchingCubes, attrib, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MarchingCubes, trianglesBuilt, OUTPUT_ONLY )
  FIELDDB_ELEMENT( MarchingCubes, alwaysGenerateOriginalFVA, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MarchingCubes, voxelsAllowedToBeReinitialized, INPUT_OUTPUT )

  HAPI::Vec3 textureCoordinateFromMatrix( const Vec3f &c, void * data ) {
    const vector< H3DFloat > &d = *static_cast< vector< H3DFloat > * >(data);
    return HAPI::Vec3( d[0] * c.x + d[1] * c.y + d[2] * c.z + d[3],
                       d[4] * c.x + d[5] * c.y + d[6] * c.z + d[7],
                       d[8] * c.x + d[9] * c.y + d[10] * c.z + d[11] );
  }

  bool generateNormals( const MarchingCubes::OctTreeNode::NormalRenderMode &normal_render_mode,
                        const vector< Vec3f > &vvertex, const vector< Vec3f > &gradient_normals,
                        vector< Vec3f > **normals ) {
    if( normal_render_mode == MarchingCubes::OctTreeNode::NORMALS_PER_FACE ) {
      vector< Vec3f > face_normals( vvertex.size()/3 );
      for( unsigned int i = 0; i < vvertex.size(); i += 3 ) {
        Vec3d n = ( Vec3d(vvertex[i+1]) - Vec3d(vvertex[i]) ) % ( Vec3d(vvertex[i+2]) - Vec3d(vvertex[i+1]) );
        n.normalize();
        Vec3f n_f( n );
        face_normals[i/3] = n_f;
      }

      *normals = new std::vector< Vec3f >(gradient_normals.size());
      for( unsigned int i = 0; i < vvertex.size(); ++i ) {
        (*normals)->at(i) = face_normals[i/3];
      }
      return true;
    }
    return false;
  }

}

string MarchingCubes::original_mc_point_name = "original_mc_point";


MarchingCubes::MarchingCubes( Inst< SFNode    > _metadata, 
                              Inst< SFBound     > _bound,
                              Inst< DisplayList   > _displayList,
                              Inst< SFFloat       > _isovalue,
                              Inst< SFOctTree     > _octTree,
                              Inst< SFTexture3DNode > _voxels,
                              Inst< SFNormalRenderMode > _normalRenderMode,
                              Inst< SFWriteMCAsITS > _writeMarchingCubesAsITS,
                              Inst< SFTextureCoordinateNode > _texCoord,
                              Inst< SFBoundTree > _boundTree,
                              Inst< MFVertexAttributeNode > _attrib,
                              Inst< SFBool > _trianglesBuilt,
                              Inst< SFBool > _alwaysGenerateOriginalFVA,
                              Inst< SFBool > _voxelsAllowedToBeReinitialized ):
  X3DGeometryNode( _metadata, _bound, _displayList, NULL, NULL, NULL, NULL, NULL, _boundTree ),
  isovalue(_isovalue),
  octTree(_octTree),
  voxels( _voxels ),
  normalRenderMode( _normalRenderMode ),
  writeMarchingCubesAsITS( _writeMarchingCubesAsITS ),
  texCoord( _texCoord ),
  attrib( _attrib ),
  trianglesBuilt( _trianglesBuilt ),
  alwaysGenerateOriginalFVA( _alwaysGenerateOriginalFVA ),
  voxelsAllowedToBeReinitialized( _voxelsAllowedToBeReinitialized ) {

  type_name = "MarchingCubes"; 
  database.initFields( this ); 
  octTree->setOwner( this ); 
  octTree->setName( "octTree" );

  isovalue->setValue(10);

  normalRenderMode->addValidValue( "GRADIENT" );
  normalRenderMode->addValidValue( "NORMALS_PER_FACE" );
  normalRenderMode->setValue( "GRADIENT" );
  previous_normal_render_mode = normalRenderMode->getNormalRenderMode();

  isovalue->route( octTree );
  voxels->route( octTree );

  isovalue->route( displayList );
  voxels->route( displayList );
  normalRenderMode->route( displayList );
  texCoord->route( displayList );

  voxels->route( bound );
  // If cacheMode is on then the render function will generate
  // displayList errors because glNewList is called between glNewList
  // and glEndList pairs.
  displayList->setCacheMode( H3DDisplayListObject::DisplayList::OFF );

  trianglesBuilt->setValue( false, id );
  alwaysGenerateOriginalFVA->setValue( false, id );
  voxelsAllowedToBeReinitialized->setValue( true );
}

void MarchingCubes::SFOctTree::updateMinMaxValues( OctTreeNode *tree ) {
  tree->bound_tree->bound.reset( new HAPI::Collision::AABoxBound( HAPI::Vec3( ( tree->x_min - 0.5f * (x_points - 1) ) * voxel_size.x,
                                                                              ( tree->y_min - 0.5f * (y_points - 1) ) * voxel_size.y,
                                                                              ( tree->z_min - 0.5f * (z_points - 1) ) * voxel_size.z ),
                                                                  HAPI::Vec3( ( tree->x_max - 0.5f * (x_points - 1) ) * voxel_size.x,
                                                                              ( tree->y_max - 0.5f * (y_points - 1) ) * voxel_size.y,
                                                                              ( tree->z_max - 0.5f * (z_points - 1) ) * voxel_size.z ) ) );
  if ( tree->isLeaf() ) {
    // base case, our node is a leaf.

    // find the lowest and highest value within the volume of this leaf.
    for( int iZ = tree->z_min; iZ < tree->z_max; ++iZ ) {
      for( int iY = tree->y_min; iY < tree->y_max; ++iY ) {
        for( int iX = tree->x_min; iX < tree->x_max; ++iX ) {
          
          // update the min/max values
          if ( getVoxelValue(iX,iY,iZ) < tree->value_min ) {
            tree->value_min = getVoxelValue(iX,iY,iZ);
          }
          if ( getVoxelValue(iX,iY,iZ) > tree->value_max ) {
            tree->value_max = getVoxelValue(iX,iY,iZ);
          }
        }
      }
    }
  } else {
    for( int i=0; i<8; ++i ) {
      updateMinMaxValues( tree->children[i] );
    
      if ( tree->children[i]->value_min < tree->value_min )
        tree->value_min = tree->children[i]->value_min;
      if ( tree->children[i]->value_max > tree->value_max )
        tree->value_max = tree->children[i]->value_max;
    }
  }
}

void MarchingCubes::render() {
  OctTreeNode *oct_tree = octTree->getValue();
  if( oct_tree ) {
    X3DTextureCoordinateNode *tex_coord_node = texCoord->getValue();
    bool tex_coord_gen = 
      !tex_coord_node || (tex_coord_node && tex_coord_node->supportsTexGen());
    if( tex_coord_gen ) {
      startTexGen( tex_coord_node );
    }
    OctTreeNode::NormalRenderMode normal_render_mode = normalRenderMode->getNormalRenderMode();
    if( normal_render_mode != previous_normal_render_mode ) {
      oct_tree->breakCache();
      previous_normal_render_mode = normal_render_mode;
    }

    GraphicsOptions * go = NULL;
    getOptionNode( go );
    if( !go ) {
      GlobalSettings * gs = GlobalSettings::getActive();
      if( gs ) {
        gs->getOptionNode( go );
      }
    }
    RenderMode render_mode = DISPLAY_LISTS;
    if( go ) {
      if( !go->useCaching->getValue() ) {
        if( go->preferVertexBufferObject->getValue() )
          render_mode = VERTEX_BUFFER_OBJECTS;
        else
          render_mode = VERTEX_ARRAYS;
      }
    }

    map< string, GLint > attribute_location_map;
    GLhandleARB shader_program = 0;
    // Set the attribute index to use for all vertex attributes
    if( GLEW_ARB_shader_objects && GLEW_ARB_vertex_shader ) {
      shader_program = glGetHandleARB( GL_PROGRAM_OBJECT_ARB );
      if( shader_program ) {
        GLint loc = glGetAttribLocationARB( shader_program, original_mc_point_name.c_str());
        if( loc != -1 )
          attribute_location_map[original_mc_point_name] = loc;

        for( unsigned int i = 0; i < attrib->size(); ++i ) {
          X3DVertexAttributeNode *attr = attrib->getValueByIndex( i );
          if( attr ) {
            const string &attrib_name = attr->name->getValue();
            GLint loc = 
              glGetAttribLocationARB( shader_program, 
              attrib_name.c_str()); 
            if( loc != -1 )
              attribute_location_map[attrib_name] = loc;
          }
        }
      }
    }
    oct_tree->render( normal_render_mode, render_mode, attribute_location_map );
    if( tex_coord_gen ) {
      stopTexGen( tex_coord_node);
    }
  }
}

#undef max

template< class A >
void buildNormalizedData( float *normalized_data,
                          void *orig_data,
                          unsigned int width,
                          unsigned int height,
                          unsigned int depth,
                          int volume_min_x,
                          int volume_min_y,
                          int volume_min_z,
                          int volume_max_x,
                          int volume_max_y,
                          int volume_max_z,
                          float scale,
                          float bias ) {
  A *d = (A*) orig_data;

 for (int z = volume_min_z; z<volume_max_z; ++z) {
   for (int y = volume_min_y; y<volume_max_y; ++y) {
     for (int x = volume_min_x; x<volume_max_x; ++x) {
        unsigned int i = ( ( z * height + y ) * width + x );
        normalized_data[i] = 
          (d[ i ] / float( numeric_limits<A >::max() ) ) * scale + bias;
        if( normalized_data[i] < 0 ) normalized_data[i] = 0;
      }
    }
  }
}

void MarchingCubes::SFOctTree::updateDataMatrix( Image *image,
                                                int volume_min_x,
                                                int volume_min_y,
                                                int volume_min_z,
                                                int volume_max_x,
                                                int volume_max_y,
                                                int volume_max_z,  
                                                TextureProperties *tp ) {
  if( image ) {
    unsigned int d = image->depth();
    unsigned int h = image->height();
    unsigned int w = image->width();

    float bias = 0;
    float scale = 1;

    if( tp ) {
      bias = tp->textureTransferBias->getValue().x;
      scale = tp->textureTransferScale->getValue().x;
    }

    // buildNormalizedData is not iterating over the entire volume
    volume_max_x = H3DMin( volume_max_x + 1, (int)w );
    volume_max_y = H3DMin( volume_max_y + 1, (int)h );
    volume_max_z = H3DMin( volume_max_z + 1, (int)d );
 
    if( image->pixelType() == Image::LUMINANCE ) {
      Image::PixelComponentType type = image->pixelComponentType();
      if( type ==Image::UNSIGNED ) {
        switch( image->bitsPerPixel() ) {
        case 8:  
          buildNormalizedData< unsigned char >( data_matrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias ); break;
        case 16:
          buildNormalizedData< unsigned short >( data_matrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias ); break;
        case 32: 
          buildNormalizedData< unsigned int >( data_matrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias ); break;
        default: 
          Console(4) << "Warning: MarchingCubesEXT::buildDataMatrix failed." << endl; 
        }
      } else if( type == Image::SIGNED ) {
        switch( image->bitsPerPixel() ) {
        case 8:  
          buildNormalizedData< char >( data_matrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias ); break;
        case 16:
          buildNormalizedData< short >( data_matrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias ); break;
        case 32: 
          buildNormalizedData< int >( data_matrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias ); break;
        default: 
          Console(4) << "Warning: MarchingCubes::buildDataMatrix failed." << endl; 
        }
      } else if( type == Image::RATIONAL ) {
        float *image_data = (float * )image->getImageData();
        
        for (int z = volume_min_z; z<volume_max_z; ++z) {
          for (int y = volume_min_y; y<volume_max_y; ++y) {
            for (int x = volume_min_x; x<volume_max_x; ++x) {
              unsigned int i = ( ( z * h + y ) * w + x );
              data_matrix[i] =   image_data[ i ] * scale + bias;
              if( data_matrix[i] < 0 ) data_matrix[i] = 0;
            }
          }
        }
      } else {
        Console(4) << "Warning: MarchingCubes::buildDataMatrix failed." << endl; 
      }
    } else {
      Console(4) << "Warning: MarchingCubes::buildDataMatrix failed." << endl; 
    }
  }
}


MarchingCubes::SFOctTree::~SFOctTree() {

    if(data_matrix) {
      delete [] data_matrix;
      data_matrix = NULL;
    }
}



void MarchingCubes::SFOctTree::buildDataMatrix( Image *image, TextureProperties * tp ) {
  if( image ) {
    unsigned int d = image->depth();
    unsigned int h = image->height();
    unsigned int w = image->width();

    unsigned int data_matrix_size = x_points * y_points * z_points;
    data_matrix= new float[ data_matrix_size ];
    updateDataMatrix( image, 0, 0, 0, w, h, d, tp );
  }
}
 
void MarchingCubes::SFOctTree::update() { 


  H3DFloat iso_value = static_cast<SFFloat*>(routes_in[0])->getValue(); 
  X3DTexture3DNode *texture =
    static_cast< SFTexture3DNode * >( routes_in[1])->getValue();

  H3D::Image *image = NULL;
  TextureProperties *texture_properties = NULL;

  if( texture ) {
    image = texture->image->getValue();
    texture_properties = texture->textureProperties->getValue();
  }

  if( !image || image->depth() == 0 || image->height() == 0 || image->width() == 0 ) {
    value = NULL;
    currently_used_image = NULL;
    return;
  }

  vector< H3DFloat > texture_coordinate_generation_parameter;
  MarchingCubes * mc = static_cast< MarchingCubes * >( getOwner() );
  if( TextureCoordinateGenerator * tcg = dynamic_cast< TextureCoordinateGenerator * >(mc->texCoord->getValue()) ) {
    vector< H3DFloat > parameter = tcg->parameter->getValue();
    if( tcg->mode->getValue() == "MATRIX" && parameter.size() >= 12 ) {
      texture_coordinate_generation_parameter.swap( parameter );
    }
  }

  if( function_count == 0 &&
      ( mc->voxelsAllowedToBeReinitialized->getValue() || currently_used_image != image ) &&
      ( !texture->image->imageChanged() || data_matrix == 0 ) ) {
    currently_used_image = image;

    if( data_matrix != NULL ) delete [] data_matrix;

    x_points = y_points = z_points = 0;

    Vec3f size( 1, 1, 1 );

    if( image ) {
      x_points = image->width();
      y_points = image->height();
      z_points = image->depth();
      // MarchingCubes can not be applied if any of the dimensions is
      // less than 2.
      if( x_points < 2 || y_points < 2 || z_points < 2 ) {
        value = NULL;
        return;
      }
      voxel_size = image->pixelSize();
      size = Vec3f( x_points * voxel_size.x,
                    y_points * voxel_size.y,
                    z_points * voxel_size.z );
      buildDataMatrix( image, texture_properties );
    }

#ifndef GRADIENTS_ON_THE_FLY
    gradients.resize( x_points * y_points * z_points );

    updateGradients( 0, x_points, 0, y_points, 0, z_points,
                     gradients );
#endif

    // create tree structure....
    value = new OctTreeNode;
    value->x_min = 0;
    value->x_max = x_points-1;
    value->y_min = 0;
    value->y_max = y_points-1;
    value->z_min = 0;
    value->z_max = z_points-1;

    value->subdivide( TREEDEPTH );

    updateMinMaxValues( value.get() );

    //TimeStamp t;

    if( mc->attrib->size() > 0 || mc->alwaysGenerateOriginalFVA->getValue() ) {
      updateMCTriangles( value.get(),
                         iso_value,
                         NULL,
                         texture_coordinate_generation_parameter );
      NodeVector attrib = mc->attrib->getValue();
      vector< unsigned int > start_index_per_attrib( attrib.size(), 0 );
      stack< OctTreeNode * > tree_nodes;
      tree_nodes.push( value.get() );
      map< unsigned int, unsigned int > v_index_to_coord_order_map;
      while( !tree_nodes.empty() ) {
        OctTreeNode * tree_node = tree_nodes.top();
        tree_nodes.pop();
        if( tree_node->isLeaf() ) {
          if( tree_node->vindex.size() > 0 ) {
            if( tree_node->float_vertex_attributes.size() != attrib.size() + 1 ) {
              tree_node->float_vertex_attributes.resize( attrib.size() + 1 );
            }
            tree_node->float_vertex_attributes[0].nr_components = 1;
            tree_node->float_vertex_attributes[0].data.resize( tree_node->vindex.size(), 1 );
            tree_node->float_vertex_attributes[0].attrib_name = original_mc_point_name;
            if( !attrib.empty() ) {
              for( unsigned int j = 0; j < attrib.size(); ++j ) {
                if( FloatVertexAttribute * fva = dynamic_cast< FloatVertexAttribute * >(attrib[j]) ) {
                  const vector< H3DFloat > &original_fva_values = fva->value->getValue();
                  unsigned int nr_components = fva->numComponents->getValue();
                  tree_node->float_vertex_attributes[j+1].nr_components = nr_components;
                  tree_node->float_vertex_attributes[j+1].data.resize( tree_node->vindex.size() * nr_components, 0 );
                  tree_node->float_vertex_attributes[j+1].attrib_name = fva->name->getValue();
                  for( unsigned int k = 0; k < tree_node->vindex.size(); ++k ) {
                    if( v_index_to_coord_order_map.find( tree_node->vindex[k] ) == v_index_to_coord_order_map.end() ) {
                      v_index_to_coord_order_map[tree_node->vindex[k]] = (unsigned int)v_index_to_coord_order_map.size();
                    }
                    vector< H3DFloat > fva_value( nr_components, 0 );
                    for( unsigned int l = 0; l < nr_components; ++l ) {
                      size_t fva_index = v_index_to_coord_order_map[tree_node->vindex[k]] * nr_components + l;
                      if( fva_index < original_fva_values.size() ) {
                        fva_value[l] = original_fva_values[fva_index];
                      }
                      tree_node->float_vertex_attributes[j+1].data[k * nr_components + l ] = fva_value[l];
                    }
                    if( tree_node->vindex_to_fva_value_map.find( tree_node->vindex[k] ) == tree_node->vindex_to_fva_value_map.end() || j >= tree_node->vindex_to_fva_value_map[tree_node->vindex[k]].size() ) {
                      tree_node->vindex_to_fva_value_map[tree_node->vindex[k]].push_back(fva_value);
                    }
                  }
                }
              }
            } else {
              for( unsigned int k = 0; k < tree_node->vindex.size(); ++k ) {
                vector< H3DFloat > fva_value;
                if( tree_node->vindex_to_fva_value_map.find( tree_node->vindex[k] ) == tree_node->vindex_to_fva_value_map.end() ) {
                  tree_node->vindex_to_fva_value_map[tree_node->vindex[k]] = vector< vector< H3DFloat > >();
                }
              }
            }
          }
        } else {
          for( unsigned int i = 0; i < 8; ++i ) {
            tree_nodes.push( tree_node->children[i] );
          }
        }
      }
      mc->trianglesBuilt->setValue( true, mc->id );
    } else {
      vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > data;

      updateMCTriangles( value.get(),
                         iso_value,
                         &data,
                         texture_coordinate_generation_parameter );

      update_thread->asynchronousCallbacks( data.begin(), data.end() );
      function_count += (unsigned int) data.size();
    }
    //Console(3) << function_count << endl;

    //Console(3) << TimeStamp() - t << endl;

  } else if( texture->image->imageChanged() ) {
      TimeStamp t;
      updateDataMatrix( image, 
                      texture->image->xMin(),
                      texture->image->yMin(), 
                      texture->image->zMin(), 
                      texture->image->xMax(),
                      texture->image->yMax(), 
                      texture->image->zMax(),
                      texture_properties );
#ifndef GRADIENTS_ON_THE_FLY
    updateGradients( texture->image->xMin(),  texture->image->xMax() + 1, 
                     texture->image->yMin(),  texture->image->yMax() + 1, 
                     texture->image->zMin(),  texture->image->zMax() + 1, 
                     gradients );
#endif    

    vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > data;

    updateMCTrianglesInVolume( value.get(),iso_value,
                              texture->image->xMin(),
                              texture->image->yMin(), 
                              texture->image->zMin(), 
                              texture->image->xMax(),
                              texture->image->yMax(), 
                              texture->image->zMax(),
                              &data,
                              texture_coordinate_generation_parameter );

    function_count += (unsigned int) data.size();
    update_thread->asynchronousCallbacks( data.begin(), data.end() );
  }
}

void MarchingCubes::SFOctTree::updateGradients( int x_min, int x_max, 
                                                int y_min, int y_max, 
                                                int z_min, int z_max, 
                                                vector< Vec3f > &_gradients ) {

  for( int z = z_min; z<z_max; ++z) {
    for( int y = y_min; y<y_max; ++y) {
      for( int x = x_min; x<x_max; ++x) {
        int index = ( z*y_points + y ) * x_points +x;
        Vec3f &gradient = _gradients[index];

        if (x==0)  {
          gradient.x=(getVoxelValue(x+1,y,z)-getVoxelValue(x,y,z))
            /voxel_size.x;          
        } else if (x==x_points-1) {
          gradient.x=(getVoxelValue(x,y,z)-getVoxelValue(x-1,y,z))
            /voxel_size.x;
        } else {
          gradient.x=(getVoxelValue(x+1,y,z)-getVoxelValue(x-1,y,z))
            /(2*voxel_size.x);
        }
        
        if (y==0)  {
          gradient.y=(getVoxelValue(x,y+1,z)-getVoxelValue(x,y,z))
            /voxel_size.y;
        } else if (y==y_points-1) {
          gradient.y=(getVoxelValue(x,y,z)-getVoxelValue(x,y-1,z))
            /voxel_size.y;
        } else {
          gradient.y=(getVoxelValue(x,y+1,z)-getVoxelValue(x,y-1,z))
            /(2*voxel_size.y);
        }
        
        if (z==0) {
          gradient.z=(getVoxelValue(x,y,z+1)-getVoxelValue(x,y,z))
            /voxel_size.z;
        } else if (z==z_points-1) {
          gradient.z=(getVoxelValue(x,y,z)-getVoxelValue(x,y,z-1))
            /voxel_size.z;
        } else{
          gradient.z=(getVoxelValue(x,y,z+1)-getVoxelValue(x,y,z-1))
            /(2*voxel_size.z);
        }
      }
    }
  }
}

void MarchingCubes::SFOctTree::updateMCTriangles( OctTreeNode *tree,
                                                  H3DFloat iso_value,
  vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > *update_data,
  vector< H3DFloat > &texture_coordinate_generator_data ) {

  if ( tree->isLeaf() ) {
    // this is a leaf node
    
    
    // if the isovalue is within this leaf generate the triangles for it.
    if ( ( iso_value <= tree->value_max &&
           iso_value >= tree->value_min ) ) {
      if( !update_data ) {
        // clean out the vertex list
        tree->vvertex.clear();
        tree->vnormal.clear();
        tree->vindex.clear();
        tree->float_vertex_attributes.clear();
        if( tree->cache != 0 ) {
          glDeleteLists( tree->cache, 1 );
          tree->cache = 0;
        }
        tree->rebuild_vbo = true;
        HAPI::Collision::AABBTree *triangle_collision_tree = NULL;
        vMarchingCubes( iso_value, 
                        tree->x_min, tree->x_max, 
                        tree->y_min, tree->y_max, 
                        tree->z_min, tree->z_max, 
                        tree->vvertex, tree->vnormal, tree->vindex,
                        &triangle_collision_tree,
                        texture_coordinate_generator_data.size() < 12 ? NULL : &MarchingCubesInternals::textureCoordinateFromMatrix,
                        texture_coordinate_generator_data.size() < 12 ? NULL : &texture_coordinate_generator_data );
        if( triangle_collision_tree ) {
          tree->bound_tree->left.reset( triangle_collision_tree );
        } else {
          tree->bound_tree->left.reset( NULL );
        }
      } else {
        UpdateData *data = new UpdateData;
        data->iso_value = iso_value;
        data->x_min = tree->x_min;
        data->y_min = tree->y_min;
        data->z_min = tree->z_min;
        data->x_max = tree->x_max;
        data->y_max = tree->y_max;
        data->z_max = tree->z_max;
        data->mc.reset( static_cast< MarchingCubes * >( getOwner() ) );
        data->oct_tree_leaf = tree;
        if( texture_coordinate_generator_data.size() < 12 ) {
          data->texture_coordinate_generator_func = NULL;
        } else {
          data->texture_coordinate_generator_func = &MarchingCubesInternals::textureCoordinateFromMatrix;
          data->texture_coordinate_generator_data = texture_coordinate_generator_data;
        }

        // make_pair does not work for VS 2010, so do it like this.
        pair< PeriodicThread::CallbackFunc, 
              UpdateData * > fd_pair( calculateNewTriangles, data ); 
        update_data->push_back( fd_pair );
      }
    } else {
      // clean out the vertex list
      tree->vvertex.clear();
      tree->vnormal.clear();
      tree->vindex.clear();
      tree->float_vertex_attributes.clear();
    }

  } else {
    for( int i=0; i < 8; ++i )
      updateMCTriangles( tree->children[i],
                         iso_value, 
                         update_data,
                         texture_coordinate_generator_data );
  }
}

void MarchingCubes::SFOctTree::updateMCTrianglesInVolume( 
                               OctTreeNode *tree,
                               H3DFloat iso_value, 
                               int volume_min_x,
                               int volume_min_y,
                               int volume_min_z,
                               int volume_max_x,
                               int volume_max_y,
                               int volume_max_z,
                               vector< pair< PeriodicThread::CallbackFunc, 
                               UpdateData * > > *update_data,
                               vector< H3DFloat > &texture_coordinate_generator_data ) {

  // Voxels that are within one voxel distance from the border of an
  // octtree will affect the cubes in that octtree. Hence the modification
  // of the input before checking if we can return.
  int tree_x_min = H3DMax( tree->x_min - 1, 0 );
  int tree_y_min = H3DMax( tree->y_min - 1, 0 );
  int tree_z_min = H3DMax( tree->z_min - 1, 0 );
  int tree_x_max = H3DMin( tree->x_max + 1, (int)x_points );
  int tree_y_max = H3DMin( tree->y_max + 1, (int)y_points );
  int tree_z_max = H3DMin( tree->z_max + 1, (int)z_points );
  if( (volume_max_x < tree_x_min && volume_min_x < tree_x_min ) ||
      (volume_max_x > tree_x_max && volume_min_x > tree_x_max ) ||
      (volume_max_y < tree_y_min && volume_min_y < tree_y_min ) ||
      (volume_max_y > tree_y_max && volume_min_y > tree_y_max ) ||
      (volume_max_z < tree_z_min && volume_min_z < tree_z_min ) ||
      (volume_max_z > tree_z_max && volume_min_z > tree_z_max ) ) {
    // updated volume does not intersect the volume this subtree occupies
    // so do nothing
    return;
  }

  if ( tree->isLeaf() ) {
    // this is a leaf node
    updateMCTriangles( tree, iso_value, 
                       update_data,
                       texture_coordinate_generator_data );
  } else {
    for( int i=0; i < 8; ++i )
      updateMCTrianglesInVolume( tree->children[i],
                                 iso_value, 
                                 volume_min_x,
                                 volume_min_y,
                                 volume_min_z,
                                 volume_max_x,
                                 volume_max_y,
                                 volume_max_z,
                                 update_data,
                                 texture_coordinate_generator_data );
  }
}



void MarchingCubes::SFOctTree::vMarchingCube( H3DFloat iso_value,
                                              unsigned int iX,
                                              unsigned int iY,
                                              unsigned int iZ, 
                                              vector< Vec3f > &vertices,
                                              vector< Vec3f > &normals,
                                              vector< unsigned int > &indices ) {
  int iCorner, iVertex,  iEdge, iTriangle, iFlagIndex, iEdgeFlags;
  H3DFloat fOffset, afCubeValue[8];
  Vec3f afCubeGrad[8];

  //Find which vertices are inside of the surface and which are outside
  iFlagIndex = 0;
  afCubeValue[0] = getVoxelValue(iX + 0,iY + 0,iZ + 0 );
  if (afCubeValue[0] <= iso_value ) iFlagIndex |= 1<<0;
  afCubeValue[1] = getVoxelValue(iX + 1,iY + 0,iZ + 0 );
  if (afCubeValue[1] <= iso_value ) iFlagIndex |= 1<<1;
  afCubeValue[2] = getVoxelValue(iX + 1,iY + 1,iZ + 0 );
  if (afCubeValue[2] <= iso_value ) iFlagIndex |= 1<<2;
  afCubeValue[3] = getVoxelValue(iX + 0,iY + 1,iZ + 0 );
  if (afCubeValue[3] <= iso_value ) iFlagIndex |= 1<<3;
  afCubeValue[4] = getVoxelValue(iX + 0,iY + 0,iZ + 1 );
  if (afCubeValue[4] <= iso_value ) iFlagIndex |= 1<<4;
  afCubeValue[5] = getVoxelValue(iX + 1,iY + 0,iZ + 1 );
  if (afCubeValue[5] <= iso_value ) iFlagIndex |= 1<<5;
  afCubeValue[6] = getVoxelValue(iX + 1,iY + 1,iZ + 1 );
  if (afCubeValue[6] <= iso_value ) iFlagIndex |= 1<<6;
  afCubeValue[7] = getVoxelValue(iX + 0,iY + 1,iZ + 1 );
  if (afCubeValue[7] <= iso_value ) iFlagIndex |= 1<<7;


  //Find which edges are intersected by the surface
  iEdgeFlags = aiCubeEdgeFlags[iFlagIndex];         

  // If any edge is intersected by surface.
  if ( iEdgeFlags ) {
    Vec3f asEdgeVertex[12];
    Vec3f asEdgeNorm[12];

    // get the gradients for each corner
    for(iVertex = 0; iVertex < 8; ++iVertex) {
      afCubeGrad[iVertex] = -getGradient( iX + a2iVertexOffset[iVertex][0],
        iY + a2iVertexOffset[iVertex][1], 
        iZ + a2iVertexOffset[iVertex][2]); 
    }

    // Find the point of intersection of the surface with each edge
    // Then find the normal to the surface at those points using
    // gradients and interpolation of corner values.
    for(iEdge = 0; iEdge < 12; ++iEdge) {

      //if there is an intersection on this edge
      if(iEdgeFlags & (1<<iEdge)) {

        int edge_p0 = a2iEdgeConnection[iEdge][0];
        int edge_p1 = a2iEdgeConnection[iEdge][1];

        fOffset = fGetOffset(afCubeValue[ edge_p0 ], 
          afCubeValue[ edge_p1 ], iso_value);

        // the intersection point on the edge.
        asEdgeVertex[iEdge] = 
          Vec3f( ( iX + 
          ( a2fVertexOffset[ edge_p0 ][0]  +  
          fOffset * a2fEdgeDirection[iEdge][0])  - 0.5f * (x_points-1) ) * 
          voxel_size.x ,
          ( iY + 
          (a2fVertexOffset[ edge_p0 ][1]  +  
          fOffset * a2fEdgeDirection[iEdge][1]) - 0.5f * (y_points-1) ) 
          * voxel_size.y ,
          ( iZ + 
          (a2fVertexOffset[ edge_p0 ][2]  +  
          fOffset * a2fEdgeDirection[iEdge][2]) - 0.5f * (z_points-1) ) 
          * voxel_size.z);

        // calculate by linear interpolation the intersection point
        // gradient (x,y,z)
        // Get the normalised gradient for the intersection point
        asEdgeNorm[iEdge] = 
          afCubeGrad[ edge_p0 ] + 
          fOffset*( afCubeGrad[ edge_p1 ] - afCubeGrad[ edge_p0 ] );
        asEdgeNorm[iEdge].normalizeSafe();
      }
    }

    // Draw the triangles that were found.
    // There can be up to five per cube
    for(iTriangle = 0; iTriangle < 5; ++iTriangle) {
      if(a2iTriangleConnectionTable[iFlagIndex][3*iTriangle] < 0) {
        break;
      }
      for(iCorner = 0; iCorner < 3; ++iCorner) {
        iVertex = 
          a2iTriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];
        vertices.push_back(asEdgeVertex[iVertex]);
        normals.push_back(asEdgeNorm[iVertex] );
        indices.push_back(cubeIndex2VertexIndex( iX, iY, iZ, iVertex ) );
      }
    }
  }
}


// vMarchingCubes iterates over the (entire) part of the dataset that is
// inside the bounding box.
void MarchingCubes::SFOctTree::vMarchingCubes( H3DFloat iso_value, 
                                               unsigned int x_min,
                                               unsigned int x_max, 
                                               unsigned int y_min,
                                               unsigned int y_max, 
                                               unsigned int z_min,
                                               unsigned int z_max, 
                                               vector< Vec3f > &vertices,
                                               vector< Vec3f > &normals,
                                               vector< unsigned int > &indices,
                                               HAPI::Collision::AABBTree **aabb_tree,
                                               HAPI::Vec3 (*texture_coordinate_generator_func)( const Vec3f &c, void *data ),
                                               void * tex_coord_generator_data ) {
  for(unsigned int iZ = z_min; iZ <= (z_max<z_points-1?z_max:z_max-1); ++iZ) {
    for(unsigned int iY = y_min; iY <= (y_max<y_points-1?y_max:y_max-1); ++iY){
      for( unsigned int iX = x_min; iX <= (x_max<x_points-1?x_max:x_max-1); ++iX) {
        vMarchingCube( iso_value, iX, iY, iZ, vertices, normals, indices );
      }
    }
  }

  if( !vertices.empty() ) {
    vector< HAPI::Collision::Triangle > triangles( vertices.size() / 3 );
    for( unsigned int i = 0; i < vertices.size(); i+=3 ) {
      HAPI::Vec3 ta, tb, tc;
      if( texture_coordinate_generator_func ) {
        ta = texture_coordinate_generator_func( vertices[i], tex_coord_generator_data );
        tb = texture_coordinate_generator_func( vertices[i+1], tex_coord_generator_data );
        tc = texture_coordinate_generator_func( vertices[i+2], tex_coord_generator_data );
      }
      triangles[i/3] = HAPI::Collision::Triangle( vertices[i], vertices[i+1], vertices[i+2], ta, tb, tc );
    }
    *aabb_tree = new HAPI::Collision::AABBTree( triangles, 100 );
  }
}

void MarchingCubes::OctTreeNode::render( NormalRenderMode normal_render_mode, RenderMode render_mode, const map< string, GLint > &attribute_location_map ) {
  if( render_mode == DISPLAY_LISTS && cache != 0 ) {
    glCallList( cache );
  } else if ( isLeaf() ) {
    if( render_mode == DISPLAY_LISTS ) {
      cache = glGenLists(1);
      glNewList( cache, GL_COMPILE_AND_EXECUTE );
    } else if( cache ) {
      glDeleteLists( cache, 1 );
      cache = 0; 
    }
    if ( vvertex.size() > 0 ) {
      std::vector< Vec3f > *normals = &vnormal;
      bool delete_normals = false;
      if( render_mode == VERTEX_BUFFER_OBJECTS ) {
        if( rebuild_vbo ) {
          delete_normals = MarchingCubesInternals::generateNormals( normal_render_mode, vvertex, vnormal, &normals );
          rebuild_vbo = false;
          if ( !vbo_id ){
            vbo_id = new GLuint[2];
            glGenBuffersARB ( 2, vbo_id );
          }
          glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo_id[0] );
          glBufferDataARB( GL_ARRAY_BUFFER_ARB,
                           vvertex.size() * 3 * sizeof(GLfloat),
                           &(*(vvertex.begin())), GL_STATIC_DRAW_ARB );
          glEnableClientState(GL_VERTEX_ARRAY);
          glVertexPointer(3, GL_FLOAT, 0, NULL );

          glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo_id[1] );
          glBufferDataARB( GL_ARRAY_BUFFER_ARB,
            normals->size() * 3 * sizeof(GLfloat),
            &(*(normals->begin())), GL_STATIC_DRAW_ARB );
          glEnableClientState(GL_NORMAL_ARRAY);
          glNormalPointer(GL_FLOAT, 0, NULL );
        } else {
          glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo_id[0] );
          glEnableClientState(GL_VERTEX_ARRAY);
          glVertexPointer(3, GL_FLOAT, 0, NULL );
          glBindBufferARB( GL_ARRAY_BUFFER_ARB, vbo_id[1] );
          glEnableClientState(GL_NORMAL_ARRAY);
          glNormalPointer(GL_FLOAT, 0, NULL );
        }
      } else {
        delete_normals = MarchingCubesInternals::generateNormals( normal_render_mode, vvertex, vnormal, &normals );
        glVertexPointer(3, GL_FLOAT, 0, &(*(vvertex.begin())) );
        glNormalPointer(GL_FLOAT, 0, &(*((*normals).begin())) );
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
      }


      if( float_vertex_attributes.empty() ) {
        for( map< string, GLint >::const_iterator i = attribute_location_map.begin(); i != attribute_location_map.end(); ++i ) {
          glDisableVertexAttribArrayARB ( (*i).second );
          glVertexAttrib4f( (*i).second, 0.0, 0.0, 0.0, 1.0 );
        }
      } else {
        for( unsigned int j = 0; j < float_vertex_attributes.size(); ++j ) {
          map< string, GLint >::const_iterator found = attribute_location_map.find( float_vertex_attributes[j].attrib_name );
          if( found != attribute_location_map.end() ) {
            float_vertex_attributes[j].render( (*found).second, render_mode );
          }
        }
      }


      glDrawArrays(GL_TRIANGLES, 0, (GLsizei) vvertex.size());
      for( unsigned int j = 0; j < float_vertex_attributes.size(); ++j ) {
        map< string, GLint >::const_iterator found = attribute_location_map.find( float_vertex_attributes[j].attrib_name );
        if( found != attribute_location_map.end() ) {
          float_vertex_attributes[j].disable( (*found).second );
        }
      }
      glDisableClientState(GL_VERTEX_ARRAY);
      glDisableClientState(GL_NORMAL_ARRAY);
      glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );
      if( delete_normals ) {
        delete normals;
      }
    }
    if( cache != 0 )
      glEndList();
  } else {
    for( int i=0; i<8; ++i )
      children[i]->render( normal_render_mode, render_mode, attribute_location_map );
  }
}

void MarchingCubes::OctTreeNode::subdivide( int tree_depth ) {
  bound_tree.reset( new AABBOctTreeBBTreeLeaf );

  if( tree_depth != 0 ) {
    int half_x = (int)floor(float(x_max-x_min)/2);
    int half_y = (int)floor(float(y_max-y_min)/2);
    int half_z = (int)floor(float(z_max-z_min)/2);

    if( half_x == 0 && half_y == 0 && half_z == 0 )
      return;

    bound_tree->children.resize( 8, NULL );
    for( int i=0; i<2; ++i ) {
      for( int j=0; j<2; ++j ) {
        for( int k=0; k<2; ++k ) {
          int index = i * 4 + j*2 + k;
          children[index] = new OctTreeNode;
          children[index]->parent = this;

          // x
          if( half_x == 0 ) {
            children[index]->x_min = x_min;
            children[index]->x_max = x_max;
          } else if ( i==0 ) {
            children[index]->x_min = x_min;
            children[index]->x_max = x_max - half_x - 1;
          } else {
            children[index]->x_min = x_max - half_x;
            children[index]->x_max = x_max;
          }

          // y
          if( half_y == 0 ) {
            children[index]->y_min = y_min;
            children[index]->y_max = y_max;
          } else if ( j==0 ) {
            children[index]->y_min = y_min;
            children[index]->y_max = y_max - half_y - 1;
          } else {
            children[index]->y_min = y_max - half_y;
            children[index]->y_max = y_max;
          }

          // z
          if( half_z == 0 ) {
            children[index]->z_min = z_min;
            children[index]->z_max = z_max;
          } else if ( k==0 ) {
            children[index]->z_min = z_min;
            children[index]->z_max = z_max - half_z - 1;
          } else {
            children[index]->z_min = z_max - half_z;
            children[index]->z_max = z_max;
          }

          children[index]->subdivide( tree_depth-1 );
          bound_tree->children.set( index, children[index]->bound_tree.get() );

          if ( children[index]->value_min < value_min )
            value_min = children[index]->value_min;
          if ( children[index]->value_max > value_max )
            value_max = children[index]->value_max;
        }
      }
    }
  }
}

MarchingCubes::OctTreeNode::~OctTreeNode(){

  for( int i= 0; i < 8 ; ++i ) {
    if( children[i] ) {
      delete children[i];
    }
    children[i] = NULL;
  }
  if( vbo_id ) {
    if( vbo_id ) {
      glDeleteBuffersARB( 2, vbo_id );
      delete [] vbo_id;
      vbo_id = NULL;
    }
  }
  clearFVA();
}

void MarchingCubes::SFBound::update() {
  X3DTexture3DNode *tex = 
    static_cast<SFTexture3DNode*>(routes_in[0])->getValue();
  H3D::Image *image = NULL;
  if( tex ) image = tex->image->getValue();
  BoxBound *bb; 
  BoxBound *b =  dynamic_cast< BoxBound * >( value.get() );
  if( b ) bb = b;
  else bb = new BoxBound;
  
  if( image ) {
    Vec3f voxel_size = image->pixelSize();
    bb->size->setValue( Vec3f( voxel_size.x * image->width(),
                               voxel_size.y * image->height(),
                               voxel_size.z * image->depth() ) );
  } else {
    bb->size->setValue( Vec3f( 0, 0, 0 ) );
  }
  value = bb;
}

// fGetOffset finds the approximate point of intersection of the surface
// between two points with the values fValue1 and fValue2
float MarchingCubes::SFOctTree::fGetOffset( float fValue1, 
                                            float fValue2, 
                                            float fValueDesired) {
  float fDelta = fValue2 - fValue1;

  if(fDelta == 0.0) {
    return 0.5;
  }
  return (fValueDesired - fValue1)/fDelta;
}


// For any edge, if one vertex is inside of the surface and the other is
// outside of the surface then the edge intersects the surface
// For each of the 8 vertices of the cube can be two possible states :
// either inside or outside of the surface
// For any cube the are 2^8=256 possible sets of vertex states
// This table lists the edges intersected by the surface for all 256 possible
// vertex states There are 12 edges. For each entry in the table, if edge #n is
// intersected, then bit #n is set to 1
const int MarchingCubes::aiCubeEdgeFlags[256]=
  {
    0x000, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00, 
    0x190, 0x099, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90, 
    0x230, 0x339, 0x033, 0x13a, 0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30, 
    0x3a0, 0x2a9, 0x1a3, 0x0aa, 0x7a6, 0x6af, 0x5a5, 0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 
    0x460, 0x569, 0x663, 0x76a, 0x066, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60, 
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0x0ff, 0x3f5, 0x2fc, 0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0, 
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x055, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950, 
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0x0cc, 0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0, 
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0x0cc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0, 
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x055, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650, 
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0x0ff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0, 
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f, 0x066, 0x76a, 0x663, 0x569, 0x460, 
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0x0aa, 0x1a3, 0x2a9, 0x3a0, 
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x033, 0x339, 0x230, 
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x099, 0x190, 
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x000
  };



// For each of the possible vertex states listed in aiCubeEdgeFlags there is
// a specific triangulation of the edge intersection points. 
// a2iTriangleConnectionTable lists all of them in the form of 0-5 edge triples
// with the list terminated by the invalid value -1. For example:
// a2iTriangleConnectionTable[3] list the 2 triangles formed when corner[0] 
// and corner[1] are inside of the surface, but the rest of the cube is not.
const int MarchingCubes::a2iTriangleConnectionTable[256][16] =
  {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  };

const int MarchingCubes::edgeVerticesConnection[12][4] = {
  {0,0,-1,-1},
  {1,0,0,-1},
  {0,0,0,-1},
  {1,-1,0,-1},
  {0,0,-1,0},
  {1,0,0,0},
  {0,0,0,0},
  {1,-1,0,0},
  {2,-1,-1,0},
  {2,0,-1,0},
  {2,0,0,0},
  {2,-1,0,0}
};

const int MarchingCubes::edgeVerticesVoxelSizeDiff[12][3] = {
  {1,2,2},
  {2,1,2},
  {1,2,2},
  {2,1,2},
  {1,2,2},
  {2,1,2},
  {1,2,2},
  {2,1,2},
  {2,2,1},
  {2,2,1},
  {2,2,1},
  {2,2,1}
};

PeriodicThread::CallbackCode 
MarchingCubes::SFOctTree::calculateNewTriangles( void * d ) {
  UpdateData *data = static_cast< UpdateData * >( d );

  vector< Vec3f > *vertices = new vector< Vec3f >();
  vector< Vec3f > *normals = new vector< Vec3f >(); 
  vector< unsigned int > *vindices = new vector< unsigned int >();

  vertices->reserve( data->oct_tree_leaf->vvertex.size() );
  normals->reserve( data->oct_tree_leaf->vnormal.size() );
  HAPI::Collision::AABBTree * triangle_collision_tree = NULL;

  data->mc->octTree->vMarchingCubes( data->iso_value,
                                     data->x_min, data->x_max, 
                                     data->y_min, data->y_max, 
                                     data->z_min, data->z_max, 
                                     *vertices, *normals, *vindices,
                                     &triangle_collision_tree,
                                     data->texture_coordinate_generator_func,
                                     &data->texture_coordinate_generator_data );

  TransferData *transfer_data = new TransferData;

  data->oct_tree_leaf->calculateNewFVA( *vindices, transfer_data->float_vertex_attributes );

  transfer_data->vertices.reset( vertices );
  transfer_data->normals.reset( normals );
  transfer_data->vindices.reset( vindices );
  transfer_data->mc.reset( data->mc.get() );
  transfer_data->oct_tree_leaf = data->oct_tree_leaf;
  transfer_data->triangle_collision_tree.reset( triangle_collision_tree );

  delete data;

  Scene::addCallback( transferUpdatedTriangles, transfer_data );
  return PeriodicThread::CALLBACK_DONE;
}

Scene::CallbackCode
  MarchingCubes::SFOctTree::transferUpdatedTriangles( void *d ) {
  TransferData *data = static_cast< TransferData * >( d );

  data->oct_tree_leaf->vvertex.swap( *data->vertices );
  data->oct_tree_leaf->vnormal.swap( *data->normals );
  data->oct_tree_leaf->vindex.swap( *data->vindices );
  if( data->triangle_collision_tree.get() ) {
    data->oct_tree_leaf->bound_tree->left.reset( data->triangle_collision_tree.get() );
  } else {
    data->oct_tree_leaf->bound_tree->left.reset( NULL );
  }

  if( data->float_vertex_attributes.empty() ) {
    data->oct_tree_leaf->clearFVA();
  } else {
    for( unsigned int i = 0; i < data->float_vertex_attributes.size(); ++i ) {
      if( i < data->oct_tree_leaf->float_vertex_attributes.size() ) {
        data->oct_tree_leaf->float_vertex_attributes[i].data.swap( data->float_vertex_attributes[i] );
        data->oct_tree_leaf->float_vertex_attributes[i].needs_update = true;
      }
    }
  }

  if( data->oct_tree_leaf->cache != 0 ) {
    glDeleteLists( data->oct_tree_leaf->cache, 1 );
    data->oct_tree_leaf->cache = 0; 
  }
  data->oct_tree_leaf->rebuild_vbo = true;
  --(data->mc->octTree->function_count);
  if( data->mc->octTree->function_count == 0) {
    data->mc->displayList->breakCache();
    data->mc->trianglesBuilt->setValue( true, data->mc->id );
  }

  delete data;

  return Scene::CALLBACK_DONE;
}

void MarchingCubes::OctTreeNode::breakCache() {
  if ( isLeaf() ) {
    if( cache != 0 ) {
      glDeleteLists( cache, 1 );
      cache = 0; 
    }
    rebuild_vbo = true;
  } else {
    for( int i=0; i < 8; ++i )
      children[i]->breakCache();
  }
}

int MarchingCubes::OctTreeNode::nrTriangles() {
  if( isLeaf() ) {
    return (int)vvertex.size() / 3;
  } else {
    int triangles = 0;
    for( int i=0; i<8; ++i ) {
      triangles += children[i]->nrTriangles();
    }
    return triangles;
  }
}

void MarchingCubes::OctTreeNode::calculateNewFVA( const vector< unsigned int > &_vindices, vector< vector< H3DFloat > > &fva ) {
  if( !vindex_to_fva_value_map.empty() ) {
    fva.resize( (*vindex_to_fva_value_map.begin()).second.size() + 1 );
    fva[0].reserve( _vindices.size() );
    for( unsigned int i = 0; i < (*vindex_to_fva_value_map.begin()).second.size(); ++i ) {
      fva[i+1].reserve( (*vindex_to_fva_value_map.begin()).second[i].size() * _vindices.size() );
    }
    for( unsigned int i = 0; i < _vindices.size(); ++i ) {
      map< unsigned int, vector< vector< H3DFloat > > >::iterator it_low = vindex_to_fva_value_map.lower_bound( _vindices[i] );
      H3DFloat original = 1;
      if( it_low == vindex_to_fva_value_map.end() ) {
        --it_low;
        original = 0;
      } else if( (*it_low).first != _vindices[i] )
        original = 0;
      fva[0].push_back( original );
      for( unsigned int k = 0; k < (*it_low).second.size(); ++k ) {
        fva[k+1].insert( fva[k+1].end(), (*it_low).second[k].begin(), (*it_low).second[k].end() );
      }
    }
  }
}

void MarchingCubes::OctTreeNode::clearFVA() {
  for( unsigned int i = 0; i < float_vertex_attributes.size(); ++i ) {
    if( float_vertex_attributes[i].vbo_id ) {
      glDeleteBuffersARB( 1, float_vertex_attributes[i].vbo_id );
      delete [] float_vertex_attributes[i].vbo_id;
      float_vertex_attributes[i].vbo_id = NULL;
    }
  }
  float_vertex_attributes.clear();
}

void MarchingCubes::SFNormalRenderMode::onValueChange( const std::string &new_value ) {
  if( new_value == "GRADIENT" ) {
    normal_render_mode = OctTreeNode::GRADIENT;
  } else if( new_value == "NORMALS_PER_FACE" ) {
    normal_render_mode = OctTreeNode::NORMALS_PER_FACE;
  }
}

void MarchingCubes::SFOctTree::writeAsITS( const string &url ) {
  getValue();
  if( url != "" ) {
    ofstream output_file( url.c_str() );
    if( output_file.is_open() ) {
      output_file << "<?xml version='1.0' encoding='utf-8'?>" << endl
                  << "<X3D profile='Full' version='3.2'>" << endl
                  << "  <head>" << endl
                  << "    <meta name='description' content='MarchingCubes triangles exported as IndexedTriangleSet.'/>" << endl
                  << "    <meta name='author' content='SenseGraphics AB'/>" << endl
                  << "  </head>" << endl
                  << "  <Scene>" << endl
                  << "    <Shape>" << endl
                  << "      <Appearance>" << endl
                  << "        <Material diffuseColor='1 1 1' />" << endl
                  << "      </Appearance>" << endl
                  << "      <IndexedTriangleSet normalPerVertex='true' index='";
      stack< OctTreeNode * > tree_nodes;
      map< unsigned int, unsigned int > index_map;
      unsigned int current_index = 0;
      tree_nodes.push( value.get() );
      stringstream coords, normals;
      while( !tree_nodes.empty() ) {
        OctTreeNode * tree_node = tree_nodes.top();
        tree_nodes.pop();
        if( tree_node->isLeaf() ) {
          for( unsigned int i = 0; i < tree_node->vindex.size(); ++i ) {
            if( !index_map.empty() )
              output_file << " ";
            if( index_map.find( tree_node->vindex[i] ) == index_map.end() ) {
              if( !index_map.empty() ) {
                coords << " ";
                normals << " ";
              }
              index_map[tree_node->vindex[i]] = current_index;
              ++current_index;
              coords << tree_node->vvertex[i];
              normals << tree_node->vnormal[i];
            }
            output_file << index_map[tree_node->vindex[i]];
          }
        } else {
          for( unsigned int i = 0; i < 8; ++i ) {
            tree_nodes.push( tree_node->children[i] );
          }
        }
      }

      output_file << "' >" << endl
                  << "        <Coordinate point='" << coords.str() << "' />" << endl
                  << "        <Normal vector='" << normals.str() << "' />" << endl
                  << "      </IndexedTriangleSet>" << endl
                  << "    </Shape>" << endl
                  << "  </Scene>" << endl
                  << "</X3D>" << endl;
      output_file.close();
    } else {
       Console(LogLevel::Warning) << "Could not open file " << url << ". Writing MarchingCubes as IndexedTriangleSet won't be done." << endl;
    }
  }
}

void MarchingCubes::startTexGen( 
  X3DTextureCoordinateNode *tex_coord_node ) {
    if( !tex_coord_node ) {

      Matrix4f m = getDefaultTexGenMatrix();

      H3DFloat *sparams = m[0];
      H3DFloat *tparams = m[1];
      H3DFloat *rparams = m[2];

      MultiTexture *mt = 
        dynamic_cast< MultiTexture * >( X3DTextureNode::getActiveTexture() );
      if( mt ) {
        size_t texture_units = mt->texture->size();
        for( size_t i = 0; i < texture_units; ++i ) {
          glActiveTexture( GL_TEXTURE0_ARB + (unsigned int) i );
          glTexGend( GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR );
          glTexGend( GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR );
          glTexGend( GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR );
          glTexGenfv( GL_S, GL_OBJECT_PLANE, sparams );
          glTexGenfv( GL_T, GL_OBJECT_PLANE, tparams );
          glTexGenfv( GL_R, GL_OBJECT_PLANE, rparams );
          glEnable( GL_TEXTURE_GEN_S );
          glEnable( GL_TEXTURE_GEN_T );
          glEnable( GL_TEXTURE_GEN_R );
        }
      } else {
        glTexGend( GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR );
        glTexGend( GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR );
        glTexGend( GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR );
        glTexGenfv( GL_S, GL_OBJECT_PLANE, sparams );
        glTexGenfv( GL_T, GL_OBJECT_PLANE, tparams );
        glTexGenfv( GL_R, GL_OBJECT_PLANE, rparams );
        glEnable( GL_TEXTURE_GEN_S );
        glEnable( GL_TEXTURE_GEN_T );
        glEnable( GL_TEXTURE_GEN_R );
      }
    } else {
      tex_coord_node->startTexGenForActiveTexture();
    }
}

void MarchingCubes::stopTexGen( 
  X3DTextureCoordinateNode *tex_coord_node ) {
    if( !tex_coord_node ) {
      MultiTexture *mt = 
        dynamic_cast< MultiTexture * >( X3DTextureNode::getActiveTexture() );
      if( mt ) {
        size_t texture_units = mt->texture->size();
        for( size_t i = 0; i < texture_units; ++i ) {
          glActiveTexture( GL_TEXTURE0_ARB + (unsigned int) i );
          glDisable( GL_TEXTURE_GEN_S );
          glDisable( GL_TEXTURE_GEN_T );
          glDisable( GL_TEXTURE_GEN_R );
        }
      } else {
        glDisable( GL_TEXTURE_GEN_S );
        glDisable( GL_TEXTURE_GEN_T );
        glDisable( GL_TEXTURE_GEN_R );
      }
    } else {
      tex_coord_node->stopTexGenForActiveTexture();
    }
}

Matrix4f MarchingCubes::getDefaultTexGenMatrix() {

  BoxBound *box_bound = 
    dynamic_cast< BoxBound * >( bound->getValue() );
  if( box_bound ) {
    const Vec3f &center = box_bound->center->getValue();
    const Vec3f &size = box_bound->size->getValue();

    Matrix4f m(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    H3DFloat *sparams = m[0];
    H3DFloat *tparams = m[1];
    H3DFloat *rparams = m[2];

    H3DFloat size_vec[]   = { size.x, size.y, size.z };
    H3DFloat center_vec[] = { center.x, center.y, center.z };

    // these variables are set to an index representing 
    // sides of the bounding box. 0 is the x-axis, 1 the y-axis
    // and 2 the z-axis.
    int largest_side, middle_side, smallest_side;

    if( size.x >= size.y ) {
      if( size.x >= size.z ) {
        largest_side = 0; 
        if( size.y >= size.z ) {
          // size.x >= size.y >= size.z
          middle_side   = 1;
          smallest_side = 2;
        } else { 
          // size.x >= size.z > size.y
          middle_side   = 2;
          smallest_side = 1;
        }
      } else {
        // size.z > size.x >= size.y
        largest_side  = 2; 
        middle_side   = 0;
        smallest_side = 1;
      }
    } else {
      if( size.z >= size.y ) {
        // size.z >= size.y > size.x
        largest_side  = 2; 
        middle_side   = 1;
        smallest_side = 0;
      } else if( size.x >= size.z ) {
        // size.y > size.x >=size.z
        largest_side  = 1; 
        middle_side   = 0;
        smallest_side = 2;
      } else {
        // size.y > size.z > size.x
        largest_side  = 1; 
        middle_side   = 2;
        smallest_side = 0;
      }
    }

    H3DFloat largest_length = size_vec[ largest_side ];
    if( H3DAbs( largest_length ) > Constants::f_epsilon ) {
      // parameters for the s coordinate
      H3DFloat length_inv = 1/largest_length;
      sparams[ largest_side ] = length_inv;
      sparams[3] = 0.5f - center_vec[ largest_side ] / largest_length;

      // parameters for the t coordinate
      tparams[ middle_side ] = length_inv;
      H3DFloat tcenter = size_vec[ middle_side ] / (2*largest_length);
      tparams[3] = tcenter - center_vec[ middle_side ] / largest_length;

      // parameters for the r coordinate
      rparams[ smallest_side ] = -length_inv;
      H3DFloat rcenter = size_vec[ smallest_side ] / (2*largest_length);
      rparams[3] = rcenter + center_vec[ smallest_side ] / largest_length;
    } else {
      sparams[3] = 0.5;
      tparams[3] = size_vec[ middle_side ] / (2*largest_length);
      rparams[3] = size_vec[ smallest_side ] / (2*largest_length);
    }
    return m;
  } else {
    stringstream s;
    s << "Could not calculate default  texture coordinate generation in IndexedFaceSet. "
      << "Requires bound object of BoxBound type. ";
    return Matrix4f();
  }
}

void MarchingCubes::SFBoundTree::update() {
  MarchingCubes * mc = static_cast< MarchingCubes * >(getOwner());
  OctTreeNode * tree_node = mc->octTree->getValue();
  if( tree_node && tree_node->bound_tree.get() ) {
    value.reset( tree_node->bound_tree.get() );
  } else
    value.reset( new HAPI::Collision::AABBTree() );
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::getConstraints( const HAPI::Vec3 &point,
                                                           HAPI::Constraints &constraints,
                                                           HAPI::Collision::FaceType face,
                                                           HAPI::HAPIFloat radius ) {
  if( children.empty() ) {
    if( left.get() )
      left->getConstraints( point, constraints, face, radius );
    if( right.get() )
      right->getConstraints( point, constraints, face, radius );
  } else {
    if( radius > 0 ) {
      HAPI::Vec3 cp = bound->boundClosestPoint( point );
      if( (cp - point).lengthSqr() > radius * radius && !bound->insideBound(point) ) return;
    }

    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->getConstraints( point, constraints, face, radius );
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::getTrianglesWithinRadius( const HAPI::Vec3 &p,
                                                                     HAPI::HAPIFloat radius,
                                                                     std::vector< HAPI::Collision::Triangle > &triangles) {
  if( children.empty() ) {
    if( left.get() )
      left->getTrianglesWithinRadius( p, radius, triangles );
    if( right.get() )
      right->getTrianglesWithinRadius( p, radius, triangles );
  } else {
    HAPI::Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > radius * radius && !bound->insideBound(p) ) return;

    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->getTrianglesWithinRadius( p, radius, triangles );
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::getPrimitivesWithinRadius( const HAPI::Vec3 &p,
                                                                      HAPI::HAPIFloat radius,
                                                                      std::vector< HAPI::Collision::Triangle > &triangles,
                                                                      std::vector< HAPI::Collision::LineSegment > &lines,
                                                                      std::vector< HAPI::Collision::Point > &points ) {
  if( children.empty() ) {
    if( left.get() )
      left->getPrimitivesWithinRadius( p, radius, triangles, lines, points );
    if( right.get() )
      right->getPrimitivesWithinRadius( p, radius, triangles, lines, points );
  } else {
    HAPI::Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > radius * radius && !bound->insideBound(p) ) return;

    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->getPrimitivesWithinRadius( p, radius, triangles, lines, points );
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::getTrianglesIntersectedByMovingSphere(
                    HAPI::HAPIFloat radius,
                    HAPI::Vec3 from,
                    HAPI::Vec3 to,
                    std::vector< HAPI::Collision::Triangle > &triangles ) {
  if( children.empty() ) {
    if( left.get() )
      left->getTrianglesIntersectedByMovingSphere( radius, from, to, triangles );
    if( right.get() )
      right->getTrianglesIntersectedByMovingSphere( radius, from, to, triangles );
  } else {
    if( !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->getTrianglesIntersectedByMovingSphere( radius, from, to, triangles );
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::getPrimitivesIntersectedByMovingSphere(
                    HAPI::HAPIFloat radius,
                    HAPI::Vec3 from,
                    HAPI::Vec3 to,
                    std::vector< HAPI::Collision::Triangle > &triangles,
                    std::vector< HAPI::Collision::LineSegment > &lines,
                    std::vector< HAPI::Collision::Point > &points ) {
  if( children.empty() ) {
    if( left.get() )
      left->getPrimitivesIntersectedByMovingSphere( radius, from, to, triangles, lines, points );
    if( right.get() )
      right->getPrimitivesIntersectedByMovingSphere( radius, from, to, triangles, lines, points );
  } else {
    if( !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->getPrimitivesIntersectedByMovingSphere( radius, from, to, triangles, lines, points );
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::render() {
  if( children.empty() ) {
    if( left.get() )
      left->render();
    if( right.get() )
      right->render();
  } else {
    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->render();
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::renderBounds( int depth ) {
  if( children.empty() ) {
    if( left.get() )
      left->renderBounds(depth-1);
    if( right.get() )
      right->renderBounds(depth-1);
  } else {
    if( depth == 0 )
      bound->render();
    else
      for( unsigned int i = 0; i < children.size(); ++i ) {
        children[i]->renderBounds(depth-1);
      }
  }
}

bool MarchingCubes::AABBOctTreeBBTreeLeaf::lineIntersect( const HAPI::Vec3 &from, 
                                                          const HAPI::Vec3 &to,
                                                          HAPI::Collision::IntersectionInfo &result,
                                                          HAPI::Collision::FaceType face ) {
  bool overlap = false;
  if( children.empty() ) {
    if( left.get() )
      overlap |= left->lineIntersect( from, to, result, face );
    if( right.get() )
      overlap |= right->lineIntersect( from, to, result, face );
  } else {
    if( !bound->boundIntersect( from, to ) ) return false;

    for( unsigned int i = 0; i < children.size(); ++i ) {
      overlap |= children[i]->lineIntersect( from, to, result, face );
    }
  }
  return overlap;
}

bool MarchingCubes::AABBOctTreeBBTreeLeaf::movingSphereIntersect( HAPI::HAPIFloat radius,
                                                                  const HAPI::Vec3 &from,
                                                                  const HAPI::Vec3 &to ) {
  if( children.empty() ) {
    if( left.get() && left->movingSphereIntersect( radius, from, to ) )
      return true;
    if( right.get() && right->movingSphereIntersect( radius, from, to ) )
      return true;
  } else {
    if( bound->boundMovingSphereIntersect( radius, from, to ) ) {
      for( unsigned int i = 0; i < children.size(); ++i ) {
        if( children[i]->movingSphereIntersect( radius, from, to ) )
          return true;
      }
    }
  }
  return false;
}

bool MarchingCubes::AABBOctTreeBBTreeLeaf::movingSphereIntersect( HAPI::HAPIFloat radius,
                                                                  const HAPI::Vec3 &from,
                                                                  const HAPI::Vec3 &to,
                                                                  HAPI::Collision::IntersectionInfo &result ) {
  bool overlap = false;
  if( children.empty() ) {
    if( left.get() )
      overlap |= left->movingSphereIntersect( radius, from, to, result );
    if( right.get() )
      overlap |= right->movingSphereIntersect( radius, from, to, result );
  } else {
    if( !bound->boundMovingSphereIntersect( radius, from, to ) ) return false;

    for( unsigned int i = 0; i < children.size(); ++i ) {
      overlap |= children[i]->movingSphereIntersect( radius, from, to, result );
    }
  }
  return overlap;
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::closestPoint( const HAPI::Vec3 &p,
                                                         HAPI::Vec3 &closest_point,
                                                         HAPI::Vec3 &closest_normal,
                                                         HAPI::Vec3 &closest_tex_coord ) {
  if( children.empty() ) {
    if( left.get() && right.get() ) {
      vector< BinaryBoundTree * > left_right_children(2);
      if( left->insideBound( p ) ) {
        left_right_children[0] = left.get();
        left_right_children[1] = right.get();
      } else {
        left_right_children[0] = right.get();
        left_right_children[1] = left.get();
      }

      left_right_children[0]->closestPoint( p, closest_point, closest_normal, closest_tex_coord );
      HAPI::Vec3 cv = p - closest_point;
      HAPI::Vec3 v;
      if( !left_right_children[1]->isLeaf() ) {
        HAPI::Vec3 cp;
        cp = cp = left_right_children[1]->boundClosestPoint(p);
        v = (cp-p);
      }

      if( left_right_children[1]->isLeaf() || v * v < cv * cv ) {
        HAPI::Vec3 cp, cn, tc;
        left_right_children[1]->closestPoint( p, cp, cn, tc );
        HAPI::Vec3 v = p - cp;
        if( v * v < cv * cv ) {
          closest_point = cp;
          closest_normal = cn;
          closest_tex_coord = tc;
        }
      }
    } else {
      if (left.get()) {
        left->closestPoint( p, closest_point, closest_normal, closest_tex_coord  );
      } else if( right.get() ) {
        right->closestPoint( p, closest_point, closest_normal, closest_tex_coord );
      }
    }
  } else {
    // In our setup it is entirely possible that the tree has a child with no triangles at all, that is, there is no closest point. Therefore I must set the original values to something really high.
    // Of course if there are not points at all inside then the closest point value won't be valid anyways.. I think.
    HAPI::Collision::AABoxBound *box_bound = static_cast< HAPI::Collision::AABoxBound * >(bound.get());
    HAPI::Vec3 box_center = (box_bound->max + box_bound->min) / 2;
    HAPI::HAPIFloat box_max_dist = H3DMax( 2 * ( box_bound->max - box_bound->min ).lengthSqr(), ( box_center - p ).lengthSqr() );
    HAPI::Vec3 point_outside_box = box_center + H3DSqrt( box_max_dist ) * HAPI::Vec3( 1.0, 0.0, 0.0 ); //
    HAPI::HAPIFloat closest_dist = numeric_limits< HAPI::HAPIFloat >::max();
    bool have_closest = false;
    for( unsigned int i = 0; i < children.size(); ++i ) {
      bool do_closest_point = !children[i]->children.empty();
      if( do_closest_point || children[i]->left.get() || children[i]->right.get() ) {
        HAPI::Vec3 cp, cn, tc;
        if( do_closest_point ) {
          cp = children[i]->boundClosestPoint( p ); // In this particular node a bound always exits.
          HAPI::Vec3 v = p - cp;
          do_closest_point = ( v * v < closest_dist );
        } else
          do_closest_point = true;
      
        if( do_closest_point ) {
          cp = point_outside_box; // It is possible that there are not triangles in the child and as such the input won't change.
          children[i]->closestPoint( p, cp, cn, tc );
          HAPI::Vec3 v = p - cp;
          HAPI::HAPIFloat ld2 = v * v;
          if( ld2 < closest_dist ) {
            have_closest = true;
            closest_point = cp;
            closest_normal = cn;
            closest_tex_coord = tc;
            closest_dist = ld2;
          }
        }
      }
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::getAllTriangles( std::vector< HAPI::Collision::Triangle > &triangles ) {
  if( children.empty() ) {
    if( left.get() )
      left->getAllTriangles( triangles );
    if( right.get() )
      right->getAllTriangles( triangles );
  } else {
    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->getAllTriangles( triangles );
    }
  }
}

void MarchingCubes::AABBOctTreeBBTreeLeaf::getAllPrimitives( std::vector< HAPI::Collision::Triangle > &triangles,
                                                             std::vector< HAPI::Collision::LineSegment > &lines,
                                                             std::vector< HAPI::Collision::Point > &points ) {
  if( children.empty() ) {
    if( left.get() )
      left->getAllPrimitives( triangles, lines, points );
    if( right.get() )
      right->getAllPrimitives( triangles, lines, points );
  } else {
    for( unsigned int i = 0; i < children.size(); ++i ) {
      children[i]->getAllPrimitives( triangles, lines, points );
    }
  }
}

void MarchingCubes::FVAContainer::updateFVAVertexBufferObject() {
  if ( needs_update ) {
    needs_update = false;
    if( !vbo_id ) {
      vbo_id = new GLuint;
      glGenBuffersARB ( 1, vbo_id );
    }
    glBindBuffer ( GL_ARRAY_BUFFER, *vbo_id );
    glBufferData ( GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat), (GLvoid*)&(*data.begin()), GL_STATIC_DRAW );
  } else
    glBindBuffer ( GL_ARRAY_BUFFER, *vbo_id );
}

void MarchingCubes::FVAContainer::render( GLint attrib_index, const RenderMode &render_mode ) {
  if( !data.empty() ) {
    if( render_mode == VERTEX_BUFFER_OBJECTS ) {
      updateFVAVertexBufferObject();
      glEnableVertexAttribArrayARB ( attrib_index );
      glVertexAttribPointerARB ( attrib_index,
        nr_components,
        GL_FLOAT,
        GL_FALSE,
        0,
        0 );
    } else {
      if( GLEW_ARB_vertex_program && attrib_index >= 0 ) {
        glEnableVertexAttribArrayARB( attrib_index );
        glVertexAttribPointerARB( attrib_index,
          nr_components,
          GL_FLOAT,
          GL_FALSE,
          0,
          &(*data.begin() ) );
      }
    }
  }
}

void MarchingCubes::FVAContainer::disable( GLint attrib_index ) {
  if( !data.empty() )
    glDisableVertexAttribArrayARB ( attrib_index );
}
