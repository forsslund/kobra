//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//    Slice based rendering code Copyright 2003-2005, Karljohan Lundin
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
/// \file X3DVolumeNode.cpp
/// \brief CPP file for X3DVolumeNode, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <fstream>

#include <H3D/MedX3D/X3DVolumeNode.h>
#include <H3D/MedX3D/VolumeGradient.h>
#include <H3D/PixelTexture.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/ShaderPart.h>
#include <H3D/Appearance.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/X3DPointingDeviceSensorNode.h>
#include <H3D/ClipPlane.h>

#include <H3D/MedX3D/RayCaster.h>
#include <H3D/MedX3D/MultiVolumeRayCaster.h>
#include <H3D/MedX3D/SliceRenderer.h>
#include <H3D/MedX3D/FrameBufferTexture.h>

#include <HAPI/CollisionObjects.h>

#include <sstream>

using namespace H3D;
  
H3DNodeDatabase X3DVolumeNode::database( "X3DVolumeNode", 
                                          NULL,
                                          typeid( X3DVolumeNode ),
                                          &X3DChildNode::database 
                                          );
  
namespace X3DVolumeNodeInternals {
  FIELDDB_ELEMENT( X3DVolumeNode, dimensions, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DVolumeNode, renderer, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DVolumeNode, emptySpaceSkippingRes, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DVolumeNode, filterType, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DVolumeNode, filterTexture, OUTPUT_ONLY )
  FIELDDB_ELEMENT( X3DVolumeNode, bboxCenter, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( X3DVolumeNode, bboxSize, INITIALIZE_ONLY )


  // debug fields for visualizing empty space textures. Normally not used.
  //  FIELDDB_ELEMENT( RayCaster, emptySpaceClassificationTexture, OUTPUT_ONLY );
  //  FIELDDB_ELEMENT( RayCaster, emptySpaceMinMaxTexture, OUTPUT_ONLY );
} 
  
X3DVolumeNode::X3DVolumeNode( Inst< DisplayList > _displayList,
                              Inst< SFVec3f > _dimensions,
                              Inst< SFNode  > _metadata,
                              Inst< SFBound > _bound,
                              Inst< SFTexture3DNode > _voxels, // obs SF
                              Inst< SurfaceNormals > _surfaceNormals,
                              Inst< SFVec3f > _bboxCenter,
                              Inst< SFVec3f > _bboxSize,
                              Inst< SFString > _filterType,
                              Inst< SFFilterKernelTexture > _filterTexture,
            Inst< SFInt32 > _emptySpaceSkippingRes,
            Inst< SFEmptySpaceMinMaxTexture3D > _emptySpaceMinMaxTexture,
            Inst< SFEmptySpaceClassificationTexture2D > _emptySpaceClassificationTexture,
            Inst< SFVolumeRendererNode > _renderer  ) :
  X3DChildNode( _metadata ),
  H3DDisplayListObject( _displayList ),
  X3DBoundedObject( _bound, _bboxCenter, _bboxSize ),
  dimensions( _dimensions ),
  voxels( _voxels ),
  surfaceNormals( _surfaceNormals ),
  rayStep( new SFFloat ),
  useSlicing( new UseSlicingField ),
  filterType( _filterType ),
  filterTexture( _filterTexture ),
  emptySpaceSkippingRes( _emptySpaceSkippingRes ),
  emptySpaceMinMaxTexture( _emptySpaceMinMaxTexture ),
  emptySpaceClassificationTexture( _emptySpaceClassificationTexture ),
  renderer( _renderer ),
  stopRaysAtGeometries( new SFBool ),
  useEmptySpaceSkipping( new SFBool ),
  showNonEmptySpace( new SFBool ),
  useStochasticJittering( new SFBool ),
  textureMatrix( new SFMatrix4f ),
  textureMatrixInverse( new SFMatrix4f ),
  updateDimensions( new UpdateDimensions ),
  rebuildShader( new RebuildShader ),
  forceRebuildShader( new Field ),
  surfaceNormals_glsl( new SFTexture3DNode ) {
    
  type_name = "X3DVolumeNode";
  database.initFields( this );
 
  displayList->setOwner( this );
  bound->setOwner( this );
  updateDimensions->setOwner( this );
  rebuildShader->setOwner( this );
  emptySpaceClassificationTexture->setOwner( this );
  emptySpaceMinMaxTexture->setOwner( this );
  emptySpaceSkippingRes->setOwner( this );

  displayList->setName( "displayList" );
  bound->setName( "bound" );
  updateDimensions->setName( "updateDimensions" );
  rebuildShader->setName( "rebuildShader" );
  emptySpaceClassificationTexture->setName( "emptySpaceClassificationTexture" );
  emptySpaceMinMaxTexture->setName( "emptySpaceMinMaxTexture" );
  emptySpaceSkippingRes->setName( "emptySpaceSkippingRes" );
  

  // deprecated fields
  stopRaysAtGeometries->setOwner( this );
  useEmptySpaceSkipping->setOwner( this );
  showNonEmptySpace->setOwner( this );
  useStochasticJittering->setOwner( this );
  rayStep->setOwner( this );
  useSlicing->setOwner( this );
  stopRaysAtGeometries->setName( "stopRaysAtGeometries" );
  useEmptySpaceSkipping->setName( "useEmptySpaceSkipping" );
  showNonEmptySpace->setName( "showNonEmptySpace" );
  useStochasticJittering->setName( "useStochasticJittering" );
  rayStep->setName( "rayStep" );
  useSlicing->setName( "useSlicing" );

  // defaults
  dimensions->setValue( Vec3f(1.0f,1.0f,1.0f) );
  rayStep->setValue(0.01f);
  useSlicing->setValue( false );
  filterType->addValidValue( "DEFAULT" );
  filterType->addValidValue( "NEAREST" );
  filterType->addValidValue( "LINEAR" );
  filterType->addValidValue( "CUBIC_B_SPLINE" );
  filterType->addValidValue( "CATMULL_ROM" );
  filterType->setValue( "DEFAULT" );
  stopRaysAtGeometries->setValue( true );
  useEmptySpaceSkipping->setValue( false );
  emptySpaceSkippingRes->setValue( 8 );
  showNonEmptySpace->setValue( false );
  useStochasticJittering->setValue( false );
  
  RayCaster *ray_caster = new RayCaster;
  stopRaysAtGeometries->route( ray_caster->stopRaysAtGeometries );
  useEmptySpaceSkipping->route( ray_caster->useEmptySpaceSkipping );
  useStochasticJittering->route( ray_caster->useStochasticJittering );
  showNonEmptySpace->route( ray_caster->showNonEmptySpace );
  rayStep->route( ray_caster->rayStep );
  renderer->setValue( ray_caster );
 
  // routings
  renderer->route( displayList );
  dimensions->route( displayList );
  voxels->route( surfaceNormals );
  dimensions->route( updateDimensions );
  dimensions->route( bound );
  voxels->route( displayList );
  rayStep->route( displayList );

  surfaceNormals->route( surfaceNormals_glsl );

  // filter routes
  voxels->route( filterTexture, id );
  filterType->route( filterTexture, id );

  filterTexture->route( rebuildShader, id );
  voxels->route( rebuildShader );
  useSlicing->route( rebuildShader );
  stopRaysAtGeometries->route( rebuildShader );
  useEmptySpaceSkipping->route( rebuildShader );
  emptySpaceSkippingRes->route( rebuildShader );
  useStochasticJittering->route( rebuildShader );
  forceRebuildShader->route( rebuildShader );

  // empty space skipping routes
  voxels->route( emptySpaceMinMaxTexture, id );
  emptySpaceSkippingRes->route( emptySpaceMinMaxTexture, id );
}
  

void X3DVolumeNode::SFFilterKernelTexture::cubicBSplineWeights( H3DFloat x, 
                                                                H3DFloat &w0,
                                                                H3DFloat &w1,
                                                                H3DFloat &w2,
                                                                H3DFloat &w3 ) {
  H3DFloat x2 = x * x;
  H3DFloat x3 = x2 * x;
  
  w0 = (-x3 + 3*x2 - 3*x + 1) / 6;
  w1 = (3*x3 - 6*x2 + 4) / 6;
  w2 = (-3*x3 + 3*x2 + 3*x + 1) / 6;
  w3 = x3 / 6;
}

void X3DVolumeNode::SFFilterKernelTexture::catmullRomSplineWeights( H3DFloat x, 
                                                                    H3DFloat &w0,
                                                                    H3DFloat &w1,
                                                                    H3DFloat &w2,
                                                                    H3DFloat &w3 ) {
  H3DFloat x2 = x * x;
  H3DFloat x3 = x2 * x;
  
  w0 = (H3DFloat)( -0.5*x3 + x2 - 0.5*x );
  w1 = (H3DFloat)( 1.5*x3 - 2.5*x2 + 1 );
  w2 = (H3DFloat)(-1.5*x3 + 2*x2 + 0.5*x );
  w3 = (H3DFloat)(0.5*x3 - 0.5 * x2 );
  
}

void X3DVolumeNode::SFFilterKernelTexture::update() {
  X3DTexture3DNode *tex = 
    static_cast< SFTexture3DNode * >( routes_in[0] )->getValue();
  const string &filter_type = 
    static_cast< SFString * >( routes_in[1] )->getValue();
  
  unsigned int dimension = 256;

  if( filter_type == "CUBIC_B_SPLINE"  || filter_type == "CATMULL_ROM") {
    bool b_spline = filter_type == "CUBIC_B_SPLINE";

    // create a RGB image with 16 bits for each channel to store the values in
    PixelImage *image = 
      new PixelImage( dimension, 1, 1, 48, Image::RGB, Image::UNSIGNED );
    
    for( unsigned i = 0; i < dimension; ++i ) {
      H3DFloat w0, w1, w2, w3; 
      H3DFloat x = i / (H3DFloat)dimension;
      if( b_spline ) cubicBSplineWeights( x, w0, w1, w2, w3 );
      else catmullRomSplineWeights( x, w0, w1, w2, w3 );
      H3DFloat h0 = 1 - w1 / ( w0 + w1 ) + x;
      H3DFloat h1 = 1 + w3 / ( w2 + w3 ) - x;
      H3DFloat g1 = w2 + w3;

      // alpha value is ignored so just set it to 1
      image->setPixel( RGBA( h1, h0, g1, 1 ), i, 0, 0 );
    }
    
    X3DTexture2DNode *new_tex = new PixelTexture;
    new_tex->image->setValue( image );
    new_tex->repeatS->setValue( true );
    value.reset( new_tex );
  } else {
    value.reset( NULL );
  }
}




void X3DVolumeNode::render() {
  H3DVolumeRendererNode *volume_renderer = renderer->getValue();
  if( !volume_renderer ) return;


  if( requiresEnabledLights() || volume_renderer->requiresEnabledLights() ) {
    vector< unsigned int > current_enabled_lights;
    GLint max_lights;
    
    glGetIntegerv( GL_MAX_LIGHTS, &max_lights );
    
    for( int i = 0; i < max_lights; ++i ) {
      GLboolean enabled;
      glGetBooleanv( GL_LIGHT0+i, &enabled );
      if( enabled ) current_enabled_lights.push_back( i );
    }
    
    if( current_enabled_lights.size() !=
        enabled_lights.size() ||
        !std::equal( current_enabled_lights.begin(), 
                     current_enabled_lights.end(),
                     enabled_lights.begin() ) ) {
      enabled_lights.swap( current_enabled_lights );
      buildShader();
    }
  }

  rebuildShader->upToDate();
  

  if( X3DShapeNode::geometry_render_mode == X3DShapeNode::ALL ||
      X3DShapeNode::geometry_render_mode == X3DShapeNode::TRANSPARENT_ONLY ||
      X3DShapeNode::geometry_render_mode == X3DShapeNode::TRANSPARENT_FRONT ) {
     glPushAttrib(GL_ALL_ATTRIB_BITS);
     // set the voxels, texture matrices, ray step, and normals if needed 
     updateUniformFields();

     // set up blending and backface culling
    
     glEnable(GL_BLEND);
     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
     glEnable(GL_CULL_FACE);
     glCullFace(GL_BACK);
     
     // do the volume rendering
     if( volume_renderer ) {
       volume_renderer->render( this );
     }
     
     glPopAttrib();
  }
}
  
void X3DVolumeNode::initialize() {
  const Vec3f &size = bboxSize->getValue();
  if( size.x == -1 && size.y == -1 && size.z == -1 ) {
    // use dimensions to set the bound
    BoxBound *bb = new BoxBound();
    bb->center->setValue( Vec3f(0,0,0) );
    bb->size->setValue( dimensions->getValue() );
    bound->setValue( bb );
  }
  else {
    BoxBound *bb = new BoxBound();
    bb->center->setValue( bboxCenter->getValue() );
    bb->size->setValue( bboxSize->getValue() );
    bound->setValue( bb );
  }
    
  X3DChildNode::initialize();
}

void X3DVolumeNode::SurfaceNormals::update() {
  X3DTexture3DNode *_voxels = 
    static_cast<SFTexture3DNode*>( routes_in[0] )->getValue();
  if( _voxels ) {
    Image *im = _voxels->image->getValue();
    if( im ) {
      Console(3) << "Computing surface normals" << endl;
  
      // gradient filter
      VolumeGradient vg;
      vg.setInput(im);
      vg.execute();
  
      // create texture and set image
      Pixel3DTexture *pt = new Pixel3DTexture;
      pt->image->setValue( vg.getOutput() );
      pt->repeatS->setValue( false );
      pt->repeatT->setValue( false );
      pt->repeatR->setValue( false );
  
      value = pt;
  
      Console(3) << "Finished computing surface normals" << endl;
    }
  }
}
  
void X3DVolumeNode::UpdateDimensions::update() {
  const Vec3f &dim = 
    static_cast<SFVec3f*>(routes_in[0])->getValue();
  
  X3DVolumeNode *vd = static_cast< X3DVolumeNode * >( getOwner() );
   
  // the dimensions is the value of this field
  value = dim;
    
  // set texture matrix
  // scaling according to total extent => [-0.5, 0.5]
  Matrix4f S  = Matrix4f(1.0f/dim.x, 0.0, 0.0, 0.0,
                         0.0, 1.0f/dim.y, 0.0, 0.0,
                         0.0, 0.0, 1.0f/dim.z, 0.0, 
                         0.0, 0.0, 0.0, 1.0f);
  // center => [0, 1]
  Matrix4f  T  = Matrix4f(1.0, 0.0, 0.0, 0.5,
                          0.0, 1.0, 0.0, 0.5,
                          0.0, 0.0, 1.0, 0.5,
                          0.0, 0.0, 0.0, 1.0);
    
  // This is not needed on hardware that supports 
  // non-power-of-two-textures
  //     // scale according to non-power-of-two-textures
  //     Matrix4f S2 = Matrix4f(float(w)/tw, 0, 0, 0,
  //                            0, float(h)/th, 0, 0,
  //                            0, 0, float(d)/td, 0, 
  //                            0, 0, 0, 1);
    
  Matrix4f mtex = T*S;
  vd->textureMatrix->setValue(mtex);
  vd->textureMatrixInverse->setValue( mtex.inverse() );
    
  // set ray step
//  H3DFloat raystep = 1.0f/H3DFloat( H3DMax(w, H3DMax(h, d)) ); 
//  vd->rayStep->setValue( raystep );
}


void X3DVolumeNode::traverseSG( TraverseInfo &ti ) {
  ti.setUserData( "VolumeNode", this );

  if( X3DAppearanceNode::getDefaultUsingMultiPassTransparency() ) {
    ti.setMultiPassTransparency( true );
    displayList->breakCache();
  }

  H3DVolumeRendererNode *volume_renderer = renderer->getValue();
  if( volume_renderer ) {
    volume_renderer->traverseSG( this, ti );
  }
}

bool X3DVolumeNode::lineIntersect( 
          const Vec3f &from,
          const Vec3f &to,
          LineIntersectResult &result ) {

  bool returnValue = false;
  Bound * the_bound = bound->getValue();
  if( the_bound ) {
    Vec3f temp_center;
    Vec3f temp_size;
    BoxBound * box_bound = dynamic_cast< BoxBound * >(the_bound);
    if( box_bound ) {
      temp_center = box_bound->center->getValue();
      temp_size = box_bound->size->getValue();
    }
    else {
      temp_center = bboxCenter->getValue();
      temp_size = bboxSize->getValue();
      if( temp_size.x < 0 )
        temp_size = Vec3f();
    }

    Vec3f result_point;
    // this is equvalent to do a lineIntersect with the bounding
    // box of this node but will return a point of intersection.
    returnValue = lineSegmentIntersect( from, to, temp_center, temp_size,
                                        result_point );
    if( returnValue ) {
      IntersectionInfo temp_result;
      temp_result.point = result_point;
      result.addResults( temp_result, this );
      result.addPtDevMap();
    }
  }
  return returnValue;
}

void X3DVolumeNode::closestPoint( const Vec3f &p,
                                  NodeIntersectResult &result ) {
  Bound * the_bound = bound->getValue();
  if( the_bound ) {
    IntersectionInfo temp_info;
    temp_info.point = the_bound->closestPoint( p );
    temp_info.normal = p - temp_info.point;
    temp_info.normal.normalizeSafe();
    result.addResults( temp_info, this );
  }
}

bool X3DVolumeNode::movingSphereIntersect( H3DFloat radius,
                                           const Vec3f &from, 
                                           const Vec3f &to,
                                           NodeIntersectResult &result ) {
  BoxBound *box_bound = dynamic_cast< BoxBound * >( bound->getValue() );
  if( box_bound ) {
    Vec3f box_size_half = box_bound->size->getValue() / 2;
    HAPI::Collision::Sphere col_sphere( box_bound->center->getValue(),
                                        box_size_half.length() );
    IntersectionInfo temp_result;
    if( col_sphere.movingSphereIntersect( radius, from, to, temp_result ) ) {
      temp_result.primitive = 0;
      result.addResults( temp_result, this );
      return true;
    } else return false;
  } else return false;
}

bool X3DVolumeNode::lineSegmentIntersect( Vec3f from, Vec3f to, Vec3f _center,
                                          Vec3f _size, Vec3f &q ) {
  H3DFloat tmin = 0.0f;
  H3DFloat tmax = 1.0f;
  Vec3f d = to - from;
    
  Vec3f bb_min = _center - _size / 2;
  Vec3f bb_max = _center + _size / 2;
       
  // For all three slabs
  for( int i = 0; i < 3; ++i ) {
    if( H3DAbs( d[i] ) < Constants::f_epsilon ) {
      // Line Segment is parallel to slab. No hit if origin not within slab
      if( from[i] < bb_min[i] || from[i] > bb_max[i] ) return false;
    } else {
      // Compute intersection t value of line segment
      // with near and far plane of slab
      H3DFloat ood = 1.0f / d[i];
      H3DFloat t1 = ( bb_min[i] - from[i] ) * ood;
      H3DFloat t2 = ( bb_max[i] - from[i] ) * ood;
      // Make t1 be intersection with near plane, t2 with far plane
      if( t1 > t2 ) {
        H3DFloat temp_t = t1;
        t1 = t2;
        t2 = temp_t;
      }
      // Compute the intersection of slab intersection intervals
      if( t1 > tmin ) tmin = t1;
      if( t2 > tmax ) tmax = t2;
      // Exit with no collision as soon as slab intersection becomes empty
      if( tmin > tmax ) return false;
    }
  }

  // Line Segment intersect all 3 slabs. Return point (q).
  q = from + d * tmin;
  return true;
}
  
void X3DVolumeNode::buildShader() {
  H3DVolumeRendererNode *volume_renderer = renderer->getValue();
  if( volume_renderer ) {
    volume_renderer->buildShader( this );
  }
}

string X3DVolumeNode::addUniforms( ComposedShader *shader ) {
  stringstream s;
  
  // add the voxel data
  s << addUniformToFragmentShader( shader,
                                   uniqueShaderName("voxels"), 
                                   "sampler3D",
                                   H3D::Field::INPUT_OUTPUT,
                                   copyAndRouteField( voxels ) );
  
  // add normals if required
  if( requiresDefaultNormals() ) {
    s << addUniformToFragmentShader( shader,
                                     uniqueShaderName("defaultNormals"), 
                                     "sampler3D",
                                     H3D::Field::INPUT_OUTPUT,
                                     copyAndRouteField( surfaceNormals_glsl ) );
  } 

  // add the texture matrix to the shader
  s << addUniformToFragmentShader( shader,
                                   //uniqueShaderName("textureMatrix"),
           "textureMatrix",
                                   "mat4",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( textureMatrix ) );
    
    // add the texture matrix inverse to the shader
  s << addUniformToFragmentShader( shader,
                                   //uniqueShaderName("textureMatrixInverse"),
           "textureMatrixInverse",
                                   "mat4",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( textureMatrixInverse ) );


  // add filter texture
  const string &filter_type = filterType->getValue();
  if( filter_type == "CUBIC_B_SPLINE" || filter_type == "CATMULL_ROM" ) {
    SFNode* f = new TypedSFNode< X3DTexture2DNode >();
    filterTexture->route( f );
    s << addUniformToFragmentShader( shader,
             uniqueShaderName("filterTex"),
                                     "sampler2D",
                                     H3D::Field::INPUT_OUTPUT,
                                     f );
  }

  return s.str();
}


string X3DVolumeNode::getRayInitializationCode() {
  return "";
}

string X3DVolumeNode::getOrigSampleColor() {
  stringstream s;
  const string &filter_type = filterType->getValue();
  X3DTexture3DNode *tex = voxels->getValue();
  Image *image = NULL;
  if( tex ) image = tex->image->getValue();

  if( image && (filter_type == "CUBIC_B_SPLINE" || filter_type == "CATMULL_ROM" ) ) {
    
    // voxel size in texture coordinates
    Vec3f voxel_size( 1.0f/image->width(), 1.0f/image->height(), 1.0f/image->depth() );

    s << "vec4 orig_sample_color = filterTriCubic( r0.xyz, "
      << uniqueShaderName( "voxels" ) << ", "
      << uniqueShaderName( "filterTex" ) << ", "
      << "vec3( " << voxel_size.x << ", 0.0, 0.0 )," 
      << "vec3( 0.0, " << voxel_size.y << ", 0.0 )," 
      << "vec3( 0.0, 0.0, " << voxel_size.z << " ),"
      << "vec3( " 
      << image->width() << ".0, " 
      << image->height() << ".0, " 
      << image->depth()  << ".0 ) );" << endl;
  } else {
    s << "vec4 orig_sample_color = texture3D("
      << uniqueShaderName( "voxels" ) << ", r0.xyz);" << endl;
    if( image && filter_type != "DEFAULT" && 
        filter_type != "LINEAR" && filter_type != "NEAREST" ){
      Console(4) << "Invalid filter type: " << filter_type 
                 << "(in X3DVolumeNode)" << endl;
    }
  }

  return s.str();
}

string X3DVolumeNode::getShaderCode() { 
  return ""; 
}

string X3DVolumeNode::getShaderCodeOpacityOnly() { 
  return getShaderCode(); 
}

string X3DVolumeNode::getShaderFunctions() { 
  stringstream s;
  s << "const int nr_enabled_lights = " << enabled_lights.size()
    << "; \n\n"
    << "#define getEnabledLight( var1, var2, var3, var4, a ) vec4 var1; vec4 var2; vec4 var3; vec4 var4;";
  if( enabled_lights.empty() )
    s << " \n";
  else
    s << " \\\n";

  for( unsigned int i = 0; i < enabled_lights.size(); ++i ) {
    if( i != 0 ) s << "\\\n else ";
    s << "if( a == " << i << " ){ var1 = gl_LightSource[" 
      << enabled_lights[i] << "].position; var2 = gl_LightSource["
      << enabled_lights[i] << "].diffuse; var3 = gl_LightSource["
      << enabled_lights[i] << "].ambient; var4 = gl_LightSource["
      << enabled_lights[i] << "].specular; } ";
  }
  s << endl;
  
  return s.str();
}


string X3DVolumeNode::
addUniformToFragmentShader( ComposedShader *shader,
          const string &name,
          const string &glsl_type,
          const Field::AccessType &access,
          Field *field,
          int array_size,
          bool delete_unadded_field ) {
  return H3DVolumeRendererNode::addUniformToFragmentShader( shader,
                  name,
                  glsl_type,
                  access,
                  field,
                  array_size,
                  delete_unadded_field );
 
}

string X3DVolumeNode::getShaderCompositingCode() {
  return frontToBackCompositionNonAssociated();
}

string X3DVolumeNode::frontToBackCompositionNonAssociated() {
  return 
    " // front-to-back compositing \n"
    "sample_color.rgb *= sample_color.a; \n"
    "rr.color += sample_color*(1.0-rr.color.a);\n"
    "if( rr.zpoint.x < 0.0 && rr.color.a > 0.0 ) { \n"
    "  rr.zpoint = vec4( r0.xyz, 1 ); \n"
    "} \n";
}

string X3DVolumeNode::frontToBackCompositionAssociated() {
  return 
    " // front-to-back compositing \n"
    "rr.color += sample_color*(1.0-rr.color.a);\n"
    "if( rr.zpoint.x < 0.0 && rr.color.a > 0.0 ) { \n"
    "  rr.zpoint = vec4( r0.xyz, 1 ); \n"
    "} \n";
}

string X3DVolumeNode::MIPComposition() {
  return 
    " // MIP compositing \n"
    "if( sample_color.a > max_intensity ) { \n"
    "  max_intensity = sample_color.a; \n" 
    "  rr.color = sample_color; \n"
    "  rr.zpoint = vec4(r0.xyz, 1); \n" 
    "} \n";
}

bool X3DVolumeNode::usingRayCaster() {
  return (dynamic_cast< RayCaster * >( renderer->getValue() ) != NULL ||
    (dynamic_cast< MultiVolumeRayCaster * >( renderer->getValue() ) != NULL ) );
}

template< class A >
void findMinMax( Image *image,
                 unsigned int volume_min_x,
                 unsigned int volume_min_y,
                 unsigned int volume_min_z,
                 unsigned int volume_max_x,
                 unsigned int volume_max_y,
                 unsigned int volume_max_z,
                 A &min, A &max ) {
   A *data = (A*) image->getImageData();
   unsigned int w = image->width();
   unsigned int h = image->height();
   unsigned int d = image->depth();
   A max_v, min_v;

   // go through each voxel in the volume and find the minimum and maximum value.
   max_v = min_v = data[ volume_min_z * w * h + volume_min_y * w + volume_min_x];
   for( unsigned int z = volume_min_z + 1; z <= volume_max_z; ++z ) {
    for( unsigned int y = volume_min_y + 1; y <= volume_max_y; ++y ) {
      for( unsigned int x = volume_min_x + 1; x <= volume_max_x; ++x ) {
        A v =  data[ z * w * h + y * w + x ];
        if( v > max_v ) max_v = v;
        else if( v < min_v ) min_v = v; 
      }
    }
   }
   
   // transfer value to output parameters 
   min = min_v;
   max = max_v;
/*
   Console( 4 ) << volume_min_x << " " << volume_max_x << " " 
                << volume_min_y << " " << volume_max_y << " "
                << volume_min_z << " " << volume_max_z << " "
                << min << " " << max << endl;
*/
                }

template< class A >
void getMinMaxData( Image *image,
                    H3DInt32 resolution,
                    void *out_data ) {
  A *data = (A*)out_data;
  unsigned int w = image->width();
  unsigned int h = image->height();
  unsigned int d = image->depth();
  
  Vec3f step_size( w / (H3DFloat) resolution,
                   h / (H3DFloat)resolution,
                   d / (H3DFloat)resolution );

  unsigned int index = 0;
  for( H3DInt32 z = 0; z < resolution; ++z ) {
    for( H3DInt32 y = 0; y < resolution; ++y ) {
      for( H3DInt32 x = 0; x < resolution; ++x, index+=2 ) {
        A min_v, max_v;
        Vec3f last_pos( H3DMax( H3DFloor( x * step_size.x), 0.f ), 
                        H3DMax( H3DFloor( y * step_size.y), 0.f ), 
                        H3DMax( H3DFloor( z * step_size.z), 0.f ) );
        Vec3f pos( H3DMin( H3DCeil( (x + 1) * step_size.x - 1), (H3DFloat) w ), 
                   H3DMin( H3DCeil( (y + 1) * step_size.y - 1), (H3DFloat) h ), 
                   H3DMin( H3DCeil( (z + 1) * step_size.z - 1), (H3DFloat) d ) );
        
        
        findMinMax( image,   
                    (unsigned int) last_pos.x,
                    (unsigned int)last_pos.y, 
                    (unsigned int)last_pos.z,
                    (unsigned int)pos.x,
                    (unsigned int)pos.y,
                    (unsigned int)pos.z,
                    min_v, max_v );
        /*Console( 4 ) << x << " " << y << " " << z << " "
                << (int)min_v << " " << (int)max_v << endl;*/
        data[index] = min_v; 
        data[index+1] = max_v;
      }
    }
  }
}

void X3DVolumeNode::SFEmptySpaceMinMaxTexture3D::update() {
  X3DTexture3DNode *_voxels = 
    static_cast< SFTexture3DNode * >( routes_in[0] )->getValue();
  H3DInt32 res = static_cast< SFInt32 * >( routes_in[1] )->getValue();
  
  // get the image data
  Image *image = NULL;
  if( _voxels ) image = _voxels->image->getValue();

  // if no image data, set field to null
  if( !image ) {
    value.reset( NULL );
    return;
  }

  if( !value.get() ) {
    Pixel3DTexture *pt = new Pixel3DTexture;
    TextureProperties *tp = new TextureProperties;
    tp->boundaryModeS->setValue("CLAMP_TO_EDGE");
    tp->boundaryModeT->setValue("CLAMP_TO_EDGE");
    tp->boundaryModeR->setValue("CLAMP_TO_EDGE");
    tp->minificationFilter->setValue("NEAREST_PIXEL");
    tp->magnificationFilter->setValue("NEAREST_PIXEL");
    tp->textureCompression->setValue("DEFAULT");
    pt->textureProperties->setValue(tp);
    value.reset( pt );
  } 
  
  Pixel3DTexture *pt = static_cast< Pixel3DTexture * >( value.get() );
  


  void *data = new unsigned char[image->bitsPerPixel() / 8 *
                                 res*res*res*2 ];

  if( image->pixelType() == Image::LUMINANCE ) {
    Image::PixelComponentType type = image->pixelComponentType();
    if( type ==Image::UNSIGNED ) {
      switch( image->bitsPerPixel() ) {
      case 8:  
        getMinMaxData< unsigned char >( image, res, data ); break;
      case 16:
        getMinMaxData< unsigned short >( image, res, data ); break;
      case 32: 
        getMinMaxData< unsigned int >( image, res, data ); break;
      default: 
        Console(4) << "Warning: Invalid data type for empty space skipping." << endl; 
      }
    } else if( type == Image::SIGNED ) {
      switch( image->bitsPerPixel() ) {
      case 8:  
        getMinMaxData< char >( image, res, data ); break;
      case 16:
        getMinMaxData< short >( image, res, data ); break;
      case 32: 
        getMinMaxData< int >( image, res, data ); break;
      default: 
        Console(4) << "Warning: Invalid data type for empty space skipping." << endl; 
      }
    } else if( type == Image::RATIONAL ) {
      switch( image->bitsPerPixel() ) {
      case 32:
        getMinMaxData< float >( image, res, data ); break;
      case 64: 
        getMinMaxData< double >( image, res, data ); break;
      default: 
        Console(4) << "Warning: Invalid data type for empty space skipping." << endl; 
      }
    } 
  } else {
    Console(4) << "Warning: Invalid voxel type for empty space skipping."
               << " Only works with LUMINANCE textures." << endl; 
  }

    PixelImage *pi = new PixelImage( res, res, res, 
                                   image->bitsPerPixel()*2, 
                                   Image::LUMINANCE_ALPHA, 
                                   image->pixelComponentType(),
                                   (unsigned char *)data );
    pt->image->setValue( pi );
}      


void X3DVolumeNode::SFEmptySpaceClassificationTexture2D::update() {
  X3DVolumeNode *volume = static_cast< X3DVolumeNode * >( getOwner() );

  // initialize the field if it does not contain a texture node.
  if( !value.get() ) {
    PixelTexture *pt =  new PixelTexture;
    TextureProperties *tp = new TextureProperties;
    tp->boundaryModeS->setValue("CLAMP_TO_EDGE");
    tp->boundaryModeT->setValue("CLAMP_TO_EDGE");
    tp->boundaryModeR->setValue("CLAMP_TO_EDGE");
    tp->minificationFilter->setValue("NEAREST_PIXEL");
    tp->magnificationFilter->setValue("NEAREST_PIXEL");
    tp->textureCompression->setValue("DEFAULT");
    pt->textureProperties->setValue(tp);
    value.reset( pt );
  }

  PixelTexture *pt = static_cast< PixelTexture * >( value.get() );

  // properties for created texture
  unsigned int max_value = 255;
  unsigned int x_dim = 256;
  unsigned int y_dim = 256;

  unsigned char *data = new unsigned char[ x_dim * y_dim ];

  // set all pixels with possible empty space to 0 and all else to 
  // maximum possible value.
  for( unsigned int y = 0; y < y_dim; ++y ) {
    for( unsigned int x = 0; x <= y; ++x ) {
      if(  volume->isEmptySpace( x / (float) x_dim,
                                 y / (float) y_dim ) ) {
        data[ y * x_dim + x ] = 0;
      } else {
        data[ y * x_dim + x ] = max_value;
      }
    }
  }

  // create new image
  PixelImage *pi = new PixelImage( x_dim, y_dim, 1, 
                                   8, 
                                   Image::LUMINANCE, 
                                   Image::UNSIGNED,
                                   data );
  // use the new image in the texture
  pt->image->setValue( pi );
}


void X3DVolumeNode::UseSlicingField::onValueChange( const bool &v ) {
  X3DVolumeNode *volume = static_cast< X3DVolumeNode * >( getOwner() );

  if( v ) {
    SliceRenderer *slice_renderer = new SliceRenderer;
    slice_renderer->nrSlices->setValue( (H3DInt32)(1.0 / volume->rayStep->getValue() ) );
    volume->renderer->setValue( slice_renderer );
  } else {
    RayCaster *raycaster = new RayCaster;
    volume->stopRaysAtGeometries->route( raycaster->stopRaysAtGeometries );
    volume->useEmptySpaceSkipping->route( raycaster->useEmptySpaceSkipping );
    volume->useStochasticJittering->route( raycaster->useStochasticJittering );
    volume->showNonEmptySpace->route( raycaster->showNonEmptySpace );
    volume->rayStep->route( raycaster->rayStep );
    volume->renderer->setValue( raycaster );
  }
}


void X3DVolumeNode::SFVolumeRendererNode::onAdd( Node *n ) {
  SFVolumeRendererNodeBase::onAdd( n );
  H3DVolumeRendererNode *_renderer = 
    static_cast< H3DVolumeRendererNode * >( n );
  X3DVolumeNode *volume = static_cast< X3DVolumeNode * >(getOwner());
  if( n ) {
    _renderer->addVolume( volume );
  }
}


void X3DVolumeNode::SFVolumeRendererNode::onRemove( Node *n ) {
  H3DVolumeRendererNode *_renderer =
    static_cast< H3DVolumeRendererNode * >( n );
  X3DVolumeNode *volume = static_cast< X3DVolumeNode * >(getOwner());
  if( n ) {
    _renderer->removeVolume( volume );
  }
  SFVolumeRendererNodeBase::onRemove( n );
}

string X3DVolumeNode::uniqueShaderName( const string &base_name ) {
  uintptr_t sid = uintptr_t(this);
  ostringstream ostr;
  ostr << hex << sid;
  return base_name + ostr.str();
}
