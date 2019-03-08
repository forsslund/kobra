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
/// \file SliceRenderer.cpp
/// \brief CPP file for SliceRenderer.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/SliceRenderer.h>
#include <H3D/MedX3D/X3DVolumeNode.h>

using namespace H3D;

H3DNodeDatabase SliceRenderer::database( "SliceRenderer", 
                                     &newInstance< SliceRenderer >, 
                                     typeid( SliceRenderer ),
                                     &H3DVolumeRendererNode::database );

namespace SliceRendererInternals {

  FIELDDB_ELEMENT( SliceRenderer, nrSlices, INPUT_OUTPUT );

  const string vertexshader_slice =
    "//BEGIN UNIFORMS\n"
    "uniform mat4 textureMatrix;\n"
    "uniform mat4 textureMatrixInverse;\n"
    "//END UNIFORMS\n"
    "\n"
    "//BEGIN VARYINGS\n"
    "varying vec3 viewDir;\n"
    "//END VARYINGS\n"
    "\n"
    "void main() {\n"
    "  vec3 rayStart = (textureMatrix*gl_Vertex).xyz;\n"
    "  viewDir = rayStart-(textureMatrix*gl_ModelViewMatrixInverse*vec4(0,0,0,1)).xyz;\n"
    " gl_TexCoord[0] = gl_MultiTexCoord0; \n"
    "  gl_Position = gl_ModelViewProjectionMatrix*gl_Vertex;\n"
    "  gl_ClipVertex = gl_ModelViewMatrix*gl_Vertex;\n"
    "}\n";

    // "(MEDX3D_DIRECTORY)/src/shaders/Slices_FS_main.glsl"
  const string slices_fs_main =
    "//////////////////////////////////////////////////////////////////////////////\n"
    "//    Copyright 2004-2019, SenseGraphics AB\n"
    "//\n"
    "//    This file is part of H3D API.\n"
    "//\n"
    "//    H3D API is free software; you can redistribute it and/or modify\n"
    "//    it under the terms of the GNU General Public License as published by\n"
    "//    the Free Software Foundation; either version 2 of the License, or\n"
    "//    (at your option) any later version.\n"
    "//\n"
    "//    H3D API is distributed in the hope that it will be useful,\n"
    "//    but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
    "//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
    "//    GNU General Public License for more details.\n"
    "//\n"
    "//    You should have received a copy of the GNU General Public License\n"
    "//    along with H3D API; if not, write to the Free Software\n"
    "//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA\n"
    "//\n"
    "//    A commercial license is also available. Please contact us at \n"
    "//    www.sensegraphics.com for more information.\n"
    "//\n"
    "//////////////////////////////////////////////////////////////////////////////\n"
    "//\n"
    "// This is the fragment shader program for volume rendering with glsl\n"
    "// shaders in the MedX3D package.\n"
    "// \n"
    "// The different render styles add code to this shader program through the\n"
    "// functions addUniforms and addShaderCode.\n"
    "//\n"
    "//////////////////////////////////////////////////////////////////////////////\n"
    "// shadows are not supporeted with slices so just use the normal \n"
    "// without shadows \n"
    "void ShadedVolumeStyleWithShadows(inout vec4 current_color,\n"
    "                                  vec4 emissive_color,\n"
    "                                  vec4 diffuse_color,\n"
    "                                  vec4 ambient_color,\n"
    "                                  vec4 specular_color,\n"
    "                                  float shininess,\n"
    "                                  vec4 pos,\n"
    "                                  vec4 viewdir,\n"
    "                                  mat4 view_to_tex,\n"
    "                                  bool enabled,\n"
    "                                  bool lighting,\n"
    "                                  vec4 normal,\n"
    "                                  float step_size ) {\n"
    "    ShadedVolumeStyle( current_color, emissive_color,\n"
    "                                  diffuse_color, ambient_color,\n"
    "                                  specular_color, shininess,\n"
    "                                  pos, viewdir, view_to_tex,\n"
    "                                  enabled, lighting, normal );\n"
    "    }\n"
    "\n"
    "varying vec3 viewDir;\n"
    "\n"
    "// main function\n"
    "void main() {\n"
    "\n"
    "  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;\n"
    "  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;\n"
    "  vec4 viewdir_tex = vec4( -normalize(viewDir.xyz), 0.0 );\n"
    "\n"
    "  //BEGIN PRE-LOOP\n"
    "  //END PRE-LOOP\n"
    "  \n"
    "\n"
    "  vec4 r0 = gl_TexCoord[0];  \n"
    "\n"
    "//GET_ORIG_SAMPLE_COLOR\n"
    "\n"
    "  //ORIG_SAMPLE_MANIP\n"
    "\n"
    "  // color of this sample\n"
    "  vec4 sample_color = orig_sample_color;  \n"
    "\n"
    "  \n"
    "\n"
    "  //BEGIN INSIDE-LOOP\n"
    "  //END INSIDE-LOOP\n"
    "\n"
    "  //  if( sample_color.a == 0.0 ) discard;\n"
    "  \n"
    "  // set color\n"
    "  gl_FragColor = sample_color;\n"
    "}\n"
    " \n";
}

SliceRenderer::SliceRenderer( Inst< SFNode  > _metadata,
                              Inst< SFInt32 > _nrSlices ):
  H3DVolumeRendererNode( _metadata ),
  nrSlices( _nrSlices ) {

  type_name = "SliceRenderer";
  database.initFields( this );

  nrSlices->setValue( 100 );

  nrSlices->route( paramsChanged );
}


// render view-plane aligned slices
// this code has been taken from 
// Volume Haptics Tool Kit (VHTK) by Karljohan Lundin
// and been slightly modified
void SliceRenderer::renderSlices(X3DVolumeNode *volume, int planes, const Vec3f &s) {
  Vec3f size = s;
  size *= 0.5;
    
  Vec3f edgeR[12] = { Vec3f(      0,-size.y,-size.z ),
                      Vec3f( size.x,-size.y,      0 ),
                      Vec3f(      0,-size.y, size.z ),
                      Vec3f(-size.x,-size.y,      0 ),
                      Vec3f(      0, size.y,-size.z ),
                      Vec3f( size.x, size.y,      0 ),
                      Vec3f(      0, size.y, size.z ),
                      Vec3f(-size.x, size.y,      0 ),
                      Vec3f(-size.x,      0,-size.z ),
                      Vec3f( size.x,      0,-size.z ),
                      Vec3f(-size.x,      0, size.z ),
                      Vec3f( size.x,      0, size.z ) };
    
  Vec3f edgeV[12] = { Vec3f( size.x, 0, 0 ), Vec3f( 0, 0, size.z ),
                      Vec3f( size.x, 0, 0 ), Vec3f( 0, 0, size.z ),
                      Vec3f( size.x, 0, 0 ), Vec3f( 0, 0, size.z ),
                      Vec3f( size.x, 0, 0 ), Vec3f( 0, 0, size.z ),
                      Vec3f( 0, size.y, 0 ), Vec3f( 0, size.y, 0 ),
                      Vec3f( 0, size.y, 0 ), Vec3f( 0, size.y, 0 ) };
    
  // Calculate the c vector - 
  // the view direction in the unrotated version of the volume
  float mm[16];
  glGetFloatv(GL_MODELVIEW_MATRIX,mm);
  Vec3f c( mm[2], mm[6], mm[10] );
  c.normalize();
    
  // Find where to put the first and the last slice polygon
  float maxz =
    max( max( fabsf( c * Vec3f(+size.x,+size.y,+size.z) ),
              fabsf( c * Vec3f(-size.x,+size.y,+size.z) ) ),
         max( fabsf( c * Vec3f(+size.x,-size.y,+size.z) ),
              fabsf( c * Vec3f(-size.x,-size.y,+size.z) ) ) );
    
  glFlush();
  glPushAttrib(GL_ALL_ATTRIB_BITS);
    
  glDepthFunc(GL_LEQUAL);
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
    
  volume->setSliceRenderBlendMode();
  
  glShadeModel(GL_FLAT);
    
  glNormal3f( c[0], c[1], c[2] );
  glColor4f( 1, 1, 1, 1 );
    
  // Draw the slices
  float step = 2*size.length()/planes;
  for( float z = -maxz +step*0.5f ; z < maxz ; z += step ){
      
    Vec3f point[6];
    int points = 0;
      
    // Find the vertices of the polygon - these are the intersection between
    // the edges and the plane built up by c
    for( int i = 0 ; i < 12 ; ++i )
      if( ( z > ( edgeR[i] - edgeV[i] ) * c &&
            z <= ( edgeR[i] + edgeV[i] ) * c ) ||
          ( z > ( edgeR[i] + edgeV[i] ) * c &&
            z <= ( edgeR[i] - edgeV[i] ) * c ) )
        point[points++] = edgeR[i] + 
          edgeV[i]*(( z - edgeR[i] * c )/( edgeV[i] * c ));
      
    // Find center of the polygon, which not necessarily is 
    // lying on the c vector
    Vec3f center( 0, 0, 0 );
    for( int i = 0 ; i < points ; ++i )
      center += point[i];
    center /= points;
      
    // Order clockwise...
    Vec3f v1 = point[0] - center;
    if( v1.length() <= Constants::f_epsilon ){
      continue; }
    v1.normalize();
    Vec3f v2 = v1 % c;
    assert( v2.length() > Constants::f_epsilon );
    v2.normalize();

    // ..using arctan ---
    for( int i = 1 ; i < points ; ++i )
      for( int j = 0 ; j < points - i ; ++j )
        if( atan2( (point[j  ] - center)*v1, (point[j  ] - center)*v2 ) > 
            atan2( (point[j+1] - center)*v1, (point[j+1] - center)*v2 ) ){
          Vec3f t = point[j];
          point[j] = point[j+1];
          point[j+1] = t;
        }
      
#define VERTEX(n)          \
    glTexCoord3f( .5f + .5f*point[n].x/size[0],    \
                  .5f + .5f*point[n].y/size[1],    \
                  .5f + .5f*point[n].z/size[2]  );  \
    glVertex3f( point[n].x, point[n].y, point[n].z );
      
    // Draw out the triangle(s) of this slice
    assert( 3 <= points && points <= 6 );
    switch(points){
    case 3:
      glBegin(GL_TRIANGLES);
      VERTEX(0);
      VERTEX(1);
      VERTEX(2);
      glEnd();
      break;
    case 4:
      glBegin(GL_TRIANGLE_STRIP);
      VERTEX(0);
      VERTEX(1);
      VERTEX(3);
      VERTEX(2);
      glEnd();
      break;
    case 5:
      glBegin(GL_TRIANGLE_STRIP);
      VERTEX(0);
      VERTEX(1);
      VERTEX(4);
      VERTEX(2);
      VERTEX(3);
      glEnd();
      break;
    case 6:
      glBegin(GL_TRIANGLE_STRIP);
      VERTEX(0);
      VERTEX(1);
      VERTEX(5);
      VERTEX(2);
      VERTEX(4);
      VERTEX(3);
      glEnd();
      break;
    }
  }
    
  glFlush();
    
  glPopAttrib();
}
  


void SliceRenderer::buildShader( X3DVolumeNode *volume ) {
  // get the shader for the volume to render.
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  MFString *fragmentShaderString = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: SliceRenderer::buildShader( volume ) called with invalid volume" << endl;
    return;
  } else {
    shader = (*i).second->shader.get();
    fragmentShaderString = (*i).second->fragmentShaderString.get();
  }

  shader->uniformFields.clear();
  shader->clearFields();
  shader->parts->clear();

  // setup fragment shader part
  ShaderPart *fs = new ShaderPart;
  fs->type->setValue( "FRAGMENT" );
  fs->setName( getName() + "_fragmentshader" );
  fragmentShaderString->route( fs->url );
  shader->parts->push_back( fs );

  // setup vertex shader part
  ShaderPart *vs = new ShaderPart;    
  vs->type->setValue( "VERTEX" );
  vs->setName( getName() + "_vertexshader" );

  vs->url->push_back( "glsl:"+ SliceRendererInternals::vertexshader_slice );
  shader->parts->push_back( vs );
    
  // setup the fragment shader code
  fragmentShaderString->resize(0);
    
  string fragmentshader;

  // TEMPORARY
  // read fragment shader from file
  /*string stylefunction_file = 
    "C:/Markus/MedX3D/src/shaders/StyleFunctions.glsl";*/
  fragmentshader += H3DVolumeRendererNode::style_function;//readShaderFromFile( stylefunction_file );

 
  /*string main_file = 
      "C:/Markus/MedX3D/src/shaders/Slices_FS_main.glsl";*/
  fragmentshader += SliceRendererInternals::slices_fs_main;//readShaderFromFile( main_file );

  fragmentShaderString->push_back( "glsl:" + fragmentshader );
  // END TEMPORARY
    
  X3DTexture3DNode *tex = volume->voxels->getValue();
  Image *image = tex ? tex->image->getValue() : NULL;
  if( image ) {
    Image::PixelType type = image->pixelType();
    if( type == Image::LUMINANCE ) {  
      insertFragmentShaderCode(fragmentShaderString,
                               "//ORIG_SAMPLE_MANIP", 
                               "orig_sample_color.a = orig_sample_color.r;\n" );
    }
  }

  if( volume->requiresDefaultNormals() ) {
    insertFragmentShaderCode(fragmentShaderString,
                             "//ORIG_SAMPLE_MANIP", 
                             "vec4 sample_default_normal = normalizedNormalFromTexture( " + volume->uniqueShaderName( "defaultNormals" ) + ", r0 );\n" );
  }

  // add the volume node uniforms
  insertFragmentShaderCode(fragmentShaderString, "//GET_ORIG_SAMPLE_COLOR", volume->getOrigSampleColor() );

  // add the volume node uniforms
  insertFragmentShaderCode(fragmentShaderString, "//END UNIFORMS", volume->addUniforms( shader ) );

  // add the functions 
  insertFragmentShaderCode(fragmentShaderString, "//END UNIFORMS", volume->getShaderFunctions() );

  // add shader code
  insertFragmentShaderCode(fragmentShaderString, "//END INSIDE-LOOP", volume->getShaderCode() );

  // add shader code
  insertFragmentShaderCode(fragmentShaderString, "//END PRE-LOOP", volume->getShaderInitCode() );
  
  // TEMPORARY
  // save final fragment shader to file
  /*string saveshaderfile = 
    "c:/tmp" + getName() +".glsl";
  writeShaderToFile( fragmentShaderString->getValue()[0], 
         saveshaderfile );*/
  // END TEMPORARY

  // force relinking
  shader->activate->setValue( true );
}

void SliceRenderer::render( X3DVolumeNode *volume ) {
   // get the shader for the volume to render.
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: SliceRenderer::render( volume ) called with invalid volume" << endl;
    return;
  } else {
    shader = (*i).second->shader.get();
  }


  // select and pre render shader
  shader->setSelected( true );
  shader->preRender();
  // render style
  shader->displayList->callList();

  // if filterMode is something else than default, we want the voxels texture to
  // be rendered with tri-linear interpolation. Filtering will be taken care
  // of in the shader.
  if( volume->filterType->getValue() == "NEAREST" ) {
    // voxel texture is always at texture 0
    glActiveTextureARB(GL_TEXTURE0_ARB );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
  } else if( volume->filterType->getValue() != "DEFAULT" ) {
    // voxel texture is always at texture 0
    glActiveTextureARB(GL_TEXTURE0_ARB );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  }

  int nr_slices = nrSlices->getValue();
  if( nr_slices <= 0 ) {
    Console(3) << "Warning: Invalid nrSlices value in node " << getName()
             << ". nrSlices must be above 0. A default value of 100 will be used." << endl;
    nr_slices = 100;
  }
  renderSlices( volume, nr_slices,  volume->dimensions->getValue() );
  // pop attributes
  shader->postRender();
}
