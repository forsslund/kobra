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
/// \file ShadowCaster.cpp
/// \brief CPP file for ShadowCaster
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ShadowCaster.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/DirectionalLight.h>
#include <H3D/NavigationInfo.h>
#include <H3D/GaussianFilterShader.h>
#include <H3D/RenderTargetTexture.h>
#include <H3D/X3D.h>
#include <H3D/Appearance.h>
#include <H3D/ShadowCasterShaders.h>
#include <H3D/PointLight.h>
#include <H3D/GlobalSettings.h>
#include <H3D/H3DWindowNode.h>
#include <H3D/Scene.h>
#include <H3D/StereoInfo.h>
#include <H3DUtil/ThreadPool.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase ShadowCaster::database( "ShadowCaster", 
                                          &(newInstance<ShadowCaster>), 
                                          typeid( ShadowCaster ),
                                          &X3DChildNode::database );

namespace ShadowCasterInternals {
  FIELDDB_ELEMENT( ShadowCaster, object, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowCaster, light, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowCaster, shadowDarkness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowCaster, shadowDepthOffset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowCaster, algorithm, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowCaster, shadowShader, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowCaster, multiThreading, INPUT_OUTPUT )
}

const string shadow_rectangle = "\
<Shape DEF=\"SHAPE\"> \
  <Appearance DEF=\"APP\"> \
  <RenderProperties blendEnabled=\"true\" blendFuncSrcFactorRGB=\"SRC_ALPHA\" blendFuncDstFactorRGB=\"ONE_MINUS_SRC_ALPHA\"/> \
    <GaussianFilterShader DEF=\"SHADER\" type=\"FULL\"> \
       <RenderTargetTexture DEF=\"TEXTURE\"/> \
    </GaussianFilterShader> \
  </Appearance> \
  <FullscreenRectangle zValue=\"-0.99\"/> \
</Shape>";


ShadowCaster::ShadowCaster( 
                           Inst< SFNode>  _metadata,
                           Inst< MFShadowObjectNode > _object,
                           Inst< MFLightNode        > _light,
                           Inst< SFFloat            > _shadowDarkness,
                           Inst< SFFloat            > _shadowDepthOffset,
                           Inst< DisplayList        > _displayList,
                           Inst< SFString           > _algorithm,
                           Inst< SFShaderNode       > _shadowShader,
                           Inst< SFBool             > _multiThreading ) :
  X3DChildNode( _metadata ),
  H3DDisplayListObject( _displayList ),
  object( _object ),
  light( _light ),
  shadowDarkness( _shadowDarkness ),
  shadowDepthOffset( _shadowDepthOffset ),
  algorithm( _algorithm ),
  generator( NULL ),
  shadowShader( _shadowShader ),
  multiThreading( _multiThreading),
  last_shader( NULL ),
  render_caps_ts( false ),
  should_transfer_data_ts( false ) {
  type_name = "ShadowCaster";
  database.initFields( this );

  displayList->setOwner( this );

  algorithm->addValidValue( "ZPASS" );
  algorithm->addValidValue( "ZFAIL" );
  algorithm->setValue( "ZPASS" );
  shadowDarkness->setValue( 0.4f );
  shadowDepthOffset->setValue( 6 );
  displayList->setCacheMode( DisplayList::OFF );
  
  multiThreading->setValue( false );
}

void ShadowCaster::addHeadLight() {
  bool head_light = false;
  NavigationInfo *ni = NavigationInfo::getActive();
  if( ni ) {
    head_light = 
      ni->headlightShadows->getValue() && 
      ni->headlight->getValue();
  }
  
  if( head_light ) {
    X3DViewpointNode *vp = X3DViewpointNode::getActive();
    Vec3f direction = Vec3f( 0, 0, -1 );
    if( vp ) {
      direction = 
        vp->accForwardMatrix->getValue().getRotationPart() * 
        (vp->totalOrientation->getValue() * Vec3f( 0, 0, -1 ));
    }
    DirectionalLight *dir_light = new DirectionalLight();
    dir_light->direction->setValue( direction );
    light->push_back( dir_light );
  }
}



void ShadowCaster::render() {
  if( X3DShapeNode::geometry_render_mode == X3DShapeNode::SOLID ||
      X3DShapeNode::geometry_render_mode == X3DShapeNode::TRANSPARENT_BACK ||
      object->empty() || light->empty() ) return;
  X3DChildNode::render();

  if( !generator.get() ) {
    // set up a FrameBufferTextureGenerator that renders the current
    // stencil buffer to a color texture to which the shadow shader
    // is applied in order to e.g. blur shadow edges.
    FrameBufferTextureGenerator *gen = new FrameBufferTextureGenerator;
    stringstream s;
    s << "ShadowCaster stencil fbo_" << this << endl;
    gen->setName( s.str() );
    gen->generateColorTextures->push_back( "RGBA" );
    gen->generateDepthTexture->setValue( true );
    gen->depthBufferType->setValue( "DEPTH24_STENCIL8" );
    gen->samples->setValue( 4 );
    gen->depthBufferStorage->setValue( "DEFAULT_COPY" );
    gen->setRenderCallback( renderShadows, this );

    AutoRef<Node> shape_node = X3D::createX3DNodeFromString( shadow_rectangle, &dn );
    shape.reset( static_cast< Shape * > ( shape_node.get() ) );
    GaussianFilterShader *shader;
    RenderTargetTexture *texture;
    dn.getNode( "SHADER", shader );
    dn.getNode( "TEXTURE", texture );
    texture->generator->setValue( gen );
    generator.reset( gen );
    Scene *scene = Scene::scenes.size ( ) > 0 ? *Scene::scenes.begin ( ) : NULL;
    if( !scene ) {
      Console(LogLevel::Error)<<"ShadowCaster has to be within scenes"<<endl;
    }
    H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue ( )[0]);
    if( window ) {
      window->singlePassStereo->route(gen->useStereo);
    }
  }

  X3DShaderNode *shader = shadowShader->getValue();
  
  if( shader ) {
    if( shader != last_shader ) {
      // new shader node, so make sure that the new shader is used 
      // in the FrameBufferTextureGenerator
      SFNode *texture_field = static_cast< SFNode * >(shader->getField( "texture" ));
      RenderTargetTexture *texture;
      dn.getNode( "TEXTURE", texture );
      texture_field->setValue( texture );

      Appearance *app;
      dn.getNode( "APP", app );
      app->shaders->clear();
      app->shaders->push_back( shader );
      last_shader = shader;
    }

    // render mail stencil buffer to fbo RGBA color buffer 
    generator->render();

    // render fbo color buffer with shader applied.
    TraverseInfo tmp_traverse_inf = TraverseInfo( vector< H3DHapticsDevice *>() );
    shape->traverseSG( tmp_traverse_inf );
    shape->render();
  } else {
    // render shadows without fbo
    renderShadows( NULL, 0, this );
  }
}

void ShadowCaster::traverseSG( TraverseInfo &ti ) {
  ti.addPostTraverseCallback( updateShadowInformation, this );
}

void  ShadowCaster::updateShadowInformation( TraverseInfo& ti, void* data ) {

  ShadowCaster *shadow_caster = static_cast< ShadowCaster * >(data);
  if( shadow_caster && !shadow_caster->object->empty()) {
    //clear the buffer before computation
    shadow_caster->single_buffer_map.clear();

    shadow_caster->accumulated_fwd_mat = ti.getAccForwardMatrix();
    // traverse all object nodes
    for( MFShadowObjectNode::const_iterator o = shadow_caster->object->begin();
      o != shadow_caster->object->end(); ++o ) {
      static_cast<H3DShadowObjectNode*>(*o)->update();
    }

    GraphicsOptions *graphics_options = NULL;
    GlobalSettings *default_settings = GlobalSettings::getActive();
    if( default_settings ) {
      default_settings->getOptionNode( graphics_options );
    }

    bool use_geometry_shader =
      GLEW_EXT_geometry_shader4 &&
      (!graphics_options ||
      (graphics_options &&
        graphics_options->defaultShadowGeometryAlgorithm->getValue() == "GEOMETRY_SHADER"));

    bool render_caps = (shadow_caster->algorithm->getValue() == "ZFAIL");

    if( !use_geometry_shader ) {
      if( shadow_caster->multiThreading->getValue() ) {
        shadow_caster->fillThreadSafeInformation( shadow_caster->accumulated_fwd_mat, render_caps );
        ThreadPool::global_pool.executeFunction( &collectInformationCallback, shadow_caster );
      } else {
        // single buffer data collection
        for( MFLightNode::const_iterator l = shadow_caster->light->begin(); l != shadow_caster->light->end(); l++ ) {
          X3DLightNode* light = static_cast<X3DLightNode*>(*l);

          H3DShadowObjectNode::LightDataStruct light_data = H3DShadowObjectNode::LightDataStruct( light );
          // Non-const reference. Access once and modify the reference.
          BufferData& buf_data = shadow_caster->single_buffer_map[light];
          std::vector< Vec4d > coord_buffer;
          // collect all buffer information
          for( MFShadowObjectNode::const_iterator o = shadow_caster->object->begin(); o != shadow_caster->object->end(); ++o ) {
            static_cast<H3DShadowObjectNode*>(*o)->computeShadowVolumeInformationCPU( light_data, shadow_caster->accumulated_fwd_mat, render_caps, coord_buffer );
          }

          // transfer information
          if( !coord_buffer.empty() ) {
            buf_data.coord4.swap( coord_buffer );
          }
        }
      }
    }
  }
}


void ShadowCaster::renderShadows( FrameBufferTextureGenerator *fbo, int i, void *args ) {
  if( fbo ) {
    glClearColor( 0, 0, 0, 0 );
    glClear( GL_COLOR_BUFFER_BIT );
  }
  ShadowCaster *shadow_caster = static_cast< ShadowCaster * >(args);

  // set the darkness so that if a point is in shadow from all lights,
  // the darkness should be shadowDarkness.
  // Basically the solution to (1-x)^nr_lights = (1-shadowDarkness)
  H3DFloat darkness = 1 - H3DPow( 1.f - shadow_caster->shadowDarkness->getValue(),
    1.f / shadow_caster->light->size() );

  GraphicsOptions *graphics_options = NULL;
  GlobalSettings *default_settings = GlobalSettings::getActive();
  if( default_settings ) {
    default_settings->getOptionNode( graphics_options );
  }
  bool use_geometry_shader =
    GLEW_EXT_geometry_shader4 &&
    (!graphics_options ||
    (graphics_options &&
      graphics_options->defaultShadowGeometryAlgorithm->getValue() == "GEOMETRY_SHADER"));

  bool render_caps = (shadow_caster->algorithm->getValue() == "ZFAIL");
  // sync point for multithreading
  if( !use_geometry_shader && shadow_caster->multiThreading->getValue()) {
    shadow_caster->synchronous_lock.lock();
    // check to make sure that data is transfered only once
    if( shadow_caster->should_transfer_data_ts ) {
      for( MFLightNode::const_iterator l = shadow_caster->light->begin(); l != shadow_caster->light->end(); l++ ) {
        X3DLightNode* light = static_cast<X3DLightNode*>(*l);
        std::vector<Vec4d>& coords = shadow_caster->data_container_cpu_ts[light];
        BufferData& buff_data = shadow_caster->single_buffer_map[light];
        // swap info
        buff_data.coord4.swap( coords );
      }
      shadow_caster->should_transfer_data_ts = false;
    }
    shadow_caster->synchronous_lock.unlock();
  }
  // render stack
  for( MFLightNode::const_iterator l = shadow_caster->light->begin();
    l != shadow_caster->light->end(); l++ ) {

    X3DLightNode* light = static_cast<X3DLightNode*>(*l);

    glClear( GL_STENCIL_BUFFER_BIT );
    glPushAttrib( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |
      GL_ENABLE_BIT | GL_POLYGON_BIT | GL_STENCIL_BUFFER_BIT );
    glDisable( GL_LIGHTING );// Turn Off Lighting
    glDepthMask( GL_FALSE );// Turn Off Writing To The Depth-Buffer
    glDepthFunc( GL_LESS );
    glDisable( GL_ALPHA_TEST );
    glEnable( GL_STENCIL_TEST );// Turn On Stencil Buffer Testing
    glColorMask( GL_FALSE, GL_FALSE,
      GL_FALSE, GL_FALSE );// Don't Draw Into The Colour Buffer
    glEnable( GL_CULL_FACE );

    const string &alg = shadow_caster->algorithm->getValue();
    glStencilFunc( GL_ALWAYS, 1, 0xFFFFFFFFL );

    if( alg == "ZPASS" ) {
      glStencilOp( GL_KEEP, GL_KEEP, GL_INCR );
      glCullFace( GL_BACK );
    } else {
      glStencilOp( GL_KEEP, GL_INCR, GL_KEEP );
      glCullFace( GL_FRONT );
    }

    // The shadow geometry does not use the exact same rendering function
    // as the original geometry, since it needs to render triangles facing the
    // light differently from the ones facing away from the light.
    // In order to do this the boundTree triangles of the geometry are used.
    // These triangles have been collected with a HAPI::FeedbackBufferCollector
    // using OpenGL. Somewhere in this process, possibly in the transformation
    // from window coordinates to world coordinates(model view matrix seems ok),
    // the original coordinate used in rendering differs from the coordinate
    // in the boundTree by some epsilon value due to floating point inaccuracies.
    // This means that when drawing caps for ZFAIL algorithm, we can get a case
    // of z-fighting between the original geometry and the shadow geometry,
    // making the geometry flicker.
    // 
    // In order to avoid this we glPolygonOffset to offset the shadow geometry
    // depth value. The values chosen depends on the scene but according to
    // Mark_Kilgard in the presentation "Shadow mapping with todays OpenGL hardware"
    // the values -1, 4 work well most of the time for shadow mapping. We 
    // noticed however that using the scale part introduces artifacts in the
    // shadow volumes(look like the gaps between edges) so instead we
    // do not use the scale and have a default offset of 6 to compensate.
    glPolygonOffset( 0, shadow_caster->shadowDepthOffset->getValue() );
    glEnable( GL_POLYGON_OFFSET_FILL );


    DirectionalLight *dir_light = dynamic_cast< DirectionalLight * >(*l);
    PointLight *point_light = dynamic_cast< PointLight * >(*l);
    // set up shader for shadow geometries
    Scene *scene = Scene::scenes.size() > 0 ? *Scene::scenes.begin() : NULL;
    if( !scene ) {
      Console( LogLevel::Error ) << "ShadowCaster has to be within scenes" << endl;
    }
    H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue()[0]);
    bool use_sps = window == NULL ? false : window->singlePassStereo->getValue();
    StereoInfo* stereo_info = StereoInfo::getActive();
    float view_shift = stereo_info == NULL ? 0 : stereo_info->matrixViewShift->getValue();
    float proj_shift = stereo_info == NULL ? 0 : stereo_info->matrixProjShift->getValue();
    ShadowCasterShaders::shaderInit( !use_geometry_shader,
                                      alg == "ZFAIL",
                                      dir_light != NULL,
                                      use_sps,
                                      view_shift,
                                      proj_shift );

    if( use_geometry_shader ) {
      if( dir_light ) ShadowCasterShaders::setDirectionalLightDir( dir_light->direction->getValue() );
      else ShadowCasterShaders::setPointLightPosition( point_light->location->getValue() );
    }
    H3DShadowObjectNode::LightDataStruct light_data = H3DShadowObjectNode::LightDataStruct( light );
    // First Pass. Increase Stencil Value In The Shadow 
    // First Pass. Increase Stencil Value In The Shadow 
    if( use_geometry_shader ) {
      for( MFShadowObjectNode::const_iterator o = shadow_caster->object->begin();
        o != shadow_caster->object->end(); o++ ) {
        static_cast< H3DShadowObjectNode * >(*o)->renderShadowGPU( light_data, shadow_caster->accumulated_fwd_mat, render_caps );
      }
    } else {
      BufferData& buf_data = shadow_caster->single_buffer_map[light];
      if( !buf_data.coord4.empty() ) {
        glEnableClientState( GL_VERTEX_ARRAY );
        glVertexPointer( 4, GL_DOUBLE, 0, &(*buf_data.coord4.begin()) );
        glDrawArrays( GL_TRIANGLES, 0, static_cast<GLsizei>(buf_data.coord4.size()) );
        glDisableClientState( GL_VERTEX_ARRAY );
      }
    }

    // Second Pass. Decrease Stencil Value In The Shadow
    if( alg == "ZPASS" ) {
      glStencilOp( GL_KEEP, GL_KEEP, GL_DECR );
      glCullFace( GL_FRONT );
    } else {
      glStencilOp( GL_KEEP, GL_DECR, GL_KEEP );
      glCullFace( GL_BACK );
    }

    if( use_geometry_shader ) {
      for( MFShadowObjectNode::const_iterator o = shadow_caster->object->begin();
        o != shadow_caster->object->end(); o++ ) {
        static_cast< H3DShadowObjectNode * >(*o)->renderShadowGPU( light_data, shadow_caster->accumulated_fwd_mat, render_caps );
      }
    } else {
      BufferData& buf_data = shadow_caster->single_buffer_map[light];
      if( !buf_data.coord4.empty() ) {
        glEnableClientState( GL_VERTEX_ARRAY );
        glVertexPointer( 4, GL_DOUBLE, 0, &(*buf_data.coord4.begin()) );
        glDrawArrays( GL_TRIANGLES, 0, static_cast<GLsizei>(buf_data.coord4.size()) );
        glDisableClientState( GL_VERTEX_ARRAY );
      }
    }

    // Enable Rendering To Colour Buffer For All Components
    glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );

    // Draw A Shadowing Rectangle Covering The Entire Screen
    glColor4f( 0.0f, 0.0f, 0.0f, darkness );

    if( fbo ) {
      // if using fbo the shadow texture is blended at a later stage so we disable it
      // here
      glDisable( GL_BLEND );
    } else {
      glEnable( GL_BLEND );
      glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    }
    glStencilFunc( GL_NOTEQUAL, 0, 0xFFFFFFFFL );
    glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
    glDisable( GL_DEPTH_TEST );
    glDisable( GL_CULL_FACE );

    ShadowCasterShaders::shaderClean();

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode( GL_PROJECTION );
    glPushMatrix();
    glLoadIdentity();
    glBegin( GL_QUADS );
    glVertex3i( -1, -1, -1 );
    glVertex3i( 1, -1, -1 );
    glVertex3i( 1, 1, -1 );
    glVertex3i( -1, 1, -1 );
    glEnd();
    glPopMatrix();
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
    glPopAttrib();
  }
}

void H3D::ShadowCaster::fillThreadSafeInformation( Matrix4f acc_fwd, bool is_render_caps ) {
  lights_ts.clear();
  for( MFLightNode::const_iterator l = light->begin(); l != light->end(); ++l ) {
    X3DLightNode* ln = static_cast<X3DLightNode*>(*l);
    lights_ts.push_back( ln );
  }

  shadow_objects_ts.clear();
  for( MFShadowObjectNode::const_iterator o = object->begin(); o != object->end(); ++o ) {
    H3DShadowObjectNode* shadow_node = static_cast<H3DShadowObjectNode*>(*o);
    shadow_objects_ts.push_back( shadow_node );
  }

  render_caps_ts = is_render_caps;
  accumulated_fwd_ts = acc_fwd;
}

void * H3D::ShadowCaster::collectInformationCallback( void * data ) {
  ShadowCaster* sc = static_cast< ShadowCaster* >(data);
  if( sc ) {
    sc->computeThreadedShadowVolumeInformation();
  }

  return NULL;
}

void H3D::ShadowCaster::computeThreadedShadowVolumeInformation() {
  synchronous_lock.lock();
  // single buffer data collection
  for( std::vector<X3DLightNode*>::const_iterator l = lights_ts.begin(); l != lights_ts.end(); l++ ) {
    H3DShadowObjectNode::LightDataStruct light_data = H3DShadowObjectNode::LightDataStruct( *l );
    // Non-const reference. Access once and modify the reference.
    std::vector<Vec4d>& buff_data = data_container_cpu_ts[*l];
    buff_data.clear();
    // collect all buffer information
    for( MFShadowObjectNode::const_iterator o = object->begin(); o != object->end(); ++o ) {
      static_cast<H3DShadowObjectNode*>(*o)->computeShadowVolumeInformationCPU( light_data, accumulated_fwd_ts, render_caps_ts, buff_data );
    }
  }
  should_transfer_data_ts = true;
  synchronous_lock.unlock();
}