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
/// \file FrameBufferTextureGenerator.cpp
/// \brief CPP file for FrameBufferTextureGenerator.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/FrameBufferTextureGenerator.h>
#include <H3D/GeneratedTexture3D.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/GlobalSettings.h>
#include <H3D/GraphicsOptions.h>
#include <H3D/H3DWindowNode.h>
#include <H3D/Scene.h>
#include <H3D/H3DWindowNode.h>
#include <H3D/H3DNavigation.h>
#include <H3D/X3DShaderNode.h>
#include <H3D/GraphicsHardwareInfo.h>
#include <H3D/X3DProgrammableShaderObject.h>

using namespace H3D;

std::set< FrameBufferTextureGenerator*> FrameBufferTextureGenerator::fbo_nodes;

H3DNodeDatabase FrameBufferTextureGenerator::database(
  "FrameBufferTextureGenerator",
  &newInstance< FrameBufferTextureGenerator >,
  typeid( FrameBufferTextureGenerator ),
  &X3DGroupingNode::database
  );

namespace FrameBufferTextureGeneratorInternals {
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, generateColorTextures, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, generateDepthTexture, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, depthBufferType, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, outputTextureType, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, samples, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, update, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, framesBeforeStop, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, depthTexture, OUTPUT_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, colorTextures, OUTPUT_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, colorTexture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, viewpoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, navigationInfo, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, width, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, height, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, widthInUse, OUTPUT_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, heightInUse, OUTPUT_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, useStereo, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, depthTextureProperties, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, colorTextureProperties, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, background, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, depthBufferStorage, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, externalFBODepthBuffer, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, colorBufferStorages, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, externalFBOColorBuffers, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, useNavigation, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, projectionWidth, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, projectionHeight, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, useSpecifiedClearColor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, clearColor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, useDSA, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, splitScene, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, useScissor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, scissorBoxX, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, scissorBoxY, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, scissorBoxWidth, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, scissorBoxHeight, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, clearColors, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, nrLayers, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, generateStencilMask, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, applyStencilMask, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FrameBufferTextureGenerator, useInverseMasking, INPUT_OUTPUT )
}

FrameBufferTextureGenerator::~FrameBufferTextureGenerator() {
  if( fbo_initialized ) {
    glDeleteFramebuffersEXT( 1, &fbo_id );
    glDeleteFramebuffersEXT( 1, &multi_samples_fbo_id );

    if( !generateDepthTexture->getValue() ) {
      glDeleteRenderbuffersEXT(1, &depth_id);
      glDeleteRenderbuffersEXT(1, &multi_samples_depth_id);
    }

    //glDeleteRenderbuffersEXT(1, &stencil_id);

    for( unsigned int i = 0; i<multi_samples_color_ids.size(); ++i ) {
      glDeleteRenderbuffersEXT(1, &multi_samples_color_ids[i] );
    }
  }
  fbo_nodes.erase( this );
}

FrameBufferTextureGenerator::FrameBufferTextureGenerator( Inst< AddChildren    > _addChildren,
  Inst< RemoveChildren > _removeChildren,
  Inst< MFChild        > _children,
  Inst< SFNode         > _metadata,
  Inst< SFBound        > _bound,
  Inst< SFVec3f        > _bboxCenter,
  Inst< SFVec3f        > _bboxSize,
  Inst< MFString       > _generateColorTextures,
  Inst< SFBool         > _generateDepthTexture,
  Inst< MFTexturePropertiesNode > _colorTextureProperties,
  Inst< SFTexturePropertiesNode > _depthTextureProperties,
  Inst< MFGeneratedTextureNode > _colorTextures,
  Inst< SFGeneratedTextureNode > _colorTexture,
  Inst< SFGeneratedTextureNode > _depthTexture,
  Inst< SFString         > _depthBufferType,
  Inst< SFString         > _outputTextureType,
  Inst< SFInt32        > _samples,
  Inst< UpdateMode     > _update,
  Inst< SFInt32        > _framesBeforeStop,
  Inst< SFViewpointNode > _viewpoint,
  Inst< SFNavigationInfo > _navigationInfo,
  Inst< SFBackgroundNode > _background,
  Inst< SFInt32         > _width,
  Inst< SFInt32         > _height,
  Inst< SFInt32         > _widthInUse,
  Inst< SFInt32         > _heightInUse,
  Inst< SFBool          > _useStereo,
  Inst< SFString        > _depthBufferStorage,
  Inst< SFFrameBufferTextureGeneratorNode > _externalFBODepthBuffer,
  Inst< MFString        > _colorBufferStorages,
  Inst< MFFrameBufferTextureGeneratorNode > _externalFBOColorBuffers,
  Inst< SFBool          > _useNavigation,
  Inst< SFInt32         > _projectionWidth,
  Inst< SFInt32         > _projectionHeight,
  Inst< SFBool          > _useSpecifiedClearColor,
  Inst< SFColorRGBA     > _clearColor,
  Inst< SFBool          > _useDSA,
  Inst< SFBool          > _splitScene,
  Inst< SFBool          > _useScissor,
  Inst< SFInt32         > _scissorBoxX,
  Inst< SFInt32         > _scissorBoxY,
  Inst< SFInt32         > _scissorBoxWidth,
  Inst< SFInt32         > _scissorBoxHeight,
  Inst< MFColorRGBA     > _clearColors,
  Inst< SFInt32         > _nrLayers,
  Inst< SFBool          > _generateStencilMask,
  Inst< SFBool          > _applyStencilMask,
  Inst< SFBool          > _useInverseMasking):
X3DGroupingNode( _addChildren, _removeChildren, _children, _metadata, _bound,
  _bboxCenter, _bboxSize ),
  colorBufferStorages( _colorBufferStorages ),
  depthBufferStorage( _depthBufferStorage ),
  externalFBOColorBuffers( _externalFBOColorBuffers ),
  externalFBODepthBuffer( _externalFBODepthBuffer ),
  generateColorTextures( _generateColorTextures ),
  generateDepthTexture( _generateDepthTexture ),
  colorTextureProperties( _colorTextureProperties ),
  depthTextureProperties( _depthTextureProperties ),
  colorTextures( _colorTextures ),
  colorTexture( _colorTexture ),
  depthTexture( _depthTexture ),
  depthBufferType( _depthBufferType ),
  outputTextureType( _outputTextureType ),
  samples( _samples ),
  update( _update ),
  framesBeforeStop( _framesBeforeStop ),
  viewpoint( _viewpoint ),
  navigationInfo( _navigationInfo ),
  background ( _background ),
  width( _width ),
  height( _height ),
  useScissor( _useScissor ),
  scissorBoxX( _scissorBoxX ),
  scissorBoxY( _scissorBoxY ),
  scissorBoxWidth( _scissorBoxWidth ),
  scissorBoxHeight( _scissorBoxHeight ),
  widthInUse( _widthInUse ),
  heightInUse( _heightInUse ),
  projectionWidth( _projectionWidth ),
  projectionHeight( _projectionHeight ),
  useStereo( _useStereo ),
  useNavigation( _useNavigation ),
  useSpecifiedClearColor( _useSpecifiedClearColor ),
  clearColor(_clearColor),
  clearColors(_clearColors),
  useDSA(_useDSA),
  splitScene(_splitScene),
  generateStencilMask(_generateStencilMask ),
  applyStencilMask(_applyStencilMask ),
  useInverseMasking(_useInverseMasking),
  nrLayers(_nrLayers ),
  fbo_initialized( false ),
  buffers_width(-1),
  buffers_height(-1),
  buffers_depth( -1 ),
  last_resize_success( true ),
  render_func( NULL ),
  render_func_data( NULL ),
  always_use_existing_viewport( false ),
  shadow_caster( new ShadowCaster ),
  child_to_render( new X3DGroupingNode ),
  depthWarningPrinted( new ResetPrintedFlag ),
  colorMismatchWarningPrinted( new ResetPrintedFlag ),
  colorInitWarningPrinted( new ResetPrintedFlags ),
  multi_samples_depth_id( 0 ),
  multi_samples_fbo_id( 0 ),
  fbo_id( 0 ),
  depth_id( 0 ),
  stencil_id( 0 ),
  use_depth_stencil( false ) {
    type_name = "FrameBufferTextureGenerator";
    database.initFields( this );

    shadow_caster->algorithm->setValue( "ZFAIL" );

    generateDepthTexture->setValue( false );
    outputTextureType->addValidValue( "2D" );
    outputTextureType->addValidValue( "2D_RECTANGLE" );
    outputTextureType->addValidValue( "3D" );
    outputTextureType->addValidValue( "2D_ARRAY" );
    outputTextureType->addValidValue( "2D_MULTISAMPLE" );
    outputTextureType->addValidValue( "2D_MULTISAMPLE_ARRAY" );
    outputTextureType->setValue( "2D" );
    samples->setValue( 0 );
    last_samples = 0;
    width->setValue( -1 );
    height->setValue( -1 );
    widthInUse->setValue( -1, id );
    heightInUse->setValue( -1, id );
    projectionWidth->setValue( -1 );
    projectionHeight->setValue( -1 );
    useStereo->setValue( false );
    useNavigation->setValue( false );
    useSpecifiedClearColor->setValue( false );
    clearColor->setValue(RGBA(0,0,0,0));
    nrLayers->setValue ( -1 );

    depthBufferType->addValidValue( "DEPTH" );
    depthBufferType->addValidValue( "DEPTH16" );
    depthBufferType->addValidValue( "DEPTH24" );
    depthBufferType->addValidValue( "DEPTH32" );
    depthBufferType->addValidValue( "DEPTH32F" );
    depthBufferType->addValidValue( "DEPTH_STENCIL" );
    depthBufferType->addValidValue( "DEPTH24_STENCIL8" );
    depthBufferType->setValue( "DEPTH24_STENCIL8" );
    update->addValidValue( "NONE" );
    update->addValidValue( "NEXT_FRAME_ONLY" );
    update->addValidValue( "SPECIFIED_FRAMES_ONLY" );
    update->addValidValue( "ALWAYS" );
    update->addValidValue( "NOW" );
    update->setValue( "ALWAYS" );

    framesBeforeStop->setValue(-1);

    depthBufferStorage->addValidValue( "LOCAL" );
    depthBufferStorage->addValidValue( "DEFAULT_COPY" );
    depthBufferStorage->addValidValue( "FBO_COPY" );
    depthBufferStorage->addValidValue( "FBO_SHARE" );
    depthBufferStorage->setValue( "LOCAL" );

    depthWarningPrinted->setName( "depthWarningPrinted" );
    depthWarningPrinted->setOwner(this);
    depthBufferStorage->route( depthWarningPrinted );
    depthWarningPrinted->setValue(false);

    colorMismatchWarningPrinted->setValue( false );

    colorMismatchWarningPrinted->setName( "colorMismatchWarningPrinted" );
    colorMismatchWarningPrinted->setOwner(this);
    colorBufferStorages->route( colorMismatchWarningPrinted );


    colorInitWarningPrinted->setName("colorInitWarningPrinted");
    colorInitWarningPrinted->setOwner(this);
    colorBufferStorages->route( colorInitWarningPrinted );


    // turn off display list since we want to get new values of the width
    // and height each loop to see if they have changed.
    displayList->setCacheMode( H3DDisplayListObject::DisplayList::OFF );

    fbo_nodes.insert( this );
    support_dsa = false;
    useDSA->setValue(false);
    splitScene->setValue(false);

    useScissor->setValue(false);
    scissorBoxX->setValue( 0 );
    scissorBoxY->setValue( 0 );
    scissorBoxWidth->setValue( -10000 );
    scissorBoxHeight->setValue( -10000 );

    needMultiSample.reset(new NeedMultiSample);
    needMultiSample->setName("needMultiSample");
    needMultiSample->setOwner(this);
    samples->route(needMultiSample);

    getNrSamples.reset(new GetNrSamples);
    getNrSamples->setName("getNrSamples");
    getNrSamples->setOwner(this);
    samples->route(getNrSamples);

    generateStencilMask->setValue( false );
    applyStencilMask->setValue( false );
    useInverseMasking->setValue( false );
}

void FrameBufferTextureGenerator::initialize()
{ // overwrite the initialize function of X3DGrouping node to stop the collecting
  // of bound from the child of FBTG, so local bound will not be routed to main
  // scene.
#ifdef GLEW_ARB_direct_state_access
  if( GLEW_ARB_direct_state_access ) {
    if( !useDSA->getValue() ) {
      support_dsa = false;
      //Console(LogLevel::Error)<<"not support dsa"<<endl;
    }
    else{
      support_dsa = true;
      //Console(LogLevel::Error)<<"support dsa"<<endl;
    }
  }
#endif
  if( depthBufferType->getValue()=="DEPTH24_STENCIL8"
    || depthBufferType->getValue() == "DEPTH_STENCIL" ) {
      use_depth_stencil = true;
  }else{
    use_depth_stencil = false;
  }
  X3DViewpointNode* v = viewpoint->getValue();
  if( v ) {
    v->set_bind->setValue(false);
    this->use_union_bound = false;
    BoxBound *bb = new BoxBound();
    bb->center->setValue( bboxCenter->getValue() );
    bb->size->setValue( bboxSize->getValue() );
    this->bound->setValue( bb );
    X3DChildNode::initialize();
    // add children to the child_to_render grouping node to for collecting local
    // bound.
    child_to_render->use_union_bound = true;
    const NodeVector &c = children->getValue();
    for( unsigned int i = 0; i < c.size(); ++i ) {
      child_to_render->children->push_back(c[i]);
    }
  } else {
    this->use_union_bound = true;
    X3DGroupingNode::initialize();
  }
  NavigationInfo* n = navigationInfo->getValue();
  if( n ) {
    n->set_bind->setValue(false);
  }

  if( n&& !v ) {
    Console(LogLevel::Error)<<"Warning: In FrameBufferTextureGenerator: "<< getName()<<", local"
      <<" navigation info is defined but no local viewpoint is defined and local"
      <<" viewpoint is necessary for local navigation info to be applied!"<<endl;
  }

  X3DBackgroundNode* b = background->getValue();
  if( b ) {
    b->set_bind->setValue( false );
  }

  for( MFNode::const_iterator i = children->begin(), i_end = children->end();
    i != i_end; ++i ) {
      NavigationInfo* n_c = dynamic_cast< NavigationInfo* >( *i );
      X3DViewpointNode* v_c = dynamic_cast< X3DViewpointNode* >( *i );
      X3DBackgroundNode* b_c = dynamic_cast< X3DBackgroundNode* >( *i );
      if( v_c&&!v ) {
        Console(LogLevel::Error)<<"Warning: In FrameBufferTextureGenerator: "<< getName()<<", local viewpoint"
          <<"is defined without setting its containerField!"<<endl;
      }
      if( n_c&&!n ) {
        Console(LogLevel::Error)<<"Warning: In FrameBufferTextureGenerator: "<< getName()<<", local"
          <<" navigation info is defined without setting containerField!"<<endl;
      }
      if( b_c&&!b ) {
        Console(LogLevel::Error)<<"Warning: In FrameBufferTextureGenerator: "<< getName()<<", local"
          <<" background is defined without setting containerField!"<<endl;
      }
  }
  // initialize all necessary color buffer init warning message printed flag to false
  for( size_t i = 0, ilen = colorBufferStorages->getValue().size()+1; i < ilen; ++i ) {
    colorInitWarningPrinted->push_back(false);
  }
  if( generateColorTextures->size()>0 ) {
    clear_color_value.assign( 4 * generateColorTextures->size(), 0 );
  }
}

#ifdef H3D_WINDOWS
#undef max
#endif
void FrameBufferTextureGenerator::traverseSG( TraverseInfo &ti ) {

  if( update->getValue()=="NONE" ) {// if update field is none , skip traverseSG
    return;
  }

  shadow_caster->object->clear();
  shadow_caster->light->clear();
  ShadowCaster *prev_shadow_caster = NULL;
  ti.getUserData( "ShadowCaster",  (void **)&prev_shadow_caster);
  ti.setUserData( "ShadowCaster", shadow_caster.get() );

  // save previous state of multi-pass transparency and reset
  // to false in order to be able to identify if any of the children
  // nodes sets it.
  bool previous_multi_pass  = ti.getMultiPassTransparency();
  ti.setMultiPassTransparency( false );

  // specify fbo_require_stereo data , so its children can use this info to decide
  // whether related shader need to be modified when single pass stereo is also needed
  ti.setUserData("fbo_require_stereo", (void*)(useStereo.get()));

  X3DGroupingNode::traverseSG( ti );

  ti.setUserData("fbo_require_stereo", NULL);

  // add the head light to shadow casting nodes if it is active.
  if( !shadow_caster->object->empty() ) {
    shadow_caster->addHeadLight();
  }

  if( shadow_caster.get() ) {
    shadow_caster->traverseSG( ti );
  }

  // set values back so that the changes made by children are ignored
  ti.setUserData( "ShadowCaster", prev_shadow_caster );
  ti.setMultiPassTransparency( previous_multi_pass );
}

bool FrameBufferTextureGenerator::haveStencilBuffer() {
  return use_depth_stencil;
  /*const string &type = depthBufferType->getValue();
  return type == "DEPTH24_STENCIL8" || type == "DEPTH_STENCIL";*/
}


void FrameBufferTextureGenerator::render()     {

  // Only render the texture once regardless of multi pass
  // rendering state.
  if ( X3DShapeNode::geometry_render_mode != X3DShapeNode::ALL &&
    X3DShapeNode::geometry_render_mode != X3DShapeNode::SOLID ) {
      return;
  }

  if( !GLEW_EXT_framebuffer_object ) {
    Console(LogLevel::Error) << "Warning: Frame Buffer Objects not supported by your graphics card "
      << "(EXT_frame_buffer_object). FrameBufferTextureGenerator nodes will "
      << "not work." << endl;
    return;
  }

  GraphicsOptions *graphics_options = NULL;
  GlobalSettings *default_settings = GlobalSettings::getActive();
  ShadowCaster *current_shadow_caster = shadow_caster.get();
  if( default_settings ) {
    default_settings->getOptionNode( graphics_options );
  }

  // set options for default shadows
  if( graphics_options ) {
    if( !graphics_options->useDefaultShadows->getValue() ) {
      current_shadow_caster = NULL;
    }

    if( current_shadow_caster ) {
      if( haveStencilBuffer() ) {
        current_shadow_caster->shadowDarkness->setValue( graphics_options->defaultShadowDarkness->getValue()  );
        current_shadow_caster->shadowDepthOffset->setValue( graphics_options->defaultShadowDepthOffset->getValue()  );
      } else {
        static bool message_printed = false;
        if( !message_printed && !current_shadow_caster->object->empty() ) {
          Console(LogLevel::Error) << "Warning: Shadows cannot be used with FrameBufferTextureGenerator (" << getName()
            << ") since it does not have a stencil buffer. Make sure that a depthBufferType that supports"
            << " stencil buffer is used, e.g. DEPTH_STENCIL" << endl;
          message_printed = true;
        }
        current_shadow_caster = 0;
      }
    }
  }

  string output_texture_type = outputTextureType->getValue();
  if( output_texture_type == "2D_ARRAY" && !GLEW_EXT_texture_array) {
    Console(LogLevel::Error) << "Warning: Texture arrays not supported by your graphics card "
      << "(EXT_texture_array). FrameBufferTextureGenerator nodes with \"2D_ARRAY\" will "
      << "not work." << endl;
    return;
  } else if( output_texture_type == "2D_RECTANGLE" && !GLEW_ARB_texture_rectangle) {
    Console(LogLevel::Error) << "Warning: Texture rectangles not supported by your graphics card "
      << "(ARB_texture_rectangle). FrameBufferTextureGenerator nodes with \"2D_RECTANGLE\" will "
      << "not work." << endl;
    return;
  } else if( (output_texture_type == "2D_MULTISAMPLE_ARRAY"
    || output_texture_type == "2D_MULTISAMPLE") && !GLEW_ARB_texture_multisample ) {
      Console(LogLevel::Error) << "Warning: Multi-sampled texture is not supported by your graphics card "
        << "(ARB_texture_multisample). FrameBufferTextureGenerator nodes with \"2D_MULTISAMPLE\" or \"2D_MULTISAMPLE_ARRAY\" will "
        << "not work." << endl;
      return;
  }

  if( output_texture_type != "3D" &&
    output_texture_type != "2D_ARRAY" &&
    output_texture_type != "2D_RECTANGLE"&&
    output_texture_type != "2D_MULTISAMPLE"&&
    output_texture_type != "2D_MULTISAMPLE_ARRAY") {
      output_texture_type = "2D";
  }

  /// Check if we need to generate any textures.
  if( generateColorTextures->size() == 0
    && (!generateDepthTexture->getValue()
    || output_texture_type == "3D" )  )
    return;

  const string &update_string = update->getValue();
  if( update_string == "SPECIFIED_FRAMES_ONLY" ) {
    if( framesBeforeStop->getValue()<=1 ) {
      // if frame left is 1 or smaller than 1, set it to NONE to stop it next frame
      // set update to NONE
      update->setValue("NONE");
      framesBeforeStop->setValue(0);
    }else{
      framesBeforeStop->setValue(framesBeforeStop->getValue()-1);
    }
  }else if( update_string == "NEXT_FRAME_ONLY" || update_string == "NOW" ) {
    update->setValue("NONE");
  }else if( update_string == "ALWAYS" ) {
    //continue
  }else if( update_string =="NONE" ) {
    if( fbo_initialized ) {
      return;
    }
  }else{
    Console(LogLevel::Warning) << "Warning: Invalid value for \"update\" field in \""
      << getName() << "\" node (\"" << update_string
      << "\"). Must be one of \"NONE\", \"NEXT_FRAME_ONLY\", , \"NEXT_FRAME_ONLY\""
      << " or \"ALWAYS\". Using \"ALWAYS\" instead." << endl;
  }
  GLint previous_fbo_id;
  glGetIntegerv( GL_DRAW_FRAMEBUFFER_BINDING, &previous_fbo_id );

  // if a viewpoint has been specified use that instead of what has already
  // been set up (current active viewpoint)
  bool have_local_vp = false;
  bool have_local_navi = false;
  X3DViewpointNode* vp = static_cast<X3DViewpointNode*>(viewpoint->getValue());
  if (vp != NULL) {
    have_local_vp = true;
  }
  else {
    vp = X3DViewpointNode::getActive();
  }
  NavigationInfo* nav_info = navigationInfo->getValue();
  if (nav_info != NULL) {
    have_local_navi = true;
  }
  else {
    nav_info = NavigationInfo::getActive();
  }
  if (useNavigation->getValue()) {
    // useNavigation will force the use of main scene viewpoint to have user navigation
    have_local_vp = false;
    vp = X3DViewpointNode::getActive();
  }

  // Save current state.
  if (have_local_vp) {
    // when there is local viewpoint, push GL_POLYGON_BIT to make use cull face mode will not be
    // affect by main scene rendering when mirroring is specified
    glPushAttrib(GL_TEXTURE_BIT | GL_COLOR_BUFFER_BIT | GL_VIEWPORT_BIT | GL_SCISSOR_BIT | GL_POLYGON_BIT| GL_STENCIL_BUFFER_BIT );
    // ignore mirroring in fbtg node
    glFrontFace(GL_CCW);
  } else {
    // when there is no local viewpoint, the global one will be used, so need to keep the face mode specified
    // in main scene, otherwise the mirroring effect will only flip the object, its cull face won't be flipped
    glPushAttrib(GL_TEXTURE_BIT | GL_COLOR_BUFFER_BIT | GL_VIEWPORT_BIT | GL_SCISSOR_BIT | GL_STENCIL_BUFFER_BIT);
  }

  /// Make sure all textures and buffers are initialized.
  if( !fbo_initialized ) initializeFBO();

  StereoInfo* stereo_info = NULL;
  Scene *scene = Scene::scenes.size ( ) > 0 ? *Scene::scenes.begin ( ) : NULL;
  H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue ( )[0]);
  // get the desired width and height of the buffer in pixels.
  int desired_fbo_width  = width->getValue();
  int desired_fbo_heigth = height->getValue();

  // the fbo copying source staring point  when FBTG is not creating a new fbo
  int buffer_src_x = window->fbo_current_x;
  int buffer_src_y = window->fbo_current_y;


  // specially handle the width and height being used to provide
  // easy fbo width scaling down
  if( desired_fbo_width <=-1 &&
    desired_fbo_heigth<=-1 ) {
      desired_fbo_width = window->fbo_current_width / -(desired_fbo_width);
      desired_fbo_heigth = window->fbo_current_height / -(desired_fbo_heigth);
  }

  // current_width and current_height should be set to default viewport width, height
  // when using DEFAULT_COPY option. it necessary for stereo rendering
  vector<string> colorbuffer_storage = colorBufferStorages->getValue();
  bool colorbuffer_default_copy =
    std::find(colorbuffer_storage.begin(), colorbuffer_storage.end(), "DEFAULT_COPY")
    !=colorbuffer_storage.end();
  if( colorbuffer_default_copy||depthBufferStorage->getValue()=="DEFAULT_COPY" ) {
    desired_fbo_width = window->fbo_current_width;
    desired_fbo_heigth = window->fbo_current_height;
  }



  bool using_stencil_buffer = haveStencilBuffer();
  if( useStereo->getValue()&&window->renderMode->isSinglePass()&&window->renderMode->isStereoMode() ) {
    // main scene is single pass stereo mode and FBTG intend to use stereo , then
    // need to send in multiple viewport to GPU
    H3DFloat viewports_size[12];
    for( int i = 0; i<12; ++i ) {
      viewports_size[i] = window->viewports_size[i];
    }

    // If an explicit size for the FBO is provided, then use this when defining
    // the viewports instead of that defined by the window
    H3DFloat w = (H3DFloat)width->getValue();
    H3DFloat h = (H3DFloat)height->getValue();
    if( w >= 0 && h >= 0 ) {
      H3DFloat scaleX = w / viewports_size[2];
      H3DFloat scaleY = h / viewports_size[3];

      viewports_size[2] = w;
      viewports_size[3] = h;

      viewports_size[4] *= scaleX;
      viewports_size[5] *= scaleY;
      viewports_size[6] *= scaleX;
      viewports_size[7] *= scaleY;

      viewports_size[8] *= scaleX;
      viewports_size[9] *= scaleY;
      viewports_size[10]*= scaleX;
      viewports_size[11]*= scaleY;
    }

    buffer_src_x = 0;
    buffer_src_y = 0;
#ifdef GLEW_ARB_viewport_array
    if( GLEW_ARB_viewport_array ) {
      glViewportArrayv(0,3, viewports_size);
      if( useScissor->getValue() ) {
        setupScissor(true, viewports_size, desired_fbo_width, desired_fbo_heigth );
      }else{
        glDisable(GL_SCISSOR_TEST);
      }

    }else{
#endif
      Console(LogLevel::Error) << "Warning: GL_ARB_viewport_array is not supported by the graphic card. "
        <<"single pass stereo can not be used."<<endl;
#ifdef GLEW_ARB_viewport_array
    }
#endif
  } else if( !always_use_existing_viewport) {
    // Set viewport to span entire frame buffer  to be used as target
    glViewport( 0 , 0, desired_fbo_width, desired_fbo_heigth );
    if( useScissor->getValue() ) {
      setupScissor( false, NULL, desired_fbo_width, desired_fbo_heigth );
    }
  }

  unsigned int current_depth;
  if ( output_texture_type == "2D" || output_texture_type == "2D_RECTANGLE" ) {
    current_depth= 1;
  } else {
    H3DInt32 nr_layers= nrLayers->getValue();
    current_depth= nr_layers > 0 ? nr_layers : std::max( (int)children->size(), 1 );
  }


  if( samples->getValue() != last_samples || buffers_width != desired_fbo_width || buffers_height != desired_fbo_heigth || buffers_depth != current_depth ) {
    last_samples = samples->getValue();
    last_resize_success = resizeBuffers( desired_fbo_width, desired_fbo_heigth, current_depth );
  }

  // Don't do anything if buffer resize had an error.
  if( !last_resize_success ) {
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previous_fbo_id);
    glPopAttrib();
    return;
  }

  // if use multi-sampled render buffer, render the sub-scene
  // into multi_samples_fbo_id as an intermediate step.
  if( needMultiSample->getValue() ) {
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, multi_samples_fbo_id );
  }else{
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, fbo_id );
  }



  // set up to render to all color buffers
  if( GLEW_ARB_draw_buffers ) {
    glDrawBuffers( H3DMax( (int)color_ids.size(), 1 ), &draw_buffers[0]);
  } else {
    Console(LogLevel::Error) << "Warning: Your graphics card does not support multiple "
      << "render targets(ARB_draw_buffers). Only one color texture will"
      << " have update to their values";
  }

  /// Check if we just need to generate depth texture.
  if( generateColorTextures->size() == 0&&generateDepthTexture->getValue() ) {
    glColorMask(false,false,false,false);
  }


  if( have_local_vp||have_local_navi )
  {
    //switch projection, if not necessary, try to avoid this, as it is expensive
    // to do so
    H3DFloat clip_near = (H3DFloat)0.01;
    H3DFloat clip_far = -1;

    if( nav_info ) {
      if( nav_info->visibilityLimit->getValue() > 0 ) {
        clip_far = nav_info->visibilityLimit->getValue();
      }
      if( nav_info->nearVisibilityLimit->getValue() > 0 ) {
        clip_near = nav_info->nearVisibilityLimit->getValue();
      }
    } else { // calculate far and near plane based on the children to be rendered
      H3DWindowNode::calculateFarAndNearPlane( clip_far, clip_near, child_to_render.get(), vp, false );
    }
    if(shadow_caster.get() &&
      !shadow_caster->object->empty()&&
      shadow_caster->algorithm->getValue() == "ZFAIL" ) {
        clip_far = -1;
    }
    X3DViewpointNode::EyeMode eye_mode = X3DViewpointNode::MONO;

    if( useStereo->getValue()&&window->getEyeMode()!=eye_mode ) {
      // when current active eye mode is not MONO and FBO need to use stereo, need to specify
      // stereo_info
      eye_mode = window->getEyeMode ( );
      stereo_info = StereoInfo::getActive();
    }

    // If projection width and height is set locally then use those values
    // else use width and height of window
    int projection_width = projectionWidth->getValue();
    int projection_height = projectionHeight->getValue();
    if (projection_width < 0 || projection_height < 0) {
      projection_width = window->projectionWidth->getValue();
      projection_height = window->projectionHeight->getValue();
    }

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glLoadIdentity();
    vp->setupViewMatrix( eye_mode, stereo_info, false );
    glMatrixMode( GL_PROJECTION );
    glPushMatrix();
    glLoadIdentity();
    vp->setupProjection( eye_mode,
      (H3DFloat) projection_width,
      (H3DFloat) projection_height,
      clip_near, clip_far, stereo_info, false );
  }
  if ( desired_fbo_width != widthInUse->getValue() ) {
    widthInUse->setValue(desired_fbo_width, id);
  }
  if ( desired_fbo_heigth != heightInUse->getValue() ) {
    heightInUse->setValue(desired_fbo_heigth, id);
  }
  if( output_texture_type == "2D" || output_texture_type == "2D_RECTANGLE" || output_texture_type == "2D_MULTISAMPLE" ) {
    // 2D textures. Render all nodes in children field into the textures.

    // render scene.
    if( render_func ) {
      preProcessFBO(buffer_src_x, buffer_src_y, desired_fbo_width,desired_fbo_heigth,current_depth);
      render_func( this, -1, render_func_data );
    } else {
      if( !useSpecifiedClearColor->getValue() ) {
        // Get background and set clear color
        X3DBackgroundNode* bg = background->getValue();
        X3DViewpointNode* bgVP= vp ? vp : X3DViewpointNode::getActive();
        if ( bg && bgVP ) {
          RGBA clear_color = bg->glClearColor();
          glClearColor( clear_color.r, clear_color.g, clear_color.b, clear_color.a );
          for( unsigned int i = 0; i<generateColorTextures->size(); ++i ) {
            clear_color_value[4*i  ] = (GLfloat)clear_color.r;
            clear_color_value[4*i+1] = (GLfloat)clear_color.g;
            clear_color_value[4*i+2] = (GLfloat)clear_color.b;
            clear_color_value[4*i+3] = (GLfloat)clear_color.a;
          }
        }
        // Prepare the fbo for rendering, it will clear or copy or share what frame buffer is being specified
        preProcessFBO(buffer_src_x, buffer_src_y, desired_fbo_width,desired_fbo_heigth,current_depth);
        // Render background
        if ( bg && bgVP ) {
          const Rotation &vp_orientation = bgVP->totalOrientation->getValue();
          const Matrix4f &vp_inv_m = bgVP->accInverseMatrix->getValue();
          Rotation vp_inv_rot = Rotation(vp_inv_m.getRotationPart());

          glMatrixMode(GL_MODELVIEW);
          glPushMatrix();
          glLoadIdentity();
          glRotatef( (H3DFloat) -(180/Constants::pi)*vp_orientation.angle,
            vp_orientation.axis.x,
            vp_orientation.axis.y,
            vp_orientation.axis.z );
          glRotatef( (H3DFloat) (180/Constants::pi)*vp_inv_rot.angle,
            vp_inv_rot.axis.x, vp_inv_rot.axis.y, vp_inv_rot.axis.z );
          glDepthMask( GL_FALSE );
          bg->renderBackground();
          glDepthMask( GL_TRUE );
          glMatrixMode(GL_MODELVIEW);
          glPopMatrix();
        }
      }else{
        // need to extract the clear color and set it, then preProcessFBO
        // which will rely on the clear color being set
        RGBA clear_color = clearColor->getValue();
        if( generateColorTextures->size()>1&&generateColorTextures->size()==clearColors->size() ) {
          // when color texture output is more than one, and have the size as clearColors
          // use clearColors instead
          MFColorRGBA::vector_return_type clear_colors = clearColors->getValue();
          for(unsigned int i = 0; i<generateColorTextures->size(); ++i){
            clear_color_value[4*i  ] = (GLfloat)clear_colors[i].r;
            clear_color_value[4*i+1] = (GLfloat)clear_colors[i].g;
            clear_color_value[4*i+2] = (GLfloat)clear_colors[i].b;
            clear_color_value[4*i+3] = (GLfloat)clear_colors[i].a;
          }
        }else{
          // if colorTexutres is size is not more than one, or clearColors size
          // mismatch colorTextues size, use colorColor instead for all color attachment
          for( unsigned int i = 0; i<generateColorTextures->size(); ++i ) {
            clear_color_value[4*i  ] = (GLfloat)clear_color.r;
            clear_color_value[4*i+1] = (GLfloat)clear_color.g;
            clear_color_value[4*i+2] = (GLfloat)clear_color.b;
            clear_color_value[4*i+3] = (GLfloat)clear_color.a;
          }
        }
        glClearColor( clear_color.r, clear_color.g, clear_color.b, clear_color.a );
        // Prepare the fbo for rendering, it will clear or copy or share what frame buffer is being specified
        preProcessFBO(buffer_src_x, buffer_src_y, desired_fbo_width,desired_fbo_heigth,current_depth);
      }
      if( generateStencilMask->getValue() ) {
        static bool stencil_message_printed = false;
        if( !stencil_message_printed&&!haveStencilBuffer() ) {
          Console( LogLevel::Error ) << "There is no stencil buffer for current FBTG, please change the depth buffer type"
            << " otherwise it is impossible to generate stencil mask" << std::endl;
          stencil_message_printed = true;
        }
        glEnable( GL_STENCIL_TEST );
        glDepthMask( GL_FALSE );  // turn off writing to the depth buffer

        glStencilOpSeparate( GL_FRONT_AND_BACK, GL_ZERO, GL_ZERO, GL_REPLACE );

        glStencilFuncSeparate( GL_FRONT_AND_BACK, GL_ALWAYS, 1, 0xFFFFFFFFL );
      }
      if( applyStencilMask->getValue() ) {
        static bool stencil_apply_message_printed = false;
        if( !stencil_apply_message_printed && !haveStencilBuffer() ) {
          Console( LogLevel::Error ) << "There is no stencil buffer for current FBTG, please change the depth buffer type"
            << " otherwise it is impossible to apply stencil mask" << std::endl;
          stencil_apply_message_printed = true;
        }
        glEnable( GL_STENCIL_TEST );
        if( useInverseMasking->getValue() ) {
          glStencilFuncSeparate( GL_FRONT_AND_BACK, GL_EQUAL, 1, 0xFFFFFFFFL );
        } else {
          glStencilFuncSeparate( GL_FRONT_AND_BACK, GL_EQUAL, 0, 0xFFFFFFFFL );
        }

      }
      if( splitScene->getValue() ) {
        const NodeVector &c = children->getValue();
        for( unsigned int i = 0; i<c.size(); ++i ) {
          GLenum target = GL_COLOR_ATTACHMENT0_EXT+i;
          glDrawBuffer(target);
          if( c[i] ) {
            H3DDisplayListObject *tmp = dynamic_cast< H3DDisplayListObject* >( c[i]);
            if( tmp )
              tmp->displayList->callList();
            else
              c[i]->render();
          }
        }
      }else{
        X3DShapeNode::GeometryRenderMode  m= X3DShapeNode::geometry_render_mode;
        if( children_multi_pass_transparency ) {
          X3DShapeNode::geometry_render_mode = X3DShapeNode::SOLID;
          X3DGroupingNode::render();
          X3DShapeNode::geometry_render_mode = X3DShapeNode::TRANSPARENT_BACK;
          X3DGroupingNode::render();
          X3DShapeNode::geometry_render_mode = X3DShapeNode::TRANSPARENT_FRONT;
          X3DGroupingNode::render();
          X3DShapeNode::geometry_render_mode = X3DShapeNode::ALL;
        } else {
          X3DShapeNode::geometry_render_mode = X3DShapeNode::ALL;
          X3DGroupingNode::render();
        }
        X3DShapeNode::geometry_render_mode= m;
      }
      if( generateStencilMask->getValue() ) {
        glDepthMask( GL_TRUE );
      }


      if( current_shadow_caster && !current_shadow_caster->object->empty() ) current_shadow_caster->render();
    }
#ifdef GLEW_ARB_invalidate_subdata
    if( GLEW_ARB_invalidate_subdata ) {
      if( !(generateDepthTexture->getValue()) ) {
        // if no depth texture is actually need to be used as texture

        if( use_depth_stencil ) {
          GLenum attachments[1] = {GL_DEPTH_STENCIL_ATTACHMENT};
          glInvalidateFramebuffer(GL_FRAMEBUFFER, 1, attachments);
        }else{
          GLenum attachments[1] = {GL_DEPTH_ATTACHMENT};
          glInvalidateFramebuffer(GL_FRAMEBUFFER, 1, attachments);
        }
      }
    }
#endif
    // blit multi sample render buffer to output textures if using multi sampling.
    if( needMultiSample->getValue() ) {
      blitFBOBuffers(multi_samples_fbo_id, fbo_id, 0, 0, desired_fbo_width, desired_fbo_heigth);
    }
  } else {
    // 3D texture. Each child in the children fields is rendered into a different
    // slice in the 3D texture, if it is not layered rendering
    H3DInt32 nr_layers= nrLayers->getValue();

    const NodeVector &c = children->getValue();
    bool use_layered_rendering = false;
    if( (output_texture_type=="2D_ARRAY"||output_texture_type=="2D_MULTISAMPLE_ARRAY")&&nr_layers>0 ) {
      use_layered_rendering = true;
    }
    if( use_layered_rendering ) {
      // layered rendering currently do not support fbo blit in the implementation
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_id);

      if ( useSpecifiedClearColor->getValue() ) {
        RGBA clear_color = clearColor->getValue();
        glClearColor( clear_color.r, clear_color.g, clear_color.b, clear_color.a );
      }

      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
      if( !checkFBOCompleteness() ) {
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previous_fbo_id);
        glPopAttrib();
        return;
      }
      if ( splitScene->getValue() ) {
        const NodeVector &c = children->getValue();
        for ( unsigned int i = 0; i<c.size(); ++i ) {
          GLenum target = GL_COLOR_ATTACHMENT0_EXT+i;
          glDrawBuffer( target );
          if ( c[i] ) {
            H3DDisplayListObject *tmp = dynamic_cast<H3DDisplayListObject*>(c[i]);
            if ( tmp )
              tmp->displayList->callList();
            else
              c[i]->render();
          }
        }
      } else {
        X3DGroupingNode::render();
      }
      if( current_shadow_caster && !current_shadow_caster->object->empty() ) current_shadow_caster->render();
    }else{ // render every child to its own slice
      for( unsigned int i = 0; i < c.size(); ++i ) {
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_id);
        // set the render target to the correct slice for depth texture.
        if( (output_texture_type == "2D_ARRAY"||output_texture_type == "2D_MULTISAMPLE_ARRAY")
          && generateDepthTexture->getValue() ) { // for 3D texture , no need to render depth
            glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, depth_id, 0, i );
            if( using_stencil_buffer ){
              glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, depth_id, 0, i );
            }
        }
        // set the render target to the correct slice for color textures.
        for( unsigned int j = 0; j < color_ids.size(); ++j ) {
          glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT + j,
            color_ids[j], 0, i );
        }
        if( needMultiSample->getValue() ) {
          glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, multi_samples_fbo_id);
        }
        if( !checkFBOCompleteness() ) {
          glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previous_fbo_id);
          glPopAttrib();
          return;
        }
        // render child
        if( render_func ) {
          render_func( this, i, render_func_data );
        } else {
          // Clear buffers.
          glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
          if( c[i] ) {
            H3DDisplayListObject *tmp = dynamic_cast< H3DDisplayListObject* >( c[i]);
            if( tmp )
              tmp->displayList->callList();
            else
              c[i]->render();
          }
        }
        if( current_shadow_caster && !current_shadow_caster->object->empty() ) current_shadow_caster->render();
        // blit multi sample render buffer to output textures if using multi sampling.
        if( needMultiSample->getValue() ) {
          blitFBOBuffers(multi_samples_fbo_id, fbo_id, 0, 0, desired_fbo_width, desired_fbo_heigth);
        }
      }
    }
  }

  if( have_local_navi||have_local_vp ) {
    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
  }

  // reset fbo
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previous_fbo_id);

  // reset previous state.
  glPopAttrib();
}

void FrameBufferTextureGenerator::initializeFBO() {
  if( !fbo_initialized ) {
    glGenFramebuffersEXT(1, &fbo_id);
    glGenFramebuffersEXT(1, &multi_samples_fbo_id);

    createOutputTextures ();

    // create multi sample color and depth texture/renderbuffer as intermedia result
    // If we are to generate depth texture use that id otherwise
    // create a new render buffer.
    if( !generateDepthTexture->getValue() ) {
      glGenRenderbuffersEXT(1, &depth_id);
    }

    //glGenRenderbuffersEXT(1, &stencil_id);
    glGenRenderbuffersEXT(1, &multi_samples_depth_id);

    GLint max_draw_buffers, max_color_attachments;
    glGetIntegerv( GL_MAX_DRAW_BUFFERS_ARB, &max_draw_buffers );
    glGetIntegerv( GL_MAX_COLOR_ATTACHMENTS_EXT, &max_color_attachments );

    const vector< string > &color_texture_types = generateColorTextures->getValue();

    size_t nr_color_textures = color_texture_types.size();
    if( (GLint)nr_color_textures > max_draw_buffers ||
      (GLint)nr_color_textures > max_color_attachments ) {
        nr_color_textures = H3DMin( max_draw_buffers, max_color_attachments );
        Console(LogLevel::Error) << "Warning: Too many color textures. Supported by your graphics card: "
          << nr_color_textures << ". Tried to use: " << color_texture_types.size()
          << ". Additional textures will be ignored(in FrameBufferTextureGenerator). "
          << endl;
    }

    // generate glDrawBuffers input array
    if( nr_color_textures == 0 ) {
      draw_buffers.resize(1);
      draw_buffers[0] = GL_NONE;
    } else {
      draw_buffers.resize(color_texture_types.size());
    }

    // initialize color textures.
    for( size_t i = 0; i < nr_color_textures; ++i ) {
      GLuint ms_id;
      glGenRenderbuffersEXT( 1, &ms_id );
      multi_samples_color_ids.push_back( ms_id );
      draw_buffers[i] = (GLenum)(GL_COLOR_ATTACHMENT0_EXT+i);
    }

    fbo_initialized = true;
  }
}

void FrameBufferTextureGenerator::createOutputTextures () {
  color_ids.clear();
  NodeVector color_textures;
  H3DSingleTextureNode* depth_texture= NULL;

  bool generate_2d = true;
  const string &output_texture_type = outputTextureType->getValue();
  if( output_texture_type == "3D" || output_texture_type == "2D_ARRAY" || output_texture_type == "2D_MULTISAMPLE_ARRAY" ) {
    generate_2d = false;
  } else {
    if( output_texture_type != "2D" && output_texture_type != "2D_RECTANGLE" &&
      output_texture_type != "2D_MULTISAMPLE" ) {
        Console(LogLevel::Error) << "Warning: Invalid outputTextureType value: \"" << output_texture_type
          << "\". Valid values are \"2D\", \"2D_RECTANGLE\", \"2D_MULTISAMPLE\","
          << "\"2D_MULTISAMPLE_ARRAY\", \"2D_ARRAY\" and \"3D\". "
          <<"Using 2D instead(in FrameBufferTextureGenerator node). " << endl;
    }
  }

  // If we are to generate depth texture use that id otherwise
  // create a new render buffer.
  if( generateDepthTexture->getValue() ) {
    if( generate_2d ) {
      GeneratedTexture *tex = new GeneratedTexture;
      // Set a name for the texture, useful for debugging
      tex->setName ( getName() + "_depth" );
      // make sure the texture id is initialized.
      if( output_texture_type == "2D_RECTANGLE" ) {
        tex->ensureInitialized( GL_TEXTURE_RECTANGLE_ARB );
      } else if( output_texture_type =="2D" ) {
        tex->ensureInitialized( GL_TEXTURE_2D );
      } else if ( output_texture_type =="2D_MULTISAMPLE" ){
        tex->ensureInitialized( GL_TEXTURE_2D_MULTISAMPLE );
      }
      depthTextureProperties->route( tex->textureProperties );
      depth_texture= tex;
      depth_id = tex->getTextureId();
    } else {
      if( output_texture_type == "2D_ARRAY" ) {
        GeneratedTexture3D *tex = new GeneratedTexture3D;
        // Set a name for the texture, useful for debugging
        tex->setName ( getName() + "_depth" );
        // make sure the texture id is initialized.
        tex->ensureInitialized( GL_TEXTURE_2D_ARRAY_EXT );
        depthTextureProperties->route( tex->textureProperties );
        depth_texture= tex;
        depth_id = tex->getTextureId();
      } else if( output_texture_type == "2D_MULTISAMPLE_ARRAY" ) {
        GeneratedTexture3D *tex = new GeneratedTexture3D;
        tex->setName( getName() + "_depth" );
        tex->ensureInitialized( GL_TEXTURE_2D_MULTISAMPLE_ARRAY );
        depthTextureProperties->route( tex->textureProperties );
        depth_texture= tex;
        depth_id = tex->getTextureId();
      }
      else if( output_texture_type == "3D" ) {
        Console(LogLevel::Error) << "Warning: 3D depth textures cannot be generated by "
          <<"FrameBufferTextureGenerator. OpenGL does not support it. Depth texture will be undefined" << endl;
      }
    }
  }
  GLint max_draw_buffers, max_color_attachments;
  glGetIntegerv( GL_MAX_DRAW_BUFFERS_ARB, &max_draw_buffers );
  glGetIntegerv( GL_MAX_COLOR_ATTACHMENTS_EXT, &max_color_attachments );

  const vector< string > &color_texture_types = generateColorTextures->getValue();

  size_t nr_color_textures = color_texture_types.size();
  if( (GLint)nr_color_textures > max_draw_buffers ||
    (GLint)nr_color_textures > max_color_attachments ) {
      nr_color_textures = H3DMin( max_draw_buffers, max_draw_buffers );
      Console(LogLevel::Error) << "Warning: Too many color textures. Supported by your graphics card: "
        << nr_color_textures << ". Tried to use: " << color_texture_types.size()
        << ". Additional textures will be ignored(in FrameBufferTextureGenerator). "
        << endl;
  }
  TextureProperties *tp = NULL;

  // initialize color textures.
  for( size_t i = 0; i < nr_color_textures; ++i ) {
    if( i < colorTextureProperties->size() ) {
      tp = colorTextureProperties->getValueByIndex( i );
    }

    if( generate_2d ) {
      GeneratedTexture *tex = new GeneratedTexture;
      // Set a name for the texture, useful for debugging
      stringstream ss;
      ss << getName() << "_color_" << i;
      tex->setName ( ss.str().c_str() );
      // make sure the texture id is initialized.
      if( output_texture_type == "2D_RECTANGLE" ) {
        tex->ensureInitialized( GL_TEXTURE_RECTANGLE_ARB );
      } else if( output_texture_type == "2D" ) {
        tex->ensureInitialized( GL_TEXTURE_2D );
      } else if( output_texture_type =="2D_MULTISAMPLE" ){
        tex->ensureInitialized( GL_TEXTURE_2D_MULTISAMPLE );
      }
      tex->textureProperties->setValue( tp );
      color_textures.push_back( tex );
      color_ids.push_back( tex->getTextureId() );
    } else {
      GeneratedTexture3D *tex = new GeneratedTexture3D;
      // Set a name for the texture, useful for debugging
      stringstream ss;
      ss << getName() << "_color_" << i;
      tex->setName ( ss.str().c_str() );
      // make sure the texture id is initialized.
      if( output_texture_type == "2D_ARRAY" ) {
        tex->ensureInitialized( GL_TEXTURE_2D_ARRAY_EXT );
      } else if( output_texture_type == "2D_MULTISAMPLE_ARRAY" ) {
        tex->ensureInitialized( GL_TEXTURE_2D_MULTISAMPLE_ARRAY );
      }else {
        tex->ensureInitialized( GL_TEXTURE_3D );
      }
      tex->textureProperties->setValue( tp );
      color_textures.push_back( tex );
      color_ids.push_back( tex->getTextureId() );
    }
  }
  colorTextures->setValue ( color_textures, id );
  depthTexture->setValue ( depth_texture, id );

  if (!colorTextures->empty())
    colorTexture->setValue(colorTextures->front());
}

void FrameBufferTextureGenerator::preProcessFBO(int x, int y,int w, int h, int d){
  depthWarningPrinted->upToDate();
  colorMismatchWarningPrinted->upToDate();
  colorInitWarningPrinted->upToDate();
  const string &output_texture_type = outputTextureType->getValue();

  Scene *scene = Scene::scenes.size() > 0?*Scene::scenes.begin():NULL;
  H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue()[0]);

  // OpenGL doesn't allow blitting when num samples are mismatched,
  // except for 0 -> X, or implementation specific cases.
  bool can_blit = (getNrSamples->getValue() == 0 ||
    getNrSamples->getValue() == window->numSamples->getValue());

  // prepare the fbo_id, multi_sample_fbo_id as user specified
  GLuint target_fbo;
  // the multi_sample_fbo_id will be used as the starting point for rendering.
  // as multi_sample_fbo_id use render buffer as color buffer and depth buffer
  // FBO_SHARE option for depth buffer and FBO_SHARE_x for color buffer will
  // not be valid, they will be copied anyway.
  target_fbo = needMultiSample->getValue()? multi_samples_fbo_id:fbo_id;

  bool using_stencil_buffer = (depthBufferType->getValue() == "DEPTH24_STENCIL8" ||
    depthBufferType->getValue() == "DEPTH_STENCIL");

  GLenum texture_type = GL_TEXTURE_2D;
  if( output_texture_type == "2D_RECTANGLE" ) {
    texture_type = GL_TEXTURE_RECTANGLE_ARB;
  } else if (output_texture_type == "2D_MULTISAMPLE"){
    texture_type = GL_TEXTURE_2D_MULTISAMPLE;
  }else if( output_texture_type == "3D" ) {
    texture_type = GL_TEXTURE_3D;
  } else if( output_texture_type == "2D_ARRAY" ) {
    texture_type = GL_TEXTURE_2D_ARRAY_EXT;
  } else if( output_texture_type == "2D_MULTISAMPLE_ARRAY" ) {
    texture_type = GL_TEXTURE_2D_MULTISAMPLE_ARRAY;
  }

  // pre process depth buffer of fbo

  FrameBufferTextureGenerator* external_FBO_depth =
    dynamic_cast<FrameBufferTextureGenerator*>(externalFBODepthBuffer->getValue());
  std::string depth_buffer_storage = depthBufferStorage->getValue();

  if( depth_buffer_storage.empty()|| depth_buffer_storage == "LOCAL" ) {
    // use locally created depth buffer, just clear the buffer for present
    clearBuffers(target_fbo, 0, 0, w, h, GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
  } else if( depth_buffer_storage == "DEFAULT_COPY" ) {
    // OpenGL doesn't allow blitting when num samples are mismatched,
    // except for some very specific cases like 4x -> 0x or 0x -> 4x.
    // But we have no guarantee that future default FBO will also use 4x
    // samples, so we leave it at only allowing default copy if samples = 0.
    if(!can_blit) {
      if(!depthWarningPrinted->getValue()) {
        Console(LogLevel::Error) << "Warning: Can only copy depth from default"
        " FBO if numSamples == 0 or it matches H3DWindowNode::numSamples." << std::endl;
        depthWarningPrinted->setValue(true);
      }

      // use locally created depth buffer, just clear the buffer for present
      clearBuffers(target_fbo, 0, 0, w, h, GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
    } else {
      // copy depth buffer from default frame buffer
      blitDepthBuffer(0,target_fbo,x,y,w,h);
    }
  } else if( depth_buffer_storage == "FBO_SHARE" ) {
    // use depth buffer from external frame buffer
    if( !external_FBO_depth ) { // however no external fbo is set
      if( !depthWarningPrinted->getValue() ) {
        Console(LogLevel::Error)<< "Warning: There is no external fbo set to be used for sharing "
          << "please add one, if it has been forgotten!" << endl;
        depthWarningPrinted->setValue(true);
      }
      clearBuffers(target_fbo,0,0,w,h, GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
    } else { // bind depth buffer from external fbo to internal fbo
      // update the depth_id and depth tex of internal fbo bounded depth buffer.
      if( can_blit ) {
        if( !depthWarningPrinted->getValue() ) {
          Console(LogLevel::Error)<< "Warning: Multi-sampled FBO can not share depth buffer "
            << "will use FBO_COPY instead" <<std::endl;
          depthWarningPrinted->setValue(true);
        }
        GLuint external_FBO_id_depth = external_FBO_depth->getFBOId();
        blitDepthBuffer(external_FBO_id_depth, target_fbo, 0, 0, w, h);
      } else { // apply FBO_SHARE option
        GeneratedTexture* external_FBO_depth_tex =
          static_cast<GeneratedTexture*>(external_FBO_depth ->getDepthTexture());
        GLenum external_depth_target = external_FBO_depth_tex->getTextureTarget();
        GLenum depth_target = depthTexture->getValue()->getTextureTarget();
        if( external_depth_target != depth_target ) {
          if( !depthWarningPrinted->getValue() ) {
            Console(LogLevel::Error)<< "Warning, the external specified texture target:["
              << external_depth_target << "] is not the same as "
              << "internal depth texture, depth texture sharing fail."
              << std::endl;
            depthWarningPrinted->setValue(true);
          }
          // clear local depth buffer when sharing fail
          clearBuffers(target_fbo,0,0,w,h, GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
        } else {
          depthTexture->setValue(external_FBO_depth_tex, id);
          depth_id = external_FBO_depth_tex->getTextureId();
          //glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, target_fbo);
          glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
            texture_type, depth_id, 0 );
          if( using_stencil_buffer ) {
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,
              texture_type, depth_id, 0);
          }
        }
      }
    }
  } else if( depth_buffer_storage == "FBO_COPY" ) {
    // blit depth buffer from external frame buffer
    if( !external_FBO_depth ) {
      // however no external fbo is set
      if( !depthWarningPrinted->getValue() ) {
        Console(LogLevel::Error)<< "Warning: There is no external fbo set to be copied "
          << "please add one, if it has been forgotten!";
        depthWarningPrinted->setValue(true);
      }
      clearBuffers(target_fbo, 0, 0, w, h, GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
    } else {
      GLuint external_FBO_id_depth = external_FBO_depth->getFBOId();
      blitDepthBuffer(external_FBO_id_depth, target_fbo, 0, 0, w, h);
    }
  } else {
    if( !depthWarningPrinted->getValue() ) {
      Console(LogLevel::Error)  << "The specified depth_buffer_storage value: ["
        <<depth_buffer_storage<<"] is not currently supported"
        << std::endl;
      depthWarningPrinted->setValue(true);
    }
    clearBuffers(target_fbo, 0, 0, w, h, GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
  }

  // pre process color buffer of fbo
  std::vector<std::string> color_buffer_storages = colorBufferStorages->getValue();
  const NodeVector &external_fbo_colors_vector = externalFBOColorBuffers->getValue();

  //GLfloat bkColor[4];
  //glGetFloatv(GL_COLOR_CLEAR_VALUE, bkColor);
  if( color_buffer_storages.empty() ) {
    // no external fbo for color buffers will be used
    // use locally created color buffers, just clear the all color buffers for present
    for( size_t i = 0; i < color_ids.size(); ++i ) {
      clearColorBuffer( target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, static_cast<GLint>(i) );
    }
    return;
  } else { // check color_buffer_storages to decide how to handle color buffers
    GLint end_index = static_cast<GLint>(color_buffer_storages.size());
    if( color_buffer_storages.size()<color_ids.size() ) {
      // not enough storages set for color buffers, just handling those being set.
      if( !colorMismatchWarningPrinted->getValue() ) {
        Console(LogLevel::Error)<<"Warning, number of generated color buffer texture is more "
          <<"than external specified color storage, locally cleared "
          <<"color buffer will be used for extra internal color buffers";
        colorMismatchWarningPrinted->setValue(true);
      }
      end_index = static_cast<GLint>(color_buffer_storages.size());
      for( GLint i = end_index, ilen = static_cast<GLint>(color_ids.size()); i < ilen; ++i ) {
        clearColorBuffer( target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, i );
      }
    } else if( color_buffer_storages.size()>color_ids.size() ) {
      if( !colorMismatchWarningPrinted->getValue() ) {
        Console(LogLevel::Error)<<"Warning, number of generated color buffer texture is "
          <<"less than external specified color storage "
          <<"extra external color storage will be ignored!"<<std::endl;
        colorMismatchWarningPrinted->setValue(true);
      }
      end_index = static_cast<GLint>(color_ids.size());
    } else { // size are the same, no warning
      end_index = static_cast<GLint>(color_ids.size());
    }
    // get the maximum limit of attachment points.
    GLint max_color_attachments = 0;
    if( GraphicsHardwareInfo::infoIsInitialized() ) {
      max_color_attachments = GraphicsHardwareInfo::getInfo().max_color_attachments;
    }else{
      glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &max_color_attachments);
    }
    // need_external_fbo_num is the the num of currently needed external FBO
    // need_external_fbo_num -1 will be the index of currently specified
    // external fbo in external_fbo_colors_vector
    size_t need_external_fbo_num = 0;



    for( GLint i = 0;i < end_index; ++i ) {
      // i is the index for current internal color buffer
      std::string color_buffer_storage = color_buffer_storages[i];
      if( color_buffer_storage.empty() || color_buffer_storage == "LOCAL" ) {
        // clear the color buffer being processed which is i.
        clearColorBuffer( target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, i);
        continue;
      } else {
        // colorBufferStorage is DEFAULT, FBO_COPY_x or FBO_SHARE_x
        if( color_buffer_storage == "DEFAULT_COPY" ) {
          if(!can_blit) {
            if(!colorInitWarningPrinted->getValue()[i]) {
              Console(LogLevel::Error) << "Warning: Can only copy color from default"
                " FBO if numSamples == 0 or it matches H3DWindowNode::numSamples." << std::endl;
              colorInitWarningPrinted->setValue(i, true);
            }

            // clear the color buffer being processed which is i.
            clearColorBuffer( target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, i);

            continue;
          } else {
            // blit color buffer from default frame buffer to the
            // i-th attachment point of internal FBO
            blitColorBuffer(0, target_fbo, x, y, w, h, -1, i);
          }
        } else { // color_buffer_storage can only be FBO_COPY_x or FBO_SHARE_x
          ++need_external_fbo_num;
          std::string style = "";
          // index of the color buffer attachment point of external frame buffer.
          int index = -1;
          parseColorBufferStorage(color_buffer_storage, style, index);
          if( index > max_color_attachments ) {
            if( !colorInitWarningPrinted->getValue()[i] ) {
              Console(LogLevel::Error) << "Warning, the specified attachment index exceeds "
                <<"the maximum limit of color attachments, "
                << "this specified color buffer from external fbo "
                <<"will not be used."<< std::endl;
              colorInitWarningPrinted->setValue(i,true);
            }
            clearColorBuffer(target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, i);
            continue;
          }
          if( style == "SHARE" ) {
            // bind index-th color buffer of external fbo
            // to i th attachment point of internal fbo
            if( external_fbo_colors_vector.size()<need_external_fbo_num ) {
              // there is no external fbo can be used
              if( !colorInitWarningPrinted->getValue()[i] ) {
                Console(LogLevel::Error)<< "Warning: There is not enough external fbo set "
                  << "to be used for sharing "
                  << "please add one, if it has been forgotten!" <<std::endl;
                colorInitWarningPrinted->setValue(i,true);
              }
              clearColorBuffer(target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, i );
              continue;
            } else {
              GeneratedTexture* external_FBO_color_tex =
                static_cast<GeneratedTexture*>(
                static_cast<FrameBufferTextureGenerator*>(
                external_fbo_colors_vector[need_external_fbo_num-1]
              )->getColorTextures()[index]);
              GLuint external_FBO_color_id = external_FBO_color_tex->getTextureId();
              GLenum external_color_target = external_FBO_color_tex->getTextureTarget();
              GLenum color_target = static_cast<H3DSingleTextureNode*>(
                colorTextures->getValue()[i])->getTextureTarget();
              if( (int)static_cast<FrameBufferTextureGenerator*>(
                external_fbo_colors_vector[need_external_fbo_num-1]
              )->getColorIds().size()<=index ) {
                if( !colorInitWarningPrinted->getValue()[i] ) {
                  Console(LogLevel::Error) <<"Warning, the specified color buffer texture index "
                    <<"is smaller than color id numbers of external fbo "
                    <<"will use cleared color buffer instead"<<std::endl;
                  colorInitWarningPrinted->setValue(i,true);
                }
                clearColorBuffer(target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, i );
                continue;
              }
              if( getNrSamples->getValue()>0 ) {
                // use FBO_COPY_x instead, as FBO_SHARE_x will not work
                GLuint external_FBO_id_color =
                  static_cast<FrameBufferTextureGenerator*>(
                  external_fbo_colors_vector[need_external_fbo_num-1])->getFBOId();
                blitColorBuffer(external_FBO_id_color, target_fbo, 0, 0, w, h, index, i );
                continue;
              }
              // replace what is set as the output of depth texture output with shared texture id.
              // update the color_ids[i] and colorTextures[i] of internal fbo bounded depth buffer.
              if( external_color_target!= color_target ) {
                if( !colorInitWarningPrinted->getValue()[i] ) {
                  Console(LogLevel::Error) <<"Warning, the external FBO color texture target "
                    <<"is different from the local color texture"
                    <<std::endl;
                  colorInitWarningPrinted->setValue(i,true);
                }
                clearColorBuffer( target_fbo,0, 0, w, h, &clear_color_value[0]+4*i, i );
                continue;
              }
              color_ids[i] = external_FBO_color_id;
              colorTextures->setValue(i, external_FBO_color_tex, id);
              //glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, target_fbo );
              glBindTextureEXT(texture_type,external_FBO_color_id);
              switch (texture_type)
              {
              case GL_TEXTURE_2D :
                glFramebufferTexture2DEXT( GL_FRAMEBUFFER_EXT,
                  (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
                  texture_type, external_FBO_color_id, 0 );
                break;
              case GL_TEXTURE_RECTANGLE_EXT:
                glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                  (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
                  texture_type, external_FBO_color_id, 0 );
                break;
              case GL_TEXTURE_2D_ARRAY_EXT:
                if ( nrLayers->getValue() > 0 ) {
                  glFramebufferTexture(GL_FRAMEBUFFER_EXT,
                    (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
                    external_FBO_color_id, 0 );
                } else {
                  glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT,
                    (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
                    external_FBO_color_id, 0,0 );
                }
                break;
              case GL_TEXTURE_3D:
                glFramebufferTexture3DEXT(GL_FRAMEBUFFER_EXT,
                  (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
                  texture_type, external_FBO_color_id, 0, 0 );
              }
            }
          } else if( style == "COPY" ) {
            // base is FBO, handling style is COPY, blit index-th color buffer
            // of the external fbo to the i th color buffer of internal fbo
            if( external_fbo_colors_vector.size()<need_external_fbo_num ) {
              if( !colorInitWarningPrinted->getValue()[i] ) {
                Console(LogLevel::Error)<< "Warning: There is no external fbo set to be used for copying "
                  << "please add one, if it has been forgotten!" <<std::endl;
                colorInitWarningPrinted->setValue(i,true);
              }
              clearColorBuffer( target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i,i );
              continue;
            }
            if( (int)static_cast<FrameBufferTextureGenerator*>(
              external_fbo_colors_vector[need_external_fbo_num-1]
            )->getColorIds().size()<=index ) {
              if( !colorInitWarningPrinted->getValue()[i] ) {
                Console(LogLevel::Error) <<"Warning, the specified color buffer texture index "
                  <<"is smaller than color id numbers of external fbo "
                  <<"will use cleared color buffer instead"<<std::endl;
                colorInitWarningPrinted->setValue(i,true);
              }
              clearColorBuffer( target_fbo, 0, 0, w, h, &clear_color_value[0]+4*i, i );
              continue;
            }
            GLuint external_FBO_id_color =
              static_cast<FrameBufferTextureGenerator*>(
              external_fbo_colors_vector[need_external_fbo_num-1])->getFBOId();
            blitColorBuffer(external_FBO_id_color, target_fbo, 0, 0, w, h, index, i );
          }
        }
      }
    }
  }
  // set the draw buffer back to original draw_buffers
  if( GLEW_ARB_draw_buffers ) {
    glDrawBuffers( H3DMax( (int)color_ids.size(), 1 ), &draw_buffers[0]);
  } else {
    Console(LogLevel::Error) << "ERROR: Your graphics card does not support multiple "
      << "render targets(ARB_draw_buffers). Only one color texture will"
      << " have update to their values" << std::endl;
  }
  // depth buffer and color buffers are processed, prepare to be rendered.
  //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previous_fbo_id);
}

bool FrameBufferTextureGenerator::parseColorBufferStorage(std::string color_buffer_storage, std::string& style, int& index){
  // retrieve the handling style and index value of color_buffer_storage, the color_buffer_stroage is FBO_SHARE_x, or FBO_COPY_x.
  std::string base;
  std::size_t found = color_buffer_storage.find("_");
  if(found==std::string::npos){
    // no "_" in the string,  such color_buffer_storage is not currently supported
    Console(LogLevel::Error) << "The color_buffer_storage value:[ "
      <<color_buffer_storage<<" ] is not currently supported." <<std::endl;
    return false;
  }else{// at least one "_" in the string. get the first part as base.
    base = color_buffer_storage.substr(0,found-0);
    if( base!="FBO" ) {
      Console(LogLevel::Error) << "The color_buffer_storage value:[ "
        <<color_buffer_storage<<" ] is not currently supported." <<std::endl;
      return false;
    }
    std::size_t second_found = color_buffer_storage.find("_",found+1);
    if(second_found==std::string::npos){
      // only one "_" exist, such color_buffer_storage is not currently supported
      Console(LogLevel::Error) << "The color_buffer_storage value:[ "
        <<color_buffer_storage<<" ] is not currently supported." <<std::endl;
      return false;
    }else{ // at least two "_" in the string. the string is either FBO_COPY_x or FBO_SHARE_x
      style = color_buffer_storage.substr(found+1,second_found-found-1);
      index = atoi(color_buffer_storage.substr(second_found+1).c_str());
      return true;
    }
  }
}

void FrameBufferTextureGenerator::_check_gl_error(const char *file, int line) {
  GLenum err (glGetError());

  while(err!=GL_NO_ERROR) {
    string error;

    switch(err) {
    case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
    case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
    case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
    case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
    }

    Console(LogLevel::Error) << "GL_" << error.c_str() <<" - "<<file<<":"<<line<<endl;
    err=glGetError();
  }
}

bool FrameBufferTextureGenerator::resizeBuffers( H3DInt32 _width, H3DInt32 _height, H3DInt32 depth ) {
  if( _width*_height==0 ) {
    Console( LogLevel::Warning ) << " FBO should not be attached with zero height or zero width render target, fbo will not get resized at all! " << std::endl;
    return false;
  }
  // specify texture width, height, depth for colorTextures, depthTexture.
  // Also reinit if texture is bindless as bindless textures are immutable once they are resident
  for ( NodeVector::const_iterator i = colorTextures->begin(); i!=colorTextures->end(); ++i ) {
    if ( GeneratedTexture* t = dynamic_cast <GeneratedTexture*> (*i) ) {
      t->setTextureWidth( _width );
      t->setTextureHeight( _height );
      t->setTextureDepth( depth );
      if( X3DProgrammableShaderObject::use_bindless_textures ) {
        t->invalidateTextureHandle();
        t->reinitialize();
      }
    }else if( GeneratedTexture3D* t = dynamic_cast < GeneratedTexture3D* > (*i) ) {
      t->setTextureWidth( _width );
      t->setTextureHeight( _height );
      t->setTextureDepth( depth );
      if( X3DProgrammableShaderObject::use_bindless_textures ) {
        t->invalidateTextureHandle();
        t->reinitialize();
      }
    }
  }
  if ( GeneratedTexture* t = dynamic_cast <GeneratedTexture*> (depthTexture->getValue()) ) {
    t->setTextureWidth( _width );
    t->setTextureHeight( _height );
    if( X3DProgrammableShaderObject::use_bindless_textures ) {
      t->invalidateTextureHandle();
      t->reinitialize();
    }
  } else if ( GeneratedTexture3D* t = dynamic_cast <GeneratedTexture3D*> (depthTexture->getValue()) ) {
    t->setTextureWidth( _width );
    t->setTextureHeight( _height );
    t->setTextureDepth( depth );
    if ( X3DProgrammableShaderObject::use_bindless_textures ) {
      t->invalidateTextureHandle();
      t->reinitialize();
    }
  }


  string output_texture_type = outputTextureType->getValue();
  const vector< string > &color_texture_types = generateColorTextures->getValue();
  bool using_stencil_buffer = haveStencilBuffer();

  GLenum texture_type = GL_TEXTURE_2D;
  if( output_texture_type == "2D_RECTANGLE" ) {
    texture_type = GL_TEXTURE_RECTANGLE_ARB;
  } else if( output_texture_type == "3D" ) {
    texture_type = GL_TEXTURE_3D;
  } else if( output_texture_type == "2D_ARRAY" ) {
    texture_type = GL_TEXTURE_2D_ARRAY_EXT;
  } else if ( output_texture_type == "2D_MULTISAMPLE" ){
    texture_type = GL_TEXTURE_2D_MULTISAMPLE;
  } else if ( output_texture_type == "2D_MULTISAMPLE_ARRAY" ) {
    texture_type = GL_TEXTURE_2D_MULTISAMPLE_ARRAY;
  }

  if( (output_texture_type=="2D_MULTISAMPLE"||output_texture_type=="2D_MULTISAMPLE_ARRAY")&&getNrSamples->getValue()==0  ) {
    samples->setValue(1);
    Console(LogLevel::Error)<<"Warning: when using 2D_MULTISAMPLE or 2D_MULTISAMPLE_ARRAY  as output, the number of samples"
      <<"specified should not be zero, will set the number of sample to be one"<<endl;

  }

  GLenum depth_internal_format = stringToInternalDepthFormat( depthBufferType->getValue() );
  GLenum depth_format = stringToDepthFormat( depthBufferType->getValue() );
  GLenum depth_type = stringToDepthType( depthBufferType->getValue() );
  if( needMultiSample->getValue() ) { // setup multi sample frame buffer object
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, multi_samples_fbo_id);
    // create multi sample render buffers
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, multi_samples_depth_id );
    glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, getNrSamples->getValue(),
      depth_internal_format, _width, _height);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
      GL_RENDERBUFFER_EXT, multi_samples_depth_id);
    if( using_stencil_buffer )
      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,
      GL_RENDERBUFFER_EXT, multi_samples_depth_id);

    for( unsigned int i = 0; i<multi_samples_color_ids.size(); ++i ) {
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, multi_samples_color_ids[i] );
      glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, getNrSamples->getValue(),
        stringToInternalFormat( color_texture_types[i]) ,
        _width, _height);
      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT + i,
        GL_RENDERBUFFER_EXT, multi_samples_color_ids[i]);
    }
    if( !checkFBOCompleteness() ) return false;
  }

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_id);
  // set up depth buffer
  if( generateDepthTexture->getValue()  ) {
    if( texture_type != GL_TEXTURE_3D ) {
      // do not support 3D depth texture
      glBindTexture( texture_type, depth_id );

      if (texture_type!= GL_TEXTURE_2D_MULTISAMPLE&&texture_type!=GL_TEXTURE_2D_MULTISAMPLE_ARRAY){
        // filter needs to be something else than GL_MIPMAP_LINEAR that is default
        // since that is not supported by FBO.
        glTexParameteri(texture_type, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(texture_type, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(texture_type, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(texture_type, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      }

      if( texture_type == GL_TEXTURE_2D_ARRAY_EXT ) {
        glTexImage3D(GL_TEXTURE_2D_ARRAY_EXT, 0, depth_internal_format, _width, _height, depth, 0,
          depth_format, depth_type, NULL);

        if ( nrLayers->getValue() > 0 ) {
          glFramebufferTexture(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, depth_id, 0 );
          if( using_stencil_buffer ) {
            glFramebufferTexture(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,depth_id, 0 );
          }
        } else {
          glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, depth_id, 0, 0 );
          if( using_stencil_buffer ) {
            glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,depth_id, 0, 0 );
          }
        }
      }else if (texture_type==GL_TEXTURE_2D_MULTISAMPLE){
        glTexImage2DMultisample( texture_type, getNrSamples->getValue(), depth_internal_format, _width, _height, GL_TRUE );
        glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, texture_type, depth_id, 0 );
        if (using_stencil_buffer){
          glFramebufferTexture2D( GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT_EXT, texture_type, depth_id, 0 );
        }
      }else if( texture_type==GL_TEXTURE_2D_MULTISAMPLE_ARRAY ) {
        glTexImage3DMultisample( texture_type, getNrSamples->getValue(), depth_internal_format, _width, _height, depth, GL_TRUE );
        if( nrLayers->getValue() > 0 ) {
          glFramebufferTexture( GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, depth_id, 0 );
          if( using_stencil_buffer ) {
            glFramebufferTexture( GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, depth_id, 0 );
          }
        }else{
          glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, depth_id, 0, 0);
          if( using_stencil_buffer ) {
            glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, depth_id, 0, 0);
          }
        }
      }else {
        glTexImage2D( texture_type, 0, depth_internal_format, _width, _height, 0,
          depth_format, depth_type, NULL);
        _check_gl_error(__FILE__, __LINE__);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
          texture_type, depth_id, 0 );

        if( using_stencil_buffer )
          glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,
          texture_type, depth_id, 0);
      }
    }
  } else {
    if (texture_type==GL_TEXTURE_2D_MULTISAMPLE){
      // create multi sample render buffers
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_id );
      glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, getNrSamples->getValue(),
        depth_internal_format, _width, _height);
      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
        GL_RENDERBUFFER_EXT, depth_id);
      if( using_stencil_buffer )
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,
        GL_RENDERBUFFER_EXT, depth_id);
    }else if( texture_type==GL_TEXTURE_2D_MULTISAMPLE_ARRAY ) {
      // currently do not specify depth buffer
      // when it is not required as output for 2D_MULTISAMPLE_ARRAY type,
      // extra renderbuffer here is mostly for depth testing, which seems not
      // really necessary for 2D_MULTISAMPLE_ARRAY type as depth
      // if renderbuffer is specified, it will have framebuffer completeness issue
    }else if( texture_type==GL_TEXTURE_2D_ARRAY_EXT ) {
      // currently do not specify depth buffer with the same reason as above
    }else{
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_id);
      glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, depth_internal_format,
        _width, _height);
      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
        GL_RENDERBUFFER_EXT, depth_id);
      if( using_stencil_buffer )
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,
        GL_RENDERBUFFER_EXT, depth_id);
    }
  }
  // set up stencil buffer
  // NOTE: seems like separate stencil buffers are not supported by current hardware.
  // Instead they use a special format and packs the depth and stencil buffer into
  // the same buffer. So for now the only supported way to have a stencil buffer
  // is to use the DEPTH_STENCIL depthBufferType.
  /*glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, stencil_id);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_STENCIL_INDEX8_EXT,
  _width, _height);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT,
  GL_RENDERBUFFER_EXT, stencil_id);
  */

  // set up color buffers
  for( size_t i = 0; i < color_texture_types.size(); ++i ) {
    glBindTexture( texture_type, color_ids[i] );
    if (texture_type!=GL_TEXTURE_2D_MULTISAMPLE&&texture_type!=GL_TEXTURE_2D_MULTISAMPLE_ARRAY){
      // filter needs to be something else than GL_MIPMAP_LINEAR that is default
      // since that is not supported by FBO.
      glTexParameteri(texture_type, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(texture_type, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(texture_type, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(texture_type, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    GLint internal_format = stringToInternalFormat( color_texture_types[i] );

    if( texture_type == GL_TEXTURE_2D ) {
      glTexImage2D(GL_TEXTURE_2D, 0, internal_format, _width, _height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, NULL);
      _check_gl_error(__FILE__, __LINE__);

      glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
        GL_TEXTURE_2D, color_ids[i], 0 );
    } else if( texture_type == GL_TEXTURE_RECTANGLE_ARB ) {
      glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, internal_format, _width, _height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, NULL);
      _check_gl_error(__FILE__, __LINE__);

      glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
        GL_TEXTURE_RECTANGLE_ARB, color_ids[i], 0 );
    } else if ( texture_type == GL_TEXTURE_2D_MULTISAMPLE ){
      glTexImage2DMultisample( GL_TEXTURE_2D_MULTISAMPLE, getNrSamples->getValue(), internal_format, _width, _height, GL_TRUE );
      glFramebufferTexture2D( GL_FRAMEBUFFER, static_cast<GLenum>(GL_COLOR_ATTACHMENT0 + i), GL_TEXTURE_2D_MULTISAMPLE, color_ids.at(i), 0);
    } else if( texture_type == GL_TEXTURE_2D_MULTISAMPLE_ARRAY ) {
      glTexImage3DMultisample( GL_TEXTURE_2D_MULTISAMPLE_ARRAY, getNrSamples->getValue(), internal_format, _width, _height, depth, GL_TRUE );
      if( nrLayers->getValue() > 0 ) {
        glFramebufferTexture( GL_FRAMEBUFFER_EXT, (GLenum)(GL_COLOR_ATTACHMENT0_EXT + i), color_ids.at(i), 0 );
      }else{
        glFramebufferTextureLayerEXT( GL_FRAMEBUFFER_EXT, (GLenum)(GL_COLOR_ATTACHMENT0_EXT + i), color_ids.at(i), 0, 0 );
      }
    } else if( texture_type == GL_TEXTURE_2D_ARRAY_EXT ) {
      glTexImage3D(GL_TEXTURE_2D_ARRAY_EXT, 0, internal_format, _width, _height, depth, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, NULL);
      if ( nrLayers->getValue() > 0 ) {
        glFramebufferTexture(GL_FRAMEBUFFER_EXT, (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
          color_ids[i], 0 );
      } else {
        glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
          color_ids[i], 0, 0 );
      }
    } else if( texture_type == GL_TEXTURE_3D ) {
      glTexImage3D(GL_TEXTURE_3D, 0, internal_format, _width, _height, depth, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, NULL);
      glFramebufferTexture3DEXT(GL_FRAMEBUFFER_EXT, (GLenum)( GL_COLOR_ATTACHMENT0_EXT + i ),
        GL_TEXTURE_3D, color_ids[i], 0, 0 );
    }
  }

  // No color textures, disable drawing/reading from color buffers.
  if( color_ids.size() == 0 ) {
    glDrawBuffer( GL_NONE );
    glReadBuffer( GL_NONE );
  }

  if( !checkFBOCompleteness() ) return false;

  buffers_width = _width;
  buffers_height = _height;
  buffers_depth = depth;
  return true;
}

bool FrameBufferTextureGenerator::checkFBOCompleteness() {
  // check for errors
  GLenum fbo_err = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if( fbo_err != GL_FRAMEBUFFER_COMPLETE_EXT ) {
    H3DConsole(LogLevel::Error) << "Warning: Frame Buffer Object error: ";
    switch(fbo_err) {
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT :
      Console(LogLevel::Error) << "Attachment not complete" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT :
      Console(LogLevel::Error) << "Wrong size of attachments" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT :
      Console(LogLevel::Error) << "Draw buffer err" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT :
      Console(LogLevel::Error) << "Color attachments have different formats" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT :
      Console(LogLevel::Error) << "No attachments" << endl;
      break;
    case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
      Console(LogLevel::Error) << "Unsupported" << endl;
      break;
    case GL_FRAMEBUFFER_UNDEFINED:
      Console(LogLevel::Error) << "Target is the default framebuffer, but the default framebuffer does not exist"<<endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
      Console(LogLevel::Error) << "Read buffer err" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
      Console(LogLevel::Error) << "mutiple sample setup error" <<endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
      Console(LogLevel::Error) << "layered error" <<endl;
      break;
    default:
      Console(LogLevel::Error) << "Unkown error" << endl;
      break;
    }
    return false;
  }
  return true;
}

GLenum FrameBufferTextureGenerator::stringToInternalFormat( const string &s ) {

  GLint internal_format = GL_RGBA;

  if( s == "RGBA32F" ) {
    if( GLEW_ARB_texture_float ) {
      internal_format = GL_RGBA32F_ARB;
    } else {
      Console(LogLevel::Error) << "Warning: Your graphics card does not support floating point textures (ARB_texture_float). Using RGBA instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "RGBA16F" ) {
    if( GLEW_ARB_texture_float ) {
      internal_format = GL_RGBA16F_ARB;
    } else {
      Console(LogLevel::Error) << "Warning: Your graphics card does not support floating point textures (ARB_texture_float). Using RGBA instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "RGB32F" ) {
    if( GLEW_ARB_texture_float ) {
      internal_format = GL_RGB32F_ARB;
    } else {
      internal_format = GL_RGB;
      Console(LogLevel::Error) << "Warning: Your graphics card does not support floating point textures (ARB_texture_float). Using RGB instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "RGB16F" ) {
    if( GLEW_ARB_texture_float ) {
      internal_format = GL_RGB16F_ARB;
    } else {
      internal_format = GL_RGB;
      Console(LogLevel::Error) << "Warning: Your graphics card does not support floating point textures (ARB_texture_float). Using RGB instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "R32F" ) {
    if( GLEW_ARB_texture_rg ) {
      internal_format = GL_R32F;
    }else{
      internal_format = GL_R8;
      Console(LogLevel::Error)<< "Warning: Your graphics card does not support floating point RED. Using R8 (8bit red channel) format instead(in FrameBufferTextureGenerator node). " << endl;
    }
  }
  else if( s == "RGB" ) {
    internal_format = GL_RGB;
  } else {
    if( s != "RGBA" ) {
      Console(LogLevel::Error) << "Warning: Invalid generateColorTextures value: \"" << s
        << "\". Using \"RGBA\" instead(in FrameBufferTextureGenerator node). " << endl;
    }
  }

  return internal_format;
}

GLenum FrameBufferTextureGenerator::stringToDepthFormat( const string &s ) {
  GLenum format = GL_DEPTH_COMPONENT;
  if( haveStencilBuffer() &&
    GLEW_EXT_packed_depth_stencil ) {
      format = GL_DEPTH_STENCIL_EXT;
  }
  return format;
}

GLenum FrameBufferTextureGenerator::stringToDepthType( const string &s ) {
  GLenum type = GL_FLOAT;
  if( haveStencilBuffer() &&
    GLEW_EXT_packed_depth_stencil ) {
      type = GL_UNSIGNED_INT_24_8_EXT;
  }
  return type;
}

GLenum FrameBufferTextureGenerator::stringToInternalDepthFormat( const string &s ) {

  GLenum internal_format = GL_DEPTH_COMPONENT;

  if( s == "DEPTH16" ) {
    if( GLEW_ARB_depth_texture ) {
      internal_format = GL_DEPTH_COMPONENT16_ARB;
    } else {
      Console(LogLevel::Error)  << "Warning: Your graphics card does not support depth "
        << "textures (ARB_depth_texture). Using DEPTH instead"
        << "(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "DEPTH24" ) {
    if( GLEW_ARB_depth_texture ) {
      internal_format = GL_DEPTH_COMPONENT24_ARB;
    } else {
      Console(LogLevel::Error) << "Warning: Your graphics card does not support depth textures (ARB_depth_texture). Using DEPTH instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "DEPTH32" ) {
    if( GLEW_ARB_depth_texture ) {
      internal_format = GL_DEPTH_COMPONENT32_ARB;
    } else {
      Console(LogLevel::Error) << "Warning: Your graphics card does not support depth textures (ARB_depth_texture). Using DEPTH instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "DEPTH32F" ) {
    if( GLEW_ARB_depth_buffer_float ) {
      internal_format = GL_DEPTH_COMPONENT32F;
    } else {
      Console(LogLevel::Error) << "Warning: Your graphics card does not support floating point depth textures (ARB_depth_buffer_float). Using DEPTH instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "DEPTH24_STENCIL8" ) {
    if( GLEW_EXT_packed_depth_stencil ) {
      internal_format = GL_DEPTH24_STENCIL8_EXT;
    } else {
      Console(LogLevel::Error) << "Warning: Your graphics card does not support packed depth stencil buffers(EXT_packed_depth_stencil). Using DEPTH instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else if( s == "DEPTH_STENCIL" ) {
    if( GLEW_EXT_packed_depth_stencil ) {
      internal_format = GL_DEPTH_STENCIL_EXT;
    } else {
      Console(LogLevel::Error) << "Warning: Your graphics card does not support packed depth stencil buffers(EXT_packed_depth_stencil). Using DEPTH instead(in FrameBufferTextureGenerator node). " << endl;
    }
  } else {
    if( s != "DEPTH" ) {
      Console(LogLevel::Error) << "Warning: Invalid depthBufferType value: \"" << s
        << "\". Using \"DEPTH\" instead(in FrameBufferTextureGenerator node). " << endl;
    }
  }

  return internal_format;
}

void FrameBufferTextureGenerator::clearBuffers(GLenum src, int x, int y,
  int _width, int _height, GLbitfield mask){
    // clear buffer defined by mask of the area defined by x, y, width, height
    //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, src);
    glPushAttrib( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_SCISSOR_BIT );
    glScissor( x, y, _width, _height );
    glEnable( GL_SCISSOR_TEST );
    glClear( mask );
    glDisable( GL_SCISSOR_TEST );
    glPopAttrib();
}

void FrameBufferTextureGenerator::clearColorBuffer( GLenum src, int x, int y,
  int _width, int _height, GLfloat* value, GLint index ){
    // clear index th attached color buffer
    //glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, src );
    glPushAttrib( GL_COLOR_BUFFER_BIT | GL_SCISSOR_BIT );
    glScissor( x, y, _width, _height );
    glEnable( GL_SCISSOR_TEST );
    glClearBufferfv( GL_COLOR, index, value );
    glDisable( GL_SCISSOR_TEST );
    glPopAttrib();
}


void FrameBufferTextureGenerator::setRenderCallback( RenderCallbackFunc func,
  void *args ) {
    render_func = func;
    render_func_data = args;
}

void FrameBufferTextureGenerator::blitDepthBuffer(GLenum src, GLenum dst,
  int srcX, int srcY, int w, int h){
    if( support_dsa ) {
#ifdef GL_ARB_direct_state_access
      if( use_depth_stencil ) {
        glBlitNamedFramebuffer(src, dst, srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, GL_NEAREST);
      }else{
        glBlitNamedFramebuffer(src, dst, srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT , GL_NEAREST);
      }
#else
      Console(LogLevel::Warning) << "Warning: H3DAPI built without direct state access. Update to a newer glew version and rebuild." << endl;
#endif
    }else{
      glBindFramebufferEXT( GL_READ_FRAMEBUFFER_EXT, src );
      glBindFramebufferEXT( GL_DRAW_FRAMEBUFFER_EXT, dst );
      if( use_depth_stencil ) {
        glBlitFramebufferEXT( srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, GL_NEAREST);
      }else{
        glBlitFramebufferEXT( srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT , GL_NEAREST);
      }
    }
}

void FrameBufferTextureGenerator::blitColorBuffer(GLenum src, GLenum dst,
  int srcX, int srcY, int w, int h, int src_index, int dst_index){
    if( support_dsa ) {
#ifdef GL_ARB_direct_state_access
      if( src_index == -1 ) {
        Scene *scene = Scene::scenes.size() > 0 ? *Scene::scenes.begin(): NULL;
        H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue()[0]);
        glNamedFramebufferReadBuffer(src, GL_BACK_LEFT);
        if( window->renderMode->getValue() == "QUAD_BUFFERED_STEREO" ) {
          X3DViewpointNode::EyeMode eye_mode = window->getEyeMode();
          if( eye_mode== X3DViewpointNode::RIGHT_EYE ) {
            glNamedFramebufferReadBuffer(src, GL_BACK_RIGHT);
          }
        }
      }else{
        glNamedFramebufferReadBuffer(src, GL_COLOR_ATTACHMENT0+src_index);
      }
      glNamedFramebufferDrawBuffer(dst, GL_COLOR_ATTACHMENT0 + dst_index);
      glBlitNamedFramebuffer(src, dst, srcX, srcY, srcX+w, srcY+h,
        0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
#else
      Console(LogLevel::Warning) << "Warning: H3DAPI built without direct state access. Update to a newer glew version and rebuild." << endl;
#endif
    }else{
      glBindFramebufferEXT( GL_READ_FRAMEBUFFER_EXT, src);
      glBindFramebufferEXT( GL_DRAW_FRAMEBUFFER_EXT, dst );
      if( src_index == -1 ) {
        Scene *scene = Scene::scenes.size() > 0 ? *Scene::scenes.begin(): NULL;
        H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue()[0]);
        glReadBuffer(GL_BACK_LEFT);
        if( window->renderMode->getValue() == "QUAD_BUFFERED_STEREO" ) {
          X3DViewpointNode::EyeMode eye_mode = window->getEyeMode();
          if( eye_mode== X3DViewpointNode::RIGHT_EYE ) {
            glReadBuffer(GL_BACK_RIGHT);
          }
        }
      } else {
        glReadBuffer( GL_COLOR_ATTACHMENT0_EXT + src_index );
      }
      glDrawBuffer( GL_COLOR_ATTACHMENT0_EXT + dst_index );
      glBlitFramebufferEXT( srcX, srcY, srcX+w, srcY+h,
        0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
}

void FrameBufferTextureGenerator::blitFBOBuffers(GLenum src, GLenum dst,
  int srcX, int srcY, int w, int h){
    if( useScissor->getValue() ) {
      glPushAttrib(GL_SCISSOR_BIT);
      glDisable(GL_SCISSOR_TEST);
    }
    if( support_dsa ) {
#ifdef GL_ARB_direct_state_access
      // blit depth
      if( use_depth_stencil ) {
        glBlitNamedFramebuffer(src, dst, srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, GL_NEAREST);
      }else{
        glBlitNamedFramebuffer(src, dst, srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT , GL_NEAREST);
      }
      // blit color
      for( unsigned int i = 0; i < color_ids.size(); ++i )  {
        glNamedFramebufferReadBuffer(src, GL_COLOR_ATTACHMENT0 + i);
        glNamedFramebufferDrawBuffer(dst, GL_COLOR_ATTACHMENT0 + i);
        glBlitNamedFramebuffer(src, dst, srcX, srcY, srcX+w, srcY+h,
          0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
      }
#else
      Console(LogLevel::Warning) << "Warning: H3DAPI built without direct state access. Update to a newer glew version and rebuild." << endl;
#endif
      // invalidate the src fbo after blit
#ifdef GLEW_ARB_direct_state_access
      vector<GLenum> attachments;
      size_t nr_attachments = 0;
      attachments.push_back(GL_DEPTH_ATTACHMENT);
      nr_attachments += 1;
      for( unsigned int i = 0; i < color_ids.size(); ++i ) {
        attachments.push_back(GL_COLOR_ATTACHMENT0 + i);
        nr_attachments += 1;
      }
      if( GLEW_ARB_direct_state_access ) {
        //GLenum attachments[2] = {GL_DEPTH_ATTACHMENT, GL_COLOR_ATTACHMENT0};
        glInvalidateNamedFramebufferData(src, static_cast<GLsizei>(nr_attachments), &attachments[0]);
      }
#endif
    }else{
      glBindFramebufferEXT( GL_READ_FRAMEBUFFER_EXT, src );
      glBindFramebufferEXT( GL_DRAW_FRAMEBUFFER_EXT, dst );
      // blit depth
      if( use_depth_stencil ) {
        glBlitFramebufferEXT( srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, GL_NEAREST);
      }else{
        glBlitFramebufferEXT( srcX, srcY, srcX+w, srcY+h, 0, 0, w, h,
          GL_DEPTH_BUFFER_BIT , GL_NEAREST);
      }
      // blit color
      for( unsigned int i = 0; i < color_ids.size(); ++i ) {
        glReadBuffer( GL_COLOR_ATTACHMENT0_EXT + i );
        glDrawBuffer( GL_COLOR_ATTACHMENT0_EXT + i );
        glBlitFramebufferEXT( srcX, srcY, srcX+w, srcY+h,
          0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
      }
      // invalidate the src fbo after blit
#ifdef GLEW_ARB_invalidate_subdata
      vector<GLenum> attachments;
      size_t nr_attachments = 0;
      attachments.push_back(GL_DEPTH_ATTACHMENT);
      nr_attachments += 1;
      for( unsigned int i = 0; i < color_ids.size(); ++i ) {
        attachments.push_back(GL_COLOR_ATTACHMENT0 + i);
        nr_attachments += 1;
      }
      if( GLEW_ARB_invalidate_subdata ) {
        //GLenum attachments[2] = {GL_DEPTH_ATTACHMENT, GL_COLOR_ATTACHMENT0};
        glInvalidateFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLsizei>(nr_attachments), &attachments[0]);
      }
#endif
    }
    if( useScissor->getValue() ) {
      glPopAttrib();
    }
}

void FrameBufferTextureGenerator::UpdateMode::onNewValue( const std::string& new_value ) {
  if ( new_value == "NOW" ) {
    static_cast < FrameBufferTextureGenerator* > ( getOwner() )->render();
  }
}

void FrameBufferTextureGenerator::setupScissor( bool needSinglePassStereo,
  float* viewports_size, int desired_fbo_width, int desired_fbo_height ){
#ifdef GL_ARB_viewport_array
    if( needSinglePassStereo ) {
      glEnable(GL_SCISSOR_TEST);

      GLint scissorBox_size[12];
      int box_x  = scissorBoxX->getValue();
      int box_y = scissorBoxY->getValue();
      int box_w = scissorBoxWidth->getValue();
      int box_h = scissorBoxHeight->getValue();
      scissorBox_size[0] = (GLint)viewports_size[0];
      scissorBox_size[1] = (GLint)viewports_size[1];
      scissorBox_size[2] = (GLint)viewports_size[2];
      scissorBox_size[3] = (GLint)viewports_size[3];
      for( int i = 1; i<3; ++i ) {
        // only modify the scissor box for the second and third viewport
        scissorBox_size[4*i] = (GLint)viewports_size[4*i]+box_x;
        scissorBox_size[4*i+1] = (GLint)viewports_size[4*i+1]+box_y;
        scissorBox_size[4*i+2] = box_w;
        scissorBox_size[4*i+3] = box_h;
        if( box_x<0 ) {
          scissorBox_size[4*i] = (GLint)viewports_size[4*i]+ (GLint)( viewports_size[4*i+2]*(-(float)box_x/(float)10000.0) );
        }
        if( box_y<0 ) {
          scissorBox_size[4*i+1] = (GLint)viewports_size[4*i+1]+ (GLint)( viewports_size[4*i+3]*(-(float)box_y/(float)10000.0) );
        }
        if( box_w<0 ) {
          scissorBox_size[4*i+2] = (GLint)( viewports_size[4*i+2]*(-(float)box_w/(float)10000.0) );
        }
        if( box_h<0 ) {
          scissorBox_size[4*i+3] = (GLint)( viewports_size[4*i+3]*(-(float)box_h/(float)10000.0) );
        }
      }
      glScissorArrayv( 0, 3, scissorBox_size );
    }else{
#endif
      glEnable(GL_SCISSOR_TEST);
      int box_x  = scissorBoxX->getValue();
      int box_y = scissorBoxY->getValue();
      int box_w = scissorBoxWidth->getValue();
      int box_h = scissorBoxHeight->getValue();
      if( box_x<0 ) {
        box_x = int ( (-(float)box_x/10000.0)*desired_fbo_width );
      }
      if( box_y<0 ) {
        box_y = int ( (-(float)box_y/10000.0)*desired_fbo_height );
      }
      if( box_w<0 ) {
        box_w = int( (-(float)box_w/10000.0)*desired_fbo_width );
      }
      if( box_h<0 ) {
        box_h = int( (-(float)box_h/10000.0)*desired_fbo_height );
      }
      glScissor( box_x, box_y, box_w, box_h  );
#ifdef GL_ARB_viewport_array
    }
#endif
}

void FrameBufferTextureGenerator::NeedMultiSample::update()
{
  FrameBufferTextureGenerator* fbtg = static_cast< FrameBufferTextureGenerator* >(getOwner());
  if( !fbtg ) {
    Console(LogLevel::Error)<<"NeedMultiSample field is not initialized with its FBTG owner!"<<endl;
    return;
  }
  string output_texture_type = fbtg->outputTextureType->getValue();
  int nr_samples = fbtg->getNrSamples->getValue();
  int nr_layers = fbtg->nrLayers->getValue();
  if( nr_samples>0&&output_texture_type!="2D_MULTISAMPLE"&&output_texture_type!="2D_MULTISAMPLE_ARRAY" ) {
    value = true;
    if( nr_layers>0&&output_texture_type=="2D_ARRAY" ) {
      value = false;
      Console(LogLevel::Error)<<"Warning: layered 2d array with multiple sample is not supported, "
        <<"will ignore the samples you just set."
        <<"Please use 2D_MULTISAMPLE_ARRAY as outputTextureType instead!"<<endl;
    }
  }else{
    value = false;
  }
  if( nr_samples ) {
  }
}

void H3D::FrameBufferTextureGenerator::GetNrSamples::update()
{
  // update number of samples when FBTG samples field is updated
  SFInt32* samples = static_cast<SFInt32*>( getRoutesIn().at(0) );
  if( samples ) {
    int nr_samples = samples->getValue();
    int max_samples = GraphicsHardwareInfo::getInfo().max_samples;
    if( nr_samples>max_samples ) {
      Console(LogLevel::Error) << "Warning: Unsupported nr of multi-samples: " << nr_samples
                               << ". Your graphics draw supports a maximum of " << max_samples
                               << " (in FrameBufferTextureGenerator)." << endl;
      value = max_samples;
    }else{
      value = nr_samples;
    }
  }
}
