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
/// \file H3DWindowNode.cpp
/// \brief CPP file for H3DWindowNode.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/OculusRiftHandler.h>
#include <H3D/H3DWindowNode.h>
#include <H3D/Viewpoint.h>
#include <H3D/Bound.h>
#include <H3D/H3DBoundedObject.h>
#include <H3D/GlobalSettings.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/DeviceInfo.h>
#include <H3D/H3DDisplayListObject.h>
#include <H3D/X3DBackgroundNode.h>
#include <H3D/MatrixTransform.h>
#include <H3D/NavigationInfo.h>
#include <H3D/StereoInfo.h>
#include <H3D/H3DMultiPassRenderObject.h>
#include <H3D/Fog.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/Scene.h>
#include <H3D/DebugOptions.h>
#include <H3D/X3DKeyDeviceSensorNode.h>
#include <H3D/MouseSensor.h>
#include <H3D/X3DLightNode.h>
#include <H3D/CollisionOptions.h>
#include <H3D/X3DPointingDeviceSensorNode.h>
#include <H3D/GraphicsOptions.h>
#include <H3D/GraphicsHardwareInfo.h>

#include <H3DUtil/TimeStamp.h>
#include <H3DUtil/Exception.h>
#include <H3DUtil/AutoPtrVector.h>
#include <H3D/H3DNavigation.h>
#include <H3D/X3DLightNode.h>

#include <GL/glew.h>
#include <H3DUtil/H3DTimer.h>
#include <H3DUtil/LoadImageFunctions.h>


using namespace H3D;


/////////////////////////////////////////////////////////////////////////////
//
// Initialize static members
//

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase H3DWindowNode::database( "H3DWindowNode", 
                                         NULL, 
                                         typeid( H3DWindowNode ) );


namespace H3DWindowNodeInternals {
  FIELDDB_ELEMENT( H3DWindowNode, posX, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, posY, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, useFullscreenAntiAliasing, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, width, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, height, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, fullscreen, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, mirrored, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, renderMode, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, viewpoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, manualCursorControl, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, cursorType, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, navigationInfo, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, clipDistances, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, viewportWidth, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, viewportHeight, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, projectionWidth, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, projectionHeight, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, singlePassStereo, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DWindowNode, pointingDeviceRefreshMode, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, resetViewPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWindowNode, numSamples, OUTPUT_ONLY )
}

bool H3DWindowNode::GLEW_init = false;
set< H3DWindowNode* > H3DWindowNode::windows;
bool H3DWindowNode::multi_pass_transparency = false;

H3DWindowNode::H3DWindowNode(
                   Inst< SFInt32     > _width,
                   Inst< SFInt32     > _height,
                   Inst< SFBool      > _fullscreen,
                   Inst< SFBool      > _mirrored,
                   Inst< RenderMode  > _renderMode, 
                   Inst< SFViewpoint > _viewpoint, 
                   Inst< SFInt32     > _posX,
                   Inst< SFInt32     > _posY,
                   Inst< SFBool      > _manualCursorControl,
                   Inst< SFString    > _cursorType,
                   Inst< SFNavigationInfo > _navigationInfo,
                   Inst< SFBool > _useFullscreenAntiAliasing,
                   Inst< SFVec2f > _clipDistances,
                   Inst< SFBool      > _singlePassStereo,
                   Inst< SFString > _pointingDeviceRefreshMode,
                   Inst< SFBool   > _resetViewPoint,
                   Inst< SFInt32 > _numSamples ) :
#ifdef H3D_WINDOWS
  rendering_context( NULL ),
  hWnd( NULL ),
#endif
  posX      ( _posX        ),
  posY      ( _posY        ),
  width     ( _width      ),
  height    ( _height     ),
  fullscreen( _fullscreen ),
  mirrored  ( _mirrored   ),
  renderMode( _renderMode ),
  viewpoint( _viewpoint ),
  manualCursorControl( _manualCursorControl ),
  cursorType( _cursorType ),
  navigationInfo( _navigationInfo ),
  clipDistances( _clipDistances ),
  useFullscreenAntiAliasing( _useFullscreenAntiAliasing ),
  viewportWidth( new SFInt32 ),
  viewportHeight( new SFInt32 ),
  projectionWidth( new SFInt32 ),
  projectionHeight( new SFInt32 ),
  singlePassStereo( _singlePassStereo ),
  pointingDeviceRefreshMode( _pointingDeviceRefreshMode ),
  resetViewPoint(_resetViewPoint),
  numSamples(_numSamples),
  fbo_current_width( 0 ),
  fbo_current_height( 0 ),
  fbo_current_x(0),
  fbo_current_y(0),
  rebuild_stencil_mask( false ),
  last_render_child( NULL ),
  window_id( 0 ),
  stencil_mask( NULL ),
  stencil_mask_height( 0 ),
  stencil_mask_width( 0 ),
  last_loop_mirrored( false ),
  last_render_mode( RenderMode::MONO ),
  eye_mode( X3DViewpointNode::MONO ),
  current_cursor( "DEFAULT" ),
  previous_left_mouse_button( false ),
  h3d_navigation( new H3DNavigation ),
  window_is_made_active( false ),
  check_if_stereo_obtained( false ),
  windowSizeChanged( new WindowSizeChanged ),
  nr_viewports( 0 )
#ifdef HAVE_LIBOVR
  ,oculus( NULL ) 
#endif
{

  type_name = "H3DWindowNode";
  database.initFields( this );



#ifdef H3D_WINDOWS
  windowInstance = GetModuleHandle( NULL );
#endif

  posX->setValue( -1 );
  posY->setValue( -1 );
  width->setValue( 800 );
  height->setValue( 600 );
  fullscreen->setValue( false );
  mirrored->setValue( false );
  clipDistances->setValue( Vec2f( 0.01f, 10 ), id );

  for( int i = 0; i<12; ++i ) {
    viewports_size[i]  = -1;
  }
  singlePassStereo->setValue(false,id);

  renderMode->addValidValue( "MONO" );
  renderMode->addValidValue( "QUAD_BUFFERED_STEREO" );
  renderMode->addValidValue( "VERTICAL_SPLIT" );
  renderMode->addValidValue("VERTICAL_SPLIT_KEEP_RATIO" );
  renderMode->addValidValue("HORIZONTAL_SPLIT" );
  renderMode->addValidValue("HORIZONTAL_SPLIT_KEEP_RATIO" );
  renderMode->addValidValue( "VERTICAL_INTERLACED" );
  renderMode->addValidValue("HORIZONTAL_INTERLACED" );
  renderMode->addValidValue( "CHECKER_INTERLACED" );
  renderMode->addValidValue( "RED_BLUE_STEREO" );
  renderMode->addValidValue("RED_GREEN_STEREO" );
  renderMode->addValidValue( "RED_CYAN_STEREO" );
  renderMode->addValidValue( "VERTICAL_INTERLACED_GREEN_SHIFT" );
  renderMode->addValidValue( "HDMI_FRAME_PACKED_720P" );
  renderMode->addValidValue( "HDMI_FRAME_PACKED_1080P" );
  renderMode->addValidValue( "NVIDIA_3DVISION" );
  renderMode->addValidValue( "VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS" );
  renderMode->addValidValue("OCULUS_RIFT");
  renderMode->setValue( "MONO" );  

  cursorType->addValidValue( "DEFAULT" );
  cursorType->setValue( "DEFAULT" ); 

  pointingDeviceRefreshMode->addValidValue( "MOUSE_MOVE" );
  pointingDeviceRefreshMode->addValidValue( "MOUSE_CLICK" );
  pointingDeviceRefreshMode->setValue( "MOUSE_MOVE" );  

  resetViewPoint->setValue(false);
  numSamples->setValue(0, id);

  useFullscreenAntiAliasing->setValue( true );
  manualCursorControl->setValue( false );

#ifdef H3D_WINDOWS
  wpOrigProc = (WNDPROC)DefWindowProc;
#endif

  windows.insert( this );

  default_nav = "EXAMINE";
  default_avatar.push_back( 0.25f );
  default_avatar.push_back( 1.6f );
  default_avatar.push_back( 0.75f );
  default_speed = 1;
  default_collision = true;
  default_transition_type.push_back( "LINEAR" );
  default_transition_time = 1.0;
  H3DNavigationDevices::setNavTypeForAll( default_nav, h3d_navigation );
  mouse_position[0] = 0;
  mouse_position[1] = 0;
  previous_mouse_position[0] = 0;
  previous_mouse_position[1] = 0;

  windowSizeChanged->setName( "windowSizeChanged" );
  windowSizeChanged->setOwner( this );
  width->route( windowSizeChanged );
  height->route( windowSizeChanged );
}

void H3DWindowNode::deinitWindow() {
#ifdef HAVE_LIBOVR
  if (oculus) {
    delete oculus;
    oculus = NULL;
  }
#endif
}

H3DWindowNode::~H3DWindowNode() {
  deinitWindow();
  delete h3d_navigation;
  h3d_navigation = NULL;
  if( stencil_mask )
    free( stencil_mask );
  windows.erase( this );
}

#ifdef __GNUC__
H3D_PUSH_WARNINGS()
H3D_DISABLE_UNUSED_PARAMETER_WARNING()
#endif
void H3DWindowNode::shareRenderingContext( H3DWindowNode *w ) {
#ifdef H3D_WINDOWS
  BOOL res = wglShareLists( rendering_context, w->getRenderingContext() );
  if( !res ) {
    TCHAR buffer[80]; 
    DWORD dw = GetLastError(); 
    FormatMessage(
                  FORMAT_MESSAGE_FROM_SYSTEM,
                  NULL,
                  dw,
                  0,
                  buffer,
                  80, NULL );
    throw Exception::H3DAPIException( buffer,
                                      H3D_FULL_LOCATION );
  }
#else
  throw Exception::H3DAPIException( "shareRenderingContext not implemented", 
                         H3D_FULL_LOCATION );
#endif // H3D_WINDOWS
}
#ifdef __GNUC__
H3D_POP_WARNINGS()
#endif

void H3DWindowNode::initialize() {
  initWindowHandler();
  initWindowWithContext();

  if( !GLEW_init ) {
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err) {
      stringstream s;
      s << "Glew init error: " << glewGetErrorString( err );
      throw Exception::H3DAPIException( s.str() );
    }
    GLEW_init = true;
  }
    
#ifdef H3D_WINDOWS
  rendering_context = wglGetCurrentContext();
  
#endif // H3D_WINDOWS

  for( set< H3DWindowNode * >::iterator i = windows.begin();
       i != windows.end();
       ++i ) {
    if( (*i)!=this && (*i)->isInitialized() )
      shareRenderingContext( *i );
  }
  
  GraphicsHardwareInfo::initializeInfo();

  // configure OpenGL context for rendering.
  glEnable( GL_DEPTH_TEST );
  glGetError(); // Clear error flag caused by bug in glewInit()
  glDepthFunc( GL_LESS );
  glDepthMask( GL_TRUE );
  glEnable( GL_LIGHTING );
  glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );
  glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );
  GLfloat no_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
  glLightModelfv( GL_LIGHT_MODEL_AMBIENT, no_ambient);
  glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
  glPixelStorei( GL_PACK_ALIGNMENT, 1 );
  if( GLEW_EXT_separate_specular_color ) {
    glLightModeli( GL_LIGHT_MODEL_COLOR_CONTROL_EXT, 
                   GL_SEPARATE_SPECULAR_COLOR_EXT );
  }
  Node::initialize();
  last_render_mode = renderMode->getRenderMode();
}

void renderStyli() {
  // Render the stylus of each H3DHapticsDevice.
  DeviceInfo *di = DeviceInfo::getActive();
  if( di ) {
    di->renderStyli();
  }
}

void renderHapticTriangles() {
  Scene *scene = Scene::scenes.size() > 0 ? *Scene::scenes.begin(): NULL;
  if( !scene ) return;

  TraverseInfo *ti = scene->getLastTraverseInfo();
  if( !ti ) return;

  bool global_render_triangles = false;

  DebugOptions *debug_options = NULL;

  GlobalSettings *default_settings = GlobalSettings::getActive();
  if( default_settings ) {
    default_settings->getOptionNode( debug_options );
  }
  if( debug_options ) 
    global_render_triangles = 
      debug_options->drawHapticTriangles->getValue();

  unsigned int current_layer = ti->getCurrentLayer();

  for( unsigned int di = 0; di < ti->getHapticsDevices().size(); ++di ) {
    for( unsigned int l = 0; l < ti->nrLayers(); ++l ) {
      ti->setCurrentLayer( l );
      const HapticShapeVector &shapes = ti->getHapticShapes( di );
      for( HapticShapeVector::const_iterator i = shapes.begin();
           i != shapes.end(); ++i ) {
        bool render_triangles = global_render_triangles;
        X3DGeometryNode *geom = 
          static_cast< X3DGeometryNode * >( (*i)->getUserData() );
        if( geom ) {
          debug_options = NULL;
          geom->getOptionNode( debug_options );
          if( debug_options ) {
            render_triangles = debug_options->drawHapticTriangles->getValue();
          }
        }
        if( render_triangles ) {
          glMatrixMode( GL_MODELVIEW );
          glPushMatrix();
          glPushAttrib( GL_CURRENT_BIT | GL_ENABLE_BIT | GL_POLYGON_BIT );
          glDisable( GL_LIGHTING );
          glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
          glColor3f( 1, 1, 1 );
          (*i)->glRender();
          glPopAttrib();
          glMatrixMode(GL_MODELVIEW);
          glPopMatrix();
        }
      }
    }
  }

  ti->setCurrentLayer( current_layer );
}

void H3DWindowNode::renderChild( X3DChildNode *c,
                                 bool render_to_fbo ) {
  H3DDisplayListObject *dlo = 
    dynamic_cast< H3DDisplayListObject * >( c );

  renderHapticTriangles();
  if( multi_pass_transparency ) {
    H3DTIMER_BEGIN("Renderpass SOLID","H3D")
    X3DShapeNode::geometry_render_mode = X3DShapeNode::SOLID; 
    renderStyli();
    if( dlo )  dlo->displayList->callList();
    else c->render();
    H3DTIMER_END("Renderpass SOLID");
    H3DTIMER_BEGIN("Renderpass BACK","H3D");
    X3DShapeNode::geometry_render_mode = X3DShapeNode::TRANSPARENT_BACK; 
    renderStyli();
    if( dlo )  dlo->displayList->callList();
    else c->render();
    H3DTIMER_END("Renderpass BACK");
    H3DTIMER_BEGIN("Renderpass FRONT","H3D");
    X3DShapeNode::geometry_render_mode = X3DShapeNode::TRANSPARENT_FRONT; 
    renderStyli();
    if( dlo )  dlo->displayList->callList();
    else c->render();
    X3DShapeNode::geometry_render_mode = X3DShapeNode::ALL; 
    H3DTIMER_END("Renderpass FRONT");
  } else {
    H3DTIMER_BEGIN("Renderpass ALL","H3D")
    X3DShapeNode::geometry_render_mode = X3DShapeNode::ALL; 
    renderStyli();
    if( dlo )  dlo->displayList->callList();
    else c->render();
    H3DTIMER_END("Renderpass ALL")
  }
}

bool H3DWindowNode::calculateFarAndNearPlane( H3DFloat &clip_far,
                                         H3DFloat &clip_near,
                                         X3DChildNode *child_to_render,
                                         X3DViewpointNode *vp,
                                         bool include_styli ) {

  // calculate the far and near clipping planes from the union of the 
  // bounding box of the child_to_render node and the styli of the 
  // haptics devices.

  bool success = true;
  vector< Bound * > bounds;
  AutoPtrVector< Bound > bounds_to_remove;
  
  if( include_styli ) {
    // get the Bounds of the styli of the H3DHapticsDevices.
    DeviceInfo *di = DeviceInfo::getActive();
    if( di ) {
      for( DeviceInfo::MFDevice::const_iterator i = di->device->begin();
           i != di->device->end();
           ++i ) {
        H3DHapticsDevice *hd = static_cast< H3DHapticsDevice * >( *i );
        H3DBoundedObject *stylus_bound = 
          dynamic_cast< H3DBoundedObject * >( hd->stylus->getValue() );
        MatrixTransform *mt = 
          dynamic_cast< MatrixTransform * >( hd->stylus->getValue() );
        Bound *b = NULL;
        if( mt ) b = mt->transformedBound->getValue();
        else if( stylus_bound ) b = stylus_bound->bound->getValue();

        if( b ) {
          BoxBound *bb = dynamic_cast< BoxBound * >( b );
          if( bb ) {
            const Vec3f &pos = hd->weightedProxyPosition->getValue();
            //const Rotation &rot = hd->trackerOrientation->getValue();
            TransformedBoxBound *tbb = new TransformedBoxBound();
            tbb->boxBound->setValue( bb );
            Matrix4f m( hd->trackerOrientation->getValue() );
            m[0][3] = pos.x;
            m[1][3] = pos.y;
            m[2][3] = pos.z;
            tbb->matrix->setValue( m );
            bounds.push_back( tbb );
            bounds_to_remove.push_back( tbb );
          } else if( dynamic_cast< EmptyBound * >( b ) ) {
            //empty bound. ignore
          } else {
            success = false;
          }
        }
      }
    }
  }
    
  // add the child node bound.
  H3DBoundedObject *bound_object = 
    dynamic_cast< H3DBoundedObject * >( child_to_render );
  MatrixTransform *transform = 
    dynamic_cast< MatrixTransform * >( child_to_render );
  Bound *b = NULL;
  if( transform ) b = transform->transformedBound->getValue();
  else if( bound_object ) b = bound_object->bound->getValue();
  
  if( b ){
    BoxBound *bb = dynamic_cast< BoxBound * >( b );
    if( bb ) {
      bounds.push_back( bb );
    } else if( dynamic_cast< EmptyBound * >( b ) ) {
      //empty bound. ignore
    } else {
      success = false;
    }
  }
  
  // if no bounded objects return
  if( bounds.size() == 0 || !success )
    return false;
  
  // calculate the total bound.
  auto_ptr< Bound > total_bound( Bound::boundUnion( bounds.begin(), 
                                                    bounds.end() ) );
    
  BoxBound *bound =  dynamic_cast< BoxBound * >( total_bound.get() );
  if( bound ) {
    const Vec3f &vp_position = vp->totalPosition->getValue();
    const Rotation &vp_orientation = vp->totalOrientation->getValue();
    const Matrix4f &vp_frw_m = vp->accForwardMatrix->getValue();

    const Vec3f &bb_center = bound->center->getValue();
    const Vec3f &half_bb_size   = bound->size->getValue() / 2.0;
    // get the lookat vector in global coordinates.
    Vec3f vp_lookat = 
      (Matrix3f)vp_frw_m.getScaleRotationPart() * 
      (Matrix3f)vp_orientation * 
      Vec3f( 0, 0, -1 );
    vp_lookat.normalize();
    // get the position in global coordinates.
    Vec3f vp_pos = vp_frw_m * vp_position;
    
    // find the maximum and minimum distances from the viewpoint position
    // in the direction of the lookat vector to the corners of the 
    // bounding box.
    Vec3f trf_bb = bb_center + half_bb_size;
    Vec3f llc_bb = bb_center - half_bb_size;
    H3DFloat min_d, max_d;
    
    // +++
    Vec3f point = trf_bb;
    Vec3f v = point - vp_pos;
    min_d = max_d = vp_lookat * v;
    
    // ++-
    point.z = llc_bb.z;
    v = point - vp_pos;
    H3DFloat d = vp_lookat * v;
    if( d < min_d ) min_d = d;
    if( d > max_d ) max_d = d;
    
    // +--
    point.y = llc_bb.y;
    v = point - vp_pos;
    d = vp_lookat * v;
    if( d < min_d ) min_d = d;
    if( d > max_d ) max_d = d;
    
    // +-+
    point.z = trf_bb.z;
    v = point - vp_pos;
    d = vp_lookat * v;
    if( d < min_d ) min_d = d;
    if( d > max_d ) max_d = d;
    
    // --+
    point.x = llc_bb.x;
    v = point - vp_pos;
    d = vp_lookat * v;
    if( d < min_d ) min_d = d;
    if( d > max_d ) max_d = d;
    
    // -++
    point.y = trf_bb.y;
    v = point - vp_pos;
    d = vp_lookat * v;
    if( d < min_d ) min_d = d;
    if( d > max_d ) max_d = d;
    
    // -+-
    point.z = llc_bb.z;
    v = point - vp_pos;
    d = vp_lookat * v;
    if( d < min_d ) min_d = d;
    if( d > max_d ) max_d = d;
    
    // ---
    point.y = llc_bb.y;
    v = point - vp_pos;
    d = vp_lookat * v;
    if( d < min_d ) min_d = d;
    if( d > max_d ) max_d = d;

    // add an epsilon value to make sure nothing is clipped away
    // unintentially.
    clip_near = min_d - 1e-4f;
    clip_far = max_d + 1e-4f;

    // Needed in order to get the correct clip factors when scaling the scene.
    Vec3f scale_factors = vp->accInverseMatrix->getValue().getScalePart();
    if( scale_factors.z > 1 ) {
      H3DFloat clip_center = clip_far + clip_near / 2;
      H3DFloat clip_edge_mov = (clip_center - clip_near) * scale_factors.z;
      clip_near = clip_center - clip_edge_mov;
      clip_far =  clip_center + clip_edge_mov;
    } else {
      clip_near = clip_near * scale_factors.z;
      clip_far = clip_far * scale_factors.z;
    }

    // make sure the clip planes are in front of the camera. Depth
    // buffer precision is affected by the values of clip_far and clip_near.
    // The greater the ratio clip_far/clip_near the less the precision will
    // be. So we do not want to set clip_near too close to 0. Therefore
    // we make sure that the ratio is at most 1000.
    if( clip_far <= 0 ) clip_far = 2*Constants::f_epsilon;
    if( clip_near <= clip_far * 0.001f ) clip_near = clip_far * 0.001f;
    
    return success;
  }
  return false;
}

void H3DWindowNode::checkIfStereoObtained() {

  if( window_is_made_active && renderMode->getRenderMode() == RenderMode::QUAD_BUFFERED_STEREO ) {
    switch (renderMode->getRenderMode())
    {
    case RenderMode::QUAD_BUFFERED_STEREO :
      {
        GLboolean quad_stereo_supported;
        glGetBooleanv( GL_STEREO, &quad_stereo_supported);
        if( !quad_stereo_supported ) {
          Console(LogLevel::Error) << "Warning: Stereo pixel format not supported by your "
            << "graphics card(or it is not enabled). Quad buffered "
            << "stereo cannot be used. "
            << "Using \"MONO\" instead. " <<endl;
          renderMode->setValue( "MONO", id );
        }
        break;
      }

    case RenderMode::VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS:
      {
#ifdef GLEW_ARB_viewport_array
        if( !GLEW_ARB_viewport_array ) {
#endif
          Console(LogLevel::Error) << "Warning: GL_ARB_viewport_array not supported by your "
            << "graphics card. VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS "
            << "stereo cannot be used. "
            << "Using \"MONO\" instead. " <<endl;
          renderMode->setValue( "MONO", id );
#ifdef GLEW_ARB_viewport_array
        }
#endif
        break;
      }
    default:
      break;
    }
    check_if_stereo_obtained = false;
  }
}

void H3DWindowNode::initWindowWithContext() {
  window_is_made_active = false;
  RenderMode::Mode stereo_mode = renderMode->getRenderMode();
  if( stereo_mode != RenderMode::MONO )
    check_if_stereo_obtained = true;
  initWindow();
  if( check_if_stereo_obtained )
    checkIfStereoObtained();
}

void H3DWindowNode::configureViewPortsSize( RenderMode::Mode stereo_mode, 
  H3DInt32& viewport_width, H3DInt32& viewport_height, H3DFloat* _viewports_size )
{
    H3DInt32 window_width = width->getValue();
    H3DInt32 window_height = height->getValue();
    // by default set viewport width height to be the same as window width, height
    viewport_width = window_width;
    viewport_height = window_height;

    projectionHeight->setValue(viewport_height,id);
    projectionWidth->setValue(viewport_width,id);

    fbo_current_width = viewport_width;
    fbo_current_height= viewport_height;
    int n = 1; // total number of view port
    //fill in two viewport origin value if they exist
    H3DInt32 vx[2] = {0, 0};   // viewport origin point x component
    H3DInt32 vy[2] = {0, 0};   // viewport origin point y component
    switch (stereo_mode)
    {
    case RenderMode::MONO:
    case RenderMode::QUAD_BUFFERED_STEREO:
    case RenderMode::HORIZONTAL_INTERLACED:
    case RenderMode::VERTICAL_INTERLACED:
    case RenderMode::CHECKER_INTERLACED:
    case RenderMode::VERTICAL_INTERLACED_GREEN_SHIFT:
    case RenderMode::RED_BLUE_STEREO:
    case RenderMode::RED_GREEN_STEREO:
    case RenderMode::RED_CYAN_STEREO:
    case RenderMode::OCULUS_RIFT:
      {// only need one viewport
        n = 1;
        break;
      }
    case RenderMode::HORIZONTAL_SPLIT:
      {
        n = 2;
        viewport_height = window_height/2;
        fbo_current_height = viewport_height;
        vy[0] = viewport_height;
        break;
      }
    case RenderMode::VERTICAL_SPLIT:
      {
        n = 2;
        viewport_width = window_width/2;
        fbo_current_width = viewport_width;
        vx[1] = viewport_width;
        break;
      }
    case RenderMode::HORIZONTAL_SPLIT_KEEP_RATIO:
      {
        n = 2;
        viewport_height = window_height/2;
        fbo_current_height = viewport_height;
        projectionHeight->setValue(viewport_height,id);
        vy[0] = viewport_height;
        break;
      }
    case RenderMode::VERTICAL_SPLIT_KEEP_RATIO:
      {
        n = 2;
        viewport_width = window_width/2;
        fbo_current_width = viewport_width;
        projectionWidth->setValue(viewport_width,id);
        vx[1] = viewport_width;
        break;
      }
    case RenderMode::HDMI_FRAME_PACKED_720P:
      {
        n = 2;
        viewport_width = 1280;
        fbo_current_width = viewport_width;
        projectionWidth->setValue(1280,id);
        viewport_height = 720;
        fbo_current_height = viewport_height;
        projectionHeight->setValue(720,id);
        vy[0] = 750;
        break;
      }
    case RenderMode::HDMI_FRAME_PACKED_1080P:
      {
        n = 2;
        viewport_width = 1920;
        fbo_current_width = viewport_width;
        projectionWidth->setValue(1920,id);
        viewport_height = 1080;
        fbo_current_height = viewport_height;
        projectionHeight->setValue(1080,id);
        vy[0] = 1125;
        break;
      }
    case RenderMode::NVIDIA_3DVISION:
      {
        n = 2;
        vx[1] = viewport_width;
        break;
      }
    case RenderMode::VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS:
      {
        n = 2;
        viewport_width = window_width/2;
        projectionWidth->setValue(viewport_width,id);
        vx[1] = viewport_width;
        fbo_current_width = window_width;
        fbo_current_height = window_height;
        break;
      }
    }
    if(_viewports_size) {
      // set the windows dimension as the first four value
      _viewports_size[0] = 0;
      _viewports_size[1] = 0;
      _viewports_size[2] = (H3DFloat)window_width;
      _viewports_size[3] = (H3DFloat)window_height;
      for( int i = 0; i < n; ++i ) {
        _viewports_size[4*(i+1)+0] = (H3DFloat)vx[i];
        _viewports_size[4*(i+1)+1] = (H3DFloat)vy[i];
        _viewports_size[4*(i+1)+2] = (H3DFloat)viewport_width;
        _viewports_size[4*(i+1)+3] = (H3DFloat)viewport_height;
      }
    }
    nr_viewports = n;
}

struct CallbackData {
  H3DWindowNode *window;
  X3DChildNode *renderChild;
};

void H3DWindowNode::fboCallback( FrameBufferTextureGenerator *g, int i, void *args ) {
  CallbackData *data = static_cast< CallbackData * >( args );

  // clear buffer every second call to renderChild (one call per eye in stereo).
  // we only want to clear it 
  static int count = 0;
  if( count % 2 == 0 ) 
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  ++count;

  // render the node to the fbo.
  data->window->renderChild( data->renderChild, true );
}

void H3DWindowNode::renderChild( X3DChildNode *child_to_render ) {
  RenderMode::Mode stereo_mode = renderMode->getRenderMode();
  if( stereo_mode == RenderMode::NVIDIA_3DVISION ) {
    // NVIDIA_3DVISION render mode will render the data to an fbo.
    // It is then up to the sub-class to read this data and display
    // it properly for stereo.
    CallbackData data;
    data.window = this;
    data.renderChild = child_to_render;
    generator->setRenderCallback( fboCallback, &data );
    generator->render();
  } else {
    // normal render mode, just render.
    renderChild( child_to_render, false );
  }
}

void H3DWindowNode::render( X3DChildNode *child_to_render ) {
  
  if ( !isInitialized() )
    initialize();
  
  RenderMode::Mode stereo_mode = renderMode->getRenderMode();
  windowSizeChanged->upToDate();
  if( stereo_mode != last_render_mode ) {
    if( stereo_mode == RenderMode::QUAD_BUFFERED_STEREO ||
        last_render_mode == RenderMode::QUAD_BUFFERED_STEREO ||
        stereo_mode == RenderMode::NVIDIA_3DVISION ||
        last_render_mode == RenderMode::NVIDIA_3DVISION ) {
      // reinitialize the window to support/remove quad buffered stereo
      initWindowWithContext();
    }
    last_render_mode = renderMode->getRenderMode();
    renderMode->updateRebuildStencilMaskFlag();
  }
  
  if (stereo_mode == RenderMode::OCULUS_RIFT) {
#ifdef HAVE_LIBOVR    
    if (!oculus) {
      oculus = new OculusRiftHandler;
      oculus->setMSAA(true);
    }

    if (!oculus->isInitialized()) {
      bool res = oculus->initVR();
      if (!res) {
        Console(LogLevel::Error)
          << "Failed to initialize OculusRift, "
          << "\"OCULUS_RIFT\" stereo mode cannot be used. Using \"MONO\" instead." << endl;
        renderMode->setValue("MONO", id);
      } else {
        res = oculus->initVRBuffers(width->getValue(), height->getValue());
        if (!res) {
          oculus->destroyVR();
          Console(LogLevel::Error)
            << "Failed to initialize OculusRift, "
            << "\"OCULUS_RIFT\" stereo mode cannot be used. Using \"MONO\" instead." << endl;
          renderMode->setValue("MONO", id);
        }
      }
    }
#else
    Console(LogLevel::Error) << "Cannot use OCULUR_RIFT rendermode. H3DAPI compiled without"
      << " LibOVR support. Recompile H3DAPI with "
      << "HAVE_LIBOVR defined in order to use it. Using \"MONO\" instead." << endl;
 
    renderMode->setValue("MONO", id);
#endif
  }

  // make this the active window
  makeWindowActive();
  if( check_if_stereo_obtained )
    checkIfStereoObtained();

  // set fullscreen mode
  setFullscreen( fullscreen->getValue() );

  if (stereo_mode == RenderMode::OCULUS_RIFT) {
#ifdef HAVE_LIBOVR
    oculus->onRenderStart();
#endif
  }

  if( !child_to_render ) {
    // clear the buffers
    glClearColor( 0, 0, 0, 0 );
    if( stereo_mode != RenderMode::QUAD_BUFFERED_STEREO ) {
      glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
      glDrawBuffer( GL_BACK );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    } else {
      glDrawBuffer( GL_BACK_LEFT );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
      glDrawBuffer( GL_BACK_RIGHT );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    } 

    swapBuffers();
    return;
  }

  glPushAttrib( GL_ENABLE_BIT );

  X3DBackgroundNode *background = X3DBackgroundNode::getActive();
  Fog *fog = Fog::getActive();
  RGBA clear_color = RGBA( 0, 0, 0, 0 );
  if( background ) clear_color = background->glClearColor();
  if( fog ) fog->renderFog(); 
  glClearColor( clear_color.r, clear_color.g, clear_color.b, clear_color.a );

  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  
  glEnable( GL_DEPTH_TEST ); 
  glEnable( GL_LIGHTING );
  glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );
  glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );
  GLfloat no_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
  glLightModelfv( GL_LIGHT_MODEL_AMBIENT, no_ambient);
  
  X3DShapeNode::disable_lighting_if_no_app = true;

  GraphicsOptions *graphics_options = NULL;
  GlobalSettings *default_settings = GlobalSettings::getActive();
  if( default_settings ) {
    default_settings->getOptionNode( graphics_options );
  }

  // enable headlight
  NavigationInfo *nav_info = navigationInfo->getValue();
  if( !nav_info ) nav_info = NavigationInfo::getActive();
  GLint headlight_index = -1;
  if( !nav_info || nav_info->headlight->getValue() ) {
    glPushAttrib( GL_LIGHTING_BIT );
    headlight_index =
      X3DLightNode::getLightIndex( "Headlight in H3DWindowNode" );
    glEnable( GL_LIGHT0 + (GLuint)(headlight_index) );
  }

  // add headlight shadows if specified in NavigationInfo
  Scene *scene = Scene::scenes.size() > 0 ? *Scene::scenes.begin(): NULL;
  TraverseInfo *ti = scene == NULL ? NULL : scene->getLastTraverseInfo();
  ShadowCaster *shadow_caster = NULL;
  if( ti ) {
    ti->getUserData( "ShadowCaster",  (void **)&shadow_caster);

    // set options for default shadows
    if( graphics_options ) {
      if( !graphics_options->useDefaultShadows->getValue() ) {
        shadow_caster = NULL;
      }

      if( shadow_caster ) {
        shadow_caster->shadowDarkness->setValue( graphics_options->defaultShadowDarkness->getValue()  );
        shadow_caster->shadowDepthOffset->setValue( graphics_options->defaultShadowDepthOffset->getValue()  );
      }
    }
  }

  // get the viewpoint. If the H3DWindowNode viewpoint field is set use that
  // otherwise use the stack top of the Viewpoint bindable stack.
  X3DViewpointNode *vp =
    static_cast< X3DViewpointNode * >( viewpoint->getValue() );
  if( !vp ) {
    // we have at least 2 Viewpoints and are using a default local viewpoint
    // this means that we should not have a default local viewpoint any more
    // so remove it and activate the first in the list of new viewpoints
    X3DViewpointNode::ViewpointList vps = X3DViewpointNode::getAllViewpoints();
    if( vps.size() > 1 && vp_ref.get() ) {
      // find a viewpoint that is not the default local viewpoint and bind it.
      for( X3DViewpointNode::ViewpointList::iterator i = vps.begin();
           i != vps.end(); ++i ) {
        if( (*i) != vp_ref.get() ) {
          (*i)->set_bind->setValue( true );
        }
      }
      vp_ref.reset( NULL );
    }
    vp = static_cast< X3DViewpointNode * >(X3DViewpointNode::getActive());
  }
  if ( ! vp ) {
    vp = new Viewpoint;
    vp->setName( "Default X3D Viewpoint" );
    vp->description->setValue( "Default X3D Viewpoint" );
    vp_ref.reset( static_cast<Viewpoint *>(vp) );
  }

  X3DViewpointNode *navigation_vp = vp;
  if( resetViewPoint->getValue() ) {
    vp->relOrn->setValue( Rotation() );
    vp->relPos->setValue( Vec3f() );
    resetViewPoint->setValue(false);
  }
  
  // only use old viewpoint on first frame when switching viewpoints if not using TELEPORT transition type
  // otherwise the old viewpoint will be rendered for a single frame
  if (!nav_info || h3d_navigation->getTransitionType(nav_info->transitionType->getValue()) != "TELEPORT") {
    vp = h3d_navigation->viewpointToUse(vp);
  }

  const Rotation &vp_orientation = vp->totalOrientation->getValue();
  const Matrix4f &vp_inv_m = vp->accInverseMatrix->getValue();
  const Rotation &vp_inv_rot = (Rotation)vp_inv_m.getRotationPart();

  H3DMultiPassRenderObject::setMultiPassTransparencyAll(
    multi_pass_transparency );
  H3DMultiPassRenderObject::renderPreViewpointAll( child_to_render, vp );

  glViewport( 0, 0, width->getValue(), height->getValue() );
  
  if( rebuild_stencil_mask ) {
    // build up stencil buffer
    int w = width->getValue();
    // width needs to be a power of 2 because of how glDrawPixels work.
    // The extra values will be ignored.
    w += 4 - (w  % 4  );
    int h = height->getValue();
    if( stencil_mask ) free( stencil_mask );
    stencil_mask = (unsigned char *) malloc( w*h );
    stencil_mask_width = w;
    stencil_mask_height = h;

    switch(stereo_mode){
      case RenderMode::HORIZONTAL_INTERLACED: {
        for( int i = 0; i < h; ++i )
          for( int j = 0; j < w; ++j )
            stencil_mask[i*w+j]=(i+1)%2;
        break;
      }
      case RenderMode::CHECKER_INTERLACED: {
        for( int i = 0; i < h; ++i )
          for( int j = 0; j < w; ++j )
            stencil_mask[i*w+j]=(i+j+1)%2;
        break;
      }
      default: {
        for( int i = 0; i < h; ++i )
          for( int j = 0; j < w; ++j )
            stencil_mask[i*w+j]=(j+1)%2;
      }
    }
    rebuild_stencil_mask = false;
  }

  H3DFloat clip_near = 0.01f;  // near viewing plane at 1cm
  H3DFloat clip_far  = 10.f; // far viewing plane at 10m
  
  if( nav_info ) {
    if( nav_info->visibilityLimit->getValue() > 0 )
      clip_far = nav_info->visibilityLimit->getValue();
    else {
      // If visibilityLimit is 0.0 or special value -1, then it means infinity.
      // We let any negative value be infinity.
      clip_far = -1; 
    }
    if( nav_info->nearVisibilityLimit->getValue() > 0 )
      clip_near = nav_info->nearVisibilityLimit->getValue();
  }
  else
  {
    // if no nav_info is not exist, calculate the near and far clipping planes
    // based on the union of bounding box of the child_to_render node and
    // the styli of the haptics devices.
    calculateFarAndNearPlane( clip_far, clip_near, child_to_render, vp, true );
  }

  // if using zfail shadow volume technique, far clip plane has to be at
  // infinity.
  if( shadow_caster && 
      !shadow_caster->object->empty() &&
      !shadow_caster->light->empty() && 
      shadow_caster->algorithm->getValue() == "ZFAIL" ) {
    clip_far = -1;
  }


  H3DFloat focal_distance = 0.6f;

  StereoInfo *stereo_info = StereoInfo::getActive();
  bool stereo_swap_eyes = false;
  if( stereo_info ) {
    focal_distance = stereo_info->focalDistance->getValue();
    stereo_swap_eyes = stereo_info->swapEyes->getValue();
  }

  bool mirror_in_y = mirrored->getValue();
    
  if( mirror_in_y != last_loop_mirrored ) 
    H3DDisplayListObject::DisplayList::rebuildAllDisplayLists();

  last_loop_mirrored = mirror_in_y;

  // Variables needed for calculating the from and to arguments to
  // X3DPointingDeviceSensorNode::updateX3DPointingDeviceSensor.
  // Projection matrix for mono. Depends on Viewpoint.
  GLdouble mono_projmatrix[16], mono_mvmatrix[16];
  bool any_pointing_device_sensors =
    X3DPointingDeviceSensorNode::instancesExists();
  //    Console(LogLevel::Error) << clip_near << " " << clip_far << endl;
  bool isStereoMode = renderMode->isStereoMode();
  bool isSinglePass = renderMode->isSinglePass();
  if( isStereoMode ) {
    // in stereo mode, need to make sure the focal plan is between the near and far
    // clipping planes.
    if( focal_distance <= clip_near ) {
      clip_near = focal_distance - 0.01f;
    }
    if( focal_distance >= clip_far && clip_far != -1 ) {
      clip_far = focal_distance + 0.01f;
    }
  }
  
  // initialize the viewport values
  H3DInt32 viewport_width = 0, viewport_height =0;
  configureViewPortsSize( stereo_mode, viewport_width, viewport_height, viewports_size);
  
  if( viewportWidth->getValue()!=viewport_width ) {
    viewportWidth->setValue(viewport_width, id);
  }
  if( viewportHeight->getValue()!=viewport_height ) {
    viewportHeight->setValue(viewport_height, id);
  }
  
  clipDistances->setValue( Vec2f(clip_near, clip_far), id );
  


  if( !isSinglePass ) { // need to render twice for stereo
    if( singlePassStereo->getValue()!=false ) {
      singlePassStereo->setValue(false,id);
    }
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if( mirror_in_y ) {
      glScalef( 1, -1, 1 );
      glFrontFace( GL_CW );
    } else {
      glFrontFace( GL_CCW );
    }

 #ifdef HAVE_LIBOVR
    // view matrix to use for Oculus
    Matrix4f ovr_mv;
    // projection matrix to use for Oculus
    Matrix4f ovr_p;
#endif

    // LEFT EYE
    // The stereo rendering is made using the parallel axis asymmetric frustum 
    // perspective projection method, which means that the viewpoint for each 
    // eye remain parallel and an the asymmetric view frustum is set up using 
    // glFrustum. This is done by calling the setupProjection function of
    // X3DViewpointNode.
    if (stereo_mode == RenderMode::OCULUS_RIFT) {
#ifdef HAVE_LIBOVR
      oculus->onEyeRender(0, ovr_mv, ovr_p );
      glMultMatrixf(&ovr_p.transpose()[0][0]);
#endif
    } else {
      eye_mode = stereo_swap_eyes ? X3DViewpointNode::RIGHT_EYE : X3DViewpointNode::LEFT_EYE;
      vp->setupProjection(eye_mode,
                          (H3DFloat)projectionWidth->getValue(),
                          (H3DFloat)projectionHeight->getValue(),
                          clip_near, clip_far,
                          stereo_info); 

      glDrawBuffer(GL_BACK_LEFT);
    }
   
    if( stereo_mode == RenderMode::RED_BLUE_STEREO ||
        stereo_mode == RenderMode::RED_GREEN_STEREO ||
        stereo_mode == RenderMode::RED_CYAN_STEREO )
      // render only the red component
      glColorMask(GL_TRUE, GL_FALSE, GL_FALSE, GL_TRUE);
    else if( stereo_mode == RenderMode::VERTICAL_INTERLACED ||
             stereo_mode == RenderMode::HORIZONTAL_INTERLACED ||
             stereo_mode == RenderMode::CHECKER_INTERLACED ||
             stereo_mode == RenderMode::VERTICAL_INTERLACED_GREEN_SHIFT ) {

      // Inserted this line to reset the position of the raster
      // so that the entire stencil_mask will be copied into the stencil
      // buffer by glDrawPixels.  This ensures that even if user calls such
      // as glDrawPixels(), glCopyPixels(), or glBindFramebufferEXT()
      // leave glRasterPos in the middle of the screen, the entire stencil
      // stencil_mack still gets copied to the stencil buffery by
      // glDrawPixels()
      glWindowPos2f(0.000000f, 0.000000f);

      // Currently we redraw the stencil mask each loop, because of
      // the stencil mask can be cleared when running on a Sharp laptop
      // from somewhere outside H3D API. When this does not happen anymore
      // we can initialize the stencil buffer once.
      if( stencil_mask )
        glDrawPixels( stencil_mask_width, stencil_mask_height, 
                      GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, stencil_mask );
      // render only every second line
      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_EQUAL,1,1);
      glStencilOp(GL_KEEP,GL_KEEP,GL_KEEP);
      fbo_current_x = 0;
      fbo_current_y = 0;
    }else if( stereo_mode==RenderMode::HORIZONTAL_SPLIT ||
              stereo_mode==RenderMode::VERTICAL_SPLIT   ||
              stereo_mode==RenderMode::HORIZONTAL_SPLIT_KEEP_RATIO ||
              stereo_mode==RenderMode::VERTICAL_SPLIT_KEEP_RATIO||
              stereo_mode==RenderMode::HDMI_FRAME_PACKED_720P||
              stereo_mode==RenderMode::HDMI_FRAME_PACKED_1080P||
              stereo_mode==RenderMode::NVIDIA_3DVISION||
              stereo_mode==RenderMode::VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS ) { 
      // set viewport for left eye, when the viewport not covering the whole window
      if( viewports_size[4] != 0 || viewports_size[5] != 0 ||
        viewports_size[6] != width->getValue() || viewports_size[7] != height->getValue() ) {
          glViewport( (int)viewports_size[4], (int)viewports_size[5], 
            (int)viewports_size[6], (int)viewports_size[7] );
          fbo_current_x = (H3DInt32)viewports_size[4];
          fbo_current_y = (H3DInt32)viewports_size[5];
      }
    }

    // clear the buffers before rendering
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if( background ) {
      glPushMatrix();
      glRotatef( (H3DFloat) -(180/Constants::pi)*vp_orientation.angle, 
                 vp_orientation.axis.x, 
                 vp_orientation.axis.y,
                 vp_orientation.axis.z );
      glRotatef( (H3DFloat) (180/Constants::pi)*vp_inv_rot.angle, 
                 vp_inv_rot.axis.x, vp_inv_rot.axis.y, vp_inv_rot.axis.z );
      glDepthMask( GL_FALSE );
      background->renderBackground();
      glDepthMask( GL_TRUE );
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }

    GLboolean norm = glIsEnabled( GL_NORMALIZE );
    if( !norm ) {
      glEnable( GL_NORMALIZE );
    }

    if( stereo_mode == RenderMode::OCULUS_RIFT) {
#ifdef HAVE_LIBOVR
      glMultMatrixf(&ovr_mv.transpose()[0][0]);
#endif
    }

    // add viewmatrix to model view matrix.
    vp->setupViewMatrix( eye_mode,
                         stereo_info );

    if( ti ) {
      for( TraverseInfo::LightVector::const_iterator i = 
                              ti->getActiveLightNodes().begin();
           i != ti->getActiveLightNodes().end();
           ++i ) {
        (*i).getLight()->enableGraphicsState();
      }
    }

    H3DMultiPassRenderObject::renderPostViewpointAll( child_to_render, 
                                                      vp );

    // render the scene
    renderChild( child_to_render );

    H3DMultiPassRenderObject::renderPostSceneAll( child_to_render, 
                                                  vp ); 

#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepBegin("Stereo_shadows");
#endif
    if( shadow_caster ) shadow_caster->render();
#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepEnd("Stereo_shadows");
#endif
    if( ti ) {
      for( TraverseInfo::LightVector::const_iterator i = 
                              ti->getActiveLightNodes().begin();
           i != ti->getActiveLightNodes().end();
           ++i ) {
        (*i).getLight()->disableGraphicsState();
      }
    }

    if( !norm ) {
      glDisable( GL_NORMALIZE );
    }

    // RIGHT EYE
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if( mirror_in_y ) {
      glScalef( 1, -1, 1 );
      glFrontFace( GL_CW );
    } else {
      glFrontFace( GL_CCW );
    }

    if (stereo_mode == RenderMode::OCULUS_RIFT) {
#ifdef HAVE_LIBOVR
      oculus->onEyeRenderFinish(0);
      oculus->onEyeRender(1, ovr_mv, ovr_p );
      glMultMatrixf(&ovr_p.transpose()[0][0]);
#endif
    } else {
      eye_mode = stereo_swap_eyes ? X3DViewpointNode::LEFT_EYE : X3DViewpointNode::RIGHT_EYE;
      vp->setupProjection(eye_mode,
                          (H3DFloat)projectionWidth->getValue(),
                          (H3DFloat)projectionHeight->getValue(),
                          clip_near, clip_far,
                          stereo_info);
    }

    if( stereo_mode == RenderMode::QUAD_BUFFERED_STEREO ) {
      glDrawBuffer(GL_BACK_RIGHT);
      // clear the buffers before rendering
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    } else if( stereo_mode == RenderMode::RED_BLUE_STEREO ) {
      glColorMask(GL_FALSE, GL_TRUE, GL_TRUE, GL_TRUE);
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      glColorMask(GL_FALSE, GL_FALSE, GL_TRUE, GL_TRUE);
    } else if( stereo_mode == RenderMode::RED_GREEN_STEREO ) {
      glColorMask(GL_FALSE, GL_TRUE, GL_TRUE, GL_TRUE);
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      glColorMask(GL_FALSE, GL_TRUE, GL_FALSE, GL_TRUE);
    } else if( stereo_mode == RenderMode::RED_CYAN_STEREO ) {
      glColorMask(GL_FALSE, GL_TRUE, GL_TRUE, GL_TRUE);
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    } else if( stereo_mode == RenderMode::VERTICAL_INTERLACED ||
               stereo_mode == RenderMode::HORIZONTAL_INTERLACED ||
               stereo_mode == RenderMode::CHECKER_INTERLACED ||
               stereo_mode == RenderMode::VERTICAL_INTERLACED_GREEN_SHIFT ) {
      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_NOTEQUAL,1,1);
      glStencilOp(GL_KEEP,GL_KEEP,GL_KEEP);
    } else if ( stereo_mode==RenderMode::HORIZONTAL_SPLIT||
                stereo_mode==RenderMode::VERTICAL_SPLIT||
                stereo_mode==RenderMode::HORIZONTAL_SPLIT_KEEP_RATIO||
                stereo_mode==RenderMode::VERTICAL_SPLIT_KEEP_RATIO||
                stereo_mode==RenderMode::HDMI_FRAME_PACKED_720P||
                stereo_mode==RenderMode::HDMI_FRAME_PACKED_1080P||
                stereo_mode==RenderMode::NVIDIA_3DVISION||
                stereo_mode==RenderMode::VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS ) {
      // set viewport for right eye when it is not covering the full window
      if( viewports_size[8] != 0 || viewports_size[9] != 0 ||
        viewports_size[10] != width->getValue() || viewports_size[11] != height->getValue() ) {
          glViewport( (int)viewports_size[8], (int)viewports_size[9], 
            (int)viewports_size[10], (int)viewports_size[11] );
          fbo_current_x = (H3DInt32)viewports_size[8];
          fbo_current_y = (H3DInt32)viewports_size[9];
      }
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    if( background ) {
      glPushMatrix();
      glRotatef( (H3DFloat)-(180/Constants::pi)*vp_orientation.angle, 
                 vp_orientation.axis.x, 
                 vp_orientation.axis.y,
                 vp_orientation.axis.z );
      glRotatef( (H3DFloat)(180/Constants::pi)*vp_inv_rot.angle, 
                 vp_inv_rot.axis.x, vp_inv_rot.axis.y, vp_inv_rot.axis.z );
      glDepthMask( GL_FALSE );
      background->renderBackground();
      glDepthMask( GL_TRUE );
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }

    norm = glIsEnabled( GL_NORMALIZE );
    if ( !norm ) {
      glEnable( GL_NORMALIZE );
    }

    if( stereo_mode == RenderMode::OCULUS_RIFT) {
#ifdef HAVE_LIBOVR
      glMultMatrixf(&ovr_mv.transpose()[0][0]);
#endif
    }

    // add viewmatrix to model view matrix.
    vp->setupViewMatrix( eye_mode,
                         stereo_info );
    
    if( ti ) {
      for( TraverseInfo::LightVector::const_iterator i = 
                              ti->getActiveLightNodes().begin();
           i != ti->getActiveLightNodes().end();
           ++i ) {
        (*i).getLight()->enableGraphicsState();
      }
    }

    H3DMultiPassRenderObject::renderPostViewpointAll( child_to_render, 
                                                      vp );

    // render the scene
    renderChild( child_to_render );

    H3DMultiPassRenderObject::renderPostSceneAll( child_to_render, 
                                                  vp );

#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepBegin("Stereo_shadows");
#endif
    if( shadow_caster ) shadow_caster->render();
#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepEnd("Stereo_shadows");
#endif

    if( ti ) {
      for( TraverseInfo::LightVector::const_iterator i = 
                              ti->getActiveLightNodes().begin();
           i != ti->getActiveLightNodes().end();
           ++i ) {
        (*i).getLight()->disableGraphicsState();
      }
    }

    if( !norm ) {
      glDisable( GL_NORMALIZE );
    }

    if( stereo_mode == RenderMode::VERTICAL_INTERLACED ||
             stereo_mode == RenderMode::HORIZONTAL_INTERLACED ||
             stereo_mode == RenderMode::CHECKER_INTERLACED ) 
      glDisable( GL_STENCIL_TEST );
    else if( stereo_mode == RenderMode::VERTICAL_INTERLACED_GREEN_SHIFT ) {
      glDisable( GL_STENCIL_TEST );
      // do green shift
      glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT | GL_POLYGON_BIT | 
                   GL_POLYGON_STIPPLE_BIT | GL_VIEWPORT_BIT | 
                   GL_TRANSFORM_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
      glDisable(GL_POLYGON_STIPPLE);
      glDisable(GL_LINE_STIPPLE);
      glDisable(GL_DITHER);
      glDisable(GL_DEPTH_TEST);
      glDisable(GL_BLEND);
      glDisable(GL_FOG);
      glDisable(GL_LIGHTING);
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_TEXTURE_1D);
      glDisable(GL_POLYGON_SMOOTH);
      glDisable(GL_NORMALIZE);
      glDisable(GL_STENCIL_TEST);
      glDisable(GL_SCISSOR_TEST);
      glDisable(GL_ALPHA_TEST);
      glPixelTransferi(GL_MAP_COLOR,0);
      glPixelTransferi(GL_MAP_STENCIL,0);
      glPixelZoom(1.000000,1.000000);
      glViewport(0,0,width->getValue(),height->getValue());
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      glOrtho(0.000000,width->getValue(),0.000000,
              height->getValue(),1.000000,100.000000);
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadIdentity();
      glTranslated(0.000000,0.000000,-10.000000);
      glColorMask(0,1,0,0);
      glRasterPos2f(1.000000,0.000000);
      glCopyPixels(0,0,width->getValue(),height->getValue(),GL_COLOR);
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
      glPopAttrib();
    } else if( stereo_mode == RenderMode::HDMI_FRAME_PACKED_720P ) {
      // set 30 lines to black as per hdmi standard 
      glScissor( 0, 719, 1280, 30 );
      glEnable( GL_SCISSOR_TEST );
      glPushAttrib( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      glClearColor( 0.0, 0.0, 0.0, 1.0 );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      glPopAttrib();
      glDisable( GL_SCISSOR_TEST );
    } else if( stereo_mode == RenderMode::HDMI_FRAME_PACKED_1080P ) {
      // set 45 lines to black as per hdmi standard 
      glScissor( 0, 1079, 1920, 45 );
      glEnable( GL_SCISSOR_TEST );
      glPushAttrib( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      glClearColor( 0.0, 0.0, 0.0, 1.0 );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      glPopAttrib();
      glDisable( GL_SCISSOR_TEST );
    }

    if (stereo_mode == RenderMode::OCULUS_RIFT) {
#ifdef HAVE_LIBOVR
      oculus->onEyeRenderFinish(1);
      oculus->submitFrame();
      oculus->blitMirror();
#endif
    }
#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepBegin("Stereo_swapBuffers");
#endif
    swapBuffers();
#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepEnd("Stereo_swapBuffers");
#endif
    if( any_pointing_device_sensors ) {
      // Get values for mono_projmatrix and mono_mvmatrix which should
      // be used when calculating argument for
      // X3DPointingDeviceSensorNode::updateX3DPointingDeviceSensors
      // Values for the matrices should be the same as when rendering mono.
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();

      if( mirror_in_y ) {
        glScalef( 1, -1, 1 );
      }

      // Use viewpoint to set up GL_PROJECTION matrix.
      vp->setupProjection( X3DViewpointNode::MONO,
                           (H3DFloat)width->getValue(),
                           (H3DFloat)height->getValue(),
                           clip_near, clip_far );
      glGetDoublev( GL_PROJECTION_MATRIX, mono_projmatrix );
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();

      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadIdentity();

      // add viewmatrix to model view matrix.
      vp->setupViewMatrix( X3DViewpointNode::MONO );

      glGetDoublev( GL_MODELVIEW_MATRIX, mono_mvmatrix );
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }
  } else {
    // isSinglePass means either MONO or single pass stereo

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if( mirror_in_y ) {
      glScalef( 1, -1, 1 );
      glFrontFace( GL_CW );
    } else {
      glFrontFace( GL_CCW );
    }

    eye_mode = (isStereoMode ? X3DViewpointNode::BOTH_EYE : X3DViewpointNode::MONO);
    fbo_current_x = 0;
    fbo_current_y = 0;
    if( nr_viewports == 1 ) { 
      if( singlePassStereo->getValue()!=false ) {
        singlePassStereo->setValue(false,id);
      }
      //MONO, for single viewport, only specify viewport when the desired dimension
      // not matching the full window dimension
      if( viewports_size[4] != 0 || viewports_size[5] != 0 ||
        viewports_size[6] != width->getValue() || viewports_size[7] != height->getValue() ) {
          glViewport( (int)viewports_size[4], (int)viewports_size[5], 
            (int)viewports_size[6], (int)viewports_size[7] );
      }
    } else {
#ifdef GLEW_ARB_viewport_array
      // single pass stereo
      if( GLEW_ARB_viewport_array ) {
        glViewportArrayv(0, nr_viewports + 1, viewports_size);
        if( singlePassStereo->getValue()!=true ) {
          singlePassStereo->setValue(true,id);
        }
        
        
        glDisable(GL_SCISSOR_TEST);
        
      } else {
#endif
        Console(LogLevel::Error) << "Warning: GL_ARB_viewport_array is not supported by the graphic card. "
          <<"single pass stereo can not be used."<<endl;
#ifdef GLEW_ARB_viewport_array
      }
#endif
    }

    vp->setupProjection( eye_mode,
                         (H3DFloat)projectionWidth->getValue(),
                         (H3DFloat)projectionHeight->getValue(),
                         clip_near, clip_far, stereo_info, true );

    glMatrixMode(GL_MODELVIEW);
    glDrawBuffer(GL_BACK);
    glLoadIdentity();
   
    // clear the buffers before rendering
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    if( background ) {
      glPushMatrix();
      glRotatef( (H3DFloat) -(180/Constants::pi)*vp_orientation.angle, 
                 vp_orientation.axis.x, 
                 vp_orientation.axis.y,
                 vp_orientation.axis.z );
      glRotatef( (H3DFloat) (180/Constants::pi)*vp_inv_rot.angle, 
                 vp_inv_rot.axis.x, vp_inv_rot.axis.y, vp_inv_rot.axis.z );
      glDepthMask( GL_FALSE );
      background->renderBackground();
      glDepthMask( GL_TRUE );
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }

    GLboolean norm = glIsEnabled( GL_NORMALIZE );
    if( !norm ) {
      glEnable( GL_NORMALIZE );
    }

    // add viewmatrix to model view matrix.
    vp->setupViewMatrix( eye_mode, stereo_info, true );

    if( any_pointing_device_sensors ) {
      // Get matrices used to calculate arguments for
      // X3DPointingDeviceSensorNode::updateX3DPointingDeviceSensors
      glGetDoublev( GL_PROJECTION_MATRIX, mono_projmatrix );
      glGetDoublev( GL_MODELVIEW_MATRIX, mono_mvmatrix );
    }

    if( ti ) {
      for( TraverseInfo::LightVector::const_iterator i = 
                              ti->getActiveLightNodes().begin();
           i != ti->getActiveLightNodes().end();
           ++i ) {
        (*i).getLight()->enableGraphicsState();
      }
    }

    H3DMultiPassRenderObject::renderPostViewpointAll( child_to_render, 
                                                      vp );
    renderChild( child_to_render );
    H3DMultiPassRenderObject::renderPostSceneAll( child_to_render, 
                                                  vp );
#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepBegin("Mono_shadows");
#endif
    if( shadow_caster ) shadow_caster->render();
#ifdef  HAVE_PROFILER
    H3DUtil::H3DTimer::stepEnd("Mono_shadows");
#endif

    if( ti ) {
      for( TraverseInfo::LightVector::const_iterator i = 
                              ti->getActiveLightNodes().begin();
           i != ti->getActiveLightNodes().end();
           ++i ) {
        (*i).getLight()->disableGraphicsState();
      }
    }

    if( !norm ) {
      glDisable( GL_NORMALIZE );
    }
#ifdef HAVE_PROFILER
    H3DUtil::H3DTimer::stepBegin("mono_swapBuffers");
#endif
    swapBuffers();
#ifdef HAVE_PROFILER
    H3DUtil::H3DTimer::stepEnd("mono_swapBuffers");
#endif
  }
  glPopAttrib();

  // if we are using automatic cursor control we now choose cursor 
  // depending on scene state.
  if( !manualCursorControl->getValue () ) {
    if ( X3DPointingDeviceSensorNode::anyIsActive() ) {
      cursorType->setValue( getCursorForMode( "ON_SENSOR_ACTIVE" ) );
    } else if ( X3DPointingDeviceSensorNode::anyIsOver() ) {
      cursorType->setValue( getCursorForMode( "ON_SENSOR_OVER" ) );
    } else if( (nav_info && nav_info->getUsedNavType() == "LOOKAT") ||
               (!nav_info && default_nav == "LOOKAT" ) ) {
      cursorType->setValue( getCursorForMode( "ON_NAV_LOOKAT" ) );
    } else {
      cursorType->setValue("DEFAULT");
    }
  }

  // update cursor if cursor changed.
  const string &cursor_type = cursorType->getValue();
  if( current_cursor != cursor_type ) {
    if( setCursorType( cursor_type ) == 0 ) {
      current_cursor = cursor_type;
    }
  }

  bool has_mouse_button_pressed = false;
  if( pointingDeviceRefreshMode->getValue() == "MOUSE_CLICK" ){
    for( unsigned int i = 0; i < left_mouse_button.size(); ++i ) {
      if ( left_mouse_button[i] ){
        has_mouse_button_pressed = true;
        break;
      }
    }
  }

  if( any_pointing_device_sensors &&
      ( mouse_position[0]- previous_mouse_position[0] != 0 ||
        mouse_position[1]- previous_mouse_position[1] != 0 ||
        has_mouse_button_pressed ) ) {
    // If mouse moved the transform mouse to world coordinate space and send
    // to updateX3DPointingDeviceSensors function.
    GLint mono_viewport[4] = { 0, 0, width->getValue(), height->getValue() };
    GLdouble wx, wy, wz;

    H3DInt32 tmp_mouse_pos[2];
    tmp_mouse_pos[0] = mouse_position[0];
    tmp_mouse_pos[1] = mono_viewport[3] - mouse_position[1] - 1;
    // Project to 0, 0.5 and 1.0 and use the values to check if
    // an infinite clip_far value in a Viewpoint node caused the
    // unproject to project to the wrong place (behind the near_plane)
    // if it did then choose a large vector in the other direction.
    gluUnProject( (GLdouble) tmp_mouse_pos[0], (GLdouble) tmp_mouse_pos[1],
      0.0, mono_mvmatrix, mono_projmatrix, mono_viewport, &wx, &wy, &wz );
    Vec3f near_plane_pos( (H3DFloat)wx, (H3DFloat)wy, (H3DFloat)wz );
    gluUnProject( (GLdouble) tmp_mouse_pos[0], (GLdouble) tmp_mouse_pos[1],
      0.5, mono_mvmatrix, mono_projmatrix, mono_viewport, &wx, &wy, &wz );
    Vec3f middle_plane_pos( (H3DFloat)wx, (H3DFloat)wy, (H3DFloat)wz );
    gluUnProject( (GLdouble) tmp_mouse_pos[0], (GLdouble) tmp_mouse_pos[1],
      1.0, mono_mvmatrix, mono_projmatrix, mono_viewport, &wx, &wy, &wz );
    Vec3f far_plane_pos( (H3DFloat)wx, (H3DFloat)wy, (H3DFloat)wz );
    Vec3f near_middle = middle_plane_pos - near_plane_pos;
    Vec3f near_far = far_plane_pos - near_plane_pos;
    if( near_middle * near_far < 0 ) {
      // Infinite far plane caused problems. Choose a large vector in the
      // other direction. Note that this is not an optimal solution, the best
      // solution in this case would be to use a rayIntersect function in
      // updateX3DPointingDeviceSensors. In this case we choose the point
      // halfway to infinity, hopefully that should be enough.
      near_middle.normalizeSafe();
      far_plane_pos = near_plane_pos + near_far.length() * near_middle;
    }

    // Update pointing device sensors in order to have them correctly
    // calculated for next turn.
    X3DPointingDeviceSensorNode::
      updateX3DPointingDeviceSensors( child_to_render,
      near_plane_pos,
      far_plane_pos );
  }

  for( unsigned int i = 0; i < left_mouse_button.size(); ++i ) {
    bool tmp_button = left_mouse_button[i];
    if( previous_left_mouse_button != tmp_button ) {
      X3DPointingDeviceSensorNode::updateButtonDependentFields( tmp_button );
      previous_left_mouse_button = tmp_button;
    }
  }
  left_mouse_button.clear();

  string nav_type = default_nav;
  bool use_collision = default_collision;
  vector< H3DFloat > &avatar_size = default_avatar;
  H3DFloat nav_speed = default_speed;
  vector< string > &transition_type = default_transition_type;
  H3DTime transition_time = default_transition_time;
  if( nav_info ) {
    nav_type = nav_info->getUsedNavType();
    use_collision = true;
    avatar_size = nav_info->avatarSize->getValue();
    nav_speed = nav_info->speed->getValue();
    transition_type = nav_info->transitionType->getValue();
    transition_time = nav_info->transitionTime->getValue();
  }

  CollisionOptions *collision_options = NULL;
  if( default_settings ) {
    default_settings->getOptionNode( collision_options );
  }
  if( collision_options )
    use_collision = collision_options->avatarCollision->getValue();

  h3d_navigation->doNavigation( nav_type,
                                navigation_vp,
                                child_to_render,
                                use_collision,
                                avatar_size,
                                nav_speed,
                                transition_type,
                                transition_time );
  // Store previous mouse position
  previous_mouse_position[0] = mouse_position[0];
  previous_mouse_position[1] = mouse_position[1];

  if( headlight_index != -1 ) {
    glPopAttrib();
    X3DLightNode::decreaseLightIndex();
  }
}

void H3DWindowNode::reshape( int w, int h ) {
  width->setValue( w );
  height->setValue( h );
}


void H3DWindowNode::display() {
  if( last_render_child ) 
    render( last_render_child );
}

H3DWindowNode::RenderMode::Mode H3DWindowNode::RenderMode::getRenderMode() {
  upToDate();
  if (value == "MONO")
    return MONO;
  else if (value == "QUAD_BUFFERED_STEREO")
    return QUAD_BUFFERED_STEREO;
  else if (value == "VERTICAL_SPLIT")
    return VERTICAL_SPLIT;
  else if (value == "VERTICAL_SPLIT_KEEP_RATIO")
    return VERTICAL_SPLIT_KEEP_RATIO;
  else if (value == "HORIZONTAL_SPLIT")
    return HORIZONTAL_SPLIT;
  else if (value == "HORIZONTAL_SPLIT_KEEP_RATIO")
    return HORIZONTAL_SPLIT_KEEP_RATIO;
  else if (value == "VERTICAL_INTERLACED")
    return VERTICAL_INTERLACED;
  else if (value == "HORIZONTAL_INTERLACED")
    return HORIZONTAL_INTERLACED;
  else if (value == "CHECKER_INTERLACED")
    return CHECKER_INTERLACED;
  else if (value == "RED_BLUE_STEREO")
    return RED_BLUE_STEREO;
  else if (value == "RED_GREEN_STEREO")
    return RED_GREEN_STEREO;
  else if (value == "RED_CYAN_STEREO")
    return RED_CYAN_STEREO;
  else if (value == "VERTICAL_INTERLACED_GREEN_SHIFT")
    return VERTICAL_INTERLACED_GREEN_SHIFT;
  else if (value == "HDMI_FRAME_PACKED_720P")
    return HDMI_FRAME_PACKED_720P;
  else if (value == "HDMI_FRAME_PACKED_1080P")
    return HDMI_FRAME_PACKED_1080P;
  else if (value == "NVIDIA_3DVISION")
    return NVIDIA_3DVISION;
  else if (value == "VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS")
    return VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS;
  else if (value == "OCULUS_RIFT")
    return OCULUS_RIFT;
  else {
    stringstream s;
    s << "Must be one of "
      << "MONO, "
      << "QUAD_BUFFERED_STEREO, "
      << "HORIZONTAL_INTERLACED, "
      << "VERTICAL_INTERLACED, "
      << "CHECKER_INTERLACED, "
      << "VERTICAL_INTERLACED_GREEN_SHIFT, "
      << "VERTICAL_SPLIT, "
      << "VERTICAL_SPLIT_KEEP_RATIO, "
      << "HORIZONTAL_SPLIT, "
      << "HORIZONTAL_SPLIT_KEEP_RATIO, "
      << "NVIDIA_3DVISION, "
      << "HDMI_FRAME_PACKED_720P, "
      << "HDMI_FRAME_PACKED_1080P, "
      << "VERTICAL_SPLIT_KEEP_ASPECT_ONE_PASS"
      << "OCULUS_RIFT"
      << "RED_CYAN_STEREO, RED_GREEN_STEREO or RED_BLUE_STEREO. ";
    throw InvalidRenderMode( value, 
                             s.str(),
                             H3D_FULL_LOCATION );
  }
}

void H3DWindowNode::onKeyDown( int key, bool special ) {
  if( special ) {
    X3DKeyDeviceSensorNode::keyboardSpecialDownCallback( key );
    h3d_navigation->handleKeyAction( key, true );
  } else
    X3DKeyDeviceSensorNode::keyboardDownCallback( key );
}

void H3DWindowNode::onKeyUp( int key, bool special ) {
  if( special ) {
    X3DKeyDeviceSensorNode::keyboardSpecialUpCallback( key );
    h3d_navigation->handleKeyAction( key, false );
  }
  else
    X3DKeyDeviceSensorNode::keyboardUpCallback( key );
}

void H3DWindowNode::onMouseButtonAction( int button, int state ) {
  MouseSensor::buttonCallback( button, state );
  if( button == MouseSensor::LEFT_BUTTON ) {
    left_mouse_button.push_back( state == MouseSensor::DOWN );
    h3d_navigation->leftButtonUpdate( state == MouseSensor::DOWN );
  }
}

void H3DWindowNode::onMouseMotionAction( int x, int y ) {
  MouseSensor::motionCallback( x, y );
  mouse_position[0] = x;
  mouse_position[1] = y;
  h3d_navigation->motionUpdate( x, y );
}

void H3DWindowNode::onMouseWheelAction( int direction ) {
  MouseSensor::wheelCallback( direction );
  h3d_navigation->scrollWheelUpdate( direction == MouseSensor::FROM );
}

#ifdef H3D_WINDOWS
// The following callback and message function is not used
// unless you make your own subclass to H3DWindowNode
// that uses Windows window handling. The intent is that
// this code should be used no matter if you use
// an external window such as GLUT.
LRESULT CALLBACK H3DWindowNode::WindowProc(HWND _hWnd, 
                                           UINT uMsg, 
                                           WPARAM wParam, 
                                           LPARAM lParam)
{
  // Find this window instance
  H3DWindowNode *thisWindow = 0;
  for( set< H3DWindowNode *>::iterator pos = windows.begin();
    pos != windows.end();
    ++pos ) {
    if( (*pos)->hWnd == _hWnd )
      thisWindow = *pos;
  }

  // If we have a window call Message otherwise call
  // the default window procedure.
  if( thisWindow ) {
    return thisWindow->Message(_hWnd, uMsg, wParam, lParam);
  }
  return DefWindowProc(_hWnd, uMsg, wParam, lParam);
}

// Message Handler
LRESULT H3DWindowNode::Message(HWND _hWnd,
                               UINT uMsg,
                               WPARAM wParam,
                               LPARAM lParam)
{  
  // Evaluate Window Message
  switch (uMsg) {    
    case WM_SYSCOMMAND: {        // Intercept System Commands
      // Check System Calls (according to msdn the & with 0xFFF0
      // is needed to obtain the correct result)
      switch (wParam & 0xFFF0)
      {
      case SC_KEYMENU: {
          // If the Window does not have a menu
          // then we do not want the default behaviour for SC_KEYMENU
          // cause that means that the keySensor does not
          // register buttons which do not generate characters.
          // Therefore we return if there is no menu.
          HMENU theMenu = GetMenu( _hWnd );
          if( !theMenu )
            return 0;
          break;
        }
        default:{}
      }
      break;
    }

    case WM_KEYDOWN:  
    case WM_SYSKEYDOWN: {
      switch( wParam ) {
        case VK_F1: onKeyDown( X3DKeyDeviceSensorNode::F1, true ); break;
        case VK_F2: onKeyDown( X3DKeyDeviceSensorNode::F2, true ); break;
        case VK_F3: onKeyDown( X3DKeyDeviceSensorNode::F3, true ); break;
        case VK_F4: onKeyDown( X3DKeyDeviceSensorNode::F4, true ); break;
        case VK_F5: onKeyDown( X3DKeyDeviceSensorNode::F5, true ); break;
        case VK_F6: onKeyDown( X3DKeyDeviceSensorNode::F6, true ); break;
        case VK_F7: onKeyDown( X3DKeyDeviceSensorNode::F7, true ); break;
        case VK_F8: onKeyDown( X3DKeyDeviceSensorNode::F8, true ); break;
        case VK_F9: onKeyDown( X3DKeyDeviceSensorNode::F9, true ); break;
        case VK_F10: onKeyDown( X3DKeyDeviceSensorNode::F10, true ); break;
        case VK_F11: onKeyDown( X3DKeyDeviceSensorNode::F11, true ); break;
        case VK_F12: onKeyDown( X3DKeyDeviceSensorNode::F12, true ); break;
        case VK_HOME: onKeyDown( X3DKeyDeviceSensorNode::HOME, true ); break;
        case VK_END: onKeyDown( X3DKeyDeviceSensorNode::END, true ); break;
        case VK_PRIOR: onKeyDown( X3DKeyDeviceSensorNode::PGUP, true ); break;
        case VK_NEXT: onKeyDown( X3DKeyDeviceSensorNode::PGDN, true ); break;
        case VK_UP: onKeyDown( X3DKeyDeviceSensorNode::UP, true ); break;
        case VK_DOWN: onKeyDown( X3DKeyDeviceSensorNode::DOWN, true ); break;
        case VK_LEFT: onKeyDown( X3DKeyDeviceSensorNode::LEFT, true ); break;
        case VK_RIGHT: onKeyDown( X3DKeyDeviceSensorNode::RIGHT, true ); break;
        case VK_MENU: onKeyDown( X3DKeyDeviceSensorNode::ALT, true ); break;
        case VK_CONTROL: onKeyDown( X3DKeyDeviceSensorNode::CONTROL, true );
          break;
        case VK_SHIFT: onKeyDown( X3DKeyDeviceSensorNode::SHIFT, true ); break;
        default: { }
      }
      break;
    }
    
    case WM_KEYUP:
    case WM_SYSKEYUP: {
      switch( wParam ) {
        case VK_F1: onKeyUp( X3DKeyDeviceSensorNode::F1, true ); break;
        case VK_F2: onKeyUp( X3DKeyDeviceSensorNode::F2, true ); break;
        case VK_F3: onKeyUp( X3DKeyDeviceSensorNode::F3, true ); break;
        case VK_F4: onKeyUp( X3DKeyDeviceSensorNode::F4, true ); break;
        case VK_F5: onKeyUp( X3DKeyDeviceSensorNode::F5, true ); break;
        case VK_F6: onKeyUp( X3DKeyDeviceSensorNode::F6, true ); break;
        case VK_F7: onKeyUp( X3DKeyDeviceSensorNode::F7, true ); break;
        case VK_F8: onKeyUp( X3DKeyDeviceSensorNode::F8, true ); break;
        case VK_F9: onKeyUp( X3DKeyDeviceSensorNode::F9, true ); break;
        case VK_F10: onKeyUp( X3DKeyDeviceSensorNode::F10, true ); break;
        case VK_F11: onKeyUp( X3DKeyDeviceSensorNode::F11, true ); break;
        case VK_F12: onKeyUp( X3DKeyDeviceSensorNode::F12, true ); break;
        case VK_HOME: onKeyUp( X3DKeyDeviceSensorNode::HOME, true ); break;
        case VK_END: onKeyUp( X3DKeyDeviceSensorNode::END, true ); break;
        case VK_PRIOR: onKeyUp( X3DKeyDeviceSensorNode::PGUP, true ); break;
        case VK_NEXT: onKeyUp( X3DKeyDeviceSensorNode::PGDN, true ); break;
        case VK_UP: onKeyUp( X3DKeyDeviceSensorNode::UP, true ); break;
        case VK_DOWN: onKeyUp( X3DKeyDeviceSensorNode::DOWN, true ); break;
        case VK_LEFT: onKeyUp( X3DKeyDeviceSensorNode::LEFT, true ); break;
        case VK_RIGHT: onKeyUp( X3DKeyDeviceSensorNode::RIGHT, true ); break;
        case VK_MENU: onKeyUp( X3DKeyDeviceSensorNode::ALT, true ); break;
        case VK_CONTROL: onKeyUp( X3DKeyDeviceSensorNode::CONTROL, true ); 
          break;
        case VK_SHIFT: onKeyUp( X3DKeyDeviceSensorNode::SHIFT, true ); break;
        default: {
          WPARAM key = wParam;
          BYTE state[ 256 ];
          WORD code[ 2 ];

          GetKeyboardState( state );

          if( ToAscii( static_cast<UINT>(key), 0, state, code, 0 ) == 1 )
            key=code[ 0 ];

          onKeyUp(static_cast<int>(key), false );
        }
      }
      break;
    }

    // When using ToAscii in the function called by
    // keyboardSpecialUpCallback the dead char gets
    // translated into something a bit strange. This
    // causes a deadchar message to look different
    // when the buttons is pushed than when it is
    // released. If the application want to take care
    // of deadchars then it should only care about
    // keyPress events.
    case WM_SYSDEADCHAR:
    case WM_DEADCHAR:
    case WM_SYSCHAR:
    case WM_CHAR: onKeyDown( static_cast<int>(wParam), false ); break;

    case WM_MENUCHAR: {
      if( GetKeyState( VK_MENU ) >= 0 )
        onKeyDown( static_cast<int>(wParam), false );
      break;
    }

    case WM_LBUTTONDOWN:
    case WM_LBUTTONDBLCLK: {
      onMouseButtonAction( MouseSensor::LEFT_BUTTON,
                           MouseSensor::DOWN );
      break;
    }

    case WM_LBUTTONUP: {
      onMouseButtonAction( MouseSensor::LEFT_BUTTON,
                           MouseSensor::UP );
      break;
    }

    case WM_MBUTTONDOWN:
    case WM_MBUTTONDBLCLK: {
      onMouseButtonAction( MouseSensor::MIDDLE_BUTTON,
                           MouseSensor::DOWN );
      break;
    }

    case WM_MBUTTONUP: {
      onMouseButtonAction( MouseSensor::MIDDLE_BUTTON,
                           MouseSensor::UP );
      break;
    }

    case WM_RBUTTONDOWN:
    case WM_RBUTTONDBLCLK: {
      onMouseButtonAction( MouseSensor::RIGHT_BUTTON,
                           MouseSensor::DOWN );
      break;
    }

    case WM_RBUTTONUP: {
      onMouseButtonAction( MouseSensor::RIGHT_BUTTON,
                           MouseSensor::UP );
      break;
    }

    case WM_MOUSEMOVE: {
      // Set mouse_position variable and call MouseSensor function.
      onMouseMotionAction( LOWORD(lParam), HIWORD(lParam) );
      break;
    }

    // WM_MOUSEWHEEL = 0x020A not defined unless 
    // _WIN32_WINNT or _WIN32_WINDOWS are defined before including windows.h
    case 0x020A: {
      short upOrDown = HIWORD( wParam );
      onMouseWheelAction( upOrDown > 0 ? 
                                  MouseSensor::FROM : MouseSensor::TOWARDS );
      break;
    }
  }

  // Call the original windows Procedure.
  return CallWindowProc(wpOrigProc, _hWnd, uMsg, 
        wParam, lParam); 
}
#endif // H3D_WINDOWS

string H3DWindowNode::takeScreenShot( const string &url ) {
  return takeScreenshot( url );
}

string H3DWindowNode::takeScreenshot( const string &url ) {
  if( url.empty() ) {
    ostringstream err;
    err << "Invalid argument(s) to function H3D.takeScreenshot( url ).";
    err << " url should not be empty.";
    return err.str();
  }

  H3DWindowNode* active_window_node = NULL;
  for( std::set< H3DWindowNode* >::iterator it = windows.begin(); it != windows.end(); ++it ) {
    if( (*it)->window_is_made_active ) {
      active_window_node = (*it);
      break;
    }
  }

  if( active_window_node == NULL ) {
    ostringstream err;
    err << "Could not take a screenshot. No active H3DWindowNode could be found.";
    return err.str();
  }

  GLint viewport[4];
  viewport[0] = static_cast< GLint >( active_window_node->viewports_size[0] );
  viewport[1] = static_cast< GLint >( active_window_node->viewports_size[1] );
  viewport[2] = static_cast< GLint >( active_window_node->viewports_size[2] );
  viewport[3] = static_cast< GLint >( active_window_node->viewports_size[3] );

  auto_ptr< Image > image( new PixelImage( viewport[2], viewport[3], 1, 24, Image::RGB,Image::UNSIGNED ) );
  glPixelStorei( GL_PACK_ALIGNMENT, 1 );
  glPixelStorei( GL_PACK_SKIP_PIXELS, 0 );
  glPixelStorei( GL_PACK_ROW_LENGTH, 0 );
  glPixelStorei( GL_PACK_SKIP_ROWS, 0 );
  glReadPixels( viewport[0], viewport[1], viewport[2], viewport[3], GL_RGB, GL_UNSIGNED_BYTE, image->getImageData() );

  GLenum error = glGetError();
  if( error != GL_NO_ERROR ) {
    ostringstream err;
    err << "ReadPixels failed"
      << ", screenshot will not be taken:"
      << gluErrorString(error);
    return err.str();
  }

#ifdef HAVE_FREEIMAGE
  if( H3DUtil::saveFreeImagePNG ( url, *image ) )
    return "";
  else {
    ostringstream err;
    err << "Could not take screenshot! Saving to url " << url
      << " failed. Make sure that the url is valid and that the application has the required permissions.";
    return err.str();
  }
#else // HAVE_FREEIMAGE
  return string( "Warning: Could not take screenshot! Compiled without the required FreeImage library." );
#endif // HAVE_FREEIMAGE
}

void H3D::H3DWindowNode::WindowSizeChanged::update() {
  TypedField< SFBool, Types<SFInt32, SFInt32> >::update();
  H3DInt32 width_current = static_cast<SFInt32 *>(routes_in[0])->getValue();
  H3DInt32 height_current = static_cast<SFInt32 *>(routes_in[1])->getValue();
  if( !prev_set || width_prev != width_current || height_prev != height_current ) {
    // window size changed
    dynamic_cast<H3DWindowNode*>(getOwner())->renderMode->updateRebuildStencilMaskFlag();
  }
  width_prev = width_current;
  height_prev = height_current;
  prev_set = true;
}
