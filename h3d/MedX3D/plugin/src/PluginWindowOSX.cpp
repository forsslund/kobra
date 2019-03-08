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
/// \file PluginWindowOSX.h
/// \brief CPP file for PluginWindowOSX. Class subclassing H3DWindowNode in
/// order to connect the plugin with H3DAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

#include "PluginWindowOSX.h"

#ifdef H3D_OSX

using namespace H3D;

// Initialize Class Default Values
PluginWindowOSX::PluginWindowOSX(
                    Inst< SFInt32     > _width,
                    Inst< SFInt32     > _height,
                    Inst< SFBool      > _fullscreen,
                    Inst< SFBool      > _mirrored,
                    Inst< RenderMode  > _renderMode, 
                    Inst< SFViewpoint > _viewpoint ) :
  H3DWindowNode( _width, _height, _fullscreen, _mirrored, _renderMode,
    _viewpoint ) {
  type_name = "PluginWindowOSX";
  database.initFields( this );
  keyboardDown = false;
  keyboardUp = false;

  aglContext            = 0;
  browserWindow           = 0;

#ifdef PRINT_ERR_FILE
  cerr << "PluginWindowOSX created " << this << endl;
#endif

  fullscreen->setValue(false);
  renderMode->setValue( "MONO" );

}

PluginWindowOSX::~PluginWindowOSX() {
#ifdef PRINT_ERR_FILE
  cerr << "~PluginWindowOSX " << this << endl;
#endif
  aglDestroyContext( aglContext );
}

void PluginWindowOSX::initWindow(){}

void PluginWindowOSX::initWindowHandler(){}

bool PluginWindowOSX::initialize( WindowRef window ) {
#ifdef PRINT_ERR_FILE
  cerr << "PluginWindowsOSX::initialize" << endl;
#endif

  // set up pixel format
  int i = 0;
  GLint value;
  AGLPixelFormat pixelFormat;
  GLint queryAttribs[15];

  queryAttribs[i++] = AGL_RGBA;
  queryAttribs[i++] = AGL_DOUBLEBUFFER;
  queryAttribs[i++] = AGL_ACCELERATED; // ??
  queryAttribs[i++] = AGL_STENCIL_SIZE;
  queryAttribs[i++] = 8;

  queryAttribs[i++] = AGL_NONE; // end attributes vector

  if ((pixelFormat = aglCreatePixelFormat (queryAttribs)) == NULL) {
#ifdef PRINT_ERR_FILE
    cerr << "Could not create pixel format" << endl;
#endif
    return false;
  }

  // create context
  if ((aglContext = aglCreateContext (pixelFormat, NULL)) == NULL) {
#ifdef PRINT_ERR_FILE
    cerr << "Could not create GL rendering context." << endl;
#endif
    return false;
  } else {
    aglDestroyPixelFormat (pixelFormat);
  }

  // Bind context to window
  if (!aglSetWindowRef (aglContext, window)) {
    //if (!aglSetDrawable (aglContext, browserPort)) {
#ifdef PRINT_ERR_FILE
    cerr << "Could not bind GL rendering context to window." << endl;
#endif
    return false;
  }

  aglSetCurrentContext (aglContext);

  return true;
}

void PluginWindowOSX::setFullscreen( bool _fullscreen ){
  if( _fullscreen )
    Console(4) << "Fullscreen mode not supported by plugin" << endl;
}

void PluginWindowOSX::makeWindowActive(){
#ifdef PRINT_ERR_FILE
  //cerr << "PluginWindowsOSX::makeWindowActive" << endl;
#endif
  // Make The Rendering Context Our Current Rendering Context
  if (!aglSetCurrentContext (aglContext) )
  {
    Console(4) << "Could not make window active" << endl;
  }
}

#endif // H3D_OSX
