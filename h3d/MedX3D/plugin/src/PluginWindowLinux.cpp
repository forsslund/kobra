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
/// \file PluginWindowLinux.h
/// \brief CPP file for PluginWindowLinux. Class subclassing H3DWindowNode in
/// order to connect the plugin with H3DAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

#include "PluginWindowLinux.h"

#ifdef H3D_LINUX

using namespace H3D;

// Initialize Class Default Values
PluginWindowLinux::PluginWindowLinux(
                    Inst< SFInt32     > _width,
                    Inst< SFInt32     > _height,
                    Inst< SFBool      > _fullscreen,
                    Inst< SFBool      > _mirrored,
                    Inst< RenderMode  > _renderMode, 
                    Inst< SFViewpoint > _viewpoint ) :
  H3DWindowNode( _width, _height, _fullscreen, _mirrored, _renderMode,
    _viewpoint ) {
  type_name = "PluginWindowLinux";
  database.initFields( this );
  keyboardDown = false;
  keyboardUp = false;

  xdisplay  = 0;
  xwindow   = 0;
  glcontext = 0;  

#ifdef PRINT_ERR_FILE
  cerr << "PluginWindowLinux created " << this << endl;
#endif

  fullscreen->setValue(false);
  renderMode->setValue( "MONO" );

}

PluginWindowLinux::~PluginWindowLinux() {
#ifdef PRINT_ERR_FILE
  cerr << "~PluginWindowLinux " << this << endl;
#endif
  glXMakeCurrent (xdisplay, None, NULL);
  glXDestroyContext (xdisplay, glcontext);
}

void PluginWindowLinux::initWindow(){}

void PluginWindowLinux::initWindowHandler(){}

bool PluginWindowLinux::initialize( Display *display, Window window, Visual *visual ) {
#ifdef PRINT_ERR_FILE
  cerr << "PluginWindowsLinux::initialize" << endl;
#endif

  xdisplay = display;
  xwindow = window;
  xvisual = visual;

  XVisualInfo     xvisualtemplate;
  xvisualtemplate.visualid = XVisualIDFromVisual ( xvisual );
  int n;
  XVisualInfo *xvisualinfo = XGetVisualInfo (xdisplay, VisualIDMask, &xvisualtemplate, &n);
  
  // create context
  if (( glcontext = glXCreateContext (xdisplay, xvisualinfo, NULL, GL_TRUE)) == NULL) {
#ifdef PRINT_ERR_FILE
    cerr << "Could not create GL rendering context." << endl;
#endif
    return false;
  }

  glXMakeCurrent (xdisplay, xwindow, glcontext);

  return true;
}

void PluginWindowLinux::setFullscreen( bool _fullscreen ){
  if( _fullscreen )
    Console(4) << "Fullscreen mode not supported by plugin" << endl;
}

void PluginWindowLinux::makeWindowActive(){
#ifdef PRINT_ERR_FILE
  //cerr << "PluginWindowsLinux::makeWindowActive" << endl;
#endif
  // Make The Rendering Context Our Current Rendering Context
  if (!  glXMakeCurrent (xdisplay, xwindow, glcontext) ) {
    Console(4) << "Could not make window active" << endl;
  }
}

#endif // H3D_LINUX
