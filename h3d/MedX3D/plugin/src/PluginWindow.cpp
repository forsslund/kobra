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
/// \file PluginWindow.h
/// \brief CPP file for PluginWindow. Class subclassing H3DWindowNode in
/// order to connect the plugin with H3DAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

#include "PluginWindow.h"

#ifdef H3D_WINDOWS

using namespace H3D;

static bool SetupH3DPixelFormat( HDC dc )
{
  /*HDC tempHDC = GetDC(NULL);
  int bitsPerPixel = GetDeviceCaps(tempHDC, BITSPIXEL);
  ReleaseDC(NULL, tempHDC);*/

  int bitsPerPixel = GetDeviceCaps(dc, BITSPIXEL);

  // pfd Tells Windows How We Want Things To Be
  PIXELFORMATDESCRIPTOR pfd = 
  {
    sizeof(PIXELFORMATDESCRIPTOR),  // Size Of This Pixel Format Descriptor
    1,                // Version Number
    PFD_DRAW_TO_WINDOW  |      // Format Must Support Window
    PFD_SUPPORT_OPENGL  |      // Format Must Support OpenGL
    PFD_DOUBLEBUFFER,        // Must Support Double Buffering
    PFD_TYPE_RGBA,          // Request An RGBA Format
    bitsPerPixel,          // Select Our Color Depth
    0, 0, 0, 0, 0, 0,        // Color Bits Ignored
    0,                // No Alpha Buffer
    0,                // Shift Bit Ignored
    0,                // No Accumulation Buffer
    0, 0, 0, 0,            // Accumulation Bits Ignored
    24,                // 24 Bit Z-Buffer (Depth Buffer)
    8,                // 8 Bit Stencil Buffer
    0,                // No Auxiliary Buffer
    PFD_MAIN_PLANE,          // Main Drawing Layer
    0,                // Reserved
    0, 0, 0              // Layer Masks Ignored
  };

  int pixelFormat;
  pixelFormat = ChoosePixelFormat( dc, &pfd );

  if( pixelFormat == 0 )
    return false;

  if( !SetPixelFormat( dc, pixelFormat, &pfd ) )
    return false;

  return true;
}

// Initialize Class Default Values
PluginWindow::PluginWindow(
                    Inst< SFInt32     > _width,
                    Inst< SFInt32     > _height,
                    Inst< SFBool      > _fullscreen,
                    Inst< SFBool      > _mirrored,
                    Inst< RenderMode  > _renderMode, 
                    Inst< SFViewpoint > _viewpoint ) :
  H3DWindowNode( _width, _height, _fullscreen, _mirrored, _renderMode,
    _viewpoint ) {
  type_name = "PluginWindow";
  database.initFields( this );
  keyboardDown = false;
  keyboardUp = false;

  hWnd            = 0;
  m_hDC           = 0;
  m_hRC           = 0;
#ifdef PRINT_ERR_FILE
  cerr << "PluginWindow created " << this << endl;
#endif

  fullscreen->setValue(false);
  renderMode->setValue( "MONO" );

}

PluginWindow::~PluginWindow() {
#ifdef PRINT_ERR_FILE
  cerr << "~PluginWindow " << this << endl;
#endif
  wglMakeCurrent(NULL, NULL);
  wglDeleteContext( m_hRC );
  ReleaseDC( hWnd, m_hDC );
}

void PluginWindow::initWindow(){}

void PluginWindow::initWindowHandler(){}

bool PluginWindow::initialize( HWND window ) {
  HDC dc = ::GetDC( window );
  if( !dc ) {
    return false;
  }

  if( !SetupH3DPixelFormat( dc ) ) {
    ReleaseDC( window, dc );
    return false;
  }

  HGLRC glrc = wglCreateContext( dc );
  if( !glrc ) {
    ReleaseDC( window, dc );
    return false;
  }

  if( !wglMakeCurrent(dc,glrc) ) {
    ReleaseDC( window, dc );
    return false;
  }

  m_hRC = glrc;
  m_hDC = dc;
  hWnd = window;

  return true;
}

void PluginWindow::setFullscreen( bool _fullscreen ){
  if( _fullscreen )
    Console(4) << "Fullscreen mode not supported by plugin" << endl;
}

void PluginWindow::makeWindowActive(){
  // Make The Rendering Context Our Current Rendering Context
  if (!wglMakeCurrent(m_hDC, m_hRC))
  {
    Console(4) << "Could not make window active" << endl;
  }
}

#endif // H3D_WINDOWS
