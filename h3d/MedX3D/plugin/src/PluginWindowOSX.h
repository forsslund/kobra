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
/// \brief Header file for PluginWindowOSX. Class subclassing H3DWindowNode in
/// order to connect the plugin with H3DAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PLUGINWINDOWOSX_H__
#define __PLUGINWINDOWOSX_H__

#include <H3D/H3DWindowNode.h>
#include <H3D/Scene.h>

#ifdef H3D_OSX

#include <AGL/agl.h>

namespace H3D {

  class PluginWindowOSX : public H3DWindowNode
  {
  public:

    /// Constructor.
    PluginWindowOSX( 
               Inst< SFInt32    > _width      = 0,
               Inst< SFInt32     > _height     = 0,
               Inst< SFBool      > _fullscreen = 0,
               Inst< SFBool      > _mirrored   = 0,
               Inst< RenderMode  > _renderMode = 0, 
               Inst< SFViewpoint > _viewpoint  = 0 );

    /// Destructor.
    ~PluginWindowOSX();

    /// Virtual function to swap buffers.
    virtual void swapBuffers() { 
      aglSwapBuffers( aglContext );
    }

    /// Virtual function that should create a new window and set its properties
    /// depending on the fields.
    virtual void initWindow();

    /// Virtual function to initialize the window handler if needed.
    virtual void initWindowHandler();

    /// Virtual function to set whether the window should be fullscreen or not.
    virtual void setFullscreen( bool fullscreen );

    /// Virtual function to make the current window active, i.e. make 
    /// subsequent OpenGL calls draw in the context of this window.
    virtual void makeWindowActive();

    void showTheWindow();
    bool initialize( WindowRef window );

  protected:
    AGLContext aglContext;
    WindowRef browserWindow;
    Rect portRect;

    int m_BitsPerPixel;                        // Bits Per Pixel
    int keyboardDown;
    int keyboardUp;
  };
}


#endif  // H3D_OSX
#endif  // __PLUGINWINDOWOSX_H__
