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
/// \file FrameBufferTexture.h
/// \brief Header file for FrameBufferTexture.
/// node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FRAMEBUFFERTEXTURE_H__
#define __FRAMEBUFFERTEXTURE_H__

#include <H3D/X3DTexture2DNode.h>
#include <H3D/MedX3D/MedX3D.h>

namespace H3D {

  /// \ingroup H3DNodes
  /// \class FrameBufferTexture
  /// This texture node transfers the current OpenGL depth buffer or color
  /// buffer to a texture.
  ///
  /// The buffer is captured at the time the texture is rendered so the
  /// result depends on where it is placed in the scene-graph. 
  /// 
  /// The type field determines which buffer to capture. "DEPTH" captures
  /// the depth buffer and "COLOR" captures the color buffer.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/FrameBufferTexture.x3d">FrameBufferTexture.x3d</a>
  ///     ( <a href="x3d/FrameBufferTexture.x3d.html">Source</a> )
  ///
  class MEDX3D_API FrameBufferTexture : public X3DTexture2DNode {
  public:
        
    /// Constructor.
    FrameBufferTexture( Inst< DisplayList > _displayList = 0,
                        Inst< SFNode>  _metadata = 0,
                        Inst< SFString > _type = 0 );
        
    /// Destructor.
    ~FrameBufferTexture() {
      if( texture_id_initialized ) {
        glDeleteTextures( 1, &texture_id );
        texture_id = 0;
      }
      if( frame_buffer_id_initialized ) {
        glDeleteFramebuffersEXT( 1, &fbo_id );
      }
    }

    /// Virtual function for making all OpenGL calls that are needed to
    /// enable texturing for the texture.
    virtual void enableTexturing() {
      glEnable( GL_TEXTURE_2D );
    }

    /// Virtual function for making all OpenGL calls that are needed to
    /// disable texturing for the texture.
    virtual void disableTexturing() {
      glDisable( GL_TEXTURE_2D );
    }
        
    /// Returns the default xml containerField attribute value.
    /// For this node it is "texture".
    virtual string defaultXMLContainerField() {
      return "texture";
    }

    /// This function will be called by the X3DShapeNode before any rendering 
    /// of geometry and before the call to the render function. So this is the
    /// place to save the states that are going to be changed in render() in
    /// order to restore it in postRender().
    virtual void preRender() {
      glPushAttrib( GL_TEXTURE_BIT | GL_LIGHTING_BIT );
      setActiveTexture( this );
    }

    /// This function will be called by the X3DShapeNode after the geometry
    /// has been rendered to restore the states to what it was before 
    /// the call to preRender().
    virtual void postRender() {
      glPopAttrib();
      setActiveTexture( NULL );
    }

    /// Performes the OpenGL calls to Capture the selected frame buffer
    /// to the texture.
    virtual void render();

    /// The type field determines which buffer to capture. "DEPTH" captures
    /// the depth buffer and "COLOR" captures the color buffer.
    /// \note The COLOR mode will not catch lighting and shadows if
    /// FrameBufferTexture is used directly in the scene graph simply
    /// because shadows are not generated until after render has been
    /// called for all other objects and the stencil buffer is also
    /// not included.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> "DEPTH"
    /// <b>Valid values:</b> "DEPTH", "COLOR"
    auto_ptr< SFString > type;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// True of the texture_id parameter has been initialized.
    bool texture_id_initialized;
    bool frame_buffer_id_initialized;
    GLuint fbo_id;
    unsigned int last_width, last_height;
    string last_type;
  };
}

#endif
