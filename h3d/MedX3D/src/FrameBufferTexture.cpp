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
/// \file FrameBufferTexture.cpp
/// \brief CPP file for FrameBufferTexture.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/FrameBufferTexture.h>

using namespace H3D;

H3DNodeDatabase FrameBufferTexture::database( 
        "FrameBufferTexture", 
        &(newInstance<FrameBufferTexture>),
        typeid( FrameBufferTexture ),
        &X3DTextureNode::database 
        );

namespace FrameBufferTextureInternals {
  FIELDDB_ELEMENT( FrameBufferTexture, type, INPUT_OUTPUT )
}

FrameBufferTexture::FrameBufferTexture( 
                               Inst< DisplayList > _displayList,
                               Inst< SFNode>  _metadata,
                               Inst< SFString > _type ):
  X3DTexture2DNode( _displayList, _metadata ),
  type( _type ),
  texture_id_initialized( false ),
  frame_buffer_id_initialized( false ),
  last_width( 0 ),
  last_height( 0 ),
  last_type( "COLOR" ) {

  type_name = "FrameBufferTexture";
  database.initFields( this ); 

  type->addValidValue( "DEPTH" );
  type->addValidValue( "COLOR" );
  type->setValue( "DEPTH" );

  // turn off display list since we want to get new values of the width
  // and height each loop to see if they have changed.
  displayList->setCacheMode( H3DDisplayListObject::DisplayList::OFF );
}

bool checkFBOCompleteness( Node *n ) {
  // check for errors
  GLenum fbo_err = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if( fbo_err != GL_FRAMEBUFFER_COMPLETE_EXT ) {
    Console(4) << "Warning: Frame Buffer Object error:";
    switch(fbo_err) {
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT :
      Console(4) << "Attachment not complete" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT :
      Console(4) << "Wrong size of attachments" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT :
      Console(4) << "Draw buffer err" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT :
      Console(4) << "Color attachments have different formats" << endl;
      break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT :
      Console(4) << "No attachments" << endl;
      break;
    case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
      Console(4) << "Unsupported" << endl;
      break;
    default:
      Console(4) << "Unkown error" << endl;
      break;
    }
    Console(4) << " (in node " << n->getName() << "). " << endl;
    return false;
  }
  return true;
}

void FrameBufferTexture::render() {
  glGetIntegerv( GL_ACTIVE_TEXTURE_ARB, &texture_unit );

  if( !texture_id_initialized ) {
    // initialized texture paramters
    glGenTextures( 1, &texture_id );
    texture_id_initialized = true;
    glBindTexture(  GL_TEXTURE_2D, texture_id );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }

  // get the width and height of the buffer in pixels.
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  unsigned int width  = viewport[2];
  unsigned int height = viewport[3];

  // capture the buffer
  GLint previous_fbo_id = 0;
  if( GLEW_EXT_framebuffer_object ) {
    glGetIntegerv( GL_FRAMEBUFFER_BINDING, &previous_fbo_id );
  }
  const string &t = type->getValue();
  if( previous_fbo_id != 0 ) {
    if( !frame_buffer_id_initialized ) {
      frame_buffer_id_initialized = true;
      glGenFramebuffersEXT(1, &fbo_id);
    }
    if( last_width != width || last_height != height || t != last_type ) {
      glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, fbo_id );
      glBindTexture( GL_TEXTURE_2D, texture_id );
      if( t == "DEPTH" ) {
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
            GL_TEXTURE_2D, 0, 0 );
        glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, 
            GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
            GL_TEXTURE_2D, texture_id, 0 );
      } else {
         glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
            GL_TEXTURE_2D, 0, 0 );
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, 
            GL_RGBA, GL_FLOAT, NULL);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
            GL_TEXTURE_2D, texture_id, 0 );
      }
      last_width = width;
      last_height = height;
      last_type = t;
    }
    glBindFramebufferEXT( GL_READ_FRAMEBUFFER_EXT, previous_fbo_id );
    glBindFramebufferEXT( GL_DRAW_FRAMEBUFFER_EXT, fbo_id );
    glBindTexture( GL_TEXTURE_2D, texture_id );
    if( checkFBOCompleteness(this) ) {
      if( t == "DEPTH" ) {
        glBlitFramebufferEXT(0, 0, width, height, 0, 0, width, height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
      } else if( t == "COLOR" ) {
        glBlitFramebufferEXT(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
      } else {
        Console(4 ) << "Invalid type \"" << t << "\" in FrameBufferTexture. "
                    << "Should be \"DEPTH\" or \"COLOR\"" << endl;
      }
    }
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previous_fbo_id);
  } else {
    glBindTexture( GL_TEXTURE_2D, texture_id );
    if( t == "DEPTH" ) {
      glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
                       viewport[0], viewport[1], width, height,0);
    } else if( t == "COLOR" ) {
      glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                        viewport[0], viewport[1], width, height,0);
    } else {
      Console(4 ) << "Invalid type \"" << t << "\" in FrameBufferTexture. "
                  << "Should be \"DEPTH\" or \"COLOR\"" << endl;
    }
  }

  enableTexturing();
}




