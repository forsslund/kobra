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
/// \file TextureBackground.cpp
/// \brief CPP file for TextureBackground, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/TextureBackground.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase TextureBackground::database( 
                           "TextureBackground", 
                           &(newInstance< TextureBackground > ), 
                           typeid( TextureBackground ),
                           &X3DBackgroundNode::database );

namespace TextureBackgroundInternals {
  FIELDDB_ELEMENT( TextureBackground, backTexture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureBackground, frontTexture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureBackground, leftTexture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureBackground, rightTexture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureBackground, topTexture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureBackground, bottomTexture, INPUT_OUTPUT )
}

TextureBackground::TextureBackground( Inst< SFSetBind > _set_bind,
                                      Inst< SFNode    > _metadata,
                                      Inst< SFTime    > _bindTime,
                                      Inst< SFBool    > _isBound,
                                      Inst< DisplayList > _displayList,
                                      Inst< MFFloat   > _groundAngle,
                                      Inst< MFColor   > _groundColor,
                                      Inst< MFFloat   > _skyAngle,
                                      Inst< MFColor   > _skyColor,
                                      Inst< SFTextureNode  > _backTexture,
                                      Inst< SFTextureNode  > _frontTexture,
                                      Inst< SFTextureNode  > _leftTexture,
                                      Inst< SFTextureNode  > _rightTexture,
                                      Inst< SFTextureNode  > _topTexture,
                                      Inst< SFTextureNode  > _bottomTexture,
                                      Inst< SFFloat   > _transparency ) :
  X3DBackgroundNode( _set_bind, _metadata, _bindTime, _isBound, _displayList,
                    _groundAngle, _groundColor, _skyAngle, _skyColor, _transparency ),
  backTexture( _backTexture ),
  frontTexture( _frontTexture ),
  leftTexture( _leftTexture ),
  rightTexture( _rightTexture ),
  topTexture( _topTexture ),
  bottomTexture( _bottomTexture ) {
    
  type_name = "TextureBackground";
  database.initFields( this );

  backTexture->route( displayList );
  frontTexture->route( displayList );
  leftTexture->route( displayList );
  rightTexture->route( displayList );
  topTexture->route( displayList );
  bottomTexture->route( displayList );
}

void TextureBackground::render() {
  if( render_enabled ) {
    X3DBackgroundNode::render();
    glPushAttrib( GL_ALL_ATTRIB_BITS );
    H3DFloat s = (H3DFloat) 0.05;
    glCullFace( GL_BACK );
    glEnable( GL_CULL_FACE );
    glDisable( GL_LIGHTING );

    Matrix4d pm = projectionMatrix->getValue();
    H3DDouble projection_matrix[16] = { pm[0][0], pm[1][0], pm[2][0], pm[3][0],
      pm[0][1], pm[1][1], pm[2][1], pm[3][1],
      pm[0][2], pm[1][2], pm[2][2], pm[3][2],
      pm[0][3], pm[1][3], pm[2][3], pm[3][3]};
    glMatrixMode( GL_PROJECTION );
    glPushMatrix();
    glLoadMatrixd( projection_matrix );

    Matrix3f m = localToGlobal->getValue().getRotationPart();

    GLfloat mv[] = { 
      m[0][0], m[1][0], m[2][0], 0,
      m[0][1], m[1][1], m[2][1], 0,
      m[0][2], m[1][2], m[2][2], 0,
      0, 0, 0, 1 };
  
    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( mv );

    X3DTextureNode *front_texture = frontTexture->getValue();
    X3DTextureNode *back_texture = backTexture->getValue();
    X3DTextureNode *left_texture = leftTexture->getValue();
    X3DTextureNode *right_texture = rightTexture->getValue();
    X3DTextureNode *top_texture = topTexture->getValue();
    X3DTextureNode *bottom_texture = bottomTexture->getValue();

    glColor4f( 1, 1, 1, 1 );

    if( front_texture  ) {
      glPushAttrib( front_texture->getAffectedGLAttribs() );
      front_texture->preRender();
      front_texture->displayList->callList();
      glBegin( GL_QUADS );
      glNormal3f( 0, 0, 1 );
      renderTexCoordForTexture( Vec3f( 1, 1, 0 ), front_texture );
      glVertex3f( s, s, -s );
      renderTexCoordForTexture( Vec3f( 0, 1, 0 ), front_texture );
      glVertex3f( -s, s, -s );
      renderTexCoordForTexture( Vec3f( 0, 0, 0 ), front_texture );
      glVertex3f( -s, -s, -s );
      renderTexCoordForTexture( Vec3f( 1, 0, 0 ), front_texture );
      glVertex3f( s, -s, -s );
      glEnd();
      front_texture->postRender();
      glPopAttrib();
    }
  
    if( back_texture ) {
      glPushAttrib( back_texture->getAffectedGLAttribs() );
      back_texture->preRender();
      back_texture->displayList->callList();
      glBegin( GL_QUADS );
      glNormal3f( 0, 0, -1 );
      renderTexCoordForTexture( Vec3f( 0, 0, 0 ), back_texture );
      glVertex3f( s, -s, s );
      renderTexCoordForTexture( Vec3f( 1, 0, 0 ), back_texture );
      glVertex3f( -s, -s, s );
      renderTexCoordForTexture( Vec3f( 1, 1, 0 ), back_texture );
      glVertex3f( -s, s, s );
      renderTexCoordForTexture( Vec3f( 0, 1, 0 ), back_texture );
      glVertex3f( s, s, s );
      glEnd();
      back_texture->postRender();
      glPopAttrib();
    }

    if( left_texture ) {
      glPushAttrib( left_texture->getAffectedGLAttribs() );
      left_texture->preRender();
      left_texture->displayList->callList();
      glBegin( GL_QUADS );
      glNormal3f( 1, 0, 0 );
      renderTexCoordForTexture( Vec3f( 0, 1, 0 ), left_texture );
      glVertex3f( -s, s, s );
      renderTexCoordForTexture( Vec3f( 0, 0, 0 ), left_texture );
      glVertex3f( -s, -s, s );
      renderTexCoordForTexture( Vec3f( 1, 0, 0 ), left_texture );
      glVertex3f( -s, -s, -s );
      renderTexCoordForTexture( Vec3f( 1, 1, 0 ), left_texture );
      glVertex3f( -s, s, -s );
      glEnd();
      left_texture->postRender();
      glPopAttrib();
    }

    if( right_texture ) {
      glPushAttrib( right_texture->getAffectedGLAttribs() );
      right_texture->preRender();
      right_texture->displayList->callList();
      glBegin( GL_QUADS );
      glNormal3f( -1, 0, 0 );
      renderTexCoordForTexture( Vec3f( 0, 1, 0 ), right_texture );
      glVertex3f( s, s, -s );
      renderTexCoordForTexture( Vec3f( 0, 0, 0 ), right_texture );
      glVertex3f( s, -s, -s );
      renderTexCoordForTexture( Vec3f( 1, 0, 0 ), right_texture );
      glVertex3f( s, -s, s );
      renderTexCoordForTexture( Vec3f( 1, 1, 0 ), right_texture );
      glVertex3f( s, s, s );
      glEnd();
      right_texture->postRender();
      glPopAttrib();
    }

    if( top_texture ) {
      glPushAttrib( top_texture->getAffectedGLAttribs() );
      top_texture->preRender();
      top_texture->displayList->callList();
      glBegin( GL_QUADS );
      glNormal3f( 0, -1, 0 );
      renderTexCoordForTexture( Vec3f( 0, 1, 0 ), top_texture );
      glVertex3f( -s, s, s );
      renderTexCoordForTexture( Vec3f( 0, 0, 0 ), top_texture );
      glVertex3f( -s, s, -s );
      renderTexCoordForTexture( Vec3f( 1, 0, 0 ), top_texture );
      glVertex3f( s, s, -s );
      renderTexCoordForTexture( Vec3f( 1, 1, 0 ), top_texture );
      glVertex3f( s, s, s );
      glEnd();
      top_texture->postRender();
      glPopAttrib();
    }

    if( bottom_texture ) {
      glPushAttrib( bottom_texture->getAffectedGLAttribs() );
      bottom_texture->preRender();
      bottom_texture->displayList->callList();
      glBegin( GL_QUADS );
      glNormal3f( 0, 1, 0 );
      renderTexCoordForTexture( Vec3f( 1, 0, 0 ), bottom_texture );
      glVertex3f( s, -s, s );
      renderTexCoordForTexture( Vec3f( 1, 1, 0 ), bottom_texture );
      glVertex3f( s, -s, -s );
      renderTexCoordForTexture( Vec3f( 0, 1, 0 ), bottom_texture );
      glVertex3f( -s, -s, -s );
      renderTexCoordForTexture( Vec3f( 0, 0, 0 ), bottom_texture );
      glVertex3f( -s, -s, s );
      glEnd();
      bottom_texture->postRender();
      glPopAttrib();
    }


    glMatrixMode( GL_MODELVIEW );
    glPopMatrix(); 
    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    glPopAttrib();
  }
}
