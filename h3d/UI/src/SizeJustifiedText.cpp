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
/// \file SizeJustifiedText.cpp
/// \brief CPP file for SizeJustifiedText, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/SizeJustifiedText.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase SizeJustifiedText::database( 
                               "SizeJustifiedText", 
                               &(newInstance<SizeJustifiedText>), 
                               typeid( SizeJustifiedText ),
                               &Text::database );

namespace SizeJustifiedTextInternals {
  FIELDDB_ELEMENT( SizeJustifiedText, size, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SizeJustifiedText, textStartPos, OUTPUT_ONLY )
}

SizeJustifiedText::SizeJustifiedText( Inst< SFNode         > _metadata,
                                      Inst< SFBound         > _bound,
                                      Inst< DisplayList     > _displayList,
                                      Inst< SFFontStyleNode > _fontStyle,
                                      Inst< MFFloat         > _length,
                                      Inst< SFFloat         > _maxExtent,
                                      Inst< MFString        > _string,
                                      Inst< MFVec2f         > _lineBounds,
                                      Inst< SFVec3f         > _origin,
                                      Inst< SFVec2f         > _textBounds,
                                      Inst< SFBool          > _solid,
                                      Inst< SFVec2f         > _size,
                                      Inst< SFVec3f > _textStartPos ) :
  Text( _metadata, _bound, _displayList, _fontStyle, _length,
        _maxExtent, _string, _lineBounds, _origin, _textBounds, _solid ),
  size( _size ),
  textStartPos( _textStartPos ) {

  type_name = "SizeJustifiedText";
  database.initFields( this );
  
  size->setValue( Vec2f( 1, 1 ) );
  textStartPos->setValue( Vec3f( 0, 0, 0 ), id );

  size->route( bound );
  size->route( displayList );

}

void SizeJustifiedText::justifyMinor( const vector< string > &lines,
                         X3DFontStyleNode *font ) {
  if( lines.size() == 0 ) return;
  
  X3DFontStyleNode::Alignment alignment = font->getAlignment();
  bool left_to_right = font->isLeftToRight();
  bool top_to_bottom = font->isTopToBottom();
  X3DFontStyleNode::Justification minor = font->getMinorJustification();
  const Vec2f &j_size = size->getValue();

  // horizontal text
  if( alignment == X3DFontStyleNode::HORIZONTAL ) {
    if( top_to_bottom ) {
      if( minor == X3DFontStyleNode::BEGIN ) {
        glTranslatef( 0, j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        glTranslatef( 0, j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        glTranslatef( 0, -j_size.y/2, 0 );
      }
    } else {
      if( minor == X3DFontStyleNode::BEGIN ) {
        glTranslatef( 0, -j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        glTranslatef( 0, -j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        glTranslatef( 0, j_size.y/2, 0 );
      }
    }
    
    // vertical text
  } else if( alignment == X3DFontStyleNode::VERTICAL ) {
    if( left_to_right ) {
      if( minor == X3DFontStyleNode::BEGIN ) {
        glTranslatef( -j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        glTranslatef( -j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        glTranslatef( j_size.x/2, 0, 0 );
      }
    } else {
      if( minor == X3DFontStyleNode::BEGIN ) {
        glTranslatef( j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        glTranslatef( j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        glTranslatef( -j_size.x/2 , 0, 0 );
      }
    }
  }
  Text::justifyMinor( lines, font );
}

void SizeJustifiedText::justifyLine( const string& text,
                                   X3DFontStyleNode *font ) {
  X3DFontStyleNode::Justification major = font->getMajorJustification();
  X3DFontStyleNode::Alignment alignment = font->getAlignment();
  bool left_to_right = font->isLeftToRight();
  bool top_to_bottom = font->isTopToBottom();
  const Vec2f &j_size = size->getValue();

  if( text != "" ) {
    // horizontal text
    if( alignment == X3DFontStyleNode::HORIZONTAL ) {
      if( major == X3DFontStyleNode::END ) {
        if( left_to_right) glTranslatef( j_size.x/2, 0, 0 );
        else  glTranslatef( -j_size.x/2, 0, 0 );
      } else if( major == X3DFontStyleNode::BEGIN ||
                 major == X3DFontStyleNode::FIRST ) {
        if( left_to_right ) glTranslatef( -j_size.x/2, 0, 0 );
        else glTranslatef( j_size.x/2, 0, 0 );
      }
      // vertical text
    } else if( alignment == X3DFontStyleNode::VERTICAL ) {
      if( major == X3DFontStyleNode::END ) {
        if( top_to_bottom ) glTranslatef( 0, -j_size.y/2, 0 );
        else glTranslatef( 0, j_size.y/2, 0 );
      } else if( major == X3DFontStyleNode::BEGIN ||
                 major == X3DFontStyleNode::FIRST ) {
        if( top_to_bottom ) glTranslatef( 0, j_size.y/2, 0 );
        else glTranslatef( 0, -j_size.y/2, 0 );
      }
    }
  }  
  Text::justifyLine( text, font );
}

Vec3f SizeJustifiedText::getMinorJustification( X3DFontStyleNode *font ) {
  X3DFontStyleNode::Alignment alignment = font->getAlignment();
  bool left_to_right = font->isLeftToRight();
  bool top_to_bottom = font->isTopToBottom();
  X3DFontStyleNode::Justification minor = font->getMinorJustification();
  const Vec2f &j_size = size->getValue();

  // horizontal text
  if( alignment == X3DFontStyleNode::HORIZONTAL ) {
    if( top_to_bottom ) {
      if( minor == X3DFontStyleNode::BEGIN ) {
        return Vec3f( 0, j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        return Vec3f( 0, j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        return Vec3f( 0, -j_size.y/2, 0 );
      }
    } else {
      if( minor == X3DFontStyleNode::BEGIN ) {
        return Vec3f( 0, -j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        return Vec3f( 0, -j_size.y/2, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        return Vec3f( 0, j_size.y/2, 0 );
      }
    }
    
    // vertical text
  } else if( alignment == X3DFontStyleNode::VERTICAL ) {
    if( left_to_right ) {
      if( minor == X3DFontStyleNode::BEGIN ) {
        return Vec3f( -j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        return Vec3f( -j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        return Vec3f( j_size.x/2, 0, 0 );
      }
    } else {
      if( minor == X3DFontStyleNode::BEGIN ) {
        return Vec3f( j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::FIRST ) {
        return Vec3f( j_size.x/2, 0, 0 );
      } else if( minor == X3DFontStyleNode::END ) {
        return Vec3f( -j_size.x/2 , 0, 0 );
      }
    }
  }
  return Vec3f( 0, 0, 0 );
}

Vec3f SizeJustifiedText::getMajorJustification( X3DFontStyleNode *font ) {
  X3DFontStyleNode::Justification major = font->getMajorJustification();
  X3DFontStyleNode::Alignment alignment = font->getAlignment();
  bool left_to_right = font->isLeftToRight();
  bool top_to_bottom = font->isTopToBottom();
  const Vec2f &j_size = size->getValue();

    // horizontal text
    if( alignment == X3DFontStyleNode::HORIZONTAL ) {
      if( major == X3DFontStyleNode::END ) {
        if( left_to_right) return Vec3f( j_size.x/2, 0, 0 );
        else  return Vec3f( -j_size.x/2, 0, 0 );
      } else if( major == X3DFontStyleNode::BEGIN ||
                 major == X3DFontStyleNode::FIRST ) {
        if( left_to_right ) return Vec3f( -j_size.x/2, 0, 0 );
        else return Vec3f( j_size.x/2, 0, 0 );
      }
      // vertical text
    } else if( alignment == X3DFontStyleNode::VERTICAL ) {
      if( major == X3DFontStyleNode::END ) {
        if( top_to_bottom ) return Vec3f( 0, -j_size.y/2, 0 );
        else return Vec3f( 0, j_size.y/2, 0 );
      } else if( major == X3DFontStyleNode::BEGIN ||
                 major == X3DFontStyleNode::FIRST ) {
        if( top_to_bottom ) return Vec3f( 0, j_size.y/2, 0 );
        else return Vec3f( 0, -j_size.y/2, 0 );
      }
    
  }  
  return Vec3f( 0, 0, 0 );
}

void SizeJustifiedText::SFBound::update() {
  X3DFontStyleNode *font_style = 
    static_cast< SFFontStyleNode * >( routes_in[0] )->getValue(); 
  const Vec2f &_size = 
    static_cast< SFVec2f * >( routes_in[4] )->getValue();

  Text::SFBound::update();
  BoxBound *bb = dynamic_cast< BoxBound * >( value.get() );

  if( bb ) {
     X3DFontStyleNode::Alignment alignment = font_style->getAlignment();
     X3DFontStyleNode::Justification minor = font_style->getMinorJustification();
     X3DFontStyleNode::Justification major = font_style->getMajorJustification();
     Vec3f bb_center_change( 0, 0, 0 );

    if( alignment == X3DFontStyleNode::HORIZONTAL ) {
      if( major == X3DFontStyleNode::BEGIN ||
          major == X3DFontStyleNode::FIRST ) 
        bb_center_change.x -= _size.x / 2;
      else if( major == X3DFontStyleNode::END )
        bb_center_change.x += _size.x / 2;
      if( minor == X3DFontStyleNode::BEGIN ||
          minor == X3DFontStyleNode::FIRST ) 
        bb_center_change.y += _size.y / 2;
      else if( minor == X3DFontStyleNode::END )
        bb_center_change.y -= _size.y / 2;
    } else {
      if( major == X3DFontStyleNode::BEGIN ||
          major == X3DFontStyleNode::FIRST ) 
        bb_center_change.y += _size.y / 2;
      else if( major == X3DFontStyleNode::END )
        bb_center_change.y -= _size.y / 2;
      if( minor == X3DFontStyleNode::BEGIN ||
          minor == X3DFontStyleNode::FIRST  ) 
        bb_center_change.x -= _size.x / 2;
      else if( minor == X3DFontStyleNode::END )
        bb_center_change.x += _size.x / 2;
    }
    
    if( !font_style->isLeftToRight() ) bb_center_change.x = -bb_center_change.x;
    if( !font_style->isTopToBottom() ) bb_center_change.y = -bb_center_change.y;

    bb->center->setValue( bb->center->getValue() + bb_center_change );
  }
}
