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
/// \file TouchButton.cpp
/// \brief CPP file for TouchButton.
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/TouchButton.h>
#include <H3D/UI/SizeJustifiedText.h>

// H3D includes
#include <H3D/TouchSensor.h> 
#include <H3D/Shape.h>

using namespace H3D;
 
H3DNodeDatabase TouchButton::database( 
        "TouchButton", 
        &(newInstance< TouchButton > ),
        typeid( TouchButton ),
        &H3DButtonNode::database 
        );

namespace ButtonInternals {
  FIELDDB_ELEMENT( TouchButton, color, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TouchButton, textColor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TouchButton, texture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TouchButton, isOver, OUTPUT_ONLY )
}

// H3D includes
#include <H3D/FontStyle.h>
#include <H3D/Box.h>
#include <H3D/Appearance.h>
#include <H3D/Material.h>
#include <H3D/X3DShapeNode.h>

TouchButton::TouchButton( Inst< SFNode           > _metadata,
                          Inst< SFBound          > _bound,
                          Inst< DisplayList      > _displayList,
                          Inst< SFString         > _tag,
                          Inst< SFBool           > _enabled,
                          Inst< SFVec3f          > _desiredSize,
                          Inst< SFVec3f          > _actualSize,
                          Inst< SFAppearanceNode > _appearance,
                          Inst< SFAppearanceNode > _textAppearance,
                          Inst< SFLayoutInfoNode > _layoutInfo,
                          Inst< MFString         > _text,
                          Inst< SFFontStyleNode  > _fontStyle,
                          Inst< SFBool           > _isPressed,
                          Inst< ButtonState      > _state,
                          Inst< SFString         > _buttonMode,
                          Inst< SFColor          > _color,
                          Inst< SFColor          > _textColor,
                          Inst< SFTextureNode    > _texture,
                          Inst< SFBool           > _isOver ):
  H3DButtonNode( _metadata, _bound, _displayList, _tag, 
                 _enabled, _desiredSize, _actualSize, _appearance, _textAppearance,
                 _layoutInfo, _text, _fontStyle,
                 _isPressed, _state, _buttonMode ),
  color( _color ),
  textColor( _textColor ),
  texture( _texture ),
  isOver( _isOver ),
  finalDiffuseColor( new FinalDiffuseColor ) {

  type_name = "TouchButton";
  database.initFields( this );

  color->setValue( RGB( 0.6f, 0.6f, 0.6f ) );
  textColor->setValue( RGB( 0, 0, 0 ) );

  TouchSensor *touch_sensor = new TouchSensor;
  transform->children->push_back( touch_sensor );

  touch_sensor->isOver->route(isOver, id);

  Box *b = new Box();
  b->size->setValue( Vec3f( 1, 1, 1 ) );
  widgetGeometry->setValue(  b );

  finalDiffuseColor->setName( "finalDiffuseColor" );
  color->route( finalDiffuseColor );
  state->route( finalDiffuseColor );

  // make the isPressed field be pressed if it is touched by haptics
  // device or pressed by mouse.
  orField.reset( new OrField() );
  orField->setName( "orField" );
  b->isTouched->route( orField );
  touch_sensor->isActive->route( orField );
  orField->route( isPressed, id );

}

void TouchButton::FinalDiffuseColor::update() {
  const RGB &base_color = 
      static_cast< SFColor * >( routes_in[0] )->getValue();
  bool is_pressed = 
      static_cast< SFBool * >( routes_in[1] )->getValue();

  if( is_pressed ) {
    Vec3f hsv = base_color.toHSV();
    hsv.y -= 0.15f;
    if( hsv.y < 0 ) {
      hsv.z -= hsv.y;
      if( hsv.z > 1 ) hsv.z = 1;
      hsv.y = 0;
    }
    value.fromHSV( hsv );
  } else {
    value = base_color;
  }
}

void TouchButton::initialize() {
  H3DButtonNode::initialize();

  Appearance *a = dynamic_cast<Appearance*>(appearance->getValue());
  if (a)
  {
    Material *m = dynamic_cast<Material*>(a->material->getValue());
    if (m)
    {
      // override color setting as there seems to be a Material provided
      color->setValue( m->diffuseColor->getValue() );
    }
    else
    {
      m = new Material();
      a->material->setValue(m);
    }
    texture->route( a->texture );
    finalDiffuseColor->route( m->diffuseColor );
  }
  Appearance *ta = dynamic_cast<Appearance*>(textAppearance->getValue());
  if (ta)
  {
    Material *m = dynamic_cast<Material*>(ta->material->getValue());
    if (m)
    {
      // override color setting as there seems to be a Material provided
      textColor->setValue( m->diffuseColor->getValue() );
    }
    else
    {
      m = new Material();
      ta->material->setValue(m);
    }
    textColor->route( m->diffuseColor );
  }
}

void TouchButton::resize( const Vec3f &new_size ) {
  const Vec3f &current_size = actualSize->getValue();
  Vec3f diff = current_size - new_size;
  if( diff * diff < Constants::f_epsilon ) return;  

  Transform *widget_xf;
  widget_defs.getNode( "TRANSFORM", widget_xf );

  Transform *text_xf;
  text_defs.getNode( "TRANSFORM", text_xf );

  Box * widget_geometry = static_cast< Box * >( widgetGeometry->getValue() );

  if( widget_geometry ) {
    widget_geometry->size->setValue( new_size ); 
  }

  Shape *_widget_shape;
  widget_defs.getNode( "SHAPE", _widget_shape );

  Shape *text_shape;
  text_defs.getNode( "SHAPE", text_shape );

  _widget_shape->geometry->upToDate();
  text_shape->geometry->upToDate();

  SizeJustifiedText *_text = 
    dynamic_cast< SizeJustifiedText * >( text_shape->geometry->getValue() );
  if( _text ) {
    _text->size->setValue( Vec2f( new_size.x, new_size.y ) );
  }
  
  BoxBound *text_bb = 
    dynamic_cast< BoxBound *>( text_xf->bound->getValue() );
  if( text_bb ) {
    const Vec3f &bound_size = text_bb->size->getValue();
    const Vec3f &bound_center = text_bb->center->getValue();
 
    Vec3f s( bound_size.x > Constants::f_epsilon &&
             bound_size.x > new_size.x ?
             new_size.x / bound_size.x : 1,
             bound_size.y > Constants::f_epsilon &&
             bound_size.y > new_size.y?
             new_size.y / bound_size.y : 1,
             bound_size.z > Constants::f_epsilon &&
             bound_size.z > new_size.z ?
             new_size.z / bound_size.z: 1 );
    H3DFloat min_scale = H3DMin( s.x, H3DMin( s.y, s.z ) );
    Vec3f center(0,0,0);
    if( _text ) {
      X3DFontStyleNode *font = _text->fontStyle->getValue();
      center = 
        _text->getMajorJustification( font ) +
        _text->getMinorJustification( font );
      text_xf->center->setValue( center );
    }

    BoxBound *wb = 
      dynamic_cast< BoxBound *>( widget_xf->transformedBound->getValue() );
    if( wb ) {
      const Vec3f &wb_size = wb->size->getValue();
      const Vec3f &wb_center = wb->center->getValue();
      H3DFloat closest_point = wb_center.z + wb_size.z /2;
      text_xf->translation->setValue( Vec3f( 0, 0, closest_point + 1e-4f ) );
    }
    text_xf->scale->setValue( s ); 
  }
  actualSize->setValue( new_size, id );
}

