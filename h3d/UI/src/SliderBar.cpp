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
/// \file SliderBar.cpp
/// \brief .cpp file for SliderBar.
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/SliderBar.h>
#include <H3D/UI/SizeJustifiedText.h>

// H3D includes
#include <H3D/Shape.h>
#include <H3D/FontStyle.h>
#include <H3D/Box.h>
#include <H3D/Appearance.h>
#include <H3D/Material.h>
#include <H3D/Coordinate.h>
#include <H3D/LineSet.h>
#include <H3D/TouchSensor.h>

#include<iostream>

using namespace H3D;

H3DNodeDatabase SliderBar::database( 
        "SliderBar", 
        &newInstance<SliderBar>,
        typeid( SliderBar ),
        &H3DLabeledWidget::database 
        );

namespace SliderBarInternals {
  FIELDDB_ELEMENT( SliderBar, value, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, valueRange, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, stepLength, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, markerColor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, horizontal, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, color, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, textColor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, texture, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SliderBar, isActive, OUTPUT_ONLY )
}

SliderBar::SliderBar( Inst< SFNode          > _metadata,
                      Inst< SFBound         > _bound,
                      Inst< DisplayList     > _displayList,
                      Inst< SFString        > _tag,
                      Inst< SFBool          > _enabled,
                      Inst< SFVec3f         > _desiredSize,
                      Inst< SFVec3f         > _actualSize,
                      Inst< SFAppearanceNode > _appearance,
                      Inst< SFAppearanceNode > _textAppearance,
                      Inst< SFLayoutInfoNode > _layoutInfo,
                      Inst< MFString        > _text,
                      Inst< SFFontStyleNode > _fontStyle,
                      Inst< SFValue         > _value,
                      Inst< SFVec2f         > _valueRange,
                      Inst< SFFloat         > _stepLength,
                      Inst< SFColor         > _markerColor,
                      Inst< SFBool          > _horizontal,
                      Inst< SFColor         > _color,
                      Inst< SFColor         > _textColor,
                      Inst< SFTextureNode   > _texture,
                      Inst< SFBool          > _isActive ) :
  H3DLabeledWidget( _metadata, _bound, _displayList, _tag, 
                    _enabled, _desiredSize, _actualSize, _appearance, _textAppearance, 
                    _layoutInfo, _text, _fontStyle ),
  value( _value ),
  valueRange( _valueRange ),
  stepLength( _stepLength ),
  markerColor( _markerColor ),
  horizontal( _horizontal ),
  color( _color ),
  textColor( _textColor ),
  texture( _texture ),
  isActive( _isActive ),
  markerTranslation( new MarkerTranslation ),
  markerRotation( new MarkerRotation ),
  marker_transform( new Transform ) {

  type_name = "SliderBar";
  database.initFields( this );

  Transform *widget_xf;
  widget_defs.getNode( "TRANSFORM", widget_xf );

  // set up the touch sensor.
  TouchSensor *touch_sensor = new TouchSensor;
  widget_xf->children->push_back( touch_sensor );

  // set up the widget geometry
  Box *b = new Box();
  b->size->setValue( Vec3f( 1, 1, 1 ) );
  widgetGeometry->setValue( b );

  // set up the slider bar marker appearance
  Shape *shape_marker = new Shape;
  Appearance *app_marker = new Appearance;
  Material *mat_marker = new Material;
  LineSet *geom_marker = new LineSet;
  Coordinate *line_coords = new Coordinate;

  marker_transform->children->push_back( shape_marker );
  shape_marker->appearance->setValue( app_marker );
  app_marker->material->setValue( mat_marker );
  shape_marker->geometry->setValue( geom_marker );
  geom_marker->coord->setValue( line_coords );
  line_coords->point->push_back( Vec3f( 0, 0.6f, 0 ) );
  line_coords->point->push_back( Vec3f( 0, -0.6f, 0 ) );
  geom_marker->vertexCount->push_back( 2 );
  widget_xf->children->push_back( marker_transform.get() );

  markerColor->route( mat_marker->emissiveColor );

  value->setValue( 0, id );
  valueRange->setValue( Vec2f( 0, 1 ) );
  stepLength->setValue( 0 );
  markerColor->setValue( RGB( 0, 0, 0 ) );
  horizontal->setValue( true );
  color->setValue( RGB( 0.6f, 0.6f, 0.6f ) );
  textColor->setValue( RGB( 0, 0, 0 ) );


  b->isTouched->route( value, id );
  b->contactPoint->route( value, id );
  valueRange->route( value, id );
  stepLength->route( value, id );
  horizontal->route( value, id );
  touch_sensor->isActive->route( value, id );
  touch_sensor->hitPoint_changed->route( value, id ); 
  enabled->route(value, id);

  // if disable then also disable the touchsensor
  enabled->route(touch_sensor->enabled);

  // isActive
  orField.reset( new OrField );
  orField->setName( "orField" );
  touch_sensor->isActive->route( orField, id );
  b->isTouched->route( orField, id );
  orField->route( isActive, id );

  markerTranslation->setName( "markerTranslation" );
  markerTranslation->setOwner( this );
  value->route( markerTranslation );
  valueRange->route( markerTranslation );
  horizontal->route( markerTranslation );
  b->size->route( markerTranslation );

  markerRotation->setName( "markerRotation" );
  markerRotation->setOwner( this );
  horizontal->route( markerRotation );

  markerTranslation->route( marker_transform->translation );
  markerRotation->route( marker_transform->rotation );
}

void SliderBar::initialize() {
  H3DLabeledWidget::initialize();
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
    color->route( m->diffuseColor );
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

void SliderBar::SFValue::update() {
  const vector< bool > &is_touched = 
    static_cast< MFBool * >( routes_in[0] )->getValue();
  const vector< Vec3f > &touch_points = 
    static_cast< MFVec3f * >( routes_in[1] )->getValue();
  const Vec2f &value_range = 
    static_cast< SFVec2f * >( routes_in[2] )->getValue();
  H3DFloat step_length = 
    static_cast< SFFloat * >( routes_in[3] )->getValue();
  bool _horizontal = 
    static_cast< SFBool * >( routes_in[4] )->getValue();
  bool ts_is_active = 
    static_cast< SFBool * >( routes_in[5] )->getValue();
  const Vec3f & ts_hitpoint = 
    static_cast< SFVec3f * >( routes_in[6] )->getValue();
  bool sb_enabled = 
    static_cast< SFBool * >( routes_in[7] )->getValue();

  if (!sb_enabled) return;
  

  // this value is set to true below if the touch_point variable
  // is set in order to calculate a new value.
  bool value_change = false;
  Vec3f touch_point;

  // check haptic interaction
  if( is_touched.size() == touch_points.size() ) {
    for( unsigned int i = 0; i < is_touched.size(); ++i ) {
      if( is_touched[i] ) {
        value_change = true;
        touch_point = touch_points[i];
        break;
      }
    }
  }

  // check TouchSensor interaction
  if( !value_change ) {
    if( ts_is_active ) {
      value_change = true;
      touch_point = ts_hitpoint;
    }
  }

  // if the touch point has been set by either haptics or TouchSensor
  // change the value base on that interaction point.
  if( value_change ) {
    Vec3f scale = static_cast< SliderBar* >( getOwner() )->
                    marker_transform->scale->getValue();
    H3DFloat fraction;
    // The only difference caused by horizontal is which touch_point to
    // use. The scale value used should always be x since x and y for scale
    // is switched if horizontal is false. See code for
    // MarkerRotation::update() and SliderBar::resize.
    if( _horizontal ) {
      fraction = H3DUtil::H3DAbs( scale.x ) > Constants::f_epsilon ?
                ( touch_point.x + 0.5f * scale.x ) / scale.x : 0;
    } else {
      fraction = H3DUtil::H3DAbs( scale.x ) > Constants::f_epsilon ?
                ( touch_point.y + 0.5f * scale.x ) / scale.x : 0;
    }

    if( fraction < 0 ) fraction = 0;
    if( fraction > 1 ) fraction = 1;
    H3DFloat new_value;
    if( value_range.x < value_range.y )
      new_value = 
        value_range.x + ( value_range.y - value_range.x ) * fraction;
    else 
      new_value = 
        value_range.y - ( value_range.x - value_range.y ) * fraction;
    if( step_length == 0 ) {
      value = new_value;
    } else {
      value = H3DFloor( new_value / step_length ) * step_length; 
    }
  }
}

void SliderBar::MarkerTranslation::update() {
  SliderBar* owner = static_cast<SliderBar*>( getOwner() );
  if( owner && !owner->enabled->getValue() ) {
    return;
  }

  H3DFloat slider_value = 
    static_cast< SFValue * >( routes_in[0] )->getValue();
  const Vec2f &value_range = 
    static_cast< SFVec2f * >( routes_in[1] )->getValue();
  bool _horizontal = 
    static_cast< SFBool * >( routes_in[2] )->getValue();
  Vec3f size = 
    static_cast< SFVec3f * >( routes_in[3] )->getValue();
  H3DFloat fraction;
  if( value_range.x < value_range.y )
    fraction = 
      (slider_value - value_range.x) / ( value_range.y - value_range.x );
  else 
    fraction = 
      (value_range.x - slider_value) / ( value_range.x - value_range.y );
  if( _horizontal ) 
    value = Vec3f( size.x * ( fraction - 0.5f ), 0, 0 );
  else 
    value = Vec3f( 0, size.y * ( fraction - 0.5f ), 0 );
}

void SliderBar::MarkerRotation::update()  {
  bool _horizontal = 
    static_cast< SFBool * >( routes_in[0] )->getValue();
  SliderBar *sb = static_cast< SliderBar* >( getOwner() );

  // Switch x and y for marker scale value in order to be able to use
  // size when resizing instead of the default behaviour of changing the
  // scale for the entire widget. The reason for this is that some
  // haptic renderers can not handle non uniform scaling properly.
  Vec3f new_size = static_cast< Box * >(
      sb->widgetGeometry->getValue() )->size->getValue();
  if( _horizontal ) {
    sb->marker_transform->scale->setValue( new_size );
    value = Rotation( 1, 0, 0, 0 );
  } else {
    sb->marker_transform->scale->setValue(
        Vec3f( new_size.y, new_size.x, new_size.z ) );
    value = Rotation( 0, 0, 1, (H3DFloat) Constants::pi/2 );
  }
}

void SliderBar::resize( const Vec3f &new_size ) {
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

  BoxBound *bb =
    dynamic_cast< BoxBound *>( marker_transform->bound->getValue() );
  if( bb ) {
    const Vec3f &bound_size = bb->size->getValue();

    // Switch x and y for marker scale value in order to be able to use
    // size when resizing instead of the default behaviour of changing the
    // scale for the entire widget. The reason for this is that some
    // haptic renderers can not handle non uniform scaling properly.
    if( horizontal->getValue() )
      marker_transform->scale->setValue( new_size );
    else
      marker_transform->scale->setValue(
        Vec3f( new_size.y, new_size.x, new_size.z ) );
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

