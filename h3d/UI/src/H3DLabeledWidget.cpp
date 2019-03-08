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
/// \file H3DLabeledWidget.cpp
/// \brief .cpp file for H3DLabeledWidget.
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/H3DLabeledWidget.h>
#include <H3D/UI/SizeJustifiedText.h>

// H3D includes
#include <H3D/Shape.h>
#include <H3D/FontStyle.h>
#include <H3D/SmoothSurface.h>
#include <H3D/Material.h>

using namespace H3D;

H3DNodeDatabase H3DLabeledWidget::database( 
        "H3DLabeledWidget", 
        NULL,
        typeid( H3DLabeledWidget ),
        &H3DWidgetNode::database 
        );

namespace H3DLabeledWidgetInternals {
  FIELDDB_ELEMENT( H3DLabeledWidget, text, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DLabeledWidget, fontStyle, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DLabeledWidget, appearance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DLabeledWidget, textAppearance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DLabeledWidget, customNodes, INPUT_OUTPUT )
}

H3DLabeledWidget::H3DLabeledWidget( Inst< SFNode          > _metadata,
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
                                    Inst< MFNode          > _customNodes ) :
  H3DWidgetNode( _metadata, _bound, _displayList, _tag, 
                 _enabled, _desiredSize, _actualSize, _layoutInfo ),
  text( _text ),
  fontStyle( _fontStyle ),
  appearance( _appearance ),
  textAppearance( _textAppearance ),
  widgetGeometry( new SFGeometryNode ),
  customNodes ( _customNodes ) {

  Group* custom_group = new Group;
  customNodes->route( custom_group->children );
  transform->children->push_back( custom_group );

  string s = "<Transform DEF=\"TRANSFORM\"> <Shape DEF=\"SHAPE\"/> \
</Transform>";

  AutoRef<Node> widget_xf = createX3DNodeFromString( s, &widget_defs );
  Shape *_widget_shape;
  widget_defs.getNode( "SHAPE", _widget_shape );
  Appearance *a = new Appearance;
  appearance->setValue( a );
  Material *m = new Material();
  SmoothSurface *surface = new SmoothSurface();
  a->material->setValue( m );
  a->surface->setValue( surface );
  appearance->route( _widget_shape->appearance );
  widgetGeometry->setName( "widgetGeometry" );
  widgetGeometry->route( _widget_shape->geometry );
  transform->children->push_back( widget_xf.get() );

  AutoRef< Node > text_xf = createX3DNodeFromString( s, &text_defs );
  Shape *text_shape;
  text_defs.getNode( "SHAPE", text_shape );
  Appearance *ta = new Appearance;
  textAppearance->setValue( ta );
  textAppearance->route( text_shape->appearance );
  SizeJustifiedText *t = new SizeJustifiedText;
  text->route( t->stringF );
  fontStyle->route( t->fontStyle );
  text_shape->geometry->setValue( t );
  transform->children->push_back( text_xf.get() );

  type_name = "H3DLabeledWidget";

  database.initFields( this );
  widgetGeometry->setOwner( this );
  textAppearance->setOwner( this );

  FontStyle *fs = new FontStyle;
  fs->justify->clear();
  fs->justify->push_back( "MIDDLE" );
  fs->justify->push_back( "MIDDLE" );
  fontStyle->setValue( fs );

  t->textBounds->route( repack );
}

void H3DLabeledWidget::render() {
  H3DWidgetNode::render();
  transform->displayList->callList();
}

void H3DLabeledWidget::traverseSG( TraverseInfo &ti ) {
  H3DWidgetNode::traverseSG( ti );
  transform->traverseSG( ti );
}

void H3DLabeledWidget::resize( const Vec3f &new_size ) {
  const Vec3f &current_size = actualSize->getValue();
  Vec3f diff = current_size - new_size;
  if( diff * diff < Constants::f_epsilon ) return;  

  Transform *widget_xf;
  widget_defs.getNode( "TRANSFORM", widget_xf );
   
  Transform *text_xf;
  text_defs.getNode( "TRANSFORM", text_xf );
  
  Shape *_widget_shape;
  widget_defs.getNode( "SHAPE", _widget_shape );

  Shape *text_shape;
  text_defs.getNode( "SHAPE", text_shape );

  _widget_shape->geometry->upToDate();
  text_shape->geometry->upToDate();

  BoxBound *bb = dynamic_cast< BoxBound *>( widget_xf->bound->getValue() );
  if( bb ) {
    const Vec3f &bound_size = bb->size->getValue();
    const Vec3f &bound_center = bb->center->getValue();
    
    widget_xf->scale->setValue(Vec3f( bound_size.x > Constants::f_epsilon ?
                                      new_size.x / bound_size.x : 1,
                                      bound_size.y > Constants::f_epsilon ?
                                      new_size.y / bound_size.y : 1,
                                      bound_size.z > Constants::f_epsilon ?
                                      new_size.z / bound_size.z: 1 ) ); 
  }

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
