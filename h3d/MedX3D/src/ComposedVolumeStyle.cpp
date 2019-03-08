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
/// \file ComposedVolumeStyle.cpp
/// \brief CPP file for ComposedVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/ComposedVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>
#include <H3D/MedX3D/OpacityMapVolumeStyle.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/TextureProperties.h>
 
using namespace H3D;
  
H3DNodeDatabase 
ComposedVolumeStyle::database( "ComposedVolumeStyle", 
                               &(newInstance<ComposedVolumeStyle>),
                               typeid( ComposedVolumeStyle ),
                               &X3DComposableVolumeRenderStyleNode::database );

namespace ComposedVolumeStyleInternals {
  FIELDDB_ELEMENT( ComposedVolumeStyle, ordered, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ComposedVolumeStyle, renderStyle, INPUT_OUTPUT )
}

ComposedVolumeStyle::ComposedVolumeStyle(
      Inst<DisplayList>_displayList,
      Inst<SFBool> _enabled,
      Inst<SFNode> _metadata,
      Inst< SFBool > _ordered,
      Inst< MFComposableVolumeRenderStyleNode > _renderStyle ) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    ordered( _ordered ),
    renderStyle( _renderStyle ) {
    type_name = "ComposedVolumeStyle";
    database.initFields( this );
    
    // defaults
    ordered->setValue( false );
    
    // routings
    ordered->route( displayList );
    renderStyle->route( displayList );

    renderStyle->route( rebuildShader );
  }
    
  string ComposedVolumeStyle::addUniforms( ComposedShader *shader ) {
    string s = "";
    for(int i=0; i<(int)renderStyle->size(); ++i) {
      // add uniforms from the renderStyle
      s += renderStyle->getValueByIndex(i)->addUniforms( shader );
    }
    return s;
  }
  
  string ComposedVolumeStyle::getShaderCode() {
    string s ="";
    for(int i=0; i<(int)renderStyle->size(); ++i) {
      // add shader code from the renderStyle
      s += renderStyle->getValueByIndex(i)->getShaderCode();
    }
    return s;
  }

  string ComposedVolumeStyle::getShaderCodeOpacityOnly() {
    string s ="";
    for(int i=0; i<(int)renderStyle->size(); ++i) {
      // add shader code from the renderStyle
      s += renderStyle->getValueByIndex(i)->getShaderCodeOpacityOnly();
    }
    return s;
  }
  
  void ComposedVolumeStyle::updateUniformFields( X3DVolumeNode *vd ) {
    for(int i=0; i<(int)renderStyle->size(); ++i) {
      // pre render fields from the renderStyle
      renderStyle->getValueByIndex(i)->updateUniformFields( vd );
    }
    
    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
  }
  

bool ComposedVolumeStyle::requiresDefaultNormals() { 
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    if( renderStyle->getValueByIndex(i)->requiresDefaultNormals() )
      return true;
  }
  return false;
}

bool ComposedVolumeStyle::requiresEnabledLights() {
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    if( renderStyle->getValueByIndex(i)->requiresEnabledLights() )
      return true;
  }
  return false;
}

string ComposedVolumeStyle::getShaderInitCode() {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    s += renderStyle->getValueByIndex(i)->getShaderInitCode();
  }
  s += X3DComposableVolumeRenderStyleNode::getShaderInitCode();
  return s;
}

string ComposedVolumeStyle::getShaderFunctions() {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    s += renderStyle->getValueByIndex(i)->getShaderFunctions();
  }
  s += X3DComposableVolumeRenderStyleNode::getShaderFunctions();
  return s;
}

void ComposedVolumeStyle::MFComposableVolumeRenderStyleNode::onAdd( Node *n) {
  X3DComposableVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DComposableVolumeRenderStyleNode * >( n );
  ComposedVolumeStyle *vd = 
    static_cast< ComposedVolumeStyle * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->route( vd->rebuildShader );
  }
}

void ComposedVolumeStyle::MFComposableVolumeRenderStyleNode::onRemove( Node *n) {
  X3DComposableVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DComposableVolumeRenderStyleNode * >( n );
  ComposedVolumeStyle *vd = 
    static_cast< ComposedVolumeStyle * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->unroute( vd->rebuildShader );
  }
}

bool ComposedVolumeStyle::producesAssociatedColor( bool input_associated ) {
  bool associated = input_associated; 
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    associated =  renderStyle->getValueByIndex(i)->producesAssociatedColor( associated );
  }
  return associated;
}

bool ComposedVolumeStyle::isEmptySpace( H3DFloat min_v, 
                                        H3DFloat max_v, 
                                        bool previous_empty ) {
  bool v = previous_empty;
  for( MFComposableVolumeRenderStyleNode::const_iterator n = 
         renderStyle->begin();
       n != renderStyle->end(); ++n ) {
    X3DComposableVolumeRenderStyleNode *style = 
      static_cast< X3DComposableVolumeRenderStyleNode *>(*n);
    v = style->isEmptySpace( min_v, max_v, v );
  }
  
  return v;
}


  
