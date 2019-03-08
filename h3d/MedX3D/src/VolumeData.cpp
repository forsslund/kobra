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
/// \file VolumeData.cpp
/// \brief CPP file for VolumeData, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/VolumeData.h>
#include <H3D/MedX3D/OpacityMapVolumeStyle.h>
#include <H3D/MedX3D/ProjectionVolumeStyle.h>

#include <H3D/X3DShapeNode.h>
#include <H3D/PixelTexture.h>

using namespace H3D;

  
H3DNodeDatabase VolumeData::database( "VolumeData", 
                                      &(newInstance<VolumeData>),
                                      typeid( VolumeData ),
                                      &X3DVolumeNode::database );

namespace VolumeDataInternals {
  FIELDDB_ELEMENT( VolumeData, renderStyle, INPUT_OUTPUT )
  FIELDDB_ELEMENT( VolumeData, voxels, INPUT_OUTPUT )
}

VolumeData::VolumeData( Inst< DisplayList > _displayList, 
                        Inst< SFVec3f > _dimensions,
                        Inst< SFNode > _metadata,
                        Inst< SFVolumeRenderStyleNode > _renderStyle,
                        Inst< SFTexture3DNode > _voxels, // obs SF
                        Inst< SurfaceNormals > _surfaceNormals,
                        Inst< SFBound > _bound,
                        Inst< SFVec3f > _bboxCenter,
                        Inst< SFVec3f > _bboxSize ) :
  X3DVolumeNode( _displayList,
                 _dimensions,
                 _metadata,
                 _bound,
                 _voxels,
                 _surfaceNormals,
                 _bboxCenter,
                 _bboxSize ),
  renderStyle( _renderStyle ) {

  type_name = "VolumeData";
  database.initFields( this );

  // routings
  //  renderStyle->route( displayList );
  
  renderStyle->route( emptySpaceClassificationTexture, id );
}

  
void VolumeData::render() {
  X3DTexture3DNode *v = voxels->getValue();
  if( v && v->image->getValue() ) {
    X3DVolumeNode::render();
  }
}
  
void VolumeData::updateUniformFields() {
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) style->updateUniformFields( this );
  X3DVolumeNode::updateUniformFields();
}

string VolumeData::addUniforms( ComposedShader *shader ) {
  string s = "";
  s += X3DVolumeNode::addUniforms( shader );

  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) s += style->addUniforms( shader );
  return s;
}

string VolumeData::getShaderCode() { 
  string s = "";
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  
  if( style && !( !usingRayCaster() && 
                  dynamic_cast< ProjectionVolumeStyle *>( style )) ) 
    s += style->getShaderCode();
  s += X3DVolumeNode::getShaderCode();
  return s;
}

string VolumeData::getShaderCodeOpacityOnly() { 
  string s = "";
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  
  if( style && !(!usingRayCaster() && 
                 dynamic_cast< ProjectionVolumeStyle *>( style )) ) 
    s += style->getShaderCodeOpacityOnly();
  //  s += X3DVolumeNode::getShaderCodeOpacityOnly();
  return s;
}

string VolumeData::getShaderPostCode() { 
  string s = "";
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) s += style->getShaderPostCode();
  s += X3DVolumeNode::getShaderPostCode();
  return s;
}

string VolumeData::getShaderInitCode() { 
  string s = "";  
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style && !(!usingRayCaster() && 
                 dynamic_cast< ProjectionVolumeStyle *>( style ) ) ) 
    s += style->getShaderInitCode();
  s += X3DVolumeNode::getShaderInitCode();
  return s;
}

string VolumeData::getShaderFunctions() { 
  string s = "";
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) s += style->getShaderFunctions();
  s += X3DVolumeNode::getShaderFunctions();
  return s;
}    

bool VolumeData::requiresDefaultNormals() { 
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) return style->requiresDefaultNormals();
  return false;
}  

bool VolumeData::requiresEnabledLights() { 
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) return style->requiresEnabledLights();
  return false;
}    

void VolumeData::SFVolumeRenderStyleNode::onAdd( Node *n) {
  SFVolumeRenderStyleNodeBase::onAdd( n );
  X3DVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DVolumeRenderStyleNode * >( n );
  VolumeData *vd = static_cast< VolumeData * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->route( vd->rebuildShader );
  }
}

void VolumeData::SFVolumeRenderStyleNode::onRemove( Node *n) {
  X3DVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DVolumeRenderStyleNode * >( n );
  VolumeData *vd = static_cast< VolumeData * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->unroute( vd->rebuildShader );
    vd->forceRebuildShader->touch();
  }
  SFVolumeRenderStyleNodeBase::onRemove( n );
}

string VolumeData::getShaderCompositingCode() {
  //const string &type = compositionType->getValue();
  X3DVolumeRenderStyleNode *style =  renderStyle->getValue();
  if( dynamic_cast< ProjectionVolumeStyle *>( style ) ) return "";//ProjectionComposition();
  else if( style && style->producesAssociatedColor( false ) ) {
    return frontToBackCompositionAssociated();
  } else {
    return frontToBackCompositionNonAssociated();
  }
}


void VolumeData::setSliceRenderBlendMode() {
  X3DVolumeRenderStyleNode *style =  renderStyle->getValue();
  if( dynamic_cast< ProjectionVolumeStyle *>( style ) ) {
    if( GLEW_EXT_blend_minmax ) {
      glBlendFunc(GL_ONE, GL_ONE);
      glBlendEquationEXT(GL_MAX_EXT);
    } else {
      Console(3) << "Warning: EXT_blend_minmax extension not supported by "
                 << "graphics board. ProjectionVolumeStyle not supported with "
                 << "slice based renderer. " << endl;
    }
  } else {
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  }
}

void VolumeData::traverseSG( TraverseInfo &ti ) {
  X3DVolumeNode::traverseSG( ti );
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) style->traverseSG( ti );

}

bool VolumeData::isEmptySpace( H3DFloat min_v, H3DFloat max_v ) {
  X3DVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) return style->isEmptySpace( min_v, max_v, max_v == 0 );
  return max_v == 0;
}
