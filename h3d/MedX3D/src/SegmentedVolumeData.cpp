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
/// \file SegmentedVolumeData.cpp
/// \brief CPP file for SegmentedVolumeData, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/SegmentedVolumeData.h>
#include <H3D/MedX3D/OpacityMapVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>
#include <H3D/X3DTexture2DNode.h>
#include <H3D/ShaderPart.h>
#include <H3D/Pixel3DTexture.h>

namespace H3D {
  
  H3DNodeDatabase SegmentedVolumeData::database( "SegmentedVolumeData",
          &(newInstance<SegmentedVolumeData>),
          typeid( SegmentedVolumeData ),
          &X3DVolumeNode::database );

  namespace SegmentedVolumeDataInternals {
    FIELDDB_ELEMENT( SegmentedVolumeData, renderStyle, INPUT_OUTPUT )
    FIELDDB_ELEMENT( SegmentedVolumeData, voxels, INPUT_OUTPUT )
    FIELDDB_ELEMENT( SegmentedVolumeData, segmentIdentifiers, INPUT_OUTPUT )
    FIELDDB_ELEMENT( SegmentedVolumeData, segmentEnabled, INPUT_OUTPUT )
  }

  SegmentedVolumeData::SegmentedVolumeData( Inst< DisplayList > _displayList, 
        Inst< SFVec3f > _dimensions,
        Inst< SFNode > _metadata,
        Inst< MFVolumeRenderStyleNode > _renderStyle,
        Inst< SFTexture3DNode > _voxels, 
        Inst< SFTexture3DNode > _segmentIdentifiers, 
        Inst< MFBool > _segmentEnabled,
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
    renderStyle( _renderStyle ),
    segmentIdentifiers( _segmentIdentifiers ),
    segmentEnabled( _segmentEnabled ),
    segmentMaxId( new SegmentMaxId ) {

  type_name = "SegmentedVolumeData";
  database.initFields( this );

  segmentMaxId->setOwner( this );

  segmentMaxId->setName( "segmentMaxId" );

  segmentMaxId->setValue( 0 );

  segmentIdentifiers->route( segmentMaxId );

  renderStyle->route( displayList );

  segmentEnabled->route( rebuildShader );
}

void SegmentedVolumeData::updateUniformFields() {
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
     X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
     if( n ) n->updateUniformFields( this );
  }
  X3DVolumeNode::updateUniformFields();
}

string SegmentedVolumeData::getShaderCode() {
  stringstream s;

  if( segmentIdentifiers->getValue() ) {
    s << "    float float_id = texture3D( segmentIdentifiers, r0.xyz ).r;\n" 
      << "    int segment_id = int( floor( 0.5 + float_id * float( segmentMaxId ) ) );\n";
  } else {
    s << "    int segment_id = 0; \n";
  }
  
  s << "    bool segment_enabled = true; \n";
  
  if( segmentEnabled->size() ) {
    s << "    if( segment_id >= 0 && segment_id < " <<
      segmentEnabled->size() << ") {\n";
    //    "      segment_enabled = segmentEnabled[ segment_id ]; \n" <<
    for( unsigned int _id = 0; _id < segmentEnabled->size(); ++_id  ) {
      s << "       if( segment_id == " << _id
        << " ) segment_enabled = segmentEnabled[" << _id << "]; \n"
        << "       else ";
    }
    
    s << "{}\n"
      << "    } \n ";
  }
  s << "    if( segment_enabled ) { \n";
  if( renderStyle->size() ) {
    for(unsigned int i=0; i < renderStyle->size(); ++i) {
      s << "if( segment_id == " << i << " ) { \n ";
      X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
      if( n ) s << n->getShaderCode();
      s << " } else ";
    }
    s << "   {} \n";
  }
    
  s << 
    "    } else { \n"
    "       sample_color = vec4( 0, 0, 0, 0 ); \n"
    "    } \n";

  return s.str();
}


string SegmentedVolumeData::getShaderCodeOpacityOnly() {
  stringstream s;

  if( segmentIdentifiers->getValue() ) {
    s << "    float float_id = texture3D( segmentIdentifiers, r0.xyz ).r;\n" 
      << "    int segment_id = int( floor( 0.5 + float_id * float( segmentMaxId ) ) );\n";
  } else {
    s << "    int segment_id = 0; \n";
  }
  
  s << "    bool segment_enabled = true; \n";
  
  if( segmentEnabled->size() ) {
    s << "    if( segment_id >= 0 && segment_id < " <<
      segmentEnabled->size() << ") {\n";
    //    "      segment_enabled = segmentEnabled[ segment_id ]; \n" <<
    for( unsigned int _id = 0; _id < segmentEnabled->size(); ++_id) {
      s << "       if( segment_id == " << _id
        << " ) segment_enabled = segmentEnabled[" << _id << "]; \n"
        << "       else ";
    }
    
    s << "{}\n"
      << "    } \n ";
  }
  s << "    if( segment_enabled ) { \n";
  if( renderStyle->size() ) {
    for(unsigned int i=0; i < renderStyle->size(); ++i) {
      s << "if( segment_id == " << i << " ) { \n ";
      X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
      if( n ) s << n->getShaderCodeOpacityOnly();
      s << " } else ";
    }
    s << "   {} \n";
  }
    
  s << 
    "    } else { \n"
    "       sample_color = vec4( 0, 0, 0, 0 ); \n"
    "    } \n";

  return s.str();
}

string SegmentedVolumeData::getShaderInitCode() {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
    if( n ) s += n->getShaderInitCode();
  }
  s += X3DVolumeNode::getShaderInitCode();
  return s;
}

string SegmentedVolumeData::getShaderPostCode() {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
    if( n ) s += n->getShaderPostCode();
  }
  s += X3DVolumeNode::getShaderPostCode();
  return s;
}

string SegmentedVolumeData::getShaderFunctions() {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
    if( n ) s += n->getShaderFunctions();
  }
  s += X3DVolumeNode::getShaderFunctions();
  return s;
}

bool SegmentedVolumeData::requiresDefaultNormals() { 
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
    if( n && n->requiresDefaultNormals() )
      return true;
  }
  return false;
}   

bool SegmentedVolumeData::requiresEnabledLights() { 
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
    if( n && n->requiresEnabledLights() )
      return true;
  }
  return false;
}    
  
string SegmentedVolumeData::addUniforms( ComposedShader *shader ) {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    X3DVolumeRenderStyleNode *n = renderStyle->getValueByIndex(i);
    if( n ) s += n->addUniforms( shader );
  }

  if( segmentEnabled->size() > 0 ) {
    // add segmentEnabled to the shader
    s += addUniformToFragmentShader( shader,
                                     "segmentEnabled", 
                                     "bool",
                                     H3D::Field::INPUT_OUTPUT,
                                     copyAndRouteField( segmentEnabled ),
                                     segmentEnabled->size() );
  }

  if( segmentIdentifiers->getValue() ) {
    // add the segmentIdentifiers to the shader
    s += addUniformToFragmentShader( shader,
                                     "segmentIdentifiers", 
                                     "sampler3D",
                                     H3D::Field::INPUT_OUTPUT,
                                     copyAndRouteField( segmentIdentifiers ) );
     
    SFInt32 *max_id = new SFInt32;
    segmentMaxId->route( max_id );
    s += addUniformToFragmentShader( shader,
                                     "segmentMaxId", 
                                     "int",
                                     H3D::Field::INPUT_OUTPUT,
                                     max_id );
  }

  s += X3DVolumeNode::addUniforms( shader );
  return s;
}
  
void SegmentedVolumeData::SegmentMaxId::update() {
  SFTexture3DNode *tf = static_cast< SFTexture3DNode * >( routes_in[0] );
  X3DTexture3DNode *t = tf ? tf->getValue() : NULL;
  
  value = 0;
  if( t ) {
    Image *i = t->image->getValue();
    if( i ) {
      unsigned int nr_bits = 0;
      switch( i->pixelType() ) {
        case Image::R:
        case Image::LUMINANCE: nr_bits = i->bitsPerPixel(); break;
        case Image::RG:
        case Image::LUMINANCE_ALPHA: nr_bits = i->bitsPerPixel() /2; break;
        case Image::VEC3:
        case Image::RGB: nr_bits = i->bitsPerPixel() /3; break;
        case Image::BGR: nr_bits = i->bitsPerPixel() /3; break;
        case Image::RGBA: nr_bits = i->bitsPerPixel() /3; break;
        case Image::BGRA: nr_bits = i->bitsPerPixel() /3; break;
      }
      value = (int)(H3DPow( 2.0, (double)nr_bits ) - 1);
    }
  }
}

  void SegmentedVolumeData::render() {
    X3DTexture3DNode *v = voxels->getValue();
    if( v && v->image->getValue() )
      X3DVolumeNode::render();
  }
  
  void  SegmentedVolumeData::traverseSG( TraverseInfo &ti ) {
    X3DVolumeNode::traverseSG( ti );
  }

void SegmentedVolumeData::MFVolumeRenderStyleNode::onAdd( Node *n) {
  X3DVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DVolumeRenderStyleNode * >( n );
  X3DVolumeNode *vd = static_cast< X3DVolumeNode * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->route( vd->rebuildShader );
  }
}

void SegmentedVolumeData::MFVolumeRenderStyleNode::onRemove( Node *n) {
  X3DVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DVolumeRenderStyleNode * >( n );
  X3DVolumeNode *vd = static_cast< X3DVolumeNode * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->unroute( vd->rebuildShader );
    vd->forceRebuildShader->touch();
  }
}
}
