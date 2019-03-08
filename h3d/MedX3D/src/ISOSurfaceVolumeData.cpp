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
/// \file ISOSurfaceVolumeData.cpp
/// \brief CPP file for ISOSurfaceVolumeData, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/ISOSurfaceVolumeData.h>
#include <H3D/MedX3D/OpacityMapVolumeStyle.h>
#include <H3D/MedX3D/MIPVolumeStyle.h>
#include <H3D/X3DShapeNode.h>

using namespace H3D;

  
H3DNodeDatabase ISOSurfaceVolumeData::database( "ISOSurfaceVolumeData", 
            "IsoSurfaceVolumeData",
              &(newInstance<ISOSurfaceVolumeData>),
              typeid( ISOSurfaceVolumeData ),
              &X3DVolumeNode::database 
              );

namespace ISOSurfaceVolumeDataInternals {
  FIELDDB_ELEMENT( ISOSurfaceVolumeData, renderStyle, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ISOSurfaceVolumeData, voxels, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ISOSurfaceVolumeData, gradients, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ISOSurfaceVolumeData, surfaceValues, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ISOSurfaceVolumeData, contourStepSize, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ISOSurfaceVolumeData, surfaceTolerance, INPUT_OUTPUT )
}

ISOSurfaceVolumeData::ISOSurfaceVolumeData( 
            Inst< DisplayList > _displayList, 
            Inst< SFVec3f > _dimensions,
            Inst< SFNode > _metadata,
            Inst< MFVolumeRenderStyleNode > _renderStyle,
            Inst< SFTexture3DNode > _voxels, // obs SF
            Inst< SurfaceNormals > _surfaceNormals,
            Inst< SFBound > _bound,
            Inst< SFVec3f > _bboxCenter,
            Inst< SFVec3f > _bboxSize,
            Inst< MFFloat > _surfaceValues,
            Inst< SFFloat > _contourStepSize,
            Inst< SFFloat > _surfaceTolerance,
            Inst< SFTexture3DNode > _gradients ) :
  X3DVolumeNode( _displayList, 
     _dimensions, 
     _metadata, 
     _bound, 
     _voxels,
     _surfaceNormals,
     _bboxCenter, 
     _bboxSize ),
  renderStyle( _renderStyle ),
  surfaceValues( _surfaceValues ),
  contourStepSize( _contourStepSize ),
  surfaceTolerance( _surfaceTolerance ),
  gradients( _gradients ) {

  type_name = "IsoSurfaceVolumeData";
  database.initFields( this );


  contourStepSize->setValue( 0 );
  surfaceTolerance->setValue( 0 );
  // routings
  //  renderStyle->route( displayList );

  gradients->route( rebuildShader );
  surfaceTolerance->route( rebuildShader );
  contourStepSize->route( rebuildShader );
  surfaceValues->route( rebuildShader );
}
  
void ISOSurfaceVolumeData::render() {
  X3DTexture3DNode *v = voxels->getValue();
  if( v && v->image->getValue() && surfaceValues->size() != 0) {
    X3DVolumeNode::render();
  }
}
  
string ISOSurfaceVolumeData::getShaderInitCode() {
  string s = "";

  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    s += renderStyle->getValueByIndex(i)->getShaderInitCode();
  }

  s += X3DVolumeNode::getShaderInitCode();

  if( usingRayCaster() ) {
    s += 
      "  // return color\n"
      "  vec4 col=vec4(0,0,0,0), tfcol;\n"
      "\n"
      "  float v = 0.0, v1 = 0.0;\n"
      "  vec3 r, r1;\n"
      "  bool iso_init=false;\n";
  }

  return s;
}

string ISOSurfaceVolumeData::getShaderPostCode() {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    s += renderStyle->getValueByIndex(i)->getShaderPostCode();
  }
  s += X3DVolumeNode::getShaderPostCode();
  return s;
}

string ISOSurfaceVolumeData::getShaderCode() {
  bool slicing = !usingRayCaster();
  stringstream s;
  
  if( !slicing ) {
    s << 
      "r = r0.xyz - dir.xyz;\n"
      "   r1 = r0.xyz;\n"
      "  if( !iso_init ) { \n"
      "    v = sample_color.r; \n"
      "     v1 = sample_color.r; \n" 
      "    iso_init = true; \n"
      "  }  else { \n"
      "    v = v1; \n"
      "     v1 = sample_color.r; \n"
      "  float min_v =  min(v,v1); \n"
    "  float max_v = max( v, v1); \n";
  }

  s <<  "    int surface_index = -1; \n    float isovalue; \n";

  if(contourStepSize->getValue() == 0 || surfaceValues->size() > 1 ) {
    for( unsigned int i = 0; i < surfaceValues->size(); ++i ) {
      if( i != 0 ) s << " else {";
      s << "isovalue = surfaceValues[" << i << "];\n";

      if( slicing ) {
        s << "     if( sample_color.r >= isovalue ) { \n";
      } else {
        s << "     if( min_v<=isovalue && isovalue<=max_v ) { \n";
      }
      s  << "          surface_index = " << i << " ;\n"
        << "       } \n";
    }
    for( unsigned int i = 1; i < surfaceValues->size(); ++i ) {
      s << "}";
    }
  } else {
    s << " float base = surfaceValues[0];\n";
    s << " int base_index = int(base / contourStepSize ); \n"; 
    s << " float nbase = base - float(base_index) *contourStepSize;\n";
    
    if( slicing ) {
      // slicing only supports the first isosurface.
      s << "     if( sample_color.r >= nbase ) { \n";
      s  << "        surface_index = base_index == 0 ? 0: 1;\n";
      s  << "     }\n";
    } else {
      s << " int na = int( (min_v - nbase) / contourStepSize ); \n";
      s << " int nb = int( (max_v - nbase) / contourStepSize ); \n";
      s << "     if( na != nb || (min_v <= nbase && nbase <= max_v) ) { \n"
        << "          if( nb == base_index ) surface_index = 0; \n else "
        << "          if( nb > base_index ) surface_index = nb; \n"
        << "          else surface_index = nb + 1;\n"
        << "          isovalue = nbase + contourStepSize * float(nb); \n"
        << "       } \n";
    }
  }
  s << "    if( surface_index != -1 ) { \n";
  s << "  sample_color.a = 1.0; \n";
  
  bool end_if = false;

  if( !slicing ) {
    s << "// interpolate position \n"
      << "float a = clamp( (isovalue-v)/(v1-v), 0.0, 1.0 ); \n"
      << " vec4 temp_r = r0; \n"
      << " r0 = vec4( mix(r,r1,a), r0.a ); \n";
    
    if( requiresDefaultNormals() )
      s << "sample_default_normal =  normalizedNormalFromTexture( " << uniqueShaderName("defaultNormals" ) <<",r0); \n";

    if( surfaceTolerance->getValue() != 0 ) {
      if( gradients->getValue() ) {
        s << "float gradient_mag = length( normalFromTexture( gradients, r0 );\n";
      } else {
        s << "float gradient_mag = sample_default_normal.w; \n";
      }
      
      s << "if( gradient_mag > surfaceTolerance ) { \n";
      end_if = true;
    }
  }
  
  if( renderStyle->size() == 1 ) {
    s << renderStyle->getValueByIndex(0)->getShaderCode();
  } else {
    for(unsigned int i=0; i < renderStyle->size(); ++i) {
      if( i != 0 ) s << " else ";
      s << "if( surface_index == " << i << ") { \n"
        << renderStyle->getValueByIndex(i)->getShaderCode()
        << " } ";
    }
    if( renderStyle->size() > 0 && 
        (renderStyle->size() < surfaceValues->size() || 
         contourStepSize->getValue()  != 0 ) ) {
      s << "else " 
        << renderStyle->getValueByIndex(renderStyle->size() - 1 )->getShaderCode();
      }
   
  }
  
  if( slicing ) s << "\n } else { discard; } \n";
  
  if( end_if ) s << " } else { sample_color = vec4( 0, 0, 0, 0 ); }\n";

  //  s += X3DVolumeNode::getShaderCode();
  return s.str();
}


string ISOSurfaceVolumeData::getShaderCodeOpacityOnly() {
  bool slicing = !usingRayCaster();
  stringstream s;
  
  if( !slicing ) {
    s << 
      "r = r0.xyz - dir.xyz;\n"
      "   r1 = r0.xyz;\n"
      "  if( !iso_init ) { \n"
      "    v = sample_color.r; \n"
      "     v1 = sample_color.r; \n" 
      "    iso_init = true; \n"
      "  }  else { \n"
      "    v = v1; \n"
      "     v1 = sample_color.r; \n"
      "  float min_v =  min(v,v1); \n"
    "  float max_v = max( v, v1); \n";
  }

  s <<  "    int surface_index = -1; \n    float isovalue; \n";

  if(contourStepSize->getValue() == 0 || surfaceValues->size() > 1 ) {
    for( unsigned int i = 0; i < surfaceValues->size(); ++i ) {
      if( i != 0 ) s << " else {";
      s << "isovalue = surfaceValues[" << i << "];\n";

      if( slicing ) {
        s << "     if( sample_color.r >= isovalue ) { \n";
      } else {
        s << "     if( min_v<=isovalue && isovalue<=max_v ) { \n";
      }
      s  << "          surface_index = " << i << " ;\n"
        << "       } \n";
    }
    for( unsigned int i = 1; i < surfaceValues->size(); ++i ) {
      s << "}";
    }
  } else {
    s << " float base = surfaceValues[0];\n";
    s << " int base_index = int(base / contourStepSize ); \n"; 
    s << " float nbase = base - float(base_index) *contourStepSize;\n";
    
    if( slicing ) {
      // slicing only supports the first isosurface.
      s << "     if( sample_color.r >= nbase ) { \n";
      s  << "        surface_index = base_index == 0 ? 0: 1;\n";
      s  << "     }\n";
    } else {
      s << " int na = int( (min_v - nbase) / contourStepSize ); \n";
      s << " int nb = int( (max_v - nbase) / contourStepSize ); \n";
      s << "     if( na != nb || (min_v <= nbase && nbase <= max_v) ) { \n"
        << "          if( nb == base_index ) surface_index = 0; \n else "
        << "          if( nb > base_index ) surface_index = nb; \n"
        << "          else surface_index = nb + 1;\n"
        << "          isovalue = nbase + contourStepSize * float(nb); \n"
        << "       } \n";
    }
  }
  s << "    if( surface_index != -1 ) { \n";
  s << "  sample_color.a = 1.0; \n";
  
  bool end_if = false;

  if( !slicing ) {
    s << "// interpolate position \n"
      << "float a = clamp( (isovalue-v)/(v1-v), 0.0, 1.0 ); \n"
      << " vec4 temp_r = r0; \n"
      << " r0 = vec4( mix(r,r1,a), r0.a ); \n";
    
    if( requiresDefaultNormals() )
      s << "sample_default_normal =  normalizedNormalFromTexture(" << uniqueShaderName("defaultNormals") << ",r0); \n";

    if( surfaceTolerance->getValue() != 0 ) {
      if( gradients->getValue() ) {
        s << "float gradient_mag = length( normalFromTexture( gradients, r0 );\n";
      } else {
        s << "float gradient_mag = sample_default_normal.w; \n";
      }
      
      s << "if( gradient_mag > surfaceTolerance ) { \n";
      end_if = true;
    }
  }
  
  if( renderStyle->size() == 1 ) {
    s << renderStyle->getValueByIndex(0)->getShaderCodeOpacityOnly();
  } else {
    for(unsigned int i=0; i < renderStyle->size(); ++i) {
      if( i != 0 ) s << " else ";
      s << "if( surface_index == " << i << ") { \n"
        << renderStyle->getValueByIndex(i)->getShaderCodeOpacityOnly()
        << " } ";
    }
    if( renderStyle->size() > 0 && 
        (renderStyle->size() < surfaceValues->size() || 
         contourStepSize->getValue()  != 0 ) ) {
      s << "else " 
        << renderStyle->getValueByIndex(renderStyle->size() - 1 )->getShaderCodeOpacityOnly();
      }
   
  }
  
  if( slicing ) s << "\n } else { discard; } \n";
  
  if( end_if ) s << " } else { sample_color = vec4( 0, 0, 0, 0 ); }\n";

  //  s += X3DVolumeNode::getShaderCode();
  return s.str();
}

string ISOSurfaceVolumeData::getShaderFunctions() {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    s += renderStyle->getValueByIndex(i)->getShaderFunctions();
  }
  s += X3DVolumeNode::getShaderFunctions();
  return s;
}

void ISOSurfaceVolumeData::updateUniformFields() {
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    renderStyle->getValueByIndex(i)->updateUniformFields( this );
  }
  X3DVolumeNode::updateUniformFields();
}

bool ISOSurfaceVolumeData::requiresDefaultNormals() { 
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    if( renderStyle->getValueByIndex(i)->requiresDefaultNormals() )
      return true;
  }
  return false;
}  

bool ISOSurfaceVolumeData::requiresEnabledLights() { 
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    if( renderStyle->getValueByIndex(i)->requiresEnabledLights() )
      return true;
  }
  return false;
}    
  
string ISOSurfaceVolumeData::addUniforms( ComposedShader *shader ) {
  string s = "";
  for(unsigned int i=0; i < renderStyle->size(); ++i) {
    s += renderStyle->getValueByIndex(i)->addUniforms( shader );
  }

  if( usingRayCaster() && 
      !(contourStepSize->getValue() == 0 || surfaceValues->size() > 1 ) ) {
    // add the contourStepSize uniform to the shader
    s += addUniformToFragmentShader( shader,
                                     "contourStepSize", 
                                     "float",
                                     H3D::Field::INPUT_OUTPUT,
                                     copyAndRouteField( contourStepSize ) );
  }

  // add the contourStepSize uniform to the shader
  s += addUniformToFragmentShader( shader,
                                   "surfaceValues", 
           "float",
           H3D::Field::INPUT_OUTPUT,
           copyAndRouteField( surfaceValues ),
           surfaceValues->size() );

  if( usingRayCaster() && 
      surfaceTolerance->getValue() != 0 ) {
    // add the contourStepSize uniform to the shader
    s += addUniformToFragmentShader( shader,
                                     "surfaceTolerance", 
                                     "float",
                                     H3D::Field::INPUT_OUTPUT,
                                     copyAndRouteField( surfaceTolerance ) );
  }
  
  if( gradients->getValue() ) {
    s+= addUniformToFragmentShader( shader,
                                    "gradients",
                                    "sampler3D",
                                    H3D::Field::INPUT_OUTPUT,
                                    copyAndRouteField(  gradients ) );
  }
  
  s += X3DVolumeNode::addUniforms( shader );
  return s;
}

void ISOSurfaceVolumeData::MFVolumeRenderStyleNode::onAdd( Node *n) {
  X3DVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DVolumeRenderStyleNode * >( n );
  X3DVolumeNode *vd = static_cast< X3DVolumeNode * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->route( vd->rebuildShader );
  }
}

void ISOSurfaceVolumeData::MFVolumeRenderStyleNode::onRemove( Node *n) {
  X3DVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DVolumeRenderStyleNode * >( n );
  X3DVolumeNode *vd = static_cast< X3DVolumeNode * >( getOwner() );
  if( cs ) {
    cs->rebuildShader->unroute( vd->rebuildShader );
    vd->forceRebuildShader->touch();
  }
}

string ISOSurfaceVolumeData::getShaderCompositingCode() { 
  return frontToBackCompositionNonAssociated() + "\n r0 = temp_r; \n} } "; 
}
