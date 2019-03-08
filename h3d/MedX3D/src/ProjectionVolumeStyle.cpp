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
/// \file ProjectionVolumeStyle.cpp
/// \brief CPP file for ProjectionVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/ProjectionVolumeStyle.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>

namespace H3D {
  
  H3DNodeDatabase 
  ProjectionVolumeStyle::database( "ProjectionVolumeStyle", 
                            &(newInstance<ProjectionVolumeStyle>),
                            typeid( ProjectionVolumeStyle ),
                            &X3DVolumeRenderStyleNode::database );
  
  namespace ProjectionVolumeStyleInternals {
    FIELDDB_ELEMENT( ProjectionVolumeStyle, intensityThreshold, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ProjectionVolumeStyle, type, INPUT_OUTPUT )
    
  }
  
  ProjectionVolumeStyle::ProjectionVolumeStyle( 
                                  Inst< DisplayList >_displayList, 
                                  Inst< SFBool > _enabled,
                                  Inst< SFNode > _metadata,
                                  Inst< SFFloat > _intensityThreshold,
                                  Inst< SFString > _type ) :
    X3DVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    intensityThreshold( _intensityThreshold ),
    type( _type ) {
    
    type_name = "ProjectionVolumeStyle";
    database.initFields( this );

    intensityThreshold->setValue( 0 );

    type->addValidValue( "MAX" );
    type->addValidValue( "MIN" );
    type->addValidValue( "AVERAGE" );

    type->setValue( "MAX" );
    
    // routings
    intensityThreshold->route( displayList );
    type->route( displayList );

    intensityThreshold->route( rebuildShader );
    type->route( rebuildShader );
  }
  
  string ProjectionVolumeStyle::addUniforms( ComposedShader *s ) {
    return 
      addUniformToFragmentShader( s,
                                  UNIFORM_ID(enabled),
                                  "bool",
                                  H3D::Field::INPUT_OUTPUT, 
                                  copyAndRouteField(enabled) );
  }

  string ProjectionVolumeStyle::getShaderInitCode() {
    string s;

    const string &t = type->getValue();
    
    if( t == "MAX" ) {
      s = "float max_intensity = 0.0; \n";
    } else if( t == "MIN" ) {
      s = "float min_intensity = 2.0; \n";
    } else if( t == "AVERAGE" ) {
      s = "float intensity_sum = 0.0; \n";
      s += "int nr_samples = 0; \n";
    } else {
      Console(4) << "Invalid value for type field in ProjectionVolumeStyle: "
                 << "\"" << t << "\". Must be \"MAX\", \"MIN\" or \"AVERAGE\""
                 << endl;
    }
    return s;
  }

  string ProjectionVolumeStyle::getShaderCode() {
    stringstream s;
    s << "if( " << UNIFORM_ID( enabled ) << ") { \n"
      << "  float intensity = (sample_color.r + sample_color.g + sample_color.b)/3.0; \n";
     
    const string &t = type->getValue();
    if( t == "MAX" ) {
      H3DFloat threshold = intensityThreshold->getValue();
      if( threshold > 0 ) {
        s <<
          "  if( intensity > " << threshold << "&& intensity > max_intensity ) { \n"
          "    max_intensity = intensity; \n" 
          "    rr.color.rgb = vec3( intensity, intensity, intensity ); \n"
          "    rr.color.a = sample_color.a; \n"
          "    rr.zpoint = vec4(r0.xyz, 1); \n" 
          "  } \n"
          "  if( max_intensity > 0.0 && intensity < max_intensity ) break; \n"
          "}\n";
      } else {
        s <<
        "  if( intensity > max_intensity ) { \n"
        "    max_intensity = intensity; \n" 
        "    rr.color.rgb = vec3( intensity, intensity, intensity ); \n"
        "    rr.color.a = sample_color.a; \n"
        "    rr.zpoint = vec4(r0.xyz, 1); \n"
        "  } \n"
        "} \n";
      }
    } else if( t == "MIN" ) {
      s <<
        "  if( intensity < min_intensity ) { \n"
        "    min_intensity = intensity; \n" 
        "    rr.zpoint = vec4(r0.xyz, 1); \n"
        "  } \n"
        "} \n";
    } else if( t == "AVERAGE" ) {
      s <<
        "  intensity_sum = intensity_sum + intensity; \n"
        "  nr_samples = nr_samples + 1; \n"
        "} \n"; 
    } else {
      s <<  "} \n"; 
  }
    return s.str();
  }

  string ProjectionVolumeStyle::getShaderPostCode() {
    const string &t = type->getValue();
    if( t == "AVERAGE" ) {
      return
        "if( nr_samples > 0 ) { \n"
        "  float i = intensity_sum / nr_samples;\n"
        "  rr.color.rgb = vec3( i, i, i ); \n"
        "  rr.color.a = i; \n"
        "} \n";

    } else if( t == "MIN" ) {
      return 
        "float i; \n"
        "if( min_intensity > 1 ) i = 0; \n"
        "else i = min_intensity; \n"
        "rr.color.rgb = vec3( i, i, i ); \n"
        "rr.color.a = i;\n";
    } 
    return  "";
  }
}




