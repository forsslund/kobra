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
/// \file X3DVolumeRenderStyleNode.cpp
/// \brief CPP file for X3DVolumeRenderStyleNode, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/X3DVolumeRenderStyleNode.h>

namespace H3D {
  
  H3DNodeDatabase 
  X3DVolumeRenderStyleNode::database( "X3DVolumeRenderStyleNode", 
                                      NULL,
                                      typeid( X3DVolumeRenderStyleNode ),
                                      &X3DNode::database );
  
  namespace X3DVolumeRenderStyleNodeInternals {
    FIELDDB_ELEMENT( X3DVolumeRenderStyleNode, enabled, INPUT_OUTPUT )
  }

  X3DVolumeRenderStyleNode::
  X3DVolumeRenderStyleNode( Inst< DisplayList > _displayList, 
                            Inst< SFBool > _enabled,
                            Inst< SFNode > _metadata  ) :
    
    X3DNode( _metadata ),
    H3DDisplayListObject( _displayList ),
    enabled( _enabled ),
    rebuildShader( new Field ) {
    type_name = "X3DVolumeRenderStyleNode";
    database.initFields(this);
    
    // style_id is used to have unique variable names in the shaders
    // it is simply the memory address in hex
    uintptr_t sid = uintptr_t(this);
    ostringstream ostr;
    ostr << hex << sid;
    style_id = ostr.str();
    
    // ownership
    displayList->setOwner( this );
    
    // default
    enabled->setValue( true );
    
    // routings
    enabled->route( displayList );
  }
  
  
  string X3DVolumeRenderStyleNode::addUniforms( ComposedShader *s ) {
    return "";
  }

  void X3DVolumeRenderStyleNode::updateUniformFields( X3DVolumeNode *vd ) {
    if( vd ) {
      // fields from VolumeData

      
//       // check if we should use the VolumeData's normals
//       if( require_normals ) {  
//   SFTexture3DNode *normals = surfaceNormals->getValue();
//   if( normals ) {
//     surfaceNormals_glsl->setValue( normals );
//   }
//   else {
//     normals = vd->surfaceNormals->getValue();
//     surfaceNormals_glsl->setValue( normals );
//   }
//       }
      
    }
  }
  
  
  string X3DVolumeRenderStyleNode::
  addUniformToFragmentShader( ComposedShader *s,
                              const string &_name,
                              const string &glsl_type,
                              const Field::AccessType &access,
                              Field *field,
                              bool delete_unadded_field ) {
    
    // if field successfully added, we add corresponding code 
    // to the fragment shader
    bool ok = s->addField( _name, access, field ); 
    if( ok ) {
      return "uniform " + glsl_type + " " + _name + ";\n";
    } else {
      if( delete_unadded_field )
        delete field;
      return "";
    }
  }
  
}
