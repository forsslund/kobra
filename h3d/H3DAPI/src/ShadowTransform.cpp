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
/// \file ShadowTransform.cpp
/// \brief CPP file for ShadowTransform
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ShadowTransform.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase ShadowTransform::database( "ShadowTransform", 
             &(newInstance<ShadowTransform>), 
             typeid( ShadowTransform ),
             &H3DShadowObjectNode::database );

namespace ShadowTransformInternals {
  FIELDDB_ELEMENT( ShadowTransform, shadowVolume, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowTransform, transform, INPUT_OUTPUT )
}

ShadowTransform::ShadowTransform( Inst< SFNode>  _metadata,
                                  Inst< SFTransformNode > _transform,
                                  Inst< MFShadowObjectNode > _shadowVolume,
                                  Inst< SFBool > _enabled ) :
  H3DShadowObjectNode( _metadata, _transform, _enabled ),
  shadowVolume( _shadowVolume ) {

  type_name = "ShadowTransform";
  database.initFields( this );

}

void H3D::ShadowTransform::update() {
  H3DShadowObjectNode::update();

  is_enabled_ts = enabled->getValue();
  if( transform->getValue() ) {
    transform_matrix_ts = transform->getValue()->matrix->getValue();
  } else {
    transform_matrix_ts = Matrix4f();
  }

  shadow_volumes_ts.clear();
  shadow_volumes_ts.reserve( shadowVolume->size() );
  for( MFShadowObjectNode::const_iterator i = shadowVolume->begin(); i != shadowVolume->end(); ++i ) {
    H3DShadowObjectNode*  shadow_node = static_cast<H3DShadowObjectNode*>(*i);
    if( shadow_node ) {
      shadow_volumes_ts.push_back( shadow_node );
      // update 
      shadow_node->update();
    }
  }
}

void H3D::ShadowTransform::renderShadowGPU( const LightDataStruct& light_data, Matrix4f accumulated_fwd, bool render_caps ) {

  bool is_enabled = is_enabled_ts;
  Matrix4f  transform_matrix = transform_matrix_ts;

  if( !is_enabled ) {
    return;
  }

  Matrix4f m = accumulated_fwd * transform_matrix;

  for( std::vector<H3DShadowObjectNode*>::const_iterator i = shadow_volumes_ts.begin(); i != shadow_volumes_ts.end(); ++i ) {
    if( *i ) {
      (*i)->renderShadowGPU( light_data, m, render_caps);
    }
  }
}

void H3D::ShadowTransform::computeShadowVolumeInformationCPU( const LightDataStruct& light_data, Matrix4f accumulated_fwd, bool render_caps, std::vector< Vec4d >& coord ) {

  bool is_enabled = is_enabled_ts;
  Matrix4f  transform_matrix = transform_matrix_ts;

  if( !is_enabled ) {
    return;
  }

  Matrix4f m = accumulated_fwd * transform_matrix;

  for( std::vector<H3DShadowObjectNode*>::const_iterator i = shadow_volumes_ts.begin(); i != shadow_volumes_ts.end(); ++i ) {
    if( *i ) {
      (*i)->computeShadowVolumeInformationCPU( light_data, m, render_caps, coord );
    }
  }
}
