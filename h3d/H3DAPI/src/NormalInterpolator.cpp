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
/// \file NormalInterpolator.cpp
/// \brief CPP file for NormalInterpolator, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/NormalInterpolator.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase NormalInterpolator::database( 
                                        "NormalInterpolator", 
                                        &(newInstance<NormalInterpolator>), 
                                        typeid( NormalInterpolator ),
                                        &X3DInterpolatorNode::database );

namespace NormalInterpolatorInternals {
  FIELDDB_ELEMENT( NormalInterpolator, keyValue, INPUT_OUTPUT )
  FIELDDB_ELEMENT( NormalInterpolator, value_changed, OUTPUT_ONLY )
}

NormalInterpolator::NormalInterpolator( Inst< SFNode  > _metadata,
                                        Inst< SFFloat > _set_fraction,
                                        Inst< MFFloat > _key,
                                        Inst< MFVec3f > _keyValue,
                                        Inst< MFValue > _value_changed ) :
  X3DInterpolatorNode( _metadata, _set_fraction, _key ),
  keyValue     ( _keyValue      ),
  value_changed( _value_changed ) {

  type_name = "NormalInterpolator";
  database.initFields( this );

  set_fraction->route( value_changed, id );
  key->route( value_changed, id );
  keyValue->route( value_changed, id );
}

void NormalInterpolator::MFValue::update() {
  NormalInterpolator *interpolator = 
    static_cast<NormalInterpolator*>( getOwner() );
  H3DFloat fraction = static_cast<SFFloat*>(routes_in[0])->getValue(interpolator->id);
  int key_size = static_cast<MFFloat*>(routes_in[1])->size();
  H3DFloat weight;
  int key_index = static_cast<NormalInterpolator*>(owner)->lookupKey( fraction, weight );
  vector< Vec3f > key_values = static_cast<MFVec3f*>(routes_in[2])->getValue();
  int value_size = 0;
  if( key_size != 0 )
    value_size = (int)key_values.size() / key_size;
  value.resize( value_size );
  
  if ( key_index >= 0 && 
       (key_index + 2)* value_size - 1 < (int)key_values.size() ) {
    if (weight<=0) 
      for (int x = 0; x < value_size; ++x )
       value[x] = key_values[ key_index*value_size + x ];
    else if (weight>=1)
      for (int x = 0; x < value_size; ++x )
        value[x] = key_values[ (key_index+1)*value_size + x];
    else { 
      for (int x = 0; x < value_size; ++x ) {
        Vec3f n1 = key_values[ key_index*value_size + x ];
        Vec3f n2 = key_values[ (key_index+1)*value_size + x  ];
        H3DFloat cos_alpha = n1 * n2;
        if( H3DAbs( 1 - cos_alpha ) < Constants::f_epsilon ) {
          value[ x ] = n1;  
        } else {
          H3DFloat alpha = H3DAcos(cos_alpha);
          H3DFloat sin_alpha = H3DSin( alpha );
          value[ x ] = 
            ( H3DSin((1-weight) * alpha )/ sin_alpha ) * n1 + 
            ( H3DSin( weight * alpha )/ sin_alpha ) * n2;
        }
      }
    }
  }
}
