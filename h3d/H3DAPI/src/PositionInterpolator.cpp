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
/// \file PositionInterpolator.cpp
/// \brief CPP file for PositionInterpolator, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/PositionInterpolator.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase PositionInterpolator::database(
                                        "PositionInterpolator",
                                        &(newInstance<PositionInterpolator>),
                                        typeid( PositionInterpolator ),
                                        &X3DInterpolatorNode::database );

namespace PositionInterpolatorInternals {
  FIELDDB_ELEMENT( PositionInterpolator, keyValue, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PositionInterpolator, value_changed, OUTPUT_ONLY )
}


PositionInterpolator::PositionInterpolator( Inst< SFNode  > _metadata,
                                            Inst< SFFloat > _set_fraction,
                                            Inst< MFFloat > _key,
                                            Inst< MFVec3f > _keyValue,
                                            Inst< SFValue > _value_changed ) :
  X3DInterpolatorNode( _metadata, _set_fraction, _key ),
  keyValue     ( _keyValue      ),
  value_changed( _value_changed ) {

  type_name = "PositionInterpolator";
  database.initFields( this );

  set_fraction->route( value_changed, id );
  key->route( value_changed, id );
  keyValue->route( value_changed, id );
}


void PositionInterpolator::SFValue::update() {
  PositionInterpolator *interpolator =
    static_cast<PositionInterpolator*>( getOwner() );
  H3DFloat fraction =
    static_cast<SFFloat*>(routes_in[0])->getValue( interpolator->id );
  const vector< Vec3f > &key_values =
    static_cast<MFVec3f*>(routes_in[2])->getValue();
  H3DFloat weight;
  int key_index = interpolator->lookupKey( fraction,
                                           weight );
  if( key_index >= 0 && key_index + 1 < (int)key_values.size() ) {
    if (weight<=0)
      value = key_values[key_index];
    else if (weight>=1)
      value = key_values[key_index+1];
    else {
      Vec3f a = key_values[ key_index ];
      Vec3f b = key_values[ key_index+1 ];
      value = (1-weight)*a + (weight)*b;
    }
  }
}

