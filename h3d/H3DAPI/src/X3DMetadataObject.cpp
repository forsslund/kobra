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
/// \file X3DMetadataObject.cpp
/// \brief CPP file for X3DMetadataObject, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/X3DMetadataObject.h>

using namespace H3D;

X3DMetadataObject::X3DMetadataObject( Inst< SFString > _name,
                                      Inst< SFString>  _reference ) :
  nameF( _name ),
  reference( _reference ) {
}

 X3DMetadataObject *X3DMetadataObject::getMetadataByName( const string &_name ) {
   if( _name == nameF->getValue() ) return this; 
   return NULL;
 }
