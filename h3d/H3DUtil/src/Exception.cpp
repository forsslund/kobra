//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3DUtil.
//
//    H3DUtil is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3DUtil is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3DUtil; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file Exception.cpp
/// \brief Base class for all H3D Exceptions
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3DUtil/Exception.h>

void H3DUtil::Exception::H3DAPIException::print( std::ostream &os ) const {
  os << className();
  if( message.length() ) {
    os << ": " << message;
  }
  if( function.length() || filename.length() || linenumber != -1 ) {
    os << " (";
    if( function.length() ) os << "" << function << "()";
    if( filename.length() ) os << " in " << filename;
    if( linenumber != -1 ) os << " line " << linenumber;
    os << ")";
  }
}
