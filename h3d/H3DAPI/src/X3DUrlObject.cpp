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
/// \file X3DUrlObject.cpp
/// \brief CPP file for X3DUrlObject, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/X3DUrlObject.h>
#include <H3D/ResourceResolver.h>

using namespace H3D;


X3DUrlObject::X3DUrlObject( 
                           Inst< MFString>  _url ):
  url( _url ),
  url_used( "" ), 
  url_base( ResourceResolver::getBaseURL() ) {
  
}


string X3DUrlObject::resolveURLAsFile( const string &_url,
                                       bool *is_tmp_file ) {
  return resolveURL ( _url, false, is_tmp_file );
}

string X3DUrlObject::resolveURLAsString( const string &_url ) {
  return resolveURL ( _url, true );
}

string X3DUrlObject::resolveURL ( const string& _url, bool return_contents, bool *is_tmp_file ) {
  int inlined_content_offset = getInlinedContentOffset( _url );
  if( inlined_content_offset > 0 ) {
    string contents= _url.substr( inlined_content_offset, _url.size()-1 );
    if ( return_contents ) {
      return contents;
    } else {
      string tmp_file = ResourceResolver::getTmpFileName();
      if( tmp_file != "" ) {
        ofstream os( tmp_file.c_str() );
        os << contents;
        os.close();
        if( is_tmp_file ) *is_tmp_file = true;
        return tmp_file;
      }
    }
  }

  string result;
  if ( return_contents ) {
    result= ResourceResolver::resolveURLAsString ( _url, url_base );
  } else {
    result= ResourceResolver::resolveURLAsFile ( _url, is_tmp_file, url_base );
  }
  return result;
}

void X3DUrlObject::addInlinePrefix( const string &s ) {
  supported_inline_prefixes.push_back( s + ":" );
}

int X3DUrlObject::getInlinedContentOffset( const std::string& url ) {
  for( list< string >::const_iterator i = supported_inline_prefixes.begin();
       i != supported_inline_prefixes.end(); ++i ) {
    size_t start = 0;
    size_t url_size = url.size();
    while( start < url_size && isspace( url[start] ) ) ++start;
    if( url.compare( start, (*i).size(), *i ) == 0 ) {
      assert( start + (*i).size() <= ( std::numeric_limits<unsigned int>::max() / 2 ) - 1 );
      return static_cast< int >(start + (*i).size());
    }
  }

  return 0;
}

bool X3DUrlObject::removeTmpFile( const string &file ) {
  return ResourceResolver::releaseTmpFileName( file );
}
