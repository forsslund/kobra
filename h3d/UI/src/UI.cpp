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
//
/// \file UI.cpp
/// \brief cpp file for UI.
///
//////////////////////////////////////////////////////////////////////////////
#include <H3D/UI/UI.h>
#include <sstream>
#include <cstring>

using namespace std;

H3D::LibraryInfo getLibraryInfo() {
  H3D::LibraryInfo r = H3D::LibraryInfo::createEmptyInfo();
  std::stringstream s;
  s << UI_MAJOR_VERSION << "."
    << UI_MINOR_VERSION << " (build "
    << UI_BUILD_VERSION << ")";

  strcpy(r.name, "UI" );
  strcpy( r.version, s.str().c_str() );
  strcpy( r.web, "http://www.h3dapi.org/modules/mediawiki/index.php/UI" );
  strcpy( r.developer, "SenseGraphics AB" );
  strcpy( r.developer_web, "http://www.sensegraphics.com" );
  strcpy( r.info, "The UI toolkit adds nodes for simple haptic 3D widgets such as buttons, menus, sliders etc." );
  return r;
}
