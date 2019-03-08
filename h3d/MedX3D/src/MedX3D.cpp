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
//
/// \file MedX3D.cpp
/// \brief cpp file for MedX3D
///
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/MedX3D.h>
#include <sstream>
#include <cstring>

using namespace std;

H3D::LibraryInfo getLibraryInfo() {
  H3D::LibraryInfo r = H3D::LibraryInfo::createEmptyInfo();
  std::stringstream s;
  s << MEDX3D_MAJOR_VERSION << "."
    << MEDX3D_MINOR_VERSION << " (build "
    << MEDX3D_BUILD_VERSION << ")";

  strcpy(r.name, "MedX3D" );
  strcpy( r.version, s.str().c_str() );
  strcpy( r.web, "http://www.h3dapi.org/modules/mediawiki/index.php/MedX3D" );
  strcpy( r.developer, "SenseGraphics AB" );
  strcpy( r.developer_web, "http://www.sensegraphics.com" );
  strcpy( r.info, "MedX3D adds the nodes from the upcoming VolumeRendering component of the X3D standard. It adds nodes for volume rendering implemented both by GPU based ray casting and slice-based 3D texture rendering." );
  return r;

}
