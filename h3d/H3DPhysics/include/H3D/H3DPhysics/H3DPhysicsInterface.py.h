////////////////////////////////////////////////////////////////////////////////
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
/// \file H3DPhysicsInterface.py.h
/// \brief header file containing H3DPhysicsInterface python module strings.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <string>

namespace H3D {

  namespace H3DPhysicsInterface {
    const std::string H3DPhysicsInterface_string = "\
import H3DPhysics\n\
\n\
class TrackedMFieldBase:\n\
  \n\
  def insertTracked ( self, index, value ):\n\
    H3DPhysics.TrackedMField_insertTracked ( self, index, value )\n\
  \n\
  def updateTracked ( self, index, value ):\n\
    H3DPhysics.TrackedMField_updateTracked ( self, index, value )\n\
  \n\
  def eraseTracked ( self, index, count= 1 ):\n\
    H3DPhysics.TrackedMField_eraseTracked ( self, index, count )\n\
  \n\
  def pushBackTracked ( self, value ):\n\
    H3DPhysics.TrackedMField_insertTracked ( self, self.size(), value )\n\
  \n\
\n\
def TrackedMField ( field_type ):\n\
  \n\
  class FieldClass ( field_type, TrackedMFieldBase ):\n\
    pass\n\
  \n\
  return FieldClass\n\
\n\
trackedTemplate= \"\"\"\n\
class Tracked%s ( TrackedMField ( %s ) ):\n\
  type = UNKNOWN_X3D_TYPE\n\
\"\"\"\n\
\n\
for (t,n) in mfield_types:\n\
  exec ( trackedTemplate % (n,n) )\n\
";
  }
}
