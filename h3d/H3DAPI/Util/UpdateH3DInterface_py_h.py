########################################################################
#
# Script to update the H3DAPI/src/H3DInterface.py.h automatically
# from based on the H3DAPI/lib/H3DInterface.py and H3DAPI/lib/H3DUtils.py files.
#
# Usage: just run file with python interpreter from the H3DAPI/Util folder.


########################################################################
#
# Imports
#
import string, os
import re

########################################################################
#
# String templates
#

header_tmpl="""
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
/// \\file H3DInterface.py.h
/// \\brief header file containing H3DInterface python module strings.
///
//
//////////////////////////////////////////////////////////////////////////////

namespace H3D {

  namespace H3DInterface {
    const string H3DInterface_string = "\\
%s";
    const string H3DUtils_string = "\\
%s";
  }
}"""

def readAndPrepareContent( file_name ):
  f = open( file_name, "r" )
  s = f.read()
  f.close()
  s = s.replace( "\\", "\\\\")
  s = s.replace( "\"", "\\\"")
  return s.replace( "\n", "\\n\\\n")


this_file_dir= os.path.dirname(os.path.abspath(__file__))

h3d_interface_file_name = os.path.join( this_file_dir, "..", "lib", "H3DInterface.py")
h3d_utils_file_name = os.path.join( this_file_dir, "..", "lib", "H3DUtils.py")
h3d_interface_py_h_file_name = os.path.join( this_file_dir, "..", "src", "H3DInterface.py.h")

h3d_interface_string = readAndPrepareContent( h3d_interface_file_name )
h3d_utils_file = readAndPrepareContent( h3d_utils_file_name )

f = open( h3d_interface_py_h_file_name, "w" )
output = header_tmpl % (h3d_interface_string, h3d_utils_file ) 
#print( output )
f.write( output )
f.close()



