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
/// \file H3DPhysics/PythonMethods.h
/// \brief Header file containing functions to add to H3DAPIs python interface.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DPHYS_PYTHONMETHODS__
#define __H3DPHYS_PYTHONMETHODS__

/// \page H3DPhysicsPythonSyntax H3DPhysicsInterface Python module
/// \section H3DPhysicsInterface
/// H3DPhysics adds an additional functionality to the H3DInterface
/// module. So if the H3DPhysics dll is loaded the functionality described
/// <a href="../python/doc/html/index.html">here</a> can be used.


#include <H3D/H3DPhysics/H3DPhysics.h>

namespace H3D {  

  /// A singleton class used to setup and manage the H3DPhysics
  /// Python interface, which extends the standard H3DAPI Python
  /// interface.
  class H3DPHYS_API H3DPhysicsPythonInterface {
  public:
    /// Returns the one and only instance
    ///
    /// This must be called to initialize the Python interface
    /// before any functions can be called from Python.
    ///
    static H3DPhysicsPythonInterface& getInstance();

  protected:
    /// Constructor
    H3DPhysicsPythonInterface ();

    /// Initialize the Python interface
    void initModule ();
  };
  
}
#endif
