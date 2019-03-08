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
/// \file H3DSolverNode.h
/// \brief Header file for H3DSolverNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DSOLVERNODE__
#define __H3DSOLVERNODE__

#include <H3D/SFString.h>
#include <H3D/H3DPhysics/H3DPhysics.h>

namespace H3D{  

  /// \ingroup AbstractNodes H3DSolverNode
  /// Abstract base node for mathematical solvers.
  ///
  /// \par Internal routes:
  /// \dotfile H3DSolverNode.dot
  class H3DPHYS_API H3DSolverNode : public Node {
  public:

    /// Constructor.
    H3DSolverNode( Inst< SFString > _solverType = 0 );

    /// The type of the solver to be used to evaluate deformation.
    /// The desired solver is requested from the physics_engine,
    /// however, in case it is not supported the default solver of
    /// the engine is used instead. The user can choose among
    /// the solver types registered in the interface of the physics
    /// engine, check the corresponding documentation for the use
    /// of possible solvers. 
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "DEFAULT_SOLVER" \n
    /// 
    /// \dotfile H3DSolverNode_solverType.dot
    auto_ptr< SFString > solverType;

    /// Field that gets an event when any of the X3D fields in the
    /// H3DSolverNode generates an event
    auto_ptr< Field > solverChanged;

    /// Returns the default xml containerField attribute value.
    /// For this node it is "h3dSolver".
    virtual string defaultXMLContainerField() {
      return "h3dSolver";
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  };
}
#endif
