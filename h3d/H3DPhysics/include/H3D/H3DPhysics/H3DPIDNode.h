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
/// \file H3DPIDNode.h
/// \brief Header file for H3DPIDNode
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DPIDNODE__
#define __H3DPIDNODE__

#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/X3DNode.h>

namespace H3D {

  /// \class H3DPIDNode
  /// Base class for nodes that should be smoothly driven to a certain state
  /// using a PID control loop.
  class H3DPHYS_API H3DPIDNode : public X3DNode {
  public:

    // Constructor.
    H3DPIDNode( Inst< SFNode  >  _metadata = 0 );

    /// Specifies the field of the parent node to which this node belongs
    virtual string defaultXMLContainerField() { return "pids"; }

    /// Traverse the scene graph.
    virtual void traverseSG( TraverseInfo &ti );

    /// Calculates the new PID output to update the actuation: Force/Torque
    virtual void updateActuation() = 0;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Initializes for the given PhysicsEngineThread.
    virtual void initialize( PhysicsEngineThread& pt );

    PhysicsEngineThread* engine_thread;

  };
}

#endif
