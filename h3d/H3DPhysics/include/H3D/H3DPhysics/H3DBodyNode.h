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
/// \file H3DBodyNode.h
/// \brief Header file for H3DBodyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DBODYNODE__
#define __H3DBODYNODE__

#include <H3D/H3DPhysics/PhysicsEngineParameters.h>
#include <H3D/X3DNode.h>

namespace H3D{  

  /// \ingroup AbstractNodes
  /// Abstract base node for bodies in the physics simulation
  ///
  /// This is an addition to the X3D RigidBodyPhysics specification to allow 
  /// common treatment of both rigid and soft bodies (e.g. as parameters to
  /// joints that support both rigid and soft bodies)
  ///
  /// \par Internal routes:
  /// \dotfile H3DBodyNode.dot
  class H3DPHYS_API H3DBodyNode : public X3DNode {
  public:

    /// Constructor.
    H3DBodyNode(
      Inst< SFNode > _metadata = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// Initialize the rigid body for the given PhysicsEngineThread. I.e. 
    /// create a new rigid body in the physics engine with the parameters
    /// of the rigid body fields. Returns 0 on success.
    virtual bool initializeBody( PhysicsEngineThread& pt );

    /// Returns true if this rigid body node has been initialized for PhysicsEngineThread.
    bool isInitialized();

    /// Deletes this rigid body node from the given PhysicsEngineThread.
    virtual bool deleteBody();

    /// Get the H3DBodyId used by this node.
    /// This is only valid if initializeRigidBody has been called before.
    PhysicsEngineParameters::H3DBodyId getBodyId();

    /// Given a H3DBodyId returns the RigidBody node that is associated
    /// with it.
    static H3DBodyNode* getBodyFromId( PhysicsEngineParameters::H3DBodyId );

  protected:
    typedef std::map< PhysicsEngineParameters::H3DBodyId, H3DBodyNode * > BodyMap;

    /// The PhysicsEngineThread that this rigid body uses.
    H3D::PhysicsEngineThread* engine_thread;

    /// The H3DBodyId used by this node in the PhysicsEngineThread of the
    /// RigidBodyCollection it resides in.
    PhysicsEngineParameters::H3DBodyId body_id;

    /// A map from rigid body id to the RigidBody node that is associated
    /// with that id.
    static BodyMap body_id_map;
  };
}
#endif
