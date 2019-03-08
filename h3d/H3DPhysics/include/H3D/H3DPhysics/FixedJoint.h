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
/// \file FixedJoint.h
/// \brief Header file for FixedJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FIXEDJOINT__
#define __FIXEDJOINT__

#include <H3D/X3DNode.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFFloat.h>
#include <H3D/MFString.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>

// H3DUtil includes
#include <H3DUtil/AutoPtrVector.h>

namespace H3D{

  /// \ingroup X3DNodes RigidBody
  /// \class FixedJoint
  /// Not part of the X3D specification. A joint which rigidly connects two
  /// bodies. If one of the bodies move, then the other moves such that
  /// their relative position and orientation stays the same.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/FixedJoint.x3d">FixedJoint.x3d</a>
  ///     ( <a href="examples/FixedJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile FixedJoint.dot
  class H3DPHYS_API FixedJoint :  public H3DRigidBodyJointNode {
  public:

    /// Constructor.
    FixedJoint( Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of 
    /// PhysicsEngineParameters::FixedJointParameters with the values updated
    /// with corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless
    /// of whether the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters *getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate
    /// for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

  };
}
#endif
