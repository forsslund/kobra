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
/// \file H3DVertexBodyConstraintNode.h
/// \brief Header file for H3DVertexBodyConstraintNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DVERTEXBODYCONSTRAINTNODE__
#define __H3DVERTEXBODYCONSTRAINTNODE__

#include <H3D/MFInt32.h>
#include <H3D/H3DPhysics/H3DBodyConstraintNode.h>

namespace H3D{  

  /// Abstract base class for vertex type constraints.
  ///
  /// \par Internal routes:
  /// \dotfile H3DVertexBodyConstraintNode.dot
  class H3DPHYS_API H3DVertexBodyConstraintNode : public H3DBodyConstraintNode {
  public:

    /// Constructor.
    H3DVertexBodyConstraintNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< MFInt32 > _index = 0 );

    /// The list of vertices the constraints will be applied to.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DVertexBodyConstraintNode_index.dot
    auto_ptr < MFInt32 > index;

    /// Initialize the constraint for the given PhysicsEngineThread. I.e. 
    /// create a new constraint in the physics engine with the parameters
    /// of the constraint fields. Returns 0 on success. Overrides the
    /// initializeConstraint function of the H3DBodyConstraint in order
    /// to check whether the engineThread is softBodyPhysicsEngineThread
    /// or not.
    virtual bool initializeConstraint( H3D::PhysicsEngineThread& pt );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Virtual function that returns a new instance of a subclass of
    /// PhysicsEngineParameters::ConstraintParameters that describes the 
    /// constraint. This should be overridden in each subclass of H3DBodyConstraintNode
    /// so that it returns the parameters for the constraint that is implemented.
    /// If all_params is true then it returns all field values regardless of whether
    /// the values have changed
    virtual PhysicsEngineParameters::ConstraintParameters *getConstraintParameters( bool all_params = false );


  };
}
#endif
