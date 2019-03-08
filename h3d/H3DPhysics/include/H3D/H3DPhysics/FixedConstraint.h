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
/// \file FixedConstraint.h
/// \brief Header file for FixedConstraint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __FIXEDCONSTRAINT__
#define __FIXEDCONSTRAINT__

#include <H3D/H3DPhysics/H3DVertexBodyConstraintNode.h>
#include <H3D/MFInt32.h>

namespace H3D{  

  /// The constraint used to fix an attribute  for a list of vertices.
  /// This node can be used to fix the vertices to its current values.
  /// In case specific values want to be assigned, the subclasses can
  /// handle that.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/Rope.x3d">Rope.x3d</a>
  ///     ( <a href="examples/Rope.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile FixedConstraint.dot
  class H3DPHYS_API FixedConstraint : public H3DVertexBodyConstraintNode {
  public:

    /// Constructor.
    FixedConstraint( Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< MFInt32 > _index = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns a new instance of 
    /// PhysicsEngineParameters::FixedConstraintParameters with the values updated with
    /// corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();


  };
}
#endif
