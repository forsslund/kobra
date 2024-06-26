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
/// \file NonUniformStiffness.h
/// \brief Header file for NonUniformStiffness, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3D_PHYSICS_NONUNIFORMSTIFFNESS__
#define __H3D_PHYSICS_NONUNIFORMSTIFFNESS__

#include <H3D/H3DPhysics/H3DPhysicsStiffnessNode.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/MFFloat.h>

namespace H3D{  

  /// Node representing inhomogenous damping for the whole soft body.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/Stiffness.x3d">Stiffness.x3d</a>
  ///     ( <a href="examples/Stiffness.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile NonUniformStiffness.dot
  class H3DPHYS_API NonUniformStiffness : public H3DPhysicsStiffnessNode {
  public:
    typedef TrackedMField < MFFloat > TrackedMFFloat;

    /// Constructor.
    NonUniformStiffness(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0,
      Inst< SFString > _unitType = 0,
      Inst< SFStiffness > _stiffness = 0,
      Inst< TrackedMFFloat > _stiffnessPerUnit = 0);

    /// The list of stiffness values for each unit of the body such as
    /// node, edge, element. The unit type is determined by the unitType
    /// field.
    ///
    /// Use the updateTracked() function to modify a subset of the field values.
    /// If the engine supports it it may be more efficient than setValue().
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile NonUniformStiffness_stiffnessPerUnit.dot
    auto_ptr < TrackedMFFloat > stiffnessPerUnit;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Creates a new instance of a subclass of MaterialPropertyParameters appropriate for the subclass.
    virtual PhysicsEngineParameters::MaterialPropertyParameters* createMaterialPropertyParameters ();

    /// Returns a MaterialPropertyParameters to describe the material property. By default
    /// the function returns a MaterialPropertyParameters with values
    /// that have changed since the last loop.
    //// \param all_params If true then it returns all field values regardless
    /// of whether the values have changed
    virtual PhysicsEngineParameters::MaterialPropertyParameters* getMaterialPropertyParameters( bool all_params = false );

  };
}
#endif
