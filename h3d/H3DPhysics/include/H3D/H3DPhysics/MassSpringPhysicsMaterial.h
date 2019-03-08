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
/// \file MassSpringPhysicsMaterial.h
/// \brief Header file for MassSpringPhysicsMaterial, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __MASSSPRINGPHYSICSMATERIAL__
#define __MASSSPRINGPHYSICSMATERIAL__

#include <H3D/H3DPhysics/H3DPhysicsMaterialNode.h>
#include <H3D/H3DPhysics/H3DPhysicsStiffnessNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>

namespace H3D{  

  /// MassSpringPhysicsMaterial node which includes physicsMaterial properties
  /// used in mass-spring deformation algorithm.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/Cloth.x3d">Cloth.x3d</a>
  ///     ( <a href="examples/Cloth.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile MassSpringPhysicsMaterial.dot
  class H3DPHYS_API MassSpringPhysicsMaterial : public H3DPhysicsMaterialNode {
  public:

    /// MassSpringPhysicsMaterial is dependent on the
    /// materialPropertyChanged field of the contained H3DPhysicsStiffnessNode.
    typedef DependentSFNode< H3DPhysicsStiffnessNode,
      FieldRef< H3DPhysicsMaterialPropertyNode,
      Field, &H3DPhysicsStiffnessNode::materialPropertyChanged> >
      SFH3DPhysicsStiffnessNode;

    /// Constructor.
    MassSpringPhysicsMaterial( Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DPhysicsMassNode > _mass = 0,
      Inst< SFH3DPhysicsDampingNode > _damping = 0,
      Inst< SFH3DPhysicsFrictionNode > _friction = 0,
      Inst< SFH3DPhysicsStiffnessNode > _stiffness = 0,
      Inst< SFH3DPhysicsStiffnessNode > _stiffnessAngular = 0,
      Inst< SFH3DPhysicsStiffnessNode > _stiffnessVolume = 0 );


    /// The linear stiffness mapping for the soft body.
    /// The mapping could be a uniform value throughout the whole soft body
    /// or a non-homogeneous distribution.
    ///
    /// If stiffnessAngular or stiffnessVolume is not specified then the
    /// value of stiffness will be used.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile MassSpringPhysicsMaterial_stiffness.dot
    auto_ptr < SFH3DPhysicsStiffnessNode > stiffness;

    /// The angular stiffness mapping for the soft body.
    /// The mapping could be a uniform value throughout the whole soft body
    /// or a non-homogeneous distribution.
    ///
    /// If not specified, then the value of stiffness is used.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile MassSpringPhysicsMaterial_stiffnessAngular.dot
    auto_ptr < SFH3DPhysicsStiffnessNode > stiffnessAngular;

    /// The volume stiffness mapping for the soft body.
    /// The mapping could be a uniform value throughout the whole soft body
    /// or a non-homogeneous distribution.
    ///
    /// If not specified, then the value of stiffness is used.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile MassSpringPhysicsMaterial_stiffnessVolume.dot
    auto_ptr < SFH3DPhysicsStiffnessNode > stiffnessVolume;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Creates a new instance of a subclass of H3DPhysicsMaterialParameters appropriate for the subclass of collidable
    virtual PhysicsEngineParameters::H3DPhysicsMaterialParameters* createH3DPhysicsMaterialParameters ();

    /// Returns a H3DPhysicsMaterialParameters to describe the collidable. By default
    /// the function returns a H3DPhysicsMaterialParameters with values
    /// that have changed since the last loop.
    //// \param all_params If true then it returns all field values regardless
    /// of whether the values have changed
    virtual PhysicsEngineParameters::H3DPhysicsMaterialParameters* getH3DPhysicsMaterialParameters( bool all_params = false );

    /// Variables used to create all parameters of the node incase the whole
    /// node is changed.
    H3DPhysicsStiffnessNode *previousStiffness;
    H3DPhysicsStiffnessNode *previousStiffnessAngular;
    H3DPhysicsStiffnessNode *previousStiffnessVolume;
  };
}
#endif
