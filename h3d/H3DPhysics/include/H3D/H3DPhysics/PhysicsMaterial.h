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
/// \file PhysicsMaterial.h
/// \brief Header file for PhysicsMaterial, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSICSMATERIAL__
#define __PHYSICSMATERIAL__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DPhysicsMaterialNode.h>
#include <H3D/H3DPhysics/H3DPhysicsElasticityNode.h>

namespace H3D{  

  /// PhysicsMaterial node which includes commonly used physicsMaterial properties
  /// such as mass, damping, elasticity and friction.
  ///
  /// \note This class is not supported by any physics engines yet.
  /// \par Internal routes:
  /// \dotfile PhysicsMaterial.dot
  class H3DPHYS_API PhysicsMaterial : public H3DPhysicsMaterialNode {
  public:

    /// PhysicsMaterial is dependent on the
    /// materialPropertyChanged field of the contained H3DPhysicsElasticityNode.
    typedef DependentSFNode< H3DPhysicsElasticityNode,
      FieldRef< H3DPhysicsMaterialPropertyNode,
      Field, &H3DPhysicsElasticityNode::materialPropertyChanged> >
      SFH3DPhysicsElasticityNode;

    /// Constructor.
    PhysicsMaterial( Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DPhysicsMassNode > _mass = 0,
      Inst< SFH3DPhysicsDampingNode > _damping = 0,
      Inst< SFH3DPhysicsFrictionNode > _friction = 0,
      Inst< SFH3DPhysicsElasticityNode > _elasticity = 0 );

    /// The SFH3DPhysicsElasticityNode including elasticity mapping for the soft body.
    /// The mapping could be a uniform value thoroughout the whole soft body
    /// as well as a non-homogenous distribution.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile PhysicsMaterial_elasticity.dot
    auto_ptr < SFH3DPhysicsElasticityNode > elasticity;

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
    H3DPhysicsElasticityNode *previousElasticity;

  };
}
#endif
