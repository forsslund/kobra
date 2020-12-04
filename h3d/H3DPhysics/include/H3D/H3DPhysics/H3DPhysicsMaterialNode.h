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
/// \file H3DPhysicsMaterialNode.h
/// \brief Header file for H3DPhysicsMaterialNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DPHYSICSMATERIALNODE__
#define __H3DPHYSICSMATERIALNODE__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/H3DPhysics/H3DPhysicsMassNode.h>
#include <H3D/H3DPhysics/H3DPhysicsDampingNode.h>
#include <H3D/H3DPhysics/H3DPhysicsFrictionNode.h>

namespace H3D{  

  /// \ingroup AbstractNodes H3DPhysicsMaterialNode
  /// Abstract base node for collection of different types of physicsMaterial property
  /// nodes. 
  ///
  /// Subclasses differ depending on which physicsMaterial properties are needed
  /// for the deformationStratergy used. The mass-spring algorithm, for example,
  /// needs stiffness, while Finite Element Method needs elasticity values.
  ///
  /// \par Internal routes:
  /// \dotfile H3DPhysicsMaterialNode.dot
  class H3DPHYS_API H3DPhysicsMaterialNode : public X3DNode {
  public:

    /// SFH3DPhysicsMaterialNode is dependent on the
    /// materialPropertyChanged field of the contained H3DPhysicsMassNode.
    typedef DependentSFNode< H3DPhysicsMassNode,
      FieldRef< H3DPhysicsMaterialPropertyNode,
      Field, &H3DPhysicsMassNode::materialPropertyChanged> >
      SFH3DPhysicsMassNode;

    /// SFH3DPhysicsMaterialNode is dependent on the
    /// materialPropertyChanged field of the contained H3DPhysicsDampingNode.
    typedef DependentSFNode< H3DPhysicsDampingNode,
      FieldRef< H3DPhysicsMaterialPropertyNode,
      Field, &H3DPhysicsDampingNode::materialPropertyChanged> >
      SFH3DPhysicsDampingNode;

    /// SFH3DPhysicsMaterialNode is dependent on the
    /// materialPropertyChanged field of the contained H3DPhysicsFrictionNode.
    typedef DependentSFNode< H3DPhysicsFrictionNode,
      FieldRef< H3DPhysicsMaterialPropertyNode,
      Field, &H3DPhysicsFrictionNode::materialPropertyChanged> >
      SFH3DPhysicsFrictionNode;

    /// A field type used to collect changes to fields of the H3DPhysicsMaterialNode
    class H3DPHYS_API ValueUpdater : 
      public EventCollectingField < PeriodicUpdate < Field > > {
    public:
      ValueUpdater() : allParams( false ) {}

      virtual PhysicsEngineParameters::H3DPhysicsMaterialParameters* getH3DPhysicsMaterialParameters( bool all_params = false );
    protected:
      virtual void update();

      AutoRef < PhysicsEngineParameters::H3DPhysicsMaterialParameters > params;
      bool allParams;
    };

    /// Constructor.
    H3DPhysicsMaterialNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0,
      Inst< SFH3DPhysicsMassNode > _mass = 0,
      Inst< SFH3DPhysicsDampingNode > _damping = 0,
      Inst< SFH3DPhysicsFrictionNode > _friction = 0 );

    /// Returns the default xml containerField attribute value.
    /// For this node it is "physicsMaterial".
    virtual string defaultXMLContainerField() {
      return "physicsMaterial";
    }

    /// The H3DPhysicsMassNode including mass mapping for the soft body.
    /// The mapping could be a uniform value thoroughout the whole soft body
    /// as well as a non-homogenous distribution.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DPhysicsMaterialNode_mass.dot
    auto_ptr < SFH3DPhysicsMassNode > mass;

    /// The H3DPhysicsDampingNode including damping mapping for the soft body.
    /// The mapping could be a uniform value thoroughout the whole soft body
    /// as well as a non-homogenous distribution.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DPhysicsMaterialNode_damping.dot
    auto_ptr < SFH3DPhysicsDampingNode > damping;

    /// The H3DPhysicsFrictionNode including friction mapping for the soft body.
    /// The mapping could be a uniform value thoroughout the whole soft body
    /// as well as a non-homogenous distribution.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DPhysicsMaterialNode_friction.dot
    auto_ptr < SFH3DPhysicsFrictionNode > friction;

    /// Field that gets an event when any of the X3D fields in the
    /// H3DPhysicsMaterialNode generates an event
    auto_ptr< Field > materialChanged;

    /// A field type used to collect changes to fields of the H3DPhysicsMaterialNode
    /// C++ only field.
    /// 
    /// \dotfile H3DPhysicsMaterialNode_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:


    /// Creates a new instance of a subclass of H3DPhysicsMaterialParameters appropriate for the subclass of collidable
    virtual PhysicsEngineParameters::H3DPhysicsMaterialParameters* createH3DPhysicsMaterialParameters ()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// Returns a H3DPhysicsMaterialParameters to describe the collidable. By default
    /// the function returns a H3DPhysicsMaterialParameters with values
    /// that have changed since the last loop.
    //// \param all_params If true then it returns all field values regardless
    /// of whether the values have changed
    virtual PhysicsEngineParameters::H3DPhysicsMaterialParameters* getH3DPhysicsMaterialParameters( bool all_params = false );

    /// Variables used to create all parameters of the node incase the whole
    /// node is changed.
    H3DPhysicsMassNode *previousMass;
    H3DPhysicsDampingNode *previousDamping;
    H3DPhysicsFrictionNode *previousFriction;

  };
}
#endif
