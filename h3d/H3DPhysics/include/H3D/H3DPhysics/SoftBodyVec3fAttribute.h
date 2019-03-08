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
/// \file SoftBodyVec3fAttribute.h
/// \brief Header file for SoftBodyVec3fAttribute, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DSOFTBODYVEC3FATTRIBUTE__
#define __H3DSOFTBODYVEC3FATTRIBUTE__

#include <H3D/MFVec3f.h>
#include <H3D/H3DPhysics/H3DSoftBodyOutputNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>

namespace H3D{  

  /// The node to dispatch vec3f type attribute from the unit components
  /// of the soft body.
  ///
  /// The following unitType values are supported:
  ///  - UNIT_NODE
  ///
  /// The following attribute names are valid for unitType UNIT_NODE:
  ///  - OUTPUT_FORCE
  ///       The sum of all forces applied to a vertex
  ///  - OUTPUT_INTERACTION_FORCE
  ///       The sum of all forces applied to a vertex by haptic interaction
  ///  - OUTPUT_EXTERNAL_FORCE
  ///       The sum of all user applied forces applied to a vertex
  ///  - OUTPUT_VELOCITY
  ///       The current velocity of a vertex
  ///
  /// The following names are supported by ALL engines:
  ///   UNIT_NODE:
  ///     OUTPUT_INTERACTION_FORCE, OUTPUT_EXTERNAL_FORCE
  ///
  /// The following names are supported by Bullet:
  ///   UNIT_NODE:
  ///     OUTPUT_FORCE, OUTPUT_VELOCITY
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/SoftBodyVec3fAttribute.x3d">SoftBodyVec3fAttribute.x3d</a>
  ///     ( <a href="examples/SoftBodyVec3fAttribute.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SoftBodyVec3fAttribute.dot
  class H3DPHYS_API SoftBodyVec3fAttribute : public H3DSoftBodyOutputNode {
  public:

    /// Constructor.
    SoftBodyVec3fAttribute(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFString > _name = 0,
      Inst< MFInt32 > _index = 0,
      Inst< SFString > _unitType = 0, 
      Inst< MFVec3f > _value = 0 );

    /// Returns a new concrete instance of H3DSoftBodyOutputParameters appropriate for this subtype of H3DSoftBodyOutputNode
    virtual PhysicsEngineParameters::SoftBodyVec3fAttributeParameters* createSoftBodyOutputParameters ();

    /// Returns a new concrete instance of H3DSoftBodyOutputParameters appropriate for this subtype of H3DSoftBodyOutputNode
    /// and populated with values that reflect the current state of this node.
    virtual PhysicsEngineParameters::SoftBodyVec3fAttributeParameters* getSoftBodyOutputParameters( bool all_params = false );

    /// Set field values from values contained in the specified H3DSoftBodyOutputParameters
    virtual void setOutputParameters ( PhysicsEngineParameters::H3DSoftBodyOutputParameters& params );

    /// The Vec3f list for correponding to each unit.
    ///
    /// <b>Access type:</b> outputOnly \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile SoftBodyVec3fAttribute_value.dot
    auto_ptr < MFVec3f > value;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns OutputType from string description
    PhysicsEngineParameters::SoftBodyVec3fAttributeParameters::OutputType getOutputTypeFromString ( const std::string& str );

  };
}
#endif
