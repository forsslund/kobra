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
/// \file SoftBodyFloatAttribute.h
/// \brief Header file for SoftBodyFloatAttribute, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DSOFTBODYFLOATATTRIBUTE__
#define __H3DSOFTBODYFLOATATTRIBUTE__

#include <H3D/MFFloat.h>
#include <H3D/H3DPhysics/H3DSoftBodyOutputNode.h>

namespace H3D{  

  /// The node to dispatch float type attribute from the unit components
  /// of the soft body.
  ///
  /// The following unitType values are supported:
  ///  - UNIT_NODE
  ///
  /// The following attribute names are valid for unitType UNIT_NODE:
  ///  - OUTPUT_FORCE_MAGNITUDE
  ///       The magnitude of all forces applied to a vertex
  ///  - OUTPUT_INTERACTION_FORCE_MAGNITUDE
  ///       The magnitude of all forces applied to a vertex by haptic interaction
  ///  - OUTPUT_EXTERNAL_FORCE_MAGNITUDE
  ///       The magnitude of all user applied forces applied to a vertex
  ///  - OUTPUT_SPEED
  ///       The current speed of a vertex
  ///
  /// The following names are supported by ALL engines:
  ///   UNIT_NODE:
  ///     OUTPUT_INTERACTION_FORCE_MAGNITUDE, OUTPUT_EXTERNAL_FORCE_MAGNITUDE
  ///
  /// The following names are supported by Bullet:
  ///   UNIT_NODE:
  ///     OUTPUT_FORCE_MAGNITUDE, OUTPUT_SPEED
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/SoftBodyFloatAttribute.x3d">SoftBodyFloatAttribute.x3d</a>
  ///     ( <a href="examples/SoftBodyFloatAttribute.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SoftBodyFloatAttribute.dot
  class H3DPHYS_API SoftBodyFloatAttribute : public H3DSoftBodyOutputNode {
  public:

    /// Constructor.
    SoftBodyFloatAttribute(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFString > _name = 0,
      Inst< MFInt32 > _index = 0,
      Inst< SFString > _unitType = 0,
      Inst< MFFloat > _value = 0);

    /// Returns a new concrete instance of H3DSoftBodyOutputParameters appropriate for this subtype of H3DSoftBodyOutputNode
    virtual PhysicsEngineParameters::SoftBodyFloatAttributeParameters* createSoftBodyOutputParameters ();

    /// Returns a new concrete instance of H3DSoftBodyOutputParameters appropriate for this subtype of H3DSoftBodyOutputNode
    /// and populated with values that reflect the current state of this node.
    virtual PhysicsEngineParameters::SoftBodyFloatAttributeParameters* getSoftBodyOutputParameters( bool all_params = false );

    /// Set field values from values contained in the specified H3DSoftBodyOutputParameters
    virtual void setOutputParameters ( PhysicsEngineParameters::H3DSoftBodyOutputParameters& params );

    /// The float list for correponding to each unit.
    ///
    /// <b>Access type:</b> outputOnly \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile SoftBodyFloatAttribute_value.dot
    auto_ptr < MFFloat > value;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns OutputType from string description
    PhysicsEngineParameters::SoftBodyFloatAttributeParameters::OutputType getOutputTypeFromString ( const std::string& str );

  };
}
#endif
