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
/// \file H3DSoftBodyOutputNode.h
/// \brief Header file for H3DSoftBodyOutputNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DSOFTBODYOUTPUTNODE__
#define __H3DSOFTBODYOUTPUTNODE__

#include <H3D/X3DNode.h>
#include <H3D/SFString.h>
#include <H3D/MFInt32.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

namespace H3D{  

  /// Abstract base node for output from H3DSoftBodyNode. The attribute
  /// which is specified by the name field is supposed to be updated in the
  /// scenegraph for each unit component which are included in the
  /// index field. Examples of the data to be dispatched are per-vertex,
  /// per-edge or per-volume. 
  ///
  /// \par Internal routes:
  /// \dotfile H3DSoftBodyOutputNode.dot
  class H3DPHYS_API H3DSoftBodyOutputNode : public X3DNode {
  public:

    /// The ValueUpdater field is used to update values in the
    /// SoftBodyPhysicsEngineThread according to changes of fields in the
    /// H3DSoftBodyOutputNode.
    ///
    /// The parent node will request an instance of H3DSoftBodyOutputParameters by calling
    /// getParameters(), which will be added to the physics engine struct for that node.
    class H3DPHYS_API ValueUpdater : 
      public EventCollectingField < Field > {
    public:
      virtual PhysicsEngineParameters::H3DSoftBodyOutputParameters* getParameters( bool all_params = false );
    protected:
      virtual void update();

      AutoRef < PhysicsEngineParameters::H3DSoftBodyOutputParameters > params;
      bool allParams;
    };

    /// Constructor.
    H3DSoftBodyOutputNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFString > _name = 0,
      Inst< MFInt32 > _index = 0,
      Inst< SFString > _unitType = 0 );

    /// Returns the default xml containerField attribute value.
    /// For this node it is "output".
    virtual string defaultXMLContainerField() {
      return "output";
    }

    /// Returns a new concrete instance of H3DSoftBodyOutputParameters appropriate for this subtype of H3DSoftBodyOutputNode
    virtual PhysicsEngineParameters::H3DSoftBodyOutputParameters* createSoftBodyOutputParameters ()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// Returns a new concrete instance of H3DSoftBodyOutputParameters appropriate for this subtype of H3DSoftBodyOutputNode
    /// and populated with values that reflect the current state of this node.
    virtual PhysicsEngineParameters::H3DSoftBodyOutputParameters* getSoftBodyOutputParameters( bool all_params = false );

    /// Set field values from values contained in the specified H3DSoftBodyOutputParameters
    virtual void setOutputParameters ( PhysicsEngineParameters::H3DSoftBodyOutputParameters& params ) {}

    /// The name of the attribute to be dispatched from the unit
    /// component such as vertices, edges etc.
    ///
    /// Valid values and an appropriate default value are defined by 
    /// the concrete sub-classes, e.g., SoftBodyFloatAttribute and SoftBodyVec3fAttribute.
    /// Refer to the main class documentation for the concrete sub-class.
    ///
    /// If the specified name is invalid, then a warning is displayed and the default
    /// is used instead.
    ///
    /// If the name is valid, but is not valid with the specified unitType, then no
    /// warning is displayed, but the value field will not be populated.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> "" \n
    /// 
    /// \dotfile H3DSoftBodyOutputNode_name.dot
    auto_ptr < SFString > name;

    /// The output value will be updated for the unit components which
    /// are included in the index list.
    ///
    /// If the index field is empty then all available attributes are 
    /// retrieved.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile H3DSoftBodyOutputNode_index.dot
    auto_ptr < MFInt32 > index;

    /// The unit type for the output.
    ///
    /// If the specified unit type is invalid, then a warning is displayed and the default
    /// is used instead.
    ///
    /// If the unit type is valid, but is not valid with the specified name field, then no
    /// warning is displayed, but the value field will not be populated.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> "UNIT_NODE" \n
    /// <b>Valid values:</b> "UNIT_NODE", "UNIT_EDGE", "UNIT_ELEMENT" \n
    /// 
    /// \dotfile H3DSoftBodyOutputNode_unitType.dot
    auto_ptr< SFString > unitType;

    /// The valueUpdater field is used to update values in the
    /// SoftBodyPhysicsEngine according to changes of fields in the
    /// H3DSoftBodyOutputNode node.
    /// C++ only field.
    ///
    /// \dotfile H3DSoftBodyOutputNode_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns UnitType enum from field string
    static PhysicsEngineParameters::H3DSoftBodyOutputParameters::UnitType getUnitTypeFromString ( const std::string& str );
  };
}
#endif
