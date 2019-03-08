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
/// \file H3DPhysicsMaterialPropertyNode.h
/// \brief Header file for H3DPhysicsMaterialPropertyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DPHYSICSMATERIALPROPERTYNODE__
#define __H3DPHYSICSMATERIALPROPERTYNODE__

#include <H3D/PeriodicUpdate.h>
#include <H3D/SFString.h>
#include <H3D/SFNode.h>
#include <H3D/X3DNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

namespace H3D{  

  /// \ingroup AbstractNodes H3DPhysicsMaterialPropertyNode
  /// Abstract base node for physicsMaterial property nodes.
  ///
  /// Subclasses are added in the scenegraph as children of H3DPhysicsMaterialNode.
  ///
  /// \par Internal routes:
  /// \dotfile H3DPhysicsMaterialPropertyNode.dot
  class H3DPHYS_API H3DPhysicsMaterialPropertyNode : public X3DNode {
  public:

    /// A field type used to collect changes to fields of the H3DPhysicsMaterialPropertyNode
    ///
    /// The parent node (H3DSoftBody) will request an instance of MaterialPropertyParameters by calling
    /// getMaterialPropertyParameters(), which will be added to the physics engine struct for that node.
    class H3DPHYS_API ValueUpdater : 
      public EventCollectingField < PeriodicUpdate < Field > > {
    public:
      virtual PhysicsEngineParameters::MaterialPropertyParameters* getMaterialPropertyParameters( bool all_params = false );
    protected:
      virtual void update();

      AutoRef < PhysicsEngineParameters::MaterialPropertyParameters > params;
      bool allParams;
    };

    /// Constructor.
    H3DPhysicsMaterialPropertyNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0,
      Inst< SFString > _unitType = 0);

    /// The unit type for the node.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "UNIT_UNIFORM" \n
    /// <b>Valid values:</b> "UNIT_UNIFORM", "UNIT_NODE", "UNIT_EDGE", "UNIT_ELEMENT" \n
    /// 
    /// \dotfile H3DPhysicsMaterialPropertyNode_unitType.dot
    auto_ptr< SFString > unitType;

    /// Field that gets an event when any of the X3D fields in the
    /// H3DPhysicsMaterialNode generates an event
    /// C++ only field
    /// 
    /// \dotfile H3DPhysicsMaterialPropertyNode_materialPropertyChanged.dot
    auto_ptr< Field > materialPropertyChanged;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// A field type used to collect changes to fields of the H3DPhysicsMaterialNode
    /// C++ only field.
    /// 
    /// \dotfile H3DPhysicsMaterialPropertyNode_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater;

  protected:

    /// Converts a string to the corresponding UnitType enum value
    PhysicsEngineParameters::MaterialPropertyParameters::UnitType unitTypeFromString ( const string& str );

    /// Creates a new instance of a subclass of MaterialPropertyParameters appropriate for the subclass.
    virtual PhysicsEngineParameters::MaterialPropertyParameters* createMaterialPropertyParameters ()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// Returns a MaterialPropertyParameters to describe the material property. By default
    /// the function returns a MaterialPropertyParameters with values
    /// that have changed since the last loop.
    //// \param all_params If true then it returns all field values regardless
    /// of whether the values have changed
    virtual PhysicsEngineParameters::MaterialPropertyParameters* getMaterialPropertyParameters( bool all_params = false );

  };
}
#endif
