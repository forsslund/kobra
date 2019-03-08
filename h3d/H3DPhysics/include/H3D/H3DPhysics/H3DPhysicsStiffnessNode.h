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
/// \file H3DPhysicsStiffnessNode.h
/// \brief Header file for H3DPhysicsStiffnessNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DPHYSICSSTIFFNESSNODE__
#define __H3DPHYSICSSTIFFNESSNODE__

#include <H3D/H3DPhysics/H3DPhysicsMaterialPropertyNode.h>

namespace H3D{  

  /// Abstract node for mass nodes. 
  ///
  /// \par Internal routes:
  /// \dotfile H3DPhysicsStiffnessNode.dot
  class H3DPHYS_API H3DPhysicsStiffnessNode : public H3DPhysicsMaterialPropertyNode {
  public:

    /// The helper class used to create stiffness field as a
    /// general interface for different types of H3DPhysicsStiffnessNode.
    /// In case of uniform homogenous stiffness the field can function
    /// as an SFFloat. In case of non-homogenous stiffness distribution
    /// the behaviour can be decided by getStiffnessPerUnit/setStiffnessPerUnit
    /// functions of the subclasses. One needs to be careful while
    /// calling getStiffnessPerUnit/setStiffnessPerUnit and set/get functions of
    /// this field since these calls might end up in unwanted behaviour
    /// if called to a wrong type of sub-class.
    class H3DPHYS_API SFStiffness : public SFFloat {
    public:

      H3DFloat getStiffnessPerUnit(int index){
        H3DPhysicsStiffnessNode *dn = 
          static_cast< H3DPhysicsStiffnessNode * >( getOwner() );
        return dn->getStiffnessPerUnit( index );     
      }

      void setStiffnessPerUnit(int index, H3DFloat _value){
        H3DPhysicsStiffnessNode *dn = 
          static_cast< H3DPhysicsStiffnessNode * >( getOwner() );
        dn->setStiffnessPerUnit( index, _value );
      }
    };

    /// Constructor.
    H3DPhysicsStiffnessNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0,
      Inst< SFString > _unitType = 0,
      Inst< SFStiffness > _stiffness = 0 );

    /// Returns the default xml containerField attribute value.
    /// For this node it is "stiffness".
    virtual string defaultXMLContainerField() {
      return "stiffness";
    }

    /// The field including the stiffness value. field as a
    /// general interface for different types of H3DPhysicsStiffnessNode.
    /// In case of uniform homogenous stiffness the field can function
    /// as an SFFloat. In case of non-homogenous stiffness distribution
    /// the behaviour can be decided by getStiffnessPerUnit/setStiffnessPerUnit
    /// functions of the subclasses. By default getStiffnessPerUnit returns 0,
    /// and setStiffnessPerUnit does nothing.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.0 \n
    /// 
    /// \dotfile H3DPhysicsStiffnessNode_stiffness.dot
    auto_ptr < SFStiffness > stiffness;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a MaterialPropertyParameters to describe the material property. By default
    /// the function returns a MaterialPropertyParameters with values
    /// that have changed since the last loop.
    //// \param all_params If true then it returns all field values regardless
    /// of whether the values have changed
    virtual PhysicsEngineParameters::MaterialPropertyParameters* getMaterialPropertyParameters( bool all_params = false );

    /// By default returns 0, should be overwritten for inhomogenous subclasses.
    virtual H3DFloat getStiffnessPerUnit(int index) { return 0; }

    /// By default does nothing, should be overwritten for inhomogenous subclasses.
    virtual void setStiffnessPerUnit(int index, H3DFloat value) { }

  };
}
#endif
