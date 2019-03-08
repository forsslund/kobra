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
/// \file H3DPhysicsFrictionNode.h
/// \brief Header file for H3DPhysicsFrictionNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DPHYSICSFRICTIONNODE__
#define __H3DPHYSICSFRICTIONNODE__

#include <H3D/H3DPhysics/H3DPhysicsMaterialPropertyNode.h>

namespace H3D{  

  /// Abstract node for friction nodes. 
  ///
  /// \par Internal routes:
  /// \dotfile H3DPhysicsFrictionNode.dot
  class H3DPHYS_API H3DPhysicsFrictionNode : public H3DPhysicsMaterialPropertyNode {
  public:

    /// The helper class used to create friction field as a
    /// general interface for different types of H3DPhysicsFrictionNode.
    /// In case of uniform homogenous friction the field can function
    /// as an SFFloat. In case of non-homogenous friction distribution
    /// the behaviour can be decided by getFrictionPerUnit/setFrictionPerUnit
    /// functions of the subclasses. One needs to be careful while
    /// calling getFrictionPerUnit/setFrictionPerUnit and set/get functions of
    /// this field since these calls might end up in unwanted behaviour
    /// if called to a wrong type of sub-class.
    class H3DPHYS_API SFFriction : public SFFloat {
    public:

      H3DFloat getFrictionPerUnit(int index){
        H3DPhysicsFrictionNode *dn = 
          static_cast< H3DPhysicsFrictionNode * >( getOwner() );
        return dn->getFrictionPerUnit( index );     
      }

      void setFrictionPerUnit(int index, H3DFloat _value){
        H3DPhysicsFrictionNode *dn = 
          static_cast< H3DPhysicsFrictionNode * >( getOwner() );
        dn->setFrictionPerUnit( index, _value );
      }
    };

    /// Constructor.
    H3DPhysicsFrictionNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0,
      Inst< SFString > _unitType = 0,
      Inst< SFFriction > _friction = 0 );

    /// Returns the default xml containerField attribute value.
    /// For this node it is "friction".
    virtual string defaultXMLContainerField() {
      return "friction";
    }

    /// The field including the friction value. field as a
    /// general interface for different types of H3DPhysicsFrictionNode.
    /// In case of uniform homogenous friction the field can function
    /// as an SFFloat. In case of non-homogenous friction distribution
    /// the behaviour can be decided by getFrictionPerUnit/setFrictionPerUnit
    /// functions of the subclasses. By default getFrictionPerUnit returns 0,
    /// and setFrictionPerUnit does nothing.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.0 \n
    /// 
    /// \dotfile H3DPhysicsFrictionNode_friction.dot
    auto_ptr < SFFriction > friction;

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
    virtual H3DFloat getFrictionPerUnit(int index) { return 0; }

    /// By default does nothing, should be overwritten for inhomogenous subclasses.
    virtual void setFrictionPerUnit(int index, H3DFloat value) { }

  };
}
#endif
