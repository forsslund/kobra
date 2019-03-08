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
/// \file H3DPhysicsPoissonRatioNode.h
/// \brief Header file for H3DPhysicsPoissonRatioNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DPHYSICSPOISSSONRATIONODE__
#define __H3DPHYSICSPOISSSONRATIONODE__

#include <H3D/H3DPhysics/H3DPhysicsMaterialPropertyNode.h>

namespace H3D{  

  /// Abstract node for poissonRatio nodes. 
  ///
  /// \par Internal routes:
  /// \dotfile H3DPhysicsPoissonRatioNode.dot
  class H3DPHYS_API H3DPhysicsPoissonRatioNode : public H3DPhysicsMaterialPropertyNode {
  public:

    /// The helper class used to create poissonRatio field as a
    /// general interface for different types of H3DPhysicsPoissonRatioNode.
    /// In case of uniform homogenous poissonRatio the field can function
    /// as an SFFloat. In case of non-homogenous poissonRatio distribution
    /// the behaviour can be decided by getPoissonRatioPerUnit/setPoissonRatioPerUnit
    /// functions of the subclasses. One needs to be careful while
    /// calling getPoissonRatioPerUnit/setPoissonRatioPerUnit and set/get functions of
    /// this field since these calls might end up in unwanted behaviour
    /// if called to a wrong type of sub-class.
    class H3DPHYS_API SFPoissonRatio : public SFFloat {
    public:

      H3DFloat getPoissonRatioPerUnit(int index){
        H3DPhysicsPoissonRatioNode *dn = 
          static_cast< H3DPhysicsPoissonRatioNode * >( getOwner() );
        return dn->getPoissonRatioPerUnit( index );     
      }

      void setPoissonRatioPerUnit(int index, H3DFloat _value){
        H3DPhysicsPoissonRatioNode *dn = 
          static_cast< H3DPhysicsPoissonRatioNode * >( getOwner() );
        dn->setPoissonRatioPerUnit( index, _value );           
      }
    };

    /// Constructor.
    H3DPhysicsPoissonRatioNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0,
      Inst< SFString > _unitType = 0,
      Inst< SFPoissonRatio > _poissonRatio = 0 );

    /// Returns the default xml containerField attribute value.
    /// For this node it is "poissonRatio".
    virtual string defaultXMLContainerField() {
      return "poissonRatio";
    }

    /// The field including the poissonRatio value. field as a
    /// general interface for different types of H3DPhysicsPoissonRatioNode.
    /// In case of uniform homogenous poissonRatio the field can function
    /// as an SFFloat. In case of non-homogenous poissonRatio distribution
    /// the behaviour can be decided by getPoissonRatioPerUnit/setPoissonRatioPerUnit
    /// functions of the subclasses. By default getPoissonRatioPerUnit returns 0,
    /// and setPoissonRatioPerUnit does nothing.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.0 \n
    /// 
    /// \dotfile H3DPhysicsPoissonRatioNode_poissonRatio.dot
    auto_ptr < SFPoissonRatio > poissonRatio;

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
    virtual H3DFloat getPoissonRatioPerUnit(int index) { return 0; }

    /// By default does nothing, should be overwritten for inhomogenous subclasses.
    virtual void setPoissonRatioPerUnit(int index, H3DFloat value) { }

  };
}
#endif
