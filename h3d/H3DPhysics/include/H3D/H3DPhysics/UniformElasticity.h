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
/// \file UniformElasticity.h
/// \brief Header file for UniformElasticity, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __UNIFORMELASTICITY__
#define __UNIFORMELASTICITY__

#include <H3D/H3DPhysics/H3DPhysicsElasticityNode.h>
#include <H3D/SFString.h>

namespace H3D{  

  /// Node representing homogenous elasticity for the whole soft body.
  ///
  /// \note This class is not supported by any physics engines yet.
  /// \par Internal routes:
  /// \dotfile UniformElasticity.dot
  class H3DPHYS_API UniformElasticity : public H3DPhysicsElasticityNode {
  public:

    /// Constructor.
    UniformElasticity(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0,
      Inst< SFString > _unitType = 0,
      Inst< SFElasticity > _elasticity = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Creates a new instance of a subclass of MaterialPropertyParameters appropriate for the subclass.
    virtual PhysicsEngineParameters::MaterialPropertyParameters* createMaterialPropertyParameters ();

    /// Returns a MaterialPropertyParameters to describe the material property. By default
    /// the function returns a MaterialPropertyParameters with values
    /// that have changed since the last loop.
    //// \param all_params If true then it returns all field values regardless
    /// of whether the values have changed
    virtual PhysicsEngineParameters::MaterialPropertyParameters* getMaterialPropertyParameters( bool all_params = false );

    /// By default returns 0, should be overwritten for inhomogenous subclasses.
    virtual H3DFloat getElasticityPerUnit(int index) { return elasticity->getValue(); }

  };
}
#endif
