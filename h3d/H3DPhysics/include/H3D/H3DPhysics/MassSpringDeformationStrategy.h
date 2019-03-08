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
/// \file MassSpringDeformationStrategy.h
/// \brief Header file for MassSpringDeformationStrategy, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __MASSSPRINGDEFORMATIONSTRATEGY__
#define __MASSSPRINGDEFORMATIONSTRATEGY__

#include <H3D/H3DPhysics/H3DDeformationStrategyNode.h>

namespace H3D{  

  /// \ingroup MassSpringDeformationStrategy
  /// Base node for mass spring deformation algorithms.
  ///
  /// \note This class is not supported by any physics engines yet.
  /// \par Internal routes:
  /// \dotfile MassSpringDeformationStrategy.dot
  class H3DPHYS_API MassSpringDeformationStrategy : public H3DDeformationStrategyNode {
  public:

    /// Constructor.
    MassSpringDeformationStrategy(
      Inst< SFFloat > _timeStep = 0,
      Inst< SFH3DSolverNode > _h3dSolver = 0,
      Inst< ValueUpdater > _valueUpdater = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Creates a new instance of a subclass of DeformationStrategyParameters
    /// appropriate for the subclass of DeformationStrategy
    virtual PhysicsEngineParameters::DeformationStrategyParameters* createDeformationStrategyParameters();

    /// Returns a DeformationStrategyParameter to describe the DeformationStrategy.
    /// By default the function returns a CollidableParameter with values
    /// that have changed since the last loop.
    ///
    /// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    //virtual PhysicsEngineParameters::DeformationStrategyParameters* getDeformationStrategyParameters( bool all_params = false );

  };
}
#endif
