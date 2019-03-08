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
/// \file FEMDeformationStrategy.h
/// \brief Header file for FEMDeformationStrategy, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __FEMDEFORMATIONSTRATEGY__
#define __FEMDEFORMATIONSTRATEGY__

#include <H3D/SFFloat.h>
#include <H3D/SFString.h>
#include <H3D/H3DPhysics/H3DDeformationStrategyNode.h>

namespace H3D{  

  /// \ingroup FEMDeformationStrategy
  /// Base node for Finite Element deformation algorithms.
  ///
  /// \note This class is not supported by any physics engines yet.
  /// \par Internal routes:
  /// \dotfile FEMDeformationStrategy.dot
  class H3DPHYS_API FEMDeformationStrategy : public H3DDeformationStrategyNode {
  public:

    /// Constructor.
    FEMDeformationStrategy(Inst< SFFloat > _timeStep = 0,
      Inst< SFH3DSolverNode > _h3dSolver = 0,
      Inst< ValueUpdater > _valueUpdater = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Creates a new instance of a subclass of DeformationStrategyParameters
    /// appropriate for the subclass of DeformationStrategy
    virtual PhysicsEngineParameters::DeformationStrategyParameters* createDeformationStrategyParameters();

  };
}
#endif
