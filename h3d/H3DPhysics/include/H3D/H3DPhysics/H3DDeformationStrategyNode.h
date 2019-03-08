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
/// \file H3DDeformationStrategyNode.h
/// \brief Header file for H3DDeformationStrategyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DDEFORMATIONSTRATEGYNODE__
#define __H3DDEFORMATIONSTRATEGYNODE__

#include <H3D/SFString.h>
#include <H3D/SFFloat.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DSolverNode.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>


namespace H3D{  

  /// \ingroup AbstractNodes H3DDeformationStrategyNode
  /// Abstract base node for deformation algorithms.
  ///
  /// \par Internal routes:
  /// \dotfile H3DDeformationStrategyNode.dot
  class H3DPHYS_API H3DDeformationStrategyNode : public Node {
  public:

    /// The SFH3DSolverNode is dependent on the
    /// solverChanged field of the contained H3DSolverNode.
    typedef  DependentSFNode< FieldRef<H3DSolverNode,
      Field,
      &H3DSolverNode::solverChanged > > 
      SFH3DSolverNode;

    /// The ValueUpdater field is used to update values in the
    /// SoftBodyPhysicsEngineThread according to changes of fields in the
    /// H3DDeformationStrategyNode.
    class H3DPHYS_API ValueUpdater : 
      public EventCollectingField < Field > {
    public:
      virtual PhysicsEngineParameters::DeformationStrategyParameters* getDeformationStrategyParameters( bool all_params = false );
    protected:
      virtual void update();

      AutoRef < PhysicsEngineParameters::DeformationStrategyParameters > params;
      bool allParams;
    };

    /// Constructor.
    H3DDeformationStrategyNode(
      Inst< SFFloat > _timeStep = 0,
      Inst< SFH3DSolverNode > _h3dSolver = 0,
      Inst< ValueUpdater > _valueUpdater = 0 );

    /// The time step used during calculation of deformation behaviour.
    /// In case of real time solutions it should be set by the simulation
    /// thread to its updateRate. For non real time solutions the value can
    /// be set independently by the user.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.01 \n
    ///
    /// \dotfile H3DDeformationStrategyNode_timeStep.dot
    auto_ptr < SFFloat > timeStep;

    /// The type of the solver to be used to evaluate deformation.
    /// The desired solver is requested from the physics_engine,
    /// however, in case it is not supported the default solver of
    /// the engine is used instead. The user can choose among
    /// the solver types registered in the interface of the physics
    /// engine, check the corresponding documentation for the use
    /// of possible solvers. 
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "DEFAULT_SOLVER" \n
    /// 
    /// \dotfile H3DDeformationStrategyNode_h3dSolver.dot
    auto_ptr< SFH3DSolverNode > h3dSolver;

    /// Field that gets an event when any of the X3D fields in the
    /// H3DDeformationStrategyNode generates an event
    auto_ptr< Field > strategyChanged;

    /// Returns the default xml containerField attribute value.
    /// For this node it is "deformationStrategy".
    virtual string defaultXMLContainerField() {
      return "deformationStrategy";
    }

    /// The valueUpdater field is used to update values in the
    /// SoftBodyPhysicsEngine according to changes of fields in the
    /// H3DDeformationStrategyNode node.
    /// C++ only field.
    ///
    /// \dotfile H3DDeformationStrategyNode_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Creates a new instance of a subclass of DeformationStrategyParameters
    /// appropriate for the subclass of DeformationStrategy
    virtual PhysicsEngineParameters::DeformationStrategyParameters* createDeformationStrategyParameters()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// Returns a DeformationStrategyParameter to describe the DeformationStrategy.
    /// By default the function returns a CollidableParameter with values
    /// that have changed since the last loop.
    ///
    /// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::DeformationStrategyParameters* getDeformationStrategyParameters( bool all_params = false );

  };
}
#endif
