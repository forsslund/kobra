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
/// \file PhysX3RigidBodyOptions.h
/// \brief Header file for PhysX3RigidBodyOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSX3RIGIDBODYOPTIONS__
#define __PHYSX3RIGIDBODYOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/SFInt32.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup PhysX3
    /// Structure describing the state of a PhysX3RigidBodyOptions node
    /// to be passed to the physics simulation thread
    struct PhysX3RigidBodyParameters : public EngineOptionParameters {

      /// Constructor
      PhysX3RigidBodyParameters () : 
        contactReportThreshold ( 1.0f ),
        solverPositionIterations ( 4 ),
        solverVelocityIterations ( 1 ),
        createAsStatic (false ){}

      // 'set' functions

      void setContactReportThreshold ( H3DFloat _contactReportThreshold ) {
        update_bit_mask|= CONTACT_REPORT_THRESHOLD;
        contactReportThreshold= _contactReportThreshold;
      }

      void setSolverPositionIterations ( H3DInt32 _solverPositionIterations ) {
        update_bit_mask|= SOLVER_POSITION_ITERATIONS;
        solverPositionIterations= _solverPositionIterations;
      }

      void setSolverVelocityIterations ( H3DInt32 _solverVelocityIterations ) {
        update_bit_mask|= SOLVER_VELOCITY_ITERATIONS;
        solverVelocityIterations= _solverVelocityIterations;
      }

      void setCreateAsStatic( bool _createAsStatic ) {
        update_bit_mask |= CREATE_AS_STATIC;
        createAsStatic = _createAsStatic;
      }
      // 'get' functions

      H3DFloat getContactReportThreshold () {
        return contactReportThreshold;
      }

      H3DInt32 getSolverPositionIterations () {
        return solverPositionIterations;
      }

      H3DInt32 getSolverVelocityIterations () {
        return solverVelocityIterations;
      }

      bool getCreateAsStatic() {
        return createAsStatic;
      }
      // 'have' functions

      bool haveContactReportThreshold () {
        return (update_bit_mask & CONTACT_REPORT_THRESHOLD) != 0;
      }

      bool haveSolverPositionIterations () {
        return (update_bit_mask & SOLVER_POSITION_ITERATIONS) != 0;
      }

      bool haveSolverVelocityIterations () {
        return (update_bit_mask & SOLVER_VELOCITY_ITERATIONS) != 0;
      }

      bool haveCreateAsStatic() {
        return (update_bit_mask & CREATE_AS_STATIC) != 0;
      }

    protected:
      // update bit mask flags
      static const unsigned int CONTACT_REPORT_THRESHOLD   = 0x0001;
      static const unsigned int SOLVER_POSITION_ITERATIONS = 0x0002;
      static const unsigned int SOLVER_VELOCITY_ITERATIONS = 0x0004;
      static const unsigned int CREATE_AS_STATIC = 0x0008;


      H3DFloat contactReportThreshold;

      H3DInt32 solverPositionIterations;

      H3DInt32 solverVelocityIterations;

      bool createAsStatic;
    };
  }

  /// \ingroup PhysX3
  /// Node used to specify options relating to a RigidBody that are specific to
  /// the PhysX3 physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an H3DSoftBodyNode node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// The documentation for each field indicates which function in PhysX is used
  /// when the field is changed. The documentation for PhysX can be found here.
  /// http://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/apireference/files/classPxRigidDynamic.html
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/PhysX3RigidBodyOptions.x3d">PhysX3RigidBodyOptions.x3d</a>
  ///     ( <a href="examples/PhysX3RigidBodyOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysX3RigidBodyOptions.dot
  class H3DPHYS_API PhysX3RigidBodyOptions : public H3DEngineOptions {
  public:

    /// Constructor.
    PhysX3RigidBodyOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFFloat > _contactReportThreshold = 0,
      Inst< SFInt32 > _solverPositionIterations = 0,
      Inst< SFInt32 > _solverVelocityIterations = 0,
      Inst< SFBool > _createAsStatic = 0);

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "PhysX3"
    virtual string getEngine () {
      return "PhysX3";
    }

    /// This field simply uses PxRigidDynamic::setContactReportThreshold function.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1.0f \n
    /// 
    /// \dotfile PhysX3RigidBodyOptions_contactReportThreshold.dot
    auto_ptr < SFFloat > contactReportThreshold;

    /// This field is the first argument to the 
    /// PxRigidDynamic::setSolverIterationCounts function.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 4 \n
    /// 
    /// \dotfile PhysX3RigidBodyOptions_solverPositionIterations.dot
    auto_ptr < SFInt32 > solverPositionIterations;

    /// This field is the second argument to the 
    /// PxRigidDynamic::setSolverIterationCounts function.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile PhysX3RigidBodyOptions_solverPositionIterations.dot
    auto_ptr < SFInt32 > solverVelocityIterations;

    /// This field is to tell whether the body is to be created as PxRigidStatic,
    /// if set to true all other PxRigidDynamic related setting will be ignored
    ///
    /// <b> Access type: </b> initializeOnly
    /// <b> Default value:</b> false \n
    ///
    /// \dotfile PhysX3RigidBodyOptions_createAsStatic.dot
    auto_ptr< SFBool > createAsStatic;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
