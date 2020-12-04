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
/// \file PhysXRigidBodyOptions.h
/// \brief Header file for PhysXRigidBodyOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSXRIGIDBODYOPTIONS__
#define __PHYSXRIGIDBODYOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/SFInt32.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup PhysX
    /// Structure describing the state of a PhysXRigidBodyOptions node
    /// to be passed to the physics simulation thread
    struct PhysXRigidBodyParameters : public EngineOptionParameters {

      /// Constructor
      PhysXRigidBodyParameters () : 
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

      void setMaxDepenetrationVelocity( const H3DFloat& _dp_velocity ) {
        update_bit_mask |= MAX_DEPENETRATION_VELOCITY;
        maxDepenetrationVelocity = _dp_velocity;
      }

      void setLinearVelocityDamping( const H3DFloat& _linear_velocity_daming ) {
        update_bit_mask |= LINEAR_VELOCITY_DAMPING;
        linearVelocityDamping = _linear_velocity_daming;
      }

      void setAngularVelocityDamping( const H3DFloat& _linear_angular_daming ) {
        update_bit_mask |= ANGULAR_VELOCITY_DAMPING;
        angularVelocityDamping = _linear_angular_daming;
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

      const H3DFloat& getMaxDepenetrationVelocity() {
        return maxDepenetrationVelocity;
      }

      const H3DFloat getLinearVelocityDamping() {
        return linearVelocityDamping;
      }

      const H3DFloat getAngularVelocityDamping() {
        return angularVelocityDamping;
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

      bool haveMaxDepenetrationVelocity() {
        return (update_bit_mask & MAX_DEPENETRATION_VELOCITY) != 0;
      }

      bool haveLinearVelocityDamping() {
        return (update_bit_mask & LINEAR_VELOCITY_DAMPING) != 0;
      }

      bool haveAngularVelocityDamping() {
        return (update_bit_mask & ANGULAR_VELOCITY_DAMPING) != 0;
      }

    protected:
      // update bit mask flags
      static const unsigned int CONTACT_REPORT_THRESHOLD   = 0x0001;
      static const unsigned int SOLVER_POSITION_ITERATIONS = 0x0002;
      static const unsigned int SOLVER_VELOCITY_ITERATIONS = 0x0004;
      static const unsigned int CREATE_AS_STATIC = 0x0008;
      static const unsigned int MAX_DEPENETRATION_VELOCITY = 0x0010;
      static const unsigned int LINEAR_VELOCITY_DAMPING = 0x0020;
      static const unsigned int ANGULAR_VELOCITY_DAMPING = 0x0040;


      H3DFloat contactReportThreshold;

      H3DInt32 solverPositionIterations;

      H3DInt32 solverVelocityIterations;

      bool createAsStatic;

      H3DFloat maxDepenetrationVelocity;

      H3DFloat linearVelocityDamping;

      H3DFloat angularVelocityDamping;
    };
  }

  /// \ingroup PhysX
  /// Node used to specify options relating to a RigidBody that are specific to
  /// the PhysX physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an H3DSoftBodyNode node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// The documentation for each field indicates which function in PhysX is used
  /// when the field is changed. The documentation for PhysX can be found here.
  /// http://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/apireference/files/classPxRigidDynamic.html
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/PhysXRigidBodyOptions.x3d">PhysXRigidBodyOptions.x3d</a>
  ///     ( <a href="examples/PhysXRigidBodyOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysXRigidBodyOptions.dot
  class H3DPHYS_API PhysXRigidBodyOptions : public H3DEngineOptions {
  public:

    /// Constructor.
    PhysXRigidBodyOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater   > _valueUpdater = 0,
      Inst< SFFloat > _contactReportThreshold = 0,
      Inst< SFInt32 > _solverPositionIterations = 0,
      Inst< SFInt32 > _solverVelocityIterations = 0,
      Inst< SFBool > _createAsStatic = 0,
      Inst< SFFloat > _maxDepenetrationVelocity = 0,
      Inst< SFFloat > _linearVelocityDamping = 0, 
      Inst< SFFloat > _angularVelocityDamping = 0);

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "PhysX"
    virtual string getEngine () {
#ifdef HAVE_PHYSX4
      return "PhysX4";
#else
      return "PhysX3";
#endif
    }

    /// This field simply uses PxRigidDynamic::setContactReportThreshold function.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1.0f \n
    /// 
    /// \dotfile PhysXRigidBodyOptions_contactReportThreshold.dot
    auto_ptr < SFFloat > contactReportThreshold;

    /// This field is the first argument to the 
    /// PxRigidDynamic::setSolverIterationCounts function.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 4 \n
    /// 
    /// \dotfile PhysXRigidBodyOptions_solverPositionIterations.dot
    auto_ptr < SFInt32 > solverPositionIterations;

    /// This field is the second argument to the 
    /// PxRigidDynamic::setSolverIterationCounts function.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile PhysXRigidBodyOptions_solverPositionIterations.dot
    auto_ptr < SFInt32 > solverVelocityIterations;

    /// This field is to tell whether the body is to be created as PxRigidStatic,
    /// if set to true all other PxRigidDynamic related setting will be ignored
    ///
    /// <b> Access type: </b> initializeOnly
    /// <b> Default value:</b> false \n
    ///
    /// \dotfile PhysXRigidBodyOptions_createAsStatic.dot
    auto_ptr< SFBool > createAsStatic;

    /// This field is to clamp the velocity a body can gain from a collision,
    /// for a complex mesh with low rest offset it may be desirable to lower this value to prevent large impulses
    ///
    /// <b> Access type: </b> inputOutput
    /// <b> Default value:</b> 10.0 \n
    ///
    auto_ptr< SFFloat > maxDepenetrationVelocity;

    /// This field is to set the linear velocity damping used internally by PhysX
    ///
    /// <b> Access type: </b> inputOutput
    /// <b> Default value:</b> 0.0 \n
    ///
    auto_ptr< SFFloat > linearVelocityDamping;

    /// This field is to set the angular velocity damping used internally by PhysX
    ///
    /// <b> Access type: </b> inputOutput
    /// <b> Default value:</b> 0.05 \n
    ///
    auto_ptr< SFFloat > angularVelocityDamping;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
