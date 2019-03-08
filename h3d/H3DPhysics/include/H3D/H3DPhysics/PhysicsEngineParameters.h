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
/// \file PhysicsEngineParameters.h
/// \brief Header file for PhysicsEngineParameters namespace
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSICSENGINEPARAMETERS__
#define __PHYSICSENGINEPARAMETERS__

#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DTypes.h>
#include <H3D/X3DGeometryNode.h>
#include <H3DUtil/AutoPtrVector.h>

namespace H3D {

  /// Parameters used by physics engine callback functions.
  class PhysicsEngineThread;

  namespace PhysicsEngineParameters {
    typedef H3DUInt64 H3DConstraintId;
    typedef H3DUInt64 H3DModifierId;
    typedef H3DUInt64 H3DBodyId;
    typedef H3DUInt64 H3DCollidableId;
    typedef H3DUInt64 H3DCollidableGroupId;
    typedef H3DUInt64 H3DCollidableExceptionGroupId;
    typedef H3DUInt64 H3DCollidableSelectionGroupId;
    typedef H3DUInt64 H3DSpaceId;

    /// Base for physics engine implementation specific parameters
    struct H3DPHYS_API EngineOptionParameters : public RefCountedClass {
      /// Constructor
      EngineOptionParameters() : update_bit_mask( 0 ) {}

    protected:
      /// Bitmask for which parameters that are set.
      unsigned long int update_bit_mask;
    };

    /// Dynamic world parameters
    struct H3DPHYS_API WorldParameters {
      WorldParameters() :
        engine_thread( NULL ),
        auto_disable( false ),
        constant_force_mix( (H3DFloat) 0.0001 ),
        contact_surface_thickness( 0 ),
        disable_angular_speed( 0 ),
        disable_linear_speed( 0 ),
        disable_time( 0 ),
        enabled( true ),
        error_correction( (H3DFloat)0.8 ),
        gravity( Vec3f( 0, (H3DFloat)-9.8, 0 ) ),
        iterations( 10 ),
        max_correction_speed( -1 ),
        prefer_accuracy( false ),
        useStaticTimeStep( false ),
        update_bit_mask( 0 ),
        engineOptions( NULL ) {
      }

      //////////////////////////////////////////////
      /// Set functions
      inline void setAutoDisable( bool b ) {
        update_bit_mask |= AUTO_DISABLE;
        auto_disable = b;
      }

      inline void setEngine( PhysicsEngineThread& _engine ) {
        engine_thread = &_engine;
      }

      inline void setEnabled( bool b ) {
        enabled = b;
      }

      inline void setConstantForceMix( H3DFloat a ) {
        update_bit_mask |= CONSTANT_FORCE_MIX;
        constant_force_mix = a;
      }

      inline void setContactSurfaceThickness( H3DFloat a ) {
        update_bit_mask |= CONTACT_SURFACE_THICKNESS;
        contact_surface_thickness = a;
      }

      inline void setDisableAngularSpeed( H3DFloat a ) {
        update_bit_mask |= DISABLE_ANGULAR_SPEED;
        disable_angular_speed = a;
      }

      inline void setDisableLinearSpeed( H3DFloat a ) {
        update_bit_mask |= DISABLE_LINEAR_SPEED;
        disable_linear_speed = a;
      }

      inline void setDisableTime( H3DFloat a ) {
        update_bit_mask |= DISABLE_TIME;
        disable_time = a;
      }

      inline void setErrorCorrection( H3DFloat a ) {
        update_bit_mask |= ERROR_CORRECTION;
        error_correction = a;
      }

      inline void setGravity( const Vec3f &a ) {
        update_bit_mask |= GRAVITY;
        gravity = a;
      }

      inline void setIterations( H3DInt32 a ) {
        update_bit_mask |= ITERATIONS;
        iterations = a;
      }

      inline void setMaxCorrectionSpeed( H3DFloat a ) {
        update_bit_mask |= ERROR_CORRECTION;
        max_correction_speed = a;
      }

      inline void setPreferAccuracy( bool a ) {
        update_bit_mask |= PREFER_ACCURACY;
        prefer_accuracy = a;
      }

      void setUseStaticTimeStep( bool _useStaticTimeStep ) {
        update_bit_mask |= USE_STATIC_TIME_STEP;
        useStaticTimeStep = _useStaticTimeStep;
      }

      inline void setEngineOptions( EngineOptionParameters* _engineOptions ) {
        update_bit_mask |= ENGINE_OPTIONS;
        engineOptions = AutoRef<EngineOptionParameters>( _engineOptions );
      }

      //////////////////////////////////////////
      /// Get functions
      inline bool getAutoDisable() {
        return auto_disable;
      }

      inline PhysicsEngineThread* getEngine() {
        return engine_thread;
      }

      inline bool getEnabled() {
        return enabled;
      }

      inline H3DFloat getConstantForceMix() {
        return constant_force_mix;
      }

      inline H3DFloat getContactSurfaceThickness() {
        return contact_surface_thickness;
      }

      inline H3DFloat getDisableAngularSpeed() {
        return disable_angular_speed;
      }

      inline H3DFloat getDisableLinearSpeed() {
        return disable_linear_speed;
      }

      inline H3DFloat getDisableTime() {
        return disable_time;
      }

      inline H3DFloat getErrorCorrection() {
        return error_correction;
      }

      inline const Vec3f &getGravity() {
        return gravity;
      }

      inline H3DInt32 getIterations() {
        return iterations;
      }

      inline H3DFloat getMaxCorrectionSpeed() {
        return max_correction_speed;
      }

      inline bool getPreferAccuracy() {
        return prefer_accuracy;
      }

      bool getUseStaticTimeStep() {
        return useStaticTimeStep;
      }

      inline EngineOptionParameters* getEngineOptions() {
        return engineOptions.get();
      }

      ////////////////////////////////////////
      /// have.. functions
      inline bool haveAutoDisable() {
        return (update_bit_mask & AUTO_DISABLE) != 0;
      }

      inline bool haveConstantForceMix() {
        return (update_bit_mask & CONSTANT_FORCE_MIX) != 0;
      }

      inline bool haveContactSurfaceThickness() {
        return (update_bit_mask & CONTACT_SURFACE_THICKNESS) != 0;
      }

      inline bool haveDisableAngularSpeed() {
        return (update_bit_mask & DISABLE_ANGULAR_SPEED) != 0;
      }

      inline bool haveDisableLinearSpeed() {
        return (update_bit_mask & DISABLE_LINEAR_SPEED) != 0;
      }

      inline bool haveDisableTime() {
        return (update_bit_mask & DISABLE_TIME) != 0;
      }

      inline bool haveErrorCorrection() {
        return (update_bit_mask & ERROR_CORRECTION) != 0;
      }

      inline bool haveGravity() {
        return (update_bit_mask & GRAVITY) != 0;
      }

      inline bool haveIterations() {
        return (update_bit_mask & ITERATIONS) != 0;
      }

      inline bool haveMaxCorrectionSpeed() {
        return (update_bit_mask & MAX_CORRECTION_SPEED) != 0;
      }

      inline bool havePreferAccuracy() {
        return (update_bit_mask & PREFER_ACCURACY) != 0;
      }

      bool haveUseStaticTimeStep() {
        return (update_bit_mask & USE_STATIC_TIME_STEP) != 0;
      }

      inline bool haveEngineOptions() {
        return (update_bit_mask & ENGINE_OPTIONS) != 0;
      }

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( WorldParameters& src );

    protected:
      static const unsigned int AUTO_DISABLE = 0x0001;
      static const unsigned int CONSTANT_FORCE_MIX = 0x0002;
      static const unsigned int CONTACT_SURFACE_THICKNESS = 0x0004;
      static const unsigned int DISABLE_ANGULAR_SPEED = 0x0008;
      static const unsigned int DISABLE_LINEAR_SPEED = 0x0010;
      static const unsigned int DISABLE_TIME = 0x0020;
      static const unsigned int ERROR_CORRECTION = 0x0040;
      static const unsigned int GRAVITY = 0x0080;
      static const unsigned int ITERATIONS = 0x0100;
      static const unsigned int MAX_CORRECTION_SPEED = 0x0200;
      static const unsigned int PREFER_ACCURACY = 0x0400;
      static const unsigned int USE_STATIC_TIME_STEP = 0x0800;
      static const unsigned int ENGINE_OPTIONS = 0x1000;

      /// Bitmask for which parameters that are set.
      unsigned int update_bit_mask;

      bool     auto_disable;
      H3DFloat constant_force_mix;
      H3DFloat contact_surface_thickness;
      H3DFloat disable_angular_speed;
      H3DFloat disable_linear_speed;
      H3DFloat disable_time;
      bool enabled;
      H3DFloat error_correction;
      Vec3f    gravity;
      H3DInt32 iterations;
      H3DFloat max_correction_speed;
      bool     prefer_accuracy;
      bool useStaticTimeStep;

      AutoRef < EngineOptionParameters > engineOptions;

      H3D::PhysicsEngineThread *engine_thread;
    };


    /// Global contact parameters
    struct H3DPHYS_API GlobalContactParameters {
      GlobalContactParameters() :
        engine_thread( NULL ),
        collision_enabled( true ),
        bounce( 0 ),
        friction_coefficients( Vec2f( 0, 0 ) ),
        min_bounce_speed( 0 ),
        slip_factors( Vec2f( 0, 0 ) ),
        softness_constant_force_mix( (H3DFloat) 0.00001 ),
        softness_error_correction( (H3DFloat) 0.8 ),
        surface_speed( Vec2f( 0, 0 ) ),
        contact_report_mode( "DEFAULT" ) {
      }

      H3D::PhysicsEngineThread *engine_thread;
      bool collision_enabled;
      vector<string> applied_parameters;
      H3DFloat bounce;
      Vec2f    friction_coefficients;
      H3DFloat min_bounce_speed;
      Vec2f    slip_factors;
      H3DFloat softness_constant_force_mix;
      H3DFloat softness_error_correction;
      Vec2f    surface_speed;
      string contact_report_mode;
    };

    /// Contact parameters
    struct H3DPHYS_API ContactParameters {
      ContactParameters() :
        body1_id( 0 ),
        body2_id( 0 ),
        bounce( 0 ),
        contact_normal( Vec3f( 0, 0, 0 ) ),
        depth( 0 ),
        friction_coefficients( Vec2f( 0, 0 ) ),
        friction_direction( Vec3f( 0, 0, 0 ) ),
        geom1_id( 0 ),
        geom2_id( 0 ),
        min_bounce_speed( 0 ),
        position( Vec3f( 0, 0, 0 ) ),
        slip_coefficients( Vec2f( 0, 0 ) ),
        softness_constant_force_mix( 0 ),
        softness_error_correction( 0 ),
        surface_speed( Vec2f( 0, 0 ) ) {
      }

      vector<string> applied_parameters;
      H3DBodyId body1_id;
      H3DBodyId body2_id;
      H3DFloat bounce;
      Vec3f    contact_normal;
      H3DFloat depth;
      Vec2f    friction_coefficients;
      Vec3f    friction_direction;
      H3DCollidableId geom1_id;
      H3DCollidableId geom2_id;
      H3DFloat min_bounce_speed;
      Vec3f    position;
      Vec2f    slip_coefficients;
      H3DFloat softness_constant_force_mix;
      H3DFloat softness_error_correction;
      Vec2f    surface_speed;
    };

    /// Parameters for a rigid body.
    struct H3DPHYS_API RigidBodyParameters {
      typedef std::vector< Vec3f > PositionList;
      typedef std::vector< Rotation > OrientationList;

      /// Constructor.
      RigidBodyParameters();

      /// Destructor
      virtual ~RigidBodyParameters() {}

      ////////////////////////////////////////
      /// set.. functions
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      inline void setName( const std::string& _name ) {
        name = _name;
      }
#endif

      inline void setBodyId( H3DBodyId _bodyId ) {
        body_id = _bodyId;
      }

      inline void setEngine( PhysicsEngineThread& _engine ) {
        engine_thread = &_engine;
      }

      inline void setAngularDampingFactor( H3DFloat v ) {
        update_bit_mask |= ANGULAR_DAMPING_FACTOR;
        angular_damping_factor = v;
      }

      inline void setStartAngularVelocity( const Vec3f &v ) {
        update_bit_mask |= START_ANGULAR_VELOCITY;
        start_angular_velocity = v;
      }

      inline void setAutoDamp( bool v ) {
        update_bit_mask |= AUTO_DAMP;
        auto_damp = v;
      }

      inline void setAutoDisable( bool v ) {
        update_bit_mask |= AUTO_DISABLE;
        auto_disable = v;
      }

      inline void setCenterOfMass( const Vec3f &v ) {
        update_bit_mask |= CENTER_OF_MASS;
        center_of_mass = v;
      }

      inline void setDisableAngularSpeed( H3DFloat v ) {
        update_bit_mask |= DISABLE_ANGULAR_SPEED;
        disable_angular_speed = v;
      }

      inline void setDisableLinearSpeed( H3DFloat v ) {
        update_bit_mask |= DISABLE_LINEAR_SPEED;
        disable_linear_speed = v;
      }

      inline void setDisableTime( H3DFloat v ) {
        update_bit_mask |= DISABLE_TIME;
        disable_time = v;
      }

      inline void setEnabled( bool v ) {
        update_bit_mask |= ENABLED;
        enabled = v;
      }

      inline void setFiniteRotationAxis( const Vec3f &v ) {
        update_bit_mask |= FINITE_ROTATION_AXIS;
        finite_rotation_axis = v;
      }

      inline void setFixed( bool v ) {
        update_bit_mask |= FIXED;
        fixed = v;
      }

      inline void setInertia( const Matrix3f &v ) {
        update_bit_mask |= INERTIA;
        inertia = v;
      }

      inline void setLinearDampingFactor( H3DFloat v ) {
        update_bit_mask |= LINEAR_DAMPING_FACTOR;
        linear_damping_factor = v;
      }

      inline void setStartLinearVelocity( const Vec3f &v ) {
        update_bit_mask |= START_LINEAR_VELOCITY;
        start_linear_velocity = v;
      }

      inline void setMass( H3DFloat v ) {
        update_bit_mask |= MASS;
        mass = v;
      }

      inline void setMassDensityModel( Node *v ) {
        update_bit_mask |= MASS_DENSITY_MODEL;
        mass_density_model = v;
      }

      inline void setStartOrientation( const Rotation &v ) {
        update_bit_mask |= START_ORIENTATION;
        start_orientation = v;
      }

      inline void setStartPosition( const Vec3f &v ) {
        update_bit_mask |= START_POSITION;
        start_position = v;
      }

      inline void setUseFiniteRotation( bool v ) {
        update_bit_mask |= USE_FINITE_ROTATION;
        use_finite_rotation = v;
      }

      inline void setUseGlobalGravity( bool v ) {
        update_bit_mask |= USE_GLOBAL_GRAVITY;
        use_global_gravity = v;
      }

      inline void setAngularVelocity( const Vec3f &v ) {
        update_bit_mask |= ANGULAR_VELOCITY;
        angular_velocity = v;
      }

      inline void setLinearVelocity( const Vec3f &v ) {
        update_bit_mask |= LINEAR_VELOCITY;
        linear_velocity = v;
      }

      inline void setPosition( const Vec3f &v ) {
        update_bit_mask |= POSITION;
        position = v;
      }

      inline void setOrientation( const Rotation &v ) {
        update_bit_mask |= ORIENTATION;
        orientation = v;
      }

      inline void setTorque( const Vec3f &t ) {
        update_bit_mask |= TORQUE;
        torque = t;
      }

      inline void setForce( const Vec3f &f ) {
        update_bit_mask |= FORCE;
        force = f;
      }

      inline void setGraphicsFrameTorque( const Vec3f &t ) {
        update_bit_mask |= GRAPHICS_FRAME_TORQUE;
        graphicsFrameTorque = t;
      }

      inline void setGraphicsFrameForce( const Vec3f &f ) {
        update_bit_mask |= GRAPHICS_FRAME_FORCE;
        graphicsFrameForce = f;
      }

      inline void setGeometry( const vector< H3DCollidableId > &geom_ids ) {
        update_bit_mask |= GEOMETRY;
        geometry = geom_ids;
      }

      inline void setKinematicControl( bool _kinematicControl ) {
        update_bit_mask |= KINEMATIC_CONTROL;
        kinematicControl = _kinematicControl;
      }

      inline void setDebug( bool _debug ) {
        update_bit_mask |= H3D_RB_DEBUG_MODE;
        debug = _debug;
      }

      inline void setEngineOptions( EngineOptionParameters* _engineOptions ) {
        update_bit_mask |= ENGINE_OPTIONS;
        engineOptions = AutoRef<EngineOptionParameters>( _engineOptions );
      }

      inline void setPositions( const PositionList& _b ) {
        update_bit_mask |= POSITIONS;
        positions = _b;
      }

      inline void setOrientations( const OrientationList& _b ) {
        update_bit_mask |= ORIENTATIONS;
        orientations = _b;
      }

      ////////////////////////////////////////
      /// get.. functions
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      inline const std::string& getName() const {
        return name;
      }
#endif

      inline H3DBodyId getBodyId() {
        return body_id;
      }

      inline PhysicsEngineThread* getEngine() {
        return engine_thread;
      }

      inline H3DFloat getAngularDampingFactor() {
        return angular_damping_factor;
      }

      inline const Vec3f & getStartAngularVelocity() {
        return start_angular_velocity;
      }

      inline const Vec3f & getAngularVelocity() {
        return angular_velocity;
      }

      inline bool getAutoDamp() {
        return auto_damp;
      }

      inline bool getAutoDisable() {
        return auto_disable;
      }

      inline const Vec3f &getCenterOfMass() {
        return center_of_mass;
      }

      inline H3DFloat getDisableAngularSpeed() {
        return disable_angular_speed;
      }

      inline H3DFloat getDisableLinearSpeed() {
        return disable_linear_speed;
      }

      inline H3DFloat getDisableTime() {
        return disable_time;
      }

      inline bool getEnabled() {
        return enabled;
      }

      inline const Vec3f &getFiniteRotationAxis() {
        return finite_rotation_axis;
      }

      inline bool getFixed() {
        return fixed;
      }

      inline const Matrix3f &getInertia() {
        return inertia;
      }

      inline H3DFloat getLinearDampingFactor() {
        return linear_damping_factor;
      }

      inline const Vec3f &getStartLinearVelocity() {
        return start_linear_velocity;
      }

      inline const Vec3f &getLinearVelocity() {
        return linear_velocity;
      }

      inline H3DFloat getMass() {
        return mass;
      }

      inline Node * getMassDensityModel() {
        return mass_density_model;
      }

      inline const Rotation &getStartOrientation() {
        return start_orientation;
      }

      inline const Vec3f &getStartPosition() {
        return start_position;
      }

      inline bool getUseFiniteRotation() {
        return use_finite_rotation;
      }

      inline bool getUseGlobalGravity() {
        return use_global_gravity;
      }

      inline const Rotation & getOrientation() {
        return orientation;
      }

      inline const Vec3f & getPosition() {
        return position;
      }

      inline const Vec3f & getTorque() {
        return torque;
      }

      inline const Vec3f & getForce() {
        return force;
      }

      inline const Vec3f & getGraphicsFrameTorque() {
        return graphicsFrameTorque;
      }

      inline const Vec3f & getGraphicsFrameForce() {
        return graphicsFrameForce;
      }

      inline const vector< H3DCollidableId > &getGeometry() {
        return geometry;
      }

      inline bool getKinematicControl() {
        return kinematicControl;
      }

      inline bool getDebug() {
        return debug;
      }

      inline EngineOptionParameters* getEngineOptions() {
        return engineOptions.get();
      }

      inline const PositionList& getPositions() {
        return positions;
      }

      inline const OrientationList& getOrientations() {
        return orientations;
      }

      ////////////////////////////////////////
      /// have.. functions
      inline bool haveAngularDampingFactor() {
        return (update_bit_mask & ANGULAR_DAMPING_FACTOR) != 0;
      }

      inline bool haveStartAngularVelocity() {
        return (update_bit_mask & START_ANGULAR_VELOCITY) != 0;
      }

      inline bool haveAutoDamp() {
        return (update_bit_mask & AUTO_DAMP) != 0;
      }

      inline bool haveAutoDisable() {
        return (update_bit_mask & AUTO_DISABLE) != 0;
      }

      inline bool haveCenterOfMass() {
        return (update_bit_mask & CENTER_OF_MASS) != 0;
      }

      inline bool haveDisableAngularSpeed() {
        return (update_bit_mask & DISABLE_ANGULAR_SPEED) != 0;
      }

      inline bool haveDisableLinearSpeed() {
        return (update_bit_mask & DISABLE_LINEAR_SPEED) != 0;
      }

      inline bool haveDisableTime() {
        return (update_bit_mask & DISABLE_TIME) != 0;
      }

      inline bool haveEnabled() {
        return (update_bit_mask & ENABLED) != 0;
      }

      inline bool haveFiniteRotationAxis() {
        return (update_bit_mask & FINITE_ROTATION_AXIS) != 0;
      }

      inline bool haveFixed() {
        return (update_bit_mask & FIXED) != 0;
      }

      inline bool haveInertia() {
        return (update_bit_mask & INERTIA) != 0;
      }

      inline bool haveLinearDampingFactor() {
        return (update_bit_mask & LINEAR_DAMPING_FACTOR) != 0;
      }

      inline bool haveStartLinearVelocity() {
        return (update_bit_mask & START_LINEAR_VELOCITY) != 0;
      }

      inline bool haveMass() {
        return (update_bit_mask & MASS) != 0;
      }

      inline bool haveMassDensityModel() {
        return (update_bit_mask & MASS_DENSITY_MODEL) != 0;
      }

      inline bool haveStartOrientation() {
        return (update_bit_mask & START_ORIENTATION) != 0;
      }

      inline bool haveStartPosition() {
        return (update_bit_mask & START_POSITION) != 0;
      }

      inline bool havePosition() {
        return (update_bit_mask & POSITION) != 0;
      }

      inline bool haveOrientation() {
        return (update_bit_mask & ORIENTATION) != 0;
      }

      inline bool haveUseFiniteRotation() {
        return (update_bit_mask & USE_FINITE_ROTATION) != 0;
      }

      inline bool haveUseGlobalGravity() {
        return (update_bit_mask & USE_GLOBAL_GRAVITY) != 0;
      }

      inline bool haveForce() {
        return (update_bit_mask & FORCE) != 0;
      }

      inline bool haveTorque() {
        return (update_bit_mask & TORQUE) != 0;
      }

      inline bool haveGraphicsFrameForce() {
        return (update_bit_mask & GRAPHICS_FRAME_FORCE) != 0;
      }

      inline bool haveGraphicsFrameTorque() {
        return (update_bit_mask & GRAPHICS_FRAME_TORQUE) != 0;
      }

      inline bool haveGeometry() {
        return (update_bit_mask & GEOMETRY) != 0;
      }

      inline bool haveKinematicControl() {
        return (update_bit_mask & KINEMATIC_CONTROL) != 0;
      }

      inline bool haveEngineOptions() {
        return (update_bit_mask & ENGINE_OPTIONS) != 0;
      }

      inline bool haveDebug() {
        return (update_bit_mask & H3D_RB_DEBUG_MODE) != 0;
      }

      inline bool havePositions() {
        return (update_bit_mask & POSITIONS) != 0;
      }

      inline bool haveOrientations() {
        return (update_bit_mask & ORIENTATIONS) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( RigidBodyParameters &src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( RigidBodyParameters &src );

    protected:

      static const unsigned int ANGULAR_DAMPING_FACTOR = 0x0001;
      static const unsigned int START_ANGULAR_VELOCITY = 0x0002;
      static const unsigned int AUTO_DAMP = 0x0004;
      static const unsigned int AUTO_DISABLE = 0x0008;
      static const unsigned int CENTER_OF_MASS = 0x0010;
      static const unsigned int DISABLE_ANGULAR_SPEED = 0x0020;
      static const unsigned int DISABLE_LINEAR_SPEED = 0x0040;
      static const unsigned int DISABLE_TIME = 0x0080;
      static const unsigned int ENABLED = 0x0100;
      static const unsigned int FINITE_ROTATION_AXIS = 0x0200;
      static const unsigned int FIXED = 0x0400;
      static const unsigned int INERTIA = 0x0800;
      static const unsigned int LINEAR_DAMPING_FACTOR = 0x1000;
      static const unsigned int START_LINEAR_VELOCITY = 0x2000;
      static const unsigned int MASS = 0x4000;
      static const unsigned int MASS_DENSITY_MODEL = 0x8000;
      static const unsigned int START_ORIENTATION = 0x10000;
      static const unsigned int START_POSITION = 0x20000;
      static const unsigned int USE_FINITE_ROTATION = 0x40000;
      static const unsigned int USE_GLOBAL_GRAVITY = 0x80000;
      static const unsigned int FORCE = 0x100000;
      static const unsigned int TORQUE = 0x200000;
      static const unsigned int GEOMETRY = 0x400000;
      static const unsigned int POSITION = 0x800000;
      static const unsigned int ORIENTATION = 0x1000000;
      static const unsigned int ANGULAR_VELOCITY = 0x2000000;
      static const unsigned int LINEAR_VELOCITY = 0x4000000;
      static const unsigned int ENGINE_OPTIONS = 0x8000000;
      static const unsigned int KINEMATIC_CONTROL = 0x10000000;
      static const unsigned int H3D_RB_DEBUG_MODE = 0x20000000;
      static const unsigned int GRAPHICS_FRAME_FORCE = 0x40000000;
      static const unsigned int GRAPHICS_FRAME_TORQUE = 0x80000000;
      static const unsigned long long POSITIONS    = 0x000800000000;
      static const unsigned long long ORIENTATIONS = 0x001000000000;

      /// Bitmask for which parameters that are set.
      unsigned long long update_bit_mask;

      // input parameters
      H3DFloat angular_damping_factor;
      Vec3f start_angular_velocity;
      bool auto_damp;
      bool auto_disable;
      Vec3f center_of_mass;
      H3DFloat disable_angular_speed;
      H3DFloat disable_linear_speed;
      H3DFloat disable_time;
      bool enabled;
      Vec3f finite_rotation_axis;
      bool fixed;
      Matrix3f inertia;
      H3DFloat linear_damping_factor;
      Vec3f start_linear_velocity;
      H3DFloat mass;
      Node *mass_density_model;
      Rotation start_orientation;
      Vec3f start_position;
      bool use_finite_rotation;
      bool use_global_gravity;

      // Force and torques accumulated each physics frame
      Vec3f force;
      Vec3f torque;

      // Force and torque accumulated each graphics frame
      Vec3f graphicsFrameForce;
      Vec3f graphicsFrameTorque;

      vector< H3DCollidableId > geometry;
      bool kinematicControl;
      bool debug;

      // output parameters
      Rotation orientation;
      Vec3f position;
      Vec3f angular_velocity;
      Vec3f linear_velocity;
      PositionList positions;
      OrientationList orientations;

      AutoRef < EngineOptionParameters > engineOptions;

      H3D::PhysicsEngineThread *engine_thread;
      H3DBodyId body_id;
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      std::string name;
#endif
    };

    /// Collidable shape parameters
    struct H3DPHYS_API ArticulatedRigidBodyParameters : public RigidBodyParameters {
      typedef std::vector< H3DBodyId > BodiesList;

      /// Constructor.
      ArticulatedRigidBodyParameters() :RigidBodyParameters(),
        max_projection_iterations( 4 ), separation_tolerance( 0.1f ) {
      };

      /// Destructor
      virtual ~ArticulatedRigidBodyParameters() {}

      ////////////////////////////////////////
      /// set.. functions
      inline void setStartPositions( const PositionList& _b ) {
        update_bit_mask |= START_POSITIONS;
        start_positions = _b;
      }

      inline void setStartOrientations( const OrientationList& _b ) {
        update_bit_mask |= START_ORIENTATIONS;
        start_orientations = _b;
      }

      inline void setMaxProjectionIterations( H3DInt32 mpi ) {
        update_bit_mask |= MAXPROJECTIONITERATIONS;
        max_projection_iterations = mpi;
      }

      inline void setSeparationTolerance( H3DFloat st ) {
        update_bit_mask |= SEPARATIONTOLERANCE;
        separation_tolerance = st;
      }

      inline void setJointInternalCompliance( H3DFloat jic ) {
        update_bit_mask |= JOINTINTERNALCOMPLIANCE;
        joint_internal_compliance = jic;
      }

      inline void setJointExternalCompliance( H3DFloat jec ) {
        update_bit_mask |= JOINTEXTERNALCOMPLIANCE;
        joint_external_compliance = jec;
      }

      inline void setSwingLimit( const vector< Vec2f > &_swing_limit ) {
        update_bit_mask |= SWINGLIMIT;
        swing_limit = _swing_limit;
      }

      inline void setTwistLimit( const vector< Vec2f > &_twist_limit ) {
        update_bit_mask |= TWISTLIMIT;
        twist_limit = _twist_limit;
      }

      // Outputs
      inline void setBodies( const BodiesList& _b ) {
        update_bit_mask |= BODIES;
        bodies = _b;
      }


      ////////////////////////////////////////
      /// get.. functions
      inline const PositionList& getStartPositions() {
        return start_positions;
      }

      inline const OrientationList& getStartOrientations() {
        return start_orientations;
      }

      inline H3DInt32 getMaxProjectionIterations() {
        return max_projection_iterations;
      }

      inline H3DFloat getSeparationTolerance() {
        return separation_tolerance;
      }

      inline H3DFloat getJointInternalCompliance() {
        return joint_internal_compliance;
      }

      inline H3DFloat getJointExternalCompliance() {
        return joint_external_compliance;
      }

      inline vector< Vec2f > getSwingLimit() {
        return swing_limit;
      }

      inline vector< Vec2f > getTwistLimit() {
        return twist_limit;
      }

      // Outputs
      inline const BodiesList& getBodies() {
        return bodies;
      }

      ////////////////////////////////////////
      /// have.. functions
      inline bool haveStartPositions() {
        return (update_bit_mask & START_POSITIONS) != 0;
      }

      inline bool haveStartOrientations() {
        return (update_bit_mask & START_ORIENTATIONS) != 0;
      }

      inline bool haveMaxProjectionIterations() {
        return (update_bit_mask & MAXPROJECTIONITERATIONS) != 0;
      }

      inline bool haveSeparationTolerance() {
        return (update_bit_mask & SEPARATIONTOLERANCE) != 0;
      }

      inline bool haveJointInternalCompliance() {
        return (update_bit_mask & JOINTINTERNALCOMPLIANCE) != 0;
      }

      inline bool haveJointExternalCompliance() {
        return (update_bit_mask & JOINTEXTERNALCOMPLIANCE) != 0;
      }

      inline bool haveSwingLimit() {
        return (update_bit_mask & SWINGLIMIT) != 0;
      }

      inline bool haveTwistLimit() {
        return (update_bit_mask & TWISTLIMIT) != 0;
      }

      // Outputs
      inline bool haveBodies() {
        return (update_bit_mask & BODIES) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( RigidBodyParameters &src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( RigidBodyParameters &src );

    protected:

      static const unsigned long long BODIES                  = 0x000100000000;
      static const unsigned long long START_POSITIONS         = 0x000200000000;
      static const unsigned long long START_ORIENTATIONS      = 0x000400000000;
      // POSITIONS and ORIENTATIONS of RigidBodyParameters use the numbers
      // between START_ORIENTATIONS AND MAXPROJECTIONITERATIONS
      static const unsigned long long MAXPROJECTIONITERATIONS = 0x002000000000;
      static const unsigned long long SEPARATIONTOLERANCE     = 0x004000000000;
      static const unsigned long long JOINTINTERNALCOMPLIANCE = 0x008000000000;
      static const unsigned long long JOINTEXTERNALCOMPLIANCE = 0x010000000000;
      static const unsigned long long SWINGLIMIT              = 0x020000000000;
      static const unsigned long long TWISTLIMIT              = 0x040000000000;

      BodiesList bodies;
      PositionList start_positions;
      OrientationList start_orientations;
      H3DInt32 max_projection_iterations;
      H3DFloat separation_tolerance;
      H3DFloat joint_internal_compliance;
      H3DFloat joint_external_compliance;
      vector< Vec2f > swing_limit;
      vector< Vec2f > twist_limit;

    };

    /// Collision space parameters
    struct H3DPHYS_API SpaceParameters {
      /// Constructor
      SpaceParameters();

      ////////////////////////////////////////
      /// set.. functions
      inline void setSpaceId( H3DSpaceId _spaceId ) {
        space_id = _spaceId;
      }

      inline void setParentSpaceId( H3DSpaceId _p_spaceId ) {
        parent_space_id = _p_spaceId;
      }

      inline void setEngine( PhysicsEngineThread& _engine ) {
        engine_thread = &_engine;
      }

      inline void setEnabled( bool  _enabled ) {
        enabled = _enabled;
      }

      inline void setUseGeometry( bool _usegeometry ) {
        use_geometry = _usegeometry;
      }

      ////////////////////////////////////////
      /// get.. functions
      inline H3DSpaceId getSpaceId() {
        return space_id;
      }

      inline H3DSpaceId getParentSpaceId() {
        return parent_space_id;
      }

      inline PhysicsEngineThread* getEngine() {
        return engine_thread;
      }

      inline bool getEnabled() {
        return enabled;
      }

      inline bool getUseGeometry() {
        return use_geometry;
      }

    protected:

      H3D::PhysicsEngineThread *engine_thread;
      H3DSpaceId space_id;
      H3DSpaceId parent_space_id;
      bool enabled;
      bool use_geometry;



    };

    /// Collidable parameters
    struct H3DPHYS_API CollidableParameters {

      typedef vector< H3DCollidableGroupId > CollidableGroupList;
      typedef vector< H3DCollidableExceptionGroupId > CollidableExceptionGroupList;
      //typedef vector< H3DCollidableSelectionGroupId > CollidableSelectionGroupList;
      typedef vector< H3DFloat > CollidableSelectionGroupList;

      /// Constructor.
      CollidableParameters();

      /// Destructor. Making the class a polymorphic type.
      virtual ~CollidableParameters() {}

      /// The constant value NO_SPACE is used for parent_space_id
      /// if the collidable should not be added to collidable space in
      /// the simulation
      static const int NO_SPACE = -1;

      /// The constant value WORLD_SPACE is used for parent_space_id
      /// if the collidable should be added to the world collidable space
      /// in the simulation
      static const int WORLD_SPACE = 0;

      ////////////////////////////////////////
      /// set.. functions
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      inline void setName( const std::string& _name ) {
        name = _name;
      }
#endif

      inline void setCollidableId( H3DCollidableId _collidableId ) {
        collidable_id = _collidableId;
      }

      inline void setParentSpaceId( H3DSpaceId  _spaceId ) {
        parent_space_id = _spaceId;
      }

      inline void setEngine( PhysicsEngineThread& _engine ) {
        engine_thread = &_engine;
      }

      inline void setEnabled( bool _v ) {
        update_bit_mask |= ENABLED;
        enabled = _v;
      }

      inline void setRotation( const Rotation& _v ) {
        update_bit_mask |= ROTATION;
        rotation = _v;
      }

      inline void setTranslation( const Vec3f& _v ) {
        update_bit_mask |= TRANSLATION;
        translation = _v;
      }

      inline void setScale( const Vec3f& _s ) {
        update_bit_mask |= SCALE;
        scale = _s;
      }

      inline void setCollidableGroupList( const CollidableGroupList& _cl ) {
        update_bit_mask |= COLLIDABLEGROUPLIST;
        collidableGroupList = _cl;
      }

      inline void setCollidableExceptionGroupList( const CollidableExceptionGroupList& _cl ) {
        update_bit_mask |= COLLIDABLEEXCEPTIONGROUPLIST;
        collidableExceptionGroupList = _cl;
      }

      inline void setCollidableSelectionGroupList( const CollidableSelectionGroupList& _cl ) {
        update_bit_mask |= COLLIDABLESELECTIONGROUPLIST;
        collidableSelectionGroupList = _cl;
      }

      inline void setEngineOptions( EngineOptionParameters* _engineOptions ) {
        update_bit_mask |= ENGINE_OPTIONS;
        engineOptions = AutoRef<EngineOptionParameters>( _engineOptions );
      }

      ////////////////////////////////////////
      /// get.. functions
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      inline const std::string& getName() const {
        return name;
      }
#endif

      inline H3DCollidableId getCollidableId() {
        return collidable_id;
      }

      inline H3DSpaceId  getParentSpaceId() {
        return parent_space_id;
      }

      inline PhysicsEngineThread* getEngine() {
        return engine_thread;
      }

      inline bool getEnabled() {
        return enabled;
      }

      inline const Rotation& getRotation() {
        return rotation;
      }

      inline const Vec3f& getTranslation() {
        return translation;
      }

      inline const Vec3f& getScale() {
        return scale;
      }

      inline const CollidableGroupList& getCollidableGroupList() {
        return collidableGroupList;
      }

      inline const CollidableExceptionGroupList& getCollidableExceptionGroupList() {
        return collidableExceptionGroupList;
      }

      inline const CollidableSelectionGroupList& getCollidableSelectionGroupList() {
        return collidableSelectionGroupList;
      }

      inline EngineOptionParameters* getEngineOptions() {
        return engineOptions.get();
      }

      ////////////////////////////////////////
      /// have.. functions
      inline bool haveEnabled() {
        return (update_bit_mask & ENABLED) != 0;
      }

      inline bool haveRotation() {
        return (update_bit_mask & ROTATION) != 0;
      }

      inline bool haveTranslation() {
        return (update_bit_mask & TRANSLATION) != 0;
      }

      inline bool haveScale() {
        return (update_bit_mask & SCALE) != 0;
      }

      inline bool haveCollidableGroupList() {
        return (update_bit_mask & COLLIDABLEGROUPLIST) != 0;
      }

      inline bool haveCollidableExceptionGroupList() {
        return (update_bit_mask & COLLIDABLEEXCEPTIONGROUPLIST) != 0;
      }

      inline bool haveCollidableSelectionGroupList() {
        return (update_bit_mask & COLLIDABLESELECTIONGROUPLIST) != 0;
      }

      inline bool haveEngineOptions() {
        return (update_bit_mask & ENGINE_OPTIONS) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( CollidableParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( CollidableParameters& src );

    protected:
      static const unsigned int ENABLED             = 0x0001;
      static const unsigned int ROTATION            = 0x0002;
      static const unsigned int TRANSLATION         = 0x0004;
      static const unsigned int SCALE               = 0x0008;
      static const unsigned int COLLIDABLEGROUPLIST = 0x0010;
      static const unsigned int COLLIDABLEEXCEPTIONGROUPLIST = 0x0020;
      static const unsigned int ENGINE_OPTIONS               = 0x0040;
      static const unsigned int COLLIDABLESELECTIONGROUPLIST = 0x0080;

      /// Bitmask for which parameters that are set.
      unsigned int update_bit_mask;

      AutoRef < EngineOptionParameters > engineOptions;

      CollidableGroupList collidableGroupList;
      CollidableExceptionGroupList collidableExceptionGroupList;
      CollidableSelectionGroupList collidableSelectionGroupList;

      H3D::PhysicsEngineThread *engine_thread;
      H3DCollidableId collidable_id;
      H3DSpaceId parent_space_id;
      Rotation  rotation;
      Vec3f     translation;
      Vec3f     scale;
      bool      enabled;
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      std::string name;
#endif
    };

    /// Collidable shape parameters
    struct H3DPHYS_API ShapeParameters : public CollidableParameters {
      /// Constructor.
      ShapeParameters();

      ////////////////////////////////////////
      /// set.. functions
      inline void setSelfCollide( bool v ) {
        update_bit_mask |= SELFCOLLIDE;
        selfCollide = v;
      }

      inline void setClipPlanes( const std::vector< Vec4d > &_clipPlanes ) {
        update_bit_mask |= CLIPPLANES;
        clipPlanes = _clipPlanes;
      }

      /// A clone of the collision geometry that can be accessed by the physics thread
      inline void setShape( X3DGeometryNode * _shape ) {
        update_bit_mask |= SHAPE;
        shape.reset( _shape );
      }

      /// Ptr to the original geometry, only used for ptr comparison to
      /// detect collision with haptics device. Otherwise it should not be accessed
      /// by the physics thread
      inline void setOriginalShape( X3DGeometryNode* _shape ) {
        update_bit_mask |= ORIGINALSHAPE;
        originalShape = _shape;
      }

      inline void setUpdateShapeBounds( bool _updatebounds ) {
        update_bit_mask |= UPDATESHAPEBOUNDS;
        updateShapeBounds = _updatebounds;
      }
      ////////////////////////////////////////
      /// get.. functions
      inline bool getSelfCollide() {
        return selfCollide;
      }

      inline std::vector<Vec4d>& getClipPlanes() {
        return clipPlanes;
      }

      /// A clone of the collision geometry that can be accessed by the physics thread
      inline X3DGeometryNode * getShape() {
        return shape.get();
      }

      /// Ptr to the original geometry, only used for ptr comparison to
      /// detect collision with haptics device. Otherwise it should not be accessed
      /// by the physics thread
      inline X3DGeometryNode * getOriginalShape() {
        return originalShape;
      }

      inline bool getUpdateShapeBounds() {
        return updateShapeBounds;
      }

      ////////////////////////////////////////
      /// have.. functions
      inline bool haveSelfCollide() {
        return (update_bit_mask & SELFCOLLIDE) != 0;
      }

      inline bool haveClipPlanes() {
        return (update_bit_mask & CLIPPLANES) != 0;
      }

      inline bool haveShape() {
        return (update_bit_mask & SHAPE) != 0;
      }

      inline bool haveOriginalShape() {
        return (update_bit_mask & ORIGINALSHAPE) != 0;
      }

      inline bool haveUpdateShapeBounds() {
        return (update_bit_mask & UPDATESHAPEBOUNDS) != 0;
      }
      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ShapeParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ShapeParameters& src );

    protected:

      bool      selfCollide;
      std::vector<Vec4d> clipPlanes;

      static const unsigned int SELFCOLLIDE = 0x0080;
      static const unsigned int CLIPPLANES = 0x0100;
      static const unsigned int SHAPE = 0x0200;
      static const unsigned int ORIGINALSHAPE = 0x0400;
      static const unsigned int UPDATESHAPEBOUNDS = 0x0800;

      /// A clone of the collision geometry that can be accessed by the physics thread
      AutoRef<X3DGeometryNode> shape;

      /// Ptr to the original geometry, only used for ptr comparison to
      /// detect collision with haptics device. Otherwise it should not be accessed
      /// by the physics thread
      X3DGeometryNode* originalShape;

      /// Informs whether to update the shape bounds or not
      bool updateShapeBounds;
    };

    /// Collidable shape parameters
    struct H3DPHYS_API OffsetParameters : public CollidableParameters {
      /// Constructor.
      OffsetParameters();

      H3DCollidableId collidable;

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( OffsetParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( OffsetParameters& src );
    };

    /// Force and torque parameters
    struct H3DPHYS_API ExternalForceTorqueParameters {
      ExternalForceTorqueParameters() :
        engine_thread( NULL ),
        body_id( 0 ),
        force( Vec3f( 0, 0, 0 ) ),
        position( Vec3f( 0, 0, 0 ) ),
        torque( Vec3f( 0, 0, 0 ) ),
        actor_index(0) {
      }
      H3D::PhysicsEngineThread *engine_thread;
      H3DBodyId body_id;
      Vec3f force;
      Vec3f position;
      Vec3f torque;
      unsigned int actor_index;
    };

    /// Constraint parameters
    struct H3DPHYS_API ConstraintParameters {

      ConstraintParameters() :
        update_bit_mask( 0 ),
        all_output( 0 ),
        engine_thread( 0 ),
        constraint_id( 0 ),
        body1_id( 0 ),
        type( "" ),
        engineOptions( NULL ) {
      }

      /// Destructor. Making the class a polymorphic type.
      virtual ~ConstraintParameters() {}

      ////////////////////////////////////////
      /// set.. functions
      inline void setConstraintId( H3DConstraintId _constraintId ) {
        constraint_id = _constraintId;
      }

      inline void setEngine( PhysicsEngineThread& _engine ) {
        engine_thread = &_engine;
      }

      inline void setBody1( H3DBodyId a ) {
        update_bit_mask |= BODY1;
        body1_id = a;
      }

      inline void setEngineOptions( EngineOptionParameters* _engineOptions ) {
        update_bit_mask |= ENGINE_OPTIONS;
        engineOptions = AutoRef<EngineOptionParameters>( _engineOptions );
      }

      ////////////////////////////////////////
      /// get.. functions
      inline const string &getType() {
        return type;
      }

      inline unsigned int getUpdateBitMask() {
        return update_bit_mask;
      }

      inline H3DConstraintId getConstraintId() {
        return constraint_id;
      }

      inline PhysicsEngineThread* getEngine() {
        return engine_thread;
      }

      inline H3DBodyId getBody1() {
        return body1_id;
      }

      inline EngineOptionParameters* getEngineOptions() {
        return engineOptions.get();
      }

      /// Have functions
      inline bool haveBody1() {
        return (update_bit_mask & BODY1) != 0;
      }

      inline bool haveEngineOptions() {
        return (update_bit_mask & ENGINE_OPTIONS) != 0;
      }

      /// Copy the part of the bit mask containing which output parameters are set (defined by all_output)
      inline void copyOutputFlags( unsigned int src_update_bit_mask ) {
        update_bit_mask = (update_bit_mask & ~all_output) | (src_update_bit_mask & all_output);
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:

      /// Type of the body constraint.
      string type;

      /// Bitmask for which parameters that are set.
      unsigned int update_bit_mask;

      /// Bit mask defining output parameters
      unsigned int all_output;

      H3D::PhysicsEngineThread *engine_thread;
      H3DConstraintId constraint_id;
      H3DBodyId body1_id;

      static const unsigned int BODY1             = 0x10000000;
      static const unsigned int ENGINE_OPTIONS    = 0x20000000;

      AutoRef < EngineOptionParameters > engineOptions;
    };

    /// Joint parameters
    struct H3DPHYS_API JointParameters : public ConstraintParameters {

      JointParameters();

      inline void setBody2( H3DBodyId a ) {
        update_bit_mask |= BODY2;
        body2_id = a;
      }

      inline void setMotorTarget( H3DFloat t ) {
        update_bit_mask |= MOTOR_TARGET;
        motor_target = t;
      }

      /// Get functions
      inline H3DBodyId getBody2() {
        return body2_id;
      }

      inline H3DFloat getMotorTarget() {
        return motor_target;
      }

      /// Have functions
      inline bool haveBody2() {
        return (update_bit_mask & BODY2) != 0;
      }

      inline bool haveMotorTarget() {
        return (update_bit_mask & MOTOR_TARGET) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      static const unsigned int BODY2             = 0x40000000;
      static const unsigned int MOTOR_TARGET      = 0x80000000;

      H3DBodyId body2_id;

      H3DFloat motor_target;
    };

    /// FixedJoint parameters
    struct H3DPHYS_API FixedJointParameters : public JointParameters {

      /// Constructor
      FixedJointParameters() {
        type = "FixedJoint";
      };
    };

    /// SingleAxisHingeJoint parameters
    struct H3DPHYS_API SingleAxisHingeJointParameters : public JointParameters {
      /// Constructor
      SingleAxisHingeJointParameters();

      //////////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setAnchorPoint( const Vec3f &a ) {
        update_bit_mask |= ANCHOR_POINT;
        anchor_point = a;
      }

      inline void setAxis( const Vec3f &a ) {
        update_bit_mask |= AXIS;
        axis = a;
      }

      inline void setMinAngle( H3DFloat a ) {
        update_bit_mask |= MIN_ANGLE;
        min_angle = a;
      }

      inline void setMaxAngle( H3DFloat a ) {
        update_bit_mask |= MAX_ANGLE;
        max_angle = a;
      }

      inline void setStopBounce( H3DFloat a ) {
        update_bit_mask |= STOP_BOUNCE;
        stop_bounce = a;
      }

      inline void setStopErrorCorrection( H3DFloat a ) {
        update_bit_mask |= STOP_ERROR_CORRECTION;
        stop_error_correction = a;
      }

      inline void setBias( H3DFloat a ) {
        update_bit_mask |= BIAS;
        bias = a;
      }

      inline void setSoftness( H3DFloat a ) {
        update_bit_mask |= SOFTNESS;
        softness = a;
      }

      /// Output
      inline void setBody1AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY1_ANCHOR_POINT;
        body1_anchor_point = a;
      }

      inline void setBody2AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY2_ANCHOR_POINT;
        body2_anchor_point = a;
      }

      inline void setAngle( H3DFloat a ) {
        update_bit_mask |= ANGLE;
        angle = a;
      }

      inline void setAngleRate( H3DFloat a ) {
        update_bit_mask |= ANGLE_RATE;
        angle_rate = a;
      }

      //////////////////////////////////////////
      /// Get functions
      /// Input
      inline const Vec3f & getAnchorPoint() {
        return anchor_point;
      }

      inline const Vec3f & getAxis() {
        return axis;
      }

      inline H3DFloat getMinAngle() {
        return min_angle;
      }

      inline H3DFloat getMaxAngle() {
        return max_angle;
      }

      inline H3DFloat getStopBounce() {
        return stop_bounce;
      }

      inline H3DFloat getStopErrorCorrection() {
        return stop_error_correction;
      }

      inline H3DFloat getBias() {
        return bias;
      }

      inline H3DFloat getSoftness() {
        return softness;
      }

      /// Output
      inline const Vec3f & getBody1AnchorPoint() {
        return body1_anchor_point;
      }

      inline const Vec3f & getBody2AnchorPoint() {
        return body2_anchor_point;
      }

      inline H3DFloat getAngle() {
        return angle;
      }

      inline H3DFloat getAngleRate() {
        return angle_rate;
      }


      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveAnchorPoint() {
        return (update_bit_mask & ANCHOR_POINT) != 0;
      }

      inline bool haveAxis() {
        return (update_bit_mask & AXIS) != 0;
      }

      inline bool haveMinAngle() {
        return (update_bit_mask & MIN_ANGLE) != 0;
      }

      inline bool haveMaxAngle() {
        return (update_bit_mask & MAX_ANGLE) != 0;
      }

      inline bool haveStopBounce() {
        return (update_bit_mask & STOP_BOUNCE) != 0;
      }

      inline bool haveStopErrorCorrection() {
        return (update_bit_mask & STOP_ERROR_CORRECTION) != 0;
      }

      inline bool haveBias() {
        return (update_bit_mask & BIAS) != 0;
      }

      inline bool haveSoftness() {
        return (update_bit_mask & SOFTNESS) != 0;
      }

      /// Output
      inline bool haveBody1AnchorPoint() {
        return (update_bit_mask & BODY1_ANCHOR_POINT) != 0;
      }

      inline bool haveBody2AnchorPoint() {
        return (update_bit_mask & BODY2_ANCHOR_POINT) != 0;
      }

      inline bool haveAngle() {
        return (update_bit_mask & ANGLE) != 0;
      }

      inline bool haveAngleRate() {
        return (update_bit_mask & ANGLE_RATE) != 0;
      }

      /// enable functions
      inline void enableBody1AnchorPoint() {
        update_bit_mask |= BODY1_ANCHOR_POINT;
      }

      inline void enableBody2AnchorPoint() {
        update_bit_mask |= BODY2_ANCHOR_POINT;
      }

      inline void enableAngle() {
        update_bit_mask |= ANGLE;
      }

      inline void enableAngleRate() {
        update_bit_mask |= ANGLE_RATE;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      /// Input
      static const unsigned int ANCHOR_POINT = 0x0001;
      static const unsigned int AXIS = 0x0002;
      static const unsigned int MAX_ANGLE = 0x0004;
      static const unsigned int MIN_ANGLE = 0x0008;
      static const unsigned int STOP_BOUNCE = 0x0010;
      static const unsigned int STOP_ERROR_CORRECTION = 0x0020;

      /// Output
      static const unsigned int ANGLE = 0x0040;
      static const unsigned int ANGLE_RATE = 0x0080;
      static const unsigned int BODY1_ANCHOR_POINT = 0x0100;
      static const unsigned int BODY2_ANCHOR_POINT = 0x0200;

      /// Input
      Vec3f    anchor_point;
      Vec3f    axis;
      H3DFloat max_angle;
      H3DFloat min_angle;
      H3DFloat stop_bounce;
      H3DFloat stop_error_correction;

      /// Output
      H3DFloat angle;
      H3DFloat angle_rate;
      Vec3f    body1_anchor_point;
      Vec3f    body2_anchor_point;

      /// Input
      static const unsigned int BIAS = 0x0400;
      static const unsigned int SOFTNESS = 0x0800;
      H3DFloat bias;
      H3DFloat softness;
    };



    /// DoubleAxisHingeJoint parameters
    struct H3DPHYS_API DoubleAxisHingeJointParameters : public JointParameters {
      //Constructor
      DoubleAxisHingeJointParameters();

      //////////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setAnchorPoint( const Vec3f &a ) {
        update_bit_mask |= ANCHOR_POINT;
        anchor_point = a;
      }

      inline void setAxis1( const Vec3f &a ) {
        update_bit_mask |= AXIS1;
        axis1 = a;
      }

      inline void setAxis2( const Vec3f &a ) {
        update_bit_mask |= AXIS2;
        axis2 = a;
      }

      inline void setDesiredAngularVelocity1( H3DFloat a ) {
        update_bit_mask |= DESIRED_ANGULAR_VELOCITY1;
        desired_angular_velocity1 = a;
      }

      inline void setDesiredAngularVelocity2( H3DFloat a ) {
        update_bit_mask |= DESIRED_ANGULAR_VELOCITY2;
        desired_angular_velocity2 = a;
      }

      inline void setMinAngle1( H3DFloat a ) {
        update_bit_mask |= MIN_ANGLE1;
        min_angle1 = a;
      }

      inline void setMaxAngle1( H3DFloat a ) {
        update_bit_mask |= MAX_ANGLE1;
        max_angle1 = a;
      }

      inline void setMaxTorque1( H3DFloat a ) {
        update_bit_mask |= MAX_TORQUE1;
        max_torque1 = a;
      }

      inline void setMaxTorque2( H3DFloat a ) {
        update_bit_mask |= MAX_TORQUE2;
        max_torque2 = a;
      }

      inline void setStopBounce1( H3DFloat a ) {
        update_bit_mask |= STOP_BOUNCE1;
        stop_bounce1 = a;
      }

      inline void setStopConstantForceMix1( H3DFloat a ) {
        update_bit_mask |= STOP_CONSTANT_FORCE_MIX1;
        stop_constant_force_mix1 = a;
      }

      inline void setStopErrorCorrection1( H3DFloat a ) {
        update_bit_mask |= STOP_ERROR_CORRECTION1;
        stop_error_correction1 = a;
      }

      inline void setSuspensionErrorCorrection( H3DFloat a ) {
        update_bit_mask |= SUSPENSION_ERROR_CORRECTION;
        suspension_error_correction = a;
      }

      inline void setSuspensionForce( H3DFloat a ) {
        update_bit_mask |= SUSPENSION_FORCE;
        suspension_force = a;
      }

      /// Output
      inline void setBody1AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY1_ANCHOR_POINT;
        body1_anchor_point = a;
      }

      inline void setBody2AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY2_ANCHOR_POINT;
        body2_anchor_point = a;
      }

      inline void setBody1Axis( const Vec3f &a ) {
        update_bit_mask |= BODY1_AXIS;
        body1_axis = a;
      }

      inline void setBody2Axis( const Vec3f &a ) {
        update_bit_mask |= BODY2_AXIS;
        body2_axis = a;
      }

      inline void setHinge1Angle( H3DFloat a ) {
        update_bit_mask |= HINGE1_ANGLE;
        hinge1_angle = a;
      }

      inline void setHinge2Angle( H3DFloat a ) {
        update_bit_mask |= HINGE2_ANGLE;
        hinge2_angle = a;
      }

      inline void setHinge1AngleRate( H3DFloat a ) {
        update_bit_mask |= HINGE1_ANGLE_RATE;
        hinge1_angle_rate = a;
      }

      inline void setHinge2AngleRate( H3DFloat a ) {
        update_bit_mask |= HINGE2_ANGLE_RATE;
        hinge2_angle_rate = a;
      }

      //////////////////////////////////////////
      /// Get functions
      /// Input
      inline const Vec3f & getAnchorPoint() {
        return anchor_point;
      }

      inline const Vec3f & getAxis1() {
        return axis1;
      }

      inline const Vec3f & getAxis2() {
        return axis2;
      }

      inline const H3DFloat getDesiredAngularVelocity1() {
        return desired_angular_velocity1;
      }

      inline const H3DFloat getDesiredAngularVelocity2() {
        return desired_angular_velocity2;
      }

      inline H3DFloat getMinAngle1() {
        return min_angle1;
      }

      inline H3DFloat getMaxAngle1() {
        return max_angle1;
      }

      inline H3DFloat getMaxTorque1() {
        return max_torque1;
      }

      inline H3DFloat getMaxTorque2() {
        return max_torque2;
      }

      inline H3DFloat getStopBounce1() {
        return stop_bounce1;
      }

      inline H3DFloat getStopConstantForceMix1() {
        return stop_constant_force_mix1;
      }

      inline H3DFloat getStopErrorCorrection1() {
        return stop_error_correction1;
      }

      inline H3DFloat getSuspensionErrorCorrection() {
        return suspension_error_correction;
      }

      inline H3DFloat getSuspensionForce() {
        return suspension_force;
      }

      /// Output
      inline const Vec3f & getBody1AnchorPoint() {
        return body1_anchor_point;
      }

      inline const Vec3f & getBody2AnchorPoint() {
        return body2_anchor_point;
      }

      inline const Vec3f & getBody1Axis() {
        return body1_axis;
      }

      inline const Vec3f & getBody2Axis() {
        return body2_axis;
      }

      inline H3DFloat getHinge1Angle() {
        return hinge1_angle;
      }

      inline H3DFloat getHinge2Angle() {
        return hinge2_angle;
      }

      inline H3DFloat getHinge1AngleRate() {
        return hinge1_angle_rate;
      }

      inline H3DFloat getHinge2AngleRate() {
        return hinge2_angle_rate;
      }

      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveAnchorPoint() {
        return (update_bit_mask & ANCHOR_POINT) != 0;
      }

      inline bool haveAxis1() {
        return (update_bit_mask & AXIS1) != 0;
      }

      inline bool haveAxis2() {
        return (update_bit_mask & AXIS2) != 0;
      }

      inline bool haveDesiredAngularVelocity1() {
        return (update_bit_mask & DESIRED_ANGULAR_VELOCITY1) != 0;
      }

      inline bool haveDesiredAngularVelocity2() {
        return (update_bit_mask & DESIRED_ANGULAR_VELOCITY2) != 0;
      }

      inline bool haveMaxAngle1() {
        return (update_bit_mask & MAX_ANGLE1) != 0;
      }

      inline bool haveMinAngle1() {
        return (update_bit_mask & MIN_ANGLE1) != 0;
      }

      inline bool haveMaxTorque1() {
        return (update_bit_mask & MAX_TORQUE1) != 0;
      }

      inline bool haveMaxTorque2() {
        return (update_bit_mask & MAX_TORQUE2) != 0;
      }

      inline bool haveStopBounce1() {
        return (update_bit_mask & STOP_BOUNCE1) != 0;
      }

      inline bool haveStopConstantForceMix1() {
        return (update_bit_mask & STOP_CONSTANT_FORCE_MIX1) != 0;
      }

      inline bool haveStopErrorCorrection1() {
        return (update_bit_mask & STOP_ERROR_CORRECTION1) != 0;
      }

      inline bool haveSuspensionErrorCorrection() {
        return (update_bit_mask & SUSPENSION_ERROR_CORRECTION) != 0;
      }

      inline bool haveSuspensionForce() {
        return (update_bit_mask & SUSPENSION_FORCE) != 0;
      }

      /// Output
      inline bool haveBody1AnchorPoint() {
        return (update_bit_mask & BODY1_ANCHOR_POINT) != 0;
      }

      inline bool haveBody2AnchorPoint() {
        return (update_bit_mask & BODY2_ANCHOR_POINT) != 0;
      }

      inline bool haveBody1Axis() {
        return (update_bit_mask & BODY1_AXIS) != 0;
      }

      inline bool haveBody2Axis() {
        return (update_bit_mask & BODY2_AXIS) != 0;
      }

      inline bool haveHinge1Angle() {
        return (update_bit_mask & HINGE1_ANGLE) != 0;
      }

      inline bool haveHinge2Angle() {
        return (update_bit_mask & HINGE2_ANGLE) != 0;
      }

      inline bool haveHinge1AngleRate() {
        return (update_bit_mask & HINGE1_ANGLE_RATE) != 0;
      }

      inline bool haveHinge2AngleRate() {
        return (update_bit_mask & HINGE2_ANGLE_RATE) != 0;
      }

      /// enable functions
      inline void enableBody1AnchorPoint() {
        update_bit_mask |= BODY1_ANCHOR_POINT;
      }

      inline void enableBody2AnchorPoint() {
        update_bit_mask |= BODY2_ANCHOR_POINT;
      }

      inline void enableBody1Axis() {
        update_bit_mask |= BODY1_AXIS;
      }

      inline void enableBody2Axis() {
        update_bit_mask |= BODY2_AXIS;
      }

      inline void enableHinge1Angle() {
        update_bit_mask |= HINGE1_ANGLE;
      }

      inline void enableHinge2Angle() {
        update_bit_mask |= HINGE2_ANGLE;
      }

      inline void enableHinge1AngleRate() {
        update_bit_mask |= HINGE1_ANGLE_RATE;
      }

      inline void enableHinge2AngleRate() {
        update_bit_mask |= HINGE2_ANGLE_RATE;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      /// Input
      static const unsigned int ANCHOR_POINT       = 0x000001;
      static const unsigned int AXIS1              = 0x000002;
      static const unsigned int AXIS2              = 0x000004;
      static const unsigned int DESIRED_ANGULAR_VELOCITY1 = 0x000008;
      static const unsigned int DESIRED_ANGULAR_VELOCITY2 = 0x000010;
      static const unsigned int MAX_ANGLE1         = 0x000020;
      static const unsigned int MIN_ANGLE1         = 0x000040;
      static const unsigned int MAX_TORQUE1        = 0x000080;
      static const unsigned int MAX_TORQUE2        = 0x000100;
      static const unsigned int STOP_BOUNCE1       = 0x000200;
      static const unsigned int STOP_CONSTANT_FORCE_MIX1    = 0x000400;
      static const unsigned int STOP_ERROR_CORRECTION1      = 0x000800;
      static const unsigned int SUSPENSION_ERROR_CORRECTION = 0x001000;
      static const unsigned int SUSPENSION_FORCE   = 0x002000;

      /// Output
      static const unsigned int BODY1_ANCHOR_POINT = 0x004000;
      static const unsigned int BODY2_ANCHOR_POINT = 0x008000;
      static const unsigned int BODY1_AXIS         = 0x010000;
      static const unsigned int BODY2_AXIS         = 0x020000;
      static const unsigned int HINGE1_ANGLE       = 0x040000;
      static const unsigned int HINGE2_ANGLE       = 0x080000;
      static const unsigned int HINGE1_ANGLE_RATE  = 0x100000;
      static const unsigned int HINGE2_ANGLE_RATE  = 0x200000;

      /// Input
      Vec3f    anchor_point;
      Vec3f    axis1;
      Vec3f    axis2;
      H3DFloat desired_angular_velocity1;
      H3DFloat desired_angular_velocity2;
      H3DFloat max_angle1;
      H3DFloat max_torque1;
      H3DFloat max_torque2;
      H3DFloat min_angle1;
      H3DFloat stop_bounce1;
      H3DFloat stop_constant_force_mix1;
      H3DFloat stop_error_correction1;
      H3DFloat suspension_error_correction;
      H3DFloat suspension_force;

      /// Output
      Vec3f    body1_anchor_point;
      Vec3f    body1_axis;
      Vec3f    body2_anchor_point;
      Vec3f    body2_axis;
      H3DFloat hinge1_angle;
      H3DFloat hinge1_angle_rate;
      H3DFloat hinge2_angle;
      H3DFloat hinge2_angle_rate;


    };

    /// MotorJoint parameters
    struct H3DPHYS_API MotorJointParameters : public JointParameters {

      MotorJointParameters();

      ////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setAxis1Angle( H3DFloat a ) {
        update_bit_mask |= AXIS1_ANGLE;
        axis1_angle = a;
      }

      inline void setAxis1Torque( H3DFloat a ) {
        update_bit_mask |= AXIS1_TORQUE;
        axis1_torque = a;
      }

      inline void setAxis2Angle( H3DFloat a ) {
        update_bit_mask |= AXIS2_ANGLE;
        axis2_angle = a;
      }

      inline void setAxis2Torque( H3DFloat a ) {
        update_bit_mask |= AXIS2_TORQUE;
        axis2_torque = a;
      }

      inline void setAxis3Angle( H3DFloat a ) {
        update_bit_mask |= AXIS3_ANGLE;
        axis3_angle = a;
      }

      inline void setAxis3Torque( H3DFloat a ) {
        update_bit_mask |= AXIS3_TORQUE;
        axis3_torque = a;
      }

      inline void setEnabledAxes( H3DInt32 a ) {
        update_bit_mask |= ENABLED_AXES;
        enabled_axes = a;
      }

      inline void setMotor1Axis( const Vec3f &a ) {
        if( a.length() > Constants::f_epsilon )
          update_bit_mask |= MOTOR1_AXIS;
        else
          update_bit_mask &= ~MOTOR1_AXIS;
        motor1_axis = a;
      }

      inline void setMotor2Axis( const Vec3f &a ) {
        if( a.length() > Constants::f_epsilon )
          update_bit_mask |= MOTOR2_AXIS;
        else
          update_bit_mask &= ~MOTOR2_AXIS;
        motor2_axis = a;
      }

      inline void setMotor3Axis( const Vec3f &a ) {
        if( a.length() > Constants::f_epsilon )
          update_bit_mask |= MOTOR3_AXIS;
        else
          update_bit_mask &= ~MOTOR3_AXIS;
        motor3_axis = a;
      }

      inline void setStop1Bounce( H3DFloat a ) {
        update_bit_mask |= STOP1_BOUNCE;
        stop1_bounce = a;
      }

      inline void setStop1ErrorCorrection( H3DFloat a ) {
        update_bit_mask |= STOP1_ERROR_CORRECTION;
        stop1_error_correction = a;
      }

      inline void setStop2Bounce( H3DFloat a ) {
        update_bit_mask |= STOP2_BOUNCE;
        stop2_bounce = a;
      }

      inline void setStop2ErrorCorrection( H3DFloat a ) {
        update_bit_mask |= STOP2_ERROR_CORRECTION;
        stop2_error_correction = a;
      }

      inline void setStop3Bounce( H3DFloat a ) {
        update_bit_mask |= STOP3_BOUNCE;
        stop3_bounce = a;
      }

      inline void setStop3ErrorCorrection( H3DFloat a ) {
        update_bit_mask |= STOP3_ERROR_CORRECTION;
        stop3_error_correction = a;
      }

      inline void setAutoCalc( bool a ) {
        update_bit_mask |= AUTO_CALC;
        auto_calc = a;
      }

      /// Output
      inline void setMotor1Angle( H3DFloat a ) {
        update_bit_mask |= MOTOR1_ANGLE;
        motor1_angle = a;
      }

      inline void setMotor1AngleRate( H3DFloat a ) {
        update_bit_mask |= MOTOR1_ANGLE_RATE;
        motor1_angle_rate = a;
      }

      inline void setMotor2Angle( H3DFloat a ) {
        update_bit_mask |= MOTOR2_ANGLE;
        motor2_angle = a;
      }

      inline void setMotor2AngleRate( H3DFloat a ) {
        update_bit_mask |= MOTOR2_ANGLE_RATE;
        motor2_angle_rate = a;
      }

      inline void setMotor3Angle( H3DFloat a ) {
        update_bit_mask |= MOTOR3_ANGLE;
        motor3_angle = a;
      }

      inline void setMotor3AngleRate( H3DFloat a ) {
        update_bit_mask |= MOTOR3_ANGLE_RATE;
        motor3_angle_rate = a;
      }

      ////////////////////////////////////////
      /// Get functions
      /// Input
      inline H3DFloat getAxis1Angle() {
        return axis1_angle;
      }

      inline H3DFloat getAxis1Torque() {
        return axis1_torque;
      }

      inline H3DFloat getAxis2Angle() {
        return axis2_angle;
      }

      inline H3DFloat getAxis2Torque() {
        return axis2_torque;
      }

      inline H3DFloat getAxis3Angle() {
        return axis3_angle;
      }

      inline H3DFloat getAxis3Torque() {
        return axis3_torque;
      }

      inline H3DInt32 getEnabledAxes() {
        return enabled_axes;
      }

      inline const Vec3f & getMotor1Axis() {
        return motor1_axis;
      }

      inline const Vec3f & getMotor2Axis() {
        return motor2_axis;
      }

      inline const Vec3f & getMotor3Axis() {
        return motor3_axis;
      }

      inline H3DFloat getStop1Bounce() {
        return stop1_bounce;
      }

      inline H3DFloat getStop1ErrorCorrection() {
        return stop1_error_correction;
      }

      inline H3DFloat getStop2Bounce() {
        return stop2_bounce;
      }

      inline H3DFloat getStop2ErrorCorrection() {
        return stop2_error_correction;
      }

      inline H3DFloat getStop3Bounce() {
        return stop3_bounce;
      }

      inline H3DFloat getStop3ErrorCorrection() {
        return stop3_error_correction;
      }

      inline bool getAutoCalc() {
        return auto_calc;
      }

      /// Output
      inline H3DFloat getMotor1Angle() {
        return motor1_angle;
      }

      inline H3DFloat getMotor1AngleRate() {
        return motor1_angle_rate;
      }

      inline H3DFloat getMotor2Angle() {
        return motor2_angle;
      }

      inline H3DFloat getMotor2AngleRate() {
        return motor2_angle_rate;
      }

      inline H3DFloat getMotor3Angle() {
        return motor3_angle;
      }

      inline H3DFloat getMotor3AngleRate() {
        return motor3_angle_rate;
      }

      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveAxis1Angle() {
        return (update_bit_mask & AXIS1_ANGLE) != 0;
      }

      inline bool haveAxis1Torque() {
        return (update_bit_mask & AXIS1_TORQUE) != 0;
      }

      inline bool haveAxis2Angle() {
        return (update_bit_mask & AXIS2_ANGLE) != 0;
      }

      inline bool haveAxis2Torque() {
        return (update_bit_mask & AXIS2_TORQUE) != 0;
      }

      inline bool haveAxis3Angle() {
        return (update_bit_mask & AXIS3_ANGLE) != 0;
      }

      inline bool haveAxis3Torque() {
        return (update_bit_mask & AXIS3_TORQUE) != 0;
      }

      inline bool haveEnabledAxes() {
        return (update_bit_mask & ENABLED_AXES) != 0;
      }

      inline bool haveMotor1Axis() {
        return (update_bit_mask & MOTOR1_AXIS) != 0;
      }

      inline bool haveMotor2Axis() {
        return (update_bit_mask & MOTOR2_AXIS) != 0;
      }

      inline bool haveMotor3Axis() {
        return (update_bit_mask & MOTOR3_AXIS) != 0;
      }

      inline bool haveStop1Bounce() {
        return (update_bit_mask & STOP1_BOUNCE) != 0;
      }

      inline bool haveStop1ErrorCorrection() {
        return (update_bit_mask & STOP1_ERROR_CORRECTION) != 0;
      }

      inline bool haveStop2Bounce() {
        return (update_bit_mask & STOP2_BOUNCE) != 0;
      }

      inline bool haveStop2ErrorCorrection() {
        return (update_bit_mask & STOP2_ERROR_CORRECTION) != 0;
      }

      inline bool haveStop3Bounce() {
        return (update_bit_mask & STOP3_BOUNCE) != 0;
      }

      inline bool haveStop3ErrorCorrection() {
        return (update_bit_mask & STOP3_ERROR_CORRECTION) != 0;
      }

      inline bool haveAutoCalc() {
        return (update_bit_mask & AUTO_CALC) != 0;
      }

      /// Output
      inline bool haveMotor1Angle() {
        return (update_bit_mask & MOTOR1_ANGLE) != 0;
      }

      inline bool haveMotor1AngleRate() {
        return (update_bit_mask & MOTOR1_ANGLE_RATE) != 0;
      }

      inline bool haveMotor2Angle() {
        return (update_bit_mask & MOTOR2_ANGLE) != 0;
      }

      inline bool haveMotor2AngleRate() {
        return (update_bit_mask & MOTOR2_ANGLE_RATE) != 0;
      }

      inline bool haveMotor3Angle() {
        return (update_bit_mask & MOTOR3_ANGLE) != 0;
      }

      inline bool haveMotor3AngleRate() {
        return (update_bit_mask & MOTOR3_ANGLE_RATE) != 0;
      }

      /// enable functions
      inline void enableMotor1Angle() {
        update_bit_mask |= MOTOR1_ANGLE;
      }

      inline void enableMotor1AngleRate() {
        update_bit_mask |= MOTOR1_ANGLE_RATE;
      }

      inline void enableMotor2Angle() {
        update_bit_mask |= MOTOR2_ANGLE;
      }

      inline void enableMotor2AngleRate() {
        update_bit_mask |= MOTOR2_ANGLE_RATE;
      }

      inline void enableMotor3Angle() {
        update_bit_mask |= MOTOR3_ANGLE;
      }

      inline void enableMotor3AngleRate() {
        update_bit_mask |= MOTOR3_ANGLE_RATE;
      }

      /// Copy the output parameters from the src parameter to this
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameter to this
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      /// Input
      static const unsigned int AXIS1_ANGLE = 0x000001;
      static const unsigned int AXIS1_TORQUE = 0x000002;
      static const unsigned int AXIS2_ANGLE = 0x000004;
      static const unsigned int AXIS2_TORQUE = 0x000008;
      static const unsigned int AXIS3_ANGLE = 0x000010;
      static const unsigned int AXIS3_TORQUE = 0x000020;
      static const unsigned int ENABLED_AXES = 0x000040;
      static const unsigned int MOTOR1_AXIS = 0x000080;
      static const unsigned int MOTOR2_AXIS = 0x000100;
      static const unsigned int MOTOR3_AXIS = 0x000200;
      static const unsigned int STOP1_BOUNCE = 0x000400;
      static const unsigned int STOP1_ERROR_CORRECTION = 0x000800;
      static const unsigned int STOP2_BOUNCE = 0x001000;
      static const unsigned int STOP2_ERROR_CORRECTION = 0x002000;
      static const unsigned int STOP3_BOUNCE = 0x004000;
      static const unsigned int STOP3_ERROR_CORRECTION = 0x008000;
      static const unsigned int AUTO_CALC = 0x010000;

      /// Output
      static const unsigned int MOTOR1_ANGLE = 0x020000;
      static const unsigned int MOTOR1_ANGLE_RATE = 0x040000;
      static const unsigned int MOTOR2_ANGLE = 0x080000;
      static const unsigned int MOTOR2_ANGLE_RATE = 0x100000;
      static const unsigned int MOTOR3_ANGLE = 0x200000;
      static const unsigned int MOTOR3_ANGLE_RATE = 0x400000;

      /// Input
      H3DFloat axis1_angle;
      H3DFloat axis1_torque;
      H3DFloat axis2_angle;
      H3DFloat axis2_torque;
      H3DFloat axis3_angle;
      H3DFloat axis3_torque;
      H3DInt32 enabled_axes;
      Vec3f motor1_axis;
      Vec3f motor2_axis;
      Vec3f motor3_axis;
      H3DFloat stop1_bounce;
      H3DFloat stop1_error_correction;
      H3DFloat stop2_bounce;
      H3DFloat stop2_error_correction;
      H3DFloat stop3_bounce;
      H3DFloat stop3_error_correction;
      bool auto_calc;

      /// Output
      H3DFloat motor1_angle;
      H3DFloat motor1_angle_rate;
      H3DFloat motor2_angle;
      H3DFloat motor2_angle_rate;
      H3DFloat motor3_angle;
      H3DFloat motor3_angle_rate;

    };


    /// SliderJoint parameters
    struct H3DPHYS_API SliderJointParameters : public JointParameters {

      SliderJointParameters();

      ////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setAxis( const Vec3f &a ) {
        update_bit_mask |= AXIS;
        axis = a;
      }

      inline void setMaxSeparation( H3DFloat a ) {
        update_bit_mask |= MAX_SEPARATION;
        max_separation = a;
      }

      inline void setMinSeparation( H3DFloat a ) {
        update_bit_mask |= MIN_SEPARATION;
        min_separation = a;
      }

      inline void setStopBounce( H3DFloat a ) {
        update_bit_mask |= STOP_BOUNCE;
        stop_bounce = a;
      }

      inline void setStopErrorCorrection( H3DFloat a ) {
        update_bit_mask |= STOP_ERROR_CORRECTION;
        stop_error_correction = a;
      }

      /// Output
      inline void setSeparation( H3DFloat a ) {
        update_bit_mask |= SEPARATION;
        separation = a;
      }

      inline void setSeparationRate( H3DFloat a ) {
        update_bit_mask |= SEPARATION_RATE;
        separation_rate = a;
      }

      inline void setSliderForce( H3DFloat a ) {
        update_bit_mask |= SLIDER_FORCE;
        slider_force = a;
      }

      ////////////////////////////////////////
      /// Get functions
      /// Input
      inline const Vec3f & getAxis() {
        return axis;
      }

      inline H3DFloat getMaxSeparation() {
        return max_separation;
      }

      inline H3DFloat getMinSeparation() {
        return min_separation;
      }

      inline H3DFloat getStopBounce() {
        return stop_bounce;
      }

      inline H3DFloat getStopErrorCorrection() {
        return stop_error_correction;
      }

      /// Output
      inline H3DFloat getSeparation() {
        return separation;
      }

      inline H3DFloat getSeparationRate() {
        return separation_rate;
      }

      inline H3DFloat getSliderForce() {
        return slider_force;
      }

      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveAxis() {
        return (update_bit_mask & AXIS) != 0;
      }

      inline bool haveMaxSeparation() {
        return (update_bit_mask & MAX_SEPARATION) != 0;
      }

      inline bool haveMinSeparation() {
        return (update_bit_mask & MIN_SEPARATION) != 0;
      }

      inline bool haveStopBounce() {
        return (update_bit_mask & STOP_BOUNCE) != 0;
      }

      inline bool haveStopErrorCorrection() {
        return (update_bit_mask & STOP_ERROR_CORRECTION) != 0;
      }

      inline bool haveSliderForce() {
        return (update_bit_mask & SLIDER_FORCE) != 0;
      }

      /// Output
      inline bool haveSeparation() {
        return (update_bit_mask & SEPARATION) != 0;
      }

      inline bool haveSeparationRate() {
        return (update_bit_mask & SEPARATION_RATE) != 0;
      }

      /// enable functions
      inline void enableSeparation() {
        update_bit_mask |= SEPARATION;
      }

      inline void enableSeparationRate() {
        update_bit_mask |= SEPARATION_RATE;
      }

      /// Copy the output parameters from the src parameter to this
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameter to this
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      /// Input
      static const unsigned int AXIS = 0x000001;
      static const unsigned int MAX_SEPARATION = 0x000002;
      static const unsigned int MIN_SEPARATION = 0x000004;
      static const unsigned int STOP_BOUNCE = 0x000008;
      static const unsigned int STOP_ERROR_CORRECTION = 0x000010;
      static const unsigned int SLIDER_FORCE = 0x000080;

      /// Output
      static const unsigned int SEPARATION = 0x000020;
      static const unsigned int SEPARATION_RATE = 0x000040;

      /// Input
      Vec3f axis;
      H3DFloat max_separation;
      H3DFloat min_separation;
      H3DFloat stop_bounce;
      H3DFloat stop_error_correction;
      H3DFloat slider_force;

      /// Output
      H3DFloat separation;
      H3DFloat separation_rate;
    };


    /// UniversalJoint parameters
    struct H3DPHYS_API UniversalJointParameters : public JointParameters {

      UniversalJointParameters();

      ////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setAnchorPoint( const Vec3f &a ) {
        update_bit_mask |= ANCHOR_POINT;
        anchor_point = a;
      }

      inline void setAxis1( const Vec3f &a ) {
        update_bit_mask |= AXIS1;
        axis1 = a;
      }

      inline void setAxis2( const Vec3f &a ) {
        update_bit_mask |= AXIS2;
        axis2 = a;
      }

      inline void setStop1Bounce( H3DFloat a ) {
        update_bit_mask |= STOP1_BOUNCE;
        stop1_bounce = a;
      }

      inline void setStop1ErrorCorrection( H3DFloat a ) {
        update_bit_mask |= STOP1_ERROR_CORRECTION;
        stop1_error_correction = a;
      }

      inline void setStop2Bounce( H3DFloat a ) {
        update_bit_mask |= STOP2_BOUNCE;
        stop2_bounce = a;
      }

      inline void setStop2ErrorCorrection( H3DFloat a ) {
        update_bit_mask |= STOP2_ERROR_CORRECTION;
        stop2_error_correction = a;
      }

      /// Output
      inline void setBody1AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY1_ANCHOR_POINT;
        body1_anchor_point = a;
      }

      inline void setBody1Axis( const Vec3f &a ) {
        update_bit_mask |= BODY1_AXIS;
        body1_axis = a;
      }

      inline void setBody2AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY2_ANCHOR_POINT;
        body2_anchor_point = a;
      }

      inline void setBody2Axis( const Vec3f &a ) {
        update_bit_mask |= BODY2_AXIS;
        body2_axis = a;
      }

      ////////////////////////////////////////
      /// Get functions
      /// Input
      inline const Vec3f & getAnchorPoint() {
        return anchor_point;
      }

      inline const Vec3f & getAxis1() {
        return axis1;
      }

      inline const Vec3f & getAxis2() {
        return axis2;
      }

      inline H3DFloat getStop1Bounce() {
        return stop1_bounce;
      }

      inline H3DFloat getStop1ErrorCorrection() {
        return stop1_error_correction;
      }

      inline H3DFloat getStop2Bounce() {
        return stop2_bounce;
      }

      inline H3DFloat getStop2ErrorCorrection() {
        return stop2_error_correction;
      }

      /// Output
      inline const Vec3f & getBody1AnchorPoint() {
        return body1_anchor_point;
      }

      inline const Vec3f & getBody1Axis() {
        return body1_axis;
      }

      inline const Vec3f & getBody2AnchorPoint() {
        return body2_anchor_point;
      }

      inline const Vec3f & getBody2Axis() {
        return body2_axis;
      }

      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveAnchorPoint() {
        return (update_bit_mask & ANCHOR_POINT) != 0;
      }

      inline bool haveAxis1() {
        return (update_bit_mask & AXIS1) != 0;
      }

      inline bool haveAxis2() {
        return (update_bit_mask & AXIS2) != 0;
      }

      inline bool haveStop1Bounce() {
        return (update_bit_mask & STOP1_BOUNCE) != 0;
      }

      inline bool haveStop1ErrorCorrection() {
        return (update_bit_mask & STOP1_ERROR_CORRECTION) != 0;
      }

      inline bool haveStop2Bounce() {
        return (update_bit_mask & STOP2_BOUNCE) != 0;
      }

      inline bool haveStop2ErrorCorrection() {
        return (update_bit_mask & STOP2_ERROR_CORRECTION) != 0;
      }

      /// Output
      inline bool haveBody1AnchorPoint() {
        return (update_bit_mask & BODY1_ANCHOR_POINT) != 0;
      }

      inline bool haveBody1Axis() {
        return (update_bit_mask & BODY1_AXIS) != 0;
      }

      inline bool haveBody2AnchorPoint() {
        return (update_bit_mask & BODY2_ANCHOR_POINT) != 0;
      }

      inline bool haveBody2Axis() {
        return (update_bit_mask & BODY2_AXIS) != 0;
      }

      /// enable functions
      inline void enableBody1AnchorPoint() {
        update_bit_mask |= BODY1_ANCHOR_POINT;
      }

      inline void enableBody1Axis() {
        update_bit_mask |= BODY1_AXIS;
      }

      inline void enableBody2AnchorPoint() {
        update_bit_mask |= BODY2_ANCHOR_POINT;
      }

      inline void enableBody2Axis() {
        update_bit_mask |= BODY2_AXIS;
      }

      /// Copy the output parameters from the src parameter to this
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameter to this
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      /// Input
      static const unsigned int ANCHOR_POINT = 0x000001;
      static const unsigned int AXIS1 = 0x000002;
      static const unsigned int AXIS2 = 0x000004;
      static const unsigned int STOP1_BOUNCE = 0x000008;
      static const unsigned int STOP1_ERROR_CORRECTION = 0x000010;
      static const unsigned int STOP2_BOUNCE = 0x000020;
      static const unsigned int STOP2_ERROR_CORRECTION = 0x000040;

      /// Output
      static const unsigned int BODY1_ANCHOR_POINT = 0x000080;
      static const unsigned int BODY1_AXIS = 0x000100;
      static const unsigned int BODY2_ANCHOR_POINT = 0x000200;
      static const unsigned int BODY2_AXIS = 0x000400;

      /// Input
      Vec3f anchor_point;
      Vec3f axis1;
      Vec3f axis2;
      H3DFloat stop1_bounce;
      H3DFloat stop1_error_correction;
      H3DFloat stop2_bounce;
      H3DFloat stop2_error_correction;

      /// Output
      Vec3f body1_anchor_point;
      Vec3f body1_axis;
      Vec3f body2_anchor_point;
      Vec3f body2_axis;

    };


    /// BallJoint parameters
    struct H3DPHYS_API BallJointParameters : public JointParameters {
      //Constructor
      BallJointParameters();

      //////////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setAnchorPoint( const Vec3f &a ) {
        update_bit_mask |= ANCHOR_POINT;
        anchor_point = a;
      }

      /// Output
      inline void setBody1AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY1_ANCHOR_POINT;
        body1_anchor_point = a;
      }

      inline void setBody2AnchorPoint( const Vec3f &a ) {
        update_bit_mask |= BODY2_ANCHOR_POINT;
        body2_anchor_point = a;
      }

      //////////////////////////////////////////
      /// Get functions
      /// Input
      inline const Vec3f & getAnchorPoint() {
        return anchor_point;
      }

      /// Output
      inline const Vec3f & getBody1AnchorPoint() {
        return body1_anchor_point;
      }

      inline const Vec3f & getBody2AnchorPoint() {
        return body2_anchor_point;
      }

      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveAnchorPoint() {
        return (update_bit_mask & ANCHOR_POINT) != 0;
      }

      /// Output
      inline bool haveBody1AnchorPoint() {
        return (update_bit_mask & BODY1_ANCHOR_POINT) != 0;
      }

      inline bool haveBody2AnchorPoint() {
        return (update_bit_mask & BODY2_ANCHOR_POINT) != 0;
      }

      //////////////////////////////
      /// enable functions
      inline void enableBody1AnchorPoint() {
        update_bit_mask |= BODY1_ANCHOR_POINT;
      }

      inline void enableBody2AnchorPoint() {
        update_bit_mask |= BODY2_ANCHOR_POINT;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      static const unsigned int ANCHOR_POINT = 0x0001;
      static const unsigned int BODY1_ANCHOR_POINT = 0x0002;
      static const unsigned int BODY2_ANCHOR_POINT = 0x0004;

      Vec3f    anchor_point;
      Vec3f    body1_anchor_point;
      Vec3f    body2_anchor_point;
    };

    /// Generic6DOFJoint parameters
    struct H3DPHYS_API Generic6DOFJointParameters : public JointParameters {

      //Constructor
      Generic6DOFJointParameters();

      //////////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setAnchorPoint( const Vec3f &a ) {
        update_bit_mask |= ANCHOR_POINT;
        anchor_point = a;
      }

      inline void setAxis1( const Vec3f &a ) {
        update_bit_mask |= AXIS1;
        axis1 = a;
      }

      inline void setAxis2( const Vec3f &a ) {
        update_bit_mask |= AXIS2;
        axis2 = a;
      }

      inline void setAxis3( const Vec3f &a ) {
        update_bit_mask |= AXIS3;
        axis3 = a;
      }

      inline void setDesiredAngularVelocity1( H3DFloat v ) {
        update_bit_mask |= DESIRED_ANGULAR_VELOCITY1;
        desired_angular_velocity1 = v;
      }

      inline void setDesiredAngularVelocity2( H3DFloat v ) {
        update_bit_mask |= DESIRED_ANGULAR_VELOCITY2;
        desired_angular_velocity2 = v;
      }

      inline void setDesiredAngularVelocity3( H3DFloat v ) {
        update_bit_mask |= DESIRED_ANGULAR_VELOCITY3;
        desired_angular_velocity3 = v;
      }

      inline void setMaxAngle1( H3DFloat a ) {
        update_bit_mask |= MAX_ANGLE1;
        max_angle1 = a;
      }

      inline void setMaxAngle2( H3DFloat a ) {
        update_bit_mask |= MAX_ANGLE2;
        max_angle2 = a;
      }

      inline void setMaxAngle3( H3DFloat a ) {
        update_bit_mask |= MAX_ANGLE3;
        max_angle3 = a;
      }

      inline void setMinAngle1( H3DFloat a ) {
        update_bit_mask |= MIN_ANGLE1;
        min_angle1 = a;
      }

      inline void setMinAngle2( H3DFloat a ) {
        update_bit_mask |= MIN_ANGLE2;
        min_angle2 = a;
      }

      inline void setMinAngle3( H3DFloat a ) {
        update_bit_mask |= MIN_ANGLE3;
        min_angle3 = a;
      }

      inline void setMaxTorque1( H3DFloat t ) {
        update_bit_mask |= MAX_TORQUE1;
        max_torque1 = t;
      }

      inline void setMaxTorque2( H3DFloat t ) {
        update_bit_mask |= MAX_TORQUE2;
        max_torque2 = t;
      }

      inline void setMaxTorque3( H3DFloat t ) {
        update_bit_mask |= MAX_TORQUE3;
        max_torque3 = t;
      }

      inline void setDesiredLinearVelocity1( H3DFloat v ) {
        update_bit_mask |= DESIRED_LINEAR_VELOCITY1;
        desired_linear_velocity1 = v;
      }

      inline void setDesiredLinearVelocity2( H3DFloat v ) {
        update_bit_mask |= DESIRED_LINEAR_VELOCITY2;
        desired_linear_velocity2 = v;
      }

      inline void setDesiredLinearVelocity3( H3DFloat v ) {
        update_bit_mask |= DESIRED_LINEAR_VELOCITY3;
        desired_linear_velocity3 = v;
      }

      inline void setMinLimit1( H3DFloat l ) {
        update_bit_mask |= MIN_LIMIT1;
        min_limit1 = l;
      }

      inline void setMinLimit2( H3DFloat l ) {
        update_bit_mask |= MIN_LIMIT2;
        min_limit2 = l;
      }

      inline void setMinLimit3( H3DFloat l ) {
        update_bit_mask |= MIN_LIMIT3;
        min_limit3 = l;
      }

      inline void setMaxLimit1( H3DFloat l ) {
        update_bit_mask |= MAX_LIMIT1;
        max_limit1 = l;
      }

      inline void setMaxLimit2( H3DFloat l ) {
        update_bit_mask |= MAX_LIMIT2;
        max_limit2 = l;
      }

      inline void setMaxLimit3( H3DFloat l ) {
        update_bit_mask |= MAX_LIMIT3;
        max_limit3 = l;
      }

      inline void setMaxForce1( H3DFloat f ) {
        update_bit_mask |= MAX_FORCE1;
        max_force1 = f;
      }

      inline void setMaxForce2( H3DFloat f ) {
        update_bit_mask |= MAX_FORCE2;
        max_force2 = f;
      }

      inline void setMaxForce3( H3DFloat f ) {
        update_bit_mask |= MAX_FORCE3;
        max_force3 = f;
      }

      //////////////////////////////////////////
      /// Get functions
      /// Input
      inline const Vec3f & getAnchorPoint() {
        return anchor_point;
      }

      inline const Vec3f & getAxis1() {
        return axis1;
      }

      inline const Vec3f & getAxis2() {
        return axis2;
      }

      inline const Vec3f & getAxis3() {
        return axis3;
      }

      inline const H3DFloat getDesiredAngularVelocity1() {
        return desired_angular_velocity1;
      }

      inline const H3DFloat getDesiredAngularVelocity2() {
        return desired_angular_velocity2;
      }

      inline const H3DFloat getDesiredAngularVelocity3() {
        return desired_angular_velocity3;
      }

      inline H3DFloat getMaxAngle1() {
        return max_angle1;
      }

      inline H3DFloat getMaxAngle2() {
        return max_angle2;
      }

      inline H3DFloat getMaxAngle3() {
        return max_angle3;
      }

      inline H3DFloat getMinAngle1() {
        return min_angle1;
      }

      inline H3DFloat getMinAngle2() {
        return min_angle2;
      }

      inline H3DFloat getMinAngle3() {
        return min_angle3;
      }

      inline H3DFloat getMaxTorque1() {
        return max_torque1;
      }

      inline H3DFloat getMaxTorque2() {
        return max_torque2;
      }

      inline H3DFloat getMaxTorque3() {
        return max_torque3;
      }

      inline const H3DFloat getDesiredLinearVelocity1() {
        return desired_linear_velocity1;
      }

      inline const H3DFloat getDesiredLinearVelocity2() {
        return desired_linear_velocity2;
      }

      inline const H3DFloat getDesiredLinearVelocity3() {
        return desired_linear_velocity3;
      }

      inline H3DFloat getMinLimit1() {
        return min_limit1;
      }

      inline H3DFloat getMinLimit2() {
        return min_limit2;
      }

      inline H3DFloat getMinLimit3() {
        return min_limit3;
      }

      inline H3DFloat getMaxLimit1() {
        return max_limit1;
      }

      inline H3DFloat getMaxLimit2() {
        return max_limit2;
      }

      inline H3DFloat getMaxLimit3() {
        return max_limit3;
      }

      inline H3DFloat getMaxForce1() {
        return max_force1;
      }

      inline H3DFloat getMaxForce2() {
        return max_force2;
      }

      inline H3DFloat getMaxForce3() {
        return max_force3;
      }

      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveAnchorPoint() {
        return (update_bit_mask & ANCHOR_POINT) != 0;
      }

      inline bool haveAxis1() {
        return (update_bit_mask & AXIS1) != 0;
      }

      inline bool haveAxis2() {
        return (update_bit_mask & AXIS2) != 0;
      }

      inline bool haveAxis3() {
        return (update_bit_mask & AXIS3) != 0;
      }

      inline bool haveDesiredAngularVelocity1() {
        return (update_bit_mask & DESIRED_ANGULAR_VELOCITY1) != 0;
      }

      inline bool haveDesiredAngularVelocity2() {
        return (update_bit_mask & DESIRED_ANGULAR_VELOCITY2) != 0;
      }

      inline bool haveDesiredAngularVelocity3() {
        return (update_bit_mask & DESIRED_ANGULAR_VELOCITY3) != 0;
      }

      inline bool haveMinAngle1() {
        return (update_bit_mask & MIN_ANGLE1) != 0;
      }

      inline bool haveMinAngle2() {
        return (update_bit_mask & MIN_ANGLE2) != 0;
      }

      inline bool haveMinAngle3() {
        return (update_bit_mask & MIN_ANGLE3) != 0;
      }

      inline bool haveMaxAngle1() {
        return (update_bit_mask & MAX_ANGLE1) != 0;
      }

      inline bool haveMaxAngle2() {
        return (update_bit_mask & MAX_ANGLE2) != 0;
      }

      inline bool haveMaxAngle3() {
        return (update_bit_mask & MAX_ANGLE3) != 0;
      }

      inline bool haveMaxTorque1() {
        return (update_bit_mask & MAX_TORQUE1) != 0;
      }

      inline bool haveMaxTorque2() {
        return (update_bit_mask & MAX_TORQUE2) != 0;
      }

      inline bool haveMaxTorque3() {
        return (update_bit_mask & MAX_TORQUE3) != 0;
      }

      inline bool haveDesiredLinearVelocity1() {
        return (update_bit_mask & DESIRED_LINEAR_VELOCITY1) != 0;
      }

      inline bool haveDesiredLinearVelocity2() {
        return (update_bit_mask & DESIRED_LINEAR_VELOCITY2) != 0;
      }

      inline bool haveDesiredLinearVelocity3() {
        return (update_bit_mask & DESIRED_LINEAR_VELOCITY3) != 0;
      }

      inline bool haveMinLimit1() {
        return (update_bit_mask & MIN_LIMIT1) != 0;
      }

      inline bool haveMinLimit2() {
        return (update_bit_mask & MIN_LIMIT2) != 0;
      }

      inline bool haveMinLimit3() {
        return (update_bit_mask & MIN_LIMIT3) != 0;
      }

      inline bool haveMaxLimit1() {
        return (update_bit_mask & MAX_LIMIT1) != 0;
      }

      inline bool haveMaxLimit2() {
        return (update_bit_mask & MAX_LIMIT2) != 0;
      }

      inline bool haveMaxLimit3() {
        return (update_bit_mask & MAX_LIMIT3) != 0;
      }

      inline bool haveMaxForce1() {
        return (update_bit_mask & MAX_FORCE1) != 0;
      }

      inline bool haveMaxForce2() {
        return (update_bit_mask & MAX_FORCE2) != 0;
      }

      inline bool haveMaxForce3() {
        return (update_bit_mask & MAX_FORCE3) != 0;
      }

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      /// Input
      static const unsigned int ANCHOR_POINT              = 1<<0;
      static const unsigned int AXIS1                     = 1<<1;
      static const unsigned int AXIS2                     = 1<<2;
      static const unsigned int AXIS3                     = 1<<3;
      static const unsigned int DESIRED_ANGULAR_VELOCITY1 = 1<<4;
      static const unsigned int DESIRED_ANGULAR_VELOCITY2 = 1<<5;
      static const unsigned int DESIRED_ANGULAR_VELOCITY3 = 1<<6;
      static const unsigned int MIN_ANGLE1                = 1<<7;
      static const unsigned int MIN_ANGLE2                = 1<<8;
      static const unsigned int MIN_ANGLE3                = 1<<9;
      static const unsigned int MAX_ANGLE1                = 1<<10;
      static const unsigned int MAX_ANGLE2                = 1<<11;
      static const unsigned int MAX_ANGLE3                = 1<<12;
      static const unsigned int MAX_TORQUE1               = 1<<13;
      static const unsigned int MAX_TORQUE2               = 1<<14;
      static const unsigned int MAX_TORQUE3               = 1<<15;
      static const unsigned int DESIRED_LINEAR_VELOCITY1  = 1<<16;
      static const unsigned int DESIRED_LINEAR_VELOCITY2  = 1<<17;
      static const unsigned int DESIRED_LINEAR_VELOCITY3  = 1<<18;
      static const unsigned int MIN_LIMIT1                = 1<<19;
      static const unsigned int MIN_LIMIT2                = 1<<20;
      static const unsigned int MIN_LIMIT3                = 1<<21;
      static const unsigned int MAX_LIMIT1                = 1<<22;
      static const unsigned int MAX_LIMIT2                = 1<<23;
      static const unsigned int MAX_LIMIT3                = 1<<24;
      static const unsigned int MAX_FORCE1                = 1<<25;
      static const unsigned int MAX_FORCE2                = 1<<26;
      static const unsigned int MAX_FORCE3                = 1<<27;

      /// Input
      Vec3f    anchor_point;

      Vec3f    axis1;
      Vec3f    axis2;
      Vec3f    axis3;

      H3DFloat desired_angular_velocity1;
      H3DFloat desired_angular_velocity2;
      H3DFloat desired_angular_velocity3;

      H3DFloat min_angle1;
      H3DFloat min_angle2;
      H3DFloat min_angle3;

      H3DFloat max_angle1;
      H3DFloat max_angle2;
      H3DFloat max_angle3;

      H3DFloat max_torque1;
      H3DFloat max_torque2;
      H3DFloat max_torque3;

      H3DFloat desired_linear_velocity1;
      H3DFloat desired_linear_velocity2;
      H3DFloat desired_linear_velocity3;

      H3DFloat min_limit1;
      H3DFloat min_limit2;
      H3DFloat min_limit3;

      H3DFloat max_limit1;
      H3DFloat max_limit2;
      H3DFloat max_limit3;

      H3DFloat max_force1;
      H3DFloat max_force2;
      H3DFloat max_force3;


    };

    /// DistanceJoint parameters
    struct H3DPHYS_API DistanceJointParameters : public JointParameters {
      //Constructor
      DistanceJointParameters();

      //////////////////////////////////////////////
      /// Set functions
      /// Input
      inline void setMaxDistance( H3DFloat d ) {
        update_bit_mask |= MAX_DISTANCE;
        max_distance = d;
      }

      inline void setMinDistance( H3DFloat d ) {
        update_bit_mask |= MIN_DISTANCE;
        min_distance = d;
      }

      inline void setStiffness( H3DFloat k ) {
        update_bit_mask |= STIFFNESS;
        stiffness = k;
      }

      inline void setDamping( H3DFloat d ) {
        update_bit_mask |= DAMPING;
        damping = d;
      }

      inline void setTolerance( H3DFloat t ) {
        update_bit_mask |= TOLERANCE;
        tolerance = t;
      }

      /// Output
      inline void setDistance( H3DFloat d ) {
        update_bit_mask |= DISTANCE;
        distance = d;
      }

      //////////////////////////////////////////
      /// Get functions
      /// Input
      inline H3DFloat getMaxDistance() {
        return max_distance;
      }

      inline H3DFloat getMinDistance() {
        return min_distance;
      }

      inline H3DFloat getStiffness() {
        return stiffness;
      }

      inline H3DFloat getDamping() {
        return damping;
      }

      inline H3DFloat getTolerance() {
        return tolerance;
      }

      /// Output
      inline H3DFloat getDistance() {
        return distance;
      }
      ////////////////////////////////////////
      /// have.. functions
      /// Input
      inline bool haveMaxDistance() {
        return (update_bit_mask & MAX_DISTANCE) != 0;
      }

      inline bool haveMinDistance() {
        return (update_bit_mask & MIN_DISTANCE) != 0;
      }

      inline bool haveStiffness() {
        return (update_bit_mask & STIFFNESS) != 0;
      }

      inline bool haveDamping() {
        return (update_bit_mask & DAMPING) != 0;
      }

      inline bool haveTolerance() {
        return (update_bit_mask & TOLERANCE) != 0;
      }

      /// Output
      inline bool haveDistance() {
        return (update_bit_mask & DISTANCE) != 0;
      }

      //////////////////////////////
      /// enable functions
      inline void enableDistance() {
        update_bit_mask |= DISTANCE;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      static const unsigned int MAX_DISTANCE = 0x0001;
      static const unsigned int MIN_DISTANCE = 0x0002;
      static const unsigned int STIFFNESS    = 0x0008;
      static const unsigned int DAMPING      = 0x0010;
      static const unsigned int DISTANCE     = 0x0020;
      static const unsigned int TOLERANCE    = 0x0040;

      H3DFloat max_distance;
      H3DFloat min_distance;
      H3DFloat stiffness;
      H3DFloat damping;
      H3DFloat distance;
      H3DFloat tolerance;

    };

  }
}
#endif
