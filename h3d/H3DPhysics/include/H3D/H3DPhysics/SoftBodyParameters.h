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
/// \file SoftBodyParameters.h
/// \brief Header file for SoftBodyParameters
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SOFTBODYPARAMETERS__
#define __SOFTBODYPARAMETERS__

#include <H3D/H3DPhysics/PhysicsEngineParameters.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/MFFloat.h>
#include <H3D/MFInt32.h>

namespace H3D{

  class SoftBodyPhysicsEngineThread;
  class H3DPhysicsMaterialNode;
  class H3DDeformationStrategyNode;
  class H3DSolverNode;
  class X3DNBodyCollidableNode;
  class X3DGeometryNode;

  class H3DPhysicsDampingNode;
  class H3DPhysicsElasticityNode;
  class H3DPhysicsFrictionNode;
  class H3DPhysicsMassNode;
  class H3DPhysicsStiffnessNode;
  class H3DPhysicsPoissonRatioNode;
  class H3DSoftBodyOutputNode;

  namespace PhysicsEngineParameters {

    /// Engine parameters for a SoftBodyOutputNode
    struct H3DPHYS_API H3DSoftBodyOutputParameters : public RefCountedClass {
      typedef std::vector<H3DInt32> IndexList;

      /// The type of unit that the attributes apply to
      enum UnitType {
        UNIT_NODE,
        UNIT_EDGE,
        UNIT_ELEMENT };

      /// Constructor
      H3DSoftBodyOutputParameters () :
        node ( NULL ),
        unitType( UNIT_NODE )
      {}

      /// Set the type of unit that the attributes apply to e.g., vertex, edge, etc
      inline void setUnitType ( UnitType _unitType ) {
        unitType= _unitType;
      }

      /// Get the type of unit that the attributes apply to e.g., vertex, edge, etc
      inline UnitType getUnitType () {
        return unitType;
      }

      /// Set the list of indices to retrieve attributes for
      inline void setIndex ( const IndexList& _index ) {
        index= _index;
      }

      /// Get the list of indices to retrieve attributes for
      inline const IndexList& getIndex () {
        return index;
      }

      /// Set the SoftBodyOutputNode that these parameters are for
      inline void setNode ( H3DSoftBodyOutputNode& _node ) {
        node= &_node;
      }

      /// Get the SoftBodyOutputNode that these parameters are for
      inline H3DSoftBodyOutputNode* getNode () {
        return node;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DSoftBodyOutputParameters& src ) {};

    protected:
      /// The type of unit that the attributes apply to e.g., vertex, edge, etc
      UnitType unitType;

      /// The list of indices to retrieve attributes for
      IndexList index;

      /// The SoftBodyOutputNode that these parameters are for
      H3DSoftBodyOutputNode* node;
    };

    /// Engine parameters for a SoftBodyFloatAttribute node
    struct H3DPHYS_API SoftBodyFloatAttributeParameters : public H3DSoftBodyOutputParameters {
      typedef H3DFloat AttributeType;
      typedef std::vector<AttributeType> AttributeVectorType;

      /// The possible types of values to output (the name field of the attribute node)
      enum OutputType {
        OUTPUT_FORCE_MAGNITUDE,
        OUTPUT_INTERACTION_FORCE_MAGNITUDE,
        OUTPUT_EXTERNAL_FORCE_MAGNITUDE,
        OUTPUT_SPEED };

      /// Constructor
      SoftBodyFloatAttributeParameters () :
        outputType ( OUTPUT_FORCE_MAGNITUDE )
      {}

      /// Set the type of values to output (the name field of the attribute node)
      inline void setOutputType ( OutputType _outputType ) {
        outputType= _outputType;
      }

      /// Get the type of values to output (the name field of the attribute node)
      inline OutputType getOutputType () {
        return outputType;
      }

      /// Set attribute values for each unit
      inline void setValues ( const AttributeVectorType& _values ) {
        values= _values;
      }

      /// Get attribute values for each unit
      inline const AttributeVectorType& getValues () {
        return values;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DSoftBodyOutputParameters& src );

    protected:
      /// The type of values to output (the name field of the attribute node)
      OutputType outputType;

      /// The attribute values for each unit
      AttributeVectorType values;
    };

    /// Engine parameters for a SoftBodyVec3fAttribute node
    struct H3DPHYS_API SoftBodyVec3fAttributeParameters : public H3DSoftBodyOutputParameters {
      typedef Vec3f AttributeType;
      typedef std::vector<AttributeType> AttributeVectorType;

      /// The possible types of values to output (the name field of the attribute node)
      enum OutputType {
        OUTPUT_FORCE,
        OUTPUT_INTERACTION_FORCE,
        OUTPUT_EXTERNAL_FORCE,
        OUTPUT_VELOCITY };

      /// Constructor
      SoftBodyVec3fAttributeParameters () :
        outputType ( OUTPUT_FORCE )
      {}

      /// Set the type of values to output (the name field of the attribute node)
      inline void setOutputType ( OutputType _outputType ) {
        outputType= _outputType;
      }

      /// Get the type of values to output (the name field of the attribute node)
      inline OutputType getOutputType () {
        return outputType;
      }

      /// Set attribute values for each unit
      inline void setValues ( const AttributeVectorType& _values ) {
        values= _values;
      }

      /// Get attribute values for each unit
      inline const AttributeVectorType& getValues () {
        return values;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DSoftBodyOutputParameters& src );

    protected:
      /// The type of values to output (the name field of the attribute node)
      OutputType outputType;

      /// The attribute values for each unit
      AttributeVectorType values;
    };

    struct H3DPHYS_API MaterialPropertyParameters : public RefCountedClass {

      /// Different interpretations of the data kept in the node.
      /// UNIT_UNIFORM   : Refers to one uniform honogenous value.
      /// UNIT_NODE    : Refers to values stored per node for the body.
      /// UNIT_EDGE    : Refers to values stored per edge for the body.
      /// UNIT_ELEMENT : Refers to values stored per element for the body.
      enum UnitType {  UNIT_UNIFORM,
        UNIT_NODE,
        UNIT_EDGE,
        UNIT_ELEMENT};

      /// Constructor
      MaterialPropertyParameters() :
        update_bit_mask( 0 ),
        all_output ( 0 ),
        unitType ( UNIT_UNIFORM ) {}

      /// Destructor. Making the class a polymorphic type.
      virtual ~MaterialPropertyParameters() {}

      // 'set' functions
      inline void setEngineOptions ( EngineOptionParameters* _engineOptions ) {
        update_bit_mask |= ENGINE_OPTIONS;
        engineOptions= AutoRef<EngineOptionParameters> (_engineOptions);
      }

      inline void setUnitType ( UnitType _unitType ) {
        update_bit_mask |= UNIT_TYPE;
        unitType= _unitType;
      }

      // 'get' functions
      inline EngineOptionParameters* getEngineOptions() {
        return engineOptions.get();
      }

      inline UnitType getUnitType () {
        return unitType;
      }

      // 'have' functions
      inline bool haveEngineOptions() {
        return (update_bit_mask & ENGINE_OPTIONS) != 0;
      }

      inline bool haveUnitType () {
        return (update_bit_mask & UNIT_TYPE) != 0;
      }

      /// Bitmask for which parameters that are set.
      unsigned int update_bit_mask;

      /// Bit mask defining output parameters
      unsigned int all_output;

      /// Copy the part of the bit mask containing which output parameters are set (defined by all_output)
      inline void copyOutputFlags ( unsigned int src_update_bit_mask ) {
        update_bit_mask = (update_bit_mask & ~all_output) | (src_update_bit_mask & all_output );
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( MaterialPropertyParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( MaterialPropertyParameters& src );

    protected:
      static const unsigned int ENGINE_OPTIONS = 0x00000001;
      static const unsigned int UNIT_TYPE      = 0x00000002;

      AutoRef < EngineOptionParameters > engineOptions;

      UnitType unitType;
    };

    struct H3DPHYS_API FloatMaterialPropertyParameters :
      public MaterialPropertyParameters {
      typedef vector<H3DFloat> FloatVector;
      typedef TrackedMFieldBase<MFFloat> FieldType;
      typedef FieldType::EditVector EditVector;
      typedef FieldType::Edit Edit;

      virtual H3DFloat getValuePerUnit ( size_t _index ) {
        if ( _index < value.size() ) {
          return value[_index];
        } else {
          if ( value.size() > 0 ) {
            return value[0];
          } else {
            return 0.0f;
          }
        }
      }

      inline void setValue ( const FloatVector& _value ) {
        value= _value;
        update_bit_mask |= VALUE;
      }

      inline void setValue ( H3DFloat _value ) {
        value.clear();
        value.push_back ( _value );
        update_bit_mask |= VALUE;
      }

      /// Set list of tracked changes to the values
      inline void setChanges ( const EditVector& _edits ) {
        edits= _edits;
      }

      /// Returns list of tracked changes to the values
      inline const EditVector& getChanges () {
        return edits;
      }

      inline bool haveValue() {
        return (update_bit_mask & VALUE) != 0;
      }

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( MaterialPropertyParameters& src );

    protected:
      static const unsigned int VALUE = 0x00000004;

      FloatVector value;

      /// A list of tracked changes to the values
      EditVector edits;
    };

    struct H3DPHYS_API DampingParameters :
      public FloatMaterialPropertyParameters {

      DampingParameters() {
      }

    };

    struct H3DPHYS_API ElasticityParameters :
      public FloatMaterialPropertyParameters {

      ElasticityParameters() {}
    };

    struct H3DPHYS_API FrictionParameters :
      public FloatMaterialPropertyParameters {

      FrictionParameters() {}
    };

    struct H3DPHYS_API MassParameters :
      public FloatMaterialPropertyParameters {

      MassParameters() {}
    };

    struct H3DPHYS_API PoissonRatioParameters :
      public FloatMaterialPropertyParameters {

      PoissonRatioParameters() {}
    };

    struct H3DPHYS_API StiffnessParameters :
      public FloatMaterialPropertyParameters {

      StiffnessParameters() {}
    };


    struct H3DPHYS_API H3DPhysicsMaterialParameters  : public RefCountedClass
    {

      H3DPhysicsMaterialParameters():
        update_bit_mask( 0 ),
        all_output ( 0 ),
        mass( NULL ),
        damping( NULL ),
        friction( NULL ),
        elasticity( NULL ),
        stiffness( NULL ),
        stiffnessAngular( NULL ),
        stiffnessVolume( NULL ),
        poissonRatio( NULL ) {}

    /// Destructor. Making the class a polymorphic type.
    virtual ~H3DPhysicsMaterialParameters() {}

    ////////////////////////////////////////
    /// set.. functions
    inline void setDamping( H3DPhysicsDampingNode* _d ) {
      update_bit_mask |= DAMPING;
      damping = _d;
    }

    inline void setDampingParameters( DampingParameters* _dp ) {
      update_bit_mask |= DAMPINGPARAMETERS;
      dampingParameters = AutoRef<DampingParameters> (_dp);
    }

    inline void setMass( H3DPhysicsMassNode* _m ) {
      update_bit_mask |= MASS;
      mass = _m;
    }

    inline void setMassParameters( MassParameters* _mp ) {
      update_bit_mask |= MASSPARAMETERS;
      massParameters = AutoRef<MassParameters> (_mp);
    }

    inline void setFriction( H3DPhysicsFrictionNode* _f ) {
      update_bit_mask |= FRICTION;
      friction = _f;
    }

    inline void setFrictionParameters( FrictionParameters* _fp ) {
      update_bit_mask |= FRICTIONPARAMETERS;
      frictionParameters = AutoRef<FrictionParameters> (_fp);
    }

    inline void setElasticity( H3DPhysicsElasticityNode* _e ) {
      update_bit_mask |= ELASTICITY;
      elasticity = _e;
    }

    inline void setElasticityParameters( ElasticityParameters* _ep ) {
      update_bit_mask |= ELASTICITYPARAMETERS;
      elasticityParameters = AutoRef<ElasticityParameters> (_ep);
    }

    inline void setStiffness( H3DPhysicsStiffnessNode* _s ) {
      update_bit_mask |= STIFFNESS;
      stiffness = _s;
    }

    inline void setStiffnessParameters( StiffnessParameters* _sp ) {
      update_bit_mask |= STIFFNESSPARAMETERS;
      stiffnessParameters = AutoRef<StiffnessParameters> (_sp);
    }

    inline void setStiffnessAngular( H3DPhysicsStiffnessNode* _s ) {
      update_bit_mask |= STIFFNESSANGULAR;
      stiffnessAngular = _s;
    }

    inline void setStiffnessAngularParameters( StiffnessParameters* _sp ) {
      update_bit_mask |= STIFFNESSANGULARPARAMETERS;
      stiffnessAngularParameters = AutoRef<StiffnessParameters>( _sp );
    }

    inline void setStiffnessVolume( H3DPhysicsStiffnessNode* _s ) {
      update_bit_mask |= STIFFNESSVOLUME;
      stiffnessVolume = _s;
    }

    inline void setStiffnessVolumeParameters( StiffnessParameters* _sp ) {
      update_bit_mask |= STIFFNESSVOLUMEPARAMETERS;
      stiffnessVolumeParameters = AutoRef<StiffnessParameters>( _sp );
    }

    inline void setPoissonRatio( H3DPhysicsPoissonRatioNode* _p ) {
      update_bit_mask |= POISSONRATIO;
      poissonRatio = _p;
    }

    inline void setPoissonRatioParameters( PoissonRatioParameters* _pp ) {
      update_bit_mask |= POISSONRATIOPARAMETERS;
      poissonRatioParameters = AutoRef<PoissonRatioParameters> (_pp);
    }

    inline void setEngineOptions ( EngineOptionParameters* _engineOptions ) {
      update_bit_mask |= ENGINE_OPTIONS;
      engineOptions= AutoRef<EngineOptionParameters> (_engineOptions);
    }

    ////////////////////////////////////////
    /// get.. functions
    inline H3DPhysicsDampingNode* getDamping() {
      return damping;
    }

    inline DampingParameters* getDampingParameters() {
      return dampingParameters.get();
    }

    inline H3DPhysicsMassNode* getMass() {
      return mass;
    }

    inline MassParameters* getMassParameters() {
      return massParameters.get();
    }

    inline H3DPhysicsFrictionNode* getFriction() {
      return friction;
    }

    inline FrictionParameters* getFrictionParameters() {
      return frictionParameters.get();
    }

    inline H3DPhysicsElasticityNode* getElasticity() {
      return elasticity;
    }

    inline ElasticityParameters* getElasticityParameters() {
      return elasticityParameters.get();
    }

    inline H3DPhysicsStiffnessNode* getStiffness() {
      return stiffness;
    }

    inline StiffnessParameters* getStiffnessParameters() {
      return stiffnessParameters.get();
    }

    inline H3DPhysicsStiffnessNode* getStiffnessAngular() {
      return stiffnessAngular;
    }

    inline StiffnessParameters* getStiffnessAngularParameters() {
      return stiffnessAngularParameters.get();
    }

    inline H3DPhysicsStiffnessNode* getStiffnessVolume() {
      return stiffnessVolume;
    }

    inline StiffnessParameters* getStiffnessVolumeParameters() {
      return stiffnessVolumeParameters.get();
    }

    // 'get' functions
    /// Get the poissonRatio.
    inline H3DPhysicsPoissonRatioNode* getPoissonRatio() {
      return poissonRatio;
    }

    inline PoissonRatioParameters* getPoissonRatioParameters() {
      return poissonRatioParameters.get();
    }


    inline EngineOptionParameters* getEngineOptions() {
      return engineOptions.get();
    }

    /// Have functions
    inline bool haveMass() {
      return (update_bit_mask & MASS) != 0;
    }

    inline bool haveMassParameters() {
      return (update_bit_mask & MASSPARAMETERS) != 0;
    }

    inline bool haveDamping() {
      return (update_bit_mask & DAMPING) != 0;
    }

    inline bool haveDampingParameters() {
      return (update_bit_mask & DAMPINGPARAMETERS) != 0;
    }

    inline bool haveFriction() {
      return (update_bit_mask & FRICTION) != 0;
    }

    inline bool haveFrictionParameters() {
      return (update_bit_mask & FRICTIONPARAMETERS) != 0;
    }

    inline bool haveElasticity() {
      return (update_bit_mask & ELASTICITY) != 0;
    }

    inline bool haveElasticityParameters() {
      return (update_bit_mask & ELASTICITYPARAMETERS) != 0;
    }

    inline bool haveStiffness() {
      return (update_bit_mask & STIFFNESS) != 0;
    }

    inline bool haveStiffnessParameters() {
      return (update_bit_mask & STIFFNESSPARAMETERS) != 0;
    }

    inline bool haveStiffnessAngular() {
      return (update_bit_mask & STIFFNESSANGULAR) != 0;
    }

    inline bool haveStiffnessAngularParameters() {
      return (update_bit_mask & STIFFNESSANGULARPARAMETERS) != 0;
    }

    inline bool haveStiffnessVolume() {
      return (update_bit_mask & STIFFNESSVOLUME) != 0;
    }

    inline bool haveStiffnessVolumeParameters() {
      return (update_bit_mask & STIFFNESSVOLUMEPARAMETERS) != 0;
    }

    inline bool havePoissonRatio() {
      return (update_bit_mask & POISSONRATIO) != 0;
    }

    inline bool havePoissonRatioParameters() {
      return (update_bit_mask & POISSONRATIOPARAMETERS) != 0;
    }

    inline bool haveEngineOptions() {
      return (update_bit_mask & ENGINE_OPTIONS) != 0;
    }

    /// Bitmask for which parameters that are set.
    unsigned int update_bit_mask;

    /// Bit mask defining output parameters
    unsigned int all_output;

    /// Copy the part of the bit mask containing which output parameters are set (defined by all_output)
    inline void copyOutputFlags ( unsigned int src_update_bit_mask )
    {
      update_bit_mask = (update_bit_mask & ~all_output) | (src_update_bit_mask & all_output );
    }

    /// Copy the output parameters from the src parameters to this.
    virtual void copyOutputParameters( H3DPhysicsMaterialParameters& src );

    /// Copy the input parameters from the src parameters to this.
    virtual void copyInputParameters( H3DPhysicsMaterialParameters& src );

    protected:

      H3DPhysicsMassNode *mass;
      H3DPhysicsDampingNode *damping;
      H3DPhysicsFrictionNode *friction;
      H3DPhysicsElasticityNode* elasticity;
      H3DPhysicsStiffnessNode* stiffness;
      H3DPhysicsStiffnessNode* stiffnessAngular;
      H3DPhysicsStiffnessNode* stiffnessVolume;
      H3DPhysicsPoissonRatioNode *poissonRatio;

      static const unsigned int MASS                   = 0x01000000;
      static const unsigned int DAMPING                = 0x02000000;
      static const unsigned int FRICTION               = 0x04000000;
      static const unsigned int MASSPARAMETERS         = 0x08000000;
      static const unsigned int DAMPINGPARAMETERS      = 0x10000000;
      static const unsigned int FRICTIONPARAMETERS     = 0x20000000;
      static const unsigned int ELASTICITY             = 0x00000001;
      static const unsigned int ELASTICITYPARAMETERS   = 0x00000002;
      static const unsigned int STIFFNESS              = 0x00000004;
      static const unsigned int STIFFNESSPARAMETERS    = 0x00000008;
      static const unsigned int POISSONRATIO           = 0x00000010;
      static const unsigned int POISSONRATIOPARAMETERS = 0x00000020;
      static const unsigned int ENGINE_OPTIONS         = 0x00000040;
      static const unsigned int STIFFNESSANGULAR              = 0x00000080;
      static const unsigned int STIFFNESSANGULARPARAMETERS    = 0x00000100;
      static const unsigned int STIFFNESSVOLUME               = 0x00000200;
      static const unsigned int STIFFNESSVOLUMEPARAMETERS     = 0x00000400;


      AutoRef < EngineOptionParameters > engineOptions;
      AutoRef < MassParameters         > massParameters;
      AutoRef < DampingParameters      > dampingParameters;
      AutoRef < FrictionParameters     > frictionParameters;
      AutoRef < ElasticityParameters   > elasticityParameters;
      AutoRef < StiffnessParameters    > stiffnessParameters;
      AutoRef < StiffnessParameters    > stiffnessAngularParameters;
      AutoRef < StiffnessParameters    > stiffnessVolumeParameters;
      AutoRef < PoissonRatioParameters > poissonRatioParameters;

    };


    struct H3DPHYS_API DeformationStrategyParameters  : public RefCountedClass {

      DeformationStrategyParameters() :
    update_bit_mask( 0 ),
      all_output ( 0 ),
      h3d_solver( NULL ) {}

    /// Destructor. Making the class a polymorphic type.
    virtual ~DeformationStrategyParameters() {}

    ////////////////////////////////////////
    /// set.. functions
    inline void setSolverType( H3DSolverNode* st ) {
      update_bit_mask |= H3DSOLVER;
      h3d_solver = st;
    }

    inline void setEngineOptions ( EngineOptionParameters* _engineOptions ) {
      update_bit_mask |= ENGINE_OPTIONS;
      engineOptions= AutoRef<EngineOptionParameters> (_engineOptions);
    }

    ////////////////////////////////////////
    /// get.. functions
    inline H3DSolverNode* getSolverType() {
      return h3d_solver;
    }

    inline EngineOptionParameters* getEngineOptions() {
      return engineOptions.get();
    }

    /// Have functions
    inline bool haveSolverType() {
      return (update_bit_mask & H3DSOLVER) != 0;
    }

    inline bool haveEngineOptions() {
      return (update_bit_mask & ENGINE_OPTIONS) != 0;
    }

    /// Bitmask for which parameters that are set.
    unsigned int update_bit_mask;

    unsigned int all_output;

    /// Copy the part of the bit mask containing which output parameters are set (defined by all_output)
    inline void copyOutputFlags ( unsigned int src_update_bit_mask )
    {
      update_bit_mask = (update_bit_mask & ~all_output) | (src_update_bit_mask & all_output );
    }

    /// Copy the output parameters from the src parameters to this.
    virtual void copyOutputParameters( DeformationStrategyParameters& src );

    /// Copy the input parameters from the src parameters to this.
    virtual void copyInputParameters( DeformationStrategyParameters& src );

    protected:

      H3DSolverNode* h3d_solver;

      static const unsigned int H3DSOLVER         = 0x10000000;
      static const unsigned int ENGINE_OPTIONS    = 0x20000000;

      AutoRef < EngineOptionParameters > engineOptions;

    };

    /// \ingroup SoftBody
    /// Parameters for a soft body
    struct H3DPHYS_API H3DSoftBodyNodeParameters {
      typedef vector<H3DInt32> IndexList;
      typedef vector<Vec3f> CoordList;
      typedef vector < X3DGeometryNode*         > X3DGeometryNodeList;
      typedef vector < X3DNBodyCollidableNode*  > X3DNBodyCollidableNodeList;
      typedef vector<H3DFloat> StiffnessList;
      typedef AutoRefVector<SoftBodyFloatAttributeParameters> FloatOutputList;
      typedef AutoRefVector<SoftBodyVec3fAttributeParameters> Vec3fOutputList;

      /// Represents a force on a vertex
      class VertexForce {
      public:
        VertexForce ( size_t _index, Vec3f _force, H3DModifierId _sourceId = -1 ) :
            index ( _index ), force ( _force ), sourceId( _sourceId ) {
            };
            size_t index;
            Vec3f force;
            H3DModifierId sourceId;

            inline H3DModifierId getSourceId() {
              return sourceId;
            }

            inline void setSourceId( H3DModifierId _sId ) {
              sourceId = _sId;
            }
      };
      //typedef vector<VertexForce> VertexForceList;
      typedef multimap<H3DModifierId, VertexForce> VertexForceList;

      /// Constructor
      H3DSoftBodyNodeParameters();

      /// Called from physics thread just after data has been copied from the physics engine to the struct.
      /// The function is called while the physics thread has locked soft_body_lock.
      ///
      /// An override point to allow additional data to be copied in and out of the physics thread
      /// synchronously with other data relating to the soft body
      ///
      virtual void onGetParametersPhysicsThread ();

      /// Called from the graphics thread just after data has been copied from the struct to the graphics thread.
      /// The function is called while the physics thread has locked soft_body_lock.
      ///
      /// An override point to allow additional data to be copied in and out of the physics thread
      /// synchronously with other data relating to the soft body
      ///
      virtual void onGetParametersGraphicsThread () {}

      // 'set' functions

      /// Set the physics engine thread this body is associated with
      inline void setEngine ( SoftBodyPhysicsEngineThread& _engine ) {
        engine= &_engine;
      }

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      /// Set the name for the soft body
      inline void setName( const std::string& _name ) {
        name = _name;
      }
#endif

      /// Set the body ID for the soft body
      inline void setBodyId ( H3DBodyId _bodyId ) {
        bodyId= _bodyId;
      }

      /// Set the transform of the soft body
      inline void setTransform ( const Matrix4f& _transform ) {
        update_bit_mask|= TRANSFORM;
        transform= _transform;
      }

      /// Set the physicsMaterial representing the physical properties of the soft body
      inline void setPhysicsMaterial( H3DPhysicsMaterialNode* _material ) {
        update_bit_mask|= MATERIAL;
        physicsMaterial= _material;
      }

      /// Set the physicsMaterialParameters of the physicsMaterial representing the
      /// physical properties of the soft body/
      inline void setH3DPhysicsMaterialParameters( PhysicsEngineParameters::H3DPhysicsMaterialParameters* _materialParam ) {
        update_bit_mask|= MATERIALPARAMETERS;
        physicsMaterialParameters= AutoRef<H3DPhysicsMaterialParameters> (_materialParam);
      }

      /// Set the deformationStrategy representing deforming the soft body.
      inline void setDeformationStrategy( H3DDeformationStrategyNode* _strategy ) {
        update_bit_mask|= DEFORMATIONSTRATEGY;
        deformationStrategy= _strategy;
      }

      /// Set the deformationStrategyParameters.
      inline void setDeformationStrategyParameters( PhysicsEngineParameters::DeformationStrategyParameters* _defParam ) {
        update_bit_mask|= DEFORMATIONSTRATEGYPARAMETERS;
        deformationStrategyParameters= AutoRef<DeformationStrategyParameters> (_defParam);
      }

      /// Set the geometry representing the structure of the soft body
      inline void setGeometry( X3DGeometryNode& _geometry ) {
        update_bit_mask|= GEOMETRY;
        geometry= &_geometry;
      }

      /// Set the geometry representing the surface of the soft body
      inline void setSurfaceGeometry( X3DGeometryNodeList& _geometry ) {
        update_bit_mask |= SURFACE_GEOMETRY;
        surfaceGeometry = _geometry;
      }

      /// Set the geometry representing the structure of the soft body
      inline void setCollisionGeometry( X3DNBodyCollidableNodeList& _geometry ) {
        update_bit_mask|= COLLISION_GEOMETRY;
        collisionGeometry= _geometry;
      }

      /// Set the coordinates representing the structure of the soft body
      inline void setCoords( const CoordList& _coords ) {
        update_bit_mask|= COORDS;
        coords= _coords;
      }

      /// Set the indices representing the structure of the soft body
      inline void setIndices( const IndexList& _indices ) {
        update_bit_mask|= INDICES;
        indices= _indices;
      }

      /// Add a force on a vertex of the soft body
      inline void addVertexForce ( VertexForce force ) {
        update_bit_mask|= VERTEX_FORCE;
        vertexForces.insert( pair<H3DModifierId, VertexForce>
          ( force.getSourceId(), force) );
        //vertexForces.push_back ( force );
      }

      /// Remove all forces on vertices of the soft body
      inline void clearVertexForces () {
        update_bit_mask|= VERTEX_FORCE;
        vertexForces.clear();
      }

      /// Remove forces added by a specific modifier
      inline void clearVertexForces ( H3DModifierId _mid ) {
        update_bit_mask|= VERTEX_FORCE;
        vertexForces.erase( _mid ) ;
      }

      /// Add a manipulation force on a vertex of the soft body
      inline void addManipulationForce ( VertexForce force ) {
        update_bit_mask|= MANIPULATION_FORCES;
        manipulationForces.insert( pair<H3DModifierId, VertexForce>
          ( force.getSourceId(), force) );
        //manipulationForces.push_back ( force );
      }

      /// Remove all manipulation forces on vertices of the soft body
      inline void clearManipulationForces () {
        update_bit_mask|= MANIPULATION_FORCES;
        manipulationForces.clear();
      }

      /// Removes manipulation forces added by a specific modifier
      inline void clearManipulationForces ( H3DModifierId _mid ) {
        update_bit_mask|= MANIPULATION_FORCES;
        manipulationForces.erase( _mid ) ;
      }

      /// Set list of float output parameters
      inline void setOutputsFloat ( const FloatOutputList& _outputs ) {
        update_bit_mask |= OUTPUTS_FLOAT;
        outputsFloat= _outputs;
      }

      /// Set list of Vec3f output parameters
      inline void setOutputsVec3f ( const Vec3fOutputList& _outputs ) {
        update_bit_mask |= OUTPUTS_VEC3F;
        outputsVec3f= _outputs;
      }

      inline void setEngineOptions ( EngineOptionParameters* _engineOptions ) {
        update_bit_mask |= ENGINE_OPTIONS;
        engineOptions= AutoRef<EngineOptionParameters> (_engineOptions);
      }

      // 'get' functions

      /// Get the physics engine thread this body is associated with
      inline SoftBodyPhysicsEngineThread* getEngine () {
        return engine;
      }

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      /// Get the name for the soft body
      inline const std::string& getName() const {
        return name;
      }
#endif

      /// Get the body ID for the soft body
      inline H3DBodyId getBodyId () {
        return bodyId;
      }

      /// Get the position of the soft body
      inline const Matrix4f& getTransform () {
        return transform;
      }

      /// Get the physicsMaterial representing the physical properties of the soft body
      inline H3DPhysicsMaterialNode* getPhysicsMaterial() {
        return physicsMaterial;
      }

      /// Get the physicsMaterialParameters representing the physical properties of the soft body
      inline PhysicsEngineParameters::H3DPhysicsMaterialParameters* getH3DPhysicsMaterialParameters() {
        return physicsMaterialParameters.get();
      }

      /// Get the deformationStrategy deforming the soft body.
      inline H3DDeformationStrategyNode* getDeformationStrategy() {
        return deformationStrategy;
      }

      /// Get the DeformationStrategyParameters.
      inline PhysicsEngineParameters::DeformationStrategyParameters* getDeformationStrategyParameters() {
        return deformationStrategyParameters.get();
      }

      /// Get the geometry representing the structure of the soft body
      inline X3DGeometryNode* getGeometry() {
        return geometry;
      }

      /// Get the geometry representing the surface of the soft body
      inline X3DGeometryNodeList& getSurfaceGeometry () {
        return surfaceGeometry;
      }

      /// Get the geometry representing the collision structure of the soft body
      inline X3DNBodyCollidableNodeList& getCollisionGeometry() {
        return collisionGeometry;
      }

      /// Get the coordinates representing the structure of the soft body
      inline const CoordList& getCoords() {
        return coords;
      }

      /// Get the indices representing the structure of the soft body
      inline const IndexList& getIndices() {
        return indices;
      }

      /// Get list of user defined vertex forces
      inline const VertexForceList& getVertexForces() {
        return vertexForces;
      }

      /// Get list of manipulation forces
      inline const VertexForceList& getManipulationForces() {
        return manipulationForces;
      }

      /// Get list of float output parameters
      inline const FloatOutputList& getOutputsFloat() {
        return outputsFloat;
      }

      /// Get list of float output parameters
      inline const Vec3fOutputList& getOutputsVec3f() {
        return outputsVec3f;
      }

      inline EngineOptionParameters* getEngineOptions() {
        return engineOptions.get();
      }

      // 'have' functions

      /// Has the position of the soft body been specified?
      inline bool haveTransform () {
        return (update_bit_mask & TRANSFORM) != 0;
      }

      /// Has the physical properties of the soft body been specified?
      inline bool havePhysicsMaterial() {
        return (update_bit_mask & MATERIAL) != 0;
      }

      /// Has the physical material parameters of the soft body been specified?
      inline bool haveH3DPhysicsMaterialParameters() {
        return (update_bit_mask & MATERIALPARAMETERS) != 0;
      }

      /// Has the deformation strategy of the soft body been specified?
      inline bool haveDeformationStrategy() {
        return (update_bit_mask & DEFORMATIONSTRATEGY) != 0;
      }

      /// Has the deformation strategy parameters of the soft body been specified?
      inline bool haveDeformationStrategyParameters() {
        return (update_bit_mask & DEFORMATIONSTRATEGYPARAMETERS) != 0;
      }

      /// Has the geometry representing the structure of the soft body been specified?
      inline bool haveGeometry() {
        return (update_bit_mask & GEOMETRY) != 0;
      }

      /// Has the geometry representing the surface of the soft body been specified?
      inline bool haveSurfaceGeometry() {
        return (update_bit_mask & SURFACE_GEOMETRY) != 0;
      }

      /// Has the geometry representing the collision structure of the soft body been specified?
      inline bool haveCollisionGeometry() {
        return (update_bit_mask & COLLISION_GEOMETRY) != 0;
      }

      /// Have the coordinates representing the structure of the soft body been specified?
      inline bool haveCoords() {
        return (update_bit_mask & COORDS) != 0;
      }

      /// Have the indices representing the structure of the soft body been specified?
      inline bool haveIndices() {
        return (update_bit_mask & INDICES) != 0;
      }

      /// Have vertex forces been specified
      inline bool haveVertexForce () {
        return (update_bit_mask & VERTEX_FORCE) != 0;
      }

      /// Have manipulation forces been specified
      inline bool haveManipulationForces() {
        return (update_bit_mask & MANIPULATION_FORCES) != 0;
      }

      /// Has a list of float parameter outputs been specified
      inline bool haveOutputsFloat() {
        return (update_bit_mask & OUTPUTS_FLOAT) != 0;
      }

      /// Has a list of Vec3f parameter outputs been specified
      inline bool haveOutputsVec3f() {
        return (update_bit_mask & OUTPUTS_VEC3F) != 0;
      }

      inline bool haveEngineOptions() {
        return (update_bit_mask & ENGINE_OPTIONS) != 0;
      }

      /// Copy the part of the bit mask containing which output parameters are set (defined by all_output)
      inline void copyOutputFlags ( unsigned int src_update_bit_mask ) {
        update_bit_mask = (update_bit_mask & ~all_output) | (src_update_bit_mask & all_output );
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DSoftBodyNodeParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( H3DSoftBodyNodeParameters& src );

    protected:

      struct GetForce {
        Vec3f operator () ( const VertexForce& vf ) { return vf.force; }
      };

      struct GetForceMag {
        H3DFloat operator () ( const VertexForce& vf ) { return vf.force.length(); }
      };

      /// Update attribute outputs that are independent of physics engine
      void updateOutputs ();

      /// Update float attribute outputs that are independent of physics engine
      void updateOutputsFloat ();

      /// Update Vec3f attribute outputs that are independent of physics engine
      void updateOutputsVec3f ();

      /// Update the manipulation forces output
      void updateOutputManipulationForce ( SoftBodyVec3fAttributeParameters& output );

      /// Update the external forces output
      void updateOutputExternalForce ( SoftBodyVec3fAttributeParameters& output );

      /// Update the manipulation force magnitude output
      void updateOutputManipulationForceMagnitude ( SoftBodyFloatAttributeParameters& output );

      /// Update the external force magnitude output
      void updateOutputExternalForceMagnitude ( SoftBodyFloatAttributeParameters& output );

      /// Helper template function for getting attributes from the soft body
      template < typename PropertyFunctor, typename AttributeParametersType >
      void setOutputAttribute (
        AttributeParametersType& output,
        PropertyFunctor propertyAccessor,
        typename AttributeParametersType::AttributeVectorType& valueCache,
        const VertexForceList& values ) {
        const typename AttributeParametersType::IndexList& index= output.getIndex();

        // Get all attributes (once only for all outputs of this type)
        if ( valueCache.empty() ) {
          valueCache.resize ( coords.size(), typename AttributeParametersType::AttributeType() );
          for ( VertexForceList::const_iterator i= values.begin(); i != values.end(); ++i ) {
            VertexForce vf= (*i).second;
            if ( /*vf.index >= 0 && */ vf.index < valueCache.size() ) {
              valueCache[vf.index]+= propertyAccessor ( vf );
            }
          }
        }

        if ( !index.empty() ) {
          // Pick only attributes requested
          typename AttributeParametersType::AttributeVectorType indexed_attributes (
      index.size(), typename AttributeParametersType::AttributeType() );
          for ( size_t i= 0; i < index.size(); ++i ) {
            size_t j= index[i];
            if ( /*j >= 0 && */j < valueCache.size() ) {
              indexed_attributes[i]= valueCache[j];
            }
          }
          output.setValues ( indexed_attributes );
        } else {
          // Use all attributes
          output.setValues ( valueCache );
        }
      }

      static const unsigned int GEOMETRY                      = 0x0000001;
      static const unsigned int SURFACE_GEOMETRY              = 0x0000002;
      static const unsigned int COLLISION_GEOMETRY            = 0x0000004;
      static const unsigned int COORDS                        = 0x0000008;
      static const unsigned int INDICES                       = 0x0000010;
      static const unsigned int VERTEX_FORCE                  = 0x0000020;
      static const unsigned int MANIPULATION_FORCES           = 0x0000040;
      static const unsigned int TRANSFORM                     = 0x0000080;
      static const unsigned int MATERIAL                      = 0x0000100;
      static const unsigned int MATERIALPARAMETERS            = 0x0000200;
      static const unsigned int DEFORMATIONSTRATEGY           = 0x0000400;
      static const unsigned int DEFORMATIONSTRATEGYPARAMETERS = 0x0000800;
      static const unsigned int ENGINE_OPTIONS                = 0x0001000;
      static const unsigned int EDGE_STIFFNESS                = 0x0400000;
      static const unsigned int OUTPUTS_FLOAT                 = 0x0800000;
      static const unsigned int OUTPUTS_VEC3F                 = 0x1000000;

      /// Bitmask for which parameters that are set.
      unsigned int update_bit_mask;

      /// The physics engine using this body
      SoftBodyPhysicsEngineThread* engine;

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
      /// The name of the body in the simulation
      std::string name;
#endif

      /// The ID of the body in the simulation
      H3DBodyId bodyId;

      /// The geometry used to define the soft body structure
      X3DGeometryNode* geometry;

      /// The body representing the surface of the soft body
      X3DGeometryNodeList surfaceGeometry;

      /// The geometry used to define the soft body collision structure
      X3DNBodyCollidableNodeList collisionGeometry;

      /// The physics material to define the soft body collision structure
      H3DPhysicsMaterialNode* physicsMaterial;

      /// The deformationStrategy deforming the soft body.
      H3DDeformationStrategyNode* deformationStrategy;

      /// Coordinates of the structure of the soft body
      CoordList coords;

      /// Indices of the structure of the soft body
      /// How these are interpreted depends of the type of soft body
      /// e.g. Sets of triangles (Cloth) or tetrahedra (SoftBody)
      IndexList indices;

      /// List of user defined forces to apply to vertices
      VertexForceList vertexForces;

      /// List of manipulation forces to apply to vertices
      /// Cleared each physics loop
      VertexForceList manipulationForces;

      /// The initial transform of the soft body
      Matrix4f transform;

      /// The PhysicsMaterialParameters stored in the physicsMaterial.
      AutoRef < H3DPhysicsMaterialParameters > physicsMaterialParameters;

      /// The deformationStrategyParameters stored in the deformationStrategy.
      AutoRef < DeformationStrategyParameters > deformationStrategyParameters;

      /// List of float output parameters
      FloatOutputList outputsFloat;

      /// List of Vec3f output parameters
      Vec3fOutputList outputsVec3f;

      /// List of engine specific options
      AutoRef < EngineOptionParameters > engineOptions;

      /// Flag denoting which parameters should be outputted.
      unsigned int all_output;

      /// All manipulation forces per node, cached here to save time when
      /// multiple outputs require information from it
      SoftBodyVec3fAttributeParameters::AttributeVectorType manipulationForceNodeAttributes;

      /// All external forces per node, cached here to save time when
      /// multiple outputs require information from it
      SoftBodyVec3fAttributeParameters::AttributeVectorType externalForceNodeAttributes;

      /// All manipulation force magnitudes per node, cached here to save time when
      /// multiple outputs require information from it
      SoftBodyFloatAttributeParameters::AttributeVectorType manipulationForceMagNodeAttributes;

      /// All external force magnitudes per node, cached here to save time when
      /// multiple outputs require information from it
      SoftBodyFloatAttributeParameters::AttributeVectorType externalForceMagNodeAttributes;
    };

    /// \ingroup SoftBody
    /// Physics engine parameters for a SoftBody node
    struct H3DPHYS_API SoftBodyParameters : public H3DSoftBodyNodeParameters {
    };

    /// \ingroup SoftBody
    /// Physics engine parameters for a Cloth node
    struct H3DPHYS_API ClothParameters : public H3DSoftBodyNodeParameters {
    };

    /// \ingroup SoftBody
    /// Physics engine parameters for a Cloth node
    struct H3DPHYS_API RopeParameters : public H3DSoftBodyNodeParameters {
    };


    /// VertexBodyConstraint parameters
    struct H3DPHYS_API VertexBodyConstraintParameters : public ConstraintParameters {
      typedef vector<H3DInt32> IndexList;

      VertexBodyConstraintParameters();

      /// Set functions

      /// Set indices of coordinates that will be attached
      inline void setIndex ( const IndexList& _index ) {
        update_bit_mask|= INDEX;
        index= _index;
      }

      /// Get functions

      /// Get indices of coordinates that will be attached
      inline const IndexList& getIndex () {
        return index;
      }

      /// Have functions

      /// Has indices for vertex body constraint been specified?
      inline bool haveIndex() {
        return (update_bit_mask & INDEX) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      static const unsigned int INDEX   = 0x40000000;

      IndexList index;

    };


    struct H3DPHYS_API FixedConstraintParameters :
      public VertexBodyConstraintParameters {

        FixedConstraintParameters() {};

    };

    /// \ingroup SoftBody
    /// Parameters for an attachment between a soft body and another body
    struct H3DPHYS_API H3DAttachmentParameters : public ConstraintParameters {
      typedef std::vector < H3DInt32 > IndexList;
      typedef TrackedMFieldBase < MFInt32 > FieldType;
      typedef FieldType::EditVector EditVector;
      typedef FieldType::Edit Edit;

      /// Constructor
      H3DAttachmentParameters ();

      // 'set' functions

      /// Set body1
      inline void setBody2 ( H3DBodyId _body2 ) {
        update_bit_mask|= BODY2;
        body2= _body2;
      }

      /// Set indices of coordinates that will be attached
      inline void setIndex ( const IndexList& _index ) {
        update_bit_mask|= INDEX;
        index= _index;
      }

      /// Set a list of tracked changes to the index values
      inline void setIndexChanges ( const EditVector& _changes ) {
        indexChanges= _changes;
      }

      // 'get' functions
      /// Get body1
      inline H3DBodyId getBody2() {
        return body2;
      }

      /// Get indices of coordinates that will be attached
      inline const IndexList& getIndex () {
        return index;
      }

      /// Get a list of tracked changes to the index values
      inline const EditVector& getIndexChanges () {
        return indexChanges;
      }

      // 'have' functions

      /// Has body2 been specified?
      inline bool haveBody2() {
        return (update_bit_mask & BODY2) != 0;
      }

      /// Has indices for attachment been specified?
      inline bool haveIndex() {
        return (update_bit_mask & INDEX) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:

      static const unsigned int BODY2          = 0x00001;
      static const unsigned int INDEX          = 0x00002;

      /// The id of the attached soft body
      H3DBodyId body2;

      /// Indicies that will be attached
      IndexList index;

      /// A list of tracked changes to the index values
      EditVector indexChanges;
    };

    /// \ingroup SoftBody
    /// Parameters for an attachment between a soft body and a rigid body
    struct H3DPHYS_API H3DRigidBodyAttachmentParameters : public H3DAttachmentParameters {
    public:

      /// Constructor
      H3DRigidBodyAttachmentParameters ();

    };

    /// \ingroup SoftBody
    /// Parameters for an attachment between a soft body and another soft body
    struct H3DPHYS_API H3DSoftBodyAttachmentParameters : public H3DAttachmentParameters {
    public:

      /// Constructor
      H3DSoftBodyAttachmentParameters ();

      /// Set indices of coordinates that will be attached
      inline void setIndex2 ( const IndexList& _index2 ) {
        update_bit_mask|= INDEX2;
        index2= _index2;
      }

      /// Set a list of tracked changes to the index2 values
      inline void setIndex2Changes ( const EditVector& _changes ) {
        index2Changes= _changes;
      }

      /// Get indices of coordinates that will be attached
      inline const IndexList& getIndex2 () {
        return index2;
      }

      /// Get a list of tracked changes to the index2 values
      inline const EditVector& getIndex2Changes () {
        return index2Changes;
      }

      /// Has indices for attachment been specified?
      inline bool haveIndex2() {
        return (update_bit_mask & INDEX2) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:

      static const unsigned int INDEX2          = 0x00008;

      /// Indicies that will be attached
      IndexList index2;

      /// A list of tracked changes to the index2 values
      EditVector index2Changes;
    };

    /// \ingroup SoftBody
    /// Parameters for an attachment between two soft bodies
    struct H3DPHYS_API SoftBodyAttachmentParameters : public H3DSoftBodyAttachmentParameters {
    public:

      /// Constructor
      SoftBodyAttachmentParameters ();

      // 'set' functions

      /// Set the physicsMaterial representing the physical properties of the soft body
      inline void setPhysicsMaterial( H3DPhysicsMaterialNode* _material ) {
        update_bit_mask|= MATERIAL;
        physicsMaterial= _material;
      }

      /// Set the physicsMaterialParameters of the physicsMaterial representing the
      /// physical properties of the soft body attachment
      inline void setH3DPhysicsMaterialParameters( PhysicsEngineParameters::H3DPhysicsMaterialParameters* _materialParam ) {
        update_bit_mask|= MATERIALPARAMETERS;
        physicsMaterialParameters= AutoRef<H3DPhysicsMaterialParameters> (_materialParam);
      }

      // 'get' functions

      /// Get the physicsMaterial representing the physical properties of the soft body
      inline H3DPhysicsMaterialNode* getPhysicsMaterial() {
        return physicsMaterial;
      }

      /// Get the physicsMaterialParameters representing the physical properties of the soft body
      inline PhysicsEngineParameters::H3DPhysicsMaterialParameters* getH3DPhysicsMaterialParameters() {
        return physicsMaterialParameters.get();
      }

      // 'have' functions

      /// Has the physical properties of the soft body been specified?
      inline bool havePhysicsMaterial() {
        return (update_bit_mask & MATERIAL) != 0;
      }

      /// Has the physical material parameters of the soft body been specified?
      inline bool haveH3DPhysicsMaterialParameters() {
        return (update_bit_mask & MATERIALPARAMETERS) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DAttachmentParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( H3DAttachmentParameters& src );

    protected:

      static const unsigned int MATERIAL                 = 0x00010;
      static const unsigned int MATERIALPARAMETERS       = 0x00020;

      /// The physics material to use for the attachment
      H3DPhysicsMaterialNode* physicsMaterial;

      /// The material parameters for the attachment
      AutoRef < H3DPhysicsMaterialParameters > physicsMaterialParameters;
    };

    /// \ingroup SoftBody
    /// Physics engine parameters for a SoftBodyLinearJoint
    struct H3DPHYS_API SoftBodyLinearJointParameters : public JointParameters {

      /// Constructor
      SoftBodyLinearJointParameters ();

      // 'set' functions

      inline void setAnchorPoint( const Vec3f &a ) {
        update_bit_mask |= ANCHOR_POINT;
        anchor_point = a;
      }

      // 'get' functions

      inline const Vec3f & getAnchorPoint() {
        return anchor_point;
      }

      // 'have' functions

      inline bool haveAnchorPoint() {
        return (update_bit_mask & ANCHOR_POINT) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      static const unsigned int ANCHOR_POINT = 0x0001;

      Vec3f anchor_point;
    };

    /// \ingroup SoftBody
    /// Physics engine parameters for a SoftBodyAngularJoint
    struct H3DPHYS_API SoftBodyAngularJointParameters : public JointParameters {

      /// Constructor
      SoftBodyAngularJointParameters ();

      // 'set' functions

      /// Set the axis of the joint constraint
      inline void setAxis( const Vec3f &a ) {
        update_bit_mask |= AXIS;
        axis = a;
      }

      // 'get' functions

      /// Get the axis of the joint constraint
      inline const Vec3f & getAxis() {
        return axis;
      }

      // 'have' functions

      /// Has the axis of the joint constraint been specified?
      inline bool haveAxis() {
        return (update_bit_mask & AXIS) != 0;
      }

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( ConstraintParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( ConstraintParameters& src );

    protected:
      static const unsigned int AXIS = 0x0001;

      /// The axis of the joint constraint
      Vec3f axis;
    };

    struct H3DPHYS_API ModifierParameters
    {
      /// Constructor
      ModifierParameters() :
    engine_thread( 0 ),
      modifier_id( 0 ),
      update_bit_mask( 0 ),
      all_output ( 0 ),
      body1( 0 ),
      device_index( -1 ){}

    /// Destructor. Making the class a polymorphic type.
    virtual ~ModifierParameters() {}

    // 'set' functions
    inline void setBody1 ( H3DBodyId  _body1 ) {
      update_bit_mask |= BODY1;
      body1= _body1;
    }

    inline void setDeviceIndex ( int  _deviceindex ) {
      device_index= _deviceindex;
    }

    inline void setModifierId ( H3DModifierId _modifierId ) {
      modifier_id= _modifierId;
    }

    inline void setEngine ( PhysicsEngineThread& _engine ) {
      engine_thread= &_engine;
    }

    inline void setEngineOptions ( EngineOptionParameters* _engineOptions ) {
      update_bit_mask |= ENGINE_OPTIONS;
      engineOptions= AutoRef<EngineOptionParameters> (_engineOptions);
    }

    // 'get' functions
    inline H3DBodyId getBody1() {
      return body1;
    }

    inline H3DConstraintId getModifierId () {
      return modifier_id;
    }

    inline int getDeviceIndex () {
      return device_index;
    }

    inline PhysicsEngineThread* getEngine () {
      return engine_thread;
    }

    inline EngineOptionParameters* getEngineOptions() {
      return engineOptions.get();
    }

    // 'have' functions
    inline bool haveBody1() {
      return (update_bit_mask & BODY1) != 0;
    }

    inline bool haveEngineOptions() {
      return (update_bit_mask & ENGINE_OPTIONS) != 0;
    }

    /// Bitmask for which parameters that are set.
    unsigned int update_bit_mask;

    /// Bit mask defining output parameters
    unsigned int all_output;

    /// Copy the part of the bit mask containing which output parameters are set (defined by all_output)
    inline void copyOutputFlags ( unsigned int src_update_bit_mask )
    {
      update_bit_mask = (update_bit_mask & ~all_output) | (src_update_bit_mask & all_output );
    }

    /// Copy the output parameters from the src parameters to this.
    virtual void copyOutputParameters( ModifierParameters& src );

    /// Copy the input parameters from the src parameters to this.
    virtual void copyInputParameters( ModifierParameters& src );

    protected:

      static const unsigned int BODY1             = 0x1000;
      static const unsigned int ENGINE_OPTIONS    = 0x2000;

      H3D::PhysicsEngineThread *engine_thread;
      H3DModifierId modifier_id;
      H3DBodyId body1;
      int device_index;

      AutoRef < EngineOptionParameters > engineOptions;
    };


    struct H3DPHYS_API FEMDeformationStrategyParameters :
      public DeformationStrategyParameters {

        FEMDeformationStrategyParameters() {}

    };

    struct H3DPHYS_API MassSpringDeformationStrategyParameters :
      public DeformationStrategyParameters {

        MassSpringDeformationStrategyParameters() {}

    };


    struct H3DPHYS_API PhysicsMaterialParameters : public H3DPhysicsMaterialParameters {
      /// Constructor
      PhysicsMaterialParameters ();

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DPhysicsMaterialParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( H3DPhysicsMaterialParameters& src );

    };

    struct H3DPHYS_API MassSpringPhysicsMaterialParameters : public H3DPhysicsMaterialParameters {
      /// Constructor
      MassSpringPhysicsMaterialParameters();

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DPhysicsMaterialParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( H3DPhysicsMaterialParameters& src );
    };

    struct H3DPHYS_API FEMPhysicsMaterialParameters : public PhysicsMaterialParameters {
      /// Constructor
      FEMPhysicsMaterialParameters();

      /// Copy the output parameters from the src parameters to this.
      virtual void copyOutputParameters( H3DPhysicsMaterialParameters& src );

      /// Copy the input parameters from the src parameters to this.
      virtual void copyInputParameters( H3DPhysicsMaterialParameters& src );

    };

  }

}

#endif
