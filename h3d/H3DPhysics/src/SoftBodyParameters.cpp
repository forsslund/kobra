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
/// \file SoftBodyParameters.cpp
/// \brief Source file for SoftBodyParameters
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/SoftBodyParameters.h>
#include <H3D/H3DPhysics/H3DPhysicsMaterialNode.h>
#include <H3D/H3DPhysics/H3DDeformationStrategyNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>

#include <H3D/H3DHapticsDevice.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

void SoftBodyFloatAttributeParameters::copyOutputParameters( H3DSoftBodyOutputParameters& src ) {
  SoftBodyFloatAttributeParameters* p= dynamic_cast<SoftBodyFloatAttributeParameters*>(&src);
  if ( p ) {
    values= p->values;
  }
}

void SoftBodyVec3fAttributeParameters::copyOutputParameters( H3DSoftBodyOutputParameters& src ) {
  SoftBodyVec3fAttributeParameters* p= dynamic_cast<SoftBodyVec3fAttributeParameters*>(&src);
  if ( p ) {
    values= p->values;
  }
}

H3DSoftBodyNodeParameters::H3DSoftBodyNodeParameters()
: update_bit_mask ( 0 ),
all_output( 0 ),
engine ( NULL ),
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
name(""),
#endif
bodyId ( 0 ),
physicsMaterial( NULL ),
deformationStrategy( NULL ),
geometry ( NULL )
{
}

void H3DSoftBodyNodeParameters::onGetParametersPhysicsThread () {
  updateOutputs ();
}

void H3DSoftBodyNodeParameters::updateOutputs () {
  updateOutputsFloat ();
  updateOutputsVec3f ();
}

void H3DSoftBodyNodeParameters::updateOutputsFloat () {
  // Reset cached data
  manipulationForceMagNodeAttributes.clear();
  externalForceMagNodeAttributes.clear();

  // Get float attributes
  for ( H3DSoftBodyNodeParameters::FloatOutputList::const_iterator i= outputsFloat.begin(); i != outputsFloat.end(); ++i ) {
    SoftBodyFloatAttributeParameters* output= *i;
    switch ( output->getOutputType() ) {
      case SoftBodyFloatAttributeParameters::OUTPUT_INTERACTION_FORCE_MAGNITUDE:
        updateOutputManipulationForceMagnitude ( *output );
        break;
      case SoftBodyFloatAttributeParameters::OUTPUT_EXTERNAL_FORCE_MAGNITUDE:
        updateOutputExternalForceMagnitude ( *output );
        break;
      default: {}
    }
  }
}

void H3DSoftBodyNodeParameters::updateOutputsVec3f () {
  // Reset cached data
  manipulationForceNodeAttributes.clear();
  externalForceNodeAttributes.clear();

  // Get Vec3f attributes
  for ( H3DSoftBodyNodeParameters::Vec3fOutputList::const_iterator i= outputsVec3f.begin(); i != outputsVec3f.end(); ++i ) {
    SoftBodyVec3fAttributeParameters* output= *i;
    switch ( output->getOutputType() ) {
      case SoftBodyVec3fAttributeParameters::OUTPUT_INTERACTION_FORCE:
        updateOutputManipulationForce ( *output );
        break;
      case SoftBodyVec3fAttributeParameters::OUTPUT_EXTERNAL_FORCE:
        updateOutputExternalForce ( *output );
        break;
      default: {}
    }
  }
}

void H3DSoftBodyNodeParameters::updateOutputManipulationForce ( SoftBodyVec3fAttributeParameters& output ) {
  switch ( output.getUnitType() ) {
    case SoftBodyVec3fAttributeParameters::UNIT_NODE: {
      setOutputAttribute ( output, GetForce(), manipulationForceNodeAttributes, manipulationForces );
      break;
    }
    default: {}
  }
}

void H3DSoftBodyNodeParameters::updateOutputExternalForce ( SoftBodyVec3fAttributeParameters& output ) {
  switch ( output.getUnitType() ) {
    case SoftBodyVec3fAttributeParameters::UNIT_NODE: {
      setOutputAttribute ( output, GetForce(), externalForceNodeAttributes, vertexForces );
      break;
    }
    default: {}
  }
}

void H3DSoftBodyNodeParameters::updateOutputManipulationForceMagnitude ( SoftBodyFloatAttributeParameters& output ) {
  switch ( output.getUnitType() ) {
    case SoftBodyFloatAttributeParameters::UNIT_NODE: {
      setOutputAttribute ( output, GetForceMag(), manipulationForceMagNodeAttributes, manipulationForces );
      break;
    }
    default: {}
  }
}

void H3DSoftBodyNodeParameters::updateOutputExternalForceMagnitude ( SoftBodyFloatAttributeParameters& output ) {
  switch ( output.getUnitType() ) {
    case SoftBodyFloatAttributeParameters::UNIT_NODE: {
      setOutputAttribute ( output, GetForceMag(), externalForceMagNodeAttributes, vertexForces );
      break;
    }
    default: {}
  }
}

void H3DSoftBodyNodeParameters::copyOutputParameters( H3DSoftBodyNodeParameters& src ) {
  copyOutputFlags ( src.update_bit_mask );

  for ( size_t i= 0; i < src.outputsFloat.size() && i < outputsFloat.size(); ++i ) {
    outputsFloat[i]->copyOutputParameters ( *src.outputsFloat[i] );
  }
  for ( size_t i= 0; i < src.outputsVec3f.size() && i < outputsVec3f.size(); ++i ) {
    outputsVec3f[i]->copyOutputParameters ( *src.outputsVec3f[i] );
  }
}

void H3DSoftBodyNodeParameters::copyInputParameters( H3DSoftBodyNodeParameters& src ) {
  copyOutputFlags ( src.update_bit_mask );

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  name = src.name;
#endif

  if ( src.haveTransform() ) {
    transform= src.transform;
  }
  if ( src.havePhysicsMaterial() ) {
    // If the physicsMaterial field is changed to another
    // node instead of copying changed parameters, change the
    // pointer completetely. It allows changing between different
    // sub-classes of H3DPhysicsMaterialNodes with different
    // H3DPhysicsMaterialPropertyNodes w.o memory leakage.
    bool changeAll = false;
    if( havePhysicsMaterial() &&
      physicsMaterial != src.physicsMaterial )
      changeAll = true;

    physicsMaterial = src.physicsMaterial;
    if ( src.haveH3DPhysicsMaterialParameters() ) {

      if( changeAll || !haveH3DPhysicsMaterialParameters() )
        setH3DPhysicsMaterialParameters( src.getH3DPhysicsMaterialParameters() );
      else
        physicsMaterialParameters.get()->copyInputParameters( *src.getH3DPhysicsMaterialParameters() );

    }
  }
  if ( src.haveDeformationStrategy() ) {
    deformationStrategy= src.deformationStrategy;
    if ( src.haveDeformationStrategyParameters() ) {

      if( haveDeformationStrategyParameters() )
        deformationStrategyParameters.get()->copyInputParameters( *src.getDeformationStrategyParameters() );
      else
        setDeformationStrategyParameters( src.getDeformationStrategyParameters() );
    }
  }
  if ( src.haveGeometry() ) {
    geometry= src.geometry;
  }
  if ( src.haveSurfaceGeometry () ) {
    surfaceGeometry= src.surfaceGeometry;
  }
  if ( src.haveCollisionGeometry() ) {
    collisionGeometry= src.collisionGeometry;
  }
  if ( src.haveCoords() ) {
    coords= src.coords;
  }
  if ( src.haveIndices() ) {
    indices= src.indices;
  }
  if ( src.haveVertexForce() ) {
    vertexForces= src.vertexForces;
  }
  if ( src.haveManipulationForces() ) {
    manipulationForces= src.manipulationForces;
  }
  if ( src.haveOutputsFloat() ) {
    outputsFloat= src.outputsFloat;
  }
  if ( src.haveOutputsVec3f() ) {
    outputsVec3f= src.outputsVec3f;
  }
}

VertexBodyConstraintParameters::VertexBodyConstraintParameters()
{
}

void VertexBodyConstraintParameters::copyOutputParameters( ConstraintParameters& src ) {
  VertexBodyConstraintParameters *s = dynamic_cast< VertexBodyConstraintParameters * >( &src );
  if( s ) {
    ConstraintParameters::copyOutputParameters ( *s );
  }
}

void VertexBodyConstraintParameters::copyInputParameters( ConstraintParameters& src ) {
  VertexBodyConstraintParameters *s = dynamic_cast< VertexBodyConstraintParameters * >( &src );
  if( s ) {
    ConstraintParameters::copyInputParameters ( *s );

    if ( s->haveIndex() ) {
      index= s->index;
    }
  }
}

H3DAttachmentParameters::H3DAttachmentParameters()
: ConstraintParameters(),
body2 ( 0 )
{
}

void H3DAttachmentParameters::copyOutputParameters( ConstraintParameters& src ) {
  H3DAttachmentParameters *s = dynamic_cast< H3DAttachmentParameters * >( &src );
  if( s ) {
    ConstraintParameters::copyOutputParameters ( *s );
  }
}

void H3DAttachmentParameters::copyInputParameters( ConstraintParameters& src ) {

  H3DAttachmentParameters *s = dynamic_cast< H3DAttachmentParameters * >( &src );
  if( s ) {
    ConstraintParameters::copyInputParameters ( *s );

    if ( s->haveBody2() ) {
      body2= s->body2;
    }
    if ( s->haveIndex() ) {
      index= s->index;
    }
  }
}

H3DRigidBodyAttachmentParameters::H3DRigidBodyAttachmentParameters()
{
  type = "RigidBodyAttachmentParameters";
}

H3DSoftBodyAttachmentParameters::H3DSoftBodyAttachmentParameters()
{
  type = "H3DSoftBodyAttachmentParameters";
}

void H3DSoftBodyAttachmentParameters::copyOutputParameters( ConstraintParameters& s ) {
  H3DSoftBodyAttachmentParameters *src = dynamic_cast< H3DSoftBodyAttachmentParameters* >( &s );
  if( src ) {
    H3DAttachmentParameters::copyOutputParameters ( *src );
  }
}

void H3DSoftBodyAttachmentParameters::copyInputParameters( ConstraintParameters& s ) {
  H3DSoftBodyAttachmentParameters *src = dynamic_cast< H3DSoftBodyAttachmentParameters * >( &s );
  if( src ) {
    H3DAttachmentParameters::copyInputParameters ( *src );

    if ( src->haveIndex2() ) {
      index2 = src->index2;
    }
  }
}

SoftBodyAttachmentParameters::SoftBodyAttachmentParameters() :
  physicsMaterial ( NULL )
{
}

void SoftBodyAttachmentParameters::copyOutputParameters( H3DAttachmentParameters& src ) {

}

void SoftBodyAttachmentParameters::copyInputParameters( H3DAttachmentParameters& src ) {
  SoftBodyAttachmentParameters* params=
    dynamic_cast<SoftBodyAttachmentParameters*>(&src);
  H3DSoftBodyAttachmentParameters::copyInputParameters ( src );

  if ( params ) {
    if ( params->havePhysicsMaterial() ) {
    // If the physicsMaterial field is changed to another
    // node instead of copying changed parameters, change the
    // pointer completetely. It allows changing between different
    // sub-classes of H3DPhysicsMaterialNodes with different
    // H3DPhysicsMaterialPropertyNodes w.o memory leakage.
    bool changeAll = false;
    if( havePhysicsMaterial() &&
      physicsMaterial != params->physicsMaterial )
      changeAll = true;

    physicsMaterial = params->physicsMaterial;
    if ( params->haveH3DPhysicsMaterialParameters() ) {

      if( changeAll || !haveH3DPhysicsMaterialParameters() )
        setH3DPhysicsMaterialParameters( params->getH3DPhysicsMaterialParameters() );
      else
        physicsMaterialParameters.get()->copyInputParameters( *params->getH3DPhysicsMaterialParameters() );

    }
  }
  }
}

SoftBodyLinearJointParameters::SoftBodyLinearJointParameters()
: JointParameters()
{
  type = "SoftBodyLinearJoint";
}
void SoftBodyLinearJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  SoftBodyLinearJointParameters *src = dynamic_cast< SoftBodyLinearJointParameters * >( &s );
  if( src ) {
    JointParameters::copyOutputParameters ( *src );
  }
}

void SoftBodyLinearJointParameters::copyInputParameters( ConstraintParameters& s ) {
  SoftBodyLinearJointParameters *src = dynamic_cast< SoftBodyLinearJointParameters * >( &s );
  if( src ) {
    JointParameters::copyInputParameters ( *src );

    if ( src->haveAnchorPoint() ) {
      anchor_point = src->anchor_point;
    }
  }
}

SoftBodyAngularJointParameters::SoftBodyAngularJointParameters() :
JointParameters()
{
  type = "SoftBodyAngularJoint";
}

void SoftBodyAngularJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  SoftBodyAngularJointParameters *src = dynamic_cast< SoftBodyAngularJointParameters * >( &s );
  if( src ) {
    JointParameters::copyOutputParameters ( *src );
  }
}

void SoftBodyAngularJointParameters::copyInputParameters( ConstraintParameters& s ) {
  SoftBodyAngularJointParameters *src = dynamic_cast< SoftBodyAngularJointParameters * >( &s );
  if( src ) {
    JointParameters::copyInputParameters ( *src );

    if ( src->haveAxis() ) {
      axis = src->axis;
    }
  }
}
// Since modifiers are not transferred to the external engines, it
// does nothing. For extension purposes, however, put in the intrerface.
void ModifierParameters::copyOutputParameters( ModifierParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
}
void ModifierParameters::copyInputParameters( ModifierParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
  body1 = s.body1;
  device_index = s.device_index;
}
void DeformationStrategyParameters::copyOutputParameters( DeformationStrategyParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
}

void DeformationStrategyParameters::copyInputParameters( DeformationStrategyParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );

  if ( s.haveSolverType() ) {
    h3d_solver = s.h3d_solver;
  }
}
void H3DPhysicsMaterialParameters::copyOutputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
}

void H3DPhysicsMaterialParameters::copyInputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );

  if ( s.haveDamping() ) {
    // If the damping field is changed to another node instead of copying
    // changed parameters, change the pointer completetely.
    bool changeAll = false;
    if( haveDamping() &&
      damping != s.damping )
      changeAll = true;

    damping = s.damping;
    if ( s.haveDampingParameters() ) {
      if( changeAll || !haveDampingParameters() )
        setDampingParameters( s.getDampingParameters() );
      else
        dampingParameters.get()->copyInputParameters( *s.getDampingParameters() );
    }
  }

  if ( s.haveMass() ) {
    // If the mass field is changed to another node instead of copying
    // changed parameters, change the pointer completetely.
    bool changeAll = false;
    if( haveMass() &&
      mass != s.mass )
      changeAll = true;

    mass = s.mass;
    if ( s.haveMassParameters() ) {
      if( changeAll || !haveMassParameters() )
        setMassParameters( s.getMassParameters() );
      else
        massParameters.get()->copyInputParameters( *s.getMassParameters() );
    }
  }

  if ( s.haveFriction() ) {
    // If the friction field is changed to another node instead of copying
    // changed parameters, change the pointer completetely.
    bool changeAll = false;
    if( haveFriction() &&
      friction != s.friction )
      changeAll = true;

    friction = s.friction;
    if ( s.haveFrictionParameters() ) {
      if( changeAll || !haveFrictionParameters() )
        setFrictionParameters( s.getFrictionParameters() );
      else
        frictionParameters.get()->copyInputParameters( *s.getFrictionParameters() );
    }
  }

}

PhysicsMaterialParameters::PhysicsMaterialParameters() :
H3DPhysicsMaterialParameters()
{

}

void PhysicsMaterialParameters::copyOutputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
}

void PhysicsMaterialParameters::copyInputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );

  PhysicsMaterialParameters *src = dynamic_cast< PhysicsMaterialParameters * >( &s );
  if( src ) {
    H3DPhysicsMaterialParameters::copyInputParameters ( *src );

    if ( src->haveElasticity() ) {
      // If the elasticity field is changed to another node instead of copying
      // changed parameters, change the pointer completetely.
      bool changeAll = false;
      if( haveElasticity() &&
        elasticity != src->elasticity )
        changeAll = true;

      elasticity = src->getElasticity();
      if ( src->haveElasticityParameters() ) {
        if( changeAll || !haveElasticityParameters() )
          setElasticityParameters( src->getElasticityParameters() );
        else
          elasticityParameters.get()->copyInputParameters( *src->getElasticityParameters() );
      }
    }
  }
}

MassSpringPhysicsMaterialParameters::MassSpringPhysicsMaterialParameters()
{
}

void MassSpringPhysicsMaterialParameters::copyOutputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
}

void MassSpringPhysicsMaterialParameters::copyInputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );

  MassSpringPhysicsMaterialParameters *src = dynamic_cast< MassSpringPhysicsMaterialParameters * >( &s );
  if( src ) {
    H3DPhysicsMaterialParameters::copyInputParameters ( *src );

    if ( src->haveStiffness() ) {
      // If the stiffness field is changed to another node instead of copying
      // changed parameters, change the pointer completetely.
      bool changeAll = false;
      if( haveStiffness() &&
        stiffness != src->stiffness )
        changeAll = true;

      stiffness = src->getStiffness();
      if ( src->haveStiffnessParameters() ) {
        if( changeAll || !haveStiffnessParameters() )
          setStiffnessParameters( src->getStiffnessParameters() );
        else
          stiffnessParameters.get()->copyInputParameters( *src->getStiffnessParameters() );
      }
    }

  }

}

FEMPhysicsMaterialParameters::FEMPhysicsMaterialParameters() :
PhysicsMaterialParameters()
{

}

void FEMPhysicsMaterialParameters::copyOutputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
}

void FEMPhysicsMaterialParameters::copyInputParameters( H3DPhysicsMaterialParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );

  FEMPhysicsMaterialParameters *src = dynamic_cast< FEMPhysicsMaterialParameters * >( &s );
  if( src ) {
    PhysicsMaterialParameters::copyInputParameters ( *src );

    if ( src->havePoissonRatio() ) {
      // If the poissonRatio field is changed to another node instead of copying
      // changed parameters, change the pointer completetely.
      bool changeAll = false;
      if( havePoissonRatio() &&
        poissonRatio != src->poissonRatio )
        changeAll = true;

      poissonRatio = src->getPoissonRatio();
      if ( src->havePoissonRatioParameters() ) {
        if( changeAll || !havePoissonRatioParameters() )
          setPoissonRatioParameters( src->getPoissonRatioParameters() );
        else
          poissonRatioParameters.get()->copyInputParameters( *src->getPoissonRatioParameters() );
      }
    }

  }
}

void MaterialPropertyParameters::copyOutputParameters( MaterialPropertyParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
}

void MaterialPropertyParameters::copyInputParameters( MaterialPropertyParameters& s ) {
  copyOutputFlags ( s.update_bit_mask );
  if ( s.haveUnitType() ) {
    unitType= s.unitType;
  }
}

void FloatMaterialPropertyParameters::copyInputParameters( MaterialPropertyParameters& s ) {
  MaterialPropertyParameters::copyInputParameters ( s );

  FloatMaterialPropertyParameters *src = dynamic_cast< FloatMaterialPropertyParameters * >( &s );
  if( src ) {
    if ( src->haveValue() ) {
      value = src->value;
    }
  }
}
