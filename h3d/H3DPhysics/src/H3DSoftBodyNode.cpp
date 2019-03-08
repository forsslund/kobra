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
/// \file H3DSoftBodyNode.cpp
/// \brief Source file for H3DSoftBodyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DSoftBodyNode.h>
#include <H3D/H3DPhysics/CollidableOffset.h>
#include <H3D/H3DPhysics/CollidableShape.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>
#include <H3D/DeviceInfo.h>
#include <H3D/Shape.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase H3DSoftBodyNode::database( "H3DSoftBodyNode",
                                          NULL,
                                          typeid( H3DSoftBodyNode ),
                                          &H3DBodyNode::database);

namespace H3DSoftBodyNodeInternals {
  FIELDDB_ELEMENT( H3DSoftBodyNode, transform, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, physicsMaterial, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, geometry, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, surfaceGeometry, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, collisionGeometry, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, surfaceMapping, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, collisionMapping, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, deformationStrategy, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, output, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyNode, engineOptions, INPUT_OUTPUT )
}

H3DSoftBodyNode::H3DSoftBodyNode(Inst< SFNode > _metadata,
                                 Inst< SFMatrix4f > _transform,
                                 Inst< SFH3DPhysicsMaterialNode > _physicsMaterial,
                                 Inst< SFX3DGeometryNode > _geometry,
                                 Inst< MFX3DGeometryNode > _surfaceGeometry,
                                 Inst< MFX3DNBodyCollidableNode > _collisionGeometry,
                                 Inst< MFH3DGeometryMapping > _surfaceMapping,
                                 Inst< MFH3DGeometryMapping > _collisionMapping,
                                 Inst< SFH3DDeformationStrategyNode > _deformationStrategy,
                                 Inst< MFH3DSoftBodyOutputNode > _output,
                                 Inst< LinkGeometry > _linkSurfaceGeometry,
                                 Inst< LinkGeometry > _linkCollisionGeometry,
                                 Inst< ValueUpdater > _valueUpdater,
                                 Inst< MFEngineOptions > _engineOptions ) :
H3DBodyNode ( _metadata ),
transform ( _transform ),
physicsMaterial ( _physicsMaterial ),
geometry ( _geometry ),
surfaceGeometry ( _surfaceGeometry ),
collisionGeometry ( _collisionGeometry ),
surfaceMapping ( _surfaceMapping ),
collisionMapping ( _collisionMapping ),
deformationStrategy ( _deformationStrategy ),
output ( _output ),
linkSurfaceGeometry ( _linkSurfaceGeometry ),
linkCollisionGeometry ( _linkCollisionGeometry ),
valueUpdater ( _valueUpdater ),
engineOptions ( _engineOptions ),
geometryChanged ( new Field ),
previousPhysicsMaterial ( NULL ),
previousDeformationStrategy ( NULL ) {
  // init fields
  type_name = "H3DSoftBodyNode";
  database.initFields( this );

  valueUpdater->setOwner ( this );
  valueUpdater->setName ( "valueUpdater" );

  geometryChanged->setOwner ( this );
  geometryChanged->setName ( "geometryChanged" );
  geometry->route ( geometryChanged );

  linkSurfaceGeometry->setOwner ( this );
  linkSurfaceGeometry->setName ( "linkSurfaceGeometry" );
  geometry->route ( linkSurfaceGeometry );
  surfaceGeometry->route ( linkSurfaceGeometry );
  surfaceMapping->route ( linkSurfaceGeometry );

  linkCollisionGeometry->setOwner ( this );
  linkCollisionGeometry->setName ( "linkCollisionGeometry" );
  geometry->route ( linkCollisionGeometry );
  collisionGeometry->route ( linkCollisionGeometry );
  collisionMapping->route ( linkCollisionGeometry );

  transform->route ( valueUpdater );
  physicsMaterial->route ( valueUpdater );
  geometry->route ( valueUpdater );
  surfaceGeometry->route ( valueUpdater );
  collisionGeometry->route ( valueUpdater );
  deformationStrategy->route ( valueUpdater );
  output->route ( valueUpdater );
  geometryChanged->route ( valueUpdater );
  engineOptions->route ( valueUpdater );

  // Initialize the variables used in the getSoftBodyParameters.
  previousPhysicsMaterial = physicsMaterial->getValue();
  previousDeformationStrategy = deformationStrategy->getValue();
}

void H3DSoftBodyNode::traverseSG(H3D::TraverseInfo &ti) {
  X3DNode::traverseSG( ti );
  SoftBodyPhysicsEngineThread* pt = NULL;
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt ) {
    // initialize the soft body if not initialized
    if( !isInitialized() ) {
      initializeBody( *pt );
    }

    if( pt == engine_thread && isInitialized() ) {

      // Any user defined changes to geometry must be handled
      // before the geometry is overwritten with the physics engine state
      valueUpdater->upToDate();

      // update output fields
      auto_ptr < PhysicsEngineParameters::H3DSoftBodyNodeParameters > params ( createSoftBodyParameters() );
      pt->getSoftBodyParameters( body_id, *params );
      setSoftBodyParameters ( *params );

      // update the surface geometry using the surface mapping
      if ( geometry->getValue() &&
        surfaceMapping->getValue().size() > 0 &&
        surfaceGeometry->getValue().size()> 0 ) {

          // Ensure geometry is linked before updating
          linkSurfaceGeometry->upToDate();
          updateSurfaceGeometry ();

      }

      // update the collision geometry using the surface mapping
      if ( geometry->getValue() &&
        collisionMapping->getValue().size() &&
        collisionGeometry->getValue().size() ) {

          // Ensure geometry is linked before updating
          linkCollisionGeometry->upToDate();
          updateCollisionGeometry ();

      }
    }

  }
}

void H3DSoftBodyNode::ValueUpdater::update()
{
  H3DSoftBodyNode *body = static_cast< H3DSoftBodyNode * >( getOwner());

  if ( body->isInitialized() ) {
    H3DSoftBodyNodeParameters *params = body->getSoftBodyParameters();
    static_cast<SoftBodyPhysicsEngineThread*>(body->engine_thread)->setSoftBodyParameters( body->body_id, *params );
  }
  EventCollectingField< PeriodicUpdate< Field > >::update();
}

bool H3DSoftBodyNode::initializeBody( PhysicsEngineThread& pt ) {
  if( isInitialized() ) return false;

  if ( SoftBodyPhysicsEngineThread* sbpt= dynamic_cast<SoftBodyPhysicsEngineThread*>(&pt) ) {

    valueUpdater->upToDate();
    engine_thread= &pt;
    PhysicsEngineParameters::H3DSoftBodyNodeParameters *p = getSoftBodyParameters( true );
    body_id = sbpt->addSoftBody( *p );
    if( body_id == 0 )
      delete p;
    engine_thread = NULL;
    return H3DBodyNode::initializeBody ( pt );
  } else {
    return false;
  }
}

bool H3DSoftBodyNode::deleteBody() {
  if ( isInitialized() ) {
    static_cast<SoftBodyPhysicsEngineThread*>(engine_thread)->removeSoftBody( body_id );
    return true;
  }
  return false;
}

PhysicsEngineParameters::H3DSoftBodyNodeParameters*
H3DSoftBodyNode::getSoftBodyParameters ( bool all_params ) {
  H3DSoftBodyNodeParameters* params= createSoftBodyParameters();

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  params->setName( this->getName() );
#endif

  if ( all_params || valueUpdater->hasCausedEvent ( transform ) ) {
    params->setTransform ( transform->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( physicsMaterial ) ) {
    params->setPhysicsMaterial ( physicsMaterial->getValue() );

    // If the pyhsicsMaterial field is set to a different node
    // create all the physics material parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousPhysicsMaterial != physicsMaterial->getValue() )
    {
      material_params = true;
      previousPhysicsMaterial = physicsMaterial->getValue();
    }
    H3DPhysicsMaterialNode* pm = physicsMaterial->getValue();
    if( pm )
      params->setH3DPhysicsMaterialParameters( pm->valueUpdater->
        getH3DPhysicsMaterialParameters( material_params ) );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( geometry ) ) {
    if ( X3DGeometryNode* g= geometry->getValue() ) {
      g->boundTree->upToDate();
    }
    params->setGeometry ( *geometry->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( surfaceGeometry ) ) {
    const NodeVector &surfaceGeoms = (surfaceGeometry->getValue());
    sgList.clear();
    for( unsigned int i = 0; i < surfaceGeoms.size(); ++i ) {
      X3DGeometryNode* s_geom = static_cast< X3DGeometryNode* >( surfaceGeoms[i] );
      sgList.push_back( s_geom );
    }
    params->setSurfaceGeometry ( sgList );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( collisionGeometry ) ) {
    const NodeVector &collisionGeoms = (collisionGeometry->getValue());
    cnList.clear();
    for( unsigned int i = 0; i < collisionGeoms.size(); ++i ) {
      X3DNBodyCollidableNode* c_geom = static_cast< X3DNBodyCollidableNode* >( collisionGeoms[i] );
      cnList.push_back( c_geom );
    }
    params->setCollisionGeometry( cnList );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( deformationStrategy ) ) {
    params->setDeformationStrategy ( deformationStrategy->getValue() );

    // If the deformationStrategy field is set to a different node
    // create all the physics strategy parameters in order to
    // initialize them in the physics_engine layer.
    bool strategy_params = all_params;
    if( previousDeformationStrategy != deformationStrategy->getValue() )
    {
      strategy_params = true;
      previousDeformationStrategy = deformationStrategy->getValue();
    }
    H3DDeformationStrategyNode* ds = deformationStrategy->getValue();
    if( ds )
      params->setDeformationStrategyParameters( ds->valueUpdater->
      getDeformationStrategyParameters( strategy_params ) );

  }

  if ( all_params || valueUpdater->hasCausedEvent ( output ) ) {
    H3DSoftBodyNodeParameters::FloatOutputList outputsFloat;
    H3DSoftBodyNodeParameters::Vec3fOutputList outputsVec3f;
    const NodeVector& outputNodes= output->getValue();
    for ( NodeVector::const_iterator i= outputNodes.begin(); i != outputNodes.end(); ++i ) {
      H3DSoftBodyOutputNode* n= static_cast < H3DSoftBodyOutputNode* > ( *i );
      H3DSoftBodyOutputParameters* p= n->valueUpdater->getParameters ( true );
      if ( SoftBodyFloatAttributeParameters* fp= dynamic_cast<SoftBodyFloatAttributeParameters*>(p) ) {
        outputsFloat.push_back ( fp );
      } else if ( SoftBodyVec3fAttributeParameters* v3p= dynamic_cast<SoftBodyVec3fAttributeParameters*>(p) ) {
        outputsVec3f.push_back ( v3p );
      }
    }

    params->setOutputsFloat ( outputsFloat );
    params->setOutputsVec3f ( outputsVec3f );
  }

  if ( H3DEngineOptions* options= engineOptions->getOptions( engine_thread->getEngine() ) ) {
    if ( all_params || valueUpdater->hasCausedEvent ( options->valueUpdater ) ) {
      params->setEngineOptions ( options->valueUpdater->getParameters( all_params ) );
    }
  }
  return params;
}

void H3DSoftBodyNode::setSoftBodyParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params ) {
  // Update per-element output attributes

  // Float
  const H3DSoftBodyNodeParameters::FloatOutputList& outputsFloat= params.getOutputsFloat();
  for ( H3DSoftBodyNodeParameters::FloatOutputList::const_iterator i= outputsFloat.begin(); i != outputsFloat.end(); ++i ) {
    (*i)->getNode()->setOutputParameters ( **i );
  }

  // Vec3f
  const H3DSoftBodyNodeParameters::Vec3fOutputList& outputsVec3f= params.getOutputsVec3f();
  for ( H3DSoftBodyNodeParameters::Vec3fOutputList::const_iterator i= outputsVec3f.begin(); i != outputsVec3f.end(); ++i ) {
    (*i)->getNode()->setOutputParameters ( **i );
  }
}

vector<Vec3f> H3DSoftBodyNode::transformCoords ( const vector<Vec3f>& coords ) {
  Matrix4f t= getTransform();

  vector<Vec3f> transformedCoords;
  transformedCoords.reserve ( coords.size() );
  for ( MFVec3f::const_iterator i= coords.begin(); i != coords.end(); ++i ) {
    transformedCoords.push_back ( t*(*i) );
  }

  return transformedCoords;
}

void H3DSoftBodyNode::LinkGeometry::update () {
  H3DSoftBodyNode* softBody= static_cast < H3DSoftBodyNode* > ( getOwner() );

  X3DGeometryNode *geom =
    static_cast< X3DGeometryNode * >( static_cast< SFX3DGeometryNode* >
    ( routes_in[0] )->getValue() );

  const NodeVector &target_geoms = static_cast< MFNode* >( routes_in[1] )->getValue();

  X3DGeometryNodeList t_list;
  for( unsigned int i = 0; i < target_geoms.size(); ++i ) {
    if( target_geoms[i] )
    {
      if( X3DNBodyCollidableNode *tc = dynamic_cast< X3DNBodyCollidableNode * >(target_geoms[i]) ) {

        Shape *s;
        X3DGeometryNode *t_geom;

        if ( CollidableOffset *co = dynamic_cast< CollidableOffset* >(tc) ) {
          s = co->getShape();
        } else if ( CollidableShape *cs = dynamic_cast< CollidableShape* >(tc) ) {
          s = static_cast< Shape* >( cs->shape->getValue() );
        }
        t_geom = dynamic_cast< X3DGeometryNode* >( s->geometry->getValue() );
        t_list.push_back( t_geom );

      } else if ( X3DGeometryNode *tg = dynamic_cast< X3DGeometryNode * >(target_geoms[i]) ) {
        t_list.push_back( tg );
      }
    }
  }

  const NodeVector &mappings = static_cast< MFH3DGeometryMapping* >( routes_in[2] )->getValue();
  H3DGeometryMappingVector m_list;
  for( unsigned int i = 0; i < mappings.size(); ++i ) {
    H3DGeometryMapping* mapping = static_cast< H3DGeometryMapping* >( mappings[i] );
    m_list.push_back( mapping );
  }
  Field::update ();

  if( geom && ( t_list.size() == m_list.size() ) )
    softBody->linkGeometries( (*geom), t_list, m_list );

}