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
/// \file FunctionSoftBodyModifier.cpp
/// \brief Source file for FunctionSoftBodyModifier, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/FunctionSoftBodyModifier.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>
#include <H3D/GaussianFunction.h>
#include <HAPI/HAPIHapticsDevice.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase FunctionSoftBodyModifier::database( "FunctionSoftBodyModifier", 
                                                   &(newInstance<FunctionSoftBodyModifier>), 
                                                   typeid( FunctionSoftBodyModifier ),
                                                   &H3DAdjacencySoftBodyDeformer::database);

namespace SoftBodyDeformerInternals {
  FIELDDB_ELEMENT( FunctionSoftBodyModifier, distanceToForce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FunctionSoftBodyModifier, maxDisplacement, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FunctionSoftBodyModifier, fadeDistance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( FunctionSoftBodyModifier, savePose, INPUT_ONLY )
}

FunctionSoftBodyModifier::FunctionSoftBodyModifier (
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DBodyNode > _body1,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFGeometryNode > _hapticGeometry,
  Inst< SFInt32 > _deviceIndex,
  Inst< SFInt32 > _maxAdjacencyDistance,
  Inst< SFFunction > _distanceToForce,
  Inst< SFFloat > _maxDisplacement,
  Inst< SFFloat > _fadeDistance,
  Inst< SavePose > _savePose ) :
H3DAdjacencySoftBodyDeformer ( _metadata, _valueUpdater, _body1, 
                               _engineOptions, _hapticGeometry, _deviceIndex, _maxAdjacencyDistance ),
                               distanceToForce ( _distanceToForce ),
    maxDisplacement ( _maxDisplacement ),
    fadeDistance ( _fadeDistance ),
    savePose ( _savePose ),
    updatePoseNow ( true ),
    gaussian_center ( 0.0f ),
    gaussian_width ( 0.0f ),
    gaussian_amplitude( 0.0f ),
    max_displacement( -1 ),
    fade_distance( 0.001f )
{
  // init fields
  type_name = "FunctionSoftBodyModifier";
  database.initFields( this );  

  maxDisplacement->setValue ( max_displacement );
  fadeDistance->setValue( fade_distance );
  savePose->setValue ( false );
}

void FunctionSoftBodyModifier::traverseSG ( TraverseInfo& ti ) {
  H3DAdjacencySoftBodyDeformer::traverseSG ( ti );

  H3DFunctionNode* fn= distanceToForce->getValue();
  traverseLock.lock();

  if ( GaussianFunction* g= dynamic_cast<GaussianFunction*>(fn) ) {
    gaussian_center= g->center->getValue();
    gaussian_width= g->width->getValue();
    gaussian_amplitude= g->amplitude->getValue();
    hapiDistanceToForce.reset ( NULL );
  } else {
    hapiDistanceToForce.reset ( fn ? fn->getAsHAPIFunctionObject() : NULL );
  }

  max_displacement= maxDisplacement->getValue();
  fade_distance= fadeDistance->getValue();

  traverseLock.unlock();
}

// Calculate forces for deformation
void FunctionSoftBodyModifier::calculateForces ( H3DSoftBodyNodeParameters& softBodyParams,
                                                 HAPI::HAPIHapticsDevice& hd ) {
  traverseLock.lock();

  if ( updatePoseNow && max_displacement > 0 ) {
    updatePose ( softBodyParams );
    updatePoseNow= false;
  }

  std::vector<Vec3f> positions;

  // For the renderer on each layer
  for( unsigned int layer = 0; layer < hd.nrLayers(); ++layer ) {
    HAPI::HAPIHapticsRenderer* renderer = 
      hd.getHapticsRenderer( layer );
    HAPI::HAPIHapticsRenderer::Contacts contacts;
    if( renderer ) {
      contacts = renderer->getContacts();
    }
    for( HAPI::HAPIHapticsRenderer::Contacts::iterator j = contacts.begin();
          j != contacts.end(); ++j ) {
      X3DGeometryNode *geom =
        static_cast< X3DGeometryNode * >( (*j).first->getUserData() );

        // If device in contact with the output geometry
        if( softBodyParams.getSurfaceGeometry().size() > 0 ) {
          const H3DSoftBodyNodeParameters::X3DGeometryNodeList& surfaceGeom= softBodyParams.getSurfaceGeometry();
          if ( find ( surfaceGeom.begin(), surfaceGeom.end(), geom ) != surfaceGeom.end() ) {
            positions.push_back ( accumulatedInverse*Vec3f((*j).second.globalContactPoint()) );
          }
        } else {
          if ( geom == softBodyParams.getGeometry() ) {
            positions.push_back ( accumulatedInverse*Vec3f((*j).second.globalContactPoint()) );
          }
        }
      }
    }

  if ( !positions.empty() ) {
    Vec3f force= Vec3f(accumulatedInverse.getRotationPart()*hd.getForce());
    const H3DSoftBodyNodeParameters::CoordList& coords= softBodyParams.getCoords();
    for ( std::vector<Vec3f>::iterator i= positions.begin(); i != positions.end(); ++i ) {
      Vec3f position= *i;

      if ( max_adjacency_distance > -1 ) {

        IndexList adjacent= getAdjacentVerticesFromPosition ( position, max_adjacency_distance, softBodyParams );
      
        for ( IndexList::iterator j= adjacent.begin(); j != adjacent.end(); ++j ) {
          applyForceToVertex ( *j, softBodyParams, position, force );
        }

      } else {
        for ( size_t j= 0; j < coords.size(); ++j ) {
          applyForceToVertex ( j, softBodyParams, position, force );
        }
      }
    }
  }

  traverseLock.unlock();
}

void FunctionSoftBodyModifier::updatePose ( H3DSoftBodyNodeParameters& softBodyParams ) {
  pose= softBodyParams.getCoords();
}

void FunctionSoftBodyModifier::applyForceToVertex ( size_t vertexIndex, PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams, Vec3f position, Vec3f force ) {
  const H3DSoftBodyNodeParameters::CoordList& coords= softBodyParams.getCoords();

  Vec3f p= coords[vertexIndex];
  H3DDouble distance= (position-p).length();
        
  H3DFloat mag;
  if ( hapiDistanceToForce.get() ) {
    mag= static_cast<H3DFloat>(hapiDistanceToForce->evaluate ( &distance ));
  } else {
    H3DFloat diff= static_cast<H3DFloat>(distance - gaussian_center);
    mag= gaussian_amplitude * H3DExp( -(diff*diff )/( gaussian_width * gaussian_width ) );
  }
  Vec3f vertexForce= -Vec3f(force*mag);

  // enforce a maximum displacement?
  if ( max_displacement > 0 ) {
    H3DFloat displacement= (pose[vertexIndex]-coords[vertexIndex]).length();

    H3DFloat forceMag= vertexForce.length();
        
    if ( displacement > max_displacement ) {
      H3DFloat dist= displacement - max_displacement;
      H3DFloat _p= 1.0f;
      if ( dist < fade_distance ) {
        _p = dist/fade_distance;
      }

      forceMag*= (1-_p);
    }

    vertexForce.normalizeSafe();
    vertexForce*= forceMag;
  }

  if ( vertexForce.length() > H3DUtil::Constants::f_epsilon ) {
    softBodyParams.addManipulationForce ( 
      H3DSoftBodyNodeParameters::VertexForce ( vertexIndex, vertexForce) );
  }
}

void FunctionSoftBodyModifier::SavePose::onNewValue ( const bool& v ) {
   FunctionSoftBodyModifier* node= static_cast<FunctionSoftBodyModifier*>(getOwner());
   node->traverseLock.lock();
   node->updatePoseNow= true;
   node->traverseLock.unlock();
}
