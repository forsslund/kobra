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
/// \file PhysXCallbacks.cpp
/// \brief Source file for PhysXCallbacks, callback functions for
/// the PhysX physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysXCallbacks.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>

#include <H3D/Box.h>
#include <H3D/Cone.h>
#include <H3D/Sphere.h>

#ifdef HAVE_PHYSX

#include <H3D/H3DPhysics/PhysX/cooking.h>
#include <H3D/H3DPhysics/PhysX/Stream.h>

using namespace H3D;

SoftBodyPhysicsEngineThread::SoftBodyPhysicsEngineRegistration 
PhysXCallbacks::registration( "PhysX", 
                              SoftBodyPhysicsEngineThread::createSoftBodyPhysicsEngineCallbacks< PhysXCallbacks >() );

PhysXCallbacks::BodyShapeMap PhysXCallbacks::bodyShapeMap;

PhysXCallbacks::ShapeInfoMap PhysXCallbacks::shapeInfoMap;
int PhysXCallbacks::ShapeInfo::created_shape_id = 1; // Starts as one because we want to treat 0 as invalid id.

PhysXCallbacks::ConstraintInfoMap PhysXCallbacks::constraintInfoMap;
int PhysXCallbacks::ConstraintInfo::created_joint_id = 0;

void PhysXCallbacks::ContactReport::onContactNotify(NxContactPair& pair, NxU32 events) {
  NxContactStreamIterator i(pair.stream);
  // PhysX reports contacts for deleted actors, we are not interested in those contacts.
  // It does however seems to be some sort of bug in such a way that new actors
  // will not report contacts after first run of doSimulation.
  if( pair.isDeletedActor[0] || pair.isDeletedActor[1] )
    return;

  while(i.goNextPair()) // user can call getNumPairs() here 
      {
      while(i.goNextPatch()) // user can also call getShape(), isDeletedShape() and getNumPatches() here
          {
          while(i.goNextPoint()) //user can also call getPatchNormal() and getNumPoints() here
              {
              //user can also call getPoint() and getSeparation() here
                
                ContactParameters c;
                c.body1_id= (H3DBodyId)static_cast<ShapeInfo*>(i.getShape(0)->userData)->actor;
                c.body2_id= (H3DBodyId)static_cast<ShapeInfo*>(i.getShape(1)->userData)->actor;
                c.geom1_id= (H3DCollidableId)static_cast<ShapeInfo*>(i.getShape(0)->userData)->shape;
                c.geom2_id= (H3DCollidableId)static_cast<ShapeInfo*>(i.getShape(1)->userData)->shape;
                c.position= toVec3f ( i.getPoint() );
                c.contact_normal= toVec3f ( i.getPatchNormal() );
                c.depth= i.getPointNormalForce();//i.getSeparation();
                
                contacts.push_back ( c );
              }
          }
      }
}

void PhysXCallbacks::ContactReport::clearContacts () {
  contacts.clear();
}

void PhysXCallbacks::ContactReport::getContacts ( list<PhysicsEngineParameters::ContactParameters>& _contacts ) {
  _contacts.insert ( _contacts.begin(), contacts.begin(), contacts.end() );
}

PhysXH3DSoftBody::PhysXH3DSoftBody ( PhysXCallbacks::PhysXSpecificData& physx_data ) 
: scene ( physx_data.scene ),
  mVertexRenderBuffer ( NULL ),
  mIndexRenderBuffer ( NULL ),
  waitForNonZeromNumVertices( false ) {
}

PhysXSoftBody::PhysXSoftBody ( PhysicsEngineParameters::SoftBodyParameters& params,
                         PhysXCallbacks::PhysXSpecificData& physx_data ) :
softBody ( NULL ), 
softBodyMesh ( NULL ),
PhysXH3DSoftBody ( physx_data ),
stiffnessUnitType ( PhysicsEngineParameters::MaterialPropertyParameters::UNIT_UNIFORM )
{
  NxSoftBodyDesc softBodyDesc;

  // Generate the mesh descrition
  NxVec3* vertices_ptr = NULL;
  NxU32 * indices_ptr = NULL;
  NxSoftBodyMeshDesc meshDesc =
    generateMeshDesc ( params, vertices_ptr, indices_ptr );

  // Cook the mesh
  if ( !cookMesh ( meshDesc ) ) {
    Console(4) << "FAILED to cook mesh" << endl;
  } else {

    // Allocate recieve buffers
    allocateReceiveBuffers(meshDesc.numVertices, meshDesc.numTetrahedra);

    softBodyDesc.softBodyMesh= softBodyMesh;
    softBodyDesc.meshData= receiveBuffers;
    softBodyDesc.particleRadius= 0.001f;
    //softBodyDesc.relativeGridSpacing= 0.01;

    // Create the soft body
    softBody= scene->createSoftBody ( softBodyDesc );
    setParameters ( params );
  }
  delete []vertices_ptr;
  delete []indices_ptr;
}

// Destructor
PhysXSoftBody::~PhysXSoftBody ()
{
  // Release soft body
  if ( softBody )
    scene->releaseSoftBody ( *softBody );

  // Release mesh data

  // Deallocate recieve buffers
  free( mVertexRenderBuffer );
  free( mIndexRenderBuffer );
  
}

// Get current properties of the simulation cloth
void PhysXSoftBody::getParameters ( H3DSoftBodyNodeParameters& params )
{
  bool get_vertices = true;
  if( waitForNonZeromNumVertices ) {
    if( mNumVertices > 0 ) waitForNonZeromNumVertices = false;
    else get_vertices = false;
  }
  if ( params.haveGeometry() && get_vertices )
  {
    H3DSoftBodyNodeParameters::IndexList indices;
    H3DSoftBodyNodeParameters::CoordList vertices;

    for ( size_t i= 0; i < mNumVertices; ++i )
    {
      RenderBufferVertexElement e= mVertexRenderBuffer[i];
      vertices.push_back ( Vec3f ( e.position.x, e.position.y, e.position.z ) );
    }

    for ( size_t i= 0; i < mNumIndices; ++i )
    {
      indices.push_back ( mIndexRenderBuffer[i] );
    }

      params.setCoords ( vertices );
    //params.setIndices ( indices );
  }
}

void PhysXSoftBody::setParameters ( H3DSoftBodyNodeParameters& params ) {

  if ( params.havePhysicsMaterial() &&
      params.haveH3DPhysicsMaterialParameters()) {
    H3DPhysicsMaterialParameters * matPar = 
        params.getH3DPhysicsMaterialParameters();
    if( matPar->haveMass() &&
        matPar->haveMassParameters() ) {
    }
    if( matPar->haveStiffness() &&
        matPar->haveStiffnessParameters() ) {
      applyStiffnessParameters ( *matPar->getStiffnessParameters() );
    }
    if( matPar->haveDamping() &&
          matPar->haveDampingParameters() ) {
      applyDampingParameters ( *matPar->getDampingParameters() );
    }
    if( matPar->haveFriction() &&
        matPar->haveFrictionParameters() ) {
      applyFrictionParameters ( *matPar->getFrictionParameters() );
    }
  }

  // Fixed vertices
  //if ( params.haveFixedVertices() ) {
  //  fixVertices ( params );
  //}
}

void PhysXSoftBody::applyExternalForces ( H3DSoftBodyNodeParameters& bodyParameters ) {

  // Apply user defined external forces
  const H3DSoftBodyNodeParameters::VertexForceList& vertexForces= 
    bodyParameters.getVertexForces();
  for ( H3DSoftBodyNodeParameters::VertexForceList::const_iterator i= vertexForces.begin();
    i != vertexForces.end(); ++i ) {
      softBody->addForceAtVertex ( toNxVec3 ( (*i).second.force ), (*i).second.index );
  }

  // Apply manipulation external forces
  const H3DSoftBodyNodeParameters::VertexForceList& manipulationForces= 
    bodyParameters.getManipulationForces();
  for ( H3DSoftBodyNodeParameters::VertexForceList::const_iterator i= manipulationForces.begin();
    i != manipulationForces.end(); ++i ) {
      softBody->addForceAtVertex ( toNxVec3 ( (*i).second.force ), (*i).second.index );
  }
}

// Generates a physX cloth mesh description from the specified geometry
NxSoftBodyMeshDesc PhysXSoftBody::generateMeshDesc ( SoftBodyParameters& params,
                                                     NxVec3*& vertices_ptr, NxU32 *&indices_ptr )
{
  NxSoftBodyMeshDesc meshDesc;

  const H3DSoftBodyNodeParameters::CoordList& coords= params.getCoords();
  const H3DSoftBodyNodeParameters::IndexList& indices= params.getIndices();
  vertices_ptr = new NxVec3[ coords.size() ];
  indices_ptr = new NxU32[ indices.size() ];
  meshDesc.numVertices            = coords.size();
  meshDesc.numTetrahedra          = indices.size() / 4;
  meshDesc.vertexStrideBytes      = sizeof(NxVec3);
  meshDesc.tetrahedronStrideBytes = 4*sizeof(NxU32);
  meshDesc.vertexMassStrideBytes  = sizeof(NxReal);
  meshDesc.vertexFlagStrideBytes  = sizeof(NxU32);
  meshDesc.vertices               = vertices_ptr;
  meshDesc.tetrahedra             = indices_ptr;
  meshDesc.vertexMasses           = 0;
  meshDesc.vertexFlags            = 0;
  meshDesc.flags                  = 0;

  // Copy vertices
  for ( size_t i= 0; i < coords.size(); ++i )
  {
    Vec3f p ( coords[i] );
    ((NxVec3*)meshDesc.vertices)[i]= NxVec3 ( p.x, p.y, p.z );
  }

  // Copy indices
  for ( size_t i= 0; i < indices.size(); ++i )
  {
    ((NxU32*)meshDesc.tetrahedra)[i]= indices[i];
  }

  cout << "Verts: " << meshDesc.numVertices << "; Indices: " << meshDesc.numTetrahedra*4 << endl;

  return meshDesc;
}

// Cooked a mesh from the mesh description
bool PhysXSoftBody::cookMesh ( NxSoftBodyMeshDesc& desc )
{
  // Store correct number to detect tearing mesh in time
  //mLastNumVertices = desc.numVertices;

  // we cook the mesh on the fly through a memory stream
  // we could also use a file stream and pre-cook the mesh
  MemoryWriteBuffer wb;
  assert(desc.isValid());
  bool success = CookSoftBodyMesh(desc, wb);

  if (!success) 
    return false;

  MemoryReadBuffer rb(wb.data);
  softBodyMesh= scene->getPhysicsSDK().createSoftBodyMesh(rb);
  return true;
}

// Allocate memory to recieve mesh data from the simulation engine
void PhysXSoftBody::allocateReceiveBuffers( size_t numVertices, size_t numTriangles)
{
  mMaxVertices= 4 * numVertices;
  mMaxIndices= 4 * numTriangles;

  if (mVertexRenderBuffer == NULL)
  {
    // Allocate Render Buffer for Vertices if it hasn't been done before
    mVertexRenderBuffer = new RenderBufferVertexElement[mMaxVertices];
    memset(mVertexRenderBuffer, 0, sizeof(RenderBufferVertexElement) * mMaxVertices);
  }

  if (mIndexRenderBuffer == NULL)
  {
    // Allocate Render Buffer for Indices if it hasn't been done before
    mIndexRenderBuffer = new NxU32[mMaxIndices];
    memset(mIndexRenderBuffer, 0, sizeof(NxU32) * mMaxIndices);
  }

  receiveBuffers.verticesPosBegin         = &(mVertexRenderBuffer[0].position.x);
  receiveBuffers.verticesNormalBegin      = &(mVertexRenderBuffer[0].normal.x);
  receiveBuffers.verticesPosByteStride    = sizeof(RenderBufferVertexElement);
  receiveBuffers.verticesNormalByteStride = sizeof(RenderBufferVertexElement);
  receiveBuffers.maxVertices              = mMaxVertices;
  receiveBuffers.numVerticesPtr           = &mNumVertices;

  // the number of triangles is constant, even if the cloth is torn
  NxU32 maxIndices = 3*numTriangles;
  receiveBuffers.indicesBegin             = mIndexRenderBuffer;
  receiveBuffers.indicesByteStride        = sizeof(NxU32);
  receiveBuffers.maxIndices               = maxIndices;
  receiveBuffers.numIndicesPtr            = &mNumIndices;

  receiveBuffers.dirtyBufferFlagsPtr = &mMeshDirtyFlags;
  waitForNonZeromNumVertices = mNumVertices > 0;

  // init the buffers in case we want to draw the mesh 
  // before the SDK as filled in the correct values
  mMeshDirtyFlags = 0;
  mNumVertices = 0;
  mNumIndices = 0;
}

void PhysXSoftBody::applyDampingParameters ( DampingParameters& dampingParams ) {
  switch ( dampingParams.getUnitType() ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->setDampingCoefficient ( dampingParams.getValuePerUnit(0) );
    break;
  default:
    Console(4) << "Warning: Damping unit type is not supported!" << endl;
  };
}

void PhysXSoftBody::applyFrictionParameters ( FrictionParameters& frictionParams ) {
  switch ( frictionParams.getUnitType() ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->setFriction ( frictionParams.getValuePerUnit(0) );
    break;
  default:
    Console(4) << "Warning: Friction unit type is not supported!" << endl;
  };
}

void PhysXSoftBody::applyStiffnessParameters ( StiffnessParameters& stiffnessParams ) {
  if ( stiffnessParams.haveUnitType() ) {
    stiffnessUnitType= stiffnessParams.getUnitType();
  }

  switch ( stiffnessUnitType ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    softBody->setStretchingStiffness ( stiffnessParams.getValuePerUnit(0)*100 );
    break;
  default:
    Console(4) << "Warning: Stiffness unit type is not supported!" << endl;
  };
}

PhysXCloth::PhysXCloth ( PhysicsEngineParameters::ClothParameters& params,
                         PhysXCallbacks::PhysXSpecificData& physx_data ) :
cloth ( NULL ), 
clothMesh ( NULL ),
PhysXH3DSoftBody ( physx_data ),
stiffnessUnitType ( PhysicsEngineParameters::MaterialPropertyParameters::UNIT_UNIFORM )
{
  cout << "Creating PhysX cloth" << endl;

  scene= physx_data.scene;

  NxClothDesc clothDesc;

  cout << "Generate the mesh descrition" << endl;

  NxVec3* vertices_ptr = NULL;
  NxU32 * indices_ptr = NULL;
  // Generate the mesh descrition
  NxClothMeshDesc meshDesc= generateMeshDesc ( params,
                                               vertices_ptr,
                                               indices_ptr );

  // Cook the mesh
  cout << "Cooking the mesh descrition" << endl;
  if ( !cookMesh ( meshDesc ) ) {
    cerr << "FAILED to cook mesh" << endl;
  } else {

    // Allocate recieve buffers
    cout << "Allocate recieve buffers" << endl;
    allocateReceiveBuffers(meshDesc.numVertices, meshDesc.numTriangles);

    clothDesc.clothMesh= clothMesh;
    clothDesc.meshData= receiveBuffers;

    // Create the soft body
    cout << "Creating cloth object" << endl;
    cloth= scene->createCloth ( clothDesc );

    cout << "All done" << endl;

    setParameters ( params );
  }
  delete []vertices_ptr;
  delete []indices_ptr;
}

// Destructor
PhysXCloth::~PhysXCloth ()
{
  // Release soft body
  if ( cloth )
    scene->releaseCloth ( *cloth );

  // Release mesh data

  // Deallocate recieve buffers
  free( mVertexRenderBuffer );
  free( mIndexRenderBuffer );
}

// Get current properties of the simulation cloth
void PhysXCloth::getParameters ( H3DSoftBodyNodeParameters& params )
{
  bool get_vertices = true;
  if( waitForNonZeromNumVertices ) {
    if( mNumVertices > 0 ) waitForNonZeromNumVertices = false;
    else get_vertices = false;
  }
  if ( params.haveGeometry() && get_vertices )
  {
    H3DSoftBodyNodeParameters::IndexList indices;
    H3DSoftBodyNodeParameters::CoordList vertices;

    for ( size_t i= 0; i < mNumVertices; ++i )
    {
      RenderBufferVertexElement e= mVertexRenderBuffer[i];
      vertices.push_back ( Vec3f ( e.position.x, e.position.y, e.position.z ) );
    }

    for ( size_t i= 0; i < mNumIndices; ++i )
    {
      indices.push_back ( mIndexRenderBuffer[i] );
    }

    params.setCoords ( vertices );
    params.setIndices ( indices );
  }
}

void PhysXCloth::setParameters ( H3DSoftBodyNodeParameters& params ) {

  if ( params.havePhysicsMaterial() &&
      params.haveH3DPhysicsMaterialParameters()) {
    H3DPhysicsMaterialParameters * matPar = 
        params.getH3DPhysicsMaterialParameters();
    if( matPar->haveMass() &&
        matPar->haveMassParameters() ) {
    }
    if( matPar->haveStiffness() &&
        matPar->haveStiffnessParameters() ) {
      applyStiffnessParameters ( *matPar->getStiffnessParameters() );
    }
    if( matPar->haveDamping() &&
          matPar->haveDampingParameters() ) {
      applyDampingParameters ( *matPar->getDampingParameters() );
    }
    if( matPar->haveFriction() &&
        matPar->haveFrictionParameters() ) {
      applyFrictionParameters ( *matPar->getFrictionParameters() );
    }
  }

  // Fixed vertices
  /*if ( params.haveFixedVertices() ) {
    fixVertices ( params );
  }*/
}

void PhysXCloth::applyExternalForces ( H3DSoftBodyNodeParameters& bodyParameters ) {

  // Apply user defined external forces
  const H3DSoftBodyNodeParameters::VertexForceList& vertexForces= 
    bodyParameters.getVertexForces();
  for ( H3DSoftBodyNodeParameters::VertexForceList::const_iterator i= vertexForces.begin();
    i != vertexForces.end(); ++i ) {
      cloth->addForceAtVertex ( toNxVec3 ( (*i).second.force ), (*i).second.index );
  }

  // Apply manipulation external forces
  const H3DSoftBodyNodeParameters::VertexForceList& manipulationForces= 
    bodyParameters.getManipulationForces();
  for ( H3DSoftBodyNodeParameters::VertexForceList::const_iterator i= manipulationForces.begin();
    i != manipulationForces.end(); ++i ) {
      cloth->addForceAtVertex ( toNxVec3 ( (*i).second.force ), (*i).second.index );
  }
}

// Generates a physX cloth mesh description from the specified geometry
NxClothMeshDesc PhysXCloth::generateMeshDesc ( ClothParameters& params,
                                               NxVec3*& vertices_ptr,
                                               NxU32 *&indices_ptr )
{
  NxClothMeshDesc meshDesc;

  const H3DSoftBodyNodeParameters::CoordList& coords= params.getCoords();
  const H3DSoftBodyNodeParameters::IndexList& indices= params.getIndices();

  vertices_ptr = new NxVec3[ coords.size() ];
  indices_ptr = new NxU32[ indices.size() ];
  meshDesc.numVertices            = coords.size();
  meshDesc.numTriangles            = indices.size() / 3;
  meshDesc.pointStrideBytes       = sizeof(NxVec3);
  meshDesc.triangleStrideBytes    = 3*sizeof(NxU32);
  meshDesc.vertexMassStrideBytes  = sizeof(NxReal);
  meshDesc.vertexFlagStrideBytes  = sizeof(NxU32);
  meshDesc.points                  = vertices_ptr;
  meshDesc.triangles              = indices_ptr;
  meshDesc.vertexMasses           = 0;
  meshDesc.vertexFlags            = 0;
  meshDesc.flags                  = NX_CLOTH_MESH_WELD_VERTICES;
  meshDesc.weldingDistance        = 0.0001f;

  // Copy vertices
  for ( size_t i= 0; i < coords.size(); ++i )
  {
    Vec3f p ( coords[i] );
    ((NxVec3*)meshDesc.points)[i]= NxVec3 ( p.x, p.y, p.z );
  }

  // Copy indices
  for ( size_t i= 0; i < indices.size(); ++i )
  {
    ((NxU32*)meshDesc.triangles)[i]= indices[i];
  }

  cout << "Verts: " << meshDesc.numVertices << "; Tris: " << meshDesc.numTriangles << endl;

  return meshDesc;
}

// Cooked a mesh from the mesh description
bool PhysXCloth::cookMesh ( NxClothMeshDesc& desc )
{
  // Store correct number to detect tearing mesh in time
  //mLastNumVertices = desc.numVertices;

  // we cook the mesh on the fly through a memory stream
  // we could also use a file stream and pre-cook the mesh
  MemoryWriteBuffer wb;
  assert(desc.isValid());
  bool success = CookClothMesh(desc, wb);

  if (!success) 
    return false;

  MemoryReadBuffer rb(wb.data);
  clothMesh= scene->getPhysicsSDK().createClothMesh(rb);
  return true;
}

// Allocate memory to recieve mesh data from the simulation engine
void PhysXCloth::allocateReceiveBuffers( size_t numVertices, size_t numTriangles)
{
  mMaxVertices= 3 * numVertices;
  mMaxIndices= 3 * numTriangles;

  if (mVertexRenderBuffer == NULL)
  {
    // Allocate Render Buffer for Vertices if it hasn't been done before
    mVertexRenderBuffer = new RenderBufferVertexElement[mMaxVertices];
    memset(mVertexRenderBuffer, 0, sizeof(RenderBufferVertexElement) * mMaxVertices);
  }

  if (mIndexRenderBuffer == NULL)
  {
    // Allocate Render Buffer for Indices if it hasn't been done before
    mIndexRenderBuffer = new NxU32[mMaxIndices];
    memset(mIndexRenderBuffer, 0, sizeof(NxU32) * mMaxIndices);
  }

  receiveBuffers.verticesPosBegin         = &(mVertexRenderBuffer[0].position.x);
  receiveBuffers.verticesNormalBegin      = &(mVertexRenderBuffer[0].normal.x);
  receiveBuffers.verticesPosByteStride    = sizeof(RenderBufferVertexElement);
  receiveBuffers.verticesNormalByteStride = sizeof(RenderBufferVertexElement);
  receiveBuffers.maxVertices              = mMaxVertices;
  receiveBuffers.numVerticesPtr           = &mNumVertices;

  // the number of triangles is constant, even if the cloth is torn
  NxU32 maxIndices = 3*numTriangles;
  receiveBuffers.indicesBegin             = mIndexRenderBuffer;
  receiveBuffers.indicesByteStride        = sizeof(NxU32);
  receiveBuffers.maxIndices               = maxIndices;
  receiveBuffers.numIndicesPtr            = &mNumIndices;

  receiveBuffers.dirtyBufferFlagsPtr = &mMeshDirtyFlags;
  waitForNonZeromNumVertices = mNumVertices > 0;

  // init the buffers in case we want to draw the mesh 
  // before the SDK as filled in the correct values
  mMeshDirtyFlags = 0;
  mNumVertices = 0;
  mNumIndices = 0;
}

void PhysXCloth::fixVertices ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params ) {
  for ( H3DSoftBodyNodeParameters::IndexList::iterator i= fixedVertices.begin();
        i != fixedVertices.end(); ++i ) {
    cloth->freeVertex ( *i );
  }

  /*fixedVertices= params.getFixedVertices();

  for ( H3DSoftBodyNodeParameters::IndexList::iterator i= fixedVertices.begin();
        i != fixedVertices.end(); ++i ) {
    if ( params.haveCoords() ) {
      cloth->attachVertexToGlobalPosition ( *i, toNxVec3 ( params.getCoords()[*i] ) );
    } else {
      cloth->attachVertexToGlobalPosition ( *i, mVertexRenderBuffer[*i].position );
    }
  }*/
}

void PhysXCloth::applyDampingParameters ( DampingParameters& dampingParams ) {
  switch ( dampingParams.getUnitType() ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    cloth->setDampingCoefficient ( dampingParams.getValuePerUnit(0) );
    break;
  default:
    Console(4) << "Warning: Damping unit type is not supported!" << endl;
  };
}

void PhysXCloth::applyFrictionParameters ( FrictionParameters& frictionParams ) {
  switch ( frictionParams.getUnitType() ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    cloth->setFriction ( frictionParams.getValuePerUnit(0) );
    break;
  default:
    Console(4) << "Warning: Friction unit type is not supported!" << endl;
  };
}

void PhysXCloth::applyStiffnessParameters ( StiffnessParameters& stiffnessParams ) {
  if ( stiffnessParams.haveUnitType() ) {
    stiffnessUnitType= stiffnessParams.getUnitType();
  }

  switch ( stiffnessUnitType ) {
  case MaterialPropertyParameters::UNIT_UNIFORM:
    cloth->setStretchingStiffness ( stiffnessParams.getValuePerUnit(0)*100 );
    break;
  default:
    Console(4) << "Warning: Stiffness unit type is not supported!" << endl;
  };
}

PeriodicThread::CallbackCode PhysXCallbacks::initEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread * >( data );
  PhysXCallbacks::PhysXSpecificData *physx_data = 
    new PhysXCallbacks::PhysXSpecificData;
  pt->setEngineSpecificData( physx_data );

  // Initialise the SDK
  physx_data->sdk = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
  if ( !physx_data->sdk ) { 
    Console(4) << "Error: Unable to initialize the PhysX SDK" << endl; 
    return PeriodicThread::CALLBACK_DONE;
  }

  // Initialise cooking
  // This is required to process the soft body meshes
  if ( !InitCooking () ) {
    Console(4) << "Error: Unable to initialize the Cooking PhysX SDK" << endl; 
    return PeriodicThread::CALLBACK_DONE;
  }

  physx_data->scene = physx_data->sdk->createScene( NxSceneDesc() );

  // Default material
  GlobalContactParameters contactParams= pt->getGlobalContactParameters();
  NxMaterialDesc materialDesc;
  materialDesc.restitution = contactParams.bounce;    
  materialDesc.staticFriction = contactParams.friction_coefficients.x;    
  materialDesc.dynamicFriction = contactParams.friction_coefficients.y;
  physx_data->default_material= physx_data->scene->createMaterial(materialDesc);

  // Add callback for contact reporting
  physx_data->scene->setUserContactReport ( &physx_data->contactReport );

  return PeriodicThread::CALLBACK_DONE;
}


PeriodicThread::CallbackCode PhysXCallbacks::deInitEngine( void *data ) {
  PhysicsEngineThread *pt = static_cast< PhysicsEngineThread *>( data );
  PhysXSpecificData *physx_data = static_cast< PhysXSpecificData * >( pt->getEngineSpecificData() );
  if ( physx_data->sdk != NULL ){
    physx_data->scene->releaseMaterial ( *physx_data->default_material );
    physx_data->sdk->releaseScene( *(physx_data->scene) );
    physx_data->scene = NULL;
    NxReleasePhysicsSDK( physx_data->sdk );
    physx_data->sdk = NULL;

    // Deinitialise cooking
    // This is required to process the soft body meshes
    CloseCooking();
  }
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysXCallbacks::doSimulationSteps(void *data) {
  PhysicsEngineThread * physics_thread = 
    static_cast< PhysicsEngineThread * >( data );

  TimeStamp t;

  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(physics_thread->getEngineSpecificData());
  if( !physx_data->sdk )
    return PeriodicThread::CALLBACK_CONTINUE;

  GlobalContactParameters contactParams= physics_thread->getGlobalContactParameters();

  physx_data->default_material->setRestitution ( contactParams.bounce );
  physx_data->default_material->setStaticFriction ( contactParams.friction_coefficients.x );
  physx_data->default_material->setDynamicFriction ( contactParams.friction_coefficients.y );

  physx_data->contactReport.clearContacts ();

  float stepSize = physics_thread->getStepSize();
  int iterations = 8;

  for( map< NxJoint *, NxReal >::iterator i = physx_data->slider_joint_forces.begin();
      i != physx_data->slider_joint_forces.end(); ++i ) {
    NxVec3 axis = (*i).first->getGlobalAxis();
    axis.normalize();
    NxActor * a1;
    NxActor * a2;
    (*i).first->getActors( &a1, &a2 );
    NxVec3 the_force = axis * (*i).second;
    if( a1 ) {
      a1->addForce( the_force );
    }
    if( a2 ) {
      a2->addForce( -the_force );
    }
  }
  //addRigidBodyForces();
  physx_data->scene->setTiming(stepSize, iterations, NX_TIMESTEP_FIXED);
  physx_data->scene->simulate(stepSize);
  physx_data->scene->flushStream();
  physx_data->scene->fetchResults(NX_RIGID_BODY_FINISHED, true);

  physics_thread->setLastLoopTime( TimeStamp() - t );

  return PeriodicThread::CALLBACK_CONTINUE;
}

PeriodicThread::CallbackCode PhysXCallbacks::synchroniseWithSceneGraph(void *data) {
  return PeriodicThread::CALLBACK_DONE;
}

// setWorldParams()
PeriodicThread::CallbackCode PhysXCallbacks::setWorldParameters( void *data ) {
  PhysicsEngineParameters::WorldParameters *params = 
    static_cast< PhysicsEngineParameters::WorldParameters *>( data );
  PhysXSpecificData *physx_data = static_cast< PhysXSpecificData * >( 
    params->getEngine()->getEngineSpecificData() );
  if( !physx_data->sdk )
    return PeriodicThread::CALLBACK_DONE;

  // gravity
  if( params->haveGravity() ) {
    Vec3f g = params->getGravity(); 
    physx_data->scene->setGravity( NxVec3( g.x, g.y, g.z) ); 
  }
  // disabling bodies
  if( params->haveAutoDisable() || 
      params->haveDisableLinearSpeed() || 
      params->haveDisableAngularSpeed() ){
    list< H3DBodyId > bodies;
    params->getEngine()->getCurrentRigidBodies( bodies );
    for( list< H3DBodyId >::iterator i = bodies.begin(); 
         i != bodies.end(); ++i ) {
      PhysicsEngineParameters::RigidBodyParameters p;
      params->getEngine()->getRigidBodyParameters( *i, p );
      if( !p.getAutoDisable() ) {
        // Set the thresholds for all rigid bodies that do not have auto_disable
        // set in its own local parameters.
        if( params->getAutoDisable() ) {
          (( NxActor* )(*i))->setSleepLinearVelocity( params->getDisableLinearSpeed() );
          (( NxActor* )(*i))->setSleepAngularVelocity( params->getDisableAngularSpeed() );
        } else {
          // Case when auto disable is false both locally and globally. PhysX 
          // internally puts bodies to sleep with its default sleep thresholds,
          // so in PhysX bodies cannot be completely disabled.
          (( NxActor* )(*i))->setSleepLinearVelocity( -1 );
          (( NxActor* )(*i))->setSleepAngularVelocity( -1 );
        }
      }
    }
  }

  if( params->haveConstantForceMix() ) {
  }
  // contactSurfaceThickness
  if( params->haveContactSurfaceThickness() ) {
    NxActor **actors = physx_data->scene->getActors();
    NxU32 n_actors = physx_data->scene->getNbActors();
    for( NxU32 i = 0; i < n_actors; ++i ) {
      NxU32 n_shapes = actors[i]->getNbShapes();
      NxShape *const *shape = actors[i]->getShapes();
      for( NxU32 j = 0; j < n_shapes; ++j ) {
        shape[j]->setSkinWidth(params->getContactSurfaceThickness());
      }
    }    
  }
  // disableTime
  if( params->haveDisableTime() ) {
    //params->getDisableTime();
  }
  // errorCorrection
  if( params->haveErrorCorrection() ) {
    //params->getErrorCorrection();
  }
  // iterations
  if( params->haveIterations() ) {
    //params->getIterations();
  }  
  // massCorrectionSpeed
  if( params->haveMaxCorrectionSpeed() ) {
    //params->getMaxCorrectionSpeed();
  }
  delete params;
  return PeriodicThread::CALLBACK_DONE;
}


// addRigidBody()
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::addRigidBody( void *data ) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());

  // Here we create an NxActor with dummy values (PhysX requires
  // mass and inertia to successfully create a dynamic body), 
  // before assigning correct values with setRigidBodyParameters.
  // Note that all bodies will be dynamic on creation. If RigidBody 
  // is fixed we will later raise a PhysX kinematic body flag.
  NxVec3 inertia;
  NxComputeBoxInertiaTensor(inertia, 0.5f, 0.05f, 0.2f, 0.05f);
  NxBodyDesc body_desc;
  body_desc.mass = 1.0;
  body_desc.massSpaceInertia = inertia;
  NxActorDesc actor_desc;
  actor_desc.body = &body_desc;
  actor_desc.density = 0.0;

  NxActor * actor;
  actor = PhysXCallbacks::createActor( physx_data, actor_desc );
  params->setBodyId( (H3DBodyId) actor );

  bodyShapeMap.insert( make_pair( params->getBodyId(), vector< ShapeInfo * >() ) );
  PhysXCallbacks::setRigidBodyParameters( params, physx_data ); 
  return PeriodicThread::CALLBACK_DONE;
}


// removeRigidBody()
PeriodicThread::CallbackCode PhysXCallbacks::removeRigidBody( void *data ) {
  RigidBodyParameters *params = static_cast< RigidBodyParameters * >( data );
  PhysXCallbacks::PhysXSpecificData *physx_data = 
    static_cast< PhysXCallbacks::PhysXSpecificData *>( 
    params->getEngine()->getEngineSpecificData() );
  list< H3DBodyId > bodies;
  params->getEngine()->getCurrentRigidBodies( bodies );
  BodyShapeMap::iterator bsmi = bodyShapeMap.find( params->getBodyId() );
  // Only release actor if not found in bodyShapeMap since otherwise the actor
  // might be needed in removecollision.
  if( bsmi == bodyShapeMap.end() ) {
    for( list< H3DBodyId >::iterator i = bodies.begin(); 
           i != bodies.end(); ++i ) {
      if ( *i == params->getBodyId() ) {
        PhysXCallbacks::releaseActor( physx_data, (NxActor*)params->getBodyId() );
      }
    }
  }
  bodyShapeMap.erase( bsmi );
  return PeriodicThread::CALLBACK_DONE;
}

void PhysXCallbacks::setRigidBodyParameters(
    PhysicsEngineParameters::RigidBodyParameters *params,
    PhysXSpecificData *physx_data ) {
  NxActor * actor = (NxActor *) params->getBodyId();
  // disabling params - linear vel, angular vel, time
  if ( params->haveAutoDisable() ) {
    if ( params->getAutoDisable() ) {
      actor->setSleepLinearVelocity( params->getDisableLinearSpeed() );
      actor->setSleepAngularVelocity( params->getDisableAngularSpeed() );
    } else {
      actor->setSleepLinearVelocity( -1 );
      actor->setSleepAngularVelocity( -1 );
    }
  }

  // NOTE: autoDamp, angular/linear damping are handled in saveUpdateRate of PET
  // actor->setAngularDamping( params->getAngularDampingFactor() );
  // actor->setLinearDamping( params->getLinearDampingFactor() );

  // useGlobalGravity
  if ( params->haveUseGlobalGravity() ) {
    if ( params->getUseGlobalGravity() && actor->readBodyFlag( NX_BF_DISABLE_GRAVITY ) ) {
      actor->clearBodyFlag( NX_BF_DISABLE_GRAVITY );
    } else if ( !params->getUseGlobalGravity() && !actor->readBodyFlag( NX_BF_DISABLE_GRAVITY ) ) {
      actor->raiseBodyFlag( NX_BF_DISABLE_GRAVITY );
    }
  }
  // mass, massDensityModel, inertia and centerOfMass
  if( params->haveMass() || params->haveMassDensityModel() ||
      params->haveInertia() || params->haveCenterOfMass() ) {
    Node *n = params->getMassDensityModel();
    if ( n != NULL ) {
      if( Box *box = dynamic_cast< Box* >( n ) ) {
        Vec3f size = box->size->getValue();
        NxVec3 inertia;
        NxComputeBoxInertiaTensor( inertia, params->getMass(), size.x, size.y, size.z);
        actor->setMassSpaceInertiaTensor( inertia );
      } else if ( Sphere *sph = dynamic_cast< Sphere* >( n ) ){
        NxVec3 inertia;
        NxComputeSphereInertiaTensor( inertia, params->getMass(), sph->radius->getValue(), false );
        actor->setMassSpaceInertiaTensor( inertia );
      }
    } else {
      // massDensityModel is NULL, use user-given values instead
      actor->setMass( params->getMass() );
      actor->setCMassOffsetLocalPosition( NxVec3( params->getCenterOfMass().x,
                                                  params->getCenterOfMass().y,
                                                  params->getCenterOfMass().z ) );
    }
  }
  // start position
  if ( params->haveStartPosition() ) {
    Vec3f start_pos = params->getStartPosition();
    //cerr << "in PhysX setRBP: " << start_pos.x << " " << start_pos.y << " " << start_pos.z << endl;
    actor->setGlobalPosition( NxVec3(start_pos.x, start_pos.y, start_pos.z) );
  }
  // start orientation
  if ( params->haveStartOrientation() ) {
    Rotation start_ort = params->getStartOrientation();
    NxVec3 r( start_ort.axis.x, start_ort.axis.y, start_ort.axis.z );
    NxQuat quat;
    quat.fromAngleAxisFast( start_ort.angle, r);
    actor->setGlobalOrientation( NxMat33( quat ) );
  }

  if ( params->havePosition() || params->haveOrientation() ) {
    actor->raiseBodyFlag ( NX_BF_KINEMATIC );

    if ( params->havePosition() ) {
      actor->moveGlobalPosition ( toNxVec3 ( params->getPosition() ) );
    }
    if ( params->haveOrientation() ) {
      /*Rotation orn = params->getOrientation();
      NxVec3 r( orn.axis.x, orn.axis.y, orn.axis.z );
      NxQuat quat;
      quat.fromAngleAxisFast( orn.angle, r);
      actor->moveGlobalOrientation( NxMat33( quat ) );*/
    }
  }

  if ( params->haveFixed() ) {
    if ( params->getFixed() && !actor->readBodyFlag( NX_BF_KINEMATIC ) ) { 
      actor->raiseBodyFlag( NX_BF_KINEMATIC );
    } else if ( !params->getFixed() && actor->readBodyFlag( NX_BF_KINEMATIC ) ) {
      actor->clearBodyFlag( NX_BF_KINEMATIC );
    }
    if( !actor->userData ) {
      actor->userData = new RigidBodyInfo;
    }
    static_cast< RigidBodyInfo * >(actor->userData)->fixed = params->getFixed();
  }
  // geometry
  // current assumption: added geometry will not be removed from rigid body
  // so no provisions made here to call removeShape from rigid body actor, only createShape
  if( params->haveGeometry() ) {
    const vector< H3DCollidableId > geom_ids = params->getGeometry();
    if ( !geom_ids.empty() ) {
      // release all existing shapes from the actor
      // create new actor, shape to replace the ShapeInfo (CollidableShape) in bodyshapemap( this_body_id )
      // erase the entry bodyShapeMap
      vector< ShapeInfo* > curr_shape_infos = bodyShapeMap.find(params->getBodyId())->second;
      for( vector< ShapeInfo* >::const_iterator si = curr_shape_infos.begin();
        si != curr_shape_infos.end(); ++si ) {
        NxActorDesc new_actor_desc;       
        if ( (*si)->shape->isBox() ) {
          NxBoxShape * box = static_cast< NxBoxShape* >( (*si)->shape );
          NxBoxShapeDesc box_desc;
          box->saveToDesc( box_desc );
          new_actor_desc.shapes.push_back( &box_desc );
        } else if ( (*si)->shape->isSphere() ) {
          NxSphereShape * sph = static_cast< NxSphereShape* >( (*si)->shape );
          NxSphereShapeDesc sph_desc;
          sph->saveToDesc( sph_desc );
          new_actor_desc.shapes.push_back( &sph_desc );
        }
        // release old shape
        actor->releaseShape( *(*si)->shape );
        // create new actor and shape for the ShapeInfo
        NxActor * new_actor = PhysXCallbacks::createActor( physx_data,
                                                           new_actor_desc,
                                                           (*si)->actor );
        assert( new_actor != NULL );
        (*si)->shape = *(new_actor->getShapes());
        //cerr << "New shape added to shapeInfo in curr bodyShapeMap: " << (*si)->shape << endl;
        (*si)->actor = new_actor;

        // Update back ptr (used, e.g., to report collisions)
        (*si)->shape->userData= (*si);
      }
      // erase the entry in bodyShapeMap
      if ( bodyShapeMap.find(params->getBodyId()) != bodyShapeMap.end() )
        bodyShapeMap.erase( params->getBodyId() );


      // update actor with new shapes and replace shape map
      vector< ShapeInfo* > new_shape_infos;
      for( vector< H3DCollidableId >::const_iterator i = geom_ids.begin(); i != geom_ids.end(); ++i ) {
        H3DCollidableId shape_id = (H3DCollidableId) *i;
        ShapeInfo * shape_info = (shapeInfoMap.find(shape_id))->second;
        NxShape * new_shape = NULL;
        //cerr << "Got shape: " << shape_info->shape << endl;
        // save the shape desc
        // use current actor to create shape with desc 
        // delete old actor and old shape
        // update shape_info
        // insert shape_info to list of shapes associated with this actor
        // map the H3DBodyId of this actor to this list OR add new entry (after old is erased)
        if ( shape_info->shape->isBox() ) {
          NxBoxShape * box = static_cast< NxBoxShape* >( shape_info->shape );
          NxBoxShapeDesc box_desc;
          box->saveToDesc( box_desc );
          new_shape = actor->createShape( box_desc );
        } else if ( shape_info->shape->isSphere() ) {
          NxSphereShape * sph = static_cast< NxSphereShape* >( shape_info->shape );
          NxSphereShapeDesc sph_desc;
          sph->saveToDesc( sph_desc );
          new_shape = actor->createShape( sph_desc );
        } else {
          continue;
        }
        assert( new_shape != NULL );
        PhysicsEngineParameters::WorldParameters world_params =
          params->getEngine()->getWorldParameters();
        new_shape->setSkinWidth( world_params.getContactSurfaceThickness() );
        shape_info->actor->releaseShape( *(shape_info->shape) );
        PhysXCallbacks::releaseActor( physx_data, shape_info->actor, false );
        shape_info->actor = actor;
        shape_info->shape = new_shape;
        //cerr << "New shape: " << shape_info->shape << endl;
        shape_info->shape->setMaterial(physx_data->default_material->getMaterialIndex());
        new_shape_infos.push_back( shape_info );
      } // end for
      bodyShapeMap.insert( make_pair(params->getBodyId(), new_shape_infos) );
    } // end if geom_ids not empty
  }
}

NxActor * PhysXCallbacks::createActor( PhysXSpecificData *physx_data,
                                       NxActorDesc &actor_desc,
                                       NxActor *replace_actor ) {
  NxActor * actor= physx_data->scene->createActor( actor_desc );
  actor->setContactReportFlags ( NX_NOTIFY_ALL );
  if( replace_actor )
    actor->userData = replace_actor->userData;
  return actor;
}

void PhysXCallbacks::releaseActor( PhysXSpecificData *physx_data,
                                   NxActor *actor,
                                   bool release_user_data ) {
  if( release_user_data && actor->userData )
    delete actor->userData;
  physx_data->scene->releaseActor( *actor );
}

// setRigidBodyParams()
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::setRigidBodyParameters( void *data ) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  PhysXCallbacks::setRigidBodyParameters( params, physx_data );
  delete params;
  return PeriodicThread::CALLBACK_DONE;
}


// getRigidBodyParams()
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::getRigidBodyParameters( void *data ) {
  PhysicsEngineParameters::RigidBodyParameters *params = 
    static_cast< PhysicsEngineParameters::RigidBodyParameters *>( data );
  NxActor * actor = (NxActor*) params->getBodyId();
  // position
  NxVec3 pos = actor->getGlobalPosition();
  params->setPosition( Vec3f((H3DFloat)pos.x, (H3DFloat)pos.y, (H3DFloat)pos.z) );
  // orientation
  NxQuat quat;
  actor->getGlobalOrientation().toQuat( quat );
  Rotation rot = Rotation(Quaternion(quat.x, quat.y, quat.z, quat.w));  
  params->setOrientation( rot );
  // linear velocity
  NxVec3 lin = actor->getLinearVelocity();
  params->setLinearVelocity( Vec3f(lin.x, lin.y, lin.z) );
  // angular velocity
  NxVec3 ang = actor->getAngularVelocity();
  params->setAngularVelocity( Vec3f(ang.x, ang.y, ang.z) );
  return PeriodicThread::CALLBACK_DONE;
}


// addShape
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::addCollidable( void *data ) {
  PhysicsEngineParameters::CollidableParameters *data_params = 
    static_cast< PhysicsEngineParameters::CollidableParameters *>( data );
  PhysicsEngineParameters::ShapeParameters *params =
    dynamic_cast< PhysicsEngineParameters::ShapeParameters *>( data_params );
  if( !params ) {
    Console(3) << "Warning: Only CollidableShapes are supported by PhysX." << endl;
    return PeriodicThread::CALLBACK_DONE;
  }
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());

  NxActorDesc actor_desc;
  if ( Sphere * sph = dynamic_cast< Sphere *>(params->getShape()) ) {
    NxSphereShapeDesc desc;
    desc.radius = (NxReal) sph->radius->getValue();
    actor_desc.shapes.pushBack( &desc );
  } else {
    NxBoxShapeDesc desc;
    if ( Box * box = dynamic_cast< Box*>(params->getShape()) ) {
      Vec3f size = box->size->getValue();
      desc.dimensions = NxVec3( (NxReal)0.5*size.x, (NxReal)0.5*size.y, (NxReal)0.5*size.z );
    } else {
      Console(3) << "Warning: Only Boxes and Sphere collidable shapes are supported by PhysX."
                 << " Current shape is of type ";
      if( params->haveShape() )
        Console(3) << params->getShape()->getTypeName();
      else
        Console(3) << " NULL ";
      Console(3) << " undesired behaviour will occur." << endl;
      if( H3DBoundedObject * bound = dynamic_cast< H3DBoundedObject * >( params->getShape() ) ) {
        if( BoxBound * box_bound = dynamic_cast< BoxBound * > ( bound->bound->getValue() ) ) {
          Vec3f size = box_bound->size->getValue();
          desc.dimensions = NxVec3( (NxReal)0.5*size.x, (NxReal)0.5*size.y, (NxReal)0.5*size.z );
        } else {
          // use ad hoc chosen shape. The reason for this is that if no ShapeInfo is created
          // and correctly used then the application will crash.
          desc.dimensions = NxVec3( 0.5, 0.5, 0.5 );
        }
      }
    }
    actor_desc.shapes.pushBack( &desc );
  }
  ShapeInfo * shape_info = new ShapeInfo;
  params->setCollidableId( shape_info->id );

  shape_info->actor = PhysXCallbacks::createActor( physx_data, actor_desc );
  shape_info->shape = *(shape_info->actor->getShapes());
  
  // Set back ptr (used, e.g., to report collisions)
  shape_info->shape->userData= shape_info;

  shapeInfoMap.insert( make_pair( shape_info->id, shape_info ) );
  PhysicsEngineParameters::WorldParameters world_params =
          params->getEngine()->getWorldParameters();
  shape_info->shape->setSkinWidth( world_params.getContactSurfaceThickness() );
  PhysXCallbacks::setCollidableParameters( params );
  return PeriodicThread::CALLBACK_DONE;
}


// removeShape()
PeriodicThread::CallbackCode PhysXCallbacks::removeCollidable( void *data ) {
  ShapeParameters *params = static_cast< ShapeParameters * >( data );
  PhysXCallbacks::PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  assert( shapeInfoMap.find(params->getCollidableId()) != shapeInfoMap.end() );
  ShapeInfo * shape_info = (shapeInfoMap.find(params->getCollidableId()))->second;
  // release shape from PhysX
  shape_info->actor->releaseShape( *(shape_info->shape) );
  // release the containing actor from PhysX, only done if it is not found in
  // bodyShapeMap because otherwise the rigid body is still used somehow and the
  // actor should be removed in removeRigidBody.
  if( bodyShapeMap.find( (H3DBodyId)shape_info->actor ) ==
      bodyShapeMap.end() ) {
    int release_actor = 0;
    for( ShapeInfoMap::iterator i = shapeInfoMap.begin(); i != shapeInfoMap.end(); ++i ) {
      if( (H3DBodyId)((*i).second->actor) == (H3DBodyId)shape_info->actor )
        ++release_actor;
      if( release_actor > 1 )
        break;
    }
    if( release_actor < 2 )
      PhysXCallbacks::releaseActor( physx_data, shape_info->actor );
  }
  // delete shape_info from shapeInfoMap
  delete shape_info;
  shapeInfoMap.erase( shapeInfoMap.find(params->getCollidableId()) );
  return PeriodicThread::CALLBACK_DONE;
}

// setShapeParams
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::setCollidableParameters( void *data ) {
  PhysicsEngineParameters::ShapeParameters *params = 
    static_cast< PhysicsEngineParameters::ShapeParameters *>( data );
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());

  ShapeInfo * shape_info = (shapeInfoMap.find(params->getCollidableId()))->second;
  // set position
  const Vec3f &translation = params->getTranslation();
  shape_info->shape->setLocalPosition( NxVec3( (NxReal)translation.x,
                                               (NxReal)translation.y,
                                               (NxReal)translation.z ) );
  // set orientation
  NxQuat quat;
  const Rotation &rotation = params->getRotation();
  NxVec3 axis( (NxReal)rotation.axis.x,
               (NxReal)rotation.axis.y,
               (NxReal)rotation.axis.z );
  quat.fromAngleAxisFast( (NxReal)rotation.angle, axis );
  shape_info->shape->setLocalOrientation( NxMat33(quat) );

  // set enabled
  if ( params->getEnabled() && shape_info->actor->readActorFlag(NX_AF_DISABLE_COLLISION) ) {
    // if enabled true and it was disable flag was riased before
    shape_info->actor->clearActorFlag( NX_AF_DISABLE_COLLISION );
  } else if ( !params->getEnabled() && !shape_info->actor->readActorFlag(NX_AF_DISABLE_COLLISION) ) {
    // if enabled is false and disable flag was not raised before
    shape_info->actor->raiseActorFlag( NX_AF_DISABLE_COLLISION );
  }
  return PeriodicThread::CALLBACK_DONE;
}


// getShapeParams
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::getCollidableParameters( void *data ) {
  PhysicsEngineParameters::ShapeParameters *params = 
    static_cast< PhysicsEngineParameters::ShapeParameters *>( data );
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  
  ShapeInfo * shape_info = (shapeInfoMap.find(params->getCollidableId()))->second;
  NxVec3 pos = shape_info->shape->getLocalPosition();
  params->setTranslation( Vec3f((H3DFloat)pos.x, (H3DFloat)pos.y, (H3DFloat)pos.z) );

  NxQuat quat;
  shape_info->shape->getLocalOrientation().toQuat( quat );
  params->setRotation( Rotation( Quaternion(quat.x, quat.y, quat.z, quat.w) ) );
  return PeriodicThread::CALLBACK_DONE;
}


H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::addGlobalExternalForceAndTorque( void *data ) {
  PhysicsEngineParameters::ExternalForceTorqueParameters *params = 
    static_cast< PhysicsEngineParameters::ExternalForceTorqueParameters *>( data );
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->engine_thread->getEngineSpecificData());

  if( params->force.length() > Constants::f_epsilon || params->torque.length() > Constants::f_epsilon ) {
    if( !((NxActor*)params->body_id)->userData ||
        !(static_cast< RigidBodyInfo * >(((NxActor*)params->body_id)->userData)->fixed) )
      ((NxActor*)params->body_id)->clearBodyFlag ( NX_BF_KINEMATIC );
  }

  ((NxActor*)params->body_id)->addForce(NxVec3(params->force.x, params->force.y, params->force.z),
                                        NX_FORCE,
                                        true );
  ((NxActor*)params->body_id)->addTorque(NxVec3(params->torque.x, params->torque.y, params->torque.z),
                                         NX_FORCE,
                                         true);
  return PeriodicThread::CALLBACK_DONE;
}


// addJoint()
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::addConstraint( void *data ) {
  PhysicsEngineParameters::JointParameters *params = 
    static_cast< PhysicsEngineParameters::JointParameters *>( data );
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  ConstraintInfo * constraint_info = NULL;

  if( params->getType() == "BallJoint" ) {
    constraint_info = attachBallJoint( params );
  }
  if( params->getType() == "DoubleAxisHingeJoint" ) {
    constraint_info = attachDoubleAxisHingeJoint( params );
  }
  if( params->getType() == "MotorJoint" ) {
    constraint_info = attachMotorJoint( params );
  }
  if( params->getType() == "SingleAxisHingeJoint" ) {
    constraint_info = attachSingleAxisHingeJoint( params );
  }  
  if( params->getType() == "SliderJoint" ) {
    constraint_info = attachSliderJoint( params );
  }
  if( params->getType() == "UniversalJoint" ) {
    constraint_info = attachUniversalJoint( params );
  }
  if ( constraint_info ) {
    /*cerr << "PhysX: created joint: " << joint_info->id << endl;
    cerr << "PhysX: createJoint() called, joint created: " << joint_info->joint << endl;*/
    
    if( constraint_info->id != 0 ) {
      params->setConstraintId( constraint_info->id );
      constraintInfoMap.insert( make_pair( constraint_info->id, constraint_info ) );
      PhysXCallbacks::setConstraintParameters( params );
    } else {
      delete constraint_info;
      Console ( 3 ) << "Warning: Constraint type " << params->getType() << " could not be created for some reason. Contact developer!" << endl;
    }
  } else {
    // Constraint creation failed
    Console ( 3 ) << "Warning: Constraint type " << params->getType() << " not supported by PhysX in H3DPhysics!" << endl;
  }

  return PeriodicThread::CALLBACK_DONE;
}

// removeJoint()
PeriodicThread::CallbackCode PhysXCallbacks::removeConstraint( void *data ) {
  ConstraintParameters *params = static_cast< ConstraintParameters *>( data );
  PhysXCallbacks::PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  assert( constraintInfoMap.find(params->getConstraintId()) != constraintInfoMap.end() );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  map< NxJoint *, NxReal >::iterator joint_force_iterator =
    physx_data->slider_joint_forces.find( constraint_info->joint );
  if( joint_force_iterator != physx_data->slider_joint_forces.end() ) {
    physx_data->slider_joint_forces.erase( joint_force_iterator );
  }
  if( constraint_info->joint )
    physx_data->scene->releaseJoint( *(constraint_info->joint) );
  //cerr << "PhysX: removing joint: " << joint_info->id << endl;
  delete constraint_info;
  constraintInfoMap.erase( constraintInfoMap.find(params->getConstraintId()) );
  return PeriodicThread::CALLBACK_DONE;
}

void PhysXCallbacks::setConstraintParameters( PhysicsEngineParameters::JointParameters * params ) {
  if( params->getType() == "BallJoint" ) {
    setBallJointParameters( params );
  }
  if( params->getType() == "DoubleAxisHingeJoint" ) {
    setDoubleAxisHingeJointParameters( params );
  }
  if( params->getType() == "MotorJoint" ) {
    setMotorJointParameters( params );
  }
  if( params->getType() == "SingleAxisHingeJoint" ) {
    setSingleAxisHingeJointParameters( params );
  }
  if( params->getType() == "SliderJoint" ) {
    setSliderJointParameters( params );
  }
  if( params->getType() == "UniversalJoint" ) {
    setUniversalJointParameters( params );
  }
}

// setJointParams
PeriodicThread::CallbackCode PhysXCallbacks::setConstraintParameters( void *data ) {
  JointParameters *params = static_cast< JointParameters *>( data );
  setConstraintParameters( params );
  delete params;
  return PeriodicThread::CALLBACK_DONE;
}


// getJointParams
PeriodicThread::CallbackCode PhysXCallbacks::getConstraintParameters( void *data ) {
  ConstraintParameters *params = static_cast< ConstraintParameters *>( data );
  if( params->getType() == "BallJoint" ) {
    getBallJointParameters( params );
  }
  if( params->getType() == "DoubleAxisHingeJoint" ) {
    getDoubleAxisHingeJointParameters( params );
  }
  if( params->getType() == "MotorJoint" ) {
    getMotorJointParameters( params );
  }
  if( params->getType() == "SingleAxisHingeJoint" ) {
    getSingleAxisHingeJointParameters( params );
  }
  if( params->getType() == "SliderJoint" ) {
    getSliderJointParameters( params );
  }
  if( params->getType() == "UniversalJoint" ) {
    getUniversalJointParameters( params );
  }  
  return PeriodicThread::CALLBACK_DONE;
}

// BallJoint
void PhysXCallbacks::setBallJointParameters( void *data ) {
  BallJointParameters *params = static_cast< BallJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  if( params->haveAnchorPoint() ) {
    const Vec3f &ap = params->getAnchorPoint();
    constraint_info->joint->setGlobalAnchor( NxVec3(ap.x, ap.y, ap.z) );
  }
}

// BallJoint
void PhysXCallbacks::getBallJointParameters( void *data ) {
  BallJointParameters *params = static_cast< BallJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxSphericalJoint * joint = static_cast< NxSphericalJoint *>( constraint_info->joint );
  NxSphericalJointDesc joint_desc;
  joint->saveToDesc( joint_desc );
  if( params->haveBody1AnchorPoint() ) {
    NxVec3 &v = joint_desc.localAnchor[0];
    params->setBody1AnchorPoint( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
  if( params->haveBody2AnchorPoint() ) {
    NxVec3 &v = joint_desc.localAnchor[1];
    params->setBody2AnchorPoint( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
}

// SingleAxisJoint
void PhysXCallbacks::setSingleAxisHingeJointParameters( void *data ) {
  SingleAxisHingeJointParameters *params = static_cast< SingleAxisHingeJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxRevoluteJoint * joint = static_cast< NxRevoluteJoint *>( constraint_info->joint );
  if( params->haveAnchorPoint() ) {
    const Vec3f &ap = params->getAnchorPoint();
    joint->setGlobalAnchor( NxVec3( ap.x, ap.y, ap.z ) );
  }
  if( params->haveAxis() ) {
    const Vec3f &axis = params->getAxis();
    joint->setGlobalAxis( NxVec3( axis.x, axis.y, axis.z ) );
  }
  if( params->haveMinAngle() &&  params->haveMaxAngle() ) {
    NxJointLimitPairDesc limit;
    limit.low.value = params->getMinAngle();
    limit.high.value = params->getMaxAngle();
    joint->setLimits( limit );
  }
  if( params->haveStopBounce() ) {
  }
  if( params->haveStopErrorCorrection() ) {
  }
}


// SingleAxisJoint
void PhysXCallbacks::getSingleAxisHingeJointParameters( void *data ) {
  SingleAxisHingeJointParameters *params = static_cast< SingleAxisHingeJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxRevoluteJoint * joint = static_cast< NxRevoluteJoint *>( constraint_info->joint );
  NxRevoluteJointDesc joint_desc;
  joint->saveToDesc( joint_desc );
  if( params->haveAngle() ) {
    params->setAngle( (H3DFloat)joint->getAngle() );
  }
  if( params->haveAngleRate() ) {
    params->setAngleRate( (H3DFloat)joint->getVelocity() );
  }
  if( params->haveBody1AnchorPoint() ) {
    NxVec3 &v = joint_desc.localAnchor[0];
    params->setBody1AnchorPoint( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
  if( params->haveBody2AnchorPoint() ) {
    NxVec3 &v = joint_desc.localAnchor[1];
    params->setBody2AnchorPoint( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
}


// SliderJoint
void PhysXCallbacks::setSliderJointParameters( void *data ) {
  SliderJointParameters *params = static_cast< SliderJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxCylindricalJoint * joint = static_cast< NxCylindricalJoint *>( constraint_info->joint );
  if( params->haveAxis() ) {
    Vec3f v = params->getAxis();
    joint->setGlobalAxis( NxVec3(v.x, v.y, v.z) );
  }
  if( params->haveMinSeparation() && params->haveMaxSeparation() ) {
    NxActor * a1;
    NxActor * a2;
    joint->getActors( &a1, &a2 );

    RigidBodyParameters body1, body2;
    params->getEngine()->getRigidBodyParameters( (H3DBodyId)a1, body1 );
    const Vec3f &p1 = body1.getPosition();

    // position of body1 in Nxvec3
    NxVec3 p = NxVec3(p1.x, p1.y, p1.z);
    NxVec3 p2( 0, 0, 0 );
    // normalized axis
    NxVec3 n_axis = joint->getGlobalAxis();
    n_axis.normalize();
    if( a2 ) {
      params->getEngine()->getRigidBodyParameters( (H3DBodyId)a2, body2 );
      const Vec3f &p2_tmp = body2.getPosition();
      p2 = NxVec3( p2_tmp.x, p2_tmp.y, p2_tmp.z );
      // center of body2 set as limit point
      joint->setLimitPoint( p2 );
    } else {
      // This is extremely annoying. There is probably some bug in PhysX or some lack of
      // documentation information about how the limit planes work when one of the bodies
      // is NULL. The planes are calculated correctly but when running a scene the result
      // seems to be off by about a quarter of the maxSeparation in both directions
      // of the axis.
      joint->setGlobalAnchor( p );
      p2 = p + n_axis * (NxReal)params->getMaxSeparation();
      joint->setLimitPoint( p2 );
      p = p2;
    }

    joint->addLimitPlane(  joint->getGlobalAxis(), -(NxReal)params->getMaxSeparation()*n_axis + ( p + p2 ) / 2 );
    joint->addLimitPlane( -joint->getGlobalAxis(), -(NxReal)params->getMinSeparation()*n_axis + ( p + p2 ) / 2  );
  }  
  if( params->haveStopBounce() ) {
  }
  if( params->haveStopErrorCorrection() ) {
  }

  // sliderForce
  PhysXSpecificData *physx_data = 
        static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  map< NxJoint *, NxReal >::iterator joint_force_iterator =
    physx_data->slider_joint_forces.find( constraint_info->joint );
  if( params->haveSliderForce() ) {
    H3DFloat slider_force = params->getSliderForce();
    if( slider_force != 0 ) {
      if( joint_force_iterator == physx_data->slider_joint_forces.end() ) {
        physx_data->slider_joint_forces[constraint_info->joint] = slider_force;
      } else {
        (*joint_force_iterator).second = slider_force;
      }
    } else if( joint_force_iterator != physx_data->slider_joint_forces.end() ) {
      physx_data->slider_joint_forces.erase( joint_force_iterator );
    }
  } else if( joint_force_iterator != physx_data->slider_joint_forces.end() ) {
    physx_data->slider_joint_forces.erase( joint_force_iterator );
  }
}


// SliderJoint
void PhysXCallbacks::getSliderJointParameters( void *data ) {
  SliderJointParameters *params = static_cast< SliderJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxCylindricalJoint * joint = static_cast< NxCylindricalJoint *>( constraint_info->joint );
  if( params->haveSeparation() ) {
    NxActor * a1, * a2;
    joint->getActors( &a1, &a2 );
    if( a2 ) {
      NxVec3 v = a1->getGlobalPosition() - a2->getGlobalPosition();
      params->setSeparation( (H3DFloat)v.magnitude() );
    } else {
      NxVec3 v = a1->getGlobalPosition() - NxVec3( 0, 0, 0 );
      params->setSeparation( (H3DFloat)v.magnitude() );
    }
  }
  if( params->haveSeparationRate() ) {
  }
}

// DoubleAxisHingeJoint
void PhysXCallbacks::setDoubleAxisHingeJointParameters( void *data ) {
  DoubleAxisHingeJointParameters *params = static_cast< DoubleAxisHingeJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxD6Joint * joint = static_cast< NxD6Joint *>( constraint_info->joint );

  if( params->haveAnchorPoint() ) {
    //const Vec3f &ap = params->getAnchorPoint();
  }
  if( params->haveAxis1() ) {
    //const Vec3f &axis1 = params->getAxis1();
  }
  if( params->haveAxis2() ) {
   // const Vec3f &axis2 = params->getAxis2();
  }
  if( params->haveDesiredAngularVelocity1() ) {
  }
  if( params->haveDesiredAngularVelocity2() ) {
  }    
  if( params->haveMinAngle1() ) {
  }
  if( params->haveMaxAngle1() ) {
  }
  if( params->haveMaxTorque1() ) {
    if( params->getMaxTorque1() < 0 ) { 
      // ODE specifies dParamFMax >= 0. X3D specifies value can be any real number
    }
  }
  if( params->haveMaxTorque2() ) {
    if( params->getMaxTorque2() < 0 ) { 
      // ODE specifies dParamFMax >= 0. X3D specifies value can be any real number
    }
  }
  if( params->haveStopBounce1() ) {
  }
  if( params->haveStopConstantForceMix1() ) {
  }
  if( params->haveStopErrorCorrection1() ) {
  }
  if( params->haveSuspensionErrorCorrection() ) {
  }
  if( params->haveSuspensionForce() ) {
  }
}

// DoubleAxisHingeJoint
void PhysXCallbacks::getDoubleAxisHingeJointParameters( void *data ) {
  DoubleAxisHingeJointParameters *params = static_cast< DoubleAxisHingeJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxD6Joint * joint = static_cast< NxD6Joint *>( constraint_info->joint );

  if( params->haveHinge1Angle() ) {
  }
  if( params->haveHinge2Angle() ) {
  }
  if( params->haveHinge1AngleRate() ) {
  }
  if( params->haveHinge2AngleRate() ) {
  }
  if( params->haveBody1AnchorPoint() ) {
  }
  if( params->haveBody2AnchorPoint() ) {
  }
  if( params->haveBody1Axis() ) {
  }
  if( params->haveBody2Axis() ) {
  }
}

// UniversalJoint
void PhysXCallbacks::setUniversalJointParameters( void *data ) {
  UniversalJointParameters *params = static_cast< UniversalJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxD6Joint * joint = static_cast< NxD6Joint *>( constraint_info->joint );

  if( params->haveAnchorPoint() ) {
    const Vec3f &v = params->getAnchorPoint();
    joint->setGlobalAnchor( NxVec3(v.x, v.y, v.z) );
  }
  if( params->haveAxis1() || params->haveAxis2() ) {
    const Vec3f &v1 = params->getAxis1();
    const Vec3f &v2 = params->getAxis2();
    joint->setGlobalAxis( NxVec3(v1.x, v1.y, v1.z) );
    
    NxActor * ac1;
    NxActor * ac2;
    NxMat34 trans1;
    NxMat34 trans2;
    joint->getActors( &ac1, &ac2 );
    if ( ac1 != NULL ) 
      trans1 = ac1->getGlobalPose();
    if ( ac2 != NULL )
      trans2 = ac2->getGlobalPose();
    trans1.getInverse( trans1 );
    trans2.getInverse( trans2 );

    NxVec3 global_normal( v2.x, v2.y, v2.z );
    NxVec3 local_normal[2];
    local_normal[0] = trans1 * global_normal;
    local_normal[1] = trans2 * global_normal;

    NxD6JointDesc joint_desc;
    joint->saveToDesc( joint_desc );
    joint_desc.localNormal[0] = local_normal[0];
    joint_desc.localNormal[1] = local_normal[1];
    joint->loadFromDesc( joint_desc );
  }
  if( params->haveStop1Bounce() ) {
  }
  if( params->haveStop2Bounce() ) {
  }
  if( params->haveStop1ErrorCorrection() ) {
  }
  if( params->haveStop2ErrorCorrection() ) {
  }
}


// UniversalJoint
void PhysXCallbacks::getUniversalJointParameters( void *data ) {
  UniversalJointParameters *params = static_cast< UniversalJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxD6Joint * joint = static_cast< NxD6Joint *>( constraint_info->joint );
  NxD6JointDesc joint_desc;
  joint->saveToDesc( joint_desc ); 

  if( params->haveBody1AnchorPoint() ) {
    NxVec3 &v = joint_desc.localAnchor[0];
    params->setBody1AnchorPoint( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
  if( params->haveBody2AnchorPoint() ) {
    NxVec3 &v = joint_desc.localAnchor[1];
    params->setBody2AnchorPoint( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
  if( params->haveBody1Axis() ) {
    NxVec3 &v = joint_desc.localAxis[0];
    params->setBody1Axis( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
  if( params->haveBody2Axis() ) {
    NxVec3 &v = joint_desc.localAxis[1];
    params->setBody2Axis( Vec3f( (H3DFloat)v.x, (H3DFloat)v.y, (H3DFloat)v.z ) );
  }
}


// MotorJoint
void PhysXCallbacks::setMotorJointParameters( void *data ) {
  MotorJointParameters *params = static_cast< MotorJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxD6Joint * joint = static_cast< NxD6Joint *>( constraint_info->joint );
}

// MotorJoint
void PhysXCallbacks::getMotorJointParameters( void *data ) {
  MotorJointParameters *params = static_cast< MotorJointParameters *>( data );
  ConstraintInfo * constraint_info = (constraintInfoMap.find(params->getConstraintId()))->second;
  NxD6Joint * joint = static_cast< NxD6Joint *>( constraint_info->joint );
}

PeriodicThread::CallbackCode PhysXCallbacks::getCurrentContacts( void *data ) {
  typedef pair< list< PhysicsEngineParameters::ContactParameters  >*,
                PhysicsEngineThread * > InputType;
  InputType *params = 
    static_cast< InputType *>( data );

  PhysXSpecificData *physX_data = 
    static_cast< PhysXSpecificData * >(params->second->getEngineSpecificData());

  physX_data->contactReport.getContacts ( *params->first );

  return PeriodicThread::CALLBACK_DONE;
}


PeriodicThread::CallbackCode PhysXCallbacks::addSpace( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysXCallbacks::removeSpace( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysXCallbacks::setSpaceParameters( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

PeriodicThread::CallbackCode PhysXCallbacks::getSpaceParameters( void *data ) {
  return PeriodicThread::CALLBACK_DONE;
}

// Attach joint functions
PhysXCallbacks::ConstraintInfo * PhysXCallbacks::attachBallJoint( ConstraintParameters *params ) {
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  ConstraintInfo * constraint_info = new ConstraintInfo;
  NxSphericalJointDesc desc;
  desc.actor[0] = (NxActor*) params->getBody1();
  desc.actor[1] = (NxActor*) static_cast< BallJointParameters * >(params)->getBody2();
  desc.flags = NX_JF_COLLISION_ENABLED;
  constraint_info->joint = physx_data->scene->createJoint( desc );
  constraint_info->id = ( H3DUInt64 )constraint_info->joint;
  return constraint_info;
}

PhysXCallbacks::ConstraintInfo * PhysXCallbacks::attachSingleAxisHingeJoint( ConstraintParameters *params ) {
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  ConstraintInfo * constraint_info = new ConstraintInfo;
  NxActor * actor1 = (NxActor*) params->getBody1();
  NxActor * actor2 = (NxActor*) static_cast< SingleAxisHingeJointParameters * >(params)->getBody2();
  NxRevoluteJointDesc joint_desc;
  joint_desc.actor[0] = actor1;
  joint_desc.actor[1] = actor2;
  joint_desc.flags = NX_JF_COLLISION_ENABLED | NX_RJF_LIMIT_ENABLED;
  constraint_info->joint = physx_data->scene->createJoint( joint_desc );
  constraint_info->id = ( H3DUInt64 )constraint_info->joint;
  return constraint_info;
}

PhysXCallbacks::ConstraintInfo * PhysXCallbacks::attachSliderJoint( ConstraintParameters *params ) {
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  ConstraintInfo * constraint_info = new ConstraintInfo;
  NxCylindricalJointDesc desc;
  desc.actor[0] = (NxActor*) params->getBody1();
  desc.actor[1] = (NxActor*) static_cast< SliderJointParameters * >(params)->getBody2();
  desc.jointFlags = NX_JF_COLLISION_ENABLED;
  constraint_info->joint = physx_data->scene->createJoint( desc );
  constraint_info->id = ( H3DUInt64 )constraint_info->joint;
  return constraint_info;
}

PhysXCallbacks::ConstraintInfo * PhysXCallbacks::attachDoubleAxisHingeJoint( ConstraintParameters *params ) {
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  /*ConstraintInfo * constraint_info = new ConstraintInfo;
  NxActor * actor1 = (NxActor*) params->getBody1();
  NxActor * actor2 = (NxActor*) static_cast< DoubleAxisHingeJointParameters * >(params)->getBody2();*/
  return NULL;
}

PhysXCallbacks::ConstraintInfo * PhysXCallbacks::attachUniversalJoint( ConstraintParameters *params ) {
  UniversalJointParameters * p = static_cast< UniversalJointParameters * >( params );
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  ConstraintInfo * constraint_info = new ConstraintInfo;
  NxActor * actor1 = (NxActor*) p->getBody1();
  NxActor * actor2 = (NxActor*) p->getBody2();
  Vec3f axis1 = p->getAxis1();
  axis1.normalizeSafe();
  Vec3f axis2 = p->getAxis2();
  axis2.normalizeSafe();
  Vec3f anchor = p->getAnchorPoint();

  NxD6JointDesc desc;
  desc.actor[0] = actor1;
  desc.actor[1] = actor2;
  desc.setGlobalAnchor( NxVec3(anchor.x, anchor.y, anchor.z) );
  desc.localAxis[0] = NxVec3( axis1.x, axis1.y, axis1.z );
  desc.localNormal[0] = NxVec3( axis2.x, axis2.y, axis2.z );
  desc.localAxis[1] = NxVec3( axis1.x, axis1.y, axis1.z );
  desc.localNormal[1] = NxVec3( axis2.x, axis2.y, axis2.z );

  desc.twistMotion = NX_D6JOINT_MOTION_FREE;
  desc.swing1Motion = NX_D6JOINT_MOTION_FREE;
  desc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

  desc.xMotion = NX_D6JOINT_MOTION_LOCKED;
  desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
  desc.zMotion = NX_D6JOINT_MOTION_LOCKED;
  
  constraint_info->joint = physx_data->scene->createJoint( desc );
  constraint_info->id = ( H3DUInt64 )constraint_info->joint;
  assert( constraint_info->joint != NULL );
  /*
    cerr << d6Desc.localAxis[0].x << " " << d6Desc.localAxis[0].y << " " << d6Desc.localAxis[0].z << endl;
    cerr << d6Desc.localNormal[0].x << " " << d6Desc.localNormal[0].y << " " << d6Desc.localNormal[0].z << endl;
    cerr << d6Desc.localAxis[1].x << " " << d6Desc.localAxis[1].y << " " << d6Desc.localAxis[1].z << endl;
    cerr << d6Desc.localNormal[1].x << " " << d6Desc.localNormal[1].y << " " << d6Desc.localNormal[1].z << endl;
  */
  return constraint_info;
}

PhysXCallbacks::ConstraintInfo * PhysXCallbacks::attachMotorJoint( ConstraintParameters *params ) {
  PhysXSpecificData *physx_data = 
    static_cast< PhysXSpecificData * >(params->getEngine()->getEngineSpecificData());
  ConstraintInfo * constraint_info = new ConstraintInfo;
  NxActor * actor1 = (NxActor*) params->getBody1();
  NxActor * actor2 = (NxActor*) static_cast< MotorJointParameters * >(params)->getBody2();

  return constraint_info;
}

// soft body callback functions
H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::addSoftBody ( void *data ) {
  PhysicsEngineParameters::H3DSoftBodyNodeParameters* params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );
  PhysXSpecificData* physXData= 
    static_cast<PhysXSpecificData*>( params->getEngine()->getEngineSpecificData() );

  // Create soft body
  PhysXH3DSoftBody* softBody= NULL;
  if ( SoftBodyParameters* bodyParams= dynamic_cast<SoftBodyParameters*>(params) ) {
    softBody= new PhysXSoftBody ( *bodyParams, *physXData );
  } else if ( ClothParameters* bodyParams= dynamic_cast<ClothParameters*>(params) ) {
    softBody= new PhysXCloth ( *bodyParams, *physXData );
  } else if( RopeParameters* bodyParams= dynamic_cast<RopeParameters*>(params) ) {
    Console ( 3 ) << "Warning: Rope softbody type is not supported by PhysX in H3DPhysics!" << endl;
  }
  params->setBodyId ( (H3DBodyId)softBody );

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::removeSoftBody ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );
  PhysXSpecificData* physXData= 
    static_cast<PhysXSpecificData*>( params->getEngine()->getEngineSpecificData() );

  PhysXH3DSoftBody* softBody= (PhysXH3DSoftBody*)params->getBodyId();

  // Delete the soft body implementation
  delete softBody;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::setSoftBodyParameters ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );

  // Set soft body parameters
  ((PhysXH3DSoftBody*)params->getBodyId())->setParameters ( *params );

  delete params;

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::getSoftBodyParameters ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );

  // Get soft body parameters
  ((PhysXH3DSoftBody*)params->getBodyId())->getParameters ( *params );

  return PeriodicThread::CALLBACK_DONE;
}

H3DUtil::PeriodicThread::CallbackCode PhysXCallbacks::applyExternalForces ( void *data ) {
  H3DSoftBodyNodeParameters *params = 
    static_cast< H3DSoftBodyNodeParameters *>( data );

  // Apply external forces
  ((PhysXH3DSoftBody*)params->getBodyId())->applyExternalForces ( *params );

  return PeriodicThread::CALLBACK_DONE;
}

#endif // HAVE_PHYSX


