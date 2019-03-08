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
/// \file PhysX3CollidableOptions.cpp
/// \brief Source file for PhysX3CollidableOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/PhysX3CollidableOptions.h>
#include <H3D/ResourceResolver.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PhysX3CollidableOptions::database( "PhysX3CollidableOptions", 
                                                 &newInstance<PhysX3CollidableOptions>, 
                                                 typeid( PhysX3CollidableOptions ),
                                                 &H3DEngineOptions::database);

namespace PhysX3SoftBodyOptionsInternals {
  FIELDDB_ELEMENT( PhysX3CollidableOptions, convex, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, convexDecomposition, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, compacityWeight, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, volumeWeight, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, scaleFactor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, nrClusters, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, nrVerticesPerCH, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, concavity, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, addExtraDistPoints, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, nrTargetTrianglesDecimatedMesh, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, addFacesPoints, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, connectDist, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, smallClusterThreshold, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, cookedFilename, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, saveConvexDecomposition, INPUT_OUTPUT )

  FIELDDB_ELEMENT( PhysX3CollidableOptions, restOffset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, contactOffset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, setFlagsForAll, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, suppressDisabledContacts, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3CollidableOptions, contactShaderPairFlags, INPUT_OUTPUT )

}

PhysX3CollidableOptions::PhysX3CollidableOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst < SFBool  > _convex,
  Inst < SFBool  > _convexDecomposition,
  Inst < SFFloat > _compacityWeight,
  Inst < SFFloat > _volumeWeight,
  Inst < SFFloat > _scaleFactor,
  Inst < SFInt32 > _nrClusters,
  Inst < SFInt32 > _nrVerticesPerCH,
  Inst < SFFloat > _concavity,
  Inst < SFBool  > _addExtraDistPoints,
  Inst < SFInt32 > _nrTargetTrianglesDecimatedMesh,
  Inst < SFBool  > _addFacesPoints,
  Inst < SFFloat > _connectDist,
  Inst < SFFloat > _smallClusterThreshold,
  Inst < SFString > _cookedFilename,
  Inst < SFString > _saveConvexDecomposition,
  Inst < SFDouble > _restOffset,
  Inst < SFDouble > _contactOffset,
  Inst < SFBool  > _setFlagsForAll,
  Inst < SFBool  > _suppressDisabledContacts,
  Inst < MFString > _contactShaderPairFlags ):
  H3DEngineOptions ( _metadata, _valueUpdater ),
  convex ( _convex ),
  convexDecomposition ( _convexDecomposition ),
  compacityWeight ( _compacityWeight ),
  volumeWeight ( _volumeWeight ),
  scaleFactor ( _scaleFactor ),
  nrClusters ( _nrClusters ),
  nrVerticesPerCH ( _nrVerticesPerCH ),
  concavity ( _concavity ),
  addExtraDistPoints ( _addExtraDistPoints ),
  nrTargetTrianglesDecimatedMesh ( _nrTargetTrianglesDecimatedMesh ),
  addFacesPoints ( _addFacesPoints ),
  connectDist ( _connectDist ),
  smallClusterThreshold ( _smallClusterThreshold ),
  cookedFilename ( _cookedFilename ),
  saveConvexDecomposition ( _saveConvexDecomposition ),
  restOffset( _restOffset ),
  contactOffset( _contactOffset ),
  setFlagsForAll( _setFlagsForAll ),
  suppressDisabledContacts( _suppressDisabledContacts ),
  contactShaderPairFlags( _contactShaderPairFlags ),
  base_url( "" ) {
  type_name = "PhysX3CollidableOptions";
  database.initFields( this );

  convex->setValue ( true );
  convexDecomposition->setValue ( false );
  compacityWeight->setValue ( 0.0001f );
  volumeWeight->setValue ( 0.0f );
  scaleFactor->setValue ( 1000.0f );
  nrClusters->setValue ( 2 );
  nrVerticesPerCH->setValue ( 100 );
  concavity->setValue ( 100.0f );
  addExtraDistPoints->setValue ( true );
  nrTargetTrianglesDecimatedMesh->setValue ( 2000 );
  addFacesPoints->setValue ( true );
  connectDist->setValue ( 30.0f );
  smallClusterThreshold->setValue ( 0.25f );
  cookedFilename->setValue ( "" );
  saveConvexDecomposition->setValue ( "" );
  restOffset->setValue( 0.0 );
  contactOffset->setValue( 0.02 );

  setFlagsForAll->setValue ( false );
  suppressDisabledContacts->setValue ( true );
  contactShaderPairFlags->push_back( "eCONTACT_DEFAULT" );
  contactShaderPairFlags->push_back( "eNOTIFY_CONTACT_POINTS" );
  contactShaderPairFlags->push_back( "eNOTIFY_TOUCH_PERSISTS" );


  convex->route ( valueUpdater );
  convexDecomposition->route ( valueUpdater );
  compacityWeight->route ( valueUpdater );
  volumeWeight->route ( valueUpdater );
  scaleFactor->route ( valueUpdater );
  nrClusters->route ( valueUpdater );
  nrVerticesPerCH->route ( valueUpdater );
  concavity->route ( valueUpdater );
  addExtraDistPoints->route ( valueUpdater );
  nrTargetTrianglesDecimatedMesh->route ( valueUpdater );
  addFacesPoints->route ( valueUpdater );
  connectDist->route ( valueUpdater );
  smallClusterThreshold->route ( valueUpdater );
  cookedFilename->route ( valueUpdater );
  saveConvexDecomposition->route ( valueUpdater );
  restOffset->route ( valueUpdater );
  contactOffset->route ( valueUpdater );
  setFlagsForAll->route ( valueUpdater );
  suppressDisabledContacts->route ( valueUpdater );
  contactShaderPairFlags->route ( valueUpdater );

}

PhysicsEngineParameters::EngineOptionParameters* PhysX3CollidableOptions::getParameters( bool all_params ) {
  PhysX3CollidableParameters* params= new PhysX3CollidableParameters( base_url );

  if ( all_params || valueUpdater->hasCausedEvent ( convex ) ) {
    params->setConvex ( convex->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( convexDecomposition ) ) {
    params->setConvexDecomposition ( convexDecomposition->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( compacityWeight ) ) {
    params->setCompacityWeight ( compacityWeight->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( volumeWeight ) ) {
    params->setVolumeWeight ( volumeWeight->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( scaleFactor ) ) {
    params->setScaleFactor ( scaleFactor->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( nrClusters ) ) {
    params->setNrClusters ( nrClusters->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( nrVerticesPerCH ) ) {
    params->setNrVerticesPerCH ( nrVerticesPerCH->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( concavity ) ) {
    params->setConcavity ( concavity->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( addExtraDistPoints ) ) {
    params->setAddExtraDistPoints ( addExtraDistPoints->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( nrTargetTrianglesDecimatedMesh ) ) {
    params->setNrTargetTrianglesDecimatedMesh ( nrTargetTrianglesDecimatedMesh->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( addFacesPoints ) ) {
    params->setAddFacesPoints ( addFacesPoints->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( connectDist ) ) {
    params->setConnectDist ( connectDist->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( smallClusterThreshold ) ) {
    params->setSmallClusterThreshold ( smallClusterThreshold->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( cookedFilename ) ) {
    params->setCookedFilename ( cookedFilename->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( saveConvexDecomposition ) ) {
    params->setSaveConvexDecomposition ( saveConvexDecomposition->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( restOffset ) ) {
    params->setRestOffset ( restOffset->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( contactOffset ) ) {
    params->setContactOffset ( contactOffset->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( setFlagsForAll ) ) {
    params->setSetFlagsForAll( setFlagsForAll->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( suppressDisabledContacts ) ) {
    params->setSuppressDisabledContacts( suppressDisabledContacts->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( contactShaderPairFlags ) ) {
    params->setContactPairFlags( contactShaderPairFlags->getValue() );
  }

  return params;
}

void PhysX3CollidableOptions::initialize() {
  base_url = ResourceResolver::getBaseURL();
  H3DEngineOptions::initialize();
}
