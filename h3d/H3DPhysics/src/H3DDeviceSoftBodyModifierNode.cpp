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
/// \file H3DDeviceSoftBodyModifierNode.cpp
/// \brief Source file for H3DDeviceSoftBodyModifierNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DDeviceSoftBodyModifierNode.h>

using namespace H3D;  

H3DNodeDatabase H3DDeviceSoftBodyModifierNode::database( "H3DDeviceSoftBodyModifierNode", 
                                                        NULL, 
                                                        typeid( H3DDeviceSoftBodyModifierNode ),
                                                        &H3DBodyModifierNode::database);

namespace H3DDeviceSoftBodyModifierNodeInternals {
  FIELDDB_ELEMENT( H3DDeviceSoftBodyModifierNode, deviceIndex, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DDeviceSoftBodyModifierNode, hapticGeometry, INPUT_OUTPUT )
}

H3DDeviceSoftBodyModifierNode::H3DDeviceSoftBodyModifierNode (
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DBodyNode > _body1,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFGeometryNode > _hapticGeometry,      
  Inst< SFInt32 > _deviceIndex ) :
H3DBodyModifierNode( _metadata, _valueUpdater, _body1, _engineOptions), 
deviceIndex ( _deviceIndex ),
hapticGeometry( _hapticGeometry ),
devIndex ( -1 )
{
  // init fields
  type_name = "H3DDeviceSoftBodyModifierNode";
  database.initFields( this );

  deviceIndex->setValue ( -1 );
  deviceIndex->route( valueUpdater );

}

void H3DDeviceSoftBodyModifierNode::traverseSG ( TraverseInfo& ti ) {
  H3DBodyModifierNode::traverseSG ( ti );

  traverseLock.lock();
  accumulatedInverse= ti.getAccInverseMatrix();
  devIndex= deviceIndex->getValue();
  traverseLock.unlock();
}
ModifierParameters* H3DDeviceSoftBodyModifierNode::getModifierParameters( bool all_params ) {

  ModifierParameters* params= H3DBodyModifierNode::getModifierParameters( all_params );
  if ( params && ( all_params || valueUpdater->hasCausedEvent ( deviceIndex ) ) ) {
    params->setDeviceIndex( deviceIndex->getValue() ); 
  }
  return params;
}

