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
/// \file VertexBodyForce.cpp
/// \brief Source file for VertexBodyForce, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/VertexBodyForce.h>

using namespace H3D;  
using namespace PhysicsEngineParameters;

H3DNodeDatabase VertexBodyForce::database( "VertexBodyForce", 
                                          &(newInstance<VertexBodyForce>), 
                                          typeid( VertexBodyForce ),
                                          &H3DVertexBodyModifierNode::database);

namespace VertexBodyForceInternals {
  FIELDDB_ELEMENT( VertexBodyForce, forces, INPUT_OUTPUT )
}

VertexBodyForce::VertexBodyForce (
                                  Inst< SFNode > _metadata,
                                  Inst< ValueUpdater > _valueUpdater,
                                  Inst< SFH3DBodyNode > _body1,
                                  Inst< MFEngineOptions > _engineOptions,
                                  Inst< MFInt32 > _index,
                                  Inst< MFVec3f > _forces):
H3DVertexBodyModifierNode( _metadata, _valueUpdater, _body1, _engineOptions, _index  ),
forces ( _forces ),
forcesChanged ( new Field ),
localUpdateForces ( false )
{
  // init fields
  type_name = "VertexBodyForce";
  database.initFields( this );

  forcesChanged->setOwner ( this );
  forcesChanged->setName ( "forcesChanged" );

  index->route ( forcesChanged );
  body1->route ( forcesChanged );
  forces->route ( forcesChanged );
}

void VertexBodyForce::traverseSG ( TraverseInfo& ti ) {
  H3DBodyModifierNode::traverseSG ( ti );

  traverseLock.lock();
  localIndex= index->getValue();
  localForces= forces->getValue();
  if ( !localUpdateForces && !forcesChanged->isUpToDate() ) {
    localUpdateForces= true;
  }
  traverseLock.unlock();
}

void VertexBodyForce::addVertexForces ( H3DSoftBodyNodeParameters& params ) {
  for ( size_t i= 0; i < localIndex.size() && localForces.size(); ++i ) {
    params.addVertexForce ( 
      H3DSoftBodyNodeParameters::VertexForce ( localIndex[i], localForces[i], getModifierId() ) );
  }  
}

void VertexBodyForce::calculateForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams,
                                        HAPI::HAPIHapticsDevice& hd ) {
  traverseLock.lock();
  if ( localUpdateForces ) {
    softBodyParams.clearVertexForces( getModifierId() );
    addVertexForces ( softBodyParams );
    localUpdateForces= false;
  }
  traverseLock.unlock();

}
