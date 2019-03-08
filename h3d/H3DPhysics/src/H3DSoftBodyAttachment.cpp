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
/// \file H3DSoftBodyAttachment.cpp
/// \brief Source file for H3DSoftBodyAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DSoftBodyAttachment.h>

using namespace H3D;  

H3DNodeDatabase H3DSoftBodyAttachment::database( "H3DSoftBodyAttachment", 
                                                 NULL, 
                                                 typeid( H3DSoftBodyAttachment ),
                                                 &H3DAttachmentNode::database);

namespace H3DSoftBodyAttachmentInternals {
  FIELDDB_ELEMENT( H3DSoftBodyAttachment, index2, INPUT_OUTPUT )
}

H3DSoftBodyAttachment::H3DSoftBodyAttachment (
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DSoftBodyNode > _body1,
  Inst< MFString     > _forceOutput,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFH3DSoftBodyNode > _body2,
  Inst< TrackedMFInt32 > _index,
  Inst< TrackedMFInt32 > _index2 ) :
H3DAttachmentNode ( _metadata, _valueUpdater, _body1, _forceOutput,
                   _engineOptions, _body2, _index ),
                   index2( _index2 )
{
  // init fields
  type_name = "H3DSoftBodyAttachment";
  database.initFields( this );

  index2->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* H3DSoftBodyAttachment::createConstraintParameters () {
  return new H3DSoftBodyAttachmentParameters();
}

PhysicsEngineParameters::ConstraintParameters* H3DSoftBodyAttachment::getConstraintParameters( bool all_params ) {

  PhysicsEngineParameters::H3DSoftBodyAttachmentParameters* params= 
    dynamic_cast<PhysicsEngineParameters::H3DSoftBodyAttachmentParameters*>(H3DAttachmentNode::getConstraintParameters(all_params));

  if( !params )
    return NULL;

  if ( all_params || valueUpdater->hasCausedEvent ( index2 ) ) {
    // Update all values
    params->setIndex2( index2->getValue() );
    if ( index2->haveTrackedChanges() ) {
      // Indicate which values have changed
      params->setIndex2Changes ( index2->getEdits () );
      index2->resetTracking();
    }
  }

  return params;
}
