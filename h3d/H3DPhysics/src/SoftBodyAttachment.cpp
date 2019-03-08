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
/// \file SoftBodyAttachment.cpp
/// \brief Source file for SoftBodyAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/SoftBodyAttachment.h>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

using namespace H3D;  

H3DNodeDatabase SoftBodyAttachment::database( "SoftBodyAttachment", 
                                              &(newInstance< SoftBodyAttachment >), 
                                              typeid( SoftBodyAttachment ),
                                              &H3DSoftBodyAttachment::database);

namespace SoftBodyAttachmentInternals {
  FIELDDB_ELEMENT( SoftBodyAttachment, physicsMaterial, INPUT_OUTPUT )
}

SoftBodyAttachment::SoftBodyAttachment (
                    Inst< SFNode > _metadata,
                    Inst< ValueUpdater > _valueUpdater,
                    Inst< SFH3DSoftBodyNode > _body1,
                    Inst< MFString     > _forceOutput,
                    Inst< MFEngineOptions > _engineOptions,
                    Inst< SFH3DSoftBodyNode > _body2,
                    Inst< TrackedMFInt32 > _index,
                    Inst< TrackedMFInt32 > _index2,
                    Inst< SFH3DPhysicsMaterialNode > _physicsMaterial )
  : H3DSoftBodyAttachment ( _metadata, _valueUpdater, _body1, _forceOutput, _engineOptions, _body2,
      _index, _index2 ),
    physicsMaterial ( _physicsMaterial ),
    previousPhysicsMaterial ( NULL )
{
  // init fields
  type_name = "SoftBodyAttachment";
  database.initFields( this );

  physicsMaterial->route ( valueUpdater );
}

SoftBodyAttachmentParameters* SoftBodyAttachment::createConstraintParameters () {
  return new SoftBodyAttachmentParameters();
}

SoftBodyAttachmentParameters* SoftBodyAttachment::getConstraintParameters( bool all_params ) {
  SoftBodyAttachmentParameters* params= 
    static_cast<SoftBodyAttachmentParameters*>(H3DSoftBodyAttachment::getConstraintParameters (all_params));

  if( !params )
    return NULL;

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

  return params;
}
