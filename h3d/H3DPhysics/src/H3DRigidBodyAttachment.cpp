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
/// \file H3DRigidBodyAttachment.cpp
/// \brief Source file for H3DRigidBodyAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DRigidBodyAttachment.h>

using namespace H3D;  

H3DNodeDatabase H3DRigidBodyAttachment::database( "RigidBodyAttachment", "H3DRigidBodyAttachment",
                                                 &(newInstance< H3DRigidBodyAttachment >), 
                                                 typeid( H3DRigidBodyAttachment ),
                                                 &H3DAttachmentNode::database);

H3DRigidBodyAttachment::H3DRigidBodyAttachment (
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DSoftBodyNode > _body1,
  Inst< MFString     > _forceOutput,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFRigidBody > _body2,
  Inst< TrackedMFInt32 > _index ) :
H3DAttachmentNode ( _metadata, _valueUpdater, _body1, _forceOutput,
                   _engineOptions, _body2, _index )
{
  // init fields
  type_name = "H3DRigidBodyAttachment";
  database.initFields( this );
}

PhysicsEngineParameters::ConstraintParameters* H3DRigidBodyAttachment::createConstraintParameters () {
  return new H3DRigidBodyAttachmentParameters();
}

PhysicsEngineParameters::ConstraintParameters* H3DRigidBodyAttachment::getConstraintParameters( bool all_params) {

  PhysicsEngineParameters::H3DRigidBodyAttachmentParameters* params= 
    static_cast<PhysicsEngineParameters::H3DRigidBodyAttachmentParameters*>(H3DAttachmentNode::getConstraintParameters(all_params));

  return params;
}
