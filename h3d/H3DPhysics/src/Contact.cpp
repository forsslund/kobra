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
/// \file Contact.cpp
/// \brief Source file for Contact, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/Contact.h>

using namespace H3D;

H3DNodeDatabase Contact::database( "Contact", 
                                  &newInstance< Contact >, 
                                  typeid( Contact ),
                                  &X3DNode::database);

namespace ContactInternals {
  FIELDDB_ELEMENT( Contact, appliedParameters, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, body1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, body2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, bounce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, contactNormal, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, depth, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, frictionCoefficients, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, frictionDirection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, geometry1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, geometry2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, minBounceSpeed, INPUT_OUTPUT )
  FieldDBInsert string( INPUT_OUTPUT( &Contact::database, "minbounceSpeed", &Contact::minBounceSpeed ) );
  FIELDDB_ELEMENT( Contact, position, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, slipCoefficients, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, softnessConstantForceMix, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, softnessErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Contact, surfaceSpeed, INPUT_OUTPUT )
}

Contact::Contact(Inst< SFNode >  _metadata,
                    Inst< MFString >  _appliedParameters,
                    Inst< SFH3DBodyNode >  _body1,
                    Inst< SFH3DBodyNode >  _body2,
                    Inst< SFFloat >  _bounce,
                    Inst< SFVec3f >  _contactNormal,
                    Inst< SFFloat >  _depth,
                    Inst< SFVec2f >  _frictionCoefficients,
                    Inst< SFVec3f >  _frictionDirection,
                    Inst< SFCollidableNode >  _geometry1,
                    Inst< SFCollidableNode >  _geometry2,
                    Inst< SFFloat >  _minBounceSpeed,
                    Inst< SFVec3f >  _position,
                    Inst< SFVec2f >  _slipCoefficients,
                    Inst< SFFloat >  _softnessConstantForceMix,
                    Inst< SFFloat >  _softnessErrorCorrection,
                    Inst< SFVec2f >  _surfaceSpeed):
X3DNode( _metadata ),
appliedParameters( _appliedParameters ),
body1( _body1 ),
body2( _body2 ),
bounce( _bounce ),
contactNormal( _contactNormal ),
depth( _depth ),
frictionCoefficients( _frictionCoefficients ),
frictionDirection( _frictionDirection ),
geometry1( _geometry1 ),
geometry2( _geometry2 ),
minBounceSpeed( _minBounceSpeed ),
position( _position ),
slipCoefficients( _slipCoefficients ),
softnessConstantForceMix( _softnessConstantForceMix ),
softnessErrorCorrection( _softnessErrorCorrection ),
surfaceSpeed( _surfaceSpeed ) {

  // init fields
  type_name = "Contact";
  database.initFields( this );

  // set default values
  appliedParameters->push_back( "BOUNCE" );
  bounce->setValue( 0 );
  contactNormal->setValue( Vec3f( 0, 1, 0 ) );
  depth->setValue( 0 );
  frictionCoefficients->setValue( Vec2f( 0, 0 ) );
  frictionDirection->setValue( Vec3f( 0, 1, 0 ) );
  minBounceSpeed->setValue( 0 );
  position->setValue( Vec3f( 0, 0 ,0 ) );
  slipCoefficients->setValue( Vec2f( 0, 0 ) );
  softnessConstantForceMix->setValue( (H3DFloat) 0.0001 );
  softnessErrorCorrection->setValue( (H3DFloat) 0.8 );
  surfaceSpeed->setValue( Vec2f( 0, 0 ) );
}

