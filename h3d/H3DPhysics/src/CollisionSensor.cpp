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
/// \file CollisionSensor.cpp
/// \brief Source file for CollisionSensor, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/CollisionSensor.h>
#include <H3D/H3DPhysics/RigidBody.h>

using namespace H3D;

H3DNodeDatabase CollisionSensor::database( "CollisionSensor", 
                                          &newInstance< CollisionSensor >, 
                                          typeid( CollisionSensor ),
                                          &X3DSensorNode::database);

namespace CollisionSensorInternals {
  FIELDDB_ELEMENT( CollisionSensor, collider, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionSensor, intersections, OUTPUT_ONLY )
  FIELDDB_ELEMENT( CollisionSensor, contacts, OUTPUT_ONLY )
}

CollisionSensor::CollisionSensor( Inst< SFBool > _enabled,
                                 Inst< SFNode > _metadata,
                                 Inst< SFBool > _isActive,
                                 Inst< SFCollisionCollection > _collider, 
                                 Inst< MFCollidableNode > _intersections,
                                 Inst< MFContact > _contacts):
X3DSensorNode( _enabled, _metadata, _isActive ),
collider( _collider ),
intersections( _intersections ),
contacts( _contacts ) {

  type_name = "CollisionSensor";
  database.initFields( this );
}

void CollisionSensor::traverseSG( TraverseInfo &ti ) {
  X3DSensorNode::traverseSG( ti );

  bool have_contacts = false;

  if( enabled->getValue() ) {
    CollisionCollection *cc = collider->getValue();
    if( cc ) {
      H3D::PhysicsEngineThread *pt = cc->getLastLoopPhysicsEngine();
      if( pt ) {

        PROFILE_BEGIN ( getContacts );
        // get all contacts
        list< PhysicsEngineParameters::ContactParameters > current_contacts;
        pt->getCurrentContacts( current_contacts );
        size_t nr_contacts = current_contacts.size();
        PROFILE_END ();

        PROFILE_BEGIN ( resizeContacts );
        // make sure contacts has the size of nr_contacts
        if( nr_contacts < contacts->size() ) {
          contacts->resize( nr_contacts, NULL, id );
        } else {
          for( unsigned int i = contacts->size(); i < nr_contacts; ++i ) {
            contacts->push_back( new Contact, id );
          }
        }
        PROFILE_END ();

        PROFILE_BEGIN ( setContacts );
        // update the contacts field to reflect the contacts collected.
        have_contacts = nr_contacts > 0;
        list< PhysicsEngineParameters::ContactParameters >::iterator ci = 
          current_contacts.begin();
        MFContact::const_iterator oc = contacts->begin();
        for( ; ci != current_contacts.end(); ++ci, ++oc ) {
          Contact *contact_node = static_cast< Contact * >( *oc );
          contact_node->bounce->setValue( (*ci).bounce );
          contact_node->minBounceSpeed->setValue( (*ci).min_bounce_speed );
          contact_node->softnessErrorCorrection->setValue( (*ci).softness_error_correction );
          contact_node->softnessConstantForceMix->setValue( (*ci).softness_constant_force_mix );
          contact_node->frictionCoefficients->setValue( (*ci).friction_coefficients );
          contact_node->slipCoefficients->setValue( (*ci).slip_coefficients );
          contact_node->surfaceSpeed->setValue( (*ci).surface_speed );
          contact_node->frictionDirection->setValue( (*ci).friction_direction );
          contact_node->appliedParameters->setValue( (*ci).applied_parameters );
          contact_node->geometry1->setValue( CollidableShape::getCollidableFromId( (*ci).geom1_id ) );
          contact_node->geometry2->setValue( CollidableShape::getCollidableFromId( (*ci).geom2_id ) );

          contact_node->body1->setValue( H3DBodyNode::getBodyFromId( (*ci).body1_id ) );
          contact_node->body2->setValue( H3DBodyNode::getBodyFromId( (*ci).body2_id ) );

          contact_node->contactNormal->setValue((*ci).contact_normal  );
          contact_node->position->setValue((*ci).position  );
          contact_node->depth->setValue((*ci).depth  );

        }
        PROFILE_END ();

        PROFILE_BEGIN ( touchContacts );
        contacts->touch();
        PROFILE_END ();
      }
    }
  }

  // The CollisionSensor is active if we have contacts.
  if( isActive->getValue() != have_contacts ) {
    isActive->setValue( have_contacts, id );
  }

}
