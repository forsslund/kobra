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
/// \file KinematicSoftBodyModifier.cpp
/// \brief Source file for KinematicSoftBodyModifier, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/KinematicSoftBodyModifier.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>
#include <HAPI/HAPIHapticsDevice.h>

using namespace H3D;

H3DNodeDatabase KinematicSoftBodyModifier::database( "KinematicSoftBodyModifier", 
                                                    &(newInstance<KinematicSoftBodyModifier>), 
                                                    typeid( KinematicSoftBodyModifier ),
                                                    &H3DDeviceSoftBodyModifierNode::database);

namespace KinematicSoftBodyModifierInternals {
  FIELDDB_ELEMENT( KinematicSoftBodyModifier, rigidBody, INPUT_OUTPUT )
  FIELDDB_ELEMENT( KinematicSoftBodyModifier, offset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( KinematicSoftBodyModifier, useProxyPosition, INPUT_OUTPUT )
  FIELDDB_ELEMENT( KinematicSoftBodyModifier, forceFeedbackType, INPUT_OUTPUT )
}

namespace {
  const H3DFloat hapticStiffness= 70.0f;

  const H3DFloat blend= 0.01f;
  const H3DFloat controlStiffness= 150.0f;
  const H3DFloat controlTorqueStiffness= 1.0f;
  const H3DFloat hapticStiffnessSpringDamper= 150.0f;
  const H3DFloat hapticTorqueStiffnessSpringDamper= 1.0f;
}

KinematicSoftBodyModifier::KinematicSoftBodyModifier (
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DBodyNode > _body1,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFGeometryNode > _hapticGeometry,      
  Inst< SFInt32 > _deviceIndex,
  Inst< SFRigidBody > _rigidBody,
  Inst< SFMatrix4f > _offset,
  Inst< SFBool > _useProxyPosition,
  Inst< SFString > _forceFeedbackType,
  Inst< UpdateForceFeedbackType > _updateFeedbackType ) :
H3DDeviceSoftBodyModifierNode ( _metadata, _valueUpdater, _body1,
                               _engineOptions, _hapticGeometry, _deviceIndex ),
                               rigidBody ( _rigidBody ),
                               offset ( _offset ),
                               useProxyPosition ( _useProxyPosition ),
                               forceFeedbackType ( _forceFeedbackType ),
                               updateFeedbackType ( _updateFeedbackType ),
                               bodyId ( -1 ),
                               kinematicControl ( true ),
                               useProxyPos ( true )
{
  // init fields
  type_name = "KinematicSoftBodyModifier";
  database.initFields( this );

  updateFeedbackType->setOwner ( this );
  updateFeedbackType->setName ( "updateFeedbackType" );

  forceFeedbackType->setValue ( "NONE" );
  forceFeedbackType->route ( updateFeedbackType );

  useProxyPosition->setValue ( true );

}

// Initialize the node
void KinematicSoftBodyModifier::initialize () {
  H3DDeviceSoftBodyModifierNode::initialize ();

  reactionForce.reset ( new ReactionForce );
}

void KinematicSoftBodyModifier::traverseSG ( TraverseInfo& ti ) {
  H3DDeviceSoftBodyModifierNode::traverseSG ( ti );

  if ( forceFeedbackType->getValue() != "NONE" ) {
    ti.addForceEffectToAll ( reactionForce.get() );
  }

  if ( bodyId == -1 ) {
    RigidBody* rb= dynamic_cast<RigidBody*>(rigidBody->getValue());
    if ( rb && rb->isInitialized() ) {
      bodyId= rb->getBodyId();
    }
  }

  traverseLock.lock();
  bodyOffset= offset->getValue();
  useProxyPos= useProxyPosition->getValue();
  traverseLock.unlock();
}
// Calculate forces for deformation
void KinematicSoftBodyModifier::calculateForces ( H3DSoftBodyNodeParameters& softBodyParams,
                                                  HAPI::HAPIHapticsDevice& hd )
{
  if ( bodyId != -1 ) {

    traverseLock.lock();
    Matrix4f accInv= accumulatedInverse;
    Matrix4f rbOffset= bodyOffset;
    bool usePP= useProxyPos;
    traverseLock.unlock();
    UpdateForceFeedbackType::FeedbackType feedback= updateFeedbackType->getFeedbackType();

    Matrix4f trans= accInv*rbOffset;

    Matrix4f devTrans ( H3DUtil::Rotation(hd.getOrientation()) );
    devTrans[0][3]= (H3DFloat)hd.getPosition().x;
    devTrans[1][3]= (H3DFloat)hd.getPosition().y;
    devTrans[2][3]= (H3DFloat)hd.getPosition().z;

    Matrix4f total= devTrans*rbOffset;

    // Get device position and orientation
    Vec3f position ( total[0][3], total[1][3], total[2][3] );
    H3DUtil::Rotation orientation ( total.getRotationPart() );

    //cout << "pos: " << position << endl;

    // Only need proxy position if not providing force feedback
    if ( usePP && feedback==UpdateForceFeedbackType::Feedback_None ) {
      bool foundContact= false;
      // For the renderer on each layer
      for( unsigned int layer = 0; layer < hd.nrLayers() && !foundContact; ++layer ) {
        HAPI::HAPIHapticsRenderer* renderer = 
          hd.getHapticsRenderer( layer );
        HAPI::HAPIHapticsRenderer::Contacts contacts;
        if( renderer ) {
          contacts = renderer->getContacts();
        }
        for( HAPI::HAPIHapticsRenderer::Contacts::iterator j = contacts.begin();
          j != contacts.end() && !foundContact; ++j ) {
            X3DGeometryNode *geom =
              static_cast< X3DGeometryNode * >( (*j).first->getUserData() );
          
          // If device in contact with the output geometry
          if( softBodyParams.getSurfaceGeometry().size() > 0 ) {
            const H3DSoftBodyNodeParameters::X3DGeometryNodeList& surfaceGeom= softBodyParams.getSurfaceGeometry();
            if ( find ( surfaceGeom.begin(), surfaceGeom.end(), geom ) != surfaceGeom.end() ) {
              position= accInv*Vec3f((*j).second.globalContactPoint());
              foundContact= true;
            }
          } else {
            if ( geom == softBodyParams.getGeometry() ) {
              position= accInv*Vec3f((*j).second.globalContactPoint());
              foundContact= true;
            }
          }
        }
      }
    }

    // Get rigid body parameters
    RigidBodyParameters* p= new RigidBodyParameters;
    softBodyParams.getEngine()->getRigidBodyParameters( bodyId, *p );

    if ( feedback==UpdateForceFeedbackType::Feedback_ContactDepth ) {

      // Get current contacts from physics engine
      list<ContactParameters> contacts;
      pair< list< ContactParameters > *,
        PhysicsEngineThread * > data = make_pair( &contacts, softBodyParams.getEngine() );
      softBodyParams.getEngine()->callbacks()->getCurrentContacts ( &data );

      // For each contact
      for ( list<ContactParameters>::iterator i= contacts.begin(); i != contacts.end(); ++i ) {
        ContactParameters contact= *i;
        if ( contact.body1_id == bodyId || contact.body2_id == bodyId )  {

          // Defensive programming: Penetration depth should always be positive
          float depth= contact.depth;
          if ( depth < 0 ) depth= -depth;

          // Flip normal if bodies are reversed
          Vec3f contactNormal= contact.contact_normal;
          if ( contact.body2_id == bodyId ) {
            contactNormal= -contactNormal;
          }

          // Calculate penetration point and add a spring for rendering forces
          Vec3f penP= contact.position+depth*contactNormal;
          reactionForce->addSpring( penP, contact.position, hapticStiffness );
        }
      }

      // Set the center of mass of the proxy for torque calculations
      reactionForce->setCenterOfMass ( position );

      // Transfer current springs for rendering
      if ( hd.getThread() ) {
        reactionForce->transferSprings( *hd.getThread() );
      }

      // Set the position and orientation of the kinematic deformer
      p->setKinematicControl ( true );
      p->setPosition ( position );
      p->setOrientation ( orientation );

    } else 
      if ( feedback==UpdateForceFeedbackType::Feedback_SpringDamper ||
        feedback==UpdateForceFeedbackType::Feedback_SpringDamperProjected ) {

          // Get current contacts from physics engine
          list<ContactParameters> contacts;
          pair< list< ContactParameters > *,
            PhysicsEngineThread * > data = make_pair( &contacts, softBodyParams.getEngine() );
          softBodyParams.getEngine()->callbacks()->getCurrentContacts ( &data );

          // Find out if the deformer is in contact 
          int inContact= 0;
          Vec3f normal;
          for ( list<ContactParameters>::iterator i= contacts.begin(); i != contacts.end(); ++i ) {
            ContactParameters contact= *i;
            if ( contact.body1_id == bodyId || contact.body2_id == bodyId )  {
              ++inContact;

              if ( contact.body1_id == bodyId ) {
                normal+= contact.contact_normal;
              } else {
                normal-= contact.contact_normal;
              }
            }
          }

          if ( inContact ) {
            normal.normalizeSafe();

            if ( !kinematicControl ) {
              lastNormal= (1-blend)*lastNormal + blend*normal;
              lastNormal.normalizeSafe();
            } else {
              lastNormal= normal;
            }
          }

          if ( kinematicControl && inContact ) {
            kinematicControl= false;
          }
          if ( !kinematicControl && !inContact ) {
            Vec3f sep= position-p->getPosition();

            if ( sep.dotProduct ( lastNormal ) > 0.001 ) {
              kinematicControl= true;
            }
          }

          if ( !kinematicControl ) {

            // Control the object using a spring
            p->setKinematicControl ( false );
            p->setUseGlobalGravity ( false );

            // Linear spring to control position
            Vec3f force= (position-p->getPosition())*controlStiffness;
            p->setForce ( p->getForce() + force );

            // Angular spring to control orientation
            H3DUtil::Quaternion cur ( p->getOrientation() );
            H3DUtil::Quaternion target ( orientation );
            H3DUtil::Quaternion springTorque= (cur*target.inverse()).inverse();
            p->setTorque ( p->getTorque() + springTorque.toEulerAngles()*controlTorqueStiffness );

            if ( feedback==UpdateForceFeedbackType::Feedback_SpringDamperProjected ) {
              Vec3f sep= position-p->getPosition();
              Vec3f projSep= sep.dotProduct ( lastNormal ) * lastNormal;
              reactionForce->addSpring ( p->getPosition(), p->getPosition()+projSep, hapticStiffnessSpringDamper );

              // Set the center of mass of the proxy for torque calculations
              reactionForce->setCenterOfMass ( p->getPosition()+projSep );
            } else {
              reactionForce->addSpring ( p->getPosition(), position, hapticStiffnessSpringDamper );

              // Set the center of mass of the proxy for torque calculations
              reactionForce->setCenterOfMass ( position );
            }

            reactionForce->addTorqueSpring ( p->getOrientation(), orientation, hapticTorqueStiffnessSpringDamper );

          } else {
            // Control the object kinematically
            p->setKinematicControl ( true );
            p->setPosition ( position );
            p->setOrientation ( orientation );
          }

          // Transfer current springs for rendering
          if ( hd.getThread() ) {
            reactionForce->transferSprings( *hd.getThread() );
          }

      } else
        if ( feedback==UpdateForceFeedbackType::Feedback_None ) {
          // Set the position and orientation of the kinematic deformer
          p->setKinematicControl ( true );
          p->setPosition ( position );
          p->setOrientation ( orientation );
        }

        // Update rigid body parameters
        softBodyParams.getEngine()->setRigidBodyParameters( bodyId, *p );
  }
}

void ReactionForce::setCenterOfMass ( Vec3f p ) {
  centerOfMass= p;
}

void ReactionForce::addSpring ( Vec3f p1, Vec3f p2, H3DFloat k ) {
  springs.push_back ( Spring ( p1, p2, k ) );
}

void ReactionForce::addTorqueSpring ( H3DUtil::Rotation r1, H3DUtil::Rotation r2, H3DFloat k ) {
  torqueSprings.push_back ( TorqueSpring ( r1, r2, k ) );
}

// Transfer springs to haptic thread
void ReactionForce::transferSprings ( PeriodicThreadBase& thread ) {
  thread.synchronousCallback ( transferSpringsCB, this );
  springs.clear();
  torqueSprings.clear();
}

ReactionForce::EffectOutput ReactionForce::calculateForces( const EffectInput &input ) {

  Vec3f totalForce;
  Vec3f totalTorque;

  for ( SpringVector::iterator i= springsCurrent.begin(); i != springsCurrent.end(); ++i ) {
    Spring s= *i;
    Vec3f force= s.getForce();

    totalForce+= force;
    Vec3f leverArm= s.p2 - centerOfMassCurrent;
    totalTorque+= leverArm.crossProduct( force );
  }

  for ( TorqueSpringVector::iterator i= torqueSpringsCurrent.begin(); i != torqueSpringsCurrent.end(); ++i ) {
    totalTorque+= (*i).getTorque();
  }

  // Output torque for debugging
  //cout << "Torque: " << totalTorque << endl;

  return EffectOutput( totalForce, totalTorque );
}

// Callback to transfer springs to haptic thread
PeriodicThread::CallbackCode ReactionForce::transferSpringsCB ( void* data ) {
  ReactionForce* forceEffect= static_cast<ReactionForce*>(data);
  forceEffect->springsCurrent= forceEffect->springs;
  forceEffect->torqueSpringsCurrent= forceEffect->torqueSprings;
  forceEffect->centerOfMassCurrent= forceEffect->centerOfMass;
  return PeriodicThread::CALLBACK_DONE;
}

KinematicSoftBodyModifier::UpdateForceFeedbackType::FeedbackType 
KinematicSoftBodyModifier::UpdateForceFeedbackType::getFeedbackType () {
  mutex.lock();
  FeedbackType tmp= feedbackType;
  mutex.unlock();
  return tmp;
}

void KinematicSoftBodyModifier::UpdateForceFeedbackType::update () {
  if ( routes_in.size()>0 ) {
    mutex.lock();
    string type= static_cast<SFString*>(routes_in[0])->getValue();
    if ( type == "NONE" ) {
      feedbackType= Feedback_None;
    } else if ( type == "SPRING_DAMPER" ) {
      feedbackType= Feedback_SpringDamper;
    } else if ( type == "SPRING_DAMPER_PROJECTED" ) {
      feedbackType= Feedback_SpringDamperProjected;
    } else if ( type == "CONTACT_DEPTH" ) {
      feedbackType= Feedback_ContactDepth;
    }
    mutex.unlock();
  }
}