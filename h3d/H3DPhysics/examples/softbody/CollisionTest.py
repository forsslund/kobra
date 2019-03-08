from H3DInterface import *
import GetTransforms

sbc, collisionSensor, root, ks= references.getValue()

class DebugText ( AutoUpdate ( TypedField ( MFString, MFNode ) ) ):

  def update ( self, value ):
    strings= []
    contacts= value.getValue()
    
    for c in contacts:
      s= "Contact: " + str(c.body1.getValue()) + " and " + str(c.body2.getValue())
      strings.append ( s )
      s= " --> Geometries: " + str(c.geometry1.getValue()) + " and " + str(c.geometry2.getValue())
      strings.append ( s )
      
    return strings

class DisplayContacts ( AutoUpdate ( MFNode ) ):

  def __init__ ( self ):
    AutoUpdate ( MFNode ).__init__ ( self )
    self.contacts= []
  
  def clear ( self ):
    for i in range(len(self.contacts)):
      c= self.contacts[0]
      root.children.erase ( c )
      self.contacts.remove ( c )
  
  def update ( self, value ):
    contacts= value.getValue()
    if len(contacts) > len(self.contacts):
      for i in range ( len(contacts)-len(self.contacts) ):
        contact= createX3DNodeFromString ( 
          """ <Transform>
                <Shape>
                  <Appearance>
                    <Material diffuseColor='1 0 0'/>
                  </Appearance>
                  <Sphere radius='0.005' />
                </Shape>
              </Transform> """ ) [0]
        root.children.push_back ( contact )
        self.contacts.append ( contact )
        
    elif len(contacts) < len(self.contacts):
      for i in range ( len(self.contacts)-len(contacts) ):
        contact= self.contacts[0]
        root.children.erase ( contact )
        self.contacts.remove ( contact )
        
    for i in range(len(contacts)):
      self.contacts[i].translation.setValue ( contacts[i].position.getValue() )
      
    return contacts

text= DebugText ()
displayContacts= DisplayContacts ()
    
class KeyDown ( AutoUpdate ( SFString ) ):

  debugOn= False

  def update ( self, event ):
    k= event.getValue().lower()
    if k == 'c':
      if self.debugOn:
        collisionSensor.contacts.unroute ( text )
        collisionSensor.contacts.unroute ( displayContacts )
        text.setValue ( [] )
        displayContacts.clear()
        self.debugOn= False
      else:
        collisionSensor.contacts.route ( text )
        collisionSensor.contacts.route ( displayContacts )
        self.debugOn= True
    return k
    
keyDown= KeyDown()
ks.keyPress.route ( keyDown )

# Updates the transformation of the soft body.
positionSoftBody1 = SFVec3f()
orientationSoftBody1 = SFRotation()
scaleSoftBody1 = SFVec3f()
getTransformSoftBody1 = GetTransforms.GetTransform()

positionSoftBody2 = SFVec3f()
orientationSoftBody2 = SFRotation()
scaleSoftBody2 = SFVec3f()
getTransformSoftBody2 = GetTransforms.GetTransform()

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

positionSoftBody2.route( getTransformSoftBody2 )
orientationSoftBody2.route( getTransformSoftBody2 )
scaleSoftBody2.route( getTransformSoftBody2 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )
getTransformSoftBody2.route( sbc.softBodies.getValue()[1].transform )

positionSoftBody1.setValue( Vec3f(0, 0, 0 ) )
orientationSoftBody1.setValue( Rotation( 1, 0, 0, 1.57 ) )
scaleSoftBody1.setValue( Vec3f(0.003, 0.003, 0.003 ) )

positionSoftBody2.setValue( Vec3f(0, 0.5, 0 ) )
orientationSoftBody2.setValue( Rotation( 1, 0, 0, 1.57 ) )
scaleSoftBody2.setValue( Vec3f(0.003, 0.003, 0.003 ) )
