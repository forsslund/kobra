from H3DInterface import *

rbc, collisionSensor, root, ks= references.getValue()

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