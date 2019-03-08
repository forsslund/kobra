from H3DInterface import *

rbc, body2, collisionSensor= references.getValue()

rbc.enabled.setValue(True)

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

text= DebugText ()
collisionSensor.contacts.route ( text )