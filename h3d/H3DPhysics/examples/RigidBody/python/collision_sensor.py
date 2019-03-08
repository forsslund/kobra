from H3DInterface import *
import string

group, cs, rbc  = references.getValue()


class ContactToText( TypedField( MFString, MFNode ) ):
  def update( self, event ):
    routes_in = self.getRoutesIn()
    contacts = routes_in[0].getValue()

    nr_contacts = len( contacts )
    s = []
    s.append( "Nr contacts: " + str( nr_contacts ) )

    if( nr_contacts > 0 ):
      c = contacts[0]
      s.append( "AP: " + string.join(c.appliedParameters.getValue() ) )
      s.append( "Normal: " + str( c.contactNormal.getValue() ) )
      s.append( "Point: " + str( c.position.getValue() ) )
      s.append( "Depth: " + str( c.depth.getValue() ) )
      s.append( "Bounce: " + str( c.bounce.getValue() ) )
      s.append( "BSpeed: " + str( c.minBounceSpeed.getValue() ) )
      s.append( "FC: " + str( c.frictionCoefficients.getValue() ) )
      s.append( "FDir: " + str( c.frictionDirection.getValue() ) )
      s.append( "Slip: " + str( c.slipCoefficients.getValue() ) )
      s.append( "SCFM: " + str( c.softnessConstantForceMix.getValue() ) )
      s.append( "SEC: " + str( c.softnessErrorCorrection.getValue() ) )
      s.append( "SSpeed: " + str( c.surfaceSpeed.getValue() ) )
                
    return s

contact_to_text = ContactToText()

cs.contacts.route( contact_to_text )


text_node = createX3DNodeFromString( "<Text><FontStyle /></Text>" )[0]

contact_to_text.route( text_node.string )

t = createNode( "Transform" )
t.scale.setValue( Vec3f( 0.03, 0.03, 0.03 ) )
t.translation.setValue( Vec3f( -0.4, 0.5, 0 ) )
t.children.push_back( text_node )
group.children.push_back( t )
  


#############################

class RBCToText( TypedField( MFString, (SFInt32, SFTime ) ) ):
  def update( self, event ):
    routes_in = self.getRoutesIn()

    s = []
    s.append( "Update rate: " + str( routes_in[0].getValue() ) )
    s.append( "Time last: " + str( routes_in[1].getValue() ) )
                
    return s

rbc_to_text = RBCToText()

rbc.updateRate.route( rbc_to_text )
rbc.stepUpdateTime.route( rbc_to_text )

rbc_text_node = createX3DNodeFromString( "<Text><FontStyle /></Text>" )[0]

rbc_to_text.route( rbc_text_node.string )

t = createNode( "Transform" )
t.scale.setValue( Vec3f( 0.02, 0.03, 0.03 ) )
t.translation.setValue( Vec3f( 0.3, 0.5, 0 ) )
t.children.push_back( rbc_text_node )
group.children.push_back( t )
