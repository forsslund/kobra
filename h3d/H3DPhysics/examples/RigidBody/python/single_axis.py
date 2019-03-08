from H3DInterface import *

group, joint = references.getValue()


class JointToText( TypedField( MFString, (SFFloat, SFFloat, SFVec3f, SFVec3f ) ) ):
  def update( self, event ):
    routes_in = self.getRoutesIn()
    s = []
    s.append( "Angle: " + str( routes_in[0].getValue() ) )
    s.append( "Angle rate: " + str( routes_in[1].getValue() ) )
    s.append( "Body1 anchor point: " + str( routes_in[2].getValue() ) )
    s.append( "Body2 anchor point: " + str( routes_in[3].getValue() ) )
    return s

joint_to_text = JointToText()

joint.angle.route( joint_to_text )
joint.angleRate.route( joint_to_text )
joint.body1AnchorPoint.route( joint_to_text )
joint.body2AnchorPoint.route( joint_to_text )

text_node = createX3DNodeFromString( "<Text><FontStyle /></Text>" )[0]

joint_to_text.route( text_node.string )

t = createNode( "Transform" )
t.scale.setValue( Vec3f( 0.03, 0.03, 0.03 ) )
t.translation.setValue( Vec3f( -0.4, 0.5, 0 ) )
t.children.push_back( text_node )
group.children.push_back( t )
  
