from H3DInterface import *
import GetTransforms

sb1,sb2,sbc,ks= references.getValue()

linearJoint= None

class KeyHandler ( AutoUpdate ( SFString ) ):
  def update ( self, value ):
    global linearJoint
    
    key= value.getValue().lower()
    
    if key == 'j':
      if not linearJoint:
        linearJoint= createNode( "SoftBodyLinearJoint" )
        linearJoint.body1.setValue ( sb1 )
        linearJoint.body2.setValue ( sb2 )
        sbc.constraints.push_back ( linearJoint )
        
        di= getActiveDeviceInfo()
        if di:
          devices= di.device.getValue()
          if len(devices)>0:
            linearJoint.anchorPoint.setValue ( devices[0].devicePosition.getValue() )
            print("anchorPoint at: " + str(linearJoint.anchorPoint.getValue()))
      else:
        sbc.constraints.erase ( linearJoint )
        linearJoint= None
        
    return key
        
keyHandler= KeyHandler()
ks.keyPress.route ( keyHandler )

# Updates the transformation of the soft body.
positionSoftBody1 = SFVec3f()
orientationSoftBody1 = SFRotation()
scaleSoftBody1 = SFVec3f()
getTransformSoftBody1 = GetTransforms.GetTransform()

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )

positionSoftBody1.setValue( Vec3f(-0.1, 0.05, 0.0 ) )
scaleSoftBody1.setValue( Vec3f(0.005, 0.005, 0.005 ) )
