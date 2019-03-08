from H3DInterface import *

sbc = references.getValue()[0]

# Updates the transformation parameters of the soft body.
class GetTransform(TypedField(SFMatrix4f,(SFVec3f,SFRotation, SFVec3f))):
  
  def update(self,event):
    
    routes_in = self.getRoutesIn()
    p = routes_in[0].getValue()
    orient = routes_in[1].getValue()
    s = routes_in[2].getValue()
    
    return Matrix4f( p, orient, s )

positionSoftBody1 = SFVec3f()
orientationSoftBody1 = SFRotation()
scaleSoftBody1 = SFVec3f()
getTransformSoftBody1 = GetTransform()

positionSoftBody2 = SFVec3f()
orientationSoftBody2 = SFRotation()
scaleSoftBody2 = SFVec3f()
getTransformSoftBody2 = GetTransform()

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

positionSoftBody2.route( getTransformSoftBody2 )
orientationSoftBody2.route( getTransformSoftBody2 )
scaleSoftBody2.route( getTransformSoftBody2 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )
getTransformSoftBody2.route( sbc.softBodies.getValue()[1].transform )

positionSoftBody1.setValue( Vec3f(-0.1, 0.01, 0.0 ) )
scaleSoftBody1.setValue( Vec3f(0.002, 0.002, 0.002 ) )

positionSoftBody2.setValue( Vec3f(0.1, 0.05, 0.0 ) )
scaleSoftBody2.setValue( Vec3f(0.003, 0.003, 0.003 ) )
