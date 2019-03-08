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

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )

positionSoftBody1.setValue( Vec3f(0.0, 0.1, 0.0 ) )
orientationSoftBody1.setValue( Rotation(0, 0, 1, 1.57 ) )
scaleSoftBody1.setValue( Vec3f(0.05, 0.05, 0.05 ) )
