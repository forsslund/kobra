from H3DInterface import *

# Provides a class to convert position, scale and orientation to a
# transform.
# 
# Create an instance of the GetTransform class which local position,
# orientation and scale variables are routed to. The route this instance to
# transform field of a soft body in order to update its transformation.
# 

class GetTransform(TypedField(SFMatrix4f,(SFVec3f,SFRotation, SFVec3f))):
  
  def update(self,event):
    
    routes_in = self.getRoutesIn()
    p = routes_in[0].getValue()
    orient = routes_in[1].getValue()
    s = routes_in[2].getValue()
    
    return Matrix4f( p, orient, s )

