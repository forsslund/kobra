from H3DInterface import *

di= getActiveDeviceInfo()
if di:
  devices= di.device.getValue()
  for d in devices:
    # Use Ruspini render to help avoid fall through and configure proxy radius
    d.hapticsRenderer.setValue ( createX3DNodeFromString ( "<RuspiniRenderer proxyRadius='0.008'/>" )[0] )
    
    sphere, dn= createX3DNodeFromString ( """  <Shape>
                            <Appearance>
                              <Material />
                            </Appearance>
                            <Sphere DEF='Sphere' />
                          </Shape>""" )
    d.hapticsRenderer.getValue().proxyRadius.route ( dn['Sphere'].radius )
    d.stylus.setValue ( sphere )
    
# Position of text messages
class TextPos( AutoUpdate(TypedField(SFVec3f, (SFVec3f, SFRotation ) ) ) ):

  def update( self, event ):
    routes_in = self.getRoutesIn();
    v = routes_in[0].getValue()
    r = Rotation()
    if len( routes_in ) > 1:
      r = routes_in[1].getValue()
    v_temp = Vec3f( v.x, v.y, v.z )
    stereo_info = getActiveStereoInfo()
    focal_distance = 0.6
    if stereo_info:
      focal_distance = stereo_info.focalDistance.getValue()
    v_new = v - r * Vec3f( 0, 0, focal_distance )
    return v_new

textPos = TextPos()