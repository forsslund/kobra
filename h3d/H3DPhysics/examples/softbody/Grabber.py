from H3DInterface import *

# Provides a simple 'grabbing' function for soft bodies
# 
# When device is touching a soft body and button is depressed
# a virtual spring links the device position to the closest vertex
# of the soft body. 
#
# The force is applied to the vertex using the VertexForce node, and
# the opposite force is applied to the device using a ForceField node.
#
# Call addGrabbersForSoftBodies() to add handlers for all soft bodies.

class SFBoolOr ( SFBool ):
  """ Field has value True if any field routed to it is True, otherwise False. """
  
  def update ( self, event ):
    values= self.getRoutesIn()
    for v in values:
      if v.getValue():
        return True
    return False

avoid_circular_dependency_list = []

class KeyDown ( AutoUpdate ( SFBool ) ):

  class OnKeyDown ( AutoUpdate ( SFString ) ):
  
    def __init__ ( self, key, keySensor, owner ):
      AutoUpdate ( SFString ).__init__ ( self )
      self.__key= key.lower()
      self.__owner= owner
      keySensor.keyPress.route ( self )
    
    def update ( self, event ):
      k= event.getValue()
      if k.lower() == self.__key:
        self.__owner.setValue ( True )
      return k
      
  class OnKeyUp ( AutoUpdate ( SFString ) ):
  
    def __init__ ( self, key, keySensor, owner ):
      AutoUpdate ( SFString ).__init__ ( self )
      self.__key= key.lower()
      self.__owner= owner
      keySensor.keyRelease.route ( self )
    
    def update ( self, event ):
      k= event.getValue()
      if k.lower() == self.__key:
        self.__owner.setValue ( False )
      return k
    
  def __init__ ( self, key, keySensor ):
    AutoUpdate ( SFBool ).__init__ ( self )
    self.setValue ( False )
    global avoid_circular_dependency_list
    avoid_circular_dependency_list.append( KeyDown.OnKeyDown ( key, keySensor, self ) )
    avoid_circular_dependency_list.append( KeyDown.OnKeyUp ( key, keySensor, self ) )

class GrabSpring ( TypedField ( MFVec3f, SFVec3f ) ):

  def __init__ ( self, index, coords, k ):
    TypedField ( MFVec3f, SFVec3f ).__init__ ( self )
    self.index= index
    self.coords= coords
    self.k= k
    
  def update ( self, event ):
    devP= event.getValue()
    points= self.coords.point.getValue()
    p= points[self.index]
    f= (devP-p)*self.k
    return [f]

class DeviceForce ( TypedField ( SFVec3f, MFVec3f ) ):

  def __init__ ( self, strength ):
    TypedField ( SFVec3f, MFVec3f ).__init__ ( self )
    self.strength= strength

  def update ( self, event ):
    forces= event.getValue()
    return -forces[0]*self.strength
    
class GrabHandler ( AutoUpdate ( SFBool ) ):

  def __init__ ( self, device, sbCollection, softBody, root, k= 50.0, forceFactor= 0.5 ):
    AutoUpdate ( SFBool ).__init__ ( self )
    self.device= device
    self.softBody= softBody
    self.sbCollection= sbCollection
    self.vertexForce = createNode( "VertexBodyForce" )
    self.vertexForce.body1.setValue( softBody )
    sbCollection.modifiers.push_back ( self.vertexForce )
    self.grabSpring= None
    di= getActiveDeviceInfo()
    if di:
      self.deviceIndex= di.device.getValue().index ( device )
    else:
      self.deviceIndex= 0
    self.k= k
    self.forceFactor= forceFactor
    self.forceEffect= createNode ( 'ForceField' )
    root.children.push_back ( self.forceEffect )

  def update ( self, event ):
    if ( event.getValue() ):
      # Only one surface geometry assumed.
      surface = 0
      if( len( self.softBody.surfaceGeometry.getValue() ) > 0 ) :
        surface= self.softBody.surfaceGeometry.getValue()[0]
      if not surface:
        surface= self.softBody.geometry.getValue()

      touchingSurface= surface.isTouched.getValue()
      if ( self.deviceIndex < len(touchingSurface) and touchingSurface[self.deviceIndex] ):
    
        mesh= self.softBody.geometry.getValue()
        coords= mesh.coord.getValue().point.getValue()
        minD= 10000
        for i in range(len(coords)):
          p= coords[i]
          d= (p-self.device.proxyPosition.getValue()).length()
          if d < minD:
            minD= d
            minI= i
        
        self.vertexForce.index.push_back ( minI )
        
        self.grabSpring= GrabSpring ( minI, mesh.coord.getValue(), self.k )
        self.device.trackerPosition.route ( self.grabSpring )
        self.grabSpring.route ( self.vertexForce.forces )
        
        self.deviceForce= DeviceForce ( self.forceFactor )
        self.grabSpring.route ( self.deviceForce )
        self.deviceForce.route ( self.forceEffect.force )
      
    else:
      self.grabSpring= None
      self.deviceForce= None
      self.vertexForce.forces.clear()
      self.vertexForce.index.clear()
      self.forceEffect.force.setValue ( Vec3f() )
      
    return event.getValue()


grabHandlers= []

def addGrabbersForSoftBody ( sbCollection, softBody, root, k= 50.0, forceFactor= 0.5, keySensor= None ):
  di= getActiveDeviceInfo()
  if di:
    for d in di.device.getValue():
      handler= GrabHandler( d, sbCollection, softBody, root, k, forceFactor)
      
      handler.triggerDemux= SFBoolOr()
      d.mainButton.route ( handler.triggerDemux )
      
      if keySensor:
        handler.triggerKey= KeyDown ( 'x', keySensor )
        handler.triggerKey.route ( handler.triggerDemux )
      
      handler.triggerDemux.route (  handler )
      
      grabHandlers.append ( handler )
      
def addGrabbersForSoftBodies ( softBodyCollection, root, k= 50.0, forceFactor= 0.5, keySensor= None ):
  bodies= softBodyCollection.softBodies.getValue()
  for sb in bodies:
    addGrabbersForSoftBody ( softBodyCollection, sb, root, k, forceFactor, keySensor )