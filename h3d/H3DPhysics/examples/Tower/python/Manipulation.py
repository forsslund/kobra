from H3DInterface import *

# Haptic interpolation speed
springPositionInterpolation= 0.03

class TouchSelector:
  """ A class which provides a method of selecting a body from the scene. Allows
      for some abstraction when implementing manipulation methods."""

  def __init__ ( self, deviceIndex ):
    self.bodyToGeometry= {}
    self.deviceIndex= deviceIndex
  
  def select ( self, bodies ):
    """ Returns the selected object from the specified list. May be None if no object
        was selected this time."""
    for b in bodies:
      for cs in b.geometry.getValue():
        touched= cs.shape.getValue().geometry.getValue().isTouched.getValue()
        if len(touched) > self.deviceIndex and touched[self.deviceIndex]:
          return b

      # Check additional dictionary of geometries
      if str(b) in self.bodyToGeometry:
        touched= self.bodyToGeometry[str(b)].isTouched.getValue()
        if len(touched) > self.deviceIndex and touched[self.deviceIndex]:
          return b

    return None

class ManipulatorCollection:

  class SumForces ( TypedField ( MFVec3f, None, SFVec3f ) ):
      """ A field used to sum all the forces routed to it. Creates a MFVec3f containing the result as the only element."""
      def update ( self, event ):
        forces= self.getRoutesIn ()
        total= Vec3f()
        for f in forces:
          total+= f.getValue()
        return [total]

  class BodyRoutes:
    def __init__ ( self, body ):
      self.totalForce= ManipulatorCollection.SumForces()
      self.totalTorque= ManipulatorCollection.SumForces()
      self.totalForce.route ( body.forces )
      self.totalTorque.route ( body.torques )
      self.manipulators= []

    def maxLinearDamping ( self ):
      maxDamping= 0
      for m in self.manipulators:
        if m.linearDamping > maxDamping:
          maxDamping= m.linearDamping
      return maxDamping

    def maxAngularDamping ( self ):
      maxDamping= 0
      for m in self.manipulators:
        if m.angularDamping > maxDamping:
          maxDamping= m.angularDamping
      return maxDamping

  def __init__ ( self ):
    self.manipulators= {}

  def addManipulator ( self, manipulator ):
    if not str(manipulator.body) in self.manipulators:
      bodyRoutes= ManipulatorCollection.BodyRoutes ( manipulator.body )
      self.manipulators[str(manipulator.body)]= bodyRoutes

      # First manipulator on body, add damping to body
      manipulator.body.autoDamp.setValue ( True )
      
    else:
      bodyRoutes= self.manipulators[str(manipulator.body)]

    manipulator.totalForce.route ( bodyRoutes.totalForce )
    manipulator.totalTorque.route ( bodyRoutes.totalTorque )
    bodyRoutes.manipulators.append ( manipulator )

    # Use highest damping required by a single active manipulator
    manipulator.body.linearDampingFactor.setValue ( bodyRoutes.maxLinearDamping () )
    manipulator.body.angularDampingFactor.setValue ( bodyRoutes.maxAngularDamping () )
    manipulator.body.autoDamp.setValue ( False ) # Work around (for bug in RBP?)
    manipulator.body.autoDamp.setValue ( True )

  def removeManipulator ( self, manipulator ):
    if str(manipulator.body) in self.manipulators:
      bodyRoutes= self.manipulators[str(manipulator.body)]
      manipulator.totalForce.unroute ( bodyRoutes.totalForce )
      manipulator.totalTorque.unroute ( bodyRoutes.totalTorque )
      bodyRoutes.manipulators.remove ( manipulator )
      manipulator.body.forces.clear()
      manipulator.body.torques.clear()
      if len(bodyRoutes.totalForce.getRoutesIn()) <= 0:
        del self.manipulators[str(manipulator.body)]
        manipulator.body.forces.clear()
        manipulator.body.torques.clear()

        # Last manipulator on body, turn damping off
        manipulator.body.autoDamp.setValue ( False )

class Manipulator ( object ):
  """ Base for classes that manipulate rigid bodies."""

  class SumForces ( TypedField ( SFVec3f, None, SFVec3f ) ):
    """ A field used to sum all the forces routed to it. Creates a MFVec3f containing the result as the only element."""
    def update ( self, event ):
      forces= self.getRoutesIn ()
      total= Vec3f()
      for f in forces:
        total+= f.getValue()
      return total

  class DampForce ( SFVec3f ):
    """ A field used to calculate a damping force in the opposite direction to the input field."""
    def __init__ ( self, dampingFactor ):
      SFVec3f.__init__ ( self )
      self.dampingFactor= dampingFactor
      
    def update ( self, event ):
      return -event.getValue()*self.dampingFactor

  def __init__ ( self ):
    """ Constructor."""

    # Inputs controling the manipulation
    # Sub-classes may route to these (e.g. from haptic device fields)
    self.inputPosition= SFVec3f()
    self.inputRotation= SFRotation()

    # Total forces on body being manipulated
    self.totalForce= Manipulator.SumForces()
    self.totalTorque= Manipulator.SumForces()

    # Body being manipulated
    self.body= None

    # Levels of damping required by this manipulator
    self.linearDamping= 1
    self.angularDamping= 0.01

  def start ( self, body ):
    """ Start manipulating the specified body."""

    if not self.body:
      self.body= body
      self.startManipulation()

  def stop ( self ):
    """ Stop manipulating the current body."""

    if self.body:
      self.stopManipulation()
      self.body= None

  def startManipulation ( self ):
    """ Start manipulating the current body. Subclasses override to implement Manipulator."""

  def stopManipulation ( self ):
    """ Start manipulating the specified body. Subclasses override to implement Manipulator."""


class ManipulatorVisualisation ( object ):
  """ Base class for visualisations of manipulations."""

  def __init__ ( self ):
    self.manipulator= None

class SpringManipulatorVisualisation ( ManipulatorVisualisation ):
  """ A visualisation of the spring manipulator."""

  class DrawLine ( TypedField ( MFVec3f, ( SFVec3f, SFVec3f ) ) ):
    """ Field used to route to the line geometry. Creates a MFVec3f containing the start and end points routed in."""
    def update ( self, event ):
      start, end= self.getRoutesIn()
      return [start.getValue(), end.getValue()]

  def __init__ ( self ):
    super ( SpringManipulatorVisualisation, self ).__init__(  )
    
    self.manipulatorPosition= SFVec3f()
    self.manipulatorRotation= SFRotation()
    self.anchorPosition= SFVec3f()
    self.anchorRotation= SFRotation()

    self.drawLine= SpringManipulatorVisualisation.DrawLine ()
    self.manipulatorPosition.route ( self.drawLine )
    self.anchorPosition.route ( self.drawLine )

    # Create some shapes to represent the manipulation: A ball to represent the anchor point,
    # and a line representing the virtual spring
    self.node, dn= createX3DFromString ( """<Group>
                                              <Transform DEF='Sphere_T'>
                                                <Shape>
                                                  <Appearance>
                                                    <Material diffuseColor='0 0 1' />
                                                  </Appearance>
                                                  <Sphere radius='0.004' />
                                                </Shape>
                                              </Transform>
                                              <Shape>
                                                <Appearance>
                                                  <Material emissiveColor='0 0 1' />
                                                </Appearance>
                                                <LineSet vertexCount='2'>
                                                  <Coordinate DEF='Line_C' />
                                                </LineSet>
                                              </Shape>
                                              <!--<Transform DEF='RotationMarker_T'>
                                                <Shape>
                                                  <Appearance>
                                                    <Material emissiveColor='0 1 0' />
                                                  </Appearance>
                                                  <LineSet vertexCount='2'>
                                                    <Coordinate point='0 0 0, 0 0 0.05' />
                                                  </LineSet>
                                                </Shape>
                                              </Transform>
                                              <Transform DEF='RotationMarkerBody_T'>
                                                <Shape>
                                                  <Appearance>
                                                    <Material emissiveColor='0 0 1' />
                                                  </Appearance>
                                                  <LineSet vertexCount='2'>
                                                    <Coordinate point='0 0 0, 0 0 0.05' />
                                                  </LineSet>
                                                </Shape>
                                              </Transform>-->
                                            </Group>""" )

    # Set up routes to update the graphic representation of the manipulation
    self.drawLine.route ( dn['Line_C'].point )
    self.anchorPosition.route ( dn['Sphere_T'].translation )

  def start ( self, manipulator ):
    if self.manipulator is None:
      manipulator.springPosition.route ( self.anchorPosition )
      manipulator.springRotation.route ( self.anchorRotation )
      manipulator.inputPosition.route ( self.manipulatorPosition )
      manipulator.body.orientation.route ( self.manipulatorRotation )
      self.manipulator= manipulator

  def stop ( self ):
    if self.manipulator is not None:
      self.manipulator.springPosition.unroute ( self.anchorPosition )
      self.manipulator.springRotation.unroute ( self.anchorRotation )
      self.manipulator.inputPosition.unroute ( self.manipulatorPosition )
      self.manipulator.body.orientation.unroute ( self.manipulatorRotation )
      self.manipulator= None

class SpringBallManipulatorVisualisation ( ManipulatorVisualisation ):
  """ A visualisation of the spring ball manipulator."""

  def __init__ ( self ):
    super ( SpringBallManipulatorVisualisation, self ).__init__(  )
    
    self.manipulatorPosition= SFVec3f()
    self.anchorPosition= SFVec3f()

    self.drawLine= SpringManipulatorVisualisation.DrawLine ()
    self.manipulatorPosition.route ( self.drawLine )
    self.anchorPosition.route ( self.drawLine )

    # Create some shapes to represent the manipulation: A ball to represent the anchor point,
    # and a line representing the virtual spring
    self.node, dn= createX3DFromString ( """<Group>
                                              <Transform DEF='AnchorSphere_T'>
                                                <Shape>
                                                  <Appearance>
                                                    <Material diffuseColor='0 1 0' />
                                                  </Appearance>
                                                  <Sphere radius='0.004' />
                                                </Shape>
                                              </Transform>
                                              <Shape>
                                                <Appearance>
                                                  <Material emissiveColor='1 0 0' />
                                                </Appearance>
                                                <LineSet vertexCount='2'>
                                                  <Coordinate DEF='Line_C' />
                                                </LineSet>
                                              </Shape>
                                            </Group>""" )

    # Set up routes to update the graphic representation of the manipulation
    self.drawLine.route ( dn['Line_C'].point )
    self.anchorPosition.route ( dn['AnchorSphere_T'].translation )

  def start ( self, manipulator ):
    if self.manipulator is None:
      manipulator.springPosition.route ( self.anchorPosition )
      manipulator.inputPosition.route ( self.manipulatorPosition )
      self.manipulator= manipulator

  def stop ( self ):
    if self.manipulator is not None:
      self.manipulator.springPosition.unroute ( self.anchorPosition )
      self.manipulator.inputPosition.unroute ( self.manipulatorPosition )
      self.manipulator= None

class SpringManipulator ( Manipulator ):
  """ Provides a means of manipulating a body using a spring connected directly to the body. """

  class SpringPosition ( TypedField ( SFVec3f, (SFVec3f, SFRotation) ) ):
    """ Calculates the anchor position of the spring effect from the body's position, orientation
        and the initial offset for the current manipulation"""

    def __init__ ( self, offset= Vec3f() ):
      TypedField ( SFVec3f, (SFVec3f, SFRotation) ).__init__ ( self )
      self.offset= offset
          
    def update ( self, event ):
      position, orientation= self.getRoutesIn()
      return Matrix4f(orientation.getValue())*self.offset+position.getValue()

  class SpringForce ( TypedField ( SFVec3f, ( SFVec3f, SFVec3f ) ) ):
    """ Field used to calculate a spring force based on the anchor point and end point routed in."""
    
    def __init__ ( self, k ):
      TypedField ( SFVec3f, ( SFVec3f, SFVec3f ) ).__init__ ( self )
      self.k= k
    
    def update ( self, event ):
      anchor, point= self.getRoutesIn()
      return -(anchor.getValue()-point.getValue())*self.k

  class SpringRotation ( SFRotation ):
    """ Calculate the angle of the torque spring based on initial rotation and current rotation."""
    
    def __init__ ( self, initialRotation= Rotation() ):
      SFRotation.__init__ ( self )
      self.initialRotation= initialRotation

    def update ( self, event ):
      return event.getValue()*self.initialRotation

  class SpringTorque ( TypedField ( SFVec3f, ( SFRotation, SFRotation ) ) ):
    """ Field used to calculate a torque based on a rotational spring from the anchor and current rotation
        axes routed to it."""
    def __init__ ( self, k ):
      TypedField ( SFVec3f, ( SFRotation, SFRotation ) ).__init__ ( self )
      self.k= k
    
    def update ( self, event ):
      current, anchor= self.getRoutesIn()

      cur= Quaternion(current.getValue())
      target= Quaternion(anchor.getValue())

      springTorque= (cur*target.inverse()).inverse()

      springTorque= springTorque*springTorque
      return springTorque.toEulerAngles() * self.k

  def __init__ ( self ):
    super ( SpringManipulator, self ).__init__ ()
    self.springPosition= SpringManipulator.SpringPosition ()
    self.springRotation= SpringManipulator.SpringRotation ()

    # Some constants for this manipulator

    # Levels of damping required by this manipulator
    self.linearDamping= 10
    self.angularDamping= 0.02

    # Spring constant used to manipulate body
    self.linearSpringConst= 120
    self.angularSpringConst= 0.2

  def startManipulation ( self ):
    """ Start manipulating the current body. """

    super ( SpringManipulator, self ).startManipulation ()

    # This is an offset vector in the local coords of the body being manipulated. It runs from the object's origin the the point of manipulation on the object's surface
    offset= Matrix4f ( self.body.orientation.getValue() ).inverse()*(self.inputPosition.getValue()-self.body.position.getValue())
    self.springPosition.offset= offset

    # Provide all the information required to update the position of the spring as the body moves
    self.body.position.route ( self.springPosition )
    self.body.orientation.route ( self.springPosition )

    self.springForce= SpringManipulator.SpringForce ( self.linearSpringConst )
    self.springPosition.route ( self.springForce )
    self.inputPosition.route ( self.springForce )

    # Route the spring force to the total body forces
    self.springForce.route ( self.totalForce )

    rotationOffset= Rotation ( Matrix3f ( self.body.orientation.getValue() ).inverse()*Matrix3f ( self.inputRotation.getValue() ) )
    self.springRotation.initialRotation= rotationOffset
    self.body.orientation.route ( self.springRotation )

    self.springTorque= SpringManipulator.SpringTorque ( self.angularSpringConst )
    self.springRotation.route ( self.springTorque )
    self.inputRotation.route ( self.springTorque )

    # Route the spring torque to the total body torques
    self.springTorque.route ( self.totalTorque )

  def stopManipulation ( self ):
    """ Stop manipulating the current body. """

    super ( SpringManipulator, self ).stopManipulation ()

    self.body.position.unroute ( self.springPosition )
    self.body.orientation.unroute ( self.springPosition )
    self.body.orientation.unroute ( self.springRotation )
    

class ManipulatorControl ( object ):

  def __init__ ( self, manipulator, visualization, group, manipulatorCollection ):
    
    self.manipulator= manipulator
    self.group= group
    self.visualization= visualization
    self.manipulatorCollection= manipulatorCollection

  def start ( self, body ):

    self.manipulator.start( body )
    self.manipulatorCollection.addManipulator ( self.manipulator )
    
    if self.visualization:
      self.group.children.push_back ( self.visualization.node )
      self.visualization.start( self.manipulator )
    
  def stop ( self ):

    if self.visualization:
      self.group.children.erase ( self.visualization.node )
      self.visualization.stop()

    self.manipulatorCollection.removeManipulator ( self.manipulator )
    self.manipulator.stop()

class HapticManipulatorControl ( ManipulatorControl ):
  """ Controls a manipulation using a haptic device. """

  class TransformPoint ( TypedField ( SFVec3f, ( SFVec3f, SFMatrix4f ) ) ):
      """ Field used to transform the device position to local coordinates of the rootTrans."""

      def update ( self, event ):
        point, transform= self.getRoutesIn()
        return transform.getValue()*point.getValue()

  def __init__ ( self, manipulator, visualization, group, manipulatorCollection, hapticDevice ):
    
    super ( HapticManipulatorControl, self ).__init__ ( manipulator, visualization, group, manipulatorCollection )
    self.hapticDevice= hapticDevice
    self.deviceIndex= 0
    di = getActiveDeviceInfo()
    self.surfaceNodes= []
    if di:
      devices= di.device.getValue()
      self.deviceIndex= devices.index ( self.hapticDevice )

  def start ( self, body ):

    # Must set up these routes before calling parent
    
    # Use the haptic device position, transformed to local coordinates of the
    # rootTrans as the SpringManipulator's 'manipulationPoint'
    self.transformDevicePos= HapticManipulatorControl.TransformPoint()
    self.hapticDevice.weightedProxyPosition.route ( self.transformDevicePos )
    self.group.accumulatedInverse.route ( self.transformDevicePos )
    self.transformDevicePos.route ( self.manipulator.inputPosition )
    
    self.hapticDevice.trackerOrientation.route ( self.manipulator.inputRotation )

    super ( HapticManipulatorControl, self ).start ( body )

    self.removeSurface()
    
  def stop ( self ):
    self.restoreSurface()
    super ( HapticManipulatorControl, self ).stop()

    self.hapticDevice.trackerOrientation.unroute ( self.manipulator.inputRotation )

  def removeSurface ( self ):
    """ Remove the shape's surface haptic surface.
        Subclasses may choose to call this function in startManipulation() to avoid the device bouncing off the body. """
    for cs in self.manipulator.body.geometry.getValue():
      shape = cs.shape.getValue()
      if shape:
        appearance = shape.appearance.getValue()
        if appearance:
          surface= appearance.surface.getValue()
          if surface:
            if( surface.getTypeName() == "MultiDeviceSurface" ) :
              surfaces = surface.surface.getValue()
              self.surfaceNodes.append ( surfaces[self.deviceIndex] )
              surfaces[self.deviceIndex] = None
              surface.surface.setValue( surfaces )
            else:
              appearance.surface.setValue(None)
              self.surfaceNodes.append ( surface )

  def restoreSurface ( self ):
    """ Restore the shape's surface after it was removed with removeSurface().
        Subclasses may choose to call this function in stopManipulation to restore the shape's surface. """
    if len(self.surfaceNodes) > 0:
      self.surfaceNodes.reverse()
      for cs in self.manipulator.body.geometry.getValue():
        appearance= cs.shape.getValue().appearance
        if appearance.getValue():
          surface_node = appearance.getValue().surface.getValue()
          if( surface_node.getTypeName() == "MultiDeviceSurface" ) :
            surfaces = surface_node.surface.getValue()
            surfaces[self.deviceIndex] = self.surfaceNodes.pop()
            surface_node.surface.setValue( surfaces )
          else:
            appearance.getValue().surface.setValue ( self.surfaceNodes.pop() )
      self.surfaceNodes= []

class HapticSpringManipulatorControl ( HapticManipulatorControl ):
  """ Controls a spring manipulation using a haptic device, providing force feedback. """

  def __init__ ( self, manipulator, visualization, group, manipulatorCollection, hapticDevice ):

    super ( HapticSpringManipulatorControl, self ).__init__ ( manipulator, visualization, group, manipulatorCollection, hapticDevice )
    self.springEffect= None

  def start ( self, body ):

    super ( HapticSpringManipulatorControl, self ).start ( body )

    # Get the index of the haptic device to add the force effect for
    index= 0
    di = getActiveDeviceInfo()
    if di:
      devices= di.device.getValue()
      index= devices.index ( self.hapticDevice )

    # Create a spring effect to provide force feedback and to calculate to force to apply to the body
    self.springEffect= createX3DNodeFromString ( "<SpringEffect deviceIndex='" + str(index) +
                                                 "' startDistance='1' escapeDistance='1' springConstant='" + str(self.manipulator.linearSpringConst) + "' " +
                                                 "positionInterpolation='" + str(springPositionInterpolation) + "' />" )[0]
    self.manipulator.springPosition.route ( self.springEffect.position )
    self.group.children.push_back ( self.springEffect )
    
  def stop ( self ):

    super ( HapticSpringManipulatorControl, self ).stop()

    if self.springEffect:
      self.group.children.erase ( self.springEffect )

class HapticAngularSpringManipulatorControl ( HapticSpringManipulatorControl ):
  """ Controls a spring manipulation using a haptic device, providing force and torque feedback. """

  class BlockInContact ( AutoUpdate ( TypedField ( SFBool, MField ) ) ):
  
    def __init__ ( self, body ):
      AutoUpdate ( TypedField ( SFBool, MField ) ).__init__ ( self )
      self.body= body
  
    def update ( self, event ):
      contacts= event.getValue()
      blockInContact= False
      for c in contacts:
        if self.body == c.body1.getValue() or self.body == c.body2.getValue():
          blockInContact= True
          break
      
      return blockInContact
  
  class Ramping ( AutoUpdate ( TypedField ( SFFloat, SFBool ) ) ):
  
    def __init__ ( self, startValue, maxValue, minValue, stepUpSize, stepDownSize, hapticDevice ):
      AutoUpdate ( TypedField ( SFFloat, SFBool ) ).__init__ ( self )
      self.currentValue= startValue
      self.maxValue= maxValue
      self.minValue= minValue
      self.stepUpSize= stepUpSize
      self.stepDownSize= stepDownSize
      self.prevRampOn= True
      self.hapticDevice= hapticDevice
      
    def update ( self, event ):
      blockContact= event.getValue ()
      
      if self.hapticDevice:
        rampOn= blockContact and (self.hapticDevice.force.getValue().length() > 0.1 ) #or self.hapticDevice.torque.getValue().length() > 10 )
      else:
        rampOn= blockContact
      
      if rampOn and self.currentValue < self.maxValue:
        self.currentValue= self.currentValue + self.stepUpSize
        if self.currentValue > self.maxValue:
          self.currentValue= self.maxValue
          
      elif not rampOn and self.currentValue > self.minValue:
        self.currentValue= self.currentValue - self.stepDownSize
        if self.currentValue < self.minValue:
          self.currentValue= self.minValue
    
      self.prevRampOn= rampOn
        
      return self.currentValue
  
  class GraphicRotation ( TypedField ( SFRotation, ( SFRotation, SFRotation ) ) ):
    
    def update ( self, event ):
      if len ( self.getRoutesIn() ) >= 2:
        current, anchor= self.getRoutesIn()

        cur= Quaternion(current.getValue())
        target= Quaternion(anchor.getValue())

        springTorque= (cur*target.inverse())
        rotation= Rotation ( springTorque )

        return current.getValue()

      return Rotation()
      
  class StylusPosition ( TypedField ( SFVec3f, ( SFVec3f, SFMatrix4f ) ) ):
  
    def update ( self, event ):
      devicePosition, containerInverse= self.getRoutesIn()
      return containerInverse.getValue() * devicePosition.getValue()
  
  def __init__ ( self, manipulator, visualization, group, manipulatorCollection, hapticDevice, rigidBodyCollection,
                       stylusBodyTranslation= False, stylusBodyRotation= False ):

    super ( HapticAngularSpringManipulatorControl, self ).__init__ ( manipulator, visualization, group, manipulatorCollection, hapticDevice )
    self.deviceStylus= None
    self.stylusTransform= None
    self.rigidBodyCollection= rigidBodyCollection
    
    # Parameters of manipulation force effect
    self.linearSpringK= 120
    self.angularSpringK= 100
    
    # Number of frames over which to ramp stiffness values
    self.rampUpFrames= 5
    self.rampDownFrames= 2

    # Option to match stylus translation to rigid body position exactly
    self.stylusBodyTranslation= stylusBodyTranslation
    
    # Option to match stylus rotation to rigid body rotation exactly
    self.stylusBodyRotation= stylusBodyRotation

  def start ( self, body ):

    # Skip immediate parent - completely replace function
    super ( HapticSpringManipulatorControl, self ).start( body )

    # Get the index of the haptic device to add the force effect for
    index= 0
    di = getActiveDeviceInfo()
    if di:
      devices= di.device.getValue()
      index= devices.index ( self.hapticDevice )

    # Create a spring effect to provide force feedback and to calculate to force to apply to the body
    self.springEffect= createX3DNodeFromString ( "<SpringEffect deviceIndex='" + str(index) +
                                                 "' startDistance='1' escapeDistance='1' springConstant='" + str(self.manipulator.linearSpringConst) + "' " +
                                                 "positionInterpolation='" + str(springPositionInterpolation) + "' />" )[0]
    self.manipulator.springPosition.route ( self.springEffect.position )
    #self.manipulator.springRotation.route ( self.springEffect.orientation )
    self.group.children.push_back ( self.springEffect )

    self.deviceStylus= self.hapticDevice.stylus.getValue()
    self.hapticDevice.stylus.setValue( None )

    self.stylusTransform= createX3DNodeFromString ( "<Transform />" )[0]
    self.stylusTransform.children.push_back ( self.deviceStylus )
    self.group.children.push_back ( self.stylusTransform )

    self.stylusTransform.scale.setValue ( self.group.accumulatedInverse.getValue().getScalePart() )

    # Stylus to match body position?
    if self.stylusBodyTranslation:
      self.manipulator.springPosition.route ( self.stylusTransform.translation )
    else:
      self.stylusPosition= HapticAngularSpringManipulatorControl.StylusPosition()
      self.hapticDevice.weightedProxyPosition.route ( self.stylusPosition )
      self.group.accumulatedInverse.route ( self.stylusPosition )
      self.stylusPosition.route ( self.stylusTransform.translation )
    
    # Stylus to match body rotation?
    if self.stylusBodyRotation:
      self.graphicRotation= HapticAngularSpringManipulatorControl.GraphicRotation()
      self.manipulator.springRotation.route ( self.graphicRotation )
      self.manipulator.inputRotation.route ( self.graphicRotation )
      self.graphicRotation.route ( self.stylusTransform.rotation )
    else:
      self.hapticDevice.trackerOrientation.route ( self.stylusTransform.rotation )

  def stop ( self ):
    super ( HapticAngularSpringManipulatorControl, self ).stop()

    if self.deviceStylus:
    
        self.hapticDevice.stylus.setValue ( self.deviceStylus )
        self.group.children.erase ( self.stylusTransform )
        
        self.deviceStylus= None
        self.stylusTransform= None
        self.collisionSensor= None
        self.rampedStiffness= None
        self.rampedAngularStiffness= None
        self.graphicRotation= None
        self.stylusPosition= None

class SpringBallManipulator ( Manipulator ):
  """ The SpringManipulator provides a way of interacting with a body using a
      virtual spring between the body and the haptic device. The body is moved using
      a temporary ball joint attached at the point where the body is 'picked up'."""

  def __init__ ( self, rigidBodyCollection, rootTrans ):
    super ( SpringBallManipulator, self ).__init__ ()
    self.springPosition= SpringManipulator.SpringPosition()

    self.rigidBodyCollection= rigidBodyCollection
    self.rootTrans= rootTrans

    # Spring constants used to manipulate body
    self.linearSpringConst= 30

  def startManipulation ( self ):

    super ( SpringBallManipulator, self ).startManipulation ()

    # Create a new body which will be connected to the manipulated body by a BallJoint
    # The body will then be influenced by the spring force
    self.manipulationBody= createX3DNodeFromString ( "<RigidBody mass='0.001' />" )[0]
    self.rigidBodyCollection.bodies.push_back ( self.manipulationBody )

    # Create a temporary BallJoint to actually move the manipulated body
    self.manipulationJoint= createX3DNodeFromString ( "<BallJoint />" )[0]
    self.manipulationJoint.body1.setValue ( self.body )
    self.manipulationJoint.body2.setValue ( self.manipulationBody )
    self.rigidBodyCollection.joints.push_back ( self.manipulationJoint )

    # The anchor point of the BallJoint is the haptic device position in local coordinates of the rootTrans (the transform containing the rigid body's shapes)
    self.manipulationJoint.anchorPoint.setValue ( self.inputPosition.getValue() )

    # This is an offset vector in the local coords of the body being manipulated. It runs from the object's origin the the point of manipulation on the object's surface
    self.springPosition.offset= Matrix4f ( self.body.orientation.getValue() ).inverse()*(self.inputPosition.getValue()-self.body.position.getValue())

    # Provide all the information required to update the position of the spring as the body moves
    self.body.position.route ( self.springPosition)
    self.body.orientation.route ( self.springPosition )

    self.springForce= SpringManipulator.SpringForce ( self.linearSpringConst )

    self.springPosition.route ( self.springForce )
    self.inputPosition.route ( self.springForce )

    # Route the final force to the body doing the manipulation
    self.manipulationForce= ManipulatorCollection.SumForces ()
    self.springForce.route ( self.manipulationForce )
    self.manipulationForce.route( self.manipulationBody.forces )

  def stopManipulation ( self ):

    self.body.position.unroute ( self.springPosition)
    self.body.orientation.unroute ( self.springPosition )

    super ( SpringBallManipulator, self ).stopManipulation ()

    # Remove the temporary body and BallJoint
    self.rigidBodyCollection.joints.erase ( self.manipulationJoint )
    self.rigidBodyCollection.bodies.erase ( self.manipulationBody )
    self.manipulationBody= None
    self.manipulationJoint= None

    self.springPosition.unroute ( self.springForce )
    self.inputPosition.unroute ( self.springForce )
    self.manipulationForce= None

class ManipulateCtrl ( AutoUpdate ( SFBool ) ):
  """ The ManipulateCtrl defines the template algorithm which manages manipulation control.
      I.e. Using a selector, the body is selected, then using a manipulator, the manipulation is started,
      then stopped. Manipulation starts when the SFBool route in becomes True, and stops when it
      becomes False."""

  def __init__( self, selector, manipulator, rigidBodyCollection ):
    """ Create a new ManipulateCtrl, specifying the selector and manipulator to use in
        the template algorithm."""
    
    AutoUpdate ( SFBool ).__init__ ( self )
    self.selector= selector
    self.manipulator= manipulator
    self.rigidBodyCollection= rigidBodyCollection

  def update ( self, event ):
    """ Start or stop manipulation based on SFBool value."""
    
    if ( event.getValue() ):
      body= self.selector.select ( self.rigidBodyCollection.bodies.getValue() )
      if body is not None:
        self.manipulator.start(body)

    else:
      
      self.manipulator.stop()

    return False

class KeyState ( AutoUpdate ( TypedField ( SFBool, (SFString, SFString) ) ) ):
  """Using input from the keyPress and keyRelease fields of a key sensor,
     create a field representing the current state (up/down) of a particular button.
     May be used as input to a ManipulateCtrl to enable use of keyboard for object manipulation"""

  class Key ( AutoUpdate ( SFString ) ):
    """ Field used to either set or unset the key down state for the specified key from
        a keyPress or keyRelease field routed to it."""
    def __init__ ( self, parent, down, key ):
      AutoUpdate ( SFString ).__init__ ( self )
      self.parent= parent
      self.down= down
      self.key= key

    def update ( self, event ):
      """ Set or unset the parent's key state based on the current state and the route in."""
      if ((self.down and not self.parent.value) or (not self.down and self.parent.value)) and event.getValue().lower()==self.key:
        self.parent.value= self.down
      return ""
  
  def __init__ ( self, key, keyDownField, keyUpField ):
    """ Create a new KeyState for the specified key, providing the keyDown and keyUp SFString fields, which should
        be routed from a KeySensor's keyPress and keyRelease fields."""
    AutoUpdate ( TypedField ( SFBool, (SFString, SFString) ) ).__init__ ( self )
    self.value= False
    self.keyDown= KeyState.Key ( self, True, key )
    self.keyUp= KeyState.Key ( self, False, key )
    keyDownField.route ( self.keyDown )
    keyUpField.route ( self.keyUp )
    keyDownField.route ( self )
    keyUpField.route ( self )

  def cleanUp( self ):
    self.keyDown = None
    self.keyUp = None
      
  def update ( self, event ):
    """ Returns the current state of the key."""
    return self.value

class MouseManipulateCtrl:
  """ This class takes a TouchSensor and PlaneSensor and uses these to control the specified
      RigidBody with the specified Manipulator."""

  class UpdateClicked ( AutoUpdate ( SFBool ) ):
    """ Field used to save the intersect point upon first click (from TouchSensor)."""
    def __init__ ( self, touchSensor ):
      AutoUpdate ( SFBool ).__init__ ( self )
      self.touchSensor= touchSensor
    
    def update ( self, event ):
      """ Save the position of the mouse on the surface."""
      self.mouseHitPoint= self.touchSensor.hitPoint_changed.getValue()
      return event.getValue()    

  class UpdateDragged ( AutoUpdate ( SFVec3f ) ):
    """ Field calculate the manipulation position from the initial intersection (TouchSensor) and the drag offset (PlaneSensor)."""
    def __init__ ( self, ballClicked ):
      AutoUpdate ( SFVec3f ).__init__ ( self )
      self.ballClicked= ballClicked
    
    def update ( self, event ):
      return event.getValue() + self.ballClicked.mouseHitPoint

  class UpdateDragActive ( AutoUpdate ( SFBool ) ):
    """ Field used to start and stop the manipulation. Routed from the PlaneSensor's 'isActive' field."""
    def __init__ ( self, mouseManipulator, rigidBody ):
      AutoUpdate ( SFBool ).__init__ ( self )
      self.rigidBody= rigidBody
      self.mouseManipulator= mouseManipulator
      
    def update ( self, event ):
      if event.getValue():
        self.mouseManipulator.start ( self.rigidBody )
      else:
        self.mouseManipulator.stop()

      return True
  
  def __init__ ( self, manipulator, touchSensor, planeSensor, rigidBody ):
    """ Create a new MouseManipulateCtrl."""

    self.updateClicked= MouseManipulateCtrl.UpdateClicked(touchSensor)
    touchSensor.isActive.route ( self.updateClicked )

    self.updateDragged= MouseManipulateCtrl.UpdateDragged(self.updateClicked)
    planeSensor.translation_changed.route ( self.updateDragged )
    self.updateDragged.route ( manipulator.inputPosition )

    self.updateDragActive= MouseManipulateCtrl.UpdateDragActive(manipulator, rigidBody )
    planeSensor.isActive.route ( self.updateDragActive )
