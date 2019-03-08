from H3DInterface import *

class TouchSelector:
  """ A class which provides a method of selecting a body from the scene. Allows
      for some abstraction when implementing manipulation methods."""
  
  def select ( self, bodies ):
    """ Returns the selected object from the specified list. May be None if no object
        was selected this time."""
    for b in bodies:
      for cs in b.geometry.getValue():
        touched= cs.shape.getValue().geometry.getValue().isTouched.getValue()
        if len(touched) > 0 and touched[0]:
          return b
    return None

class SpringManipulator:
  """ The SpringManipulator provides a way of interacting with a body using a
      virtual spring between the body and the haptic device. The body is moved using
      a temporary ball joint attached at the point where the body is 'picked up'."""

  class DampForce ( SFVec3f ):
    """ A field used to calculate a damping force in the opposite direction to the input field."""
    def __init__ ( self, dampingFactor ):
      SFVec3f.__init__ ( self )
      self.dampingFactor= dampingFactor
      
    def update ( self, event ):
      return -event.getValue()*self.dampingFactor

  class BodyForce ( TypedField ( MFVec3f, None, SFVec3f ) ):
    """ A field used to sum all the forces routed to it. Creates a MFVec3f containing the result as the only element."""
    def update ( self, event ):
      forces= self.getRoutesIn ()
      total= Vec3f()
      for f in forces:
        total+= f.getValue()
      return [total]

  class DrawLine ( TypedField ( MFVec3f, ( SFVec3f, SFVec3f ) ) ):
    """ Field used to route to the line geometry. Creates a MFVec3f containing the start and end points routed in."""
    def update ( self, event ):
      start, end= self.getRoutesIn()
      return [start.getValue(), end.getValue()]

  class SpringPosition ( TypedField ( SFVec3f, (SFVec3f, SFRotation) ) ):
    """ Calculates the anchor position of the spring effect from the body's position, orientation
        and the initial offset for the current manipulation"""
    def update ( self, event ):
      position, orientation= self.getRoutesIn()
      return Matrix4f(orientation.getValue())*self.offset+position.getValue()

  class SpringForce ( TypedField ( SFVec3f, ( SFVec3f, SFVec3f ) ) ):
    """ Field used to calculate a spring force based on the anchor point and end point routed in."""
    def update ( self, event ):
      anchor, point= self.getRoutesIn()
      return -(anchor.getValue()-point.getValue())*self.k

  def __init__ ( self, rootTrans, rigidBodyCollection ):
    """ Create a new SpringManipulator for the specified hapticDevice. The rootTrans parameter is
        the transformation which contains the shapes representing the rigid bodies, and it may specify
        a translation and/or rotation."""

    self.manipulationPoint= SFVec3f()
    self.springForce= SpringManipulator.SpringForce()
    self.springForce.k= 150   # Spring constant used for manipulation
    
    self.rootTrans= rootTrans
    self.rigidBodyCollection= rigidBodyCollection
    self.body= None
    self.dampForce= SpringManipulator.DampForce(15)
    self.dampTorque= SpringManipulator.DampForce(0.01)
    self.bodyForce= SpringManipulator.BodyForce()
    self.bodyTorque= SpringManipulator.BodyForce()
    self.springPosition= SpringManipulator.SpringPosition()
    self.drawLine= SpringManipulator.DrawLine()
    
  def start ( self, body ):
    """ Start the manipulation."""
    
    if self.body is None:
      
      self.body= body

      # Remove the shape's surface for the duration of the
      # manipulation to avoid the device bouncing off it
      self.surfaceNodes= []
      for cs in self.body.geometry.getValue():
        surface= cs.shape.getValue().appearance.getValue().surface
        surfaceNode= surface.getValue()
        surface.setValue(None)
        self.surfaceNodes.append ( surfaceNode )

      # Create a new body which will be connected to the manipulated body by a BallJoint
      # The body will then be influenced by the spring force

      cs= createX3DNodeFromString ( """
          <CollidableShape containerField='geometry'>
                <Shape containerField='shape'>
                  <Appearance />
                  <!--<Sphere radius='0.0001' />-->
                </Shape>
              </CollidableShape>""" )[0]
      #self.rigidBodyCollection.collider.getValue().collidables.push_back ( cs )

      self.manipulationBody= createX3DNodeFromString ( """
            <RigidBody >
              <Box containerField='massDensityModel' />
            </RigidBody>""" )[0]

      #self.manipulationBody.geometry.push_back ( cs )
      
      self.rigidBodyCollection.bodies.push_back ( self.manipulationBody )

      # Create a temporary BallJoin to actually move the manipulated body
      self.manipulationJoint= createNode( "BallJoint" )
      self.manipulationJoint.body1.setValue ( self.body )
      self.manipulationJoint.body2.setValue ( self.manipulationBody ) # Bullet crash: If this line is removed- no crash
      self.rigidBodyCollection.joints.push_back ( self.manipulationJoint )

      # The anchor point of the BallJoint is the haptic device position in local coordinates of the rootTrans (the transform containing the rigid body's shapes)
      #self.manipulationJoint.anchorPoint.setValue ( self.manipulationPoint.getValue() )

      # This is an offset vector in the local coords of the body being manipulated. It runs from the object's origin the the point of manipulation on the object's surface
      self.springPosition.offset= Matrix4f ( self.body.orientation.getValue() ).inverse()*(self.manipulationPoint.getValue()-self.body.position.getValue())

      # Provide all the information required to update the position of the spring as the body moves
      self.springPosition.parentTransform= self.rootTrans.accumulatedForward.getValue()
      self.body.position.route ( self.springPosition)
      self.body.orientation.route ( self.springPosition )

      self.springPosition.route ( self.springForce )
      self.manipulationPoint.route ( self.springForce )
      self.springForce.route ( self.bodyForce )

      # Apply linear damping
      self.body.linearVelocity.route ( self.dampForce )
      self.dampForce.route ( self.bodyForce )

      # Apply angular damping
      self.body.angularVelocity.route ( self.dampTorque )
      self.dampTorque.route ( self.bodyTorque )
      self.bodyTorque.route ( self.body.torques )

      # Route the final force to the body doing the manipulation
      self.bodyForce.route( self.manipulationBody.forces )

      # Create some shapes to represent the manipulation: A ball to represent the anchor point,
      # and a line representing the virtual spring
      self.jointSphere= createX3DNodeFromString ( "<Transform>\
                                                   <Shape>\
                                                   <Appearance>\
                                                     <Material diffuseColor='1 0 0' />\
                                                   </Appearance>\
                                                   <Sphere radius='0.004' />\
                                                   </Shape>\
                                                   </Transform>" )[0]
      self.jointLine, dn= createX3DNodeFromString ( "<Transform>\
                                                   <Shape>\
                                                   <Appearance>\
                                                     <Material emissiveColor='1 0 0' />\
                                                   </Appearance>\
                                                   <LineSet vertexCount='2'>\
                                                     <Coordinate DEF='COORD' />\
                                                   </LineSet>\
                                                   </Shape>\
                                                   </Transform>" )

      # Set up routes to update the graphic representation of the manipulation
      self.lineCoord= dn['COORD']
      self.springPosition.route ( self.drawLine )
      self.manipulationPoint.route ( self.drawLine )
      self.drawLine.route ( self.lineCoord.point )
      self.rootTrans.children.push_back ( self.jointLine )
      self.springPosition.route ( self.jointSphere.translation )
      self.rootTrans.children.push_back ( self.jointSphere )

  def stop ( self ):
    """ Stop the manipulation."""
    
    if self.body is not None:

      # Unroute everything
      self.body.position.unroute ( self.springPosition)
      self.body.orientation.unroute ( self.springPosition )
      self.springForce.unroute ( self.bodyForce )
      self.body.linearVelocity.unroute ( self.dampForce )
      self.dampForce.unroute ( self.bodyForce )
      self.bodyForce.unroute( self.manipulationBody.forces )
      self.body.angularVelocity.unroute ( self.dampTorque )
      self.dampTorque.unroute ( self.bodyTorque )
      self.bodyTorque.unroute ( self.body.torques )

      # Remove all forces on the body
      self.body.forces.clear()
      self.body.torques.clear()

      # Unroute and erase the graphical representation of manipulation
      self.springPosition.unroute ( self.jointSphere.translation )
      self.rootTrans.children.erase ( self.jointSphere )
      self.jointSphere= None
      self.springPosition.unroute ( self.drawLine )
      self.manipulationPoint.unroute ( self.drawLine )
      self.drawLine.unroute ( self.lineCoord.point )
      self.rootTrans.children.erase ( self.jointLine )
      self.jointLine= None

      # Remove the temporary body and BallJoint
      self.rigidBodyCollection.joints.erase ( self.manipulationJoint )
      self.rigidBodyCollection.bodies.erase ( self.manipulationBody )
      self.manipulationBody= None
      self.manipulationJoint= None

      # Restore the shape's surface now that manipulation has ended
      self.surfaceNodes.reverse()
      for cs in self.body.geometry.getValue():
        cs.shape.getValue().appearance.getValue().surface.setValue ( self.surfaceNodes.pop() )

      self.body= None
      self.springEffect= None

class HapticSpringManipulator ( SpringManipulator ):
  """ A SpringManipulator which uses the haptic device as the manipulation input."""

  class TransformPoint ( TypedField ( SFVec3f, ( SFVec3f, SFMatrix4f ) ) ):
    """ Field used to transform the device position to local coordinates of the rootTrans."""

    def update ( self, event ):
      point, transform= self.getRoutesIn()
      return transform.getValue()*point.getValue()

  def __init__ ( self, hapticDevice, rootTrans, rigidBodyCollection ):
    """ Create a new HapticSpringManipulator for the specified haptic device."""

    SpringManipulator.__init__ ( self, rootTrans, rigidBodyCollection )

    # Use the haptic device position, transformed to local coordinates of the
    # rootTrans as the SpringManipulator's 'manipulationPoint'
    self.transformDevicePos= HapticSpringManipulator.TransformPoint()
    hapticDevice.weightedProxyPosition.route ( self.transformDevicePos )
    self.rootTrans.accumulatedInverse.route ( self.transformDevicePos )
    self.transformDevicePos.route ( self.manipulationPoint )
    self.hapticDevice= hapticDevice

  def start ( self, body ):
    """ Override start() to add haptic spring effect."""

    if self.body is None:

      # Get the index of the haptic device to add the force effect for
      index= 0
      di = getActiveDeviceInfo()
      if di:
        devices= di.device.getValue()
        index= devices.index ( self.hapticDevice )

      # Create a spring effect to provid force feedback and to calculate to force to apply to the body
      self.springEffect= createX3DNodeFromString ( "<SpringEffect deviceIndex='" + str(index) +
                                                   "' startDistance='1' escapeDistance='1' springConstant='50' "+
                                                   "positionInterpolation='0.03' />" )[0]
      self.springPosition.route ( self.springEffect.position )
      self.rootTrans.children.push_back ( self.springEffect )

    # Call super class start()
    SpringManipulator.start ( self, body )

  def stop ( self ):
    """ Override stop() to remove haptic spring effect."""

    if self.body is not None:
      self.springPosition.unroute ( self.springEffect.position )
      self.rootTrans.children.erase ( self.springEffect )

    # Call super class stop()
    SpringManipulator.stop ( self )

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

no_circular_reference_list = []

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
    global no_circular_reference_list
    self.value= False
    no_circular_reference_list.append( KeyState.Key ( self, True, key ) )
    no_circular_reference_list.append( KeyState.Key ( self, False, key ) )
    keyDownField.route ( no_circular_reference_list[ len(no_circular_reference_list) - 2] )
    keyUpField.route ( no_circular_reference_list[ len(no_circular_reference_list) - 2] )
    keyDownField.route ( self )
    keyUpField.route ( self )
      
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

  class UpdateDragged ( AutoUpdate ( TypedField ( SFVec3f, (SFVec3f,SFBool) ) ) ):
    """ Field calculate the manipulation position from the initial intersection (TouchSensor) and the drag offset (PlaneSensor)."""
    def __init__ ( self, ballClicked ):
      AutoUpdate ( TypedField ( SFVec3f, (SFVec3f,SFBool) ) ).__init__ ( self )
      self.ballClicked= ballClicked
    
    def update ( self, event ):
      if isinstance(event.getValue(),int) and event.getValue():
        # This case will execute when the drag first starts, this is required because, at this point
        # the planeSensor.translation_changed field (route 0) will still have a non-zero value from the
        # last drag- resulting in the incorrect offset.
        return self.ballClicked.mouseHitPoint
      else:
        return self.getRoutesIn()[0].getValue() + self.ballClicked.mouseHitPoint

  class UpdateDragActive ( PeriodicUpdate ( SFBool ) ):
    """ Field used to start and stop the manipulation. Routed from the PlaneSensor's 'isActive' field."""
    def __init__ ( self, mouseManipulator, rigidBody ):
      PeriodicUpdate ( SFBool ).__init__ ( self )
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
    touchSensor.isActive.route ( self.updateDragged )
    self.updateDragged.route ( manipulator.manipulationPoint )

    self.updateDragActive= MouseManipulateCtrl.UpdateDragActive(manipulator, rigidBody )
    touchSensor.isActive.route ( self.updateDragActive )
