from H3DInterface import *
import Manipulation

temp= None

# Get references to nodes created in the X3D file
rootTrans, group, rigidBodyCollection= references.getValue()

class Ball:
  """ Represents a single ball and string in the Newton's Cradel"""

  class DrawLine ( TypedField ( MFVec3f, SFVec3f ) ):
    """ Generates a list of two coordinates used to draw a line between the ball and the anchor point"""
    def update ( self, event ):
      return [self.anchorPoint, event.getValue()]
  
  def __init__ ( self, ballRadius, ballMass, jointLength, anchorPoint, damping, rigidBodyCollection ):
    """ Create a new ball with the specified parameters and add it to rigidBodyCollection"""

    # Load the ball model and customise parameters
    self.node, dn= createX3DNodeFromString ( """<Transform>
                                                  <TouchSensor DEF='TouchSensor' enabled='true' />
                                                  <PlaneSensor DEF='PlaneSensor'
                                                              enabled='true' autoOffset='false' />
                                                  <Shape>
                                                    <Appearance>
                                                      <Material />
                                                    </Appearance>
                                                    <LineSet vertexCount='2'>
                                                      <Coordinate DEF='C' point='0 0 0, 1 0 0' />
                                                    </LineSet>
                                                  </Shape>
                                                </Transform>""" )

    # Now add the ball
    ball, dn1= createX3DFromString ( """  <Transform DEF='T'>
                                            <Shape DEF='S'>
                                              <Appearance>
                                                <Material diffuseColor='0.7 0.7 0.7' specularColor='1 1 1' shininess='1' />
                                                  <FrictionalSurface staticFriction='0.5' dynamicFriction='1' />
                                                </Appearance>
                                                <Sphere DEF='Ball_G' radius='""" + str(ballRadius) + """' />
                                            </Shape>
                                          </Transform>""" )
    self.node.children.push_back ( ball )

    # Create the CollidableShape representing the ball and add it to the collection
    self.collidableShape= createX3DNodeFromString ( "<CollidableShape />" )[0]
    self.collidableShape.shape.setValue ( dn1['S'] )
    rigidBodyCollection.collider.getValue().collidables.push_back ( self.collidableShape )

    # Create the RigidBody representing the ball and add it to the collection
    self.rigidBody= createX3DNodeFromString ( "<RigidBody mass='"+str(ballMass)+"'/>" )[0]
    self.rigidBody.position.setValue ( anchorPoint - Vec3f (0, jointLength, 0) )
    self.rigidBody.geometry.push_back ( self.collidableShape )
    self.rigidBody.massDensityModel.setValue ( dn1['Ball_G'] )
    self.rigidBody.linearDampingFactor.setValue ( damping )
    self.rigidBody.autoDamp.setValue ( True )
    rigidBodyCollection.bodies.push_back ( self.rigidBody )

    # Create a single axis hinge joint to connect the ball to the anchor point on the frame
    self.joint= createX3DNodeFromString ( "<SingleAxisHingeJoint axis='0 0 1' />" )[0]
    self.joint.anchorPoint.setValue ( anchorPoint )
    self.joint.body1.setValue ( self.rigidBody )
    rigidBodyCollection.joints.push_back ( self.joint )

    # Set up routes to update the rendered position of the ball from the rigid body parameters
    self.rigidBody.position.route ( dn1['T'].translation )
    self.rigidBody.orientation.route ( dn1['T'].rotation )

    # Set up routes to update the line drawn to represent the string the ball hangs from
    self.drawLine= Ball.DrawLine()
    self.drawLine.anchorPoint= anchorPoint
    self.rigidBody.position.route ( self.drawLine )
    self.drawLine.route ( dn['C'].point )

    # Create a SpringManipulator which will be controlled by the mouse
    self.mouseManipulation= Manipulation.MouseManipulateCtrl ( Manipulation.SpringManipulator ( rootTrans, rigidBodyCollection ),
                                                               dn['TouchSensor'], dn['PlaneSensor'], self.rigidBody )

    # Save a reference to the rigid body collection so that we can remove the ball later
    self.rigidBodyCollection= rigidBodyCollection

  def __del__ ( self ):
    """ Delete the ball and remove all of its nodes from the simulation"""
    self.rigidBodyCollection.joints.erase ( self.joint )
    self.rigidBodyCollection.bodies.erase ( self.rigidBody )
    self.rigidBodyCollection.collider.getValue().collidables.erase ( self.collidableShape )

class NewtonsCradle:
  """ Represents the Newton's Cradle. I.e. The frame structure and the balls attached to it. """
  
  def __init__ ( self, ballCount, ballRadius, ballMass, jointLength, damping, rigidBodyCollection ):
    """ Create a new Newton's Cradle with the specified parameters and add it to rigidBodyCollection"""

    # First, create the frame structure based on the parameters given
    barLength= 2*ballRadius*(ballCount-1) + jointLength
    crossBarLength= ballRadius*4
    legHeight= jointLength + ballRadius + 0.02
    startPos= -ballRadius*(ballCount-1)
    # Leaving a small gap between the balls helps transmit the collision forces along the row better
    stepSize= 2*ballRadius + 0.0001
    self.balls= []

    # Create the Newton's Cradle frame model from file, then customise parameters, such as leg length etc.
    self.node= createX3DNodeFromString ( """<Group>
                                              <Transform translation='0 """+str(legHeight)+""" 0'>
                                                <Transform rotation='0 0 1 1.57'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material DEF='METAL' diffuseColor='0.7 0.7 0.7' specularColor='1 1 1' shininess='1' />
                                                    </Appearance>
                                                    <Cylinder radius='0.003' height='"""+str(barLength)+"""' />
                                                  </Shape>
                                                </Transform>
                                                <Transform rotation='1 0 0 1.57' translation='"""+str(barLength/2)+""" 0 0'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Cylinder DEF='CROSS_BAR' radius='0.003' height='"""+str(crossBarLength)+"""' />
                                                  </Shape>
                                                </Transform>
                                                <Transform rotation='1 0 0 1.57' translation='"""+str(-barLength/2)+""" 0 0'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Cylinder USE='CROSS_BAR' />
                                                  </Shape>
                                                </Transform>
                                                <Transform translation='"""+str(barLength/2)+" 0 "+str(-crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Sphere DEF='LEG_TOP' radius='0.003' />
                                                  </Shape>
                                                </Transform>
                                                <Transform translation='"""+str(barLength/2)+" 0 "+str(crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Sphere USE='LEG_TOP' />
                                                  </Shape>
                                                </Transform>
                                                <Transform translation='"""+str(-barLength/2)+" 0 "+str(-crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Sphere USE='LEG_TOP' />
                                                  </Shape>
                                                </Transform>
                                                <Transform translation='"""+str(-barLength/2)+" 0 "+str(crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Sphere USE='LEG_TOP' />
                                                  </Shape>
                                                </Transform>
                                              </Transform>
                                              <Transform translation='0 """+str(legHeight/2)+""" 0'>
                                                <Transform translation='"""+str(-barLength/2)+" 0 "+str(crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Cylinder DEF='LEG_BAR' radius='0.003' height='"""+str(legHeight)+"""' />
                                                  </Shape>
                                                </Transform>
                                                <Transform translation='"""+str(-barLength/2)+" 0 "+str(-crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Cylinder USE='LEG_BAR' />
                                                  </Shape>
                                                </Transform>
                                                <Transform translation='"""+str(barLength/2)+" 0 "+str(crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Cylinder DEF='LEG_BAR' radius='0.003' height='"""+str(legHeight)+"""' />
                                                  </Shape>
                                                </Transform>
                                                <Transform translation='"""+str(barLength/2)+" 0 "+str(-crossBarLength/2)+"""'>
                                                   <Shape>
                                                    <Appearance>
                                                      <Material USE='METAL' />
                                                    </Appearance>
                                                    <Cylinder USE='LEG_BAR' />
                                                  </Shape>
                                                </Transform>
                                              </Transform>
                                            </Group>""" )[0]

    # Now create the balls
    for i in range(ballCount):
      ball= Ball ( ballRadius, ballMass, jointLength, Vec3f( startPos+stepSize*i, legHeight, 0), damping, rigidBodyCollection )
      self.balls.append ( ball )
      self.node.children.push_back ( ball.node )

# These fields are used to specify the creation parameters for the Newton's Cradle object
bounce= SFFloat()
damping= SFFloat()
mass= SFFloat()
ballCount= SFFloat ()
ballRadius= SFFloat ()
stringLength= SFFloat ()

class CreateCradle ( AutoUpdate ( SFBool ) ):
  """ This field type is used to create the Newton's Cradle. When a value of True is recieved
      a new Cradle is created using the current creation field parameters. The old Cradle is removed
      if it exists.
  """
  def __init__ ( self ):
    AutoUpdate ( SFBool ).__init__ ( self )
    self.newtonsCradle= None
  
  def update ( self, event ):
    if event.getValue():
      self.create ()
      
    return True

  def create ( self ):
    """ Remove the old Cradle (if it exists) and add a new one, created with the current parameters"""
    
    if self.newtonsCradle is not None:
      group.children.erase ( self.newtonsCradle.node )

    self.newtonsCradle= NewtonsCradle ( int(ballCount.getValue()),
                                            ballRadius.getValue(),
                                            mass.getValue(),
                                            stringLength.getValue(),
                                            damping.getValue(),
                                            rigidBodyCollection )
    group.children.push_back ( self.newtonsCradle.node )

# Field used to create the Newton's Cradle (the 'Update' button is routed here)
createCradle= CreateCradle()

# These fields are used to control the grabbing action for the keyboard key
# (The KeySensor's keyPress and keyRelease fields are routed here)
keyDown= SFString()
keyUp= SFString()

# This class turns the keyDown and keyUp fields into a boolean state of the specified
# key, in this case "g". If "g" is depressed, the field will become True and vice versa
grabKey= Manipulation.KeyState ("g", keyDown, keyUp)
ctrlGrabKey= None
pickUpCtrls= []

# Attach handlers used to 'grab' objects in the simulation
di = getActiveDeviceInfo()
if( di ):
  devices = di.device.getValue()
  
  # Create handler for keyboard button 'g'. This will control the grab action for device index 0
  if len(devices)> 0:
    ctrlGrabKey= Manipulation.ManipulateCtrl ( Manipulation.TouchSelector(),
                                               Manipulation.HapticSpringManipulator(devices[0],rootTrans,rigidBodyCollection),
                                               rigidBodyCollection )
    grabKey.route ( ctrlGrabKey )

  # Create and attach button handlers to each haptic device
  for d in devices:
    ctrl= Manipulation.ManipulateCtrl ( Manipulation.TouchSelector(),
                                        Manipulation.HapticSpringManipulator(d,rootTrans,rigidBodyCollection),
                                        rigidBodyCollection )
    d.mainButton.route ( ctrl )
    pickUpCtrls.append ( ctrl )
    
    d.hapticsRenderer.setValue ( createNode ( 'RuspiniRenderer' ) )

def traverseSG():
  """Executed every scenegraph loop"""

  # On first loop create default model
  # (on first loop UI routes are in place)
  if createCradle.newtonsCradle is None:
    createCradle.create()
