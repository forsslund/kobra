import random
import math
import time
import threading
import Manipulation
import DeviceCalibration

#import the H3D fields and types
from H3DInterface import *

MAX_BRICKS = 100
INI_BRICKS = 35

di = getActiveDeviceInfo()
if( di  ):
  devices = di.device.getValue()
  for d in devices:
    d.hapticsRenderer.setValue( createX3DNodeFromString( "<RuspiniRenderer alwaysFollowSurface=\"true\" />" )[0] )

#Counter for naming nodes, for instance CS1, T1, RB1...
counter = 0
radius = 0.12
twopi = 6.28318531
pi = 3.14159265
bricks = 5

# get the reference to the group to put the spheres in
root, rootTrans, shadowedNodes, shadowedNodesToggle, rigidBodyCollection, timeSensor, lampBody, lampGeometry = references.getValue()
collider= rigidBodyCollection.collider.getValue()

# These fields are used to control the grabbing action for the keyboard key
# (The KeySensor's keyPress and keyRelease fields are routed here)
keyDown= SFString()
keyUp= SFString()

# Generate texture nodes to assign to blocks
# Nodes are generated at load time so that the relative paths to images work
brickTextures= []
brickTextures.append ( createX3DNodeFromString ( "<ImageTexture url='images/wood1.jpg' />" )[0] )
brickTextures.append ( createX3DNodeFromString ( "<ImageTexture url='images/wood2.jpg' />" )[0] )
brickTextures.append ( createX3DNodeFromString ( "<ImageTexture url='images/wood3.jpg' />" )[0] )

# H3DAPI logo texture node
h3dApiLogoTexture= createX3DNodeFromString ( "<ImageTexture url='images/h3dapi_logo_block.jpg' />" )[0]

# The number of special H3D logo bricks
h3dApiLogoBlockCount= 3

# List of def nodes for each created blocks
blockDefs= []

stylusMaterial= createX3DNodeFromString ( "<Material />" )[0]

block_url = resolveURLAsFile( 'x3d/block.x3d' )

class AddBox( AutoUpdate( SFBool ) ):
  def update( self, event ):
    if( event.getValue() ):
      global counter, blockDefs

      t, dn= createX3DNodeFromURL( block_url )
      
      blockDefs.append ( dn )
      
      # Assign a texture to the block at random
      brickTexture= brickTextures[random.randint(1,len(brickTextures))-1]
      dn['APPEARANCE'].texture.setValue ( brickTexture )
        
      offset = ((counter/bricks)%2)*pi/bricks #Every second row have their angles offset by one half brick
      angle = (counter%bricks)*twopi/bricks+offset #Brick angle
      height = 0.025+0.05*(counter/bricks)-0.1
      pos = Vec3f( radius*math.cos(angle), height, radius*math.sin(angle) )
      rot = Rotation( 0, 1, 0, -angle )
      size = Vec3f( 0.05, 0.05, 0.1 )
      
      dn["BOX"].size.setValue( size )           # Size for physics and haptics
      dn["GRAPHIC_T"].scale.setValue ( size )   # Scaling for graphics
      
      shadowedNodes.children.push_back( t )
      
      #Create the CollidableShape node
      cs, dn2 = createX3DNodeFromString( """\
        <CollidableShape DEF='CS""" + str(counter) + """' containerField='collidables'/>""")

      trans = Vec3f( 0.0, 0.0, 0.0 )        
      cs.translation.setValue( trans )
      dn2["CS" + str(counter)].shape.setValue( dn["SHAPE"] )   #Sets the SHAPEx node created above as the shape field of CSx (x = counter value)
      collider.collidables.push_back( cs ) #Push the CollidableShape node cs into the CollisionCollection group CC in the x3d-file
      
      #Create the RigidBody node
      rb, dn3 = createX3DNodeFromString( """<RigidBody DEF='RB""" + str(counter) + """' mass='0.3'/>""")
      rb.position.setValue( pos )
      rb.orientation.setValue( rot )
      rb.massDensityModel.setValue( dn["BOX"] )
      rb.autoDamp.setValue( True )
      rb.linearDampingFactor.setValue( 0 )
      rb.angularDampingFactor.setValue( 0.01 )
      rb.autoDisable.setValue( True )
      rb.disableLinearSpeed.setValue( 0.08 )
      if rigidBodyCollection.physicsEngine.getValue() == "ODE":
        rb.disableAngularSpeed.setValue( 0.15 )
      else:
        rb.disableAngularSpeed.setValue( 0.08 )
      
      dn3["RB"+str(counter)].geometry.push_back( dn2["CS"+str(counter)] ) #Pushes the CS node into the bodies mfnode of the RB
      rigidBodyCollection.bodies.push_back( rb ) #Pushes the rb node into the RigidBodyCollection node RBC of the x3d-file
      
      # Set up the routes
      dn3["RB"+str(counter)].position.route ( dn["T"].translation )  #CSx translation to Tx translation
      dn3["RB"+str(counter)].orientation.route ( dn["T"].rotation )        #CSx rotation to Tx rotation


      counter = counter + 1
      
    if counter == MAX_BRICKS:
      return True
    return False
 
# create an instance of the AddSphere class
add_box = AddBox()

# key handler to add new box
class KeyAddBox ( AutoUpdate ( TypedField ( SFBool, SFString ) ) ):
  def update ( self, event ):
    return event.getValue() == ' '

kAddBox= KeyAddBox()
kAddBox.routeNoEvent ( add_box )

# Force to move lamp using key board
class KeyMoveLamp ( AutoUpdate ( TypedField ( SFVec3f, SFBool ) ) ):
  def update ( self, event ):
    if event.getValue():
      return Vec3f ( -0.2, 0, 0.4 )
    return Vec3f ()

# Force to damp lamp movement using key board
class KeyDampLamp ( AutoUpdate ( TypedField ( SFVec3f, ( SFBool, SFVec3f ) ) ) ):
  def update ( self, event ):
    if len(self.getRoutesIn()) > 1:
      dampingOn, velocity= self.getRoutesIn()
      if dampingOn.getValue():
        return -self.dHigh * velocity.getValue()
      return -self.dLow * velocity.getValue()
    return Vec3f()

kMoveLamp= Manipulation.KeyState("m", keyDown, keyUp)
kMoveLampForce= KeyMoveLamp()
kMoveLamp.route ( kMoveLampForce )

kDampLamp= Manipulation.KeyState("x", keyDown, keyUp)
kDampLampForce= KeyDampLamp()
kDampLampForce.dHigh= 5  # Damping factor
kDampLampForce.dLow= 0.03  # Damping factor
kDampLamp.route ( kDampLampForce )

# Calculate the total force on a geometry due to haptic device
# Take a MFVec3f field and sum the forces to one SFVec3f, then negate
class GeometryTouchForce ( TypedField ( SFVec3f, MFVec3f ) ):
  def update ( self, event ):
    totalForce= Vec3f ()
    forces= event.getValue()
    for f in forces:
      totalForce-= f
    return totalForce

lampGeometryForces= GeometryTouchForce()

# Sum total force on rigid body
class SumForces ( TypedField ( MFVec3f, None, SFVec3f ) ):
  def update ( self, event ):
    totalForce= Vec3f ()
    for f in self.getRoutesIn():
      totalForce+= f.getValue()
    return [totalForce]

# Total forces on the lamp rigid body
lampForces= SumForces()
kMoveLampForce.route ( lampForces )
kDampLampForce.route ( lampForces )
lampGeometryForces.route ( lampForces )

# Additional function and fields for Tower and stuff
# NOTICES
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

# TOWER CREATION AND RESETS
last_wall_collidable = collider.collidables.back()
last_wall_rigidbody = rigidBodyCollection.bodies.back()
wall_collidable = collider.collidables.getValue()
wall_rigidbody = rigidBodyCollection.bodies.getValue()
auto_on = False

def customizeBricks ():
  """ Choose and customize specific bricks (e.g. add H3D logo) """
  global blockDefs, h3dApiLogoTexture
  
  blockPool= list(range( 0, len(blockDefs)))
  
  if len(blockDefs) < h3dApiLogoBlockCount:
    blockCount= len(blockDefs)
  else:
    blockCount= h3dApiLogoBlockCount
  
  for i in range ( 0, blockCount ):
    blockIndex= blockPool[random.randint(0,len(blockPool)-1)]
    blockPool.remove ( blockIndex )
    blockDef= blockDefs[blockIndex]
  
    # Use a multi-texture to add a layer containing the H3DAPI logo
    multiTexture= createX3DNodeFromString ( "<MultiTexture />" )[0]
    multiTexture.texture.setValue ( [brickTextures[2], h3dApiLogoTexture] )
    blockDef['APPEARANCE'].texture.setValue ( multiTexture )
    blockDef['MATERIAL'].diffuseColor.setValue ( RGB ( 1, 1, 0.3 ) )
  
def createTower( num_bricks ):

  # Add the bricks
  for n in range( num_bricks ):
    dupe.touch()
    
  # Choose and customize specific bricks (e.g. add H3D logo)
  customizeBricks()
  
    
def reset():
  global counter, nodes, auto_on
  
  auto_on = False
  timeSensor.enabled.setValue( False )
  
  
  for n in range( counter ):
    shadowedNodes.children.pop_back()
    if rigidBodyCollection.bodies.getValue() != wall_rigidbody:
      rigidBodyCollection.bodies.pop_back()
    if collider.collidables.getValue() != wall_collidable:
      collider.collidables.pop_back()
  counter = 0
  blockDefs= []
  createTower( INI_BRICKS )
  vp = getActiveViewpoint()
  vp.position.setValue( vp.position.getValue() )
  vp.orientation.setValue( vp.orientation.getValue() )

class MouseReset( PeriodicUpdate( SFBool ) ):
  def update( self, event ):
    if event.getValue():
      reset()
    return True

class KeyReset( PeriodicUpdate( SFString ) ):
  def update( self, event ):
    if event.getValue() == 'r':
      reset()
    return ''

dupe = SFBool()
dupe.setValue( True )
dupe.routeNoEvent( add_box )
mReset = MouseReset()
kReset = KeyReset()
  
# FORCE RESET AND WARNING 
class ForceReset( AutoUpdate( SFBool ) ):
  def update( self, event ):
    if event.getValue():
      reset()
    return False

forceReset = ForceReset()
add_box.route( forceReset )


# AUTO BOX FALL
class Auto( AutoUpdate( TypedField( SFBool, (SFString, SFTime) ) ) ):
  def update( self, event ):
    global nodes, auto_on
    ri = self.getRoutesIn()
    if  event == ri[0] and ri[0].getValue() == 'a':
      if auto_on:
        timeSensor.enabled.setValue( False )
        auto_on = False
      else:
        timeSensor.enabled.setValue( True )
        timeSensor.startTime.setValue( time.getValue() )
        auto_on = True
    if len(ri) > 1 and event == ri[1] and auto_on:
      return True
    return False
        
auto = Auto()
auto.routeNoEvent( add_box )

createTower( INI_BRICKS )

# Device calibration function for each device
deviceCalibration= []

# key handler toggle device calibration
class KeyToggleCalibration ( AutoUpdate ( SFString ) ):

  def enableDisableCalibration ( self ):
    """ Common functionality when enabling / disabling calibration """
    
    # Enabled if any kind of calibration is running
    enable= deviceCalibration[0].enabledXY or deviceCalibration[0].enabledRotation
    
    # Disable haptic rendering when calibrating
    root.hapticsOn.setValue ( not enable )
    shadowedNodesToggle.hapticsOn.setValue ( not enable )

    # Change color based on enable state
    if enable:
      stylusMaterial.diffuseColor.setValue ( RGB ( 1, 0, 0 ) )
    else:
      stylusMaterial.diffuseColor.setValue ( RGB ( 1, 1, 1 ) )
    
  def update ( self, event ):
  
    # Handle device calibration keys
    if len ( deviceCalibration ) > 0:
    
      # Toggle translational device calibration
      if event.getValue() == 'c':
        enable= not deviceCalibration[0].enabledXY
        for c in deviceCalibration:
          c.enabledXY= enable
          c.enabledZ= enable
        
        self.enableDisableCalibration()

      # Toggle rotational device calibration
      elif event.getValue()=='x':
        enable= not deviceCalibration[0].enabledRotation
        for c in deviceCalibration:
          c.enabledRotation= enable
          
        self.enableDisableCalibration()
        
    return ""

toggleCalibration= KeyToggleCalibration()

pickUpCtrls= []
manipulatorCollection= Manipulation.ManipulatorCollection()

# Attach handlers used to 'grab' objects in the simulation
di = getActiveDeviceInfo()
if( di ):
  devices = di.device.getValue()
  # Common initialization for each device
  
  deviceIndex= 0
  for d in devices:
    
    # Ball joint manipulation control
    ctrl= Manipulation.ManipulateCtrl ( Manipulation.TouchSelector( deviceIndex ),
                                        Manipulation.HapticSpringManipulatorControl ( 
                                        Manipulation.SpringBallManipulator(rigidBodyCollection,rootTrans),
                                        Manipulation.SpringBallManipulatorVisualisation(), rootTrans, manipulatorCollection, d ),
                                        rigidBodyCollection )
    d.mainButton.route ( ctrl )
    pickUpCtrls.append ( ctrl )

    # 6 DoF angular spring manipulation control
    ctrl= Manipulation.ManipulateCtrl ( Manipulation.TouchSelector( deviceIndex ),
                                        Manipulation.HapticAngularSpringManipulatorControl ( 
                                        Manipulation.SpringManipulator(), 
                                        None,
                                        rootTrans,
                                        manipulatorCollection,
                                        d, rigidBodyCollection,
                                        # Some options for the 6DoF manipulation
                                        True,   # stylusBodyTranslation
                                        False), # stylusBodyRotation
                                        rigidBodyCollection )
    #d.mainButton.route ( ctrl )
    d.secondaryButton.route ( ctrl )
    pickUpCtrls.append ( ctrl )

    # Device calibration control 
    calibration= DeviceCalibration.DeviceCalibration ( d, root )
    d.mainButton.route ( calibration )
    deviceCalibration.append ( calibration )
    d.followViewpoint.setValue ( False )

    # Calibration for PHANToM 6DoF 
    d.positionCalibration.setValue ( Matrix4f ( 2, 0, 0, -0.0158491,
                                                0, 2, 0, -0.0560279,
                                                0, 0, 2, -0.297714,
                                                0, 0, 0, 1, ) )
    
    # Device stylus
    stylus, dn= createX3DNodeFromURL ( "x3d/stylus.x3d" )
    dn["STYLUS_A"].material.setValue ( stylusMaterial )
    d.stylus.setValue ( stylus )
    
    deviceIndex= deviceIndex+1

class CleanUpClass:
  def __init__(self, classes_to_clean = [] ):
    self.classes_to_clean = classes_to_clean
  
  def __del__(self):
    for a_class in self.classes_to_clean:
      a_class.cleanUp()

clean_up_class = CleanUpClass( [ kMoveLamp, kDampLamp ] )
