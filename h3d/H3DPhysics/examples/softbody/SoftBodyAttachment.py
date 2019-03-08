from H3DInterface import *
import SoftBodyUtility
import math
import random

sbc= getNamedNode ( 'SBC' )
shape= getNamedNode ( 'S' )
shape2= getNamedNode ( 'S2' )
attachmentVisuals= getNamedNode ( 'ATTACHMENT_VISUALS' )

center= Vec3f()
size= Vec3f(0.3, 0.02, 0.2)

constraints= []

class SoftBodyAttachmentVisualization ( object ):
  """ A class which creates and updates a visual representation
      of a SoftBodyAttachment node for debugging and testing.
  """
  
  class UpdateVisuals ( PeriodicUpdate ( TypedField ( SFBool, (MFVec3f, MFVec3f, TrackedMFInt32, TrackedMFInt32) ) ) ):
    """ A field used to update visual representation from coords and indices. """
    
    def __init__ ( self, line, coords ):
      PeriodicUpdate ( TypedField ( SFBool, (MFVec3f, MFVec3f, TrackedMFInt32, TrackedMFInt32) ) ).__init__ ( self )
      self._line= line
      self._coords= coords
    
    def update ( self, event ):
      coord1, coord2, index1, index2= self.getRoutesIn()
      
      point1= coord1.getValue()
      point2= coord2.getValue()
      idx1= index1.getValue()
      idx2= index2.getValue()
      
      index= []
      coord= []
      
      for i in range(min(len(idx1),len(idx2))):
        coord.append ( point1[idx1[i]] )
        coord.append ( point2[idx2[i]] )
        index.append ( len(coord)-2 )
        index.append ( len(coord)-1 )
        index.append ( -1 )
        
      self._line.coordIndex.setValue ( index )
      self._coords.point.setValue ( coord )
      
      return False
  
  def __init__ ( self, softBodyAttachment ):
    """ Create a visualization for the specified attachment node. """

    self._softBodyAttachment= softBodyAttachment
    
    self.node, self._dn= createX3DNodeFromString ( """
      <Shape>
        <Appearance>
          <Material emissiveColor='1 0 0' />
        </Appearance>
        <IndexedLineSet DEF='LINE'>
          <Coordinate DEF='COORD' />
        </IndexedLineSet>
      </Shape> """ )
      
    # Set up routes to update the visuals as coords change
    self._updateVisuals= self.UpdateVisuals ( self._dn['LINE'], self._dn['COORD'] )
    softBodyAttachment.body1.getValue().geometry.getValue().coord.getValue().point.route ( self._updateVisuals )
    softBodyAttachment.body2.getValue().geometry.getValue().coord.getValue().point.route ( self._updateVisuals )
    softBodyAttachment.index.route ( self._updateVisuals )
    softBodyAttachment.index2.route ( self._updateVisuals )
    
def getDevicePosition ():
  """ Helper function to return first device position. """
  
  position= Vec3f()
  di= getActiveDeviceInfo()
  if di:
    devices= di.device.getValue()
    if devices:
      position= devices[0].weightedProxyPosition.getValue()
  return position

def getClosestAttachment ( position ):
  """ Returns the attachment whose original volume is closest 
      to the given position. """
  
  minDist= None
  closest= None
  for a in constraints:
    center= a[1].transform*a[1].center
    d= (position-center).lengthSqr()
    if minDist is None or d < minDist:
      minDist= d
      closest= a
      
  return closest
  
def getLinksInsideVolume ( attachment, volume ):
  """ Returns a list of indices of links in the attachment that are inside
      the specified volume. """
      
  index1= attachment.index.getValue()
  index2= attachment.index2.getValue()
  
  point1= attachment.body1.getValue().geometry.getValue().coord.getValue().point.getValue()
  point2= attachment.body2.getValue().geometry.getValue().coord.getValue().point.getValue()
  
  index= []
  for i in range(min(len(index1),len(index2))):
    p= ( point1[index1[i]] + point2[index2[i]] ) / 2.0
    if volume.inside ( p ):
      index.append ( i )
      
  return index
  
def getClosestCoordIndex ( softBody, position ):
  """ Returns the index of the closest coordinate to position
      in the soft body goemetry. 
  """
  
  point= softBody.geometry.getValue().coord.getValue().point.getValue()
  
  closestDistance= None
  closestIndex= None
  for i, p in enumerate ( point ):
    d= (p-position).lengthSqr()
    if closestIndex is None or d < closestDistance:
      closestDistance= d
      closestIndex= i
  
  return closestIndex
  
def isLinkInAttachment ( attachment, index1, index2 ):
  """ Returns true if there is a link in the attachment 
      between the specified indices. 
  """

  a= attachment.index.getValue()
  b= attachment.index2.getValue()
  for i in range(min(len(a),len(b))):
    l1= a[i]
    l2= b[i]
    if ((l1 == index1 and l2 == index2) or
        (l2 == index1 and l1 == index2)):
      return True
  
  return False
  
def renderAttachments ():
  """ Update the shapes used to visualize the attachment positions. """
  
  attachmentVisuals.children.clear()
  for a in constraints:
    attachmentVisuals.children.push_back ( a[1].getNode() )
    attachmentVisuals.children.push_back ( a[2].node )

def removeAttachment ():
  """ Remove the soft body attachment first added. """
  
  if constraints:
    sbc.constraints.erase ( constraints[-1][0] )
    del constraints[-1]
    
  renderAttachments()
    
def addAttachment ( volume ):
  """ Add an attachment at the device position. """
  
  # Get device position
  p= getDevicePosition()
  volume.transform.setElement ( 0, 3, p.x )
  volume.transform.setElement ( 1, 3, p.y )
  volume.transform.setElement ( 2, 3, p.z )
      
  # Create the attachment
  SoftBodyUtility.attachCoincidentVertices ( 
    sbc, softBody, softBody2, volume, tolerance= 0.001 )
  c= sbc.constraints.getValue()[-1]
  constraints.append ( (c,volume,SoftBodyAttachmentVisualization(c)) )
  
  renderAttachments()

def reduceLinkStiffness ():
  """ Reduce the stiffness of some links near to the device. """
  
  # Amount to reduce stiffness by each step
  delta= 0.02
  
  # Get device position
  position= getDevicePosition()
  
  # Get closest attachment volume, links will be updated in that attachment node
  attachment= getClosestAttachment ( position )[0]
  
  # Get links within the specified volume near the device
  linkIndices= getLinksInsideVolume ( attachment, 
    SoftBodyUtility.ImplicitVolumeSphere ( 
      center= position, 
      radius= 0.04 ) )
  
  # Update the stiffness of the links, reduce all by some constant
  
  # First time, change to NonUniformStiffness from UniformStiffness
  material= attachment.physicsMaterial.getValue()
  if material is None or material.getTypeName() != 'MassSpringPhysicsMaterial':
    material= createNode ( 'MassSpringPhysicsMaterial' )
    attachment.physicsMaterial.setValue ( material )
  
  stiffnessNode= material.stiffness.getValue()
  if stiffnessNode is None or stiffnessNode.getTypeName() != 'NonUniformStiffness' :
    n= createNode ( 'NonUniformStiffness' )
    defaultStiffness= 1.0
    if stiffnessNode:
      defaultStiffness= stiffnessNode.stiffness.getValue()
    n.stiffnessPerUnit.setValue ( len(attachment.index.getValue())*[defaultStiffness] )
    material.stiffness.setValue ( n )
    stiffnessNode= n
    
  currentStiffness= stiffnessNode.stiffnessPerUnit.getValue()
  newStiffness= []
  for i in linkIndices:
    s= max(0.0,currentStiffness[i]-delta)
    newStiffness.append ( s )
    
  # Update the specified indexed values in the stiffnessPerUnit field
  stiffnessNode.stiffnessPerUnit.updateTracked ( linkIndices, newStiffness )
  
def removeLinks ():
  """ Remove links close to the haptics device position. """
  
  # Get device position
  position= getDevicePosition()
  
  # Get closest attachment volume, links will be removed from that attachment node
  attachment= getClosestAttachment ( position )[0]
  
  # Get links within the specified volume near the device
  linkIndices= getLinksInsideVolume ( attachment, 
    SoftBodyUtility.ImplicitVolumeSphere ( 
      center= position, 
      radius= 0.04 ) )
  
  # Remove those links
  attachment.index.eraseTracked ( linkIndices )
  attachment.index2.eraseTracked ( linkIndices )
  
def addLinks ():
  """ Adds links close to the haptics device position. """
  
  # Get device position
  position= getDevicePosition()
  
  # Get closest attachment volume, links will be inserted to that attachment node
  attachment= getClosestAttachment ( position )[0]
  
  # Get closest coord indexed in body 1 and 2 geomety to device positon
  closestIndex1= getClosestCoordIndex ( attachment.body1.getValue(), position )
  closestIndex2= getClosestCoordIndex ( attachment.body2.getValue(), position )
  
  if not isLinkInAttachment ( attachment, closestIndex1, closestIndex2 ):
    attachment.index.pushBackTracked ( closestIndex1 )
    attachment.index2.pushBackTracked ( closestIndex2 )
  
def updateLink():
  """ Updates a link. Picks an arbitrary link, updates it
      so that it now connects the two vertices on each soft body
      closest to the current device position. """
  
  # Get device position
  position= getDevicePosition()
  
  # Get closest attachment volume, links will be removed from that attachment node
  attachment= getClosestAttachment ( position )[0]
  
  if not attachment.index.empty() and not attachment.index2.empty():
  
    # Pick a link to update in the closest attachment
    linkIndex= random.randint ( 0, min(attachment.index.size(),attachment.index2.size())-1 )
    
    # Update the link with new soft body geometry vertex indices
    # Get closest coord indexed in body 1 and 2 geomety to device positon
    closestIndex1= getClosestCoordIndex ( attachment.body1.getValue(), position )
    closestIndex2= getClosestCoordIndex ( attachment.body2.getValue(), position )
  
    if not isLinkInAttachment ( attachment, closestIndex1, closestIndex2 ):
      attachment.index.updateTracked ( linkIndex, closestIndex1 )
      attachment.index2.updateTracked ( linkIndex, closestIndex2 )
  
class KeyHandler ( AutoUpdate ( SFString ) ):
  def update ( self, event ):
    k= event.getValue().lower()
    
    if k == 'g':
      # Toggle gravity on/off
      g= sbc.gravity.getValue()
      if abs(g.y) > 0:
        g.y= 0
      else:
        g.y= -1
      sbc.gravity.setValue ( g )
      
    if k == 'x':
      # Remove an attachment
      removeAttachment()
      
    if k == 'a':
      # Add an attachment
      addAttachment ( 
        SoftBodyUtility.ImplicitVolumeSphere ( 
          center= Vec3f(), 
          radius= 0.02 ) )
        
    if k == 'c':
      # Add an attachment
      addAttachment ( 
        SoftBodyUtility.ImplicitVolumeCylinder ( 
          center= Vec3f(), 
          radius= 0.02,
          height= 0.1,
          transform= Matrix4f(Rotation(1,0,0,math.pi/2))) )
      
    if k == 'd':
      # Show attachment volumes
      attachmentVisuals.graphicsOn.setValue ( 
        not attachmentVisuals.graphicsOn.getValue() )
        
    if k == 's':
      # Reduce stiffness of nearby links
      reduceLinkStiffness()
      
    if k == 'r':
      # Remove nearby links
      removeLinks()
      
    if k == 'i':
      # Inserts links near the device
      addLinks()
      
    if k == 'u':
      # Updates a link. Picks an arbitrary link, updates it
      # so that it now connects the two vertices on each soft body
      # closest to the current device position
      updateLink()
    
    return k

# Soft body 1
# -----------

geomtry= SoftBodyUtility.regularTetraVolume ( 
  center= center-Vec3f(size.x/2,0,0), 
  size= size, 
  nrNodesX= 20, 
  nrNodesY= 4, 
  nrNodesZ= 20 )
  
softBody= createX3DNodeFromString ( """
  <SoftBody>
    <MassSpringPhysicsMaterial>
      <UniformMass mass="0.1" /> 
      <UniformStiffness stiffness="0.2" /> 
      <UniformDamping damping="0.001" /> 
      <UniformFriction friction="0.01" /> 
    </MassSpringPhysicsMaterial>
    <BulletSoftBodyOptions pIterations='1' nrClusters='1' />
  </SoftBody>""" )[0]
sbc.softBodies.push_back ( softBody )

deformer= createX3DNodeFromString ( """
  <FunctionSoftBodyModifier DEF='Deformer'>
    <GaussianFunction amplitude='0.000001' width='0.03' containerField='distanceToForce' />
  </FunctionSoftBodyModifier>""" )[0]
deformer.body1.setValue ( softBody )
sbc.modifiers.push_back ( deformer )

softBody.geometry.setValue ( geomtry )
shape.geometry.setValue ( geomtry )

# Fix two corners
#
fixRadius= 0.01

SoftBodyUtility.fixVertices ( 
  sbc,
  softBody, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(-size.x,0,-size.z/2), 
    radius= fixRadius ) )
    
SoftBodyUtility.fixVertices ( 
  sbc,
  softBody, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(-size.x,0,size.z/2), 
    radius= fixRadius ) )

# Soft body 2
# -----------

geomtry2= SoftBodyUtility.regularTetraVolume ( 
  center= center+Vec3f(size.x/2,0,0), 
  size= size, 
  nrNodesX= 20, 
  nrNodesY= 4, 
  nrNodesZ= 20 )
  
softBody2= createX3DNodeFromString ( """
  <SoftBody>
    <MassSpringPhysicsMaterial>
      <UniformMass mass="0.1" /> 
      <UniformStiffness stiffness="0.2" /> 
      <UniformDamping damping="0.001" /> 
      <UniformFriction friction="0.01" /> 
    </MassSpringPhysicsMaterial>
    <BulletSoftBodyOptions pIterations='1' nrClusters='1' />
  </SoftBody>""" )[0]
sbc.softBodies.push_back ( softBody2 )

deformer= createX3DNodeFromString ( """
  <FunctionSoftBodyModifier DEF='Deformer'>
    <GaussianFunction amplitude='0.000001' width='0.03' containerField='distanceToForce' />
  </FunctionSoftBodyModifier>""" )[0]
deformer.body1.setValue ( softBody2 )
sbc.modifiers.push_back ( deformer )

softBody2.geometry.setValue ( geomtry2 )
shape2.geometry.setValue ( geomtry2 )

# Fix two corners
#
fixRadius= 0.01

SoftBodyUtility.fixVertices ( 
  sbc,
  softBody2, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(size.x,0,-size.z/2), 
    radius= fixRadius ) )
    
SoftBodyUtility.fixVertices ( 
  sbc,
  softBody2, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(size.x,0,size.z/2), 
    radius= fixRadius ) )
    
# Attach the two soft bodies at coincident vertices
#
volume= SoftBodyUtility.ImplicitVolumeSphere ( 
  center= Vec3f(0,0,-size.z/2), 
  radius= 0.02 )
SoftBodyUtility.attachCoincidentVertices ( 
  sbc, softBody, softBody2, volume, tolerance= 0.00001 )
c= sbc.constraints.getValue()[-1]
constraints.append ( (c,volume,SoftBodyAttachmentVisualization(c)) )
  
volume= SoftBodyUtility.ImplicitVolumeSphere ( 
  center= Vec3f(0,0,size.z/2), 
  radius= 0.02 )
SoftBodyUtility.attachCoincidentVertices ( 
  sbc, softBody, softBody2, volume, tolerance= 0.00001 )
c= sbc.constraints.getValue()[-1]
constraints.append ( (c,volume,SoftBodyAttachmentVisualization(c)) )

renderAttachments()
  
keyHandler= KeyHandler()
getNamedNode ( 'KS' ).keyPress.route ( keyHandler )