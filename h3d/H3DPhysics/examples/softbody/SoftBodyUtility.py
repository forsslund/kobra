from H3DInterface import *
  
def regularTetraVolume ( center, size, nrNodesX, nrNodesY, nrNodesZ, edgeIndices= None, orientation = None ):
  """ Returns a regular volume as a IndexedTetraSet node.
        center -- the center of the grid -- Vec3f
        size -- the dimensions of the volume in x, y and z -- Vec3f
        nrNodesX -- the number of vertices along the x-axis -- int (>1)
        nrNodesY -- the number of vertices along the y-axis -- int (>1)
        nrNodesZ -- the number of vertices along the z-axis -- int (>1)
        edgeIndices -- if a list is specified, indices of edge vertices will be added to it -- [] """
        
  tetraSet, dn= createX3DNodeFromString ( """
    <IndexedTetraSet ccw='false' >
      <Coordinate DEF='C' />
    </IndexedTetraSet>
    """ )
  
  rot = Rotation( 0, 0, 1, 0 )
  if orientation != None:
    rot = orientation
  
  # Create vertices
  points= []
  for x in range(nrNodesX):
    for y in range(nrNodesY):
      for z in range(nrNodesZ):
        points.append ( center + rot * Vec3f ( size.x * ( float(x) / (nrNodesX-1) - 0.5 ),
                                         size.y * ( float(y) / (nrNodesY-1) - 0.5 ),
                                         size.z * ( float(z) / (nrNodesZ-1) - 0.5 ) ) )
        if ( not edgeIndices is None and ( 
             y == 0 or y == nrNodesY-1 or x == 0 or x == nrNodesX-1 or z == 0 or z == nrNodesZ-1 ) ):
          edgeIndices.append ( len(points)-1 )
  
  # Link vertices to form tetra
  indices= []
  for x in range(nrNodesX-1):
    for y in range(nrNodesY-1):
      for z in range(nrNodesZ-1):
        i= z + nrNodesZ*y + nrNodesZ*nrNodesY*x
        
        # define vertex indices
        a= z + nrNodesZ*y + nrNodesZ*nrNodesY*x
        b= z + nrNodesZ*y + nrNodesZ*nrNodesY*(x+1)
        c= (z+1) + nrNodesZ*y + nrNodesZ*nrNodesY*x
        d= (z+1) + nrNodesZ*y + nrNodesZ*nrNodesY*(x+1)
        e= z + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*x
        f= z + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*(x+1)
        g= (z+1) + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*x
        h= (z+1) + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*(x+1)
        
        # define tetra to build a cube
        indices.append ( e )
        indices.append ( f )
        indices.append ( g )
        indices.append ( a )
        
        indices.append ( c )
        indices.append ( d )
        indices.append ( b )
        indices.append ( h )
        
        indices.append ( f )
        indices.append ( h )
        indices.append ( g )
        indices.append ( b )
        
        indices.append ( g )
        indices.append ( h )
        indices.append ( c )
        indices.append ( b )
        
        indices.append ( a )
        indices.append ( b )
        indices.append ( f )
        indices.append ( g )
        
        indices.append ( c )
        indices.append ( a )
        indices.append ( g )
        indices.append ( b )
  
  tetraSet.index.setValue ( indices )
  dn['C'].point.setValue ( points )
  
  return tetraSet

class ImplicitVolume ( object ):

  def __init__ ( self, transform= None ):
    if transform is None:
      transform= Matrix4f()
    self.transform= transform
    self.inverseTransform= transform.inverse()
    
  def inside ( self, point ):
    return True
    
  def getNode ( self ):
    return None
  
class ImplicitVolumeSphere ( ImplicitVolume ):

  def __init__ ( self, center= None, radius= 1.0, transform= None ):
    super ( ImplicitVolumeSphere, self ).__init__ ( transform )
    if center is None:
      center= Vec3f()
    self.center= center
    self.radius= radius
    
  def inside ( self, point ):
    localPoint= self.inverseTransform*point
    return (localPoint-self.center).length() < self.radius
    
  def getNode ( self ):
    """ Return a node that represents the volume graphically. 
        Useful for debugging. """
    
    n, dn= createX3DNodeFromString ( """
      <MatrixTransform DEF='T1'>
        <Transform DEF='T'>
          <Shape>
            <Appearance>
              <Material diffuseColor='1 0 0' transparency='0.5' />
            </Appearance>
            <Sphere DEF='G' />
          </Shape>
        </Transform>
      </MatrixTransform> """ )
    dn['G'].radius.setValue ( self.radius )
    dn['T'].translation.setValue ( self.center )
    dn['T1'].matrix.setValue ( self.transform )
    return n
    
class ImplicitVolumeCylinder ( ImplicitVolume ):

  def __init__ ( self, center, radius, height, transform= None ):
    super ( ImplicitVolumeCylinder, self ).__init__ ( transform )
    self.center= center
    self.radius= radius
    self.height= height
    
  def inside ( self, point ):
    localPoint= self.inverseTransform*point
    localPointXZ= Vec2f ( localPoint.x, localPoint.z )
    centerXZ= Vec2f ( self.center.x, self.center.z )
    return ( (localPointXZ-centerXZ).length() < self.radius and
              localPoint.y >= self.center.y-self.height/2 and 
              localPoint.y <= self.center.y+self.height/2 )
    
  def getNode ( self ):
    """ Return a node that represents the volume graphically. 
        Useful for debugging. """
    
    n, dn= createX3DNodeFromString ( """
      <MatrixTransform DEF='T1'>
        <Transform DEF='T'>
          <Shape>
            <Appearance>
              <Material diffuseColor='1 0 0' transparency='0.5' />
            </Appearance>
            <Cylinder DEF='G' />
          </Shape>
        </Transform>
      </MatrixTransform>""" )
    dn['G'].radius.setValue ( self.radius )
    dn['G'].height.setValue ( self.height )
    dn['T'].translation.setValue ( self.center )
    dn['T1'].matrix.setValue ( self.transform )
    return n
    
class ImplicitVolumeBox ( ImplicitVolume ):

  def __init__ ( self, center, size, transform= None ):
    super ( ImplicitVolumeBox, self ).__init__ ( transform )
    self.center= center
    self.size= size
    
  def inside ( self, point ):
    localPoint= self.inverseTransform*point
    
    topRightNear = self.center + self.size/2
    bottomLeftFar= self.center - self.size/2
    
    return ( localPoint.x > bottomLeftFar.x and localPoint.x < topRightNear.x and
             localPoint.y > bottomLeftFar.y and localPoint.y < topRightNear.y and
             localPoint.z > bottomLeftFar.z and localPoint.z < topRightNear.z )
    
  def getNode ( self ):
    """ Return a node that represents the volume graphically. 
        Useful for debugging. """
    
    n, dn= createX3DNodeFromString ( """
      <MatrixTransform DEF='T1'>
        <Transform DEF='T'>
          <Shape>
            <Appearance>
              <Material diffuseColor='1 0 0' transparency='0.5' />
            </Appearance>
            <Box DEF='G' />
          </Shape>
        </Transform>
      </MatrixTransform>""" )
    dn['G'].size.setValue ( self.size )
    dn['T'].translation.setValue ( self.center )
    dn['T1'].matrix.setValue ( self.transform )
    return n

def fixVertices ( physicsCollection, softBody, implicitVolume ):
  """ Fix some vertices in the soft body that are inside the specified volume. """
  
  # Create a fixed vertex constraint
  fixedVertices= createNode ( 'FixedConstraint' )
  fixedVertices.body1.setValue ( softBody )
  physicsCollection.constraints.push_back ( fixedVertices )
  
  g= softBody.geometry.getValue()
  c= g.coord.getValue()
  
  nrVerts= 0
  for i, p in enumerate(c.point.getValue()):
    if implicitVolume.inside ( p ):
      fixedVertices.index.push_back ( i )
      nrVerts= nrVerts + 1
      
  return nrVerts
    
def getEdgeSoftBody ( elementIndex, localEdgeIndex ):
  baseIndex= elementIndex*4
  if localEdgeIndex==0:
    return (baseIndex,baseIndex+1)
  elif localEdgeIndex==1:
    return (baseIndex+1,baseIndex+2)
  elif localEdgeIndex==2:
    return (baseIndex+2,baseIndex)
  elif localEdgeIndex==3:
    return (baseIndex,baseIndex+3)
  elif localEdgeIndex==4:
    return (baseIndex+1,baseIndex+3)
  elif localEdgeIndex==5:
    return (baseIndex+2,baseIndex+3)
  
def getEdgeCloth ( elementIndex, localEdgeIndex ):
  baseIndex= elementIndex*3
  if localEdgeIndex==0:
    return (baseIndex,baseIndex+1)
  elif localEdgeIndex==1:
    return (baseIndex+1,baseIndex+2)
  elif localEdgeIndex==2:
    return (baseIndex+2,baseIndex)
  
def getEdgeRope ( elementIndex, localEdgeIndex ):
  return elementIndex
    
def setStiffnessImplicitVolume ( softBody, implicitVolume, stiffness ):
  """ Sets the stiffness of links of a soft body within a volume defined by implicitVolume. 
      The region outside the volume remains unchanged.
      
      The implicitVolume should be a class that implements the function: bool inside ( Vec3f point)
  
      Raises ValueError if type of softBody is not of a supported type.
      Raises ValueError if type of softBody geometry does not contain a Coordinate node.
  """
  
  typename= softBody.getTypeName()
  if typename == "SoftBody":
    coordPerElement= 4
    edgesPerElement= 6
    getEdge= getEdgeSoftBody
  elif typename == "Cloth":
    coordPerElement= 3
    edgesPerElement= 3
    getEdge= getEdgeCloth
  elif typename == "Rope":
    coordPerElement= 2
    edgesPerElement= 1
    getEdge= getEdgeRope
  else:
    raise ValueError, "Unsupported soft body type: " + str(typename)
  
  g= softBody.geometry.getValue()
  c= g.coord.getValue()
  try:
    p= c.point.getValue()
  except:
    raise ValueError, "Soft body geometry must contain a Coordinate node."
  try:
    index= g.index.getValue()
  except AttributeError:
    index= g.coordIndex.getValue()
    
  l= len(index)
  nrEdges= (l/coordPerElement)*edgesPerElement
  
  globalStiffness= 1.0
  stiffnessNode= None
  physicsMaterial= softBody.physicsMaterial.getValue()
  if physicsMaterial:
    try:
      stiffnessNode= physicsMaterial.stiffness.getValue()
    except AttributeError:
      pass
    if stiffnessNode:
      if stiffnessNode.getTypeName() == 'UniformStiffness':
        globalStiffness= stiffnessNode.stiffness.getValue()
      if stiffnessNode.getTypeName() != 'NonUniformStiffness':
        stiffnessNode= createNode ( 'NonUniformStiffness' )
        physicsMaterial.stiffness.setValue ( stiffnessNode )
        
  if not stiffnessNode:
    raise ValueError, "Unsupported or missing physicsMaterial or stiffness node."
  
  edgeStiffnesses= stiffnessNode.stiffnessPerUnit.getValue()
  
  updateIndices= []
  updateValues= []
  
  updateAll= False
  if len(edgeStiffnesses) < nrEdges:
    edgeStiffnesses= [globalStiffness]*nrEdges
    updateAll= True
  
  updateCount= 0
  globalEdgeIndex= 0
  for elementIndex in range(len(index)/coordPerElement):
    for edgeIndex in range(edgesPerElement):
      a, b= getEdge ( elementIndex, edgeIndex )

      p1= p[index[a]]
      p2= p[index[b]]
      
      edgePos= (p1+p2)/2
      
      if implicitVolume.inside ( edgePos ):
        updateCount= updateCount+1
        if updateAll:
          edgeStiffnesses[globalEdgeIndex]= stiffness
        else:
          updateIndices.append ( globalEdgeIndex )
          updateValues.append ( stiffness )
        
      globalEdgeIndex= globalEdgeIndex+1
  
  if updateAll:
    stiffnessNode.stiffnessPerUnit.setValue ( edgeStiffnesses )
  else:
    stiffnessNode.stiffnessPerUnit.updateTracked ( updateIndices, updateValues )
    
def attachCoincidentVertices ( 
  physicsCollection, 
  softBody1, 
  softBody2, 
  implicitVolume, 
  tolerance= 0.00001, 
  stiffness= 0.8,
  attachPositions= None ):
  """ Attach coincident vertices in softBody and softBody2 (i.e. those closer than tolerance) using
      a SoftBodyAttachment node. 
      
      A SoftBodyAttachment will be created and added to the simulation. The SoftBodyAttachment can be accessed immediately after
      by accessing the last item in the constraints field of the physicsCollection.
      
      \param attachPositions If not None, this should be a list to which the position of each attachment that is created will be added.
                             The order will match the order that the links appear in the SoftBodyAttachment. The position is the center 
                             of the link.
      
      \return the number of vertices attached (from one body). """
      
  g1= softBody1.geometry.getValue()
  g2= softBody2.geometry.getValue()
  
  c1= g1.coord.getValue()
  c2= g2.coord.getValue()
  
  p1= c1.point.getValue()
  p2= c2.point.getValue()
  
  index1= []
  index2= []
  tolerance2= tolerance*tolerance
  for i1, a in enumerate ( p1 ):
    if implicitVolume.inside ( a ):
      for i2, b in enumerate ( p2 ):
        if (a-b).lengthSqr() < tolerance2:
          index1.append ( i1 )
          index2.append ( i2 )
          
          if not attachPositions is None:
            attachPositions.append ( (a+b)/2 )
  
  attachment= createNode ( 'SoftBodyAttachment' )
  attachment.body1.setValue ( softBody1 )
  attachment.index.setValue ( index1 )
  attachment.body2.setValue ( softBody2 )
  attachment.index2.setValue ( index2 )
  
  material, dn= createX3DNodeFromString ( """ 
    <MassSpringPhysicsMaterial>
      <UniformStiffness DEF='STIFFNESS' />
    </MassSpringPhysicsMaterial> """ )
  dn['STIFFNESS'].stiffness.setValue ( stiffness )
  attachment.physicsMaterial.setValue ( material )
  
  physicsCollection.constraints.push_back ( attachment )
  
  return len(index1)


def regularHexaVolume ( center, size, nrNodesX, nrNodesY, nrNodesZ, edgeIndices= None, orientation = None ):
  """ Returns a regular volume as a IndexedTetraSet node.
        center -- the center of the grid -- Vec3f
        size -- the dimensions of the volume in x, y and z -- Vec3f
        nrNodesX -- the number of vertices along the x-axis -- int (>1)
        nrNodesY -- the number of vertices along the y-axis -- int (>1)
        nrNodesZ -- the number of vertices along the z-axis -- int (>1)
        edgeIndices -- if a list is specified, indices of edge vertices will be added to it -- [] """
        
  hexaSet, dn= createX3DNodeFromString ( """
    <IndexedHexaSet>
      <Coordinate DEF='C' />
    </IndexedHexaSet>
    """ )
  
  rot = Rotation( 0, 0, 1, 0 )
  if orientation != None:
    rot = orientation
  
  # Create vertices
  points= []
  for x in range(nrNodesX):
    for y in range(nrNodesY):
      for z in range(nrNodesZ):
        points.append ( center + rot * Vec3f ( size.x * ( float(x) / (nrNodesX-1) - 0.5 ),
                                         size.y * ( float(y) / (nrNodesY-1) - 0.5 ),
                                         size.z * ( float(z) / (nrNodesZ-1) - 0.5 ) ) )
        if ( not edgeIndices is None and ( 
             y == 0 or y == nrNodesY-1 or x == 0 or x == nrNodesX-1 or z == 0 or z == nrNodesZ-1 ) ):
          edgeIndices.append ( len(points)-1 )
  
  # Link vertices to form hexahedron
  indices= []
  for x in range(nrNodesX-1):
    for y in range(nrNodesY-1):
      for z in range(nrNodesZ-1):
        # define vertex indices
        a= z + nrNodesZ*y + nrNodesZ*nrNodesY*x
        b= z + nrNodesZ*y + nrNodesZ*nrNodesY*(x+1)
        c= (z+1) + nrNodesZ*y + nrNodesZ*nrNodesY*x
        d= (z+1) + nrNodesZ*y + nrNodesZ*nrNodesY*(x+1)
        e= z + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*x
        f= z + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*(x+1)
        g= (z+1) + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*x
        h= (z+1) + nrNodesZ*(y+1) + nrNodesZ*nrNodesY*(x+1)
        
        # define hexahedron to build a cube
        indices.append ( a )
        indices.append ( c )
        indices.append ( g )
        indices.append ( e )

        indices.append ( b )
        indices.append ( d )
        indices.append ( h )
        indices.append ( f )

  hexaSet.index.setValue ( indices )
  dn['C'].point.setValue ( points )
  
  return hexaSet