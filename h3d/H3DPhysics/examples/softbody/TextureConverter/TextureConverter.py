from H3DInterface import *
import time

tolerance= 0.001
eps= 0.0001
maxFlt= 100000.0
maxPointFaces= 10

# Time interval for reporting progress
lastReportTime= time.clock()
reportInterval= 5

def isReportTime ():
  global lastReportTime
  curTime= time.clock()
  if curTime-lastReportTime > reportInterval:
    lastReportTime= curTime
    return True
  return False

def triangleNormal ( a, b, c ):
  """ Returns the surface normal of triangle formed by points a, b and c. """
  return (c-b).crossProduct ( b-a ).normalizeSafe()
  
def projectToPlane ( pt, planePt, planeNm ):
  """ Returns the closest point to pt on the plane specified by planePt (point on plane) 
      and planeNm (normal). """
  w= planePt.dotProduct ( planeNm )

  d= planeNm.dotProduct(pt - planeNm*w)
  return pt - planeNm*d

def parallelogramArea ( a, b, c, u ):
  """ Returns the area of the parallelogram defined by points a, b and c.
      The area is signed w.r.t. vector u. """
  v= b - a
  w= c - a
  u1 = v.crossProduct ( w )
  
  if u.dotProduct ( u1 ) < 0:
    return -u1.length()
  else:
    return u1.length()
  
def barycentric ( a, b, c, pt ):
  """ Returns a vector containing barycentric coordinates for point pt in the
      triangle a, b, c. """
  v= b - a
  w= c - a
  u = v.crossProduct ( w )

  A= parallelogramArea ( a, b, c, u )
  A1= parallelogramArea ( c, pt, b, u )
  A2= parallelogramArea ( a, pt, c, u )
  A3= parallelogramArea ( a, b, pt, u )
  
  return Vec3f ( A1/A, A2/A, A3/A )
  
def textureCoordsClosestMatch ( index, point, origPoint, origIndex, origTexCoordIndex, origTexPoint ):
  """ Picks a texture coordinate by trying to match exactly new triangles to old triangles. 
      Fast but doesn't match all triangles, only those that remain unchanged in the new mesh. """

  texPoints= []
  texIndex= []
  
  notFound= []
  for t1 in range(0,len(index),4):
    found= False
    
    if isReportTime():
      print "Direct match: %.0f%% Complete" % ((t1/float(len(index)))*100.0)
    
    for t2 in range(0,len(origIndex),4):
    
      # Check faces are trianglse
      if origIndex[t2+3] != -1 or index[t1+3] != -1 :
        raise ValueError ( 'Meshes must contain only triangular faces!' )
    
      if (((point[index[t1]]-origPoint[origIndex[t2]]).length() < tolerance and
           (point[index[t1+1]]-origPoint[origIndex[t2+1]]).length() < tolerance and
           (point[index[t1+2]]-origPoint[origIndex[t2+2]]).length() <tolerance)):
        
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2 ] ] )
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2+1 ] ] )
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2+2 ] ] )
        found= True
        
      elif (((point[index[t1]]-origPoint[origIndex[t2+2]]).length() < tolerance and
             (point[index[t1+1]]-origPoint[origIndex[t2]]).length() < tolerance and
             (point[index[t1+2]]-origPoint[origIndex[t2+1]]).length() <tolerance)):
        
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2+2 ] ] )
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2 ] ] )
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2+1 ] ] )
        found= True
        
      elif (((point[index[t1]]-origPoint[origIndex[t2+1]]).length() < tolerance and
             (point[index[t1+1]]-origPoint[origIndex[t2+2]]).length() < tolerance and
             (point[index[t1+2]]-origPoint[origIndex[t2]]).length() <tolerance)):
        
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2+1 ] ] )
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2+2 ] ] )
        texPoints.append ( origTexPoint [ origTexCoordIndex [ t2 ] ] )
        found= True
      
      if found:
        texIndex.append ( len(texPoints)-3 )
        texIndex.append ( len(texPoints)-2 )
        texIndex.append ( len(texPoints)-1 )
        texIndex.append ( -1 )
        break
    
    if not found:
      notFound.append ( t1 )
    
  return (texPoints,texIndex,notFound)
  
class TriInfo ( object ):
  def __init__ ( self, t, d, bc ):
    self.t= t
    self.d= d
    self.bc= bc

def findTriangle ( t, triangles ):
  for tri in triangles:
    if tri.t == t:
      return tri
  return None

def maxTriangle ( triangles ):
  max_tri= None
  index= -1
  for i in range(len(triangles)):
    tri= triangles[i]
    if not max_tri or (max_tri and tri.d > max_tri.d):
      max_tri= tri
      index= i
  return index

def minTriangle ( triangles ):
  min_tri= None
  index= -1
  for i in range(len(triangles)):
    tri= triangles[i]
    if not min_tri or (min_tri and tri.d < min_tri.d):
      min_tri= tri
      index= i
  return index
  
def textureCoordsBarycentric ( notFound, index, point, origPoint, origIndex, origTexCoordIndex, origTexPoint, texPoints, texIndex ):
  """ Picks a texture coordinate by matching the new vertex to a triangle on the original mesh, 
      then uses barycentric coordinates to interpolate a value based on the values at the 3 
      vertices of the original triangle. 
      Slow but can handle cases where new and old triangles do not match. """

  # Tolerance for point inside triangle test
  # Must be large enough to consistently report that a point on the vertex of a triangle
  # is inside that triangle in order for the logic to work
  eps_inside= 0.0005
  
  # For each new triangle that has not been matched by the simple algorithm
  for t1_i in range(len(notFound)):
    t1= notFound[t1_i]
    pts= [t1,t1+1,t1+2]
    
    # Report progress
    if isReportTime():
      print "Interpolation: %.0f%% Complete" % ((t1_i/float(len(notFound)))*100.0)
    
    # For each node of the new triangle a list
    # of old triangles that the node falls within
    nodeToTriangle= []
    
    # For each node in the new mesh
    for i in pts:
      p= point[index[i]]
      
      # List of triangles that this node falls within
      # Finds the x closest triangles
      triangles= []
      
      # For each triangle in the old mesh
      max_tri= None
      min_tri= None
      max_tri_i= -1
      for t in range(0,len(origIndex),4):
        
        # Check face is a triangle
        if origIndex[t+3] != -1:
          raise ValueError ( 'Original mesh must contain only triangular faces!' )
          
        # The current triangle
        a= origPoint[origIndex[t]]
        b= origPoint[origIndex[t+1]]
        c= origPoint[origIndex[t+2]]
        
        # Project point p onto the plane formed by t
        p_t= projectToPlane ( p, a, triangleNormal(a,b,c) )
        
        # Calculate Barycentric coordinates for new point in old triangle
        bc= barycentric ( a, b, c, p_t )
        
        # If inside triangle
        if (bc.x >= -eps_inside and bc.y >= -eps_inside and bc.z >= -eps_inside and 
            bc.x <= 1+eps_inside and bc.y <= 1+eps_inside and bc.z <= 1+eps_inside):
        
          # Distance between p and p_t
          d= (p-p_t).length()
          
          # Track closest triangle
          if not min_tri or (min_tri and d < min_tri.d):
            min_tri= TriInfo (t,d,bc)
            
          # Maintain list of the closest maxPointFaces triangles
          if len(triangles) < maxPointFaces:
            triangles.append ( TriInfo (t,d,bc) )
            max_tri_i= maxTriangle ( triangles )
            max_tri= triangles[max_tri_i]
          elif d < max_tri.d:
            del triangles[max_tri_i]
            triangles.append ( TriInfo (t,d,bc) )
            max_tri_i= maxTriangle ( triangles )
            max_tri= triangles[max_tri_i]
      
      # Some assertions that might indicate a problem with the matched triangles
      if min_tri.d > 0.001:
        raise ValueError ( 'Sanity check: Min distance to triangle is large: ' + str (min_tri.d) )
      if max_tri.d < min_tri.d:
        raise ValueError ( 'Sanity check: Max distance is larger than min distance!' )
      
      # Store the list of matched triangles for this node of the new triangle
      nodeToTriangle.append ( triangles )
      
    # Count up the frequencies of each original triangle index
    frq= {}
    for tris in nodeToTriangle:
      for tri in tris:
        if tri.t in frq:
          frq[tri.t]= frq[tri.t] + 1
        else:
          frq[tri.t]= 1
          
    # Find the triangle with the maximum occurance
    max_v= 0
    max_k= -1
    for k,v in frq.iteritems():
      if v > max_v:
        max_v= v
        max_k= k
    
    # For each vertex of the new triangle, generate a texture coordinate
    vertIndex= 0
    
    # If all three vertices of the new triangle fall within the same old triangle
    # then make sure we take all the texture coordinates from that same old triangle
    # Otherwise there may be wrap around problems at texture seams
    if max_v >= 3:
      # All points in the new triangle fall within some common old triangle
      # Use that triangle to calculate the texture coordinates for the new triangle
      for tris in nodeToTriangle:
        tri= findTriangle ( max_k, tris )

        t_a= origTexPoint [ origTexCoordIndex [ tri.t ] ]
        t_b= origTexPoint [ origTexCoordIndex [ tri.t+1 ] ]
        t_c= origTexPoint [ origTexCoordIndex [ tri.t+2 ] ]

        tx= t_a*tri.bc.x + t_b*tri.bc.y + t_c*tri.bc.z
          
        texPoints.append ( tx )
        texIndex.insert ( t1+vertIndex, len(texPoints)-1 ) 
        vertIndex= vertIndex+1
    
    else:
      # Not all of the new triangles fall within some common old triangle
      # Use arbartary triangles to calculate the texture coordinates
      for tris in nodeToTriangle:
        tri= tris[minTriangle(tris)]        

        t_a= origTexPoint [ origTexCoordIndex [ tri.t ] ]
        t_b= origTexPoint [ origTexCoordIndex [ tri.t+1 ] ]
        t_c= origTexPoint [ origTexCoordIndex [ tri.t+2 ] ]

        tx= t_a*tri.bc.x + t_b*tri.bc.y + t_c*tri.bc.z
          
        texPoints.append ( tx )
        texIndex.insert ( t1+vertIndex, len(texPoints)-1 )  
        vertIndex= vertIndex+1
  
    texIndex.insert ( t1+3, -1 )
  
  return texPoints

origGeometry, geometry= references.getValue()

if origGeometry.getTypeName() != "IndexedFaceSet":
  raise TypeError ( 'Original geometry must be IndexedFaceSet' )
if geometry.getTypeName() != "IndexedFaceSet":
  raise TypeError ( 'New geometry must be IndexedFaceSet' )

index= geometry.coordIndex.getValue()
coord= geometry.coord.getValue()
point= coord.point.getValue()

origIndex= origGeometry.coordIndex.getValue()
origCoord= origGeometry.coord.getValue()
origPoint= origCoord.point.getValue()
origTexCoordIndex= origGeometry.texCoordIndex.getValue()
origTexCoord= origGeometry.texCoord.getValue()
origTexPoint= origTexCoord.point.getValue()

print "Mapping texture coordinates... Please wait..."

texPoints, texIndex, notFound= textureCoordsClosestMatch ( index, point, origPoint, origIndex, origTexCoordIndex, origTexPoint )

if len(notFound) > 0:
  print "%d triangles with no match. Passing these on for barycentric interpolation..." % len(notFound)
  print "Created %d texture indices (%d face indices)." % (len(texIndex),len(index))
  textureCoordsBarycentric ( notFound, index, point, origPoint, origIndex, origTexCoordIndex, origTexPoint, texPoints, texIndex )
else:
  print "All triangles directly matched."
  
print "Created %d texture indices (%d face indices)." % (len(texIndex),len(index))
print "Done"

texCoords= createNode( "TextureCoordinate" )
texCoords.point.setValue ( texPoints )
geometry.texCoord.setValue ( texCoords )
geometry.texCoordIndex.setValue ( texIndex )

f= open ( 'newGeometry.x3d', 'w' )
f.write ( writeNodeAsX3D ( geometry ) )
print "Written new geometry to: " + str(f.name)