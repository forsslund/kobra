from H3DInterface import *
import random
import Grabber
import GetTransforms

cloth,meshToggle,clothMat,ks,sbc,root= references.getValue()
triangleSet= cloth.geometry.getValue()

fixedClothVertices = createNode( "FixedConstraint" )
fixedClothVertices.body1.setValue( cloth )
sbc.constraints.push_back ( fixedClothVertices )

width= 0.35
height= 0.35
resolution= 100

def index ( x, y ):
  return y*int(width*resolution+1) + x

coords= []
indices= []
fixedVertices= []
texCoords= []
for y in range(int(height*resolution+1)):
  for x in range(int(width*resolution+1)):
    coords.append ( Vec3f ( x*(1.0/resolution), y*(1.0/resolution), 0 ) )
    texCoords.append ( Vec2f ( x/(width*resolution), y/(height*resolution) ) )
    if y > 0 and x > 0:
      indices.append ( index ( x, y ) )
      indices.append ( index ( x-1, y ) )
      indices.append ( index ( x-1, y-1 ) )
      
      indices.append ( index ( x, y ) )
      indices.append ( index ( x-1, y-1 ) )
      indices.append ( index ( x, y-1 ) )
      
# pin all four corners
fixedVertices.append ( index ( 0, 0 ) )
fixedVertices.append ( index ( int(width*resolution), 0 ) )
fixedVertices.append ( index ( 0, int(height*resolution) ) )
fixedVertices.append ( index ( int(width*resolution), int(height*resolution) ) )
    
triangleSet.index.setValue ( indices )
triangleSet.coord.getValue().point.setValue ( coords )
triangleSet.texCoord.getValue().point.setValue ( texCoords )
fixedClothVertices.index.setValue ( fixedVertices )

vertexForce1= None
vertexForce2= None

def applyVertexForce1 ():
  global vertexForce1

  if not vertexForce1:
    vertexForce1 = createX3DNodeFromString ( "<VertexBodyForce forces='0 0 0'/>" )[0]
    vertexForce1.body1.setValue( cloth )
    vertexForce1.index.push_back ( index (int(width*resolution)/2,int(height*resolution)/2) )
    sbc.modifiers.push_back ( vertexForce1 )    
    
  forces= vertexForce1.forces.getValue()
  forces[0]= forces[0] + Vec3f ( 0, 0.1, 0 )
  vertexForce1.forces.setValue ( forces )
  
def clearVertexForce1 ():
  global vertexForce1
  
  sbc.modifiers.erase( vertexForce1 )
  vertexForce1= None

def applyVertexForce2 ():
  global vertexForce2

  if not vertexForce2:
    vertexForce2 = createX3DNodeFromString ( "<VertexBodyForce forces='0 0 0'/>" )[0]
    vertexForce2.body1.setValue( cloth )
    vertexForce2.index.push_back ( index (int(width*resolution)/4,int(height*resolution)/2) )
    sbc.modifiers.push_back ( vertexForce2 )
    
  forces= vertexForce2.forces.getValue()
  forces[0]= forces[0] + Vec3f ( 0, 0.1, 0 )
  vertexForce2.forces.setValue ( forces )
  
def clearVertexForce2 ():
  global vertexForce2
  
  sbc.modifiers.erase( vertexForce2 )
  vertexForce2= None
  
# Hide and show the mesh model
class KeyHandler ( AutoUpdate ( SFString ) ):
  def update ( self, value ):
    key= value.getValue().lower()
    
    if key == 'm':
      if meshToggle.graphicsOn.getValue():
        meshToggle.graphicsOn.setValue ( False )
        clothMat.transparency.setValue ( 0.0 )
      else:
        meshToggle.graphicsOn.setValue ( True )
        clothMat.transparency.setValue ( 0.5 )
        
    elif key == '1':
      applyVertexForce1 ()
      
    elif key == '2':
      clearVertexForce1 ()

    elif key == '3':
      applyVertexForce2 ()
      
    elif key == '4':
      clearVertexForce2 ()
      
    return key
        
keyHandler= KeyHandler()
ks.keyPress.route ( keyHandler )

Grabber.addGrabbersForSoftBody ( sbc, cloth, root )

# Updates the transformation of the soft body.
positionSoftBody1 = SFVec3f()
orientationSoftBody1 = SFRotation()
scaleSoftBody1 = SFVec3f()
getTransformSoftBody1 = GetTransforms.GetTransform()

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )

positionSoftBody1.setValue( Vec3f(-0.175, 0, 0 ) )
orientationSoftBody1.setValue( Rotation(1, 0, 0, 1.57 ) )
scaleSoftBody1.setValue( Vec3f(1.0, 1.0, 1.0 ) )

