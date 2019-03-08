from H3DInterface import *
import random
import GetTransforms

cloth,sbc,attachment,attachment_option= references.getValue()
triangleSet= cloth.geometry.getValue()

fixedClothVertices = createNode( "FixedConstraint" )
fixedClothVertices.body1.setValue( cloth )
sbc.constraints.push_back ( fixedClothVertices )

width= 0.2
height= 0.3
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
      
# pin the top left and right corners
fixedVertices.append ( index ( 0, int(height*resolution) ) )
fixedVertices.append ( index ( int(width*resolution), int(height*resolution) ) )
fixedVertices.append ( index ( 0, 0 ) )
fixedVertices.append ( index ( int(width*resolution), 0 ) )
    
# Attach the bottom corners to the rigid body
attachment.index.push_back ( index ( 0, int(height*resolution) / 2 ) )
attachment.index.push_back ( index ( int(width*resolution), int(height*resolution) / 2 ) )
    
triangleSet.index.setValue ( indices )
triangleSet.coord.getValue().point.setValue ( coords )
triangleSet.texCoord.getValue().point.setValue ( texCoords )
fixedClothVertices.index.setValue ( fixedVertices )

# Updates the transformation of the soft body.
positionSoftBody1 = SFVec3f()
orientationSoftBody1 = SFRotation()
scaleSoftBody1 = SFVec3f()
getTransformSoftBody1 = GetTransforms.GetTransform()

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )

positionSoftBody1.setValue( Vec3f(0.15, -0.05, 0.1 ) )
scaleSoftBody1.setValue( Vec3f(1.0, 1.0, 1.0 ) )
orientationSoftBody1.setValue( Rotation( 1, 0, 0, -1.2 ) * Rotation( 0, 0, 1, 1.57 ) )

class KeyHandler( PeriodicUpdate(SFString) ):
  def update( self, event ):
    global attachment_option
    key = event.getValue()
    if key == ' ':
      attachment_option.collide.setValue( not attachment_option.collide.getValue() )
    return key

keyHandler = KeyHandler()
key_sensor = createNode( "KeySensor" )
key_sensor.keyRelease.routeNoEvent( keyHandler )
