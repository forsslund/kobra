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

geometries= []
geometries.append ( createX3DNodeFromString ("<Sphere radius='0.03' />")[0] )
geometries.append ( createX3DNodeFromString ("<Box size='0.05 0.05 0.05' />")[0] )
geometries.append ( createX3DNodeFromString ("<Cylinder radius='0.02' height='0.05'/>")[0] )

def fireRigidBody ():
  """ Add a new rigid body and fire it at the soft body."""

  shape, dn= createX3DNodeFromString ( """  <Transform>
                                              <Shape DEF='Shape'>
                                                <Appearance>
                                                  <Material diffuseColor='0.5 0.9 0.9' />
                                                </Appearance>
                                              </Shape>
                                            </Transform>""" )
  dn['Shape'].geometry.setValue ( geometries[random.randint(0,len(geometries)-1)] )
  root.children.push_back ( shape )
  
  collidableShape= createNode( "CollidableShape" )
  collidableShape.shape.setValue ( dn['Shape'] )
  sbc.collider.getValue().collidables.push_back ( collidableShape )
  
  rigidBody= createX3DNodeFromString ( """<RigidBody mass='0.1'
                                                     position='0 0 0.1' />""" )[0]
  rigidBody.geometry.push_back ( collidableShape )
  rigidBody.massDensityModel.setValue ( dn['Shape'].geometry.getValue() )
  sbc.rigidBodies.push_back ( rigidBody )
  
  rigidBody.position.route ( shape.translation )
  rigidBody.orientation.route ( shape.rotation )
  
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
        
    elif key == ' ':
      fireRigidBody ()
    return key
        
keyHandler= KeyHandler()
ks.keyPress.route ( keyHandler )

Grabber.addGrabbersForSoftBodies ( sbc, root, 50.0 )

# Updates the transformation of the soft body.
positionSoftBody1 = SFVec3f()
orientationSoftBody1 = SFRotation()
scaleSoftBody1 = SFVec3f()
getTransformSoftBody1 = GetTransforms.GetTransform()

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )

positionSoftBody1.setValue( Vec3f(-0.175, 0.05, -0.1 ) )
orientationSoftBody1.setValue( Rotation(1, 0, 0, 1.57 ) )
scaleSoftBody1.setValue( Vec3f(1.0, 1.0, 1.0 ) )

