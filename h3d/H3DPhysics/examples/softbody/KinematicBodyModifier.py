from H3DInterface import *
import random

cloth,meshToggle,clothMat,ks,sbc,root,deformerToggle= references.getValue()

geometries= []
geometries.append ( createX3DNodeFromString ("<Sphere radius='0.01' />")[0] )
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
  
  rigidBody= createX3DNodeFromString ( """<RigidBody mass='0.3'
                                                     position='0 0 0.1' 
                                                     linearVelocity='0 0 -0.5' />""" )[0]
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
        
    elif key == 'd':
      if deformerToggle.graphicsOn.getValue():
        deformerToggle.graphicsOn.setValue ( False )
      else:
        deformerToggle.graphicsOn.setValue ( True )
        
    elif key == ' ':
      fireRigidBody()
        
    return key
        
keyHandler= KeyHandler()
ks.keyPress.route ( keyHandler )
