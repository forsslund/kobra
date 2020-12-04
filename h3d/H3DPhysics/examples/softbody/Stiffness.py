from H3DInterface import *
import SoftBodyUtility
import random

sbc= getNamedNode ( 'SBC' )
shape= getNamedNode ( 'S' )
keySensor= getNamedNode ( 'KS' )

center= Vec3f()
size= Vec3f(0.3, 0.02, 0.3)

geometry= SoftBodyUtility.regularTetraVolume ( 
  center, 
  size, 
  nrNodesX= 20, 
  nrNodesY= 4, 
  nrNodesZ= 20 )

# Note: bendingContraintDistance='0' is a requirement if we want to update the stiffness of the
# edges later. Otherwise the soft body will become unstable.
  
softBody= createX3DNodeFromString ( """
  <SoftBody>
    <MassSpringPhysicsMaterial>
      <UniformMass mass="0.1" /> 
      <UniformStiffness stiffness="0.1" /> 
      <UniformDamping damping="0.001" /> 
      <UniformFriction friction="0.01" /> 
    </MassSpringPhysicsMaterial>
    <BulletSoftBodyOptions pIterations='1' nrClusters='1' bendingContraintDistance='0' enablePerEdgeStiffness='true' />
  </SoftBody>""" )[0]
sbc.softBodies.push_back ( softBody )

deformer= createX3DNodeFromString ( """
  <FunctionSoftBodyModifier DEF='Deformer'>
    <GaussianFunction amplitude='0.000001' width='0.03' containerField='distanceToForce' />
  </FunctionSoftBodyModifier>""" )[0]
deformer.body1.setValue ( softBody )
sbc.modifiers.push_back ( deformer )

softBody.geometry.setValue ( geometry )
shape.geometry.setValue ( geometry )

# Fix all four corners
#
fixRadius= 0.01

SoftBodyUtility.fixVertices ( 
  sbc,
  softBody, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(size.x,0,size.z)/2, 
    radius= fixRadius ) )
    
SoftBodyUtility.fixVertices (
  sbc,
  softBody, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(-size.x,0,size.z)/2, 
    radius= fixRadius ) )
    
SoftBodyUtility.fixVertices ( 
  sbc,
  softBody, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(size.x,0,-size.z)/2, 
    radius= fixRadius ) )
    
SoftBodyUtility.fixVertices ( 
  sbc,
  softBody, 
  SoftBodyUtility.ImplicitVolumeSphere ( 
    center= center+Vec3f(-size.x,0,-size.z)/2, 
    radius= fixRadius ) )
    
# Create a region of different stiffness
#
SoftBodyUtility.setStiffnessImplicitVolume ( 
  softBody, 
  SoftBodyUtility.ImplicitVolumeSphere(
    center,
    radius= 0.1), 
  stiffness= 0.001 )
  
class KeyHandler ( AutoUpdate ( SFString ) ):
  def update ( self, event ):
    k= event.getValue().lower()
    
    if k == '1':
      # Create another region of different stiffness
      print("Create another region of different stiffness")
      SoftBodyUtility.setStiffnessImplicitVolume ( 
        softBody, 
        SoftBodyUtility.ImplicitVolumeSphere(
          center-size/2+Vec3f(size.x*random.random(),size.y*random.random(),size.z*random.random()),
          radius= 0.05), 
        stiffness= 0.001 )
    elif k == '2':
      # Clear per edge stiffness
      print("Clear per edge stiffness")
      softBody.physicsMaterial.getValue().stiffness.setValue ( createX3DNodeFromString ( "<UniformStiffness stiffness='0.1' />" )[0] )
    
    return k
    
keyHandler= KeyHandler()
keySensor.keyPress.route ( keyHandler )