from H3DInterface import *
import random
import GetTransforms

sbc = references.getValue()[0]

# Updates the transformation of the soft body.
positionSoftBody1 = SFVec3f()
orientationSoftBody1 = SFRotation()
scaleSoftBody1 = SFVec3f()
getTransformSoftBody1 = GetTransforms.GetTransform()

positionSoftBody2 = SFVec3f()
orientationSoftBody2 = SFRotation()
scaleSoftBody2 = SFVec3f()
getTransformSoftBody2 = GetTransforms.GetTransform()

positionSoftBody1.route( getTransformSoftBody1 )
orientationSoftBody1.route( getTransformSoftBody1 )
scaleSoftBody1.route( getTransformSoftBody1 )

positionSoftBody2.route( getTransformSoftBody2 )
orientationSoftBody2.route( getTransformSoftBody2 )
scaleSoftBody2.route( getTransformSoftBody2 )

getTransformSoftBody1.route( sbc.softBodies.getValue()[0].transform )
getTransformSoftBody2.route( sbc.softBodies.getValue()[1].transform )

positionSoftBody1.setValue( Vec3f(0.2, 0.0, 0.0 ) )
scaleSoftBody1.setValue( Vec3f(0.003, 0.003, 0.003 ) )

positionSoftBody2.setValue( Vec3f(-0.12, 0.1, 0.0 ) )
scaleSoftBody2.setValue( Vec3f(0.003, 0.003, 0.003 ) )
