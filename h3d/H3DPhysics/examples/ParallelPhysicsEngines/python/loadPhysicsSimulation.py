# load physics simulation part as node into the scene
# Requirements for the physics scene
# 1. rigidbody node with position must be specified
# 2. It is better to not set the viewport and other scene related parameters

from H3DInterface import *
from xml.dom import minidom
import os


# get references to nodes created in the x3d file
physicsGroupNodeAttachementPoint, = references.getValue()

# default physics scene file
rootDir = os.path.dirname(resolveURLAsFile("ParallelPhysicsEngines.x3d"))

zShift = 0.4


# engines specify the physics engine for each scene
# physicsScriptStr specify the x3d from which the physics node group will be created.
# posZ means the shift value in direction for every scene
engines =MFString()
# set default engines value to ODE and Bullet
engines.setValue(['ODE','Bullet'])

physicsScriptFileName = SFString()
physicsScriptFileName.setValue("./x3d/seeSaw.x3d")

constantForceMixs = MFFloat()
constantForceMixDefault = 0.00001
constantForceMixs.setValue([constantForceMixDefault,constantForceMixDefault])
errorCorrections = MFFloat()
errorCorrectionDefault = 0.2
errorCorrections.setValue([errorCorrectionDefault,errorCorrectionDefault])
iterations = MFInt32()
iterationsDefault = 10
iterations.setValue([iterationsDefault,iterationsDefault])
maxCorrectionSpeeds = MFFloat()
maxCorrectionSpeedDefault = -1
maxCorrectionSpeeds.setValue([maxCorrectionSpeedDefault,maxCorrectionSpeedDefault])

rigidBodyCollectionFields = ['physicsEngine','constantForceMix','errorCorrection','iterations','maxCorrectionSpeed']


def loadPhysicsX3DFile(physicsScriptFileName):
    parsedXMLString = minidom.parse(os.path.join(rootDir,physicsScriptFileName.getValue()))
    return parsedXMLString

def replaceJointsAnchorPosition(physicsX3DFileString,possibleJointNames,posZ):
    x3dFileString = physicsX3DFileString
    for jointName in possibleJointNames:
        joints = x3dFileString.getElementsByTagName(jointName)
        if joints:
            # joints is not empty
            for joint in joints:
                anchorPositionOrigin = joint.getAttribute('anchorPoint')
                anchorPositionOriginArray = anchorPositionOrigin.encode('utf8').split()
                anchorPositionOriginArray[2] = str(posZ)
                anchorPositionNew = anchorPositionOriginArray[0]+' '+anchorPositionOriginArray[1]+' '+anchorPositionOriginArray[2]
                joint.setAttribute('anchorPoint',anchorPositionNew)
        joints = physicsX3DFileString.getElementsByTagName('SingleAxisHingeJoint')
    return x3dFileString
        
def specifyParameters(physicsX3DFileString,engine,constantForceMix,errorCorrection,iteration,maxCorrectionSpeed,posZ):

    # 1. change the engine related fields in RigidBodyCollection, if it exists
    speicfiedRBCFValue = [engine,constantForceMix,errorCorrection,iteration,maxCorrectionSpeed]
    rigidBodyCollections = physicsX3DFileString.getElementsByTagName('RigidBodyCollection')
    if rigidBodyCollections :
        for rbc in rigidBodyCollections:
        # for every rigidbody collection node, to edit the specified fields
            for idx, rbcf in enumerate(rigidBodyCollectionFields):
            # for every rigid body collection field specified
                if not( rbc.hasAttribute('containerField')):
                # if current rigid body collection is not used as input to other node such as python script node
                    rbc.setAttribute(rbcf,str(speicfiedRBCFValue[idx]))
                else:
                    print('current rbc have no corresponding attributes')
    
    # 3. specify the z axis shift for every RigidBody position
    rigidBodies = physicsX3DFileString.getElementsByTagName('RigidBody')
    for rb in rigidBodies:
        if rb.hasAttribute('position'):
            # do string operation to change the z component of position field of every rigid body
            positionOrigin = rb.getAttribute('position')
            positionOriginArray = positionOrigin.encode('utf8').split()
            positionOriginArray[2] = str(posZ)
            positionNew = positionOriginArray[0]+' '+positionOriginArray[1]+' '+positionOriginArray[2]
            rb.setAttribute('position',positionNew)
    # 4. specify the z axis shift for every necessary joint anchorPoint if these joints exist
    possibleJointNames = ['BallJoint','SingleAxisHingeJoint']
    physicsX3DFileString = replaceJointsAnchorPosition(physicsX3DFileString, possibleJointNames,posZ)
    
    # 5. handle the url issue in h3dviewer
    nodesContainUrl = ['PythonScript','ImageTexture','ShaderPart']
    for ncu in nodesContainUrl:
        nodes = physicsX3DFileString.getElementsByTagName(ncu)
        for n in nodes:
            if n.hasAttribute('url'):
                urlOrigin = n.getAttribute('url')
                urlNew = os.path.join(rootDir,os.path.normpath(urlOrigin))
                n.setAttribute('url',urlNew)
    return physicsX3DFileString

    
def newPhysicsGroupNode():
    # it represent the physicsGroupNode
    # create a new physics nodes group based on certain global field definition
    # for each physics engine in the engines vector, create a new physics scene
    # edit corresponding settings
    # adding it to the attach place

    physicsGroupNodeAttachementPoint.children.clear()

    # decide the z axis position of every physics scene to separate them in the final result
    enginesNum = len(engines.getValue())
    posZs = None
    if enginesNum and enginesNum%2 == 0:
        # if engines number is not empty and is even
        posZs =[zShift/2]*enginesNum
        for idx, num in enumerate(posZs):
            posZs[idx]= num*((idx*2)-(enginesNum-1))
    if enginesNum and enginesNum%2 is not 0:
        # if engines number is not empty and is odd
        posZs =[zShift]*enginesNum
        for idx, num in enumerate(posZs):
            posZs[idx]= num*(idx-(enginesNum-1)/2)
    # update engine parameters in case that not enough engine parameters are set
    engineParameters = [constantForceMixs,errorCorrections,iterations,maxCorrectionSpeeds]
    engineParametersDefault = [constantForceMixDefault,errorCorrectionDefault,iterationsDefault,maxCorrectionSpeedDefault]
    # add or remove extra parameters if not enough or too many paramters exist for engine
    for idx, engineParameter in enumerate(engineParameters):
        if len(engineParameter.getValue())<len(engines.getValue()):
            # if not enough parameters are exist for specified engine
            additionalPart = [engineParametersDefault[idx]]*(len(engines.getValue())-len(engineParameter.getValue()))
            newParameter= engineParameter.getValue() + additionalPart
            engineParameter.setValue(newParameter)
        elif len(engineParameter.getValue())>len(engines.getValue()):
            # engines has decreased, too many engine parameters exist
            newParameter = engineParameter.getValue()
            del newParameter[len(engines.getValue())-1:-1]
            engineParameter.setValue(newParameter)
            
    for idx, engine in enumerate(engines.getValue()):
        if engine=="ODE" or "PhysX" or "Bullet":
            physicsX3DFileString = loadPhysicsX3DFile(physicsScriptFileName)
            # handle the specified parameters
            
            xmldoc = specifyParameters(physicsX3DFileString,engine,constantForceMixs.getValue()[idx],errorCorrections.getValue()[idx],iterations.getValue()[idx],maxCorrectionSpeeds.getValue()[idx],posZs[idx])

            node, dn = createX3DFromString(xmldoc.toxml('utf-8'))
            physicsGroupNodeAttachementPoint.children.push_back(node)
        else:
            print("please specify a supported engine. SOFA is not accepted.") 


# updatePhysicsGroup is used to react to the user key event, engine name changing event and physics scene file changing event
class updatePhysicsGroup(AutoUpdate(TypedField(SFBool,(MFString,SFString,MFFloat,MFFloat,MFInt32,MFFloat,SFString)))):
    # 
    def update(self,event):
        # get all the route in fields
        routeIns = self.getRoutesIn()
        if len(routeIns) == 7:
            # all three fields are routed in
            if event == routeIns[6]:
                # if the event is the user input
                if event.getValue()=='l' or event.getValue()=='L':
                    newPhysicsGroupNode()
            else:
            # if event is other two filed change,directly recreate the physics nodes group
                newPhysicsGroupNode()
        return True

updatePhysicsGroup = updatePhysicsGroup()

engines.route(updatePhysicsGroup)
physicsScriptFileName.route(updatePhysicsGroup)
constantForceMixs.route(updatePhysicsGroup)
errorCorrections.route(updatePhysicsGroup)
iterations.route(updatePhysicsGroup)
maxCorrectionSpeeds.route(updatePhysicsGroup)

# add programSetting to the H3DViewer gui addProgramSetting(field,setting_name,section_name)
addProgramSetting(physicsScriptFileName,'physics scene file','physics scene x3d')
addProgramSetting(engines,'engine name','engine names')

addProgramSetting(constantForceMixs,'constant force mix','engine parameters')
addProgramSetting(errorCorrections,'error correction','engine parameters')
addProgramSetting(iterations,'iterations','engine parameters')
addProgramSetting(maxCorrectionSpeeds,'max correction speed','engine parameters')

