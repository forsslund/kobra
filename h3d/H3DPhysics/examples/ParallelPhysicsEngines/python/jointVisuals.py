from H3DInterface import *
import math

group= references.getValue()[0]

# Define properties shared across all joint visualisations

# Common appearance for connecting parts of joints
lineAppearance= createX3DNodeFromString ( """<Appearance>
                                                <Material emissiveColor='0 0 0' />
                                             </Appearance>""" )[0]

# Common appearance axes of joints
axisAppearance= createX3DNodeFromString ( """<Appearance>
                                                <Material diffuseColor='1 1 1' />
                                             </Appearance>""" )[0]

# Common appearance for main body of joints
jointAppearance= createX3DNodeFromString ( """<Appearance>
                                                <Material diffuseColor='1 0 0' />
                                              </Appearance>""" )[0]

# Common appearance for text displayed to label joints
textAppearance= createX3DNodeFromString ( """<Appearance>
                                                <Material diffuseColor='0 0 0' />
                                             </Appearance>""" )[0]

# Common font style for text displayed to label joints
fontStyle= createX3DNodeFromString ( """<FontStyle size="0.015" />""" )[0]

# Common appearance for details text displayed to label joints
textDetailAppearance= createX3DNodeFromString ( """<Appearance>
                                                <Material diffuseColor='0.3 0.3 0.3' />
                                             </Appearance>""" )[0]

# Common font style for details text displayed to label joints
fontDetailStyle= createX3DNodeFromString ( """<FontStyle size="0.013" />""" )[0]

class ToggleKey ( TypedField ( SFBool, SFString ) ):
    """ Field type used to toggle a boolean value based on key press"""
    
    def __init__ ( self, key ):
        TypedField ( SFBool, SFString ).__init__ ( self )
        self.value= False
        self.key= key
        
    def update ( self, event ):
        if event.getValue().lower() == self.key:
            self.value= not self.value
        return self.value

# A field used to toggle the display of joint details, i.e. field values
toggleDetails= ToggleKey ( "d" )

def angleBetweenVectors ( a, b ):
    """ Return angle in radians between a and b."""

    # Avoid errors due to accuracy of dot product
    # resulting in invalid input to acos
    dot= a.dotProduct ( b )
    if dot > 1:
        dot= 1
    if dot < -1:
        dot= -1
        
    return math.acos ( dot )

def getAnyPerp ( vector ):
    """ Get any vector perpendicular to specified vector."""
    other= Vec3f ( 1, 0, 0 )
    if other == vector or -other == vector:
        other= Vec3f ( 0, 1, 0 )
    perp= vector.crossProduct ( other )
    perp.normalizeSafe()

    return perp

def strVec3f ( v, places= 3 ):
    """ Limit the decimal places displayed in vector"""
    format= "(%%.%df, %%.%df, %%.%df)" % (places, places, places)
    return format % ( v.x, v.y, v.z )

class DrawLine ( TypedField ( MFVec3f, None, SFVec3f ) ):
    """ Field used to route to the line geometry. Creates a MFVec3f containing the points routed in."""
    def update ( self, event ):
        line= []
        for f in self.getRoutesIn():
            line.append ( f.getValue() )
        return line

class Joint ( object ):
    """Base class for joint visualisations"""

    class EnableDetailsTouch ( AutoUpdate ( TypedField ( SFBool, SFBool ) ) ):
        """ Field used to turn joint details display on/off"""
    
        def __init__ ( self, jointVisual ):
            AutoUpdate ( TypedField ( SFBool, SFBool ) ).__init__ ( self )
            self.jointVisual= jointVisual
            
        def update ( self, event ):
            if event.getValue():
                value= not self.jointVisual.detailsString
                if value:
                    self.jointVisual.showDetails()
                else:
                    self.jointVisual.hideDetails()
            return event.getValue()

    class EnableDetails ( AutoUpdate ( TypedField ( SFBool, SFBool ) ) ):
        """ Field used to turn joint details display on/off"""
    
        def __init__ ( self, jointVisual ):
            AutoUpdate ( TypedField ( SFBool, SFBool ) ).__init__ ( self )
            self.jointVisual= jointVisual
            
        def update ( self, event ):
            if event.getValue() and not self.jointVisual.detailsString:
                self.jointVisual.showDetails()
            elif not event.getValue() and self.jointVisual.detailsString:
                self.jointVisual.hideDetails()

            return event.getValue()

    class InitialRotation ( TypedField ( SFRotation, SFVec3f ) ):
        """ Field used to save the initial body orientation when axis is set."""
        def __init__ ( self, body ):
            TypedField ( SFRotation, SFVec3f ).__init__ ( self )
            self.body= body

        def update ( self, event ):
            return self.body.orientation.getValue ()

    class RelativeRotation ( TypedField ( SFRotation, (SFRotation,SFRotation) ) ):
        def update ( self, event ):
            initialRot, curRotation= self.getRoutesIn()
            return Rotation ( Matrix3f ( curRotation.getValue() ) * Matrix3f ( initialRot.getValue() ).inverse() )

    def __init__ ( self, joint ):
        self.enableDetails= Joint.EnableDetails ( self )
        self.enableDetailsTouch= Joint.EnableDetailsTouch ( self )

        self.group, self.dn= createX3DNodeFromString ( """<Group>
                                                            <Transform DEF='Title_T'>
                                                                <Billboard DEF='Detail_BB' axisOfRotation='0 0 0'>
                                                                    <Transform DEF='Detail_T' translation='0.025 -0.01 0.02'>
                                                                        <Shape DEF='TextDetail_S'>
                                                                            <Text DEF='TextDetail' />
                                                                        </Shape>
                                                                    </Transform>
                                                                </Billboard>
                                                            </Transform>
                                                            <Group DEF='JointGroup'>
                                                                <TouchSensor DEF='TS' />
                                                                <Transform DEF='AnchorPoint_T'>
                                                                    <Billboard axisOfRotation='0 0 0'>
                                                                        <Transform translation='0.01 0.01 0.02'>
                                                                            <Shape DEF='Text_S'>
                                                                                <Text DEF='Text' />
                                                                            </Shape>
                                                                        </Transform>
                                                                    </Billboard>
                                                                </Transform>
                                                            </Group>
                                                        </Group>""" )

        # Apply appearances etc.
        self.dn["Text"].fontStyle.setValue ( fontStyle )
        self.dn["Text_S"].appearance.setValue ( textAppearance )
        self.dn["TextDetail"].fontStyle.setValue ( fontDetailStyle )
        self.dn["TextDetail_S"].appearance.setValue ( textDetailAppearance )
        
        # Enable all output fields
        joint.forceOutput.setValue ( ["ALL"] )

        # Update the anchor point transform using an available body
        try:
            if joint.body1.getValue():
                joint.body1AnchorPoint.route ( self.dn["Title_T"].translation )
                joint.body1AnchorPoint.route ( self.dn["AnchorPoint_T"].translation )
            else:
                joint.body2AnchorPoint.route ( self.dn["Title_T"].translation )
                joint.body2AnchorPoint.route ( self.dn["AnchorPoint_T"].translation )
        except AttributeError:
            pass

        # Display the type name of the joint, and if it is fixed or not
        if joint.body1.getValue() and joint.body2.getValue():
            self.dn["Text"].string.push_back ( joint.getTypeName() )
        else:
            self.dn["Text"].string.push_back ( joint.getTypeName() + " (Fixed)" )

        self.joint= joint
        self.detailsString= None

        self.dn["Detail_BB"].children.erase ( self.dn["Detail_T"] )
        self.dn["TS"].isActive.route ( self.enableDetailsTouch )

    def showDetails ( self ):
        """ Show output fields.
            Subclasses override this to create self.detailsString, then call default."""
        if self.detailsString:
            self.dn["Detail_BB"].children.push_back ( self.dn["Detail_T"] )
            self.detailsString.route ( self.dn["TextDetail"].string )

    def hideDetails ( self ):
        """ Hide output fields"""
        self.dn["Detail_BB"].children.erase ( self.dn["Detail_T"] )
        
        self.detailsString= None
    
    def cleanUp( self ):
      self.enableDetails = None
      self.enableDetailsTouch = None

class BallJoint ( Joint ):
    """ A visual representation of a BallJoint"""

    class DetailsString ( TypedField ( MFString, (SFVec3f,SFVec3f) ) ):
        """ Field to convert output fields to strings"""
        def update ( self, event ):
            r= self.getRoutesIn()
            return [ "body1AnchorPoint: " + strVec3f(r[0].getValue()),
                     "body2AnchorPoint: " + strVec3f(r[1].getValue())]

    def __init__ ( self, joint ):
        super ( BallJoint, self ).__init__ ( joint )

        # Ball representing anchor point
        ball= createX3DNodeFromString ( """<Shape>
                                                <Sphere radius='0.008' />
                                            </Shape>""" )[0]
        ball.appearance.setValue ( jointAppearance )
        self.dn["AnchorPoint_T"].children.push_back ( ball )

        # Lines connecting bodies to anchor
        lines, dnLines= createX3DNodeFromString ( """<Group>
                                                        <Shape DEF='Line1_S'>
                                                            <LineSet DEF='Line1'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                        <Shape DEF='Line2_S'>
                                                            <LineSet DEF='Line2'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                    </Group>""" )
        self.dn["JointGroup"].children.push_back ( lines )

        # Apply appearances etc.
        ball.appearance.setValue ( jointAppearance )
        dnLines["Line1_S"].appearance.setValue ( lineAppearance )
        dnLines["Line2_S"].appearance.setValue ( lineAppearance )

        # Set up routes to draw line 1
        if joint.body1.getValue():
            self.drawLine1= DrawLine ()
            joint.body1AnchorPoint.route ( self.drawLine1 )
            joint.body1.getValue().position.route ( self.drawLine1 )
            self.drawLine1.route ( dnLines["Line1"].coord.getValue().point )
            dnLines["Line1"].vertexCount.push_back ( 2 )

        # Set up routes to draw line 2
        if joint.body2.getValue():
            self.drawLine2= DrawLine ()
            joint.body2AnchorPoint.route ( self.drawLine2 )
            joint.body2.getValue().position.route ( self.drawLine2 )
            self.drawLine2.route ( dnLines["Line2"].coord.getValue().point )
            dnLines["Line2"].vertexCount.push_back ( 2 )

    def showDetails ( self ):
        """ Show output field details"""
        self.detailsString= BallJoint.DetailsString ()
        self.joint.body1AnchorPoint.route ( self.detailsString )
        self.joint.body2AnchorPoint.route ( self.detailsString )
        super ( BallJoint, self ).showDetails ()

class AxisRotation ( TypedField ( SFRotation, SFVec3f ) ):

    def update ( self, event ):
        xAxis= event.getValue()
        xAxis.normalizeSafe()

        return Rotation ( Vec3f ( 0, 1, 0 ), xAxis )
        

class SingleAxisHingeJoint ( Joint ):

    class DetailsString ( TypedField ( MFString,
                                       (SFFloat,SFFloat,SFVec3f,SFVec3f) ) ):
        """ Field to convert output fields to strings"""
        def update ( self, event ):
            r= self.getRoutesIn()
            return [ "angle: %.3f" % r[0].getValue(),
                     "angleRate: %.3f" % r[1].getValue(),
                     "body1AnchorPoint: " + strVec3f(r[2].getValue()),
                     "body2AnchorPoint: " + strVec3f(r[3].getValue())]

    def __init__ ( self, joint ):
        super ( SingleAxisHingeJoint, self ).__init__ ( joint )

        hinge, dnHinge= createX3DNodeFromString ( """<Transform DEF='Hinge_T'>
                                                        <Transform DEF='HingeAngle_T'>
                                                            <Shape DEF='HingeBody_S'>
                                                                <Cylinder radius='0.008' height='0.01' />
                                                            </Shape>
                                                            <Shape DEF='HingeAxis_S'>
                                                                <Cylinder radius='0.002' height='0.03' />
                                                            </Shape>
                                                        </Transform>
                                                     </Transform>""" )
        dnHinge["HingeBody_S"].appearance.setValue ( jointAppearance )
        dnHinge["HingeAxis_S"].appearance.setValue ( axisAppearance )
        self.dn["AnchorPoint_T"].children.push_back ( hinge )

        lines, dnLines= createX3DNodeFromString ( """<Group>
                                                        <Shape DEF='Line1_S'>
                                                            <LineSet DEF='Line1'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                        <Shape DEF='Line1_S'>
                                                            <LineSet DEF='Line2'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                    </Group>""" )
        dnLines["Line1_S"].appearance.setValue ( lineAppearance )
        dnLines["Line1_S"].appearance.setValue ( lineAppearance )
        self.dn["JointGroup"].children.push_back ( lines )

        self.axisRotation= AxisRotation()
        joint.axis.route ( self.axisRotation )
        self.axisRotation.route ( dnHinge["HingeAngle_T"].rotation )

        if joint.body1.getValue():
            self.body1InitRot= Joint.InitialRotation ( joint.body1.getValue() )
            joint.axis.route ( self.body1InitRot )
            self.body1RelRot= Joint.RelativeRotation ()
            self.body1InitRot.route ( self.body1RelRot )
            joint.body1.getValue().orientation.route ( self.body1RelRot )
            self.body1RelRot.route ( dnHinge['Hinge_T'].rotation )
        else:
            self.body2InitRot= Joint.InitialRotation ( joint.body2.getValue() )
            joint.axis.route ( self.body2InitRot )
            self.body2RelRot= Joint.RelativeRotation ()
            self.body2InitRot.route ( self.body2RelRot )
            joint.body2.getValue().orientation.route ( self.body2RelRot )
            self.body2RelRot.route ( dnHinge['Hinge_T'].rotation )

        if joint.body1.getValue():
            self.drawLine1= DrawLine ()
            joint.body1AnchorPoint.route ( self.drawLine1 )
            joint.body1.getValue().position.route ( self.drawLine1 )
            self.drawLine1.route ( dnLines["Line1"].coord.getValue().point )
            dnLines["Line1"].vertexCount.push_back ( 2 )

        if joint.body2.getValue():
            self.drawLine2= DrawLine ()
            joint.body2AnchorPoint.route ( self.drawLine2 )
            joint.body2.getValue().position.route ( self.drawLine2 )
            self.drawLine2.route ( dnLines["Line2"].coord.getValue().point )
            dnLines["Line2"].vertexCount.push_back ( 2 )

    def showDetails ( self ):
        """ Show output field details"""
        self.detailsString= SingleAxisHingeJoint.DetailsString ()
        self.joint.angle.route ( self.detailsString )
        self.joint.angleRate.route ( self.detailsString )
        self.joint.body1AnchorPoint.route ( self.detailsString )
        self.joint.body2AnchorPoint.route ( self.detailsString )
        super ( SingleAxisHingeJoint, self ).showDetails ()

class DoubleAxisHingeJoint ( Joint ):

    class DetailsString ( TypedField ( MFString,
                                       (SFVec3f,SFVec3f,SFVec3f,SFVec3f,
                                        SFFloat,SFFloat,SFFloat,SFFloat) ) ):
        """ Field to convert output fields to strings"""
        def update ( self, event ):
            r= self.getRoutesIn()
            return [ "body1AnchorPoint: " + strVec3f(r[0].getValue()),
                     "body1Axis: "        + strVec3f(r[1].getValue()),
                     "body2AnchorPoint: " + strVec3f(r[2].getValue()),
                     "body2Axis: "        + strVec3f(r[3].getValue()),
                     "hinge1Angle: %.3f"      % r[4].getValue(),
                     "hinge1AngleRate: %.3f"  % r[5].getValue(),
                     "hinge2Angle: %.3f"      % r[6].getValue(),
                     "hinge2AngleRate: %.3f"  % r[7].getValue()]

    class FlipAxis ( TypedField ( SFVec3f, (SFVec3f,SFVec3f) ) ):

        def __init__ ( self, body, minAngle= math.pi/2 ):
            TypedField ( SFVec3f, (SFVec3f,SFVec3f) ).__init__ ( self )
            self.minAngle= minAngle
            self.body= body

        def update ( self, event ):
            axis, anchor= self.getRoutesIn()

            # Axis from body to anchor point
            bodyAxis= anchor.getValue()-self.body.position.getValue()
            bodyAxis= -self.body.orientation.getValue()*bodyAxis
            bodyAxis.normalizeSafe()
            
            a1= axis.getValue()
            a1.normalizeSafe()

            if angleBetweenVectors ( a1, bodyAxis ) < self.minAngle:
                return -a1
            else:
                return a1

    class AxisAttachPos ( TypedField ( SFVec3f, (SFVec3f,SFVec3f,SFRotation) ) ):

        def __init__ ( self, axisLength= 0.015 ):
            TypedField ( SFVec3f, (SFVec3f,SFVec3f,SFRotation) ).__init__ ( self )
            self.axisLength= axisLength

        def update ( self, event ):
            anchorPoint, axis, bodyRot= self.getRoutesIn()
            a= axis.getValue()
            a.normalizeSafe()
            return anchorPoint.getValue() + (bodyRot.getValue()*a)*self.axisLength

    class CenterAttachPos ( TypedField ( SFVec3f, (SFVec3f,SFVec3f,SFRotation) ) ):

        def __init__ ( self, distance= 0.03 ):
            TypedField ( SFVec3f, (SFVec3f,SFVec3f,SFRotation) ).__init__ ( self )
            self.distance= distance

        def update ( self, event ):
            anchorPoint, bodyPos, bodyRot= self.getRoutesIn()
            length= (anchorPoint.getValue() - bodyPos.getValue()).length()
            return anchorPoint.getValue() + self.distance * ((bodyPos.getValue()-anchorPoint.getValue()) / length)

    class BodyAxis ( SFVec3f ):

        def __init__ ( self, body ):
            SFVec3f.__init__ ( self )
            self.body= body
            
        def update ( self, event ):
            anchorPos= event.getValue()
            
            # Axis from body to anchor point
            bodyAxis= anchorPos-self.body.position.getValue()
            bodyAxis= -self.body.orientation.getValue()*bodyAxis
            bodyAxis.normalizeSafe()
            
            return bodyAxis

    class SideAttachPos ( TypedField ( SFVec3f, (SFVec3f,SFVec3f,SFVec3f,SFVec3f,SFRotation,SFVec3f) ) ):

        def __init__ ( self, distance= 0.015 ):
            TypedField ( SFVec3f, (SFVec3f,SFVec3f,SFVec3f,SFVec3f,SFRotation,SFVec3f) ).__init__ ( self )
            self.distance= distance
            self.directionSign= 0
            
        def update ( self, event ):
            startPos, axis, anchorPos, bodyPos, bodyRot, bodyAxis= self.getRoutesIn()

            a= axis.getValue()
            a.normalizeSafe()

            # Axis from body to anchor point
            bodyAxis= bodyAxis.getValue()

            # Get any vector perp to body axis
            angle= math.acos ( bodyAxis.dotProduct ( a ) )
            eps= 0.05
            if angle < eps or angle > math.pi-eps:
                perpBodyAxis= getAnyPerp ( a )
                perpBodyAxis.normalizeSafe()
            else:
                perpBodyAxis= bodyAxis.crossProduct ( a )
                perpBodyAxis.normalizeSafe()

            # Direction of line should be perp to perpBodyAxis and hinge axis
            direction= perpBodyAxis.crossProduct ( a )
            direction= bodyRot.getValue()*direction
            direction.normalizeSafe()

            # Choose direction once
            if self.directionSign == 0:
                # Direction should always be towards body
                if math.acos ( bodyAxis.dotProduct ( direction ) ) < math.pi/2:
                    self.directionSign= -1
                else:
                    self.directionSign= 1

            return startPos.getValue() + direction * self.distance * self.directionSign

    def __init__ ( self, joint ):

        super ( DoubleAxisHingeJoint, self ).__init__ ( joint )

        node, dnNode= createX3DNodeFromString ( """<Group>
                                                        <Transform DEF='Hinge1_T'>
                                                            <Transform translation='0 0 0'>
                                                                <Transform DEF='Hinge1Angle_T'>
                                                                    <Group DEF='HingeNodes'>
                                                                        <Transform translation='0 0.015 0'>
                                                                            <Shape DEF='Joint_S'>
                                                                                <Cylinder radius='0.004' height='0.01' />
                                                                            </Shape>
                                                                        </Transform>
                                                                        <Transform translation='0 0.0075 0'>
                                                                            <Shape DEF='Axis_S'>
                                                                                <Cylinder radius='0.002' height='0.015' />
                                                                            </Shape>
                                                                        </Transform>
                                                                    </Group>
                                                                </Transform>
                                                            </Transform>
                                                        </Transform>
                                                        <Shape DEF='AxisBall_S'>
                                                            <Sphere radius='0.002' />
                                                        </Shape>
                                                        <Transform DEF='Hinge2_T'>
                                                            <Transform translation='0 0 0'>
                                                                <Transform DEF='Hinge2Angle_T'>
                                                                    <Group USE='HingeNodes' />
                                                                </Transform>
                                                            </Transform>
                                                        </Transform>
                                                    </Group>""" )
        dnNode["Joint_S"].appearance.setValue ( jointAppearance )
        dnNode["Axis_S"].appearance.setValue ( axisAppearance )
        dnNode["AxisBall_S"].appearance.setValue ( axisAppearance )
        self.dn["AnchorPoint_T"].children.push_back ( node )
                                                                

        lines, dnLines= createX3DNodeFromString ( """<Group>
                                                        <Shape DEF='Line1_S'>
                                                            <LineSet DEF='Line1'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                        <Shape DEF='Line2_S'>
                                                            <LineSet DEF='Line2'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                    </Group>""" )
        dnLines["Line1_S"].appearance.setValue ( lineAppearance )
        dnLines["Line2_S"].appearance.setValue ( lineAppearance )
        self.dn["JointGroup"].children.push_back ( lines )

        self.axis1Displayed= DoubleAxisHingeJoint.FlipAxis ( joint.body1.getValue() )
        joint.axis1.route ( self.axis1Displayed )
        joint.anchorPoint.route ( self.axis1Displayed )
        
        self.axis2Displayed= DoubleAxisHingeJoint.FlipAxis( joint.body2.getValue() )
        joint.axis2.route ( self.axis2Displayed )
        joint.anchorPoint.route ( self.axis2Displayed )

        self.axis1Rotation= AxisRotation()
        self.axis1Displayed.route ( self.axis1Rotation )
        self.axis1Rotation.route ( dnNode["Hinge1Angle_T"].rotation )

        self.axis2Rotation= AxisRotation()
        self.axis2Displayed.route ( self.axis2Rotation )
        self.axis2Rotation.route ( dnNode["Hinge2Angle_T"].rotation )

        if joint.body1.getValue():

            self.body1InitRot= Joint.InitialRotation ( joint.body1.getValue() )
            joint.axis1.route ( self.body1InitRot )
            self.body1RelRot= Joint.RelativeRotation ()
            self.body1InitRot.route ( self.body1RelRot )
            joint.body1.getValue().orientation.route ( self.body1RelRot )
            self.body1RelRot.route ( dnNode['Hinge1_T'].rotation )

            self.axis1AttachPos= DoubleAxisHingeJoint.AxisAttachPos()
            joint.body1AnchorPoint.route ( self.axis1AttachPos )
            self.axis1Displayed.route ( self.axis1AttachPos )
            self.body1RelRot.route ( self.axis1AttachPos )

            self.axis1CenterAttachPos= DoubleAxisHingeJoint.CenterAttachPos()
            joint.body1AnchorPoint.route ( self.axis1CenterAttachPos )
            joint.body1.getValue().position.route ( self.axis1CenterAttachPos )
            self.body1RelRot.route ( self.axis1CenterAttachPos )

            self.bodyAxis1= DoubleAxisHingeJoint.BodyAxis ( joint.body1.getValue() )
            joint.anchorPoint.route ( self.bodyAxis1 )

            self.axis1SideAttachPos= DoubleAxisHingeJoint.SideAttachPos()
            self.axis1AttachPos.route ( self.axis1SideAttachPos )
            self.axis1Displayed.route ( self.axis1SideAttachPos )
            joint.body1AnchorPoint.route ( self.axis1SideAttachPos )
            joint.body1.getValue().position.route ( self.axis1SideAttachPos )
            self.body1RelRot.route ( self.axis1SideAttachPos )
            self.bodyAxis1.route ( self.axis1SideAttachPos )
            
            self.drawLine1= DrawLine ()
            self.axis1AttachPos.route ( self.drawLine1 )
            self.axis1SideAttachPos.route ( self.drawLine1 )
            self.axis1CenterAttachPos.route ( self.drawLine1 )
            joint.body1.getValue().position.route ( self.drawLine1 )
            self.drawLine1.route ( dnLines["Line1"].coord.getValue().point )
            dnLines["Line1"].vertexCount.push_back ( 4 )

        if joint.body2.getValue():

            self.body2InitRot= Joint.InitialRotation ( joint.body2.getValue() )
            joint.axis2.route ( self.body2InitRot )
            self.body2RelRot= Joint.RelativeRotation ()
            self.body2InitRot.route ( self.body2RelRot )
            joint.body2.getValue().orientation.route ( self.body2RelRot )
            self.body2RelRot.route ( dnNode['Hinge2_T'].rotation )

            self.axis2AttachPos= DoubleAxisHingeJoint.AxisAttachPos()
            joint.body2AnchorPoint.route ( self.axis2AttachPos )
            self.axis2Displayed.route ( self.axis2AttachPos )
            self.body2RelRot.route ( self.axis2AttachPos )

            self.axis2CenterAttachPos= DoubleAxisHingeJoint.CenterAttachPos()
            joint.body2AnchorPoint.route ( self.axis2CenterAttachPos )
            joint.body2.getValue().position.route ( self.axis2CenterAttachPos )
            self.body2RelRot.route ( self.axis2CenterAttachPos )

            self.bodyAxis2= DoubleAxisHingeJoint.BodyAxis ( joint.body2.getValue() )
            joint.anchorPoint.route ( self.bodyAxis2 )

            self.axis2SideAttachPos= DoubleAxisHingeJoint.SideAttachPos()
            self.axis2AttachPos.route ( self.axis2SideAttachPos )
            self.axis2Displayed.route ( self.axis2SideAttachPos )
            joint.body2AnchorPoint.route ( self.axis2SideAttachPos )
            joint.body2.getValue().position.route ( self.axis2SideAttachPos )
            self.body2RelRot.route ( self.axis2SideAttachPos )
            self.bodyAxis2.route ( self.axis2SideAttachPos )
            
            self.drawLine2= DrawLine ()
            self.axis2AttachPos.route ( self.drawLine2 )
            self.axis2SideAttachPos.route ( self.drawLine2 )
            self.axis2CenterAttachPos.route ( self.drawLine2 )
            joint.body2.getValue().position.route ( self.drawLine2 )
            self.drawLine2.route ( dnLines["Line2"].coord.getValue().point )
            dnLines["Line2"].vertexCount.push_back ( 4 )

    def showDetails ( self ):
        """ Show output field details"""
        self.detailsString= DoubleAxisHingeJoint.DetailsString ()
        self.joint.body1AnchorPoint.route ( self.detailsString )
        self.joint.body1Axis.route ( self.detailsString )
        self.joint.body2AnchorPoint.route ( self.detailsString )
        self.joint.body2Axis.route ( self.detailsString )
        self.joint.hinge1Angle.route ( self.detailsString )
        self.joint.hinge1AngleRate.route ( self.detailsString )
        self.joint.hinge2Angle.route ( self.detailsString )
        self.joint.hinge2AngleRate.route ( self.detailsString )
        super ( DoubleAxisHingeJoint, self ).showDetails ()

class UniversalJoint ( Joint ):

    class DetailsString ( TypedField ( MFString,
                                       (SFVec3f,SFVec3f,SFVec3f,SFVec3f) ) ):
        """ Field to convert output fields to strings"""
        def update ( self, event ):
            r= self.getRoutesIn()
            return [ "body1AnchorPoint: " + strVec3f(r[0].getValue()),
                     "body1Axis: "        + strVec3f(r[1].getValue()),
                     "body2AnchorPoint: " + strVec3f(r[2].getValue()),
                     "body2Axis: "        + strVec3f(r[3].getValue()) ]

    class DrawUniversalLine ( TypedField ( MFVec3f, ( SFVec3f, SFVec3f, SFVec3f ) ) ):

        def __init__ ( self, offset ):
            TypedField ( MFVec3f, ( SFVec3f, SFVec3f, SFVec3f ) ).__init__ ( self )
            self.offset= offset
        
        def update ( self, event ):
            bodyPos, hingeAxis, jointAnchor= self.getRoutesIn()

            start= bodyPos.getValue() + self.offset*hingeAxis.getValue()
            end= jointAnchor.getValue() + self.offset*hingeAxis.getValue()
            
            return [start, end]

    def __init__ ( self, joint ):

        super ( UniversalJoint, self ).__init__ ( joint )

        node, dnNode= createX3DNodeFromString ( """<Group>
                                                        <Transform DEF='HingeAngle1_T'>
                                                            <Shape DEF='Axis1_S'>
                                                                <Cylinder radius='0.002' height='0.03' />
                                                            </Shape>
                                                            <Transform translation='0 0.015 0'>
                                                                <Shape DEF='Joint1A_S'>
                                                                    <Cylinder DEF='JointCylinder' radius='0.005' height='0.005' />
                                                                </Shape>
                                                            </Transform>
                                                            <Transform translation='0 -0.015 0'>
                                                                <Shape DEF='Joint1B_S'>
                                                                    <Cylinder USE='JointCylinder' />
                                                                </Shape>
                                                            </Transform>
                                                        </Transform>
                                                        <Transform DEF='HingeAngle2_T'>
                                                            <Shape DEF='Axis2_S'>
                                                                <Cylinder radius='0.002' height='0.03' />
                                                            </Shape>
                                                            <Transform translation='0 0.015 0'>
                                                                <Shape DEF='Joint2A_S'>
                                                                    <Cylinder USE='JointCylinder' />
                                                                </Shape>
                                                            </Transform>
                                                            <Transform translation='0 -0.015 0'>
                                                                <Shape DEF='Joint2B_S'>
                                                                    <Cylinder USE='JointCylinder' />
                                                                </Shape>
                                                            </Transform>
                                                        </Transform>
                                                    </Group>""" )
        dnNode["Axis1_S"].appearance.setValue ( axisAppearance )
        dnNode["Axis2_S"].appearance.setValue ( axisAppearance )
        dnNode["Joint1A_S"].appearance.setValue ( jointAppearance )
        dnNode["Joint1B_S"].appearance.setValue ( jointAppearance )
        dnNode["Joint2A_S"].appearance.setValue ( jointAppearance )
        dnNode["Joint2B_S"].appearance.setValue ( jointAppearance )
        self.dn["AnchorPoint_T"].children.push_back ( node )

        lines, dnLines= createX3DNodeFromString ( """<Group>
                                                        <Shape DEF='Line1A_S'>
                                                            <LineSet DEF='Line1A'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                        <Shape DEF='Line2A_S'>
                                                            <LineSet DEF='Line2A'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                        <Shape DEF='Line1B_S'>
                                                            <LineSet DEF='Line1B'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                        <Shape DEF='Line2B_S'>
                                                            <LineSet DEF='Line2B'>
                                                                <Coordinate />
                                                            </LineSet>
                                                        </Shape>
                                                    </Group>""" )
        dnLines["Line1A_S"].appearance.setValue ( lineAppearance )
        dnLines["Line2A_S"].appearance.setValue ( lineAppearance )
        dnLines["Line1B_S"].appearance.setValue ( lineAppearance )
        dnLines["Line2B_S"].appearance.setValue ( lineAppearance )
        self.dn["JointGroup"].children.push_back ( lines )

        # Axes of rotation
        self.axisRotation1= AxisRotation()
        self.axisRotation1.route ( dnNode["HingeAngle1_T"].rotation )

        self.axisRotation2= AxisRotation()
        self.axisRotation2.route ( dnNode["HingeAngle2_T"].rotation )

        joint.body1Axis.route ( self.axisRotation1 )
        joint.body2Axis.route ( self.axisRotation2 )

        # Half the separation of the two lines representing the joint
        # i.e. offset from center
        offset= 0.015

        if joint.body1.getValue():
            self.drawLine1A= UniversalJoint.DrawUniversalLine (offset)

            joint.body1.getValue().position.route ( self.drawLine1A )
            joint.body1Axis.route ( self.drawLine1A )
            joint.body1AnchorPoint.route ( self.drawLine1A )
            
            self.drawLine1A.route ( dnLines["Line1A"].coord.getValue().point )
            dnLines["Line1A"].vertexCount.push_back ( 2 )

            self.drawLine2A= UniversalJoint.DrawUniversalLine (-offset)

            joint.body1.getValue().position.route ( self.drawLine2A )
            joint.body1Axis.route ( self.drawLine2A )
            joint.body1AnchorPoint.route ( self.drawLine2A )
            
            self.drawLine2A.route ( dnLines["Line2A"].coord.getValue().point )
            dnLines["Line2A"].vertexCount.push_back ( 2 )

        if joint.body2.getValue():
            self.drawLine1B= UniversalJoint.DrawUniversalLine (offset)

            joint.body2.getValue().position.route ( self.drawLine1B )
            joint.body2Axis.route ( self.drawLine1B )
            joint.body2AnchorPoint.route ( self.drawLine1B )
            
            self.drawLine1B.route ( dnLines["Line1B"].coord.getValue().point )
            dnLines["Line1B"].vertexCount.push_back ( 2 )

            self.drawLine2B= UniversalJoint.DrawUniversalLine (-offset)

            joint.body2.getValue().position.route ( self.drawLine2B )
            joint.body2Axis.route ( self.drawLine2B )
            joint.body2AnchorPoint.route ( self.drawLine2B )
            
            self.drawLine2B.route ( dnLines["Line2B"].coord.getValue().point )
            dnLines["Line2B"].vertexCount.push_back ( 2 )

    def showDetails ( self ):
        """ Show output field details"""
        self.detailsString= UniversalJoint.DetailsString ()
        self.joint.body1AnchorPoint.route ( self.detailsString )
        self.joint.body1Axis.route ( self.detailsString )
        self.joint.body2AnchorPoint.route ( self.detailsString )
        self.joint.body2Axis.route ( self.detailsString )
        super ( UniversalJoint, self ).showDetails ()

def closestPointOnLine ( pt, lnA, lnB ):
    """ Return the closest point to pt on the line lnA-lnB."""
    ln= lnB-lnA
    v= pt-lnA
    ln.normalizeSafe()
    
    return lnA + (ln.dotProduct ( v ) * ln)

class SliderJoint ( Joint ):

    class DetailsString ( TypedField ( MFString,
                                       (SFFloat,SFFloat) ) ):
        """ Field to convert output fields to strings"""
        def update ( self, event ):
            r= self.getRoutesIn()
            return [ "separation: %.3f" % r[0].getValue(),
                     "separationRate: %.3f" % r[1].getValue()]

    class AxisCenter ( TypedField ( SFVec3f, (SFVec3f,SFVec3f) ) ):
        def __init__ ( self, body1Pos, body2Pos, axisOffset ):
            TypedField ( SFVec3f, (SFVec3f,SFVec3f) ).__init__ ( self )
            self.body1Pos= body1Pos
            self.body2Pos= body2Pos
            self.axisOffset= axisOffset
            
        def update ( self, event ):
            axis, offset= self.getRoutesIn()
            return ((self.body1Pos.getValue()+self.body2Pos.getValue())*0.5 + self.axisOffset*offset.getValue())

    class SliderAxisPos ( TypedField ( SFVec3f, (SFVec3f,SFFloat,SFVec3f) ) ):

        def __init__ ( self, body1Pos ):
            TypedField ( SFVec3f, (SFVec3f,SFFloat,SFVec3f) ).__init__ ( self )
            self.body1Pos= body1Pos
            
        def update ( self, event ):
            axis, length, center= self.getRoutesIn()
            
            return center.getValue() - self.body1Pos.getValue() - (length.getValue()/2)*axis.getValue().normalizeSafe()    

    class AxisOffset ( SFVec3f ):
        
        def __init__ ( self, body1Pos, body2Pos ):
            SFVec3f.__init__ ( self )
            self.body1Pos= body1Pos
            self.body2Pos= body2Pos

        def update ( self, event ):
            jointAxis= event.getValue()

            # Calculate a suitable offset for the axis so that it will be visible
            jointAxis.normalizeSafe()

            viewOrientation= Rotation()
            vp= getActiveViewpoint()
            if vp:
                viewOrientation= vp.totalOrientation.getValue()

            viewVector= viewOrientation*Vec3f(0,0,-1)

            offsetVector= viewVector.crossProduct ( jointAxis )
            offsetVector.normalizeSafe()

            separation= self.body2Pos.getValue()-self.body1Pos.getValue()
            projection= offsetVector.dotProduct ( separation ) * offsetVector

            if projection.length() > 0.05:
                offsetVector= Vec3f ()

            return offsetVector

    class DrawLine ( TypedField ( MFVec3f, SFVec3f ) ):

        def __init__ ( self, bodyPos ):
            TypedField ( MFVec3f, SFVec3f ).__init__ ( self )
            self.bodyPos= bodyPos
        
        def update ( self, event ):
            center= event.getValue()
            return [ Vec3f(), center-self.bodyPos.getValue() ]

    class TextPos ( TypedField ( SFVec3f, (SFVec3f,SFVec3f) ) ):            
        def update ( self, event ):
            axisOffset, body1Pos= self.getRoutesIn()
            return axisOffset.getValue()+body1Pos.getValue()
        
    def __init__ ( self, joint, axisOffset= 0.05 ):

        super ( SliderJoint, self ).__init__ ( joint )

        node, dnNode= createX3DNodeFromString ( """<Group>
                                                        <Transform DEF='Body1_T'>
                                                            <Shape DEF='Line1_S'>
                                                                <LineSet DEF='Line1'>
                                                                    <Coordinate />
                                                                </LineSet>
                                                            </Shape>

                                                            <Transform DEF='AxisOffset_T'>
                                                                <Transform DEF='Axis_T'>
                                                                    <Shape DEF='Joint_S'>
                                                                        <Cylinder DEF='Axis_Cyl' radius='0.002' height='0.05'/>
                                                                    </Shape>
                                                                </Transform>
                                                            </Transform>
                                                            
                                                        </Transform>

                                                        <Transform DEF='Body2_T'>
                                                            <Shape DEF='Line2_S'>
                                                                <LineSet DEF='Line2'>
                                                                    <Coordinate />
                                                                </LineSet>
                                                            </Shape>
                                                        </Transform>
                                                    </Group>""" )
        dnNode["Line1_S"].appearance.setValue ( lineAppearance )
        dnNode["Line2_S"].appearance.setValue ( lineAppearance )
        dnNode["Joint_S"].appearance.setValue ( jointAppearance )
        self.dn["JointGroup"].children.push_back ( node )

        self.body1Pos= SFVec3f()
        if joint.body1.getValue():
            joint.body1.getValue().position.route ( self.body1Pos )
        else:
            self.body1Pos.setValue ( joint.body2.getValue().position.getValue() )

        self.body2Pos= SFVec3f()
        if joint.body2.getValue():
            joint.body2.getValue().position.route ( self.body2Pos )
        else:
            self.body2Pos.setValue ( joint.body1.getValue().position.getValue() )
        
        self.body1Rot= SFRotation()
        
        if joint.body1.getValue():
            self.body1InitRot= Joint.InitialRotation ( joint.body1.getValue() )
            joint.axis.route ( self.body1InitRot )
            self.body1RelRot= Joint.RelativeRotation ()
            self.body1InitRot.route ( self.body1RelRot )
            joint.body1.getValue().orientation.route ( self.body1RelRot )
            self.body1RelRot.route ( self.body1Rot )

        self.body2Rot= SFRotation()
        
        if joint.body2.getValue():
            self.body2InitRot= Joint.InitialRotation ( joint.body2.getValue() )
            joint.axis.route ( self.body2InitRot )
            self.body2RelRot= Joint.RelativeRotation ()
            self.body2InitRot.route ( self.body2RelRot )
            joint.body2.getValue().orientation.route ( self.body2RelRot )
            self.body2RelRot.route ( self.body2Rot )

        self.axisRotation= AxisRotation()
        joint.axis.route ( self.axisRotation )
        self.axisRotation.route ( dnNode["Axis_T"].rotation )

        self.body1Pos.route ( dnNode["Body1_T"].translation )
        self.body1Rot.route ( dnNode["Body1_T"].rotation )
        self.body2Pos.route ( dnNode["Body2_T"].translation )
        self.body2Rot.route ( dnNode["Body2_T"].rotation )

        self.offsetVector= SliderJoint.AxisOffset(self.body1Pos, self.body2Pos)
        joint.axis.route ( self.offsetVector )

        self.axisCenter= SliderJoint.AxisCenter (self.body1Pos, self.body2Pos, axisOffset )
        joint.axis.route ( self.axisCenter )
        self.offsetVector.route ( self.axisCenter )

        self.axisPos= SliderJoint.SliderAxisPos (self.body1Pos)
        joint.axis.route ( self.axisPos )
        joint.maxSeparation.route ( self.axisPos )
        self.axisCenter.route ( self.axisPos )
        self.axisPos.route ( dnNode["AxisOffset_T"].translation )

        # Customise position of text
        self.textPos= SliderJoint.TextPos()
        self.axisPos.route ( self.textPos )
        self.body1Pos.route ( self.textPos )
        self.textPos.route ( self.dn["AnchorPoint_T"].translation )
        self.textPos.route ( self.dn["Title_T"].translation )

        joint.maxSeparation.route ( dnNode["Axis_Cyl"].height )

        self.drawLine1= SliderJoint.DrawLine (self.body1Pos)
        self.axisCenter.route ( self.drawLine1 )
        self.drawLine1.route ( dnNode["Line1"].coord.getValue().point )
        dnNode["Line1"].vertexCount.push_back ( 2 )

        self.drawLine2= SliderJoint.DrawLine (self.body2Pos)
        self.axisCenter.route ( self.drawLine2 )
        self.drawLine2.route ( dnNode["Line2"].coord.getValue().point )
        dnNode["Line2"].vertexCount.push_back ( 2 )

    def showDetails ( self ):
        """ Show output field details"""
        self.detailsString= SliderJoint.DetailsString ()
        self.joint.separation.route ( self.detailsString )
        self.joint.separationRate.route ( self.detailsString )
        super ( SliderJoint, self ).showDetails ()

sliderOffset= 0.05

class UpdateJoints ( AutoUpdate ( MFNode ) ):

    def __init__ ( self ):
        AutoUpdate ( MFNode ).__init__ ( self )
        self.joints= []
        self.sliderOffsetDirection= {}
        self.inited= False
    
    def __del__(self):
      for j in self.joints:
        j.cleanUp()

    def getSliderOffsetDirection ( self, slider ):
        if not self.inited:
            for i in range(len(references.getValue())):
                r= references.getValue()[i]
                if r.getTypeName()=="MetadataString" and r.name.getValue()=="SliderOffsetDirection":
                    self.sliderOffsetDirection[str(references.getValue()[i+1])]= r.value.getValue()[0]
            self.inited= True

        if str(slider) in self.sliderOffsetDirection:
            if self.sliderOffsetDirection[str(slider)] == "Negative":
                return -1

        return 1

    def update ( self, event ):

        # Remove previous joints
        for j in self.joints:
            group.children.erase ( j.group )
            j.cleanUp()
        self.joints= []

        # Create new joints
        joints= event.getValue()
        for j in joints:
            joint= None
            
            if j.getTypeName()=="BallJoint":
                joint= BallJoint( j )
            if j.getTypeName()=="SingleAxisHingeJoint":
                joint= SingleAxisHingeJoint( j )
            if j.getTypeName()=="DoubleAxisHingeJoint":
                joint= DoubleAxisHingeJoint( j )
            if j.getTypeName()=="UniversalJoint":
                joint= UniversalJoint( j )
            if j.getTypeName()=="SliderJoint":
                joint= SliderJoint( j, self.getSliderOffsetDirection(j)*sliderOffset )

            if joint:
                group.children.push_back ( joint.group )
                toggleDetails.route ( joint.enableDetails )
                self.joints.append ( joint )
                

        return joints

joints= UpdateJoints ()

# Position of text messages
class TextPos( AutoUpdate(TypedField(SFVec3f, (SFVec3f, SFRotation ) ) ) ):

  def update( self, event ):
    routes_in = self.getRoutesIn();
    v = routes_in[0].getValue()
    r = Rotation()
    if len( routes_in ) > 1:
      r = routes_in[1].getValue()
    v_temp = Vec3f( v.x, v.y, v.z )
    stereo_info = getActiveStereoInfo()
    focal_distance = 0.6
    if stereo_info:
      focal_distance = stereo_info.focalDistance.getValue()
    v_new = v - r * Vec3f( 0, 0, focal_distance )
    return v_new

textPos = TextPos()
