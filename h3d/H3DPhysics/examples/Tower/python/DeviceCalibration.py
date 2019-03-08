from H3DInterface import *

class DeviceCalibration ( AutoUpdate ( SFBool ) ):
    """ Used to adjust the positionCalibration of the haptic device in order to
        align the haptic device with the viewpoint """

    def __init__ ( self, device, group ):
        AutoUpdate ( SFBool ).__init__ ( self )
        self.device= device
        self.group= group
        self.marker= None
        self.enabledXY= False
        self.enabledZ= False
        self.enabledRotation= False

    def update ( self, event ):
        if self.enabledXY or self.enabledZ or self.enabledRotation:
            if event.getValue():
                self.startCalibration()
            else:
                self.endCalibration()

        return event.getValue()

    def startCalibration ( self ):

        self.marker= createX3DNodeFromString ( """  <Transform>
                                                        <Shape>
                                                            <Appearance>
                                                                <Material diffuseColor='1 0 0' />
                                                            </Appearance>
                                                            <Sphere radius='0.01' />
                                                        </Shape>
                                                    </Transform>""" )[0]
        self.marker.translation.setValue ( self.device.trackerPosition.getValue() )

        # Is rotation calibration enabled?
        if self.enabledRotation:
          # Add rotation marker to calibration marker transform
          rotationMarker= createX3DNodeFromString ( """<Transform translation="0 0 0.08" rotation="1 0 0 1.570796" >
                                                          <Shape>
                                                            <Appearance>
                                                              <Material  diffuseColor='1 0 0' />
                                                            </Appearance>
                                                            <Cylinder radius="0.005" height="0.1"/>
                                                          </Shape>
                                                        </Transform>""" )[0]
          self.marker.children.push_back ( rotationMarker )
          # Mark the current tracker orientation
          self.marker.rotation.setValue ( self.device.trackerOrientation.getValue() )
        
        self.group.children.push_back ( self.marker )
        self.stylus= self.device.stylus.getValue ()
        self.device.stylus.setValue ( None )

        try:
          print "Start calibration of device: " + str(self.device.deviceName.getValue())
        except AttributeError:
          print "Start calibration of device: " + str ( self.device )

    def endCalibration ( self ):
        if self.marker:
            self.group.children.erase ( self.marker )
            self.device.stylus.setValue ( self.stylus )

            translation= self.marker.translation.getValue() - self.device.trackerPosition.getValue()

            viewplaneNormal= Matrix3f(getActiveViewpoint().orientation.getValue())*Vec3f ( 0, 0, 1 )
            viewplaneNormal.normalize()

            totalTranslation= Vec3f()

            if self.enabledZ:
                totalTranslation+= (viewplaneNormal.dotProduct(translation))*viewplaneNormal

            if self.enabledXY:
                totalTranslation+= translation - (translation.dotProduct(viewplaneNormal)*viewplaneNormal)

            # Is rotation calibration enabled?
            rotation= Quaternion()
            if self.enabledRotation:
              # Difference in rotation between starting orientation and current orientation
              initialOrientation= Quaternion ( self.marker.rotation.getValue() )
              currentOrientation= Quaternion ( self.device.trackerOrientation.getValue() )
              rotation= initialOrientation * currentOrientation.inverse()
                
            self.device.positionCalibration.setValue (
                self.device.positionCalibration.getValue() * Matrix4f ( rotation ) *
                    Matrix4f ( 1, 0, 0, totalTranslation.x,
                               0, 1, 0, totalTranslation.y,
                               0, 0, 1, totalTranslation.z,
                               0, 0, 0, 1 ) )

            try:
              print "End calibration of device: " + str(self.device.deviceName.getValue())
            except AttributeError:
              print "End calibration of device: " + str ( self.device )
            
            
            print str(self.device.positionCalibration.getValue())
