from H3DInterface import *
import H3DUtils

sliderJoint, axis, period, speed= references.getValue()

slideTime=  float(period.value.getValue()[0])
slideSpeed= float(speed.value.getValue()[0])

timerCallbacks= H3DUtils.TimerCallback()

def setVelocity ( v ):
  c= axis.value.getValue()[0]
  if c == 'x':
    sliderJoint.desiredLinearVelocity1.setValue ( v )
  elif c == 'y':
    sliderJoint.desiredLinearVelocity2.setValue ( v )
  elif c == 'z':
    sliderJoint.desiredLinearVelocity3.setValue ( v )

def slideRight ():
  setVelocity ( slideSpeed )
  timerCallbacks.addCallback ( time.getValue()+slideTime, slideLeft, [] )
  
def slideLeft ():
  setVelocity ( -slideSpeed )
  timerCallbacks.addCallback ( time.getValue()+slideTime, slideRight, [] )
  
slideRight()