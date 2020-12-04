# Toggle RigidBodyPhysics (keypress: 'g')
from H3DInterface import *

rbc, = references.getValue()
class ToggleRBP(AutoUpdate(SFString)):
  def update(self, event):
    global rbc
    if event.getValue() == 'g' or event.getValue() == 'G':
      val = 1 - rbc.enabled.getValue()
      print('RBP toggled: ' + str(val))
      rbc.enabled.setValue(val)
    return ''
toggleRBP = ToggleRBP()