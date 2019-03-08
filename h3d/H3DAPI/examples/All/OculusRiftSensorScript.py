from H3DInterface import *
from H3DUtils import *
import math

window= getCurrentScenes()[0].window.getValue()[0]
window.renderMode.setValue( "OCULUS_RIFT" )

description_node = references.getValue()[0]

class FireGun( AutoUpdate( TypedField( SFFloat, SFFloat ) ) ):
  def __init__( self ):
    super(FireGun, self).__init__()

  def update( self, event ):
    trigger_push = self.getRoutesIn()[0].getValue()
    return trigger_push


class ToggleColorLeft( AutoUpdate( TypedField( SFColor, SFInt32 ) ) ):
  def __init__( self ):
    super(ToggleColorLeft, self).__init__()

  def update( self, event ):
    button_mask_X = 256
    button_mask_Y = 512
    button_mask = self.getRoutesIn()[0].getValue()

    red = RGB(1.0, 0.2, 0.2)
    green = RGB(0.2, 1.0, 0.2)
    blue = RGB(0.2, 0.2, 1.0)
    
    if button_mask & button_mask_X == button_mask_X:
        return blue

    if button_mask & button_mask_Y == button_mask_Y:
        return green

    return red


class ToggleColorRight( AutoUpdate( TypedField( SFColor, SFInt32 ) ) ):
  def __init__( self ):
    super(ToggleColorRight, self).__init__()

  def update( self, event ):
    button_mask_A = 1
    button_mask_B = 2
    button_mask = self.getRoutesIn()[0].getValue()

    red = RGB(1.0, 0.2, 0.2)
    green = RGB(0.2, 1.0, 0.2)
    blue = RGB(0.2, 0.2, 1.0)

    if button_mask & button_mask_A == button_mask_A:
        return blue

    if button_mask & button_mask_B == button_mask_B:
        return green

    return red


class ToggleDescription( AutoUpdate( TypedField(SFBool, SFInt32) ) ):
  def __init__(self):
    super(ToggleDescription, self).__init__()
    self.pressing_button = False

  def update(self, event):
    global description_node
    button_is_pressed = self.getRoutesIn()[0].getValue() & 1024 == 1024

    if button_is_pressed and self.pressing_button == False:
      currently_enabled = description_node.graphicsOn.getValue()
      description_node.graphicsOn.setValue( not currently_enabled )
      self.pressing_button = True
    elif not button_is_pressed:
      self.pressing_button = False

    return True


left_fire_gun = FireGun()
right_fire_gun = FireGun()

left_color_toggler = ToggleColorLeft()
right_color_toggler = ToggleColorRight()

description_toggle = ToggleDescription()