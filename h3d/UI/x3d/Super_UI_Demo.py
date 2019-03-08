# Import to access H3D types
from H3DInterface import*

# Field class that changes a float into a string.
class Fl2St( TypedField( MFString, SFFloat ) ):
  def update(self, event):
    routes_in = self.getRoutesIn()
    theFloat = "%.02f" % routes_in[0].getValue()
    return [theFloat]

PS_ss1_m = Fl2St()
PS_ss1_a = Fl2St()
PS_ss1_b = Fl2St()
PS_ss1_n1 = Fl2St()
PS_ss1_n2 = Fl2St()
PS_ss1_n3 = Fl2St()
PS_ss2_m = Fl2St()
PS_ss2_a = Fl2St()
PS_ss2_b = Fl2St()
PS_ss2_n1 = Fl2St()
PS_ss2_n2 = Fl2St()
PS_ss2_n3 = Fl2St()

# To set the text of the button that toggle of and on haptics.
class HapticsOnOff( TypedField( MFString, SFBool ) ):
  def update(self, event):
    if event.getValue():
      return ["Disable Supershape haptics"]
    else:
      return ["Enable Supershape haptics"]
      
hapticsOnOff = HapticsOnOff()

# Get the Frame node containing the slides.
sliderFrame, = references.getValue()

# Set the value of the sliders depending on which field gave a true event
class ChoosePreset( AutoUpdate( \
                    TypedField( SFBool, (SFBool, SFBool, SFBool ) ) ) ):
  def update(self, event):
    routes_in = self.getRoutesIn()
    if event.getValue():
      if routes_in[0] == event:
        theChildren = sliderFrame.children.getValue()
        theChildren[1].value.setValue( 2 )
        theChildren[4].value.setValue( 1 )
        theChildren[7].value.setValue( 1 )
        theChildren[10].value.setValue( 0.7 )
        theChildren[13].value.setValue( 0.3 )
        theChildren[16].value.setValue( 0.3 )
        theChildren[19].value.setValue( 3 )
        theChildren[22].value.setValue( 1 )
        theChildren[25].value.setValue( 1 )
        theChildren[28].value.setValue( 100 )
        theChildren[31].value.setValue( 100 )
        theChildren[34].value.setValue( 100 )
        return routes_in[0].getValue()
      if routes_in[1] == event:
        theChildren = sliderFrame.children.getValue()
        theChildren[1].value.setValue( 5 )
        theChildren[4].value.setValue( 1 )
        theChildren[7].value.setValue( 1 )
        theChildren[10].value.setValue( 0.1 )
        theChildren[13].value.setValue( 1.7 )
        theChildren[16].value.setValue( 1.7 )
        theChildren[19].value.setValue( 1 )
        theChildren[22].value.setValue( 1 )
        theChildren[25].value.setValue( 1 )
        theChildren[28].value.setValue( 0.3 )
        theChildren[31].value.setValue( 0.5 )
        theChildren[34].value.setValue( 0.5 )
        return routes_in[1].getValue()
      if routes_in[2] == event:
        theChildren = sliderFrame.children.getValue()
        theChildren[1].value.setValue( 4 )
        theChildren[4].value.setValue( 1 )
        theChildren[7].value.setValue( 1 )
        theChildren[10].value.setValue( 1 )
        theChildren[13].value.setValue( 1 )
        theChildren[16].value.setValue( 1 )
        theChildren[19].value.setValue( 4 )
        theChildren[22].value.setValue( 1 )
        theChildren[25].value.setValue( 1 )
        theChildren[28].value.setValue( 1 )
        theChildren[31].value.setValue( 1 )
        theChildren[34].value.setValue( 1 )
        return routes_in[2].getValue()
    return 0

choosePreset = ChoosePreset()

# If r is pressed the supershape node resets to default values.
class ResetShape( AutoUpdate( TypedField( SFBool, SFString ) ) ):
  def update( self, event):
    if event.getValue() == 'r':
        theChildren = sliderFrame.children.getValue()
        theChildren[1].value.setValue( 4 )
        theChildren[4].value.setValue( 1 )
        theChildren[7].value.setValue( 1 )
        theChildren[10].value.setValue( 100 )
        theChildren[13].value.setValue( 100 )
        theChildren[16].value.setValue( 100 )
        theChildren[19].value.setValue( 4 )
        theChildren[22].value.setValue( 1 )
        theChildren[25].value.setValue( 1 )
        theChildren[28].value.setValue( 100 )
        theChildren[31].value.setValue( 100 )
        theChildren[34].value.setValue( 100 )
    return 0
    
resetShape = ResetShape()

# Used to rotate the Supershape.
class RotateSS( TypedField( SFRotation, SFInt32 ) ):
  def __init__( self ):
    TypedField( SFRotation, SFInt32 ).__init__( self )
    self.rotate_value = Rotation()

  def update( self, event):    
    key = event.getValue()
    if key == 17:
      self.rotate_value = Rotation( 1,0,0,-0.1 ) * self.rotate_value
    elif key == 18:
      self.rotate_value = Rotation( 1,0,0,0.1 ) * self.rotate_value
    elif key == 19:
      self.rotate_value = Rotation( 0,1,0,-0.1 ) * self.rotate_value
    elif key == 20:
      self.rotate_value = Rotation( 0,1,0,0.1 ) * self.rotate_value
    return self.rotate_value

rotateSS = RotateSS()
