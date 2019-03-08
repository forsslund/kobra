# Import to access H3D types
from H3DInterface import*

# Field class that changes a float into a string.
class Fl2St( TypedField( MFString, SFFloat ) ):
  def update(self, event):
    routes_in = self.getRoutesIn()
    theFloat = "%.02f" % routes_in[0].getValue()
    return [theFloat]

PS_t = Fl2St()