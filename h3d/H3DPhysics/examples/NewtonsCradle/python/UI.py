from H3DInterface import *

class FormatToString ( TypedField ( MFString, SFFloat ) ):
  """A field type which formats an SFFloat to an MFString containing
     a single string containing the number formated according to the specified
     format string"""
  
  def __init__ ( self, format= "%.2f" ):
    TypedField ( MFString, SFFloat ).__init__ ( self )
    self.format= format
  
  def update ( self, event ):
    return [self.format % event.getValue()]

# Fields to convert slider values to string for the
# associated labels in the UI
bounce= FormatToString()
damping= FormatToString()
mass= FormatToString()
ballCount= FormatToString("%d")
ballRadius= FormatToString("%.4f")
stringLength= FormatToString()
