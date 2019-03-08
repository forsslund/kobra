from H3DInterface import *

shader= getNamedNode ( 'SHADER' )
ks= getNamedNode ( 'KS' )
displayModeText= getNamedNode ( 'DISPLAY_MODE_TEXT' )

softBodyAtribute= getNamedNode ( 'SB_ATTRIB' )
vertexAtribute= getNamedNode ( 'VERTEX_ATTRIB' )
softBodyAtributeIndex= getNamedNode ( 'SB_ATTRIB_INDEX' )

forceScale= 100.0

displayModes= [
  None,
  ["OUTPUT_INTERACTION_FORCE", "UNIT_NODE"],
  ["OUTPUT_EXTERNAL_FORCE", "UNIT_NODE"],
  ["OUTPUT_FORCE", "UNIT_NODE"],
  ["OUTPUT_VELOCITY", "UNIT_NODE"],
]

class KeyHandler ( AutoUpdate ( SFString ) ):
  def update ( self, value ):
    key= value.getValue().lower()
    
    if key == 's':
      # Switch display mode
      mode= shader.displayMode.getValue()+1
      if mode >= len(displayModes):
        mode= 0
      shader.displayMode.setValue ( mode )
      if displayModes[mode]:
        softBodyAtribute.name.setValue ( displayModes[mode][0] )
        softBodyAtribute.unitType.setValue ( displayModes[mode][1] )
        displayModeText.string.setValue ( [ "Now showing: " + displayModes[mode][0] + " per " + displayModes[mode][1], "Press s to cycle" ] )
      else:
        displayModeText.string.setValue ( [ "Now showing: No attributes", "Press s to cycle" ] )
      
    return key
        
class MFVec3fToMFFloat ( TypedField ( MFFloat, MFVec3f ) ):
  
  def update ( self, event ):
    floats= []
    for v in event.getValue():
      floats.extend ( [v.x, v.y, v.z] )
    return floats
    
class MFVec3fToMFString ( TypedField ( MFString, MFVec3f ) ):

  def update ( self, event ):
    message= "Interaction forces on specific vertices: "
    index= softBodyAtributeIndex.index.getValue()
    for i, v in enumerate(event.getValue()):
      v= v*forceScale
      message= message + "%d: (%.2f,%.2f,%.2f) " % (index[i], v.x, v.y, v.z)
    return [message]
        
keyHandler= KeyHandler()
ks.keyPress.route ( keyHandler )

indexedVertsString= MFVec3fToMFString ()
softBodyAtributeIndex.value.route ( indexedVertsString )
indexedVertsString.route ( getNamedNode ( 'INDEXED_VERTS_MESSAGE' ).string )

convertToFloats= MFVec3fToMFFloat()
softBodyAtribute.value.route ( convertToFloats )
convertToFloats.route ( vertexAtribute.value )