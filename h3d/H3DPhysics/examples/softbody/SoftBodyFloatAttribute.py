from H3DInterface import *

shader= getNamedNode ( 'SHADER' )
ks= getNamedNode ( 'KS' )
displayModeText= getNamedNode ( 'DISPLAY_MODE_TEXT' )

softBodyAtribute= getNamedNode ( 'SB_ATTRIB' )
softBodyAtributeIndex= getNamedNode ( 'SB_ATTRIB_INDEX' )

forceScale= 100.0

displayModes= [
  None,
  ["OUTPUT_INTERACTION_FORCE_MAGNITUDE", "UNIT_NODE"],
  ["OUTPUT_EXTERNAL_FORCE_MAGNITUDE", "UNIT_NODE"],
  ["OUTPUT_FORCE_MAGNITUDE", "UNIT_NODE"],
  ["OUTPUT_SPEED", "UNIT_NODE"],
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
    
class MFVec3fToMFFloat ( TypedField ( MFString, MFFloat ) ):

  def update ( self, event ):
    message= "Interaction forces on specific vertices: "
    index= softBodyAtributeIndex.index.getValue()
    for i, f in enumerate(event.getValue()):
      f= f*forceScale
      message= message + "%d: (%.2f) " % (index[i], f)
    return [message]
        
keyHandler= KeyHandler()
ks.keyPress.route ( keyHandler )

indexedVertsString= MFVec3fToMFFloat ()
softBodyAtributeIndex.value.route ( indexedVertsString )
indexedVertsString.route ( getNamedNode ( 'INDEXED_VERTS_MESSAGE' ).string )