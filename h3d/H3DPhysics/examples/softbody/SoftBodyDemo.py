from H3DInterface import *
import Grabber

tetraToggle,bodyMat,ks,sbc,root= references.getValue()
  
# Hide and show the mesh model
class KeyHandler ( AutoUpdate ( SFString ) ):
  def update ( self, value ):
    key= value.getValue().lower()
    
    if key == 'm':
      if tetraToggle.graphicsOn.getValue():
        tetraToggle.graphicsOn.setValue ( False )
        bodyMat.transparency.setValue ( 0.0 )
      else:
        tetraToggle.graphicsOn.setValue ( True )
        bodyMat.transparency.setValue ( 0.5 )

    return key
        
keyHandler= KeyHandler()
ks.keyPress.route ( keyHandler )

Grabber.addGrabbersForSoftBodies ( sbc, root, 10.0 )
