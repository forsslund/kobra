from H3DInterface import *

kernels = [ ("kernelSize 1, sigma 1.0, no blur", 1),
            ("kernelSize 3, sigma 1.0", 3 ),
            ("kernelSize 9, sigma 1.0", 9 ),
            ("kernelSize 19, sigma 1.0", 19 ), ]
current_kernel = 2

shader_node = getNamedNode( "SHADER" )
text_node = getNamedNode( "TEXT" )

class KeyHandler( AutoUpdate( SFString ) ):
  def update( self, event ):
    global current_kernel
    key = event.getValue()

    current_kernel = (current_kernel  + 1) % len( kernels ) 
    text_node.string.setValue( [kernels[current_kernel][0] ] )
    shader_node.kernelSize.setValue( kernels[current_kernel][1] ) 

    return key

keyhandler = KeyHandler()

keysensor = createNode( "KeySensor" )

keysensor.keyPress.route( keyhandler )
