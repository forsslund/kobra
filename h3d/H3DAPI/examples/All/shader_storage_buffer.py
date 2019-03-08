from H3DInterface import *

shader_node = getNamedNode( "SHADER" )
shader_node_read = getNamedNode( "SHADER_READ" )
shader_node_clear = getNamedNode( "SHADER_CLEAR" )
ssbo_node = getNamedNode( "TEXTURE_SSBO" )
scene = getCurrentScenes()[0]
window = scene.window.getValue()[0]

window.width.route(shader_node.width)
window.width.route(shader_node_read.width)
window.width.route(shader_node_clear.width)

window.width.route(ssbo_node.width)
window.height.route(ssbo_node.height)

