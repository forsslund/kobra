from H3DInterface import *

def PrintFieldValue( base_class ):
  class PrintValueClass( AutoUpdate( base_class ) ):
    def update( self, event ):
      v = event.getValue()
      print(v)
      return v
  return PrintValueClass()

def FieldValue2String( base_class ):
  class Value2StringClass( TypedField( SFString, base_class ) ):
    def update( self, event ):
      v = event.getValue()
      return str( v )
  return Value2StringClass()

def FieldValue2StringList( base_class ):
  class Value2StringListClass( TypedField( MFString, base_class ) ):
    def update( self, event ):
      v = event.getValue()
      return [str( v )]
  return Value2StringListClass()

def FieldValue2Int( base_class ):
  class Value2IntClass( TypedField( SFInt32, base_class ) ):
    def update( self, event ):
      v = event.getValue()
      return int( v )
  return Value2IntClass()

def SField2MField( sfield, mfield ):
  valid_sfield = False
  for t in sfield_types:
    if sfield.type == t[0]:
      valid_sfield = True
      break
  if not valid_sfield:
    raise Exception( "Warning: Could not create SField2MField. Invalid sfield type " + sfield.__name__ + " used." )
  valid_mfield = False
  for t in mfield_types:
    if mfield.type == t[0]:
      valid_mfield = True
      break
  if not valid_mfield:
    raise Exception( "Warning: Could not create SField2MField. Invalid mfield type " + mfield.__name__ + " used." )
  class SField2MFieldClass( TypedField( mfield, sfield ) ):
    def update( self, event ):
      v = event.getValue()
      return [v]
  return SField2MFieldClass()

## The TimerCallback field is a field in which you can set callback functions
## to be called at a later time that you specify.
## <b>Example usage:</b>
##
## <pre>
## def test( v ):
##   print v
##
## tc = TimerCallback()
## tc.addCallback( time.getValue()+3, test, ("Hello world!",) )
## </pre>
## This will call the function test 3 seconds from it is run. 
class TimerCallback( AutoUpdate( SFTime ) ):
  ## Constructor.
  def __init__( self ):
    AutoUpdate( SFTime ).__init__( self )
    ## The list of callbacks currently in use.
    self.callbacks = []

    # Set up a route from H3DInterface.time in order for the update
    # function to run once per scene graph loop.
    time.route( self )

  ## Specialized update function to call callback functions when the time
  ## is right.
  def update( self, event ):
    t = event.getValue()
    cbs_to_remove = []
    for cb in self.callbacks:
      if t > cb[0]:
        cb[1](*cb[2])
        cbs_to_remove.append( cb )
    for cb in cbs_to_remove:
      self.callbacks.remove( cb )

    return event.getValue()

  ## Add a callback function. The function will be called at the specified
  ## time with the given arguments and then removed.
  ## @param time The time to run the function.
  ## @param func The function to call.
  ## @param args Tuple with the arguments to call.
  ## @return Handle to callback function.
  def addCallback( self, time, func, args ):
    cb = (time, func, args )
    self.callbacks.append( cb )
    return cb

  ## Remove a callback function before it has executed.
  ## @param cb The handle of the callback to remove.
  def removeCallback( self, cb ):
    try:
      self.callbacks.remove( cb )
    except:
      pass

import gc
import sys
from types import FrameType
import H3DInterface

from types import ModuleType

def printPythonPath(path, prefix = "", outstream = sys.stdout ):
  def _printPythonPathStep ( step, next, prefix ):
    #if not next:
    #  outstream.write( repr(step) )
    outstream.write("%s%s -- " % (prefix, str(type(step))))
    if isinstance(step, dict):
      found = False
      for key, val in step.items():
        if val is next:
          outstream.write("[%s]" % repr(key))
          found = True
          break
        if key is next:
          outstream.write("[key] = %s" % repr(val))
          found = True
          break
      if not found:
        print( "Not found: ", repr(next) )
        print( gc.get_referents(step))
    elif isinstance(step, list):
      outstream.write("[%d]" % step.index(next))
    elif isinstance(step, tuple):
      outstream.write("[%d]" % list(step).index(next))
    else:
      outstream.write(repr(step))  

  for i, step in enumerate(path):
    # next "wraps around"
    next = path[(i + 1) % len(path)]
    _printPythonPathStep( step, next, prefix )
    if( i < len(path) - 1 ):
      outstream.write(" ->\n")  
 
def printNodePath(path, prefix = "", outstream = sys.stdout):

  def _printNodePathStep ( step, next, prefix ):
    #if not next:
    #  outstream.write( repr(step) )
    n = step
    if( isinstance( step, tuple ) ):
      n = step[0]
 
    outstream.write("%s%s -- " % (prefix, n.getTypeName()) )
    outstream.write(repr(n))    

    if( isinstance( next, tuple ) ):
      if isinstance( next[1], list ):
        outstream.write( "\n" )          
        printPythonPath( next[1], prefix + "  ", outstream )
      else:
        outstream.write( repr(next[1]) )

  # function start
  for i, step in enumerate(path):
    # next "wraps around"
    if( i < len( path ) - 1 ):
      next = path[(i + 1) % len(path)]
    else:
      next = None
    _printNodePathStep( step, next, prefix )
    if( next ):
      outstream.write(" ->\n")
  outstream.write("\n\n")

def _getPythonScriptNodeReferents( pythonscript_node ):
  """ Get all nodes that are held by references by variables in the python code in a PythonScript node. 
      @return [ list[ tuple( Node, list ) ] A list of tuples for each node where the first element is the node itself and the second
      element a representation of the path within the python script code to the variable that holds the reference
      to the node. 
  """
  module_name = pythonscript_node.moduleName.getValue()
  module = sys.modules[ module_name ]
  nodes = _getPythonObjectNodeReferents( module, {}, [module] ) 
 
  return nodes
      
def _getNodeReferents( node ):
  """ Get all nodes that are directly referenced in the given node, i.e. nodes that are contained in a
      field of the node or in the case of a PythonScript nodes that are held by references by variables
      in the PythonScript.
      @return [ list[ tuple( Node, str|list ) ] A list of tuples for each node where the first element is the node itself and the second
      element is the name of the field (string) it resides in. If the node is a PythonScript the second element
      will be a representation of the path within the python script code to the variable that holds the reference
      to the node (see _getPythonReferents for details)
  """
  fields = node.getFieldList()
  referents = []
  for field_name in fields:
    field = node.getField( field_name )
    if field.type == H3DInterface.MFNODE:
      ns = field.getValue()
      referents.extend( [ (n, field_name) for n in ns ] )
    elif field.type == H3DInterface.SFNODE:
      n = field.getValue()
      if( n ):
        referents.append( (n, field_name ) )
  if( node.getTypeName() == "PythonScript" ):
    referents.extend(_getPythonScriptNodeReferents( node ) )
  return referents

def _getPythonObjectNodeReferents( obj, visited = {}, current_path = [] ):
  """ Get all nodes that are held in variables directly or indirectly referenced by a python object. .  
      @param obj [any] The object to check for nodes in
      @param visited [dict[id]] A dict of python objects already visited. Only used for internal purposes, should not be set by user.
      @param current_path list[python objects] A list of python objects representing the hierarchical references in how to get
      to the current object. Only used for internal purposes, should not be set by user.
      @return [ list[ tuple( Node, list[ python_objects ] ) A list of tuples for each node fount where the first element is the 
      node itself and the second element is a representation of the path within the python script code to the variable that holds
      the reference to the node is found. It is defined as list of python objects where the first element is the topmost object
      that the last element is the object that actually holds the reference to the node.
   """
  refs = gc.get_referents( obj )

  # ignore the __scriptnode__ member of PythonScript python modules as that variable does
  # not increase the reference count of the node
  if( isinstance(obj, ModuleType ) ):
    n = obj.__dict__.get("__scriptnode__", None)
    if( not id(n) in visited ):
      visited[id(n)] = None
      
  nodes = []
  for ref in refs:
    object_id = id( ref )
    if not object_id in visited:
      visited[ object_id ] = None      
      if( isinstance( ref, H3DInterface.Node ) ):
        nodes.append( (ref, current_path + [ref] ) )
      elif( hasattr( ref, "__fieldptr__" ) ):
        if( ref.type == H3DInterface.SFNode ):
          n = ref.getValue()
          nodes.append( (n, current_path+[ref]) )
        elif( ref.type == H3DInterface.MFNode ):
          ns = ref.getValue()
          nodes.extend( (n, current_path+[ref]) for n in ns ) 
         
      else:
        # ignoring modules to keep things within the current python script code
        if( not isinstance( ref, ModuleType ) ):
          nodes.extend( _getPythonObjectNodeReferents( ref, visited, current_path + [ref] ) )
        
  return nodes

def _getNodeRefsFromNode(node, start, all, current_path, output):
  """ Recursive function to step through all child node references to node.
     @param node [ Node instance ] The node which children to step through.
     @param start [ Node instance ] The node that we are looking for.
     @param all [ dict[str] ]A dict object with the name of all nodes encountered so far while recursing
     @param current_path [ list[ tuple( Node, str|list ) ] ] A representation of the hieararchy of where to find the node parameter. 
     @param output list[ list[ tuple( Node, str|list[ python objects ] List to add references to as they are found
     It is a list of tuple objects with the first parameter being a node and the second the field name of the parent where this node 
     resided. If the node is a PythonScript node the second parameter is a description of the path in python to the node, see
     _getReferents for details.
     @return None
  """
  if not node:
    return

  # Add the current node to the dict of all visited nodes
  # todo: if slow, find a better key to use than this
  all[repr(node)] = None

  # get all children of the node
  referents = _getNodeReferents(node)
  for referent, origin in referents:
    if not referent:
      continue
 
    # The node is what we want, collect it
    if referent == start:
      output.append( current_path + [(referent, origin)] ) 

    # We haven't seen this object before, so recurse
    if repr(referent) not in all :
      _getNodeRefsFromNode(referent, start, all, current_path + [(referent, origin)], output)
  
def getNodeRefsFromScene(node):
  """Get all references to a node in the current scene.
  @param node [Node] Node instance to get all references of.
  @return [list[ list[ tuple( Node, str|list[ python objects ] ) ] ] Returns a list with one entry for each reference with the "path" to that 
  reference. Each path is represented by a list of tuples where the first element is the topmost object
  that the last element is the object that actually holds the reference to the node. The each tuple contains a node and the name of the 
  field in the parent node (or in case of a reference held in python code a list of python objects describing the path within the
  python code)
  """
    
  s = H3DInterface.getCurrentScenes()[0]
  root = s.sceneRoot.getValue()
  output = []
  _getNodeRefsFromNode( s, node, { }, [ (s, "")], output )
  return output              
      
def getNodeRefsFromPythonModules( node ):
  """Get all references to a node that can be reached from any python modules in sys.modules.
  @param node [Node] Node instance to get all references of.
  @return [list[ list[ tuple( Node, str|list[ python objects ] ) ] ] Returns a list with one entry for each reference with the "path" to that 
  reference. Each path is represented by a list of tuples where the first element is the topmost object
  that the last element is the object that actually holds the reference to the node. The each tuple contains a node and the name of the 
  field in the parent node (or in case of a reference held in python code a list of python objects describing the path within the
  python code)
  """
  visited = {}
  node_referents = []

  # add all nodes that are held by any python object that can be reached from sys.modules to node_referents
  for module_name, module in sys.modules.items():
    node_referents.extend( _getPythonObjectNodeReferents( module, visited, [module] ) )

  output = []
  
  # for each node in python objects, find the ones that have the node of interest in its children and add them to output.
  for n, path in node_referents:
    if( n == node ):
      # the python object itself is holding the node of interest, so just add it to output
      output.append( [ (n, path) ] )
    else:
      _getNodeRefsFromNode( n, node, {}, [ (n, path) ], output )

  return output

 