
////////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file H3DInterface.py.h
/// \brief header file containing H3DInterface python module strings.
///
//
//////////////////////////////////////////////////////////////////////////////

namespace H3D {

  namespace H3DInterface {
    const string H3DInterface_string = "\
﻿from H3D import *\n\
\n\
import sys\n\
\n\
class LogLevel:\n\
  Debug = 1\n\
  Info = 3\n\
  Warning = 4\n\
  Error = 5\n\
\n\
_ConsoleStdout = H3DConsole( LogLevel.Info )\n\
_ConsoleStderr = H3DConsole( LogLevel.Error )\n\
# Console kept for backwards compatibility reasons \n\
Console = _ConsoleStdout\n\
\n\
sys.stdout = _ConsoleStdout\n\
sys.stderr = _ConsoleStderr\n\
  \n\
def log ( level, message ):\n\
  Console.writeAtLevel ( level, message + \"\\n\" )\n\
\n\
###################################################################\n\
#\n\
# Base types; Vec2f, Vec3f, Vec2d, Vec2d, etc, etc, etc\n\
#\n\
###################################################################\n\
\n\
###################################################################\n\
#\n\
# Field base class and SF* MF* class definitions\n\
#\n\
###################################################################\n\
class Field(object):\n\
  type = UNKNOWN_X3D_TYPE\n\
  def __init__( self, auto_update = 0 ):\n\
    super(Field, self).__init__()\n\
    module = self.__class__.__dict__[\"__module__\"]\n\
    createField( self, auto_update, module + \".\" + self.__class__.__name__ )\n\
\n\
  def setName( self, name ):\n\
    return fieldSetName( self, name )\n\
\n\
  def getName( self ):\n\
    return fieldGetName( self )\n\
    \n\
  def getFullName( self ):\n\
    return fieldGetFullName( self )\n\
 \n\
  def getTypeName( self ):\n\
    return fieldGetTypeName( self )\n\
\n\
  def getOwner( self ):\n\
    return fieldGetOwner( self )\n\
\n\
  def setOwner( self, n ):\n\
    return fieldSetOwner( self, n )\n\
\n\
  def route( self, dest ):\n\
    return fieldRoute( self, dest )\n\
\n\
  def routeNoEvent( self, dest ):\n\
    return fieldRouteNoEvent( self, dest )\n\
\n\
  def unroute( self, dest ):\n\
    return fieldUnroute( self, dest )\n\
\n\
  def replaceRoute( self, dest, i ):\n\
    return fieldReplaceRoute( self, dest, i )\n\
\n\
  def replaceRouteNoEvent( self, dest, i ):\n\
    return fieldReplaceRouteNoEvent( self, dest, i )\n\
\n\
  def unrouteAll( self ):\n\
    return fieldUnrouteAll( self )\n\
    \n\
  def touch( self ):\n\
    return fieldTouch( self )\n\
\n\
  def routesTo( self, f ):\n\
    return fieldRoutesTo( self, f )\n\
\n\
  def hasRouteFrom( self, f ):\n\
    return fieldHasRouteFrom( self, f )\n\
\n\
  def getRoutesIn( self ):\n\
    return fieldGetRoutesIn( self )\n\
\n\
  def getRoutesOut( self ):\n\
    return fieldGetRoutesOut( self )\n\
\n\
  def setAccessType( self, access_type ):\n\
    return fieldSetAccessType( self, access_type )\n\
\n\
  def getAccessType( self ):\n\
    return fieldGetAccessType( self )\n\
\n\
  def setAccessCheck( self, access_check ):\n\
    return fieldSetAccessCheck( self, access_check )\n\
\n\
  def isAccessCheckOn( self ):\n\
    return fieldIsAccessCheckOn( self )\n\
\n\
  def setValueFromString( self, value ):\n\
    return fieldSetValueFromString( self, value )\n\
\n\
  def getValueAsString( self ):\n\
    return fieldGetValueAsString( self )\n\
\n\
  def upToDate( self ):\n\
    return fieldUpToDate( self )\n\
\n\
  def isUpToDate( self ):\n\
    return fieldIsUpToDate( self )\n\
    \n\
  def __cmp__( self, o ):\n\
    return getCPtr(self) - getCPtr(o)\n\
\n\
  def __eq__( self, o ):\n\
    return not self.__cmp__( o )\n\
\n\
  def __ne__( self, o ):\n\
    return not self.__eq__( o )\n\
  \n\
  def __lt__( self, o ):\n\
    return NotImplemented\n\
  \n\
  def __le__( self, o ):\n\
    return NotImplemented\n\
  \n\
  def __gt__( self, o ):\n\
    return NotImplemented\n\
  \n\
  def __ge__( self, o ):\n\
    return NotImplemented\n\
\n\
class SField( Field ):\n\
  type = UNKNOWN_X3D_TYPE\n\
  def setValue( self, value ):\n\
    fieldSetValue( self, value )\n\
\n\
  def getValue( self ):\n\
    return fieldGetValue( self )\n\
\n\
class MField( Field ):\n\
  type = UNKNOWN_X3D_TYPE\n\
  def setValue( self, value ):\n\
    fieldSetValue( self, value )\n\
\n\
  def getValue( self ):\n\
    return fieldGetValue( self )\n\
\n\
  def push_back( self, v ):\n\
    MFieldPushBack( self, v ) \n\
\n\
  def pop_back( self ):\n\
    MFieldPopBack( self )\n\
\n\
  def empty( self ):\n\
    return MFieldEmpty( self )\n\
\n\
  def front( self ):\n\
    return MFieldFront( self )\n\
\n\
  def back( self ):\n\
    return MFieldBack( self )\n\
\n\
  def clear( self ):\n\
    MFieldClear( self )\n\
\n\
  def erase( self, v ):\n\
    MFieldErase( self, v ) \n\
    \n\
  def size( self ):\n\
    return MFieldSize( self )\n\
\n\
\n\
# Install all built-in Field types:\n\
sfield_types = [ \n\
  ( SFFLOAT,    \"SFFloat\" ),\n\
  ( SFDOUBLE,   \"SFDouble\" ),\n\
  ( SFTIME,     \"SFTime\" ),\n\
  ( SFINT32,    \"SFInt32\" ),\n\
  ( SFVEC2F,    \"SFVec2f\" ),\n\
  ( SFVEC2D,    \"SFVec2d\" ),\n\
  ( SFVEC3F,    \"SFVec3f\" ),\n\
  ( SFVEC3D,    \"SFVec3d\" ),\n\
  ( SFVEC4F,    \"SFVec4f\" ),\n\
  ( SFVEC4D,    \"SFVec4d\" ),\n\
  ( SFBOOL,     \"SFBool\"  ),\n\
#  ( SFSTRING,   \"SFString\" ),\n\
  ( SFCOLOR,    \"SFColor\" ),\n\
  ( SFCOLORRGBA,\"SFColorRGBA\" ),\n\
  ( SFROTATION, \"SFRotation\" ),\n\
  ( SFQUATERNION, \"SFQuaternion\" ),\n\
  ( SFMATRIX3F, \"SFMatrix3f\" ),\n\
  ( SFMATRIX4F, \"SFMatrix4f\" ),\n\
  ( SFMATRIX3D, \"SFMatrix3d\" ),\n\
  ( SFMATRIX4D, \"SFMatrix4d\" ),\n\
  ( SFNODE    , \"SFNode\"     ) ]\n\
\n\
mfield_types = [\n\
  ( MFFLOAT,    \"MFFloat\" ),\n\
  ( MFDOUBLE,   \"MFDouble\" ),\n\
  ( MFTIME,     \"MFTime\" ),\n\
  ( MFINT32,    \"MFInt32\" ),\n\
  ( MFVEC2F,    \"MFVec2f\" ),\n\
  ( MFVEC2D,    \"MFVec2d\" ),\n\
  ( MFVEC3F,    \"MFVec3f\" ),\n\
  ( MFVEC3D,    \"MFVec3d\" ),\n\
  ( MFVEC4F,    \"MFVec4f\" ),\n\
  ( MFVEC4D,    \"MFVec4d\" ),\n\
  ( MFBOOL,     \"MFBool\"  ),\n\
  ( MFSTRING,   \"MFString\" ),\n\
  ( MFCOLOR,    \"MFColor\" ),\n\
  ( MFCOLORRGBA,\"MFColorRGBA\" ),\n\
  ( MFROTATION, \"MFRotation\" ),\n\
  ( MFQUATERNION, \"MFQuaternion\" ),\n\
  ( MFMATRIX3F, \"MFMatrix3f\" ),\n\
  ( MFMATRIX4F, \"MFMatrix4f\" ),\n\
  ( MFMATRIX3D, \"MFMatrix3d\" ),\n\
  ( MFMATRIX4D, \"MFMatrix4d\" ),\n\
  ( MFNODE    , \"MFNode\"     )\n\
]\n\
\n\
for t in sfield_types:\n\
  exec(\"\"\"\n\
class %s( SField ):\n\
  type = %s\n\
\"\"\" % (t[1], t[0] ))\n\
\n\
for t in mfield_types:\n\
  exec(\"\"\"\n\
class %s( MField ):\n\
  type = %s\n\
\"\"\" % (t[1], t[0] ))\n\
\n\
\n\
class SFString( SField ):\n\
  type = SFSTRING\n\
\n\
  def getValidValues( self ):\n\
    return SFStringGetValidValues( self )\n\
\n\
  def isValidValue( self, value ):\n\
    return SFStringIsValidValue( self, value )\n\
\n\
typed_field_classes = {}\n\
\n\
def TypedField( base_class, type_info = None, opt_type_info = None ):\n\
  global typed_field_classes\n\
  if( (base_class, type_info, opt_type_info) in typed_field_classes ):\n\
    return typed_field_classes[(base_class, type_info, opt_type_info)]\n\
\n\
  class TypedBase( base_class ):\n\
    pass\n\
  if type_info == None:\n\
    TypedBase.__type_info__ = ()\n\
  elif type( type_info ) != type(()):\n\
    TypedBase.__type_info__ = ( type_info, )\n\
  else:\n\
    TypedBase.__type_info__ = type_info\n\
\n\
  if opt_type_info == None:\n\
    TypedBase.__opt_type_info__ = ()\n\
  elif type( opt_type_info ) != type(()):\n\
    TypedBase.__opt_type_info__ = ( opt_type_info, )\n\
  else:\n\
    TypedBase.__opt_type_info__ = opt_type_info\n\
\n\
  typed_field_classes[(base_class, type_info, opt_type_info)] = TypedBase\n\
  return TypedBase\n\
\n\
\n\
auto_update_classes = {}\n\
\n\
\n\
# AutoUpdate \"template\" as in C++\n\
def AutoUpdate( base_class ):\n\
\n\
  global auto_update_classes\n\
  if( base_class in auto_update_classes ):\n\
    return auto_update_classes[base_class]\n\
  else:\n\
    class AutoUpdateBase( base_class ):\n\
      def __init__( self ):\n\
        super(AutoUpdateBase, self).__init__(1)\n\
    auto_update_classes[base_class] = AutoUpdateBase\n\
    return AutoUpdateBase\n\
\n\
periodic_update_classes = {}\n\
\n\
def PeriodicUpdate( base_class ):\n\
  global periodic_update_classes\n\
  if( base_class in periodic_update_classes ):\n\
    return periodic_update_classes[base_class]\n\
  else:\n\
    class PeriodicUpdateBase( base_class ):\n\
      def __init__( self ):\n\
        super(PeriodicUpdateBase, self).__init__(0)\n\
        self.route( eventSink )\n\
    periodic_update_classes[base_class] = PeriodicUpdateBase\n\
    return PeriodicUpdateBase\n\
";
    const string H3DUtils_string = "\
from H3DInterface import *\n\
\n\
def PrintFieldValue( base_class ):\n\
  class PrintValueClass( AutoUpdate( base_class ) ):\n\
    def update( self, event ):\n\
      v = event.getValue()\n\
      print(v)\n\
      return v\n\
  return PrintValueClass()\n\
\n\
def FieldValue2String( base_class ):\n\
  class Value2StringClass( TypedField( SFString, base_class ) ):\n\
    def update( self, event ):\n\
      v = event.getValue()\n\
      return str( v )\n\
  return Value2StringClass()\n\
\n\
def FieldValue2StringList( base_class ):\n\
  class Value2StringListClass( TypedField( MFString, base_class ) ):\n\
    def update( self, event ):\n\
      v = event.getValue()\n\
      return [str( v )]\n\
  return Value2StringListClass()\n\
\n\
def FieldValue2Int( base_class ):\n\
  class Value2IntClass( TypedField( SFInt32, base_class ) ):\n\
    def update( self, event ):\n\
      v = event.getValue()\n\
      return int( v )\n\
  return Value2IntClass()\n\
\n\
def SField2MField( sfield, mfield ):\n\
  valid_sfield = False\n\
  for t in sfield_types:\n\
    if sfield.type == t[0]:\n\
      valid_sfield = True\n\
      break\n\
  if not valid_sfield:\n\
    raise Exception( \"Warning: Could not create SField2MField. Invalid sfield type \" + sfield.__name__ + \" used.\" )\n\
  valid_mfield = False\n\
  for t in mfield_types:\n\
    if mfield.type == t[0]:\n\
      valid_mfield = True\n\
      break\n\
  if not valid_mfield:\n\
    raise Exception( \"Warning: Could not create SField2MField. Invalid mfield type \" + mfield.__name__ + \" used.\" )\n\
  class SField2MFieldClass( TypedField( mfield, sfield ) ):\n\
    def update( self, event ):\n\
      v = event.getValue()\n\
      return [v]\n\
  return SField2MFieldClass()\n\
\n\
## The TimerCallback field is a field in which you can set callback functions\n\
## to be called at a later time that you specify.\n\
## <b>Example usage:</b>\n\
##\n\
## <pre>\n\
## def test( v ):\n\
##   print v\n\
##\n\
## tc = TimerCallback()\n\
## tc.addCallback( time.getValue()+3, test, (\"Hello world!\",) )\n\
## </pre>\n\
## This will call the function test 3 seconds from it is run. \n\
class TimerCallback( AutoUpdate( SFTime ) ):\n\
  ## Constructor.\n\
  def __init__( self ):\n\
    AutoUpdate( SFTime ).__init__( self )\n\
    ## The list of callbacks currently in use.\n\
    self.callbacks = []\n\
\n\
    # Set up a route from H3DInterface.time in order for the update\n\
    # function to run once per scene graph loop.\n\
    time.route( self )\n\
\n\
  ## Specialized update function to call callback functions when the time\n\
  ## is right.\n\
  def update( self, event ):\n\
    t = event.getValue()\n\
    cbs_to_remove = []\n\
    for cb in self.callbacks:\n\
      if t > cb[0]:\n\
        cb[1](*cb[2])\n\
        cbs_to_remove.append( cb )\n\
    for cb in cbs_to_remove:\n\
      self.callbacks.remove( cb )\n\
\n\
    return event.getValue()\n\
\n\
  ## Add a callback function. The function will be called at the specified\n\
  ## time with the given arguments and then removed.\n\
  ## @param time The time to run the function.\n\
  ## @param func The function to call.\n\
  ## @param args Tuple with the arguments to call.\n\
  ## @return Handle to callback function.\n\
  def addCallback( self, time, func, args ):\n\
    cb = (time, func, args )\n\
    self.callbacks.append( cb )\n\
    return cb\n\
\n\
  ## Remove a callback function before it has executed.\n\
  ## @param cb The handle of the callback to remove.\n\
  def removeCallback( self, cb ):\n\
    try:\n\
      self.callbacks.remove( cb )\n\
    except:\n\
      pass\n\
\n\
import gc\n\
import sys\n\
from types import FrameType\n\
import H3DInterface\n\
\n\
from types import ModuleType\n\
\n\
def printPythonPath(path, prefix = \"\", outstream = sys.stdout ):\n\
  def _printPythonPathStep ( step, next, prefix ):\n\
    #if not next:\n\
    #  outstream.write( repr(step) )\n\
    outstream.write(\"%s%s -- \" % (prefix, str(type(step))))\n\
    if isinstance(step, dict):\n\
      found = False\n\
      for key, val in step.items():\n\
        if val is next:\n\
          outstream.write(\"[%s]\" % repr(key))\n\
          found = True\n\
          break\n\
        if key is next:\n\
          outstream.write(\"[key] = %s\" % repr(val))\n\
          found = True\n\
          break\n\
      if not found:\n\
        print( \"Not found: \", repr(next) )\n\
        print( gc.get_referents(step))\n\
    elif isinstance(step, list):\n\
      outstream.write(\"[%d]\" % step.index(next))\n\
    elif isinstance(step, tuple):\n\
      outstream.write(\"[%d]\" % list(step).index(next))\n\
    else:\n\
      outstream.write(repr(step))  \n\
\n\
  for i, step in enumerate(path):\n\
    # next \"wraps around\"\n\
    next = path[(i + 1) % len(path)]\n\
    _printPythonPathStep( step, next, prefix )\n\
    if( i < len(path) - 1 ):\n\
      outstream.write(\" ->\\n\")  \n\
 \n\
def printNodePath(path, prefix = \"\", outstream = sys.stdout):\n\
\n\
  def _printNodePathStep ( step, next, prefix ):\n\
    #if not next:\n\
    #  outstream.write( repr(step) )\n\
    n = step\n\
    if( isinstance( step, tuple ) ):\n\
      n = step[0]\n\
 \n\
    outstream.write(\"%s%s -- \" % (prefix, n.getTypeName()) )\n\
    outstream.write(repr(n))    \n\
\n\
    if( isinstance( next, tuple ) ):\n\
      if isinstance( next[1], list ):\n\
        outstream.write( \"\\n\" )          \n\
        printPythonPath( next[1], prefix + \"  \", outstream )\n\
      else:\n\
        outstream.write( repr(next[1]) )\n\
\n\
  # function start\n\
  for i, step in enumerate(path):\n\
    # next \"wraps around\"\n\
    if( i < len( path ) - 1 ):\n\
      next = path[(i + 1) % len(path)]\n\
    else:\n\
      next = None\n\
    _printNodePathStep( step, next, prefix )\n\
    if( next ):\n\
      outstream.write(\" ->\\n\")\n\
  outstream.write(\"\\n\\n\")\n\
\n\
def _getPythonScriptNodeReferents( pythonscript_node ):\n\
  \"\"\" Get all nodes that are held by references by variables in the python code in a PythonScript node. \n\
      @return [ list[ tuple( Node, list ) ] A list of tuples for each node where the first element is the node itself and the second\n\
      element a representation of the path within the python script code to the variable that holds the reference\n\
      to the node. \n\
  \"\"\"\n\
  module_name = pythonscript_node.moduleName.getValue()\n\
  module = sys.modules[ module_name ]\n\
  nodes = _getPythonObjectNodeReferents( module, {}, [module] ) \n\
 \n\
  return nodes\n\
      \n\
def _getNodeReferents( node ):\n\
  \"\"\" Get all nodes that are directly referenced in the given node, i.e. nodes that are contained in a\n\
      field of the node or in the case of a PythonScript nodes that are held by references by variables\n\
      in the PythonScript.\n\
      @return [ list[ tuple( Node, str|list ) ] A list of tuples for each node where the first element is the node itself and the second\n\
      element is the name of the field (string) it resides in. If the node is a PythonScript the second element\n\
      will be a representation of the path within the python script code to the variable that holds the reference\n\
      to the node (see _getPythonReferents for details)\n\
  \"\"\"\n\
  fields = node.getFieldList()\n\
  referents = []\n\
  for field_name in fields:\n\
    field = node.getField( field_name )\n\
    if field.type == H3DInterface.MFNODE:\n\
      ns = field.getValue()\n\
      referents.extend( [ (n, field_name) for n in ns ] )\n\
    elif field.type == H3DInterface.SFNODE:\n\
      n = field.getValue()\n\
      if( n ):\n\
        referents.append( (n, field_name ) )\n\
  if( node.getTypeName() == \"PythonScript\" ):\n\
    referents.extend(_getPythonScriptNodeReferents( node ) )\n\
  return referents\n\
\n\
def _getPythonObjectNodeReferents( obj, visited = {}, current_path = [] ):\n\
  \"\"\" Get all nodes that are held in variables directly or indirectly referenced by a python object. .  \n\
      @param obj [any] The object to check for nodes in\n\
      @param visited [dict[id]] A dict of python objects already visited. Only used for internal purposes, should not be set by user.\n\
      @param current_path list[python objects] A list of python objects representing the hierarchical references in how to get\n\
      to the current object. Only used for internal purposes, should not be set by user.\n\
      @return [ list[ tuple( Node, list[ python_objects ] ) A list of tuples for each node fount where the first element is the \n\
      node itself and the second element is a representation of the path within the python script code to the variable that holds\n\
      the reference to the node is found. It is defined as list of python objects where the first element is the topmost object\n\
      that the last element is the object that actually holds the reference to the node.\n\
   \"\"\"\n\
  refs = gc.get_referents( obj )\n\
\n\
  # ignore the __scriptnode__ member of PythonScript python modules as that variable does\n\
  # not increase the reference count of the node\n\
  if( isinstance(obj, ModuleType ) ):\n\
    n = obj.__dict__.get(\"__scriptnode__\", None)\n\
    if( not id(n) in visited ):\n\
      visited[id(n)] = None\n\
      \n\
  nodes = []\n\
  for ref in refs:\n\
    object_id = id( ref )\n\
    if not object_id in visited:\n\
      visited[ object_id ] = None      \n\
      if( isinstance( ref, H3DInterface.Node ) ):\n\
        nodes.append( (ref, current_path + [ref] ) )\n\
      elif( hasattr( ref, \"__fieldptr__\" ) ):\n\
        if( ref.type == H3DInterface.SFNode ):\n\
          n = ref.getValue()\n\
          nodes.append( (n, current_path+[ref]) )\n\
        elif( ref.type == H3DInterface.MFNode ):\n\
          ns = ref.getValue()\n\
          nodes.extend( (n, current_path+[ref]) for n in ns ) \n\
         \n\
      else:\n\
        # ignoring modules to keep things within the current python script code\n\
        if( not isinstance( ref, ModuleType ) ):\n\
          nodes.extend( _getPythonObjectNodeReferents( ref, visited, current_path + [ref] ) )\n\
        \n\
  return nodes\n\
\n\
def _getNodeRefsFromNode(node, start, all, current_path, output):\n\
  \"\"\" Recursive function to step through all child node references to node.\n\
     @param node [ Node instance ] The node which children to step through.\n\
     @param start [ Node instance ] The node that we are looking for.\n\
     @param all [ dict[str] ]A dict object with the name of all nodes encountered so far while recursing\n\
     @param current_path [ list[ tuple( Node, str|list ) ] ] A representation of the hieararchy of where to find the node parameter. \n\
     @param output list[ list[ tuple( Node, str|list[ python objects ] List to add references to as they are found\n\
     It is a list of tuple objects with the first parameter being a node and the second the field name of the parent where this node \n\
     resided. If the node is a PythonScript node the second parameter is a description of the path in python to the node, see\n\
     _getReferents for details.\n\
     @return None\n\
  \"\"\"\n\
  if not node:\n\
    return\n\
\n\
  # Add the current node to the dict of all visited nodes\n\
  # todo: if slow, find a better key to use than this\n\
  all[repr(node)] = None\n\
\n\
  # get all children of the node\n\
  referents = _getNodeReferents(node)\n\
  for referent, origin in referents:\n\
    if not referent:\n\
      continue\n\
 \n\
    # The node is what we want, collect it\n\
    if referent == start:\n\
      output.append( current_path + [(referent, origin)] ) \n\
\n\
    # We haven't seen this object before, so recurse\n\
    if repr(referent) not in all :\n\
      _getNodeRefsFromNode(referent, start, all, current_path + [(referent, origin)], output)\n\
  \n\
def getNodeRefsFromScene(node):\n\
  \"\"\"Get all references to a node in the current scene.\n\
  @param node [Node] Node instance to get all references of.\n\
  @return [list[ list[ tuple( Node, str|list[ python objects ] ) ] ] Returns a list with one entry for each reference with the \"path\" to that \n\
  reference. Each path is represented by a list of tuples where the first element is the topmost object\n\
  that the last element is the object that actually holds the reference to the node. The each tuple contains a node and the name of the \n\
  field in the parent node (or in case of a reference held in python code a list of python objects describing the path within the\n\
  python code)\n\
  \"\"\"\n\
    \n\
  s = H3DInterface.getCurrentScenes()[0]\n\
  root = s.sceneRoot.getValue()\n\
  output = []\n\
  _getNodeRefsFromNode( s, node, { }, [ (s, \"\")], output )\n\
  return output              \n\
      \n\
def getNodeRefsFromPythonModules( node ):\n\
  \"\"\"Get all references to a node that can be reached from any python modules in sys.modules.\n\
  @param node [Node] Node instance to get all references of.\n\
  @return [list[ list[ tuple( Node, str|list[ python objects ] ) ] ] Returns a list with one entry for each reference with the \"path\" to that \n\
  reference. Each path is represented by a list of tuples where the first element is the topmost object\n\
  that the last element is the object that actually holds the reference to the node. The each tuple contains a node and the name of the \n\
  field in the parent node (or in case of a reference held in python code a list of python objects describing the path within the\n\
  python code)\n\
  \"\"\"\n\
  visited = {}\n\
  node_referents = []\n\
\n\
  # add all nodes that are held by any python object that can be reached from sys.modules to node_referents\n\
  for module_name, module in sys.modules.items():\n\
    node_referents.extend( _getPythonObjectNodeReferents( module, visited, [module] ) )\n\
\n\
  output = []\n\
  \n\
  # for each node in python objects, find the ones that have the node of interest in its children and add them to output.\n\
  for n, path in node_referents:\n\
    if( n == node ):\n\
      # the python object itself is holding the node of interest, so just add it to output\n\
      output.append( [ (n, path) ] )\n\
    else:\n\
      _getNodeRefsFromNode( n, node, {}, [ (n, path) ], output )\n\
\n\
  return output\n\
\n\
 ";
  }
}