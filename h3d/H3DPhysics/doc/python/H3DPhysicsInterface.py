## \file H3DPhysicsInterface.py Documentation file for the H3DPhysicsInterface module.
import H3DPhysics

## Base class for H3DPhysicsInterface.TrackedMField types which implements functions to track 
## and retrieve modifications to the field's values.
##
## See H3DPhysicsInterface.TrackedMField for more information.
##
class TrackedMFieldBase:
  type = UNKNOWN_X3D_TYPE
  
  ## Inserts one or more new value(s) into the field at specified location(s) and remembers the change
  ##
  ## \param index The index or indices after which the new values will be inserted. Either a single
  ##              int or a list of ints. Each index in the list corresponds to the value in the same 
  ##              place in the values list. The indices are into the updated array, so each new 
  ##              index must take into account insertions that have already happened since the last 
  ##              call to resetTracking(), including those made in this call.
  ## \param value The value or values to insert into the field. Must be equal in number to the
  ##              index parameter.
  ##
  def insertTracked ( self, index, value ):
    H3DPhysics.TrackedMField_insertTracked ( self, index, value )
  
  ## Updates one or more values in the field and remembers the change
  ##
  ## \param index The index or indices of values to update. Either a single int
  ##              or a list of ints. Each index in the list corresponds to the 
  ##              value in the same place in the values list.
  ## \param value The new value or values to assign to the indexed elements.
  ##
  def updateTracked ( self, index, value ):
    H3DPhysics.TrackedMField_updateTracked ( self, index, value )
  
  ## Erase one or more values from the field and remember this change
  ##
  ## \param index The index or indices of elements to erase. Either a single 
  ##              int or a list of ints. The indices are into the state
  ##              of the array before this call is made. I.e., the indices
  ##              supplied should not take into account elements removed during
  ##              this call.
  ## \param count The number of elements to erase after and including index. 
  ##              This should only be used if just one index is specified. If
  ##              count is > 1 and more that one index is specified, then only
  ##              the first index is considered. Default: 1
  ##
  def eraseTracked ( self, index, count= 1 ):
    H3DPhysics.TrackedMField_eraseTracked ( self, index, count )
  
  ## Append a single value to the end of the vector and remember the change
  ##
  ## \param value Value to append.
  ##
  def pushBackTracked ( self, value ):
    H3DPhysics.TrackedMField_insertTracked ( self, self.size(), value )

## A template which adds tracking to an MFField. With tracking the field knows
## what values have been inserted, updated or erased since the last call to resetTracking().
##
## Template functions in H3DPhysicsInterface are similar to templates in C++.
## They return a new class definition based on the paramters.
##
## Using this information a physics engine implementation can optimize the way it handles
## changes in the field value. For example, if only one new link in a SoftBodyAttachment is
## added, there is no need to rebuild all other links.
##
## In order to track changes to the field value, the caller must use the *Tracked() functions
## to edit the field. Changes are still possible using the standard MFField functions, but will
## not be recorded and will trigger a full update.
##
## *Tracked() functions should not be combined with standard MFField modify function in the same
## scene-graph loop. This will cause incorrect results in the physics engine.
##
## <b>Examples:</b>
##   - <a href="../../../../examples/softbody/SoftBodyAttachment.x3d">SoftBodyAttachment.x3d</a>
##     ( <a href="../../../../doc/html/examples/softbody/SoftBodyAttachment.x3d.html">Source</a> )
## 
def TrackedMField ( field_type ):
  
  class FieldClass ( field_type, TrackedMFieldBase ):
    pass
    
  return FieldClass
    
# The following doxygen is generated using this loop:
# mfield_types = [
  # "MFFloat",
  # "MFDouble",
  # "MFTime",
  # "MFInt32",
  # "MFVec2f",
  # "MFVec2d",
  # "MFVec3f",
  # "MFVec3d",
  # "MFVec4f",
  # "MFVec4d",
  # "MFBool",
  # "MFString",
  # "MFColor",
  # "MFColorRGBA",
  # "MFRotation",
  # "MFQuaternion",
  # "MFMatrix3f",
  # "MFMatrix4f",
  # "MFMatrix3d",
  # "MFMatrix4d",
  # "MFNode"
# ]

# field_doc= """## \\namespace H3DPhysicsInterface
# ## \\class H3DPhysicsInterface.Tracked%s
# ## \\extends TrackedMFieldBase
# ## \\brief Adds change tracking capabilities to %s.

# """
    
# for s in mfield_types:
  # print field_doc % (s,s)
    
## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFFloat
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFFloat.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFDouble
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFDouble.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFTime
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFTime.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFInt32
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFInt32.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFVec2f
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFVec2f.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFVec2d
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFVec2d.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFVec3f
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFVec3f.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFVec3d
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFVec3d.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFVec4f
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFVec4f.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFVec4d
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFVec4d.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFBool
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFBool.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFString
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFString.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFColor
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFColor.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFColorRGBA
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFColorRGBA.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFRotation
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFRotation.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFQuaternion
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFQuaternion.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFMatrix3f
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFMatrix3f.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFMatrix4f
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFMatrix4f.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFMatrix3d
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFMatrix3d.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFMatrix4d
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFMatrix4d.


## \namespace H3DPhysicsInterface
## \class H3DPhysicsInterface.TrackedMFNode
## \extends TrackedMFieldBase
## \brief Adds change tracking capabilities to MFNode.