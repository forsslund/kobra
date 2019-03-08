//////////////////////////////////////////////////////////////////////////////
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
/// \file H3DPhysics/src/PythonMethods.cpp
/// \brief Source file containing functions to add to H3DAPIs python interface.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PythonMethods.h>
#include <H3D/H3DPhysics/H3DPhysicsInterface.py.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/PythonMethods.h>
#include <H3D/PythonTypes.h>
#include <memory>

#include <H3DUtil/Console.h>

using namespace H3D;
using namespace H3DUtil;

#ifdef HAVE_PYTHON

#if defined(__APPLE__) && defined(__MACH__) && defined( HAVE_PYTHON_OSX_FRAMEWORK )
#include <Python/Python.h>
#else
#include <Python.h>
#endif

namespace PyFunctions {
  int H3DPyDouble_Check( PyObject *v ) {
    return PyFloat_Check( v ) || PyInt_Check( v ) || PyLong_Check( v );
  }

  double H3DPyDouble_AsDouble( PyObject *v ) {
    if( PyFloat_Check( v ) )
      return PyFloat_AsDouble( v );
    else if( PyInt_Check( v ) ) 
      return (double)PyInt_AsLong( v );
      else if( PyLong_Check( v ) ) 
      return PyLong_AsDouble( v );
    return 0.0;
  }

  PyObject *H3DPyString_FromString( const string &s ) {
    return PyString_FromString( s.c_str() );
  }

  /// Helper function to avoid compiler warning about forcing long to bool
  bool PyInt_AsBool ( PyObject* v ) {
    return PyInt_AsLong ( v ) != 0;
  }

}

// Insert macro
#define MFIELD_INSERT( check_func, value_func, from_func, \
                             value_type, field_type, \
                             field, value, pos ) \
 if( ! value || ! check_func( value ) ) { \
    PyErr_SetString( PyExc_ValueError,  \
                     "Invalid argument type to TrackedMField.insertTracked() function" ); \
    return 0; \
 } \
 static_cast< field_type *> \
    (field_ptr)->insertTracked ( static_cast< field_type *>(field)->begin()+pos, (value_type)value_func( value ) );

// Update macro
#define MFIELD_UPDATE( check_func, value_func, from_func, \
                             value_type, field_type, \
                             field, value, pos ) \
 if( ! value || ! check_func( value ) ) { \
    PyErr_SetString( PyExc_ValueError,  \
                     "Invalid argument type to TrackedMField.updateTracked() function" ); \
    return 0; \
 } \
 if ( pos < 0 ) { \
   pos= static_cast< field_type *>(field)->size()+pos; \
 } \
 static_cast< field_type *> \
    (field_ptr)->updateTracked ( static_cast< field_type *>(field)->begin()+pos, (value_type)value_func( value ) );

// Erase macro
#define MFIELD_ERASE( check_func, value_func, from_func, \
                             value_type, field_type, \
                             field, count, pos ) \
 if ( pos < 0 ) { \
   pos= static_cast< field_type *>(field)->size()+pos; \
 } \
 static_cast< field_type *> \
    (field_ptr)->eraseRangeTracked ( static_cast< field_type *>(field)->begin()+pos, count );

// Insert range macro
#define MFIELD_INSERT_RANGE( check_func, value_func, from_func, \
                       value_type, field_type, \
                       field, value, pos ) \
  if( ! value || ! PyList_Check( value ) ) {                     \
    PyErr_SetString( PyExc_ValueError,                         \
  "Argument type must be a list of values in TrackedMField.insertRangeTracked()" );      \
    return 0;                                                  \
  }                                                            \
  Py_ssize_t n = PyList_GET_SIZE( value );                     \
  vector< value_type > fv;                                     \
  fv.resize( n );                                              \
  for ( int j=0; j < n; ++j ) {                                \
    if ( check_func( PyList_GET_ITEM( value, j ) ) ) {         \
              fv[j] = (value_type)value_func( PyList_GET_ITEM( value, j ) ); \
    } else { \
      PyErr_SetString( PyExc_ValueError, \
                       "Invalid argument type to TrackedMField.insertRangeTracked() function " ); \
      return 0;                                                \
    }                                                          \
  }                                                          \
  static_cast<field_type *>(field)->insertTracked(pos, fv);

// Update range macro
#define MFIELD_UPDATE_RANGE( check_func, value_func, from_func, \
                       value_type, field_type, \
                       field, value, pos ) \
  if( ! value || ! PyList_Check( value ) ) {                     \
    PyErr_SetString( PyExc_ValueError,                         \
  "Argument type must be a list of values in TrackedMField.insertRangeTracked()" );      \
    return 0;                                                  \
  }                                                            \
  Py_ssize_t n = PyList_GET_SIZE( value );                     \
  vector< value_type > fv;                                     \
  fv.resize( n );                                              \
  for ( int j=0; j < n; ++j ) {                                \
    if ( check_func( PyList_GET_ITEM( value, j ) ) ) {         \
              fv[j] = (value_type)value_func( PyList_GET_ITEM( value, j ) ); \
    } else { \
      PyErr_SetString( PyExc_ValueError, \
                       "Invalid argument type to TrackedMField.insertRangeTracked() function " ); \
      return 0;                                                \
    }                                                          \
  }                                                          \
  static_cast<field_type *>(field)->updateTracked(pos, fv);

// Erase range macro
#define MFIELD_ERASE_RANGE( check_func, value_func, from_func, \
                       value_type, field_type, \
                       field, size, pos ) \
  static_cast<field_type *>(field)->eraseTracked(pos);

// Apply function macro
#define APPLY_MFIELD_MACRO( field_ptr, x3d_type, value, pos, macro, success  ) \
  {\
    success = true; \
      switch( x3d_type ) { \
      case X3DTypes::MFFLOAT: { \
        macro( PyFunctions::H3DPyDouble_Check, PyFunctions::H3DPyDouble_AsDouble, \
               PyFloat_FromDouble, H3DFloat, TrackedMField < MFFloat >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFDOUBLE: { \
        macro( PyFunctions::H3DPyDouble_Check, PyFunctions::H3DPyDouble_AsDouble, \
               PyFloat_FromDouble, H3DDouble, TrackedMField < MFDouble >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFTIME: { \
        macro( PyFunctions::H3DPyDouble_Check, PyFunctions::H3DPyDouble_AsDouble, \
               PyFloat_FromDouble, H3DTime, TrackedMField < MFTime >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFINT32: { \
        macro( PyInt_Check, PyInt_AsLong,  PyInt_FromLong, \
               H3DInt32, TrackedMField < MFInt32 >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFVEC2F: { \
        macro( PyVec2f_Check, PyVec2f_AsVec2f, PyVec2f_FromVec2f, \
               Vec2f, TrackedMField < MFVec2f >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFVEC2D: { \
        macro( PyVec2d_Check, PyVec2d_AsVec2d, PyVec2d_FromVec2d, \
               Vec2d, TrackedMField < MFVec2d >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFVEC3F: { \
        macro( PyVec3f_Check, PyVec3f_AsVec3f, PyVec3f_FromVec3f, \
               Vec3f, TrackedMField < MFVec3f >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFVEC3D: { \
        macro( PyVec3d_Check, PyVec3d_AsVec3d, PyVec3d_FromVec3d, \
               Vec3d, TrackedMField < MFVec3d >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFVEC4F: { \
        macro( PyVec4f_Check, PyVec4f_AsVec4f, PyVec4f_FromVec4f, \
               Vec4f, TrackedMField < MFVec4f >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFVEC4D: { \
        macro( PyVec4d_Check, PyVec4d_AsVec4d, PyVec4d_FromVec4d, \
               Vec4d, TrackedMField < MFVec4d >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFBOOL: { \
        macro( PyInt_Check, PyFunctions::PyInt_AsBool, PyInt_FromLong, bool,  \
               TrackedMField < MFBool >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFSTRING: { \
        macro( PyString_Check, PyString_AsString, \
               PyFunctions::H3DPyString_FromString, \
               string, TrackedMField < MFString >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFNODE: { \
      macro( PyNode_Check, PyNode_AsNode, PyNode_FromNode,  \
                    Node *, TrackedMField < MFNode >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFCOLOR: { \
        macro( PyRGB_Check, PyRGB_AsRGB, PyRGB_FromRGB, \
               RGB, TrackedMField < MFColor >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFCOLORRGBA: { \
        macro( PyRGBA_Check, PyRGBA_AsRGBA, PyRGBA_FromRGBA, \
               RGBA, TrackedMField < MFColorRGBA >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFROTATION: { \
        macro( PyRotation_Check, PyRotation_AsRotation, PyRotation_FromRotation, \
               Rotation, TrackedMField < MFRotation >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFQUATERNION: { \
        macro( PyQuaternion_Check, PyQuaternion_AsQuaternion, PyQuaternion_FromQuaternion, \
               Quaternion, TrackedMField < MFQuaternion >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFMATRIX3F: { \
        macro( PyMatrix3f_Check, PyMatrix3f_AsMatrix3f, PyMatrix3f_FromMatrix3f, \
               Matrix3f, TrackedMField < MFMatrix3f >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFMATRIX4F: { \
        macro( PyMatrix4f_Check, PyMatrix4f_AsMatrix4f, PyMatrix4f_FromMatrix4f, \
               Matrix4f, TrackedMField < MFMatrix4f >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFMATRIX3D: { \
        macro( PyMatrix3d_Check, PyMatrix3d_AsMatrix3d, PyMatrix3d_FromMatrix3d, \
               Matrix3d, TrackedMField < MFMatrix3d >, field_ptr, value, pos ); break; \
      } \
      case X3DTypes::MFMATRIX4D: { \
        macro( PyMatrix4d_Check, PyMatrix4d_AsMatrix4d, PyMatrix4d_FromMatrix4d, \
               Matrix4d, TrackedMField < MFMatrix4d >, field_ptr, value, pos ); break; \
      } \
      default: success = false; \
    } \
  }

PyObject* TrackedMField_insertTracked( PyObject *self, PyObject *args );
PyObject* TrackedMField_updateTracked( PyObject *self, PyObject *args );
PyObject* TrackedMField_eraseTracked( PyObject *self, PyObject *args );

namespace {
  static PyMethodDef H3DPhysics_PythonMethods[] = {
      {"TrackedMField_insertTracked", TrackedMField_insertTracked, METH_VARARGS, NULL},
      {"TrackedMField_updateTracked", TrackedMField_updateTracked,  METH_VARARGS, NULL},
      {"TrackedMField_eraseTracked",  TrackedMField_eraseTracked,  METH_VARARGS, NULL},
      {NULL, NULL, 0, NULL}        /* Sentinel */
  };
}

#endif

H3DPhysicsPythonInterface::H3DPhysicsPythonInterface () {
  initModule();
}

H3DPhysicsPythonInterface& H3DPhysicsPythonInterface::getInstance () {
  static std::auto_ptr < H3DPhysicsPythonInterface > the_instance ( new H3DPhysicsPythonInterface );
  return *the_instance.get();
}

void H3DPhysicsPythonInterface::initModule () {
#ifdef HAVE_PYTHON
  // Py_Initialize really should be done in the DLL loader function:
  if ( !Py_IsInitialized() ) {
    Py_Initialize();  
    PythonScript::disallowMainThreadPython();
  }

  // ensure we have the GIL lock to work with multiple python threads.
  PyGILState_STATE state = PyGILState_Ensure();

  // Standard H3D Python modules must be created first
  PythonInternals::initH3DInternal();

  // Create H3DPhysics module and functions
#if PY_MAJOR_VERSION >= 3
  static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT,
    "H3DPhysics",     /* m_name */
    "This is a module for H3DPhysics",  /* m_doc */
    -1,                  /* m_size */
    H3DPhysics_PythonMethods,    /* m_methods */
    NULL,                /* m_reload */
    NULL,                /* m_traverse */
    NULL,                /* m_clear */
    NULL,                /* m_free */
  };

  PyObject* h3dPhysicsModule =  PyModule_Create(&moduledef);
#else
  PyObject* h3dPhysicsModule= Py_InitModule("H3DPhysics", H3DPhysics_PythonMethods);
#endif
  PyObject* h3dPhysicsModuleDict= PyModule_GetDict( h3dPhysicsModule );

  
  PyImport_AddModule( "H3DPhysics" );

  // Add builtins to H3DPhysics module
  if (PyDict_GetItemString( static_cast< PyObject * >(h3dPhysicsModuleDict), "__builtins__") == NULL) {
    if (PyDict_SetItemString( static_cast< PyObject * >(h3dPhysicsModuleDict), "__builtins__",
                             PyEval_GetBuiltins()) != 0)
      Console(3) << "Warning: PyEval_GetBuiltins() could not be installed in module dictionary!" << endl;
  }  

  // Import the H3DInterface module so that we can add our new field types to it
  PyObject* h3dPhysicsInterfaceModule= PyImport_ImportModule( "H3DInterface" );
  PyObject* h3dPhysicsInterfaceModuleDict= PyModule_GetDict( h3dPhysicsInterfaceModule );

  // Define classes/types etc using Python script
  string a = H3DPhysicsInterface::H3DPhysicsInterface_string;
  PyObject *r = PyRun_String( H3DPhysicsInterface::H3DPhysicsInterface_string.c_str(), 
    Py_file_input,
    h3dPhysicsInterfaceModuleDict,
    h3dPhysicsInterfaceModuleDict );
  if ( r == NULL ) {
    Console( 3 ) << "Python error in file H3DPhysicsInterface.py.h:" << endl;
    PyErr_Print();
  }

  PyGILState_Release(state);
#endif
}


#ifdef HAVE_PYTHON

PyObject* TrackedMField_insertTracked( PyObject *self, PyObject *args ) {
  if( !args || ! PyTuple_Check( args ) || PyTuple_Size( args ) != 3  ) {
    PyErr_SetString( PyExc_ValueError, 
"Invalid argument(s) to function H3DPhysics.TrackedMField_insertTracked( self, index, value )" );  
    return 0;
  }

  // Get field argument
  PyObject *field = PyTuple_GetItem( args, 0 );
  if( !PyObject_TypeCheck( field, &PyBaseObject_Type ) ) {
    PyErr_SetString( PyExc_ValueError, 
"Invalid Field type given as argument to H3DPhysics.TrackedMField_insertTracked( self, index, value )" );
    return 0;
  }

  // Get index argument
  PyObject *index = PyTuple_GetItem( args, 1 );
  vector<H3DInt32> indices;
  if( PyInt_Check( index ) ) {
    indices.push_back ( PyInt_AsLong ( index ) );
  } else if ( PyList_Check( index ) ) {
    Py_ssize_t n = PyList_GET_SIZE( index );
    indices.resize( n );
    for ( int j=0; j < n; ++j ) {
      if ( PyInt_Check ( PyList_GET_ITEM( index, j ) ) ) {
        indices[j] = PyInt_AsLong ( PyList_GET_ITEM ( index, j ) );
      } else {
        PyErr_SetString( PyExc_ValueError,
                         "Invalid argument type to TrackedMField.insertRangeTracked() function " );
        return 0;
      }
    }
  } else {
    PyErr_SetString( PyExc_ValueError, 
"Invalid index type given as argument to H3DPhysics.TrackedMField_insertTracked( self, index, value )" );
    return 0;
  }

  PyObject *py_field_ptr = PyObject_GetAttrString( field, "__fieldptr__" );
  if( !py_field_ptr ) {
    PyErr_SetString( PyExc_ValueError, 
                      "Python object not a Field type. Make sure that if you \
have defined an __init__ function in a specialized field class, you \
call the base class __init__ function." );
    return 0;
  }
  Field *field_ptr = static_cast< Field * >
    ( PyCapsule_GetPointer( py_field_ptr, NULL ) );

  if( field_ptr && !indices.empty() ) {
    // Get value argument
    PyObject *v = PyTuple_GetItem( args, 2 );
    bool success;
    if ( !PyList_Check( v ) ) {
      APPLY_MFIELD_MACRO( field_ptr, field_ptr->getX3DType(), 
                          v, indices[0], MFIELD_INSERT, success );
    } else {
      APPLY_MFIELD_MACRO( field_ptr, field_ptr->getX3DType(), 
                          v, indices, MFIELD_INSERT_RANGE, success );
    }
    if( !success ) {
      PyErr_SetString( PyExc_ValueError, 
                        "Error: not a valid MField instance" );
      return 0;  
    }
  }
  Py_DECREF( py_field_ptr );
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* TrackedMField_updateTracked( PyObject *self, PyObject *args ) {
  if( !args || ! PyTuple_Check( args ) || PyTuple_Size( args ) != 3  ) {
    PyErr_SetString( PyExc_ValueError, 
"Invalid argument(s) to function H3DPhysics.TrackedMField_updateTracked( self, index, value )" );  
    return 0;
  }

  // Get field argument
  PyObject *field = PyTuple_GetItem( args, 0 );
  if( !PyObject_TypeCheck( field, &PyBaseObject_Type ) ) {
    PyErr_SetString( PyExc_ValueError, 
"Invalid Field type given as argument to H3DPhysics.TrackedMField_updateTracked( self, index, value )" );
    return 0;
  }

  // Get index argument
  PyObject *index = PyTuple_GetItem( args, 1 );
  vector<H3DInt32> indices;
  if( PyInt_Check( index ) ) {
    indices.push_back ( PyInt_AsLong ( index ) );
  } else if ( PyList_Check( index ) ) {
    Py_ssize_t n = PyList_GET_SIZE( index );
    indices.resize( n );
    for ( int j=0; j < n; ++j ) {
      if ( PyInt_Check ( PyList_GET_ITEM( index, j ) ) ) {
        indices[j] = PyInt_AsLong ( PyList_GET_ITEM ( index, j ) );
      } else {
        PyErr_SetString( PyExc_ValueError,
                         "Invalid argument type to TrackedMField.insertRangeTracked() function " );
        return 0;
      }
    }
  } else {
    PyErr_SetString( PyExc_ValueError, 
"Invalid index type given as argument to H3DPhysics.TrackedMField_insertTracked( self, index, value )" );
    return 0;
  }

  PyObject *py_field_ptr = PyObject_GetAttrString( field, "__fieldptr__" );
  if( !py_field_ptr ) {
    PyErr_SetString( PyExc_ValueError, 
                      "Python object not a Field type. Make sure that if you \
have defined an __init__ function in a specialized field class, you \
call the base class __init__ function." );
    return 0;
  }
  Field *field_ptr = static_cast< Field * >
    ( PyCapsule_GetPointer( py_field_ptr, NULL ) );


  if( field_ptr && !indices.empty() ) {
    // Get value argument
    PyObject *v = PyTuple_GetItem( args, 2 );
    bool success;
    if ( !PyList_Check( v ) ) {
      APPLY_MFIELD_MACRO( field_ptr, field_ptr->getX3DType(), 
                          v, indices[0], MFIELD_UPDATE, success );
    } else {
      APPLY_MFIELD_MACRO( field_ptr, field_ptr->getX3DType(), 
                          v, indices, MFIELD_UPDATE_RANGE, success );
    }
    if( !success ) {
      PyErr_SetString( PyExc_ValueError, 
                        "Error: not a valid MField instance" );
      return 0;  
    }
  }
  Py_DECREF( py_field_ptr );
  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* TrackedMField_eraseTracked( PyObject *self, PyObject *args ) {
  if( !args || ! PyTuple_Check( args ) || ( PyTuple_Size( args ) != 2 && PyTuple_Size( args ) != 3 )  ) {
    PyErr_SetString( PyExc_ValueError, 
"Invalid argument(s) to function H3DPhysics.TrackedMField_eraseTracked( self, index, count= 1 )" );  
    return 0;
  }

  // Get field argument
  PyObject *field = PyTuple_GetItem( args, 0 );
  if( !PyObject_TypeCheck( field, &PyBaseObject_Type ) ) {
    PyErr_SetString( PyExc_ValueError, 
"Invalid Field type given as argument to H3DPhysics.TrackedMField_eraseTracked( self, index, count= 1 )" );
    return 0;
  }

  // Get index argument
  PyObject *index = PyTuple_GetItem( args, 1 );
  vector<H3DInt32> indices;
  if( PyInt_Check( index ) ) {
    indices.push_back ( PyInt_AsLong ( index ) );
  } else if ( PyList_Check( index ) ) {
    Py_ssize_t n = PyList_GET_SIZE( index );
    indices.resize( n );
    for ( int j=0; j < n; ++j ) {
      if ( PyInt_Check ( PyList_GET_ITEM( index, j ) ) ) {
        indices[j] = PyInt_AsLong ( PyList_GET_ITEM ( index, j ) );
      } else {
        PyErr_SetString( PyExc_ValueError,
                         "Invalid argument type to TrackedMField.insertRangeTracked() function " );
        return 0;
      }
    }
  } else {
    PyErr_SetString( PyExc_ValueError, 
"Invalid index type given as argument to H3DPhysics.TrackedMField_insertTracked( self, index, value )" );
    return 0;
  }

  // Get count argument
  long count= 1;
  if ( PyTuple_Size( args ) == 3 ) {
    PyObject *pyCount = PyTuple_GetItem( args, 2 );
    if( ! PyInt_Check( pyCount ) ) {
      PyErr_SetString( PyExc_ValueError, 
  "Invalid count type given as argument to H3DPhysics.TrackedMField_eraseTracked( self, index, count= 1 )" );
      return 0;
    }
    count= PyInt_AsLong ( pyCount );
  }

  PyObject *py_field_ptr = PyObject_GetAttrString( field, "__fieldptr__" );
  if( !py_field_ptr ) {
    PyErr_SetString( PyExc_ValueError, 
                      "Python object not a Field type. Make sure that if you \
have defined an __init__ function in a specialized field class, you \
call the base class __init__ function." );
    return 0;
  }
  Field *field_ptr = static_cast< Field * >
    ( PyCapsule_GetPointer( py_field_ptr, NULL ) );

  if( field_ptr && !indices.empty() ) {
    bool success;
    if ( count > 1 ) {
      APPLY_MFIELD_MACRO( field_ptr, field_ptr->getX3DType(), 
                          count, indices[0], MFIELD_ERASE, success );
    } else {
      APPLY_MFIELD_MACRO( field_ptr, field_ptr->getX3DType(), 
                          count, indices, MFIELD_ERASE_RANGE, success );
    }
    if( !success ) {
      PyErr_SetString( PyExc_ValueError, 
                        "Error: not a valid MField instance" );
      return 0;  
    }
  }

  Py_DECREF( py_field_ptr );
  Py_INCREF(Py_None);
  return Py_None;
}

#endif
