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
/// \file Python3Compatibility.h
/// \brief Defines and function for easy porting to Python 3
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PYTHON3COMPATIBILITY_H__
#define __PYTHON3COMPATIBILITY_H__

#include <H3D/H3DApi.h>
#include <H3D/Field.h>
#if defined(_MSC_VER)
// undefine _DEBUG since we want to always link to the release version of
// python and pyconfig.h automatically links debug version if _DEBUG is
// defined.
#if defined _DEBUG && ! defined HAVE_PYTHON_DEBUG_LIBRARY 
#define _DEBUG_UNDEFED
#undef _DEBUG
#endif

// define HAVE_ROUND if not defined
#if _MSC_VER >= 1800
#ifndef HAVE_ROUND
#define HAVE_ROUND 1
#endif
#endif // _MSC_VER >= 1800

#endif
#if defined(__APPLE__) && defined(__MACH__) && defined( HAVE_PYTHON_OSX_FRAMEWORK )
#include <Python/Python.h>
#else
#ifdef __GNUC__
#ifdef _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE
#endif
#ifdef _XOPEN_SOURCE
#undef _XOPEN_SOURCE
#endif
#endif
#include <Python.h>
#endif
#if defined(_MSC_VER)
// redefine _DEBUG if it was undefed
#ifdef _DEBUG_UNDEFED
#define _DEBUG
#endif
#endif

// Get Field pointer encapsulated in an PyCapsule object.
inline H3D::Field *getFieldPointer(PyObject * po) {
  return static_cast< H3D::Field * >
    ( PyCapsule_GetPointer( po, NULL ) );

}

#if PY_MAJOR_VERSION >= 3

inline PyObject *Py_FindMethod(PyMethodDef[], PyObject *o, const char *name){ 
  PyObject *nameobj = PyUnicode_FromString(name);
  return PyObject_GenericGetAttr((PyObject *)o, nameobj);
}

// replace PyInt with PyLong variants
#define PyInt_Check PyLong_Check
#define PyInt_FromLong PyLong_FromLong
#define PyInt_AsLong PyLong_AsLong
#define PyInt_FromSize_t PyLong_FromSize_t

// removed in python 3, used in bit masks. set to 0 to do nothing 
// if used in such masks
#define  Py_TPFLAGS_CHECKTYPES 0

// replace PyString with PyUnicode
#define PyString_Check PyUnicode_Check
#define PyString_AsString(X) ( PyBytes_AsString( PyUnicode_AsUTF8String(X) )  )
#define PyString_FromString(X) PyUnicode_FromString(X)
#endif //PY_MAJOR_VERSION >= 3


#endif
