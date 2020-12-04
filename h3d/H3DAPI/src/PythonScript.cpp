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
/// \file PythonScript.cpp
/// \brief cpp file for PythonScript
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/PythonScript.h>
#include <H3D/X3DTypes.h>
#include <H3D/X3DFieldConversion.h>
#include <H3D/Scene.h>
#include <H3D/X3D.h>
#include <H3D/X3DSAX2Handlers.h>
#include <H3D/MFNode.h>
#include <H3DUtil/ResourceResolver.h>
#include <H3D/GlobalSettings.h>

#ifdef HAVE_PYTHON

#include <string>
#include <algorithm>
#include <cctype>
#include <fstream>
#define DONT_HAVE_SYS_STAT_H
#undef HAVE_STAT_H

#if defined(_MSC_VER)
// undefine _DEBUG since we want to always link to the release version of
// python and pyconfig.h automatically links debug version if _DEBUG is
// defined.
#if defined _DEBUG && ! defined HAVE_PYTHON_DEBUG_LIBRARY 
#define _DEBUG_UNDEFED
#undef _DEBUG
#endif
#endif

#if defined(__APPLE__) && defined(__MACH__) && defined( HAVE_PYTHON_OSX_FRAMEWORK )
#include <Python/pyconfig.h>
#else
#ifdef __GNUC__
#ifdef _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE
#endif
#ifdef _XOPEN_SOURCE
#undef _XOPEN_SOURCE
#endif
#endif
#include <pyconfig.h>
#endif

// define HAVE_ROUND if not defined
#if _MSC_VER >= 1800
#ifndef HAVE_ROUND
#define HAVE_ROUND 1
#endif
#endif // _MSC_VER >= 1800

#if defined(__APPLE__) && defined(__MACH__) && defined(HAVE_PYTHON_OSX_FRAMEWORK)
#include <Python/Python.h>
#include <Python/pythonrun.h>
#include <Python/ceval.h>
#include <Python/moduleobject.h>
#include <Python/structmember.h>
#else
#include <Python.h>
#include <pythonrun.h>
#include <ceval.h>
#include <moduleobject.h>
#include <structmember.h>
#endif

#if defined(_MSC_VER)
// redefine _DEBUG if it was undefed
#ifdef _DEBUG_UNDEFED
#define _DEBUG
#endif
#endif

#include <H3D/PythonMethods.h>
#include <H3D/PythonTypes.h>
using namespace H3D;
using namespace X3D;


int PythonScript::argc = 0;
char ** PythonScript::argv = NULL;
wchar_t ** PythonScript::w_argv = NULL;
bool PythonScript::python_h3d_initialized = false;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase PythonScript::database(
                                       "PythonScript",
                                       &(newInstance<PythonScript>),
                                       typeid( PythonScript ),
                                       &H3DScriptNode::database );

namespace PythonScriptInternals {
  FIELDDB_ELEMENT( PythonScript, references, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( PythonScript, moduleName, INITIALIZE_ONLY )

  // The saved thread state when disallowMainThreadPython is called.
  // Will be restored upon allowMainThreadPyton call.
  PyThreadState *main_python_thread_state = NULL;

  /// The normal PyRun_String method does not allow a filename to
  /// be passed along for better error messages and trace backs.
  /// This function does the same as PyRun_String but
  /// also takes the filename where the string originated from.
  inline PyObject *PyRun_StringFilename( const char *str,
                                         const char *filename,
                                         int start,
                                         PyObject *globals,
                                         PyObject *locals) {
#if PY_MAJOR_VERSION >= 3
    PyObject *co = Py_CompileString(str, filename, start);
#else
    PyCodeObject *co = (PyCodeObject *)Py_CompileString(str, filename, start);
#endif
    if (co == NULL)
      return NULL;
    PyObject *v = PyEval_EvalCode(co, globals, locals);
    Py_DECREF(co);
    return v;
  }



}

#if PY_MAJOR_VERSION >= 3
void PythonScript::pythonSetargv() {

  if( w_argv ) {
    // input w_argv can be used directly
    PySys_SetArgv(argc,w_argv);
  } else {
    // argv as char ** but wchar_t ** required, do conversion

    // use a vector to store the data, auto_ptr or similar cannot be used
    // as they do not use delete []
    static vector< wchar_t > argv_converted;

    //check how many bytes are required for result
    size_t nr_bytes = 0;
    for ( int i = 0; i < argc; ++i ) {
      nr_bytes += mbstowcs( NULL, argv[i], 0 ) + 1;
    }


    size_t argv_ptr_part_size = sizeof( wchar_t *) * (PythonScript::argc+1);
    argv_converted.resize( nr_bytes + argv_ptr_part_size , 0 );

    wchar_t **argv_local = (wchar_t **)&(argv_converted[0]);
    size_t arg_i = argv_ptr_part_size;
    for ( int i = 0; i < argc; ++i ) {
      argv_local[i] = &(argv_converted[arg_i]);
      arg_i += mbstowcs( &(argv_converted[arg_i]), argv[i], nr_bytes + argv_ptr_part_size - arg_i ) + 1;
    }
    argv_local[argc] = NULL;
    PySys_SetArgv(PythonScript::argc, argv_local);
  }
}
#else
void PythonScript::pythonSetargv() {

  if( PythonScript::argv ) {
    PySys_SetArgv(PythonScript::argc, PythonScript::argv);
  }
  else if (w_argv) {
    // argv as wchar_t ** but char ** required, do conversion

    // use a vector to store the data, auto_ptr or similar cannot be used
    // as they do not use delete []
    static vector< char > argv_converted;

    //check how many bytes are required for result
    size_t nr_bytes = 0;
    for ( int i = 0; i < argc; ++i ) {
      nr_bytes += wcstombs( NULL, w_argv[i], 0 ) + 1;
    }

    size_t argv_ptr_part_size = sizeof( char *) * (PythonScript::argc+1);
    argv_converted.resize( nr_bytes + argv_ptr_part_size , 0 );

    char **argv_local = (char **)&(argv_converted[0]);
    size_t arg_i = argv_ptr_part_size;
    for ( int i = 0; i < argc; ++i ) {
      argv_local[i] = &(argv_converted[arg_i]);
      arg_i += wcstombs( &(argv_converted[arg_i]), w_argv[i], nr_bytes + argv_ptr_part_size - arg_i ) + 1;
    }
    argv_local[argc] = NULL;
    PySys_SetArgv(PythonScript::argc, argv_local);
  }
}
#endif


void PythonScript::setargv( int _argc, char *_argv[] ) {
  argc = _argc;
  argv = _argv;
  w_argv = NULL;
}

void PythonScript::setargv( int _argc, wchar_t *_argv[] ) {
  argc = _argc;
  argv = NULL;
  w_argv = _argv;
}

#ifdef PYTHON_USE_LOCAL_ENV
namespace {
#define XTEXTIFY(A) TEXTIFY(A)
#define TEXTIFY(A) #A

  /// Test the existence of a path
  /// \param path The path to test its existance of
  /// \param is_dir Whether or not the path should be a directory
  /// \return True if the \p path exists and respects '\p is_dir', False otherwise
  bool doesPathExist( const std::string& path, bool is_dir = true ) {
    struct stat info;
    if( stat( path.c_str(), &info ) == 0 ) {
      if( is_dir ) {
        return ( info.st_mode & S_IFDIR ) != 0;
      } else {
        return ( info.st_mode & S_IFDIR ) == 0;
      }
    }
    return false;
  }

  /// Test the existence of a Python .py module file given its path.
  /// The input should end with ".py", and the existence of the
  /// byte-compiled .pyc/.pyo versions are checked if the .py is not found.
  /// \param module_path The path of the module
  /// \return True if the python \p module_path exists, False otherwise
  bool doesPythonModuleExist( const std::string& module_path ) {
    return ( doesPathExist( module_path, false )
             || doesPathExist( module_path + 'c', false )
             || doesPathExist( module_path + 'o', false ) );
  }

  /// Get the parent directory from a folder or file path.
  /// For e.g. "/some/path/toto" => "/some/path".
  /// If the parent can't be determined, an empty string
  /// is returned.
  /// \param path The path to get the parent folder from
  /// \return The parent folder
  std::string getParent( const std::string& path ) {
    std::string result = path;
    while( result.back() == '\\' || result.back() == '/' ) {
      result.pop_back();
    }
    size_t index = result.find_last_of( '\\' );
    if( index == std::string::npos ) {
      index = result.find_last_of( '/' );
      if( index == std::string::npos ) {
        result = "";
      } else {
        result = result.substr( 0, index );
      }
    }
    else {
      result = result.substr( 0, index );
    }
    return result;
  }

}
#endif // PYTHON_USE_LOCAL_ENV

#ifndef PYTHON_USE_LOCAL_ENV
H3D_PUSH_WARNINGS()
H3D_DISABLE_UNUSED_PARAMETER_WARNING()
#endif
bool PythonScript::initPythonHome( const std::string& exe_with_full_path ) {
#ifndef PYTHON_USE_LOCAL_ENV
H3D_POP_WARNINGS()
#endif
  bool python_home_set = false;
#ifdef PYTHON_USE_LOCAL_ENV
  // Py_SetPythonHome expects a parameter which is a static buffer always accessible, thus the static keyword
  static std::string python_home;

  std::string exe_dir = getParent( exe_with_full_path );
  if( exe_dir.empty() ) {
    exe_dir = ".";
  }

  // To properly detect where the Python home should be set, we use the os.py[c|o]
  // landmark module (this is common to use this module, and also done in the
  // Python source code itself).
  // If not found, we also try to find a "pythonXY.zip" file, as it can contain the
  // python libs, and is also used as a landmark since Python 3.
#ifdef H3D_WINDOWS
  #define PYLIB_LANDMARK_OS_PY "Lib/os.py"
  #define PYLIB_LANDMARK_ZIP "python" XTEXTIFY(PY_MAJOR_VERSION) XTEXTIFY(PY_MINOR_VERSION) ".zip"
#else
  #define PYLIB_LANDMARK_OS_PY "lib/python" XTEXTIFY(PY_MAJOR_VERSION) "." XTEXTIFY(PY_MINOR_VERSION) "/os.py"
  #define PYLIB_LANDMARK_ZIP "lib/python" XTEXTIFY(PY_MAJOR_VERSION) XTEXTIFY(PY_MINOR_VERSION) ".zip"
#endif

  if( doesPythonModuleExist( exe_dir + "/" PYLIB_LANDMARK_OS_PY )
      || doesPathExist( exe_dir + "/" PYLIB_LANDMARK_ZIP, false ) ) {
    python_home = exe_dir;
  } else if( doesPythonModuleExist( exe_dir + "/../" PYLIB_LANDMARK_OS_PY )
              || doesPathExist( exe_dir + "/../" PYLIB_LANDMARK_ZIP, false ) ) {
    python_home = exe_dir + "/..";
  }

  if( !python_home.empty() ) {
    Console( H3DUtil::LogLevel::Debug ) << "Setting the Python home to " << python_home << std::endl;
    Py_SetPythonHome( const_cast<char*>( python_home.c_str() ) );
    python_home_set = true;
  } else {
    Console( H3DUtil::LogLevel::Error ) << "Could not find the local Python home" << std::endl;
  }
#endif // PYTHON_USE_LOCAL_ENV
  return python_home_set;
}

void PythonScript::allowMainThreadPython() {
  using namespace PythonScriptInternals;
  if( main_python_thread_state ) {
    PyEval_RestoreThread( main_python_thread_state );
    main_python_thread_state = NULL;
  }
}

bool PythonScript::mainThreadPythonAllowed() {
  return PythonScriptInternals::main_python_thread_state == NULL;
}

void PythonScript::disallowMainThreadPython() {
  using namespace PythonScriptInternals;
  if( !main_python_thread_state ) {
    main_python_thread_state = PyEval_SaveThread();
  }
}

PyObject *PythonScript::getPythonAttribute( const string &_name ) {
  if( module_dict ) {
    // ensure we have the GIL lock to work with multiple python threads.
    PyGILState_STATE state = PyGILState_Ensure();
    PyObject *fname =
      PyDict_GetItemString( static_cast< PyObject * >( module_dict ),
                            _name.c_str() );
    PyGILState_Release(state);
    return fname;
  }
  return NULL;
}

Field *PythonScript::lookupField( const string &_name) const {
  if( module_dict ) {
    // ensure we have the GIL lock to work with multiple python threads.
    PyGILState_STATE state = PyGILState_Ensure();
    PyObject *fname =
      PyDict_GetItemString( static_cast< PyObject * >( module_dict ),
                            _name.c_str() );
    if ( fname ) {
      // it was a variable in the python script, so extract the C++ type
      // pointer and return it
      PyObject *fieldptr = PyObject_GetAttrString( fname, "__fieldptr__" );  // new ref

      if ( fieldptr ) {
        if( PyCapsule_CheckExact( fieldptr ) ) {
          Field *f = getFieldPointer( fieldptr );
          Py_DECREF( fieldptr );
          PyGILState_Release(state);
          return f;
        }
      } else {
        // PyObject_GetAttrString sets an error when returning NULL. Clear it here to ignore it.
        PyErr_Clear();
      }
    }
    PyGILState_Release(state);
  }
  return NULL;
}


void PythonScript::getTopLevelFields( vector< pair< string, Field *> > &fields ) {
  if( module_dict ) {
    // ensure we have the GIL lock to work with multiple python threads.
    PyGILState_STATE state = PyGILState_Ensure();

    PyObject *key, *value;
    Py_ssize_t pos = 0;

    // add all fields from the dictionary.
    while (PyDict_Next(static_cast< PyObject * >( module_dict ), &pos, &key, &value)) {
      // key and value are borrowed references

      // new reference
      PyObject *fieldptr = PyObject_GetAttrString( value, "__fieldptr__" ); // new ref

      if ( fieldptr ) {
        if( PyCapsule_CheckExact( fieldptr ) && PyString_Check( key ) ) {
          Field *f = getFieldPointer( fieldptr );
          string _name = PyString_AsString( key );
          fields.push_back( make_pair( _name, f ) );
        }
        Py_DECREF( fieldptr );
      } else {
        // PyObject_GetAttrString sets an error when returning NULL. Clear it here to ignore it.
        PyErr_Clear();
      }
    }

    PyGILState_Release(state);
  }
}





PythonScript::PythonScript( Inst< MFString > _url,
                            Inst< MFNode    > _references,
                            Inst< SFString > _moduleName ) :
  H3DScriptNode( _url ),
  references( _references ),
  moduleName( _moduleName ),
  module( NULL ),
  module_dict( NULL ) {
  type_name = "PythonScript";
  database.initFields( this );

  addInlinePrefix( "python" );

  // Py_Initialize really should be done in the DLL loader function:
  if ( !Py_IsInitialized() ) {
#if PY_MAJOR_VERSION >= 3
    PyImport_AppendInittab("H3D", PythonInternals::PyInit_H3D);
#endif
    Py_Initialize();

    // Decide if we should initialize multi-threaded C API usage
    bool multi_threaded= false;
    if ( GlobalSettings* gs= GlobalSettings::getActive() ) {
      multi_threaded= gs->multiThreadedPython->getValue();
    }
    if ( multi_threaded ) {
      PyEval_InitThreads();
    }

    pythonSetargv();

    disallowMainThreadPython();
  }

  // ensure we have the GIL lock to work with multiple python threads.
  PyGILState_STATE state = PyGILState_Ensure();
  python_h3d_initialized = initialiseParser();
  PyGILState_Release(state);
}

// Find python modules in dictionary and store them and the python reference counter
// in module_names.
void findModulesInDict( void * _dict, list< pair< string, Py_ssize_t > > &module_names ) {
  PyObject *key, *value;
  Py_ssize_t pos = 0;
  while( PyDict_Next( static_cast< PyObject * >( _dict ),
                      &pos, &key, &value ) ) {

    if( PyString_Check( key ) && PyModule_Check( value ) ) {
      // The name is a module. add it to imported_module_names.
      string mod_name = PyString_AsString( key );
      if( mod_name == "H3D" || mod_name == "H3DInterface" || mod_name == "H3DUtils" ) {
        // Ignore the H3D python modules so they are not accidentally removed.
        continue;
      }
      list< pair< string, Py_ssize_t > >::iterator i = module_names.begin();
      for( ; i != module_names.end(); ++i )
        if( (*i).first == mod_name )
          break;
      if( i == module_names.end() && value->ob_refcnt > 1 ) {
        PyObject *tmp_module_dict = PyModule_GetDict( value ); // borrowed ref
        if( PyDict_Contains( tmp_module_dict, PyString_FromString( "__scriptnode__" ) ) == 0 ) { // Only consider modules which are not added by a PythonScript node.
          pair< string, Py_ssize_t > data;
          data.first = mod_name;
          data.second = value->ob_refcnt;
          module_names.push_back( data );
        }
      }
    }
  }
}

PythonScript::~PythonScript() {
  if( !Py_IsInitialized() ) {return;}
 // ensure we have the GIL lock to work with multiple python threads.
  PyGILState_STATE state = PyGILState_Ensure();

  if( module_dict ) {

    // Call an onExit function, if one is defined
    PyObject *func =
      PyDict_GetItemString( static_cast< PyObject * >( module_dict ),
                            "onExit" );
    if( func && PyFunction_Check( func ) ) {
      PyObject *args = PyTuple_New(0);
      PyObject *result = PyEval_CallObject( func, args );
      if ( result == NULL ) {
        PyErr_Print();
      } else {
        Py_DECREF( result );
      }
      Py_DECREF( args );
    }


    PyObject *temp_sys_module_dict = PyImport_GetModuleDict();
    // Find all modules with a reference count above 1. Use the
    // imported_module_names vector later to remove python modules included
    // by this PythonScript.
    list< pair< string, Py_ssize_t > > imported_module_names;
    findModulesInDict( temp_sys_module_dict, imported_module_names );

    // Setting module_dict to null just to be on the safe side. It should not
    // really be needed.
    module_dict = NULL;

    // Removing the PythonScript module module_name from database.
    // If it is already removed then there are two PythonScripts in the
    // scene using the same module_name ( or DEF ).
    if( PyDict_DelItemString( temp_sys_module_dict,
                              (char*)module_name.c_str() ) == -1 ) {
      Console(LogLevel::Error) << "Could not remove the python module " << module_name
                 << " from the sys.modules database. " << endl;
    } else {
      // Starting from python 3.4, python no longer forcibly break cycles 
      // through the module globals when the module is deallocated
      // check https://bugs.python.org/issue28202 for details
#if PY_VERSION_HEX >= ((3 << 24) | (4 << 16) | (0 <<  8))
      PyGC_Collect();
#endif
    }

    // Go through imported_module_names. Those modules whose reference count
    // has decreased to 1 only belonged to this PythonScript and should be
    // removed from the global module list. Note that this
    // code will not take care of circular imports. That is, if module A
    // imports module B while module B imports module A.
    list< pair< string, Py_ssize_t > >::iterator i = imported_module_names.begin();
    while( i != imported_module_names.end() ) {
      PyObject *module_to_check = PyDict_GetItemString( temp_sys_module_dict,
                                                        (char*)(*i).first.c_str() );
      if( module_to_check && PyModule_Check( module_to_check ) ) {
        if( module_to_check->ob_refcnt < (*i).second &&
            module_to_check->ob_refcnt == 1 ) {
          // Remove module.
          if ( PyDict_DelItemString( temp_sys_module_dict,
            (*i).first.c_str() )==0 ) {
#if PY_VERSION_HEX >= ((3 << 24) | (4 << 16) | (0 <<  8))
            // Starting from python 3.4, python no longer forcibly break cycles 
            // through the module globals when the module is deallocated
            // check https://bugs.python.org/issue28202 for details
            PyGC_Collect();
#endif
          }
          imported_module_names.erase( i );
          // There might still be modules left, restart the loop since the
          // removal of this module might have lowered the reference count for
          // other module.
          if( imported_module_names.empty() )
            break;
          i = imported_module_names.begin();
        } else
          ++i;
      } else {
        // This module does not exist anymore, simply remove it from the list.
        list< pair< string, Py_ssize_t > >::iterator to_erase = i;
        ++i;
        imported_module_names.erase( to_erase );
      }
    }
  }

  PyGILState_Release(state);
}

bool PythonScript::initialiseParser() {
  H3DScriptNode::initialiseParser();
  return PythonInternals::initH3DInternal() != NULL;
}

void PythonScript::loadScript( const string &script_filename, const string &script_content ) {
  if(!python_h3d_initialized ) {
    Console( LogLevel::Error ) << "Python error while loading \"" << script_filename << "\":" << endl;
    Console( LogLevel::Error ) << "Python module H3D not initialized. "<< endl;
    return;
  }
  // ensure we have the GIL lock to work with multiple python threads.
  PyGILState_STATE state = PyGILState_Ensure();
  PyObject *ref = (PyObject*)PythonInternals::fieldAsPythonObject( references.get(), false );
  PyDict_SetItem( (PyObject *)module_dict,
                  PyString_FromString( "references" ),
                  ref );
  Py_DECREF( ref );

  if (PyDict_GetItemString( static_cast< PyObject * >(module_dict), "__builtins__") == NULL) {
    if (PyDict_SetItemString( static_cast< PyObject * >(module_dict), "__builtins__",
                             PyEval_GetBuiltins()) != 0)
      Console(LogLevel::Warning) << "Warning: PyEval_GetBuiltins() could not be installed in module dictionary!" << endl;
  }

  PyDict_SetItemString(
    static_cast<PyObject *>(module_dict), "__file__",
    PyString_FromString( script_filename.c_str() ) );

  if ( script_content != "" ) {
    PyErr_Clear();
    PyObject *r =
      PythonScriptInternals::PyRun_StringFilename( script_content.c_str(), script_filename.c_str(),
                                                   Py_file_input,
                                                   static_cast< PyObject * >(module_dict),
                                                   static_cast< PyObject * >(module_dict) );

    if ( r == NULL ) {
      Console( LogLevel::Error ) << "Python error in file \"" << script_filename << "\":" << endl;
      PyErr_Print();
    }
  } else {
#ifdef H3D_WINDOWS
    // have to read the script into a buffer instead of using FILE *
    // since it is unsafe to use FILE * sent over DLL boundaries.
    ifstream is( script_filename.c_str() );
    if( is.good() ) {
      int length;
      char * buffer;

      // get length of file:
      is.seekg (0, ios::end);
      length = static_cast<int>(is.tellg());
      is.seekg (0, ios::beg);

      // allocate memory:
      buffer = new char [length + 1];
      // read data as a block:
      is.read (buffer,length);
      length = static_cast<int>(is.gcount());
      is.close();
      buffer[length ] = '\0';

      PyErr_Clear();
      PyObject *r =
        PythonScriptInternals::PyRun_StringFilename( buffer, script_filename.c_str(),
                                               Py_file_input,
                                               static_cast< PyObject * >(module_dict),
                                               static_cast< PyObject * >(module_dict) );

      if ( r == NULL ) {
        Console( LogLevel::Error ) << "Python error in file \"" << script_filename << "\":" << endl;
        PyErr_Print();
      }
      delete[] buffer;
    }
#else
    FILE *f = fopen( script_filename.c_str(), "r" );
    if ( f ) {
      PyErr_Clear();
      PyObject *r = PyRun_FileEx( f, script_filename.c_str(), Py_file_input,
                                  static_cast< PyObject * >(module_dict),
                                  static_cast< PyObject * >(module_dict),
                                  true );
      if ( r == NULL )
        PyErr_Print();


    }

#endif // H3D_WINDOWS
    else {
      Console(LogLevel::Error) << "Could not open \""<< script_filename << endl;
    }
  }

  PyGILState_Release(state);
}

std::string PythonScript::execute ( const std::string& _command ) {
  std::string result;

  PyGILState_STATE state = PyGILState_Ensure();

  // Temporary redirect of pyton output
  std::string stdOutErr =
"import sys\n\
class CatchOutErr:\n\
    def __init__ ( self ):\n\
        self.value = ''\n\
        self.stdout = sys.stdout\n\
        self.stderr = sys.stderr\n\
        sys.stdout = self\n\
        sys.stderr = self\n\
\n\
    def write(self, txt):\n\
        self.value += txt\n\
\n\
    def restore ( self ):\n\
        sys.stdout = self.stdout\n\
        sys.stderr = self.stderr\n\
\n\
catchOutErr = CatchOutErr ()\n\
";

  PyObject* r= PyRun_StringFlags (
    stdOutErr.c_str(),
    Py_file_input,
    static_cast< PyObject * >(module_dict),
    static_cast< PyObject * >(module_dict),
    NULL );
  if ( r == NULL ) {
    Console(LogLevel::Error) << "Python console error!" << endl;
    PyErr_Print ();
    PyGILState_Release(state);
    return "ERROR: Internal console error!\n";
  } else {
    Py_DECREF ( r );
  }

  // Execute
  r= PyRun_StringFlags (
    _command.c_str(),
    Py_single_input,
    static_cast< PyObject * >(module_dict),
    static_cast< PyObject * >(module_dict),
    NULL );

  PyObject *catcher = PyObject_GetAttrString(static_cast< PyObject * >(module),"catchOutErr"); // new ref

  // Format the result/output
  if ( r ) {
    PyObject *output = PyObject_GetAttrString(catcher,"value"); // new ref
    result= PyString_AsString(output);
    Py_DECREF ( output );

    if ( result.empty () ) {
      if ( r != Py_None ) {
        PyObject* str= PyObject_Str ( r );
        if ( str ) {
          result= PyString_AsString( str );

          Py_DECREF ( str );
        }
      }
    }
    Py_DECREF ( r );
  } else {
    PyErr_Print();
    PyObject *output = PyObject_GetAttrString(catcher,"value"); // new ref
    result = PyString_AsString(output);
    Py_DECREF ( output );
  }

  Py_DECREF ( catcher );

  // Restore stdout/stderr
  r= PyRun_StringFlags (
    "catchOutErr.restore ()",
    Py_file_input,
    static_cast< PyObject * >(module_dict),
    static_cast< PyObject * >(module_dict),
    NULL );
  if ( r == NULL ) {
    Console(LogLevel::Error) << "Python console error!" << endl;
    PyErr_Print ();
    PyGILState_Release(state);
    return "ERROR: Internal console error!\n";
  } else {
    Py_DECREF ( r );
  }

  PyGILState_Release(state);

  return result;
}

// Traverse the scenegraph. Used in PythonScript to call a function
// in python once per scene graph loop.
void PythonScript::traverseSG( TraverseInfo &/*ti*/ ) {
#ifdef HAVE_PROFILER
  string timer_string = "PythonScript traverseSG (" + (url->size() > 0 ? url->getValue()[0]:"" ) + ")";
  if( H3D::Profiling::profile_python_fields ) {
    H3DUtil::H3DTimer::stepBegin(timer_string, "PYTHON");
  }
#endif
  // ensure we have the GIL lock to work with multiple python threads.
  PyGILState_STATE state = PyGILState_Ensure();

  PyObject *func =
    PyDict_GetItemString( static_cast< PyObject * >( module_dict ),
                          "traverseSG" );
  if( func && PyFunction_Check( func ) ) {
    PyObject *args = PyTuple_New(0);
    PyObject *result = PyEval_CallObject( func, args );
    if ( result == NULL )
      PyErr_Print();
    else
      Py_DECREF( result );

    Py_DECREF( args );
  }

  PyGILState_Release(state);
#ifdef HAVE_PROFILER
  if( H3D::Profiling::profile_python_fields ) {
    H3DUtil::H3DTimer::stepEnd(timer_string);
  }
#endif
}

void PythonScript::initialize() {
#ifdef HAVE_PROFILER
  string timer_string = "PythonScript initialize (" + (url->size() > 0 ? url->getValue()[0]:"" ) + ")";
  if( H3D::Profiling::profile_python_fields ) {
    H3DUtil::H3DTimer::stepBegin(timer_string, "PYTHON");
  }
#endif
  module_name = moduleName->getValue();

  // if no module name set by user, use the instance name which is
  // a unique identifier for each PythonScript instance.
  if( module_name == "" ) {
    module_name = getInstanceName();
    moduleName->setValue( module_name, id );
  }

  H3DScriptNode::initialize();

  // ensure we have the GIL lock to work with multiple python threads.
  PyGILState_STATE state = PyGILState_Ensure();

  PyObject *temp_sys_module_dict = PyImport_GetModuleDict(); // borrowed ref
  if( PyDict_GetItemString( temp_sys_module_dict,
                            (char*)module_name.c_str() ) ) { // borrowed ref
    Console(LogLevel::Error) << "The module " << module_name << " already exists. "
               << "It will be overridden which might cause strange behaviour. "
               << "Check the moduleName field of PythonScript "
               << "in the scene if this behaviour is undesired." << endl;
  }


  module = PyImport_AddModule( (char*)module_name.c_str() ); // borrowed ref

  if (!module) {
    PyErr_Print();
  } else {
    module_dict =
      PyModule_GetDict( static_cast< PyObject * >( module ) ); // borrowed ref


  // Add __scriptnode__ to module dictionary which is the node the script resides in.
  // Do not let the PyNode hold a reference count for the PythonScript node, otherwise
  // it could never be deleted.
    PyObject *scriptnode = PyNode_FromNode( this, false );

    PyDict_SetItem( (PyObject *)module_dict,
                    PyString_FromString( "__scriptnode__" ),
                    scriptnode);
    Py_DECREF( scriptnode );

    bool script_loaded = false;
    for( MFString::const_iterator i = url->begin(); i != url->end(); ++i ) {
      // First try to resolve URL to file contents, if that is not supported
      // by the resolvers then fallback to resolve as local filename
      string url_contents = resolveURLAsString( *i );
      bool is_tmp_file = false;
      string _url;
      if( url_contents == "" ) {
        _url = resolveURLAsFile( *i, &is_tmp_file );
      }
      if( _url != "" || url_contents != "" ) {
        std::string script_filename = _url.empty() ? *i : _url;
        if( getInlinedContentOffset( *i ) > 0 ) {
          // If the script is inlined we don't want to use the script contents as the filename
          // used in error messages etc., so define our own
          script_filename = "<Inlined from " + getName() + ">";
        }

        loadScript( script_filename, url_contents );
        if( is_tmp_file ) ResourceResolver::releaseTmpFileName( _url );
        setURLUsed( *i );
        script_loaded = true;
        break;
      }
    }

    if( script_loaded ) {
      PyObject *func =
        PyDict_GetItemString( static_cast< PyObject * >( module_dict ), "initialize" );
      if( func && PyFunction_Check( func ) ) {
        PyObject *args = PyTuple_New( 0 );
        PyObject *result = PyEval_CallObject( func, args );
        if( result == NULL ) {
          PyErr_Print();
        } else {
          Py_DECREF( result );
        }

        Py_DECREF( args );
      }
    } else {
      Console( LogLevel::Error ) << "Warning: None of the urls in the PythonScript node \""
                                 << getName() << "\" with url [";
      for( MFString::const_iterator i = url->begin();
           i != url->end(); ++i ) {
        Console( LogLevel::Error ) << " \"" << *i << "\"";
      }
      Console( LogLevel::Error ) << "] could be found. " << endl;
    }
  }
  PyGILState_Release(state);
#ifdef HAVE_PROFILER
  if( H3D::Profiling::profile_python_fields ) {
    H3DUtil::H3DTimer::stepEnd(timer_string);
  }
#endif
}

#endif // HAVE_PYTHON
