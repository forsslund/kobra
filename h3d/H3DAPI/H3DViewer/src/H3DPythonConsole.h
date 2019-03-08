//////////////////////////////////////////////////////////////////////////////
//    Copyright 2009-2019, SenseGraphics AB
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
/// \file H3DPythonConsole.h
/// \brief Header file for H3DPythonConsole.
/// Subclass of PluginsDialog, which is generated by wxFormBuilder.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DPYTHONCONSOLE__
#define __H3DPYTHONCONSOLE__

#include "H3DViewer.h"
#include "H3DViewerConfig.h"
#include <H3D/PythonScript.h>

#if defined(HAVE_PYTHON) && defined(USE_PYTHON_CONSOLE)

/** Implementing PythonConsole */
class H3DPythonConsole : public PythonConsole
{
public:
  H3DPythonConsole ( 
    wxWindow* parent, 
    H3D::PythonScript& _pythonNode );

protected:

  // Handlers
  virtual void onKeyDown( wxKeyEvent& event );
  virtual void onExecuteCommand ( wxCommandEvent& event );

  H3DUtil::AutoRef < H3D::PythonScript > python_node;

  typedef std::vector < std::string > CommandVec;
  CommandVec command_history;

  int command_index;
};

#endif // HAVE_PYTHON
#endif // __H3DPYTHONCONSOLE__
