//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file H3DPluginNetscape.h
/// \brief header file for H3DPluginNetscape. This class and some static
/// functions handles keyboard bindings for the plugin for netscape.
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DPLUGIN_NETSCAPE_H
#define __H3DPLUGIN_NETSCAPE_H


#include <string>

#include "../H3DPluginInstance.h"


#include "npapi/pluginbase.h"
#include <H3D/H3DApi.h>


/// NPAPI plugin.
class H3DPluginNetscape : public nsPluginInstanceBase
{
public:
  H3DPluginNetscape( nsPluginCreateData* createData );
  virtual ~H3DPluginNetscape();

  // nsPluginInstanceBase virtual functions
  virtual NPBool  init( NPWindow* window );
  virtual NPError SetWindow( NPWindow* window );
  virtual void    shut();
  virtual NPBool  isInitialized();
  virtual NPError NewStream( NPMIMEType type, NPStream* stream,
                             NPBool seekable, uint16* stype );


  // class functions
  virtual void render();
  virtual void invalidateScreenImage() {}
  virtual void OnDestroy();


  /// Callback function for creating a menu when right-button is
  /// clicked in the plugin window. x and y are the pixel coordinates
  /// the mouse was clicked on.
  virtual void OnContextMenu( int x, int y ) {}

  void onMenuCommand( int cmd );

  H3DPluginInstance *getPluginInstance() { return &m_InstanceThread; }

  bool isRunning() { return m_InstanceThread.IsStarted(); }

  void setControlClass( bool _is_control_class );
  void printControlClass();
  inline bool getIsControlClass() {
    return is_control_class;
  }

  inline void removeInstanceThread() {
    m_InstanceThread.resetRoot();
  }

  inline NPWindow *getNPWindow() {
    return m_np_window;
  }

protected:
  enum {
    ID_NAVIGATION_MENU     = 40000,
    ID_VIEWPOINT_MENU      = 40100
  };

  static list< H3DPluginNetscape * > pn_instances;
  bool is_control_class;

  virtual void InternalInitWindow();
  virtual void InternalCleanupWindow();

  NPP          m_Instance;
  NPWindow *   m_np_window;
  NPBool       m_Initialized;
  unsigned int m_width;
  unsigned int m_height;
  std::string  m_fileToLoad;
  bool         m_hasSrc;

  H3DPluginInstance  m_InstanceThread;
};

static H3DPluginNetscape *controlclass = NULL;

#endif
