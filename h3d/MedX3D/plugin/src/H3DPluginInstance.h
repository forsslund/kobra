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
/// \file H3DPluginInstance.h
/// \brief Header file for H3DPluginInstance, contains functionality of the
/// plugin.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DPLUGIN_INSTANCE_H
#define __H3DPLUGIN_INSTANCE_H

#include "PluginWindow.h"
#include "PluginWindowOSX.h"
#include "PluginWindowLinux.h"

#include <H3DUtil/Console.h>
#include <H3D/NavigationInfo.h>
#include <H3D/KeySensor.h>
#ifdef PRINT_ERR_FILE
#include <fstream>
#endif

#ifdef H3D_OSX
#include <AGL/agl.h>
#endif

using namespace std;
using namespace H3D;

class H3DPluginInstance
{
public:
  H3DPluginInstance();
  ~H3DPluginInstance();

#ifdef H3D_WINDOWS
  bool initWindow( HWND window, int _width, int _height );
#endif

#ifdef H3D_OSX
  bool initWindow( WindowRef window, int _width, int _height );
#endif

#ifdef H3D_LINUX
  bool initWindow( Display *display, Window window, Visual *visual,
       int _width, int _height );
#endif

  bool initializedWindow(){ return window_initialized; }
  

#ifdef PRINT_ERR_FILE
  std::ofstream errfile;
  class PrintingCerrInfo {
  public:
    PrintingCerrInfo() {
      startCerrThing();
    }
    ~PrintingCerrInfo() {
      endCerrThing();
    }
    //  protected:
    static std::ofstream* cerrfile;
    static std::streambuf* cerrbuf;
    static bool cerrfile_initialized;

    static void startCerrThing() {
      if( !cerrfile_initialized ) {
        cerrfile_initialized = true;
#ifdef H3D_WINDOWS
        cerrfile = new std::ofstream("C:/cerrInfo.log");
#endif

#ifdef H3D_OSX
        cerrfile = new std::ofstream("/Users/Knok/cerrInfo.log");
#else
        cerrfile = new std::ofstream("/home/daniel/cerrInfo.log");
#endif
        std::streambuf* errbuf = cerrfile->rdbuf();
        cerrbuf = std::cerr.rdbuf();
        std::cerr.rdbuf(errbuf);
      }
    }
    static void endCerrThing() {
      if( cerrfile_initialized ) {
        std::cerr.rdbuf(cerrbuf);
        cerrfile_initialized = false;
        delete cerrfile;
      }
    }
  };
#endif

  // Small wrapper to provide some convenience for managing 'menu list entries'
  class MenuList {
  public:
    MenuList() {
      m_items = new vector<string>();
      m_enabled = false; m_checked = "Unavailable";
    }

    ~MenuList() { m_items->clear(); delete m_items; }

    inline void setChecked( string item, bool enabled ) {
      m_checked = item;m_enabled = enabled;
    }

    inline string getChecked() { return m_checked; }

    inline void enableChecked( bool enable = true ) {m_enabled = enable;}
    inline bool isCheckedEnabled() { return m_enabled; }
    
    inline vector<string>* getItems() { return m_items; }
    inline void setItems( vector<string> *items ) {
      m_items->assign(items->begin(), items->end());
    }
    inline void addItem( string item ) { m_items->push_back(item); }

    inline unsigned int getCount() { return m_items->size(); }

    inline void reset() {
      m_items->clear();
      m_checked = "Unavailable";
      m_enabled = false;
    }

  private:
    vector<string>*  m_items;
    string      m_checked;
    bool      m_enabled;
  };

  
  enum {
    ID_NAVIGATION_MENU  = 1,
    ID_VIEWPOINT_MENU  = 2
  }; 


  bool Start( string theFileToLoad );
  void Stop();

  void Update();
  bool IsStarted() const { return m_IsStarted; }

  /* onEvent wrappers */
  // onMenuCommand for menu selections
  void onMenuCommand( int menu, string cmd );
  // Mouse movement
  void onMouseMotion( int x, int y );
  // Mouse buttons
  void onLeftMouseButtonDown();
  void onLeftMouseButtonUp();
  void onMiddleMouseButtonDown();
  void onMiddleMouseButtonUp();
  void onRightMouseButtonDown();
  void onRightMouseButtonUp();
  void onMouseWheel( int rotation );

  // Keys (See KeySensor::ActionKeys)
  void onKeyDown( int key, bool special );
  void onKeyUp( int key, bool special );

  inline MenuList* getNavigationMenuList() { return m_navigationItems; }
  inline MenuList* getViewpointMenuList() { return m_viewpointItems; }

  void resetRoot();

  void setPluginWindowSize( int width, int height ) {
    if( m_RenderWindow ) {
      m_RenderWindow->width->setValue( width );
      m_RenderWindow->height->setValue( height );
    }
  }

private:

  bool validateNavType( string type );
  void buildNavigationMenuList();
  void buildViewpointMenuList();
  void onViewpointChange( string cmd );
  void onNavigationChange( string cmd );

  bool              m_IsStarted;
  bool              window_initialized;

  H3DWindowNode *          m_RenderWindow;

  AutoRef< Scene >        m_Scene;
  NavigationInfo *        m_navigation;
  string              m_NavigationItem;
  AutoRef< Node >          m_viewpoint;
  X3DViewpointNode::ViewpointList m_viewpointList;
  H3DPluginInstance::MenuList*  m_navigationItems;
  H3DPluginInstance::MenuList*  m_viewpointItems;
  
  // Counters
  int                m_navTypeCount;
  int                m_viewpointCount;
};


#endif
