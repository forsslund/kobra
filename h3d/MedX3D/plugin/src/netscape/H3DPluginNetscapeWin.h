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

#ifndef __H3DPLUGIN_NETSCAPE_WINH
#define __H3DPLUGIN_NETSCAPE_WINH

#include "H3DPluginNetscape.h"

#ifdef H3D_WINDOWS

class H3DPluginNetscapeWin : public H3DPluginNetscape {
public:
  /// Constructor. 
  H3DPluginNetscapeWin( nsPluginCreateData* createData ):
    H3DPluginNetscape( createData ),
    m_window( NULL ),
    m_OldWndProc( NULL ) {
    m_navMenu = CreatePopupMenu();
    m_vpMenu = CreatePopupMenu();
  }

  virtual ~H3DPluginNetscapeWin();

  // class functions
  virtual void OnDestroy();
  virtual void invalidateSceenImage() {
    InvalidateRect( m_window, NULL, FALSE ); 
  }
  virtual NPError SetWindow( NPWindow* window );

  /// Callback function for creating a menu when right-button is
  /// clicked in the plugin window. x and y are the pixel coordinates
  /// the mouse was clicked on.
  virtual void OnContextMenu( int x, int y );

  WNDPROC getParentWndProc() { return m_OldWndProc; }

private:
  enum {
    ID_NAVIGATION_MENU     = 40000,
    ID_VIEWPOINT_MENU      = 40100
  };


  virtual void InternalInitWindow();
  virtual void InternalCleanupWindow();

  static LRESULT CALLBACK PluginWindowProc( HWND hWnd, UINT msg,
                                            WPARAM wParam, LPARAM lParam );
  HWND         m_window;
  WNDPROC      m_OldWndProc;
  HMENU        m_navMenu;
  HMENU        m_vpMenu;
};

#endif // H3D_WINDOWS

#endif
