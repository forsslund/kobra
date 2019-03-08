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

#ifndef __H3DPLUGIN_NETSCAPE_LINUXH
#define __H3DPLUGIN_NETSCAPE_LINUXH

#include "H3DPluginNetscape.h"

#ifdef H3D_LINUX

#include <signal.h>
#include <X11/Intrinsic.h>
#include <X11/Xlib.h>

class H3DPluginNetscapeLinux : public H3DPluginNetscape {
public:
  /// Constructor. 
  H3DPluginNetscapeLinux( nsPluginCreateData* createData ):
  H3DPluginNetscape( createData ) {

  }

  /// Destructor.
  virtual ~H3DPluginNetscapeLinux();

  // class functions
  virtual void OnDestroy();
  virtual void invalidateSceenImage() {
    //InvalidateRect( m_window, NULL, FALSE ); 
  }
  virtual NPError SetWindow( NPWindow* window );


  /// Callback function for creating a menu when right-button is
  /// clicked in the plugin window. x and y are the pixel coordinates
  /// the mouse was clicked on.
  //virtual void OnContextMenu( int x, int y );

protected:

  int translateXKey( KeySym &keysym, bool &is_special );

#ifdef USE_TIMER
  static void timerCallbackFunc (XtPointer arg, XtIntervalId *xtinterval);
#endif

  static void handleMessages (Widget xtwidget, 
            H3DPluginNetscapeLinux *plugin, 
            XEvent *xevent, 
            Boolean *b);

  //static void handleMessages (Widget xtwidget, 
  //nsPluginInstance *plugin, 
  //XEvent *xevent, Boolean *b);

  virtual void InternalInitWindow();
  virtual void InternalCleanupWindow();

#ifdef USE_TIMER
  XtIntervalId  xtinterval;
#endif
  Display *display;
  XtAppContext  xtcontext;
};

#endif // H3D_LINUX

#endif
