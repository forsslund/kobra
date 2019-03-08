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
/// \file H3DPluginNetscape.cpp
/// \brief CPP file for H3DPluginNetscape. This class and some static functions
/// handles keyboard bindings for the plugin for netscape.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "Resource.h"
#include "H3DPluginNetscape.h"
#include "../keys.h"
#include <cassert>
//#include <windowsx.h>
#include <vector>
#include "H3D/X3DTextureNode.h"
#include <H3D/ProfilesAndComponents.h>
#ifdef HAVE_MEDX3D
#include <H3D/MedX3D/VolumeData.h>
#endif

#ifdef PRINT_ERR_FILE
H3DPluginInstance::PrintingCerrInfo printing_cerr_info;
#endif
list< H3DPluginNetscape * > H3DPluginNetscape::pn_instances;
/**
 * Hook function needs to know how to access the controlclass
 */
//H3DPluginNetscape *controlclass = NULL;


/**
 * Plugin initialization functions
 *
 */
#ifdef PRINT_ERR_FILE
static bool running_cerr_thing = false;
#endif

// ---------------------------------------------------------------------------
/**
 * Plugin class functions
 *
 */
H3DPluginNetscape::H3DPluginNetscape( nsPluginCreateData* createData )
:  m_Instance( createData->instance )
,  m_Initialized( false )
,  m_np_window( NULL )
,  m_hasSrc( false ),
is_control_class( false )
{
  m_fileToLoad = "";

  // Just check if the src attribute exists. We assign the real value
  // (since this includes the absolute path) later in NewStream
  for( int i = 0; i < createData->argc; ++i ) {
    if( strcmp( createData->argn[i], "src" ) == 0 ) {
      m_fileToLoad = string( createData->argv[i] );
      if( m_fileToLoad.compare( "" ) == 0 ) {
        // MessageBox( HWND_DESKTOP, "src-attribute found but contains no
        // filename. Required to load x3d-file.", "Error", MB_ICONERROR);
      }
    }
  }

  pn_instances.push_back( this );
#ifdef PRINT_ERR_FILE
  cerr << "H3DPluginNetscape " << this << endl;
  cerr << "pn_instances: " << endl;
  for( list< H3DPluginNetscape * >::iterator i = pn_instances.begin();
         i != pn_instances.end(); ++i ) {
           cerr << (*i) << " ";
         }
         cerr << endl;
#endif
}


H3DPluginNetscape::~H3DPluginNetscape()
{
#ifdef PRINT_ERR_FILE
  cerr << "~H3DPluginNetscape " << this << endl;
#endif
  OnDestroy();
  pn_instances.remove( this );
}

void H3DPluginNetscape::setControlClass( bool _is_control_class ) {
  if( _is_control_class ) {
    for( list< H3DPluginNetscape * >::iterator i = pn_instances.begin();
         i != pn_instances.end(); ++i ) {
      if( (*i) == this ) {
        (*i)->is_control_class = true;
      }
      else {
        (*i)->is_control_class = false;
      }
    }
  }
  else {
    is_control_class = false;
  }
}

void H3DPluginNetscape::printControlClass() {
#ifdef PRINT_ERR_FILE
  for( list< H3DPluginNetscape * >::iterator i = pn_instances.begin();
       i != pn_instances.end(); ++i ) {
    cerr << "control class: " << (*i) << " " << (*i)->is_control_class << endl;
  }
#endif
}

NPError H3DPluginNetscape::NewStream( NPMIMEType type, NPStream* stream, 
                                NPBool seekable, uint16* stype )
{ 
  // Get the url
  m_fileToLoad = string( stream->url );
  m_hasSrc = true;
  return NPERR_NO_ERROR;
}


NPError H3DPluginNetscape::SetWindow( NPWindow* window ) {
#ifdef PRINT_ERR_FILE
  cerr << "H3DPluginNetscape::SetWindow" << endl;
#endif
  m_np_window  = window;
  return NPERR_NO_ERROR;
}


void H3DPluginNetscape::InternalInitWindow() {
}

void H3DPluginNetscape::InternalCleanupWindow() {
  if( m_np_window ) {
    m_np_window = NULL;
  }
}

NPBool H3DPluginNetscape::init( NPWindow* aWindow )
{
  if( aWindow == NULL )
    return FALSE;

  m_np_window = aWindow;
  if( m_np_window == NULL )
    return FALSE;

  m_width = aWindow->width;
  m_height = aWindow->height;

#ifdef PRINT_ERR_FILE
  cerr << "H3DPluginNetscape init window: " << m_width << " " << m_height << endl;
#endif

  InternalInitWindow();

  if( !m_InstanceThread.initializedWindow() ) {
#ifdef H3D_WINDOWS
    m_InstanceThread.initWindow( (HWND) m_np_window->window, m_width, m_height );
#endif
#ifdef H3D_OSX
    NP_Port* npPort = (NP_Port*) aWindow->window;
    CGrafPtr browserPort = npPort->port;
    m_InstanceThread.initWindow( GetWindowFromPort (browserPort), 
         m_width, m_height );
#endif

#ifdef H3D_LINUX

    Display *f = ((NPSetWindowCallbackStruct*) aWindow->ws_info)->display;
    NPSetWindowCallbackStruct *w = 
      (NPSetWindowCallbackStruct*) aWindow->ws_info;
    m_InstanceThread.initWindow( w->display,
         (Window) aWindow->window, 
         w->visual,
         m_width, m_height );
#endif
  }

  m_Initialized = TRUE;
  return TRUE;
}

void H3DPluginNetscape::shut()
{
#ifdef PRINT_ERR_FILE
  cerr << "H3DPluginNetscape::shut() " << this << endl;
#endif
  OnDestroy();
  InternalCleanupWindow();
  m_Initialized = FALSE;
}

NPBool H3DPluginNetscape::isInitialized()
{
  return m_Initialized;
}

// ---------------------------------------------------------------------------

void H3DPluginNetscape::OnDestroy(){
 
  if( !m_np_window )
    return;
#ifdef PRINT_ERR_FILE    
 cerr << "H3DPluginInstance::OnDestroy" << endl;
#endif
  m_InstanceThread.Stop();
}

void H3DPluginNetscape::onMenuCommand( int cmd ) {
  if( is_control_class ) {
    if( cmd > (ID_NAVIGATION_MENU - 1) && cmd < ID_VIEWPOINT_MENU ) {
      // Navigation menu command
      /* Locate item that corresponds to menu selection */
      vector<string> *items =
        m_InstanceThread.getNavigationMenuList()->getItems();
      int i = 0;
      vector<string>::iterator it;
      for( it = items->begin(); it != items->end(); ++it ) {
        if( ID_NAVIGATION_MENU + i == cmd ) {
          break;
        }
        ++i;
      }
      m_InstanceThread.onMenuCommand( H3DPluginInstance::ID_NAVIGATION_MENU,
                                      (*it) );
    } else {
      // Viewpoint menu
      /* Locate item that corresponds to menu selection */
      vector<string> *items =
        m_InstanceThread.getViewpointMenuList()->getItems();
      int i = 0;
      vector<string>::iterator it;
      for( it = items->begin(); it != items->end(); ++it ) {
        if( ID_VIEWPOINT_MENU + i == cmd ) {
          break;
        }
        ++i;
      }
      m_InstanceThread.onMenuCommand( H3DPluginInstance::ID_VIEWPOINT_MENU,
                                      (*it) );
    }
  }
}

void H3DPluginNetscape::render()
{
  if( is_control_class ) {

    if( m_np_window == NULL )
      return;

    if( m_InstanceThread.IsStarted() )
    {
      m_InstanceThread.Update();
      return;
    }

    if( !m_InstanceThread.IsStarted() && m_hasSrc)
    {
      invalidateScreenImage();
      if( !m_InstanceThread.Start( m_fileToLoad ) ) {
        m_hasSrc = false; // Reset the 'invalid' source to nonexistent.
        // MessageBox( HWND_DESKTOP, message.c_str(), "Error", MB_ICONERROR );
      }
      return;
    }

    invalidateScreenImage();
  }
}

