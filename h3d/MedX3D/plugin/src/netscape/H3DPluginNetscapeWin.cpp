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
#include "H3DPluginNetscapeWin.h"
#include "../keys.h"
#include <cassert>
#include <vector>
#include "H3D/X3DTextureNode.h"
#include <H3D/ProfilesAndComponents.h>
#ifdef HAVE_MEDX3D
#include <H3D/MedX3D/VolumeData.h>
#endif

#ifdef H3D_WINDOWS
#include <windowsx.h>

/**
 * Local globals
 */

#ifdef USE_TIMER
const int kTimerEvent = 1;
#endif

/** 
 * Uses hooking to get arrow keys since they are considered special by windows
 * This hook needs to be global (for compatibility with windows version < 2000)
 */
HHOOK hook = NULL;

/**
 * Hook function for keyboard input used by the plugin
 */
LRESULT CALLBACK KeyboardProc( int code, WPARAM wparam, LPARAM lparam ) {
  // Check if we should handle message and assert controlclass is set
  if( code >= 0 && controlclass != NULL) {
    // Handle keycode, should check for HC_ACTION / HC_NOREMOVE?
    if( code == HC_ACTION ) {
      // Repeatcount stored in bits 0-15
      UINT repeatcount = lparam & 0xFFFF;
      // The bits in lparam except the repeatcount
      UINT flags = lparam & 0xFFFF0000;
      // We need the transition state so we know which function to call
      // (onkeyup or onkeydown)
      bool released = ((lparam & 0x80000000) != 0);
      char buf[200];
      sprintf( buf, "Repeatcount: %d\nCode: %x\n, Flags: %x\n, Released: %d\n",
               repeatcount, code, flags, released );
      //MessageBox(HWND_DESKTOP, buf, "Debug", MB_OK);
      switch( wparam ) {
        case VK_UP:
        case VK_DOWN:
        case VK_LEFT:
        case VK_RIGHT:
          int key = getKeyFromKeycode( wparam );
          if( released ) {
            if( key > 0 ) {
              controlclass->getPluginInstance()->onKeyUp( key, true );
            } else {
              controlclass->getPluginInstance()->onKeyUp( wparam, false );
            }
          } else {
            if( key > 0 ) {
              controlclass->getPluginInstance()->onKeyDown( key, true );
            } else {
              controlclass->getPluginInstance()->onKeyDown( wparam, false );
            }
          }
      }
    }
  }

  return CallNextHookEx(hook, code, wparam, lparam);
}

BOOL WINAPI DllMain( HINSTANCE hDLL, DWORD dwReason, LPVOID lpReserved )
{
#ifdef PRINT_ERR_FILE
  cerr << "DllMain() " << hDLL << endl;
#endif
  // No need to do anything
  return TRUE;
}


H3DPluginNetscapeWin::~H3DPluginNetscapeWin() {
  OnDestroy();
}



NPError H3DPluginNetscapeWin::SetWindow( NPWindow* window ) {
  // set keyboard focus to window.
  if( window->window != 0 )
    SetFocus( (HWND) window->window );

  return H3DPluginNetscape::SetWindow( window );
}


void H3DPluginNetscapeWin::InternalInitWindow()
{
   #ifdef PRINT_ERR_FILE
    cerr << "InternalInitWindow" << endl;
#endif

    m_window =(HWND) (m_np_window->window);


  // Init hook
  if( !hook ) {
#ifdef PRINT_ERR_FILE
    cerr << "hook set" << endl;
#endif
    hook = SetWindowsHookEx( WH_KEYBOARD, (HOOKPROC) &KeyboardProc, NULL,
                             GetCurrentThreadId() );
  }

  if( hook == NULL ) {
    // Ooops: Internal error, should never ever occur
    MessageBox(HWND_DESKTOP, "Hook failed to install", "Debug", MB_OK );
  }

  // Subclass parent window so we can handle the messages we want
  m_OldWndProc = SubclassWindow( m_window, PluginWindowProc );

  // Set a class pointer so we can access this instance from the WindowProc
  SetWindowLongPtr( m_window, GWL_USERDATA, (LONG_PTR)this );

#ifdef USE_TIMER
  SetTimer( m_window, kTimerEvent, 1000/PlayerFPS, NULL );
#endif
}

void H3DPluginNetscapeWin::InternalCleanupWindow()
{
#ifdef PRINT_ERR_FILE
  cerr << "InternalCleanupWindow" << endl;
#endif
  if( m_window )
  {
#ifdef USE_TIMER
    KillTimer( m_window, kTimerEvent );
#endif
    SubclassWindow( m_window, m_OldWndProc );
    m_OldWndProc = NULL;
    m_window = NULL;
  }
}



LRESULT CALLBACK H3DPluginNetscapeWin::PluginWindowProc( HWND hWnd,
                                                      UINT msg,
                                                      WPARAM wParam,
                                                      LPARAM lParam )
{
  int key = 0;
  // Return default wndproc if the plugininstance is not set yet
  H3DPluginNetscapeWin *plugin =
    (H3DPluginNetscapeWin*)GetWindowLongPtr( hWnd, GWL_USERDATA );
  if( !plugin )
    return DefWindowProc( hWnd, msg, wParam, lParam );
  
  switch( msg ) {
    case WM_SIZING:
      cerr << "Window is sizing" << endl;
      break;
    case WM_SIZE:
      cerr << "Window changes size" << endl;
      plugin->m_width = LOWORD(lParam);
      plugin->m_height = HIWORD(lParam);
      plugin->getPluginInstance()->setPluginWindowSize( plugin->m_width,
                                                        plugin->m_height );
      break;
    // Generic messages & timer events
#ifdef USE_TIMER
    case WM_TIMER:
      plugin->render();
      return 0;
#endif
    case WM_DESTROY:
      plugin->OnDestroy();
      return 0;
    case WM_COMMAND:
      /* HIWORD(wParam) == 0 indicates Menuselection,
         HIWORD(wParam) == 1 indicates accelerator,
         other values may appear but should not for this plugin */
      plugin->onMenuCommand(LOWORD(wParam) /* Menu command id */);
      return 0;
    case WM_CONTEXTMENU:
      /* lParam has x & y coordinates */
      plugin->OnContextMenu( GET_X_LPARAM(lParam ), GET_Y_LPARAM(lParam) );
      return 0;
    // Manage mouse buttons
    // All messages return defwindowproc
    case WM_LBUTTONDOWN:
      plugin->getPluginInstance()->onLeftMouseButtonDown();
      break;
    case WM_LBUTTONUP:
      plugin->getPluginInstance()->onLeftMouseButtonUp();
      break;
    case WM_RBUTTONDOWN:
      plugin->getPluginInstance()->onRightMouseButtonDown();
      break;
    case WM_RBUTTONUP:
      plugin->getPluginInstance()->onRightMouseButtonUp();
      break;
    case WM_MBUTTONDOWN:
      plugin->getPluginInstance()->onMiddleMouseButtonDown();
      break;
    case WM_MBUTTONUP:
      plugin->getPluginInstance()->onMiddleMouseButtonUp();
      break;
    case WM_MOUSEMOVE:
        // Ignore all messages until the rendering instance has been started
      if( !plugin->getPluginInstance()->IsStarted() )
        break;

      plugin->getPluginInstance()->onMouseMotion( GET_X_LPARAM(lParam),
                                                  GET_Y_LPARAM(lParam) );
      break;
  }

  // DefWindowProc( hWnd, msg, wParam, lParam );
  return CallWindowProc( plugin->getParentWndProc(), hWnd, msg, wParam,
                         lParam );
}

// ---------------------------------------------------------------------------

void H3DPluginNetscapeWin::OnDestroy() {
  if( !m_window )
    return;

 if( getIsControlClass() && hook ) {
#ifdef PRINT_ERR_FILE
    cerr << "hook removed" << endl;
#endif
    UnhookWindowsHookEx(hook);
    hook = 0;
 }

#ifdef USE_TIMER
  KillTimer( m_window, kTimerEvent );
#endif
  m_InstanceThread.Stop();
}


// Create and display a popupmenu
void H3DPluginNetscapeWin::OnContextMenu( int x, int y )
{
  if( is_control_class ) {
    HMENU popup = CreatePopupMenu();
    HMENU navMenu = CreatePopupMenu();
    HMENU vpMenu = CreatePopupMenu();


    // ---------- Build navigation menu ----------
    H3DPluginInstance::MenuList *navList =
      m_InstanceThread.getNavigationMenuList();
    vector<string>* items = navList->getItems();

    // Build menu from list
    int i = 0;
    int checkedPos = 0;
    string checked = navList->getChecked();
    for( vector<string>::iterator it = items->begin();
         it != items->end(); ++it ) {
      AppendMenu( navMenu, MF_STRING | MF_ENABLED, ID_NAVIGATION_MENU + i,
                  (*it).c_str() );
      if( (*it) == checked ) {
        checkedPos = i;
      }
      ++i;
    }

    CheckMenuRadioItem( navMenu, ID_NAVIGATION_MENU,
                        ID_NAVIGATION_MENU+items->size() - 1,
                        ID_NAVIGATION_MENU+checkedPos, MF_BYCOMMAND );
    if( !navList->isCheckedEnabled() ) {
      EnableMenuItem( navMenu, ID_NAVIGATION_MENU+checkedPos,
                      MF_BYCOMMAND | MF_GRAYED );
    }

    // ---------- x ----------

    // ---------- Build viewpoint menu ----------
    H3DPluginInstance::MenuList *vpList =
      m_InstanceThread.getViewpointMenuList();
    items = vpList->getItems();

    i = 0;
    checkedPos = 0;
    checked = vpList->getChecked();
    for( vector<string>::iterator it = items->begin();
         it != items->end(); ++it ) {
      AppendMenu( vpMenu, MF_STRING | MF_ENABLED,
                  ID_VIEWPOINT_MENU + i, (*it).c_str() );
      if( (*it) == checked ) {
        checkedPos = i;
      }
      ++i;
    }

    CheckMenuRadioItem( vpMenu, ID_VIEWPOINT_MENU,
                        ID_VIEWPOINT_MENU + items->size() - 1,
                        ID_VIEWPOINT_MENU + checkedPos, MF_BYCOMMAND );
    if( !vpList->isCheckedEnabled() ) {
      EnableMenuItem( vpMenu, ID_VIEWPOINT_MENU+checkedPos,
                      MF_BYCOMMAND | MF_GRAYED );
    }

    // ---------- x ----------

    // Add menus to container and display them
    AppendMenu(popup, MF_POPUP, (UINT) navMenu, "Navigation");
    AppendMenu(popup, MF_POPUP, (UINT) vpMenu, "Viewpoint");

    // Parent window gets the menu commands (PluginParentWindowProc)
    TrackPopupMenuEx( popup, 0, x, y, m_window, NULL ); 
  }
}

#endif // H3D_WINDOWS
