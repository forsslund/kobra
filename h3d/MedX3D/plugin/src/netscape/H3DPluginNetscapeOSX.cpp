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
#include "H3DPluginNetscapeOSX.h"
#include "../keys.h"
#include <cassert>
#include <vector>
#include <H3D/X3DTextureNode.h>
#include <H3D/KeySensor.h>
#include <H3D/ProfilesAndComponents.h>
#ifdef HAVE_MEDX3D
#include <H3D/MedX3D/VolumeData.h>
#endif

#ifdef H3D_OSX

#ifdef USE_TIMER
pascal void TimerAction (EventLoopTimerRef, void* userData) {
  ((H3DPluginNetscapeOSX*) userData)->render();
}
#endif


H3DPluginNetscapeOSX::~H3DPluginNetscapeOSX() {
  OnDestroy();
}



NPError H3DPluginNetscapeOSX::SetWindow( NPWindow* window ) {
  GLint bufferRect[4];
  Rect windowRect;
  NP_Port* npPort = (NP_Port*) window->window;
  CGrafPtr browserPort = npPort->port;
  WindowRef browserWindow = GetWindowFromPort (browserPort);
  GetWindowBounds (browserWindow, kWindowStructureRgn, &windowRect);
  GetWindowBounds (browserWindow, kWindowContentRgn, &portRect);
  
  bufferRect[0] = (portRect.left - windowRect.left) + window->clipRect.left;
  bufferRect[1] = (windowRect.bottom - windowRect.top) - (portRect.top - windowRect.top) - window->clipRect.bottom;
  bufferRect[2] = window->clipRect.right - window->clipRect.left;
  bufferRect[3] = window->clipRect.bottom - window->clipRect.top;

  aglSetInteger (aglGetCurrentContext(), AGL_BUFFER_RECT, bufferRect);
  aglEnable (aglGetCurrentContext(), AGL_BUFFER_RECT);

  return H3DPluginNetscape::SetWindow( window );
}


void H3DPluginNetscapeOSX::InternalInitWindow()
{
   #ifdef PRINT_ERR_FILE
    cerr << "InternalInitWindow" << endl;
#endif
    
#ifdef USE_TIMER
    timerUPP = NewEventLoopTimerUPP (TimerAction);
    InstallEventLoopTimer (GetMainEventLoop (), 
         kEventDurationNoWait, 0.01, timerUPP, this, &timer);

#endif
}

void H3DPluginNetscapeOSX::InternalCleanupWindow()
{
#ifdef PRINT_ERR_FILE
  cerr << "InternalCleanupWindow" << endl;
#endif

#ifdef USE_TIMER
  OSStatus error;
  error = RemoveEventLoopTimer(timer); 
  DisposeEventLoopTimerUPP(timerUPP);
#endif


  /*



  if( m_window )
  {
#ifdef USE_TIMER
    KillTimer( m_window, kTimerEvent );
#endif
    SubclassWindow( m_window, m_OldWndProc );
    m_OldWndProc = NULL;
    m_window = NULL;
    }*/
}


#if 0
LRESULT CALLBACK H3DPluginNetscapeOSX::PluginWindowProc( HWND hWnd,
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

#endif

// ---------------------------------------------------------------------------

void H3DPluginNetscapeOSX::OnDestroy() {
  /*  if( !m_window )
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
  #endif*/
  m_InstanceThread.Stop();
}


// Create and display a popupmenu
void H3DPluginNetscapeOSX::OnContextMenu( int x, int y ) {
  if( is_control_class ) {
    MenuRef popup; 
    MenuRef navMenu;
    MenuRef vpMenu;
    
    CreateNewMenu( 0, 0, &popup );
    
    CreateNewMenu( 1, 0, &navMenu );
    CreateNewMenu( 2, 0, &vpMenu );
    
    AppendMenuItemTextWithCFString( popup, CFStringCreateWithCString(NULL, "Navigation", kCFStringEncodingISOLatin1 ), 0, 0, NULL );
    AppendMenuItemTextWithCFString( popup, CFStringCreateWithCString(NULL, "Viewpoint", kCFStringEncodingISOLatin1 ), 0, 0, NULL);
    
    // ---------- Build navigation menu ----------
    H3DPluginInstance::MenuList *navList =
      m_InstanceThread.getNavigationMenuList();
    vector<string>* items = navList->getItems();
    
    // Build menu from list
    int i = 1;
    int checkedPos = 1;
    string checked = navList->getChecked();
    cerr << "Checked: " << checked << endl;
    for( vector<string>::iterator it = items->begin();
         it != items->end(); ++it ) {
      AppendMenuItemTextWithCFString( navMenu, 
              CFStringCreateWithCString(NULL, (*it).c_str(), kCFStringEncodingISOLatin1 ),
              0, 0, NULL );
      cerr << *it << endl;
      if( (*it) == checked ) {
  cerr << "FF: " << i << endl;
        checkedPos = i;
      }
      ++i;
    }

    // highlight(check) the currently selected navigation mode so it is easy to see.
    CheckMenuItem( navMenu, checkedPos, true );

    // we disable the choice when specified(usually if we only have NONE navigation mode
    // and no other choices)
    if( !navList->isCheckedEnabled() ) {
      DisableMenuItem( navMenu, checkedPos );
    }

    // ---------- x ----------

    // ---------- Build viewpoint menu ----------
    H3DPluginInstance::MenuList *vpList =
      m_InstanceThread.getViewpointMenuList();
    items = vpList->getItems();

    // build menu from list
    i = 1;
    checkedPos = 1;
    checked = vpList->getChecked();
    for( vector<string>::iterator it = items->begin();
         it != items->end(); ++it ) {
       AppendMenuItemTextWithCFString( vpMenu, 
               CFStringCreateWithCString(NULL, (*it).c_str(), kCFStringEncodingISOLatin1 ), 
               0, 0, NULL );
      if( (*it) == checked ) {
        checkedPos = i;
      }
      ++i;
    }

    // highlight(check) the currently selected viewpoint so it is easy to see.
    CheckMenuItem( vpMenu, checkedPos, true );

    // we disable the choice when specified(usually if we do not have anu other 
    // viewpoint than default)
    if( !vpList->isCheckedEnabled() ) {
      DisableMenuItem( vpMenu, checkedPos );
    }

    // ---------- x ----------

    // set submenus
    SetMenuItemHierarchicalMenu( popup, 1, navMenu ) ;
    SetMenuItemHierarchicalMenu( popup, 2, vpMenu ) ;
    
    // show popup menu
    uint32 result = PopUpMenuSelect ( popup , y, x, 0 );

    MenuID menu_id =  HiWord(result);
    MenuItemIndex menu_index = LoWord(result);
    
    if( menu_index != 0 ) { 
      if( menu_id == 2 ) {
  m_InstanceThread.onMenuCommand( menu_id,
          (*vpList->getItems())[ menu_index -1 ] );
      } else if( menu_id == 1 ) {
  m_InstanceThread.onMenuCommand( menu_id,
          (*navList->getItems())[ menu_index -1 ] );
      }
    }
    cerr << "Menu choicde:" << HiWord(result) << " " << LoWord(result) << endl;
    cerr << "Menu choice:" << menu_id << " " << menu_index << endl;
  }
}

uint16 H3DPluginNetscapeOSX::HandleEvent (void* event) {
  EventRecord *inEvent = (EventRecord*) event;
  
  switch (inEvent->what) {
    
  case nullEvent:
    // idle event
    if( inEvent->where.h > -1 && inEvent->where.v > -1 ) {
      getPluginInstance()->onMouseMotion(inEvent->where.h - m_np_window->x - portRect.left, 
           inEvent->where.v - m_np_window->y - portRect.top);
    }
    break;
    /*
    case getFocusEvent:
  // plugin gets focus
    break;
    
  case loseFocusEvent:
    engine->getInputQueue ()->lostFocus ();
    break;
    
  case adjustCursorEvent:
    break;
    
    */

  case mouseDown:
    // testing indicates that bit 6 in modifiers is set if middle mouse
    // button and bit 12(control key) if right. have not found documentation
    // about this though.
    if( inEvent->modifiers & (1<<7) ) {
      
      getPluginInstance()->onMiddleMouseButtonDown();
    } else if( inEvent->modifiers & (1<<12) ) {
      OnContextMenu( inEvent->where.h, inEvent->where.v);
      getPluginInstance()->onRightMouseButtonDown();
    } else {
     getPluginInstance()->onLeftMouseButtonDown();
    }
    return TRUE;
    break;
    
  case mouseUp:
    // testing indicates that bit 6 in modifiers is set if middle mouse
    // button and bit 12(control key) if right. have not found documentation
    // about this though.
    if( inEvent->modifiers & (1<<7) ) {
      getPluginInstance()->onMiddleMouseButtonUp();
    } else if( inEvent->modifiers & (1<<12) ) {
      getPluginInstance()->onRightMouseButtonUp();
    } else {
      getPluginInstance()->onLeftMouseButtonUp();
    }
    return TRUE;
    break;
    
  case keyDown: {
    bool is_special;
    int key = translateMacKey (inEvent->modifiers, 
             inEvent->message, 
             inEvent->message, is_special );
    getPluginInstance()->onKeyDown( key, is_special );
    return TRUE;
    break;
    }
  case keyUp: {
    bool is_special;
    int key = translateMacKey (inEvent->modifiers, 
             inEvent->message, 
             inEvent->message,
             is_special );
    getPluginInstance()->onKeyUp( key, is_special );
    return TRUE;
    break;
    }
  case autoKey:
    bool is_special;
    int key = translateMacKey (inEvent->modifiers, 
             inEvent->message, 
             inEvent->message,
             is_special);
    getPluginInstance()->onKeyDown( key, is_special );
    return TRUE;
    break;
    
  case updateEvt:
    break;
    
  case activateEvt:
    break;
    
  case osEvt:
    break;
    
  default:
    ;
  }
  
  return FALSE;
}

int H3DPluginNetscapeOSX::translateMacKey (UInt32 modifier, UInt32 keyCode, SInt8 charCode, bool &is_special ) {
  //cerr << modifier << " " << keyCode << " " << charCode << endl;
  //cerr << modifier << " " <<(keyCode & 0x0000ff00) << " " << (keyCode & 0x000000ff )<< endl;
  /*switch (modifier) {
  case shiftKey: cerr << "Shift" << endl;
  case rightShiftKey: return 16 + 256;
  case controlKey: return 17 + 256;
  case rightControlKey: return 17 + 256;
  case optionKey: return 18 + 256;
  case rightOptionKey: return 18 + 256;
  case alphaLock: return 20 + 256;
  }*/
  
  is_special = true;

  switch (keyCode) {
  case 0x7A: return KeySensor::F1;
  case 0x78: return KeySensor::F2;
  case 0x63: return KeySensor::F3;
  case 0x76: return KeySensor::F4;
  case 0x60: return KeySensor::F5;
  case 0x61: return KeySensor::F6;
  case 0x62: return KeySensor::F7;
  case 0x64: return KeySensor::F8;
  case 0x65: return KeySensor::F9;
  case 0x6D: return KeySensor::F10;
  case 0x67: return KeySensor::F11;
  case 0x6F: return KeySensor::F12;
    //case 0x69:      // F13 / Print Screen
    //case 0x6B:   // F14 / Scroll Lock
    //case 0x71: return 19 + 256;  // F15 / Pause
  }
  

  switch (charCode) {
    //case kBackspaceCharCode: //
    //case kTabCharCode: 
    //case kEnterCharCode: return 13;
    //case kEscapeCharCode: return 27;
    //case kSpaceCharCode: return 32;
  case kPageUpCharCode: return KeySensor::PGUP;
  case kPageDownCharCode: return KeySensor::PGDN;
  case kEndCharCode: return KeySensor::END;
  case kHomeCharCode: return KeySensor::HOME;
  case kLeftArrowCharCode: return KeySensor::LEFT;
  case kUpArrowCharCode: return KeySensor::UP;
  case kRightArrowCharCode: return KeySensor::RIGHT;
  case kDownArrowCharCode: return KeySensor::DOWN;
    //case kHelpCharCode: return 45 + 256;    // Insert
    //case kDeleteCharCode: return 46 + 256;
  default: {
    is_special = false;
    return charCode;
  }
  }
  
  return 0;
}





#endif // H3D_OSX
