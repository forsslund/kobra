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
#include "H3DPluginNetscapeLinux.h"
#include "../keys.h"
#include <cassert>
#include <vector>
#include <H3D/X3DTextureNode.h>
#include <H3D/KeySensor.h>
#include <H3D/MouseSensor.h>
#include <H3D/ProfilesAndComponents.h>
#ifdef HAVE_MEDX3D
#include <H3D/MedX3D/VolumeData.h>
#endif

#ifdef H3D_LINUX

#ifdef USE_TIMER
void H3DPluginNetscapeLinux::timerCallbackFunc (XtPointer arg, 
            XtIntervalId *xtinterval) {
  // cerr << "Timer" << endl;
  H3DPluginNetscapeLinux *plugin = (H3DPluginNetscapeLinux*)arg;
  // cerr << "Calling render()" << endl;
  plugin->render();
  //cerr << "Done" << endl;

  // cerr << "NP: " << plugin->m_np_window << endl;
  //cerr << "ws; " << plugin->m_np_window->ws_info;
  //NPSetWindowCallbackStruct *w = 
  //  (NPSetWindowCallbackStruct*) plugin->m_np_window->ws_info;
  //cerr << "w:  " << w << endl;
  //cerr << "Display:" << plugin->display << endl;
  //cerr << w->display << " " << XtDisplayToApplicationContext (plugin->display) << endl;

  plugin->xtinterval = 
    XtAppAddTimeOut( XtDisplayToApplicationContext (plugin->display),
         1, 
         (XtTimerCallbackProc) H3DPluginNetscapeLinux::timerCallbackFunc, 
         plugin);

}
#endif

void H3DPluginNetscapeLinux::handleMessages (Widget xtwidget, 
               H3DPluginNetscapeLinux *plugin, 
               XEvent *xevent, 
               Boolean *b) {
  cerr << "message xevent: " << xevent->type << endl;
  KeySym keysym;
  switch (xevent->type) {
  case ButtonPress:
    switch (xevent->xbutton.button) {
      
    case 1:
      plugin->getPluginInstance()->onLeftMouseButtonDown();
      break;
    case 2:
      plugin->getPluginInstance()->onMiddleMouseButtonDown();
      break;
    case 3:
      plugin->getPluginInstance()->onRightMouseButtonDown();
      break;
    case 4:
      plugin->getPluginInstance()->onMouseWheel( MouseSensor::TOWARDS );
      break;
    case 5:
      plugin->getPluginInstance()->onMouseWheel( MouseSensor::FROM );
      break;
    }

    break;
    
  case ButtonRelease:
    switch (xevent->xbutton.button) {
    case 1:
       plugin->getPluginInstance()->onLeftMouseButtonUp();
      break;
    case 2:
      plugin->getPluginInstance()->onMiddleMouseButtonUp();
      break;
    case 3:
      plugin->getPluginInstance()->onRightMouseButtonUp();
      break;
    }
    break;
    
  case FocusIn:
    break;
    
  case FocusOut:
    break;
    
  case KeyPress: {
    XLookupString (&(xevent->xkey), NULL, 0, &keysym, NULL);
    bool is_special;
    int key = plugin->translateXKey ( keysym, is_special );
    plugin->getPluginInstance()->onKeyDown( key, is_special );
    break;
  }
  case KeyRelease: {
    XLookupString (&(xevent->xkey), NULL, 0, &keysym, NULL);
    bool is_special;
    int key = plugin->translateXKey ( keysym, is_special );
    plugin->getPluginInstance()->onKeyUp( key, is_special );
    break;
  }
  case MotionNotify:
    plugin->getPluginInstance()->onMouseMotion( xevent->xmotion.x, 
            xevent->xmotion.y );
    break;
    
  case ConfigureNotify:
    plugin->getPluginInstance()->setPluginWindowSize( xevent->xconfigure.width,
                  xevent->xconfigure.height );
    break;
    
  default:
    break;
  }
}



H3DPluginNetscapeLinux::~H3DPluginNetscapeLinux() {
  OnDestroy();
}



NPError H3DPluginNetscapeLinux::SetWindow( NPWindow* window ) {
  cerr << "SetWindow" << endl;
  NPSetWindowCallbackStruct *w = 
    (NPSetWindowCallbackStruct*) window->ws_info;
  cerr << w->display << " " << XtDisplayToApplicationContext (w->display) << endl;
  //xtcontext =  XtDisplayToApplicationContext (w->display);
  XtAppContext t = XtDisplayToApplicationContext( w->display );
  cerr << XtDisplayToApplicationContext (w->display) << " " << xtcontext << endl;
  
  cerr << "NP: " << window << endl;
  cerr << "ws; " << window->ws_info;
  cerr << "w:  " << w << endl;
  cerr << "Display:" << w->display << endl;
  cerr << w->display << " " << XtDisplayToApplicationContext (w->display) << endl;

  return H3DPluginNetscape::SetWindow( window );
}


void H3DPluginNetscapeLinux::InternalInitWindow()
{
   #ifdef PRINT_ERR_FILE
    cerr << "InternalInitWindow" << endl;
#endif
    
#ifdef USE_TIMER
    
    NPSetWindowCallbackStruct *w = 
      (NPSetWindowCallbackStruct*) m_np_window->ws_info;
    cerr << w->display << " " << XtDisplayToApplicationContext (w->display) << endl;
    //xtcontext =  XtDisplayToApplicationContext (w->display);
    XtAppContext t = XtDisplayToApplicationContext( w->display );
    cerr << XtDisplayToApplicationContext (w->display) << " " << xtcontext << endl;

    cerr << "NP: " << m_np_window << endl;
    cerr << "ws; " << m_np_window->ws_info;
    cerr << "w:  " << w << endl;
    cerr << "Display:" << w->display << endl;
    display = w->display;
    cerr << w->display << " " << XtDisplayToApplicationContext (w->display) << endl;


    // Add event handler for mouse and keyboard
    Window xwindow = (Window)m_np_window->window;

    Widget widget = XtWindowToWidget (display, xwindow);
    long eventmask = 
      ButtonPressMask |
      ButtonReleaseMask |
      FocusChangeMask |
      KeyPressMask |
      KeyReleaseMask |
      PointerMotionMask;
    XSelectInput (display, xwindow, eventmask);
    XtAddRawEventHandler (widget, eventmask, FALSE, (XtEventHandler) H3DPluginNetscapeLinux::handleMessages, this);
    

    // Add timer callback.
    //xtcontext = t;
    xtinterval = 
      XtAppAddTimeOut ( XtDisplayToApplicationContext (w->display), //xtcontext,
      10, 
      (XtTimerCallbackProc) H3DPluginNetscapeLinux::timerCallbackFunc, 
      this);
 
#endif
}

void H3DPluginNetscapeLinux::InternalCleanupWindow()
{
#ifdef PRINT_ERR_FILE
  cerr << "InternalCleanupWindow" << endl;
#endif

#ifdef USE_TIMER
  XtRemoveTimeOut( xtinterval );
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
LRESULT CALLBACK H3DPluginNetscapeLinux::PluginWindowProc( HWND hWnd,
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

void H3DPluginNetscapeLinux::OnDestroy() {
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

int H3DPluginNetscapeLinux::translateXKey( KeySym &keysym, 
             bool &is_special ) {
  is_special = false;
  switch (keysym) {
  case XK_BackSpace: return 8;
  case XK_Tab: return 9;
  case XK_Return: return 13;
  case XK_Escape: return 27;
  case XK_space: return 32;
  case XK_exclam: return 33;
  case XK_quotedbl: return 34;
  case XK_numbersign: return 35;
  case XK_dollar: return 36;
  case XK_percent: return 37;
  case XK_ampersand: return 38;
  case XK_apostrophe: return 39;
  case XK_parenleft: return 40;
  case XK_parenright: return 41;
  case XK_asterisk: return 42;
  case XK_plus: return 43;
  case XK_comma: return 44;
  case XK_minus: return 45;
  case XK_period: return 46;
  case XK_slash: return 47;
  case XK_colon: return 58;
  case XK_semicolon: return 59;
  case XK_less: return 60;
  case XK_equal: return 61;
  case XK_greater: return 62;
  case XK_question: return 63;
  case XK_at: return 64;
  case XK_bracketleft: return 91;
  case XK_backslash: return 92;
  case XK_bracketright: return 93;
  case XK_asciicircum: return 94;
  case XK_underscore: return 95;
  case XK_grave: return 96;
  case XK_braceleft: return 123;
  case XK_bar: return 124;
  case XK_braceright: return 125;
  case XK_asciitilde: return 126;
    
  case XK_0: return 48;
  case XK_1: return 49;
  case XK_2: return 50;
  case XK_3: return 51;
  case XK_4: return 52;
  case XK_5: return 53;
  case XK_6: return 54;
  case XK_7: return 55;
  case XK_8: return 56;
  case XK_9: return 57;
    
  case XK_a: return 97;
  case XK_b: return 98;
  case XK_c: return 99;
  case XK_d: return 100;
  case XK_e: return 101;
  case XK_f: return 102;
  case XK_g: return 103;
  case XK_h: return 104;
  case XK_i: return 105;
  case XK_j: return 106;
  case XK_k: return 107;
  case XK_l: return 108;
  case XK_m: return 109;
  case XK_n: return 110;
  case XK_o: return 111;
  case XK_p: return 112;
  case XK_q: return 113;
  case XK_r: return 114;
  case XK_s: return 115;
  case XK_t: return 116;
  case XK_u: return 117;
  case XK_v: return 118;
  case XK_w: return 119;
  case XK_x: return 120;
  case XK_y: return 121;
  case XK_z: return 122;
    
  case XK_A: return 65;
  case XK_B: return 66;
  case XK_C: return 67;
  case XK_D: return 68;
  case XK_E: return 69;
  case XK_F: return 70;
  case XK_G: return 71;
  case XK_H: return 72;
  case XK_I: return 73;
  case XK_J: return 74;
  case XK_K: return 75;
  case XK_L: return 76;
  case XK_M: return 77;
  case XK_N: return 78;
  case XK_O: return 79;
  case XK_P: return 80;
  case XK_Q: return 81;
  case XK_R: return 82;
  case XK_S: return 83;
  case XK_T: return 84;
  case XK_U: return 85;
  case XK_V: return 86;
  case XK_W: return 87;
  case XK_X: return 88;
  case XK_Y: return 89;
  case XK_Z: return 90;
  }

  is_special = true;
  switch( keysym ) {
    //case XK_Pause: return 19 + 256;
  case XK_Shift_L: return KeySensor::SHIFT;
  case XK_Shift_R: return KeySensor::SHIFT;;
  case XK_Control_L: return KeySensor::CONTROL;
  case XK_Control_R: return KeySensor::CONTROL;
  case XK_Alt_L: return KeySensor::ALT;
  case XK_Alt_R: return KeySensor::ALT;
  case XK_Page_Up: return KeySensor::PGUP;
  case XK_Page_Down: return KeySensor::PGDN;
  case XK_End: return KeySensor::END;
  case XK_Home: return KeySensor::HOME;
  case XK_Left: return KeySensor::LEFT;
  case XK_Up: return KeySensor::UP;
  case XK_Right: return KeySensor::RIGHT;
  case XK_Down: return KeySensor::DOWN;
    //  case XK_Insert: return 45 + 256;
    //case XK_Delete: return 46 + 256;
    //case XK_Caps_Lock: return 20 + 256;
    //case XK_Num_Lock: return 144 + 256;
    //case XK_Scroll_Lock: return 145 + 256;
    
  case XK_F1: return KeySensor::F1;
  case XK_F2: return KeySensor::F2;
  case XK_F3: return KeySensor::F3;
  case XK_F4: return KeySensor::F4;
  case XK_F5: return KeySensor::F5;
  case XK_F6: return KeySensor::F6;
  case XK_F7: return KeySensor::F7;
  case XK_F8: return KeySensor::F8;
  case XK_F9: return KeySensor::F9;
  case XK_F10: return KeySensor::F10;
  case XK_F11: return KeySensor::F11;
  case XK_F12: return KeySensor::F12;
  default: return 0;
  }
}




#if 0
// Create and display a popupmenu
/*void H3DPluginNetscapeLinux::OnContextMenu( int x, int y ) {
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

uint16 H3DPluginNetscapeLinux::HandleEvent (void* event) {
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
  */

int H3DPluginNetscapeLinux::translateMacKey (UInt32 modifier, UInt32 keyCode, SInt8 charCode, bool &is_special ) {


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
  #endif




#endif // H3D_LINUX
