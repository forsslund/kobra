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
/// \file H3DPluginInstance.cpp
/// \brief CPP file for H3DPluginInstance, contains functionality of the
/// plugin.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "H3DPluginInstance.h"
#include <assert.h>

#include <stdio.h>
#include <exception>
#include "H3D/X3DSAX2Handlers.h"
#include "H3D/X3D.h"
#include "H3D/Viewpoint.h"
#include "H3D/ViewpointGroup.h"
#include "H3D/MouseSensor.h"

#ifdef PRINT_ERR_FILE
ofstream* H3DPluginInstance::PrintingCerrInfo::cerrfile = 0;
streambuf* H3DPluginInstance::PrintingCerrInfo::cerrbuf = 0;
bool H3DPluginInstance::PrintingCerrInfo::cerrfile_initialized = false;
#endif

H3DPluginInstance::H3DPluginInstance()
:  m_IsStarted(false),
  window_initialized(false),
  m_navTypeCount(0),
  m_viewpointCount(0),
  m_RenderWindow(0),
  m_navigation(0)
{
#ifdef PRINT_ERR_FILE
  errfile.open("C:/Markus/MedX3D/console_messages.log");
  H3DUtil::Console.setOutputStream( errfile );
#endif
  m_Scene.reset( 0 );
  m_viewpoint.reset( NULL );

  m_navigationItems = 0;
  m_viewpointItems = 0;
#ifdef PRINT_ERR_FILE
  cerr << "H3DPluginInstance " << this << endl;
  errfile.flush();
#endif
}


H3DPluginInstance::~H3DPluginInstance()
{
  Stop();
#ifdef PRINT_ERR_FILE
  errfile.flush();
  errfile.close();
#endif
}


#ifdef H3D_WINDOWS
bool H3DPluginInstance::initWindow( HWND window, int _width, int _height ) {
  PluginWindow *w = new PluginWindow();
  m_RenderWindow = w;
  m_RenderWindow->width->setValue( _width );
  m_RenderWindow->height->setValue( _height );
  

  if( !w->initialize( window ) ) {
    delete m_RenderWindow;
    return false;
  }
#ifdef PRINT_ERR_FILE
  errfile.flush();
#endif
  window_initialized = true;
  return true;
}
#endif


#ifdef H3D_OSX
bool H3DPluginInstance::initWindow( WindowRef window, int _width, int _height ) {
  PluginWindowOSX *w = new PluginWindowOSX();
  m_RenderWindow = w;
  m_RenderWindow->width->setValue( _width );
  m_RenderWindow->height->setValue( _height );
  
  if( !w->initialize( window ) ) {
    delete m_RenderWindow;
    return false;
  }
#ifdef PRINT_ERR_FILE
  errfile.flush();
#endif
  window_initialized = true;
  return true;
}
#endif


#ifdef H3D_LINUX
bool H3DPluginInstance::initWindow(  Display *display, 
             Window window, 
             Visual *visual, 
             int _width, 
             int _height ) {
  PluginWindowLinux *w = new PluginWindowLinux();
  m_RenderWindow = w;
  m_RenderWindow->width->setValue( _width );
  m_RenderWindow->height->setValue( _height );
  
  if( !w->initialize( display, window, visual ) ) {
    delete m_RenderWindow;
    return false;
  }
#ifdef PRINT_ERR_FILE
  errfile.flush();
#endif
  window_initialized = true;
  return true;
}
#endif

bool H3DPluginInstance::Start( string theFileToLoad )
{
#ifdef PRINT_ERR_FILE
  errfile.flush();
#endif
  if( !window_initialized )
    return false;
  // Should assert m_RenderWindow != null
  m_RenderWindow->makeWindowActive();

  if( !m_Scene.get() )
    m_Scene.reset( new Scene );
  // Initialize scene
  m_Scene->window->push_back( m_RenderWindow );

  // Add children to scene
  X3D::DEFNodes dn;
  AutoRef< Group > g( new Group );
  Group *children = NULL;

  try {
    // Don't add children if we detect error in src file
    children = X3D::createX3DFromURL( theFileToLoad, &dn );
  } catch (X3D::XMLParseError &e) {
    // Ignore specific error for now 
    string msg = "Failed to read file: " + theFileToLoad +
                 " correctly.\nReason: " + e.message;
    Console(4) << msg << endl;
    cerr << msg << endl;
    //MessageBox(HWND_DESKTOP, msg.c_str(), "Error", MB_ICONERROR | MB_OK);
    return false;
  }

    g->children->push_back( children );
    m_Scene->sceneRoot->setValue( g.get() );

  // Guarantee we have a viewpoint if none existed in the x3d-file
  if( !Viewpoint::getActive() ) {
    Viewpoint *vp = new Viewpoint();
    m_viewpoint = AutoRef< Node >(vp);
    X3DGroupingNode *c = static_cast< X3DGroupingNode * >
      ( m_Scene->sceneRoot->getValue() );
    c->children->push_back(vp);
  }

  // All set up
  m_IsStarted = true;

  buildNavigationMenuList();
  buildViewpointMenuList();

  return true;
}

void H3DPluginInstance::Stop(void)
{
  resetRoot();
  
  // shutdown GL
  
#ifdef PRINT_ERR_FILE
  cerr << "H3DPluginInstance::Stop " << this << endl;
  errfile.flush();
#endif

  m_IsStarted = false;
}

void H3DPluginInstance::resetRoot() {
  if( m_Scene.get() )
    m_Scene->sceneRoot->setValue( 0 );
  m_Scene.reset( 0 );
  m_RenderWindow = 0;
  m_viewpoint.reset( 0 );
  m_viewpointList.clear();
  if( m_navigationItems ) {
    delete m_navigationItems;
    m_navigationItems = 0;
  }
  if( m_viewpointItems ) {
    delete m_viewpointItems;
    m_viewpointItems = 0;
  }
#ifdef PRINT_ERR_FILE
  cerr << "H3DPluginInstance::resetRoot " << this << endl;
  errfile.flush();
#endif
}

void H3DPluginInstance::Update()
{
#ifdef PRINT_ERR_FILE
  errfile.flush();
#endif
  
  m_RenderWindow->makeWindowActive();
  glClearColor( 0.0f, 0.0f, 1.0f, 1.0f );
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  for( set< Scene * >::iterator i = Scene::scenes.begin();
       i != Scene::scenes.end();
       ++i ) {
    if( (*i)->isActive() ) {
      NodeVector window_vector = (*i)->window->getValue();
      for( NodeVector::const_iterator j = window_vector.begin();
     j != window_vector.end();
     ++j ) {
  if( (*j) == m_RenderWindow ) {
    (*i)->idle();
  }
      }
    }
  }

  //SwapContext( openGL_context_info );
}

//Checks whether a navigation type is valid
bool H3DPluginInstance::validateNavType( string type ) {
  return( type == "ANY"     ||
      type == "EXAMINE" ||
      type == "FLY"     ||
      type == "WALK"    ||
      type == "LOOKAT"  ||
      type == "NONE" );
}

void H3DPluginInstance::buildNavigationMenuList() {
  if( !m_navigationItems )
    m_navigationItems = new MenuList();
  //Get active navigation info object
  if( NavigationInfo::getActive() ) {
    m_navigation = NavigationInfo::getActive();
    //Store allowed navigation types and count
    vector<string> navTypes = m_navigation->type->getValue();

    if( m_navigation->getUsedNavType() == "NONE" ) {
      // Special case
      m_navigationItems->addItem("NONE");
      m_navigationItems->setChecked( "NONE", false );
    } else {
      vector<string> allowedTypes;
      vector<string>::iterator navList = navTypes.begin();
      bool hasAny = false;
      for( vector<string>::iterator navList = navTypes.begin();
           navList != navTypes.end(); ++navList ) {
        if( validateNavType(*navList) && (*navList != "NONE") ) {
          if( allowedTypes.empty() ) {
            if( (*navList != "ANY") ) {
              allowedTypes.push_back(*navList);
            } else {
                            allowedTypes.push_back("EXAMINE");
              allowedTypes.push_back("FLY");
              allowedTypes.push_back("WALK");
              allowedTypes.push_back("LOOKAT");
              break;
            }
          } else {
            bool found = false;
            for( vector<string>::iterator allowedList = allowedTypes.begin();
                 allowedList != allowedTypes.end(); ++allowedList ) {
              if( *navList == "ANY" ) {
                hasAny = true;
                found = true;
              } else if( *allowedList == *navList ) {
                found = true;
                break;
              }
            }
            if( !found ) {
              allowedTypes.push_back(*navList);
            }
          }
        }
      }
      if (hasAny) {
        vector<string> allTypes;
        allTypes.push_back("EXAMINE");
        allTypes.push_back("FLY");
        allTypes.push_back("WALK");
        allTypes.push_back("LOOKAT");
        vector<string>::iterator allList = allTypes.begin();
        for( vector<string>::iterator allList = allTypes.begin();
             allList != allTypes.end(); ++allList ) {
          bool found = false;
          for( vector<string>::iterator allowedList = allowedTypes.begin();
               allowedList != allowedTypes.end(); ++allowedList ) {
            if( *allowedList == *allList ) {
              found = true;
              break;
            }
          }
          if( !found ) {
            allowedTypes.push_back(*allList);
          }
        }
      }
      m_navTypeCount = allowedTypes.size();
      int j = 0;
      for( vector<string>::iterator menuList = allowedTypes.begin();
           menuList != allowedTypes.end(); ++menuList ) {
        string typeName = (*menuList);
        m_navigationItems->addItem( (*menuList) );
        if (m_navigation->getUsedNavType() == (*menuList)) {
          m_navigationItems->setChecked( (*menuList), true );
        }
        ++j;
      }
    }
  } else {
    vector<string> allTypes;
    allTypes.push_back("EXAMINE");
    allTypes.push_back("FLY");
    allTypes.push_back("WALK");
    allTypes.push_back("LOOKAT");
    allTypes.push_back("NONE");
    m_navTypeCount = allTypes.size();

    m_navigationItems->setItems(&allTypes);

    m_navigationItems->setChecked( "EXAMINE", true );
  }  
}

void H3DPluginInstance::buildViewpointMenuList() {
  if( !m_viewpointItems )
    m_viewpointItems = new MenuList();

  X3DViewpointNode::ViewpointList vplist;
  vplist = X3DViewpointNode::getAllViewpoints();
  
  list< ViewpointGroup *> groups;

  // Do getViewpointHierarchy()
  // Single out the groups
  for( X3DViewpointNode::ViewpointList::iterator i = vplist.begin();
       i != vplist.end(); ++i ) {
      if( ViewpointGroup *vg = dynamic_cast< ViewpointGroup * >( *i ) ) {
      groups.push_back( vg );
    }
  }

  // No groups found, use all viewpoints
  if( groups.size() == 0 ) {
    m_viewpointList = vplist;
  } else {
    // Go through all the viewpoints again and 'remove' the ones in the groups
    for( X3DViewpointNode::ViewpointList::iterator i = vplist.begin();
         i != vplist.end(); ++i ) {
      bool standalone_vp = true;
      if( !dynamic_cast< ViewpointGroup * >( *i ) ) {
        for( list< ViewpointGroup * >::iterator g = groups.begin();
             g != groups.end(); ++g ) {
          if( (*g)->containsViewpoint( *i ) ) {
            standalone_vp = false;
            break;
          }
        }
      }
      if( standalone_vp ) m_viewpointList.push_back( *i );
    }
  }

  /*
  string msg = "Hierarchy\n";
  if( m_viewpointList.empty() ) {
    for( X3DViewpointNode::ViewpointList::iterator vp =m_viewpointList.begin();
         vp != m_viewpointList.end(); ++vp ) {
      string desc = (*vp)->description->getValue();
      msg += "Item: " + desc + "\n";
    }
  } else {
    msg += "empty";
  }

  MessageBox(HWND_DESKTOP, msg.c_str(), "Debug", MB_OK);

  msg = "Everyone";
  for( X3DViewpointNode::ViewpointList::iterator vp = vplist.begin();
       vp != vplist.end(); ++vp ) {
    string desc = (*vp)->description->getValue();
    msg += "Item: " + desc + "\n";
  }
  MessageBox(HWND_DESKTOP, msg.c_str(), "Debug", MB_OK );
  */
  m_viewpointCount = m_viewpointList.size();
    if( m_viewpointCount <= 1 ) {
    m_viewpointItems->addItem("Default");
    m_viewpointItems->setChecked( "Default", false );
  } else {
    for( X3DViewpointNode::ViewpointList::iterator vp =m_viewpointList.begin();
         vp != m_viewpointList.end(); ++vp ) {
      string desc = (*vp)->description->getValue();
      m_viewpointItems->addItem( desc );
      if( (*vp) == Viewpoint::getActive() ) {
        m_viewpointItems->setChecked( desc, true );
      }
    }
  }
}

void H3DPluginInstance::onViewpointChange( string viewpoint ) {
  // If no extra viewpoints, do nothing
  if( m_viewpointCount <= 1 ) return;
  
  X3DViewpointNode::ViewpointList::iterator vp;
  for( vp = m_viewpointList.begin(); vp != m_viewpointList.end(); ++vp ) {
    if( (*vp)->description->getValue() == viewpoint ) break;
  }

  m_viewpointItems->setChecked( (*vp)->description->getValue(), true );

  //Enable that viewpoint
  (*vp)->set_bind->setValue(true);
}

void H3DPluginInstance::onNavigationChange( string cmd ) {
  // Should validate cmd (for now, just assume it will be ok)

  m_NavigationItem = cmd;

  if( m_navigation ) {
    m_navigation->setNavType(m_NavigationItem);
  } else {
    m_RenderWindow->default_nav = m_NavigationItem;
  }

  m_navigationItems->setChecked( m_NavigationItem, true );
}


void H3DPluginInstance::onMenuCommand( int menu, string cmd ) {
  switch( menu ) {
    case ID_NAVIGATION_MENU:
      onNavigationChange( cmd );
      break;
    case ID_VIEWPOINT_MENU:
      onViewpointChange( cmd );
  }
}

void H3DPluginInstance::onKeyDown( int key, bool special) {
  m_RenderWindow->onKeyDown( key, special );
}

void H3DPluginInstance::onKeyUp( int key, bool special ) {
  m_RenderWindow->onKeyUp( key, special );
}

void H3DPluginInstance::onLeftMouseButtonDown() {
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseButtonAction( MouseSensor::LEFT_BUTTON,
                                         MouseSensor::DOWN );
  }
}

void H3DPluginInstance::onLeftMouseButtonUp()     { 
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseButtonAction( MouseSensor::LEFT_BUTTON,
                                         MouseSensor::UP );
  }
}

void H3DPluginInstance::onMiddleMouseButtonDown() { 
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseButtonAction( MouseSensor::MIDDLE_BUTTON,
                                         MouseSensor::DOWN );
  }
}

void H3DPluginInstance::onMiddleMouseButtonUp()   { 
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseButtonAction( MouseSensor::MIDDLE_BUTTON,
                                         MouseSensor::UP );
  }
}

void H3DPluginInstance::onRightMouseButtonDown()  {
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseButtonAction( MouseSensor::RIGHT_BUTTON,
                                         MouseSensor::DOWN );
  }
}

void H3DPluginInstance::onRightMouseButtonUp()    {
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseButtonAction( MouseSensor::RIGHT_BUTTON,
                                         MouseSensor::UP );
  }
}

void H3DPluginInstance::onMouseMotion( int x, int y ) {
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseMotionAction( x, y );
  }
}

void H3DPluginInstance::onMouseWheel( int rotation ) {
  if( m_RenderWindow ) {
    m_RenderWindow->onMouseWheelAction(rotation );
  }
}
