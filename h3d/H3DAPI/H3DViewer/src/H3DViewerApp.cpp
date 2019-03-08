//////////////////////////////////////////////////////////////////////////////
//    Copyright 2006-2019, SenseGraphics AB
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
/// \file H3DViewer.cpp
/// \brief CPP file for H3DViewer.
///
//
//////////////////////////////////////////////////////////////////////////////

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------
#include "WxFrame.h"
#include "WxWidgetsWindow.h"

#include <H3D/Scene.h>

#include <H3D/PythonScript.h>

#include <wx/cmdline.h>
#include <H3DUtil/TimeStamp.h>
#include <fstream>
#include <wx/debugrpt.h>
#include <H3D/H3DApi.h>
#if defined( H3D_WINDOWS )
#include "MiniDump.h"
#endif

#ifdef HAVE_PYTHON
#include <wx/stdpaths.h>
#endif

// ---------------------------------------------------------------------------
//  Required classes and definitions
// ---------------------------------------------------------------------------
using namespace std;
using namespace H3D;

#if (defined( wxUSE_UNICODE ) && wxMAJOR_VERSION == 2 && wxMINOR_VERSION <= 8)
#define CMDLINEDESC(SHORTNAME,LONGNAME,DESCRIPTION)  wxT(SHORTNAME),wxT(LONGNAME),wxT(DESCRIPTION) 
#else
#define CMDLINEDESC(SHORTNAME,LONGNAME,DESCRIPTION)  SHORTNAME,LONGNAME,DESCRIPTION 
#endif

const wxCmdLineEntryDesc gCmdLineDesc[] = {
  { 
    wxCMD_LINE_SWITCH,
    CMDLINEDESC("dp","disable_plugins", "No registered plugins are loaded on startup. They are however still listed in the plugins dialog and can be enabled if desired"),
    wxCMD_LINE_VAL_NONE, 0
  },
  {
    wxCMD_LINE_OPTION,
    CMDLINEDESC("","screen","Specify opengl rendering area width and height. The <str> is of the form <W>x<H>. Note that the operating system might limit the size range due to captions and borders. Consider using --no_border if this is a problem."),
    wxCMD_LINE_VAL_STRING, 0
  },
  {
    wxCMD_LINE_OPTION,
    CMDLINEDESC("","window_position","Specify window origin x and y value. The <str> is of the form <X>x<Y>. Note that the operating system might limit the allowed positions due to captions and borders. Consider using --no_border if this is a problem."),
    wxCMD_LINE_VAL_STRING, 0
  },
  {
    wxCMD_LINE_OPTION,
    CMDLINEDESC("","rendermode","Specify the window stereo mode."),
    wxCMD_LINE_VAL_STRING, 0
  },
  {
    wxCMD_LINE_SWITCH,
    CMDLINEDESC("f","fullscreen","Enable fullscreen"),
    wxCMD_LINE_VAL_NONE, 0
  },
  {
    wxCMD_LINE_SWITCH,
    CMDLINEDESC("logInitTime","logInitTime","Log the time used for viewer initialization"),
    wxCMD_LINE_VAL_NONE, 0
  },
  {
    wxCMD_LINE_SWITCH,
    CMDLINEDESC("","no_border","Remove borders and captions from the window."),
    wxCMD_LINE_VAL_NONE, 0
  },
  {
    wxCMD_LINE_SWITCH,
    CMDLINEDESC("","no_menu","Remove menu and statusbar. This will disable keyboard shortcuts for such things as navigation."),
    wxCMD_LINE_VAL_NONE, 0
  },
  {
    wxCMD_LINE_SWITCH,
    CMDLINEDESC("silent","silent","Run H3DViewer without displaying window"),
    wxCMD_LINE_VAL_NONE, 0
  },
  {
    wxCMD_LINE_SWITCH,
    CMDLINEDESC("","help","Display this help message and exit."),
    wxCMD_LINE_VAL_NONE, 0
  },
#ifdef HAVE_NVAPI
  {
    wxCMD_LINE_OPTION,
    CMDLINEDESC("","nvsettings","Path to a file that contains a NvidiaSettings node as it top most node."),
    wxCMD_LINE_VAL_STRING, 0
  },
#endif
  { wxCMD_LINE_PARAM, NULL, NULL, 
#if( defined( wxUSE_UNICODE ) && wxMAJOR_VERSION == 2 && wxMINOR_VERSION <= 8 )
    wxT("File to load"), 
#else 
    "File to load", 
#endif
wxCMD_LINE_VAL_STRING,
    wxCMD_LINE_PARAM_OPTIONAL }, 
  { wxCMD_LINE_NONE, NULL, NULL, NULL, wxCMD_LINE_VAL_NONE, 0} };


#if defined( H3D_WINDOWS )

bool WriteCrashDump(PEXCEPTION_POINTERS pExceptionPtrs)
{
  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );
  stringstream timestamp;

  timestamp << (now->tm_year + 1900)
    << (now->tm_mon + 1)
    << std::setw( 2 ) << std::setfill( '0' )
    <<  now->tm_mday << '_'
    << std::setw( 2 ) << std::setfill( '0' )
    <<  now->tm_hour
    << std::setw( 2 ) << std::setfill( '0' )
    <<  now->tm_min
    << std::setw( 2 ) << std::setfill( '0' )
    <<  now->tm_sec;

  string session_uuid;
  ifstream fh;
  fh.open("session_uuid", std::ofstream::in);
  if (fh.fail()) {
    session_uuid = "";
  }
  else {
    fh >> session_uuid;
    session_uuid = "_" + session_uuid;
  }

  string threadcrashFilename = "log\\" + timestamp.str() + session_uuid + "_ThreadCrashDump";

  std::wstring sPathFilename(threadcrashFilename.size(), L' '); // Overestimate number of code points.
  sPathFilename.resize(std::mbstowcs(&sPathFilename[0], threadcrashFilename.c_str(), threadcrashFilename.size())); // Shrink to fit.
  sPathFilename += L".dmp";
  MiniDump mndmp;
  if(mndmp.Create(sPathFilename.c_str(),MiniDump::kInfoLevelSmall,pExceptionPtrs,false))
  {
    return true;
  }
  else {
    return false;
  }
}

LONG WINAPI VectoredExceptionHandler(PEXCEPTION_POINTERS pExceptionInfo)
{
  WriteCrashDump(pExceptionInfo);

  return EXCEPTION_CONTINUE_SEARCH;
}

LONG WINAPI MyUnhandledExceptionFilter(PEXCEPTION_POINTERS pExceptionPtrs)
{
  WriteCrashDump(pExceptionPtrs);

  // Execute default exception handler next
  return EXCEPTION_EXECUTE_HANDLER;
}

#endif

// Define a new application type
class MyApp: public wxApp
{
public:
  MyApp():
    theWxFrame( NULL ),
    startupTime(TimeStamp()){
      window_height = -1;
      window_width = -1;
      window_pos_x = -1;
      window_pos_y = -1;
      fullscreen = false;
      stereo_mode = wxString( "NONE", wxConvUTF8 );
      silent = false;
      logInitTime = false;
      no_border = false;
      no_menu = false;
      wxHandleFatalExceptions();
#if defined( H3D_WINDOWS )
      //AddVectoredExceptionHandler(1, VectoredExceptionHandler);
      if (char *buffer = getenv("H3D_CREATE_DBGDUMPFILE")) {
          if (strcmp(buffer, "TRUE") == 0) {
              SetUnhandledExceptionFilter(MyUnhandledExceptionFilter);
          }
      }      
#endif
  }
  virtual bool OnInit();
  virtual void MacOpenFile(const wxString &fileName) {
    if( theWxFrame ) {
      theWxFrame->clearData();
      theWxFrame->loadFile(toStr(fileName)); 
    } else {
      cmd_line_filename = fileName; 
    }
  }
  virtual void OnIdle(wxIdleEvent& event);
  virtual bool OnExceptionInMainLoop();
  virtual void OnFatalException();
  void GenerateReport(wxDebugReport::Context ctx);
  virtual void OnInitCmdLine(wxCmdLineParser& parser) {
    parser.SetDesc (gCmdLineDesc); 
  }

  // Used to parse arguments whose value should be of the style "<int>x<int>".
  bool parseWindowSettingsArgumentValue( const wxString &window_settings, int &first_value, int &second_value, const wxString &option_name_for_error_message ) {
    wxString window_settings_first_part = window_settings.BeforeFirst(wxChar('x') );
    wxString window_settings_second_part = window_settings.AfterFirst(wxChar('x') );
    if( !window_settings_first_part.ToLong( (long*)&first_value ) || !window_settings_second_part.ToLong( (long*)&second_value ) ) {
      wxMessageDialog invalid_value_dialog ( NULL, wxString("'",wxConvUTF8) + window_settings +
                                                   wxString("' is not a correct value for option '",wxConvUTF8) +
                                                   option_name_for_error_message +
                                                   wxString("'",wxConvUTF8), wxString(H3DVIEWER_APP_NAME,wxConvUTF8), wxOK);
      invalid_value_dialog.ShowModal();
      return false;
    }
    return true;
  }

  virtual bool OnCmdLineParsed(wxCmdLineParser& parser) {
    if( parser.Found( wxString( "help",wxConvUTF8 ) ) ) {
      parser.Usage();
      return false;
    }
    disable_plugin_dialog = parser.Found( wxString( "dp", wxConvUTF8 ) );
    wxString window_settings;
    if( parser.Found( wxString( "screen",wxConvUTF8 ), &window_settings ) ) {
      if( !parseWindowSettingsArgumentValue( window_settings, window_width, window_height, wxString("screen",wxConvUTF8) ) ) {
        parser.Usage();
        return false;
      }
    }
    if( parser.Found( wxString( "window_position",wxConvUTF8 ), &window_settings ) ) {
      if( !parseWindowSettingsArgumentValue( window_settings, window_pos_x, window_pos_y, wxString("window_position",wxConvUTF8) ) ) {
        parser.Usage();
        return false;
      }
    }
    parser.Found( wxString( "rendermode",wxConvUTF8 ), &stereo_mode );
    logInitTime = parser.Found(wxString("logInitTime",wxConvUTF8));
    silent = parser.Found(wxString("silent", wxConvUTF8));
    fullscreen = parser.Found( wxString( "f", wxConvUTF8 ));
    no_border = parser.Found(  wxString( "no_border", wxConvUTF8 ));
    no_menu = parser.Found(  wxString( "no_menu", wxConvUTF8 ));
#ifdef HAVE_NVAPI
    wxString nvsettings;
    if( parser.Found( wxString( "nvsettings", wxConvUTF8 ), &nvsettings ) ) {
      Scene::nvidia_graphics_options_file_url = toStr( nvsettings );
    }
#endif
    if( parser.GetParamCount() > 0 ) {
      cmd_line_filename = parser.GetParam(0);
    }

    return true;
  }
protected:
  wxString cmd_line_filename;
  wxString stereo_mode;
  int window_width;
  int window_height;
  int window_pos_x;
  int window_pos_y;
  bool fullscreen;
  bool silent;
  bool logInitTime;
  bool no_border;
  bool no_menu;

  TimeStamp startupTime;
  bool disable_plugin_dialog;
  WxFrame *theWxFrame;
  DECLARE_EVENT_TABLE()
};

void MyApp::GenerateReport(wxDebugReport::Context ctx)
{
  wxDebugReportCompress *report = new wxDebugReportCompress;
  wxString dumpPath = wxGetCwd() + wxString("\\log", wxConvUTF8);

  string session_uuid;
  ifstream fh;
  fh.open("session_uuid", std::ofstream::in);
  if (fh.fail()) {
    session_uuid = "";
  }
  else {
    fh >> session_uuid;
    session_uuid = "_" + session_uuid;
  }

  wxDateTime dt = wxDateTime::Now();
  wxString datepart = dt.FormatISODate();
  datepart.Replace(wxString("-", wxConvUTF8),wxString("", wxConvUTF8));
  wxString timepart = dt.FormatISOTime();
  timepart.Replace(wxString(":", wxConvUTF8),wxString("", wxConvUTF8));
  wxString dumpFilename = datepart + wxString("_", wxConvUTF8) + timepart + wxString(session_uuid.c_str(), wxConvUTF8) +  wxString("_wxCrashDump", wxConvUTF8);

#if wxMAJOR_VERSION > 2 || (wxMAJOR_VERSION == 2 && wxMINOR_VERSION > 8)
  report->SetCompressedFileDirectory(dumpPath);
  report->SetCompressedFileBaseName(dumpFilename);
#endif

  // this will add the minidump and an xml file with the loaded DLLs
  report->AddAll(wxDebugReport::Context_Exception);

  report->Process();
  delete report;
}

BEGIN_EVENT_TABLE(MyApp, wxApp)
  EVT_IDLE(MyApp::OnIdle)
END_EVENT_TABLE()

void MyApp::OnIdle(wxIdleEvent& event) {
  for( set< Scene * >::iterator i = Scene::scenes.begin();
    i != Scene::scenes.end();
    ++i ) {
    if( (*i)->isActive() )
      (*i)->idle();
  }

#if defined( H3D_WINDOWS ) || defined( H3D_OSX ) || ( wxMAJOR_VERSION <= 2 && wxMINOR_VERSION < 9 )
  wxApp::OnIdle(event);
#endif
}

IMPLEMENT_APP(MyApp)

bool MyApp::OnExceptionInMainLoop() {
  try {
    throw;
  }
  catch (const Exception::QuitAPI &) {
    // Ensure we exit cleanly even on QuitAPI exception
    wxCommandEvent fake_event;
    theWxFrame->OnExit ( fake_event );
    return false;
  }
  catch (const Exception::H3DException &e) {
     stringstream s;
   s << e;
     wxMessageBox( wxString(s.str().c_str(),wxConvLibc),
                   wxT("Error"), wxOK | wxICON_EXCLAMATION);
     GenerateReport(wxDebugReport::Context_Exception);
     return false;
  }

  GenerateReport(wxDebugReport::Context_Exception);
  wxApp::OnExceptionInMainLoop();  
}

void MyApp::OnFatalException()
{
  GenerateReport(wxDebugReport::Context_Exception);
}

bool MyApp::OnInit()
{
  H3DTIMER_BEGIN( "Init H3DViewer" );
  try {
    // call default behaviour (mandatory)
    if (!wxApp::OnInit())
      return false;

    SetVendorName(_T("SenseGraphics AB"));
    wxString tmp_string = H3DVIEWER_APP_NAME;
#ifdef H3D_ARCH64
    tmp_string += wxT("(x64)");
#else
    tmp_string += wxT("(x86)");
#endif 
#ifndef H3DVIEWER_STANDALONE
    tmp_string += wxT("(dev)");
#endif
#ifdef _DEBUG
    tmp_string += wxT("(debug)");
#endif
    SetAppName( tmp_string );
  
    Console.setShowLevel( false );

#ifdef H3DAPI_LIB
    initializeH3D(); 
#endif
    
#ifdef HAVE_PYTHON
#ifdef wxUSE_UNICODE
    // argv is wxCmdBufferArray when unicode that defines a wchar_t** converter.
    PythonScript::setargv( wxApp::argc, (wchar_t **)wxApp::argv );
#else
    // argv is char * in non unicode builds
    PythonScript::setargv( wxApp::argc, wxApp::argv );
#endif
    PythonScript::initPythonHome( toStr( wxStandardPaths::Get().GetExecutablePath() ) );
#endif
    int window_height_to_use = 600;
    int window_width_to_use = 800;
    if(window_width!=-1&&window_height!=-1){
      // use specified window size only when both specified width and height
      // are not -1
      window_height_to_use = window_height;
      window_width_to_use = window_width;
    }
    wxPoint window_position = wxDefaultPosition;
    if( window_pos_x!=-1 && window_pos_y!=-1 ) {
      window_position = wxPoint(window_pos_x,window_pos_y);
    }
    long window_style = wxDEFAULT_FRAME_STYLE;
    if( silent ) {
      // when silent mode is required, disable caption and also
      // force the width, height to be zero to make sure nothing will
      // be able to be displayed, also disable fullscreen
      window_style = wxDEFAULT_FRAME_STYLE&~(wxCAPTION);
    }
    if( no_border ) {
      window_style = wxBORDER_NONE;
    }
    // create a window to display
    theWxFrame = new WxFrame(NULL, wxID_ANY, wxT("H3DViewer"),
           window_position, wxSize(window_width_to_use, window_height_to_use), window_style, wxT("H3D Player"), !cmd_line_filename.IsEmpty(), disable_plugin_dialog, fullscreen, no_menu );
    if( silent ) {
      theWxFrame->Hide();
    }else{
      theWxFrame->Show(true);
    }
    // switch to fullscreen right now, and also change the window fullscreen value
    theWxFrame->glwindow->fullscreen->setValue(fullscreen);
    theWxFrame->glwindow->setFullscreen(fullscreen);
    if( toStr(stereo_mode) != "NONE" ){
      theWxFrame->glwindow->renderMode->setValue(toStr(stereo_mode));    
    }
    if( fullscreen&&window_width!=-1&&window_height!=-1 ) {
      // resize window size to required value, after fullscreen is applied
      // only do this when no default windows size is used
      // also reset the position of the window as go to fullscreen may modified
      // the starting position as well
      theWxFrame->SetPosition(window_position);
      theWxFrame->SetSize(window_width_to_use, window_height_to_use);
      theWxFrame->SetClientSize(window_width_to_use, window_height);
    }
    
    if( !cmd_line_filename.IsEmpty() ) 
    {
        theWxFrame->clearData();
        theWxFrame->loadFile(toStr(cmd_line_filename));
    }
  

    //This next line is used to set the icon file h3d.ico, when created.
    //theWxframe->SetIcon(wxIcon(_T("h3d_icn")));
    
    
    // Using this line instead of the two previous lines will make
    // WxWidgetsWindow create an instance of a wxframe with no menus and use
    // this as parent to the canvas.
    // WxWidgetsWindow *glwindow = new WxWidgetsWindow();

  } catch (const Exception::H3DException &e) {
    Console(LogLevel::Error) << e << endl;
    return false;
  }


  if( logInitTime ) {
    float init_time = TimeStamp()-startupTime;
    ofstream logFile("H3DViewerLog.txt");
    logFile<<"init_time:"<<init_time<<endl;
    logFile.close();
  }
  H3DTIMER_END( "Init H3DViewer" );
  return true;
}

