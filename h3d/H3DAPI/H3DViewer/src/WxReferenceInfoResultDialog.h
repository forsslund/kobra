//////////////////////////////////////////////////////////////////////////////
//    Copyright 2018-2019, SenseGraphics AB
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
/// \file WxReferenceInfoResultDialog.h
/// \brief Header file for WxReferenceInfoResultDialog.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __WXREFERENCEINFORESULTDIALOG__
#define __WXREFERENCEINFORESULTDIALOG__


// ---------------------------------------------------------------------------
//  Console Dialog Class
// ---------------------------------------------------------------------------
#include <wx/wx.h>
#include <wx/clipbrd.h>
#include <wx/richtext/richtextctrl.h>
#include <wx/dialog.h>
#include <wx/dcbuffer.h>
#include <sstream>
#include <memory>
#include <H3DUtil/Threads.h>
#include <H3D/Scene.h>
#include <H3DUtil/Console.h>
#ifdef HAVE_PROFILER
/// This class provides a wxDialog with the output from scene
/// profiledResultStr.
class WxReferenceInfoResultDialog: public wxDialog
{
public:
  /// Constructor
  WxReferenceInfoResultDialog ( wxWindow *parent,
          wxWindowID id,
          const wxString &title,
          const wxPoint& pos = wxDefaultPosition,
          const wxSize& size = wxDefaultSize,
          long style = wxDEFAULT_DIALOG_STYLE
          );

  /// Destructor.
  ~WxReferenceInfoResultDialog();

  /// wx interface.
  wxTextCtrl *logText;

  void setDisplayedNode( H3D::Node *n, bool refresh = true );

  /// 
  wxClipboard *clip_board;
  
  wxString output;

  /// Used to make sure that output is done in the main wx thread.
  friend void wxLockGUI( void * );

  /// Reset changes in wxLockGUI.
  friend void wxUnlockGUI( void * );

protected:
  H3DUtil::AutoRef< H3D::Node > node;


  H3D::TimeStamp last_update;
  // The console stream. The contents of other_thread_output is eventuelly
  // transferred to this stream.
  //std::auto_ptr< std::ostream >console_stream;

  void updateTextOutput();

  void OnWindowClose( wxCloseEvent &event );
 
  void OnRefresh(wxCommandEvent & event);

  void OnCopyToClipboard(wxCommandEvent &event);

  // Called on idle.
  void OnIdle( wxIdleEvent &event );
 
  // Declare event table.
  DECLARE_EVENT_TABLE()
 
};
#endif
#endif
