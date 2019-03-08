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
/// \file MedX3DDemoMainFrame.cpp
/// \brief CPP file for MedX3DDemoMainFrame.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoMainFrame.h"
#include "MedX3DDemoApp.h"
#include "MedX3DDemoLoadRawImageDialog.h"

#include <H3D/Scene.h>
#include <H3D/X3D.h>
#include <H3D/MFString.h>
#include <H3D/Image3DTexture.h>
#include <H3D/ImageTexture.h>

using namespace H3D;

MedX3DDemoMainFrame::MedX3DDemoMainFrame( wxWindow* parent )
:
MainFrame( parent )
{

}

void MedX3DDemoMainFrame::OnClose( wxCloseEvent& event )
{
  MedX3DDemoApp::h3d_scene.reset( NULL );
  MedX3DDemoApp::h3d_window.reset( NULL );
  Destroy();
}

void MedX3DDemoMainFrame::OnQuit( wxCommandEvent& event ) {
  Close(true);
}

void MedX3DDemoMainFrame::OnIdle( wxIdleEvent& event )
{
   for( set< Scene * >::iterator i = Scene::scenes.begin();
       i != Scene::scenes.end();
       ++i ) {
    if( (*i)->isActive() )
      (*i)->idle();
   }
}

void MedX3DDemoMainFrame::OnLoadVolumeData( wxCommandEvent& event )
{
  auto_ptr< wxFileDialog > openFileDialog( new wxFileDialog ( this,
                                                              wxT("Open file"),
                                                              wxT(""),
                                                              wxT(""),
                                                              wxT("*.*"),
                                                              wxFD_OPEN,
                                                              wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    wxGetApp().loadVolumeData( string(openFileDialog->GetPath().mb_str()) );
  }
}

void MedX3DDemoMainFrame::OnLoadRawData( wxCommandEvent& event )
{
  auto_ptr< wxFileDialog > openFileDialog( new wxFileDialog ( this,
                                                              wxT("Open file"),
                                                              wxT(""),
                                                              wxT(""),
                                                              wxT("*.*"),
                                                              wxFD_OPEN,
                                                              wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    auto_ptr< MedX3DDemoLoadRawImageDialog > raw_image_dialog(
      new MedX3DDemoLoadRawImageDialog( this ) );
    if( raw_image_dialog->ShowModal() == wxID_OK ) { 
      wxGetApp().loadVolumeData( string( openFileDialog->GetPath().mb_str() ),
                                 raw_image_dialog->getNewFileReader() );
    }
  }
}

void MedX3DDemoMainFrame::OnClearCurrentData( wxCommandEvent& event )
{
}

void MedX3DDemoMainFrame::OnColorVolumeStyleMenuItem( wxCommandEvent& event )
{
  MedX3DDemoMainDialog *main_dialog = wxGetApp().main_dialog;
  if( event.IsChecked() ) {
    main_dialog->Show();
  } else {
    main_dialog->Hide();
  }
}

void MedX3DDemoMainFrame::OnConsoleWindowsMenu( wxCommandEvent& event )
{
  WxConsoleDialog *console_dialog =  wxGetApp().console_dialog;
  if( event.IsChecked() ) {
    console_dialog->Show();
  } else {
    console_dialog->Hide();
  }
}
