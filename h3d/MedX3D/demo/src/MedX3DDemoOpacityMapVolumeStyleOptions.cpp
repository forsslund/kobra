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
/// \file MedX3DDemoOpacityMapVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoOpacityMapVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoOpacityMapVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

#include <H3D/ImageTexture.h>
#include <H3D/Image3DTexture.h>
#include <H3D/MedX3D/WindowFunctionTexture.h>

using namespace H3D;

MedX3DDemoOpacityMapVolumeStyleOptions::MedX3DDemoOpacityMapVolumeStyleOptions( wxWindow* parent,
                                                                                OpacityMapVolumeStyle *style )
:
  OpacityMapVolumeStyleOptions( parent ),
  opacity_map_style( style ),
  window_texture( new WindowFunctionTexture ),
  file_texture( new Image3DTexture )
{
  string type = opacity_map_style->type->getValue();
  X3DTextureNode *t = opacity_map_style->transferFunction->getValue();

  bool use_window_texture = true;

  string filename = "None";

  if( ImageTexture *t2d = dynamic_cast< ImageTexture * >( t ) ) {
    const vector< string > &urls = t2d->url->getValue();
    file_texture->url->setValue( urls );
    use_window_texture = false;
    if( urls.size() > 0 ) {
      filename = urls[0];
    }
  } else if ( Image3DTexture *t3d = dynamic_cast< Image3DTexture * >( t ) ) {
    const vector< string > &urls = t3d->url->getValue();
    file_texture->url->setValue( urls );
    use_window_texture = false;
    if( urls.size() > 0 ) {
      filename = urls[0];
    }
  } else if( WindowFunctionTexture *wt = 
             dynamic_cast< WindowFunctionTexture * >( t ) ) {
    unsigned int width  =  wt->windowWidth->getValue();
    unsigned int center =  wt->windowCenter->getValue();
    window_texture->windowWidth->setValue( width );
    window_texture->windowCenter->setValue( center );

    WindowWidthSlider->SetValue( width * 100 / 256.f );
    WindowCenterSlider->SetValue( center * 100 / 256.f );
    stringstream w;
    w << width / 256.f;
    WindowWidthText->SetLabel( wxString( w.str().c_str(),wxConvUTF8  ) );
    stringstream c;
    c << center / 256.f;
    WindowCenterText->SetLabel( wxString( c.str().c_str(),wxConvUTF8  ) );
  }

  FilenameText->SetLabel( wxString( filename.c_str(),wxConvUTF8 ) );
  
  if( use_window_texture ) {
    opacity_map_style->transferFunction->setValue( window_texture );
    setTransferFunctionMode( wxT( "Window" ) );
  } else {
    opacity_map_style->transferFunction->setValue( file_texture );
    setTransferFunctionMode( wxT( "File" ) );
  }
  
  TypeChoice->SetStringSelection( wxString( type.c_str(),wxConvUTF8 ) );
}

void MedX3DDemoOpacityMapVolumeStyleOptions::OnTypeChoice( wxCommandEvent& event ) {
  wxString choice = event.GetString();
  opacity_map_style->type->setValue( string( choice.mb_str() ) );
}

void MedX3DDemoOpacityMapVolumeStyleOptions::OnTransferFunctionChoice( wxCommandEvent& event ) {
  setTransferFunctionMode( event.GetString() );
}

void MedX3DDemoOpacityMapVolumeStyleOptions::OnLoadImageButton( wxCommandEvent& /*event*/ ) {
  wxFileDialog *openFileDialog( new wxFileDialog ( this,
                                                   wxT("Open file"),
                                                   wxT(""),
                                                   wxT(""),
                                                   wxT("*.*"),
                                                   wxFD_OPEN,
                                                   wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    wxString path = openFileDialog->GetPath();
    FilenameText->SetLabel( path );
    file_texture->url->clear();
    file_texture->url->push_back( string( path.mb_str() ) );
    if( file_texture->image->getValue() )
      opacity_map_style->transferFunction->setValue( file_texture );
    wxGetApp().style_dialog->setStyleModified( true );
  }
}

void MedX3DDemoOpacityMapVolumeStyleOptions::OnWindowCenterScroll( wxScrollEvent& event ) {
  int pos = event.GetPosition();
  stringstream s;
  s << pos / 100.f; 
  WindowCenterText->SetLabel( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  window_texture->windowCenter->setValue( pos * 255 / 100.f );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoOpacityMapVolumeStyleOptions::OnWindowWidthScroll( wxScrollEvent& event ) {
  int pos = event.GetPosition();
  stringstream s;
  s << pos / 100.f; 
  WindowWidthText->SetLabel( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  window_texture->windowWidth->setValue( pos * 255 / 100.f );
  wxGetApp().style_dialog->setStyleModified( true );
}

bool MedX3DDemoOpacityMapVolumeStyleOptions::setTransferFunctionMode( wxString mode ) {
  TransferFunctionChoice->SetStringSelection( mode );
  wxGetApp().style_dialog->setStyleModified( true );
  if( mode == wxT("File") ) {
    // enable file options
    wxGetApp().enableAllWidgetsInSizer( FileOptionsSizer );
    // disable window options
    wxGetApp().disableAllWidgetsInSizer( WindowOptionsSizer );
    if( file_texture->image->getValue() )
      opacity_map_style->transferFunction->setValue( file_texture );
    else 
      opacity_map_style->transferFunction->setValue( NULL );
  } else if( mode == wxT("Window") ) {
    // enable window options
    wxGetApp().enableAllWidgetsInSizer( WindowOptionsSizer );
    // disable file options
    wxGetApp().disableAllWidgetsInSizer( FileOptionsSizer );
    opacity_map_style->transferFunction->setValue( window_texture );
    window_texture->image->getValue();
  } else {
    return false;
  }
  
  return true;
}
