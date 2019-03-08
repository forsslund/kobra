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
/// \file MedX3DDemoMIPVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoMIPVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoMIPVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

using namespace H3D;

MedX3DDemoMIPVolumeStyleOptions::MedX3DDemoMIPVolumeStyleOptions( wxWindow* parent,
                                                                  ProjectionVolumeStyle *style )
:
  MIPVolumeStyleOptions( parent ),
  mip_style( style )
{
  H3DFloat threshold = mip_style->intensityThreshold->getValue(); 
  IntensityThresholdSlider->SetValue( threshold * 100 );
  stringstream t;
  t << threshold;
  IntensityThresholdText->SetValue( wxString( t.str().c_str(),wxConvUTF8  ) );

  string type = mip_style->type->getValue();
  ProjectionStyleTypeChoice->SetStringSelection( wxString( type.c_str(),wxConvUTF8  ) );
}


void MedX3DDemoMIPVolumeStyleOptions::OnIntensityThresholdSlider( wxScrollEvent& event ) {
  int threshold = event.GetPosition();
  mip_style->intensityThreshold->setValue( threshold / 100.f );
  stringstream s;
  s << threshold / 100.f; 
  IntensityThresholdText->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoMIPVolumeStyleOptions::OnProjectionStyleTypeChoice( wxCommandEvent& event ) {
  wxString choice = event.GetString();
  mip_style->type->setValue( string( choice.mb_str() ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

