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
/// \file MedX3DDemoToneMappedVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoToneMappedVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoToneMappedVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

using namespace H3D;

MedX3DDemoToneMappedVolumeStyleOptions::MedX3DDemoToneMappedVolumeStyleOptions( wxWindow* parent,
                                                                                ToneMappedVolumeStyle *style )
:
                                                                                ToneMappedVolumeStyleOptions( parent ),
                                                                                tone_mapped_style( style )
{
   RGBA cool_color = style->coolColor->getValue();
   RGBA warm_color = style->warmColor->getValue();
   
   CoolColorPicker->SetColour( wxColour( 255 * cool_color.r,
                                         255 * cool_color.g,
                                         255 * cool_color.b ) );

   std::stringstream ts;
   ts << cool_color.a; 
   ToneMappedCoolColorTransparencySlider->SetValue( cool_color.a * 100.0 );
   ToneMappedCoolColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) );
                                               
   WarmColorPicker->SetColour( wxColour( 255 * warm_color.r,
                                         255 * warm_color.g,
                                         255 * warm_color.b ) );  

   ts.str("");
   ts << warm_color.a; 
   ToneMappedWarmColorTransparencySlider->SetValue( warm_color.a * 100.0 );
   ToneMappedWarmColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) );

}


void MedX3DDemoToneMappedVolumeStyleOptions::OnCoolColorChanged( wxColourPickerEvent& event ) {
  wxColour c = event.GetColour();
  int transparency = ToneMappedCoolColorTransparencySlider->GetValue();
  tone_mapped_style->coolColor->setValue( RGBA( c.Red() / 255.f,
                                                c.Green() / 255.f,
                                                c.Blue() / 255.f,
                        transparency / 100.f ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoToneMappedVolumeStyleOptions::OnWarmColorChanged( wxColourPickerEvent& event ) {
  wxColour c = event.GetColour();
  int transparency = ToneMappedWarmColorTransparencySlider->GetValue();
  tone_mapped_style->warmColor->setValue( RGBA( c.Red() / 255.f,
                                               c.Green() / 255.f,
                                               c.Blue() / 255.f,
                         transparency / 100.f ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll( wxScrollEvent& event ) {
  int transparency = event.GetPosition();
  std::stringstream ts;
  ts << transparency/100.f;
  ToneMappedCoolColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) );
  RGBA color = tone_mapped_style->coolColor->getValue();
  color.a = transparency/100.f;
  tone_mapped_style->coolColor->setValue(color);
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll( wxScrollEvent& event ) {
  int transparency = event.GetPosition();
  std::stringstream ts;
  ts << transparency/100.f;
  ToneMappedWarmColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) );
  RGBA color = tone_mapped_style->warmColor->getValue();
  color.a = transparency/100.f;
  tone_mapped_style->warmColor->setValue(color);
  wxGetApp().style_dialog->setStyleModified( true );
}
