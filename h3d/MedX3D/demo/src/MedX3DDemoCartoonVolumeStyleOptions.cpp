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
/// \file MedX3DDemoCartoonVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoCartoonVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoCartoonVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

using namespace H3D;

MedX3DDemoCartoonVolumeStyleOptions::MedX3DDemoCartoonVolumeStyleOptions( wxWindow* parent,
                                                                          CartoonVolumeStyle *style )
:
  CartoonVolumeStyleOptions( parent ),
  cartoon_style( style )
{
   RGBA par_color = style->parallelColor->getValue();
   RGBA orth_color = style->orthogonalColor->getValue();
   int nr_steps = style->colorSteps->getValue();
   
   OrthogonalColorPicker->SetColour( wxColour( 255 * orth_color.r,
                                               255 * orth_color.g,
                                               255 * orth_color.b ) );
   CartoonOrthogonalColorTransparencySlider->SetValue( orth_color.a * 100 );
   std::stringstream ts;
   ts << orth_color.a; 
   CartoonOrthogonalColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) );
                                               
   ParallelColorPicker->SetColour( wxColour( 255 * par_color.r,
                                             255 * par_color.g,
                                             255 * par_color.b ) );
   CartoonParallelColorTransparencySlider->SetValue( par_color.a * 100 );
   ts.str("");
   ts << par_color.a; 
   CartoonParallelColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) );
                                             
   NrStepsSlider->SetValue( nr_steps );
   wxString t;
   t << nr_steps;
   NrStepsText->SetValue( t );
                                                
}

void MedX3DDemoCartoonVolumeStyleOptions::OnParallelColorChange( wxColourPickerEvent& event ) {
  wxColour c = event.GetColour();
  int transparency = CartoonParallelColorTransparencySlider->GetValue();
  cartoon_style->parallelColor->setValue( RGBA( c.Red() / 255.f,
                                               c.Green() / 255.f,
                                               c.Blue() / 255.f,
                         transparency / 100.f ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoCartoonVolumeStyleOptions::OnOrthogonalColorChange( wxColourPickerEvent& event ) {
  wxColour c = event.GetColour();
  int transparency = CartoonOrthogonalColorTransparencySlider->GetValue();
  cartoon_style->orthogonalColor->setValue( RGBA( c.Red() / 255.f,
                                                 c.Green() / 255.f,
                                                 c.Blue() / 255.f,
                         transparency / 100.f ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoCartoonVolumeStyleOptions::OnParallelColorTransparencyScroll( wxScrollEvent& event ) {
  int transparency = event.GetPosition();
  std::stringstream ts;
  ts << transparency/100.f;
  CartoonParallelColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) ); 
  RGBA color = cartoon_style->parallelColor->getValue();
  color.a = transparency/100.f;
  cartoon_style->parallelColor->setValue(color);
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoCartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll( wxScrollEvent& event ){
  int transparency = event.GetPosition();
  std::stringstream ts;
  ts << transparency/100.f;
  CartoonOrthogonalColorTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) ); 
  RGBA color = cartoon_style->orthogonalColor->getValue();
  color.a = transparency/100.f;
  cartoon_style->orthogonalColor->setValue(color);
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoCartoonVolumeStyleOptions::OnStepsScroll( wxScrollEvent& event ) {
  int nr_steps = event.GetPosition();
  cartoon_style->colorSteps->setValue( nr_steps );
  wxString s;
  s << nr_steps; 
  NrStepsText->SetValue( s ); 
  wxGetApp().style_dialog->setStyleModified( true );
}


