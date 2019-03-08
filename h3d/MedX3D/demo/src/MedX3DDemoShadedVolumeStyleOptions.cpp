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
/// \file MedX3DDemoShadedVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoShadedVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoShadedVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

#include <sstream>

using namespace H3D;

MedX3DDemoShadedVolumeStyleOptions::MedX3DDemoShadedVolumeStyleOptions( wxWindow* parent,
                                                                        ShadedVolumeStyle *_style )
:
  ShadedVolumeStyleOptions( parent ),
  shaded_volume_style( _style )
{

  if( !shaded_volume_style->material->getValue() ) {
    shaded_volume_style_material.reset( new Material );
  } else {
    shaded_volume_style_material.reset( (Material *)shaded_volume_style->material->getValue() );
  }

  RGB diffuse_color = shaded_volume_style_material->diffuseColor->getValue();
  RGB specular_color = shaded_volume_style_material->specularColor->getValue();
  RGB emissive_color = shaded_volume_style_material->emissiveColor->getValue();
  float transparency = shaded_volume_style_material->transparency->getValue();
  float ambient_intensity = shaded_volume_style_material->ambientIntensity->getValue();


  // colors
  DiffuseColorPicker->SetColour( wxColour( 255 * diffuse_color.r,
                                           255 * diffuse_color.g,
                                           255 * diffuse_color.b ) );
  SpecularColorPicker->SetColour( wxColour( 255 * specular_color.r,
                                            255 * specular_color.g,
                                            255 * specular_color.b ) );  
  EmissiveColorPicker->SetColour( wxColour( 255 * emissive_color.r,
                                            255 * emissive_color.g,
                                            255 * emissive_color.b ) );  

  // transparency
  std::stringstream ts;
  ts << transparency; 
  ShadedTransparencySlider->SetValue( transparency * 100 );
  ShadedTransparencyText->SetValue( wxString(ts.str().c_str(),wxConvUTF8) );

  // ambient intensity
  std::stringstream as;
  as << ambient_intensity; 
  ShadedAmbientIntensitySlider->SetValue( ambient_intensity * 100 );
  ShadedAmbientIntensityText->SetValue( wxString(as.str().c_str(),wxConvUTF8) );

  // check boxes
  ShadedLightingBox->SetValue( shaded_volume_style->lighting->getValue() );
  ShadedShadowsBox->SetValue( shaded_volume_style->shadows->getValue() );

  useMaterialAsColor( shaded_volume_style->material->getValue() != NULL );
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedDiffuseColorChanged( wxColourPickerEvent& event )
{
  wxColour c = event.GetColour();
  shaded_volume_style_material->diffuseColor->setValue( RGB( c.Red() / 255.f,
                                                             c.Green() / 255.f,
                                                             c.Blue() / 255.f ) );
  wxGetApp().style_dialog->setStyleModified( true );                                                           
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedEmissiveColorChanged( wxColourPickerEvent& event )
{
  wxColour c = event.GetColour();
  shaded_volume_style_material->emissiveColor->setValue( RGB( c.Red() / 255.f,
                                                              c.Green() / 255.f,
                                                              c.Blue() / 255.f ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedSpecularColorChanged( wxColourPickerEvent& event )
{
  wxColour c = event.GetColour();
  shaded_volume_style_material->specularColor->setValue( RGB( c.Red() / 255.f,
                                                              c.Green() / 255.f,
                                                              c.Blue() / 255.f ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedTransparencyScroll( wxScrollEvent& event )
{
  int pos = event.GetPosition();
  float value = pos / 100.f; 
  std::stringstream s;
  s << value;
  ShadedTransparencyText->SetValue( wxString(s.str().c_str(),wxConvUTF8) );
  shaded_volume_style_material->transparency->setValue( value );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedTransparencyText( wxCommandEvent& event )
{
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll( wxScrollEvent& event )
{
  int pos = event.GetPosition();
  float value = pos / 100.f;
  stringstream s;
  s << value;
  ShadedAmbientIntensityText->SetValue( wxString(s.str().c_str(),wxConvUTF8) );
  shaded_volume_style_material->ambientIntensity->setValue( value );
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedAmbientIntensityText( wxCommandEvent& event )
{
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedLightingBox( wxCommandEvent& event )
{
  shaded_volume_style->lighting->setValue( ShadedLightingBox->IsChecked() );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoShadedVolumeStyleOptions::OnShadedShadowsBox( wxCommandEvent& event )
{
  shaded_volume_style->shadows->setValue( ShadedShadowsBox->IsChecked() );
  wxGetApp().style_dialog->setStyleModified( true );
}


void MedX3DDemoShadedVolumeStyleOptions::OnShadedUseMaterialCheck( wxCommandEvent& event ) {
  useMaterialAsColor( event.IsChecked() );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoShadedVolumeStyleOptions::useMaterialAsColor( bool v ) {
  ShadedOnMaterialCheck->SetValue( v );
  if( v ) {
    shaded_volume_style->material->setValue( shaded_volume_style_material.get() );
    DiffuseColorPicker->Enable( true );
    SpecularColorPicker->Enable( true );
    EmissiveColorPicker->Enable( true );
    ShadedTransparencySlider->Enable( true );
    ShadedTransparencyText->Enable( true );
    ShadedAmbientIntensitySlider->Enable( true );
  ShadedAmbientIntensityText->Enable( true );
  
  DiffuseColorText->Enable( true );
  SpecularColorText->Enable( true );
  EmissiveColorText->Enable( true );
  ShadedTransparencyLabel->Enable( true );
  ShadedAmbientIntensityLabel->Enable( true );
  } else {
    shaded_volume_style->material->setValue( NULL );
  DiffuseColorPicker->Enable( false );
    SpecularColorPicker->Enable( false );
    EmissiveColorPicker->Enable( false );
    ShadedTransparencySlider->Enable( false );
    ShadedTransparencyText->Enable( false );
    ShadedAmbientIntensitySlider->Enable( false );
    ShadedAmbientIntensityText->Enable( false );

  DiffuseColorText->Enable( false );
  SpecularColorText->Enable( false );
  EmissiveColorText->Enable( false );
  ShadedTransparencyLabel->Enable( false );
  ShadedAmbientIntensityLabel->Enable( false );
  }
}
