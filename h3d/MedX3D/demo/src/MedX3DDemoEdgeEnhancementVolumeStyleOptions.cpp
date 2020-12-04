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
/// \file MedX3DDemoEdgeEnhancementVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoEdgeEnhancementVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoEdgeEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

using namespace H3D;

MedX3DDemoEdgeEnhancementVolumeStyleOptions::MedX3DDemoEdgeEnhancementVolumeStyleOptions( wxWindow* parent,
                                                                                          EdgeEnhancementVolumeStyle *style)
:
  EdgeEnhancementVolumeStyleOptions( parent ),
  edge_enhancement_style( style )
{
  /*RGB edge_color = */edge_enhancement_style->edgeColor->getValue();
  H3DFloat gradient_threshold = 
    edge_enhancement_style->gradientThreshold->getValue();

  GradientThresholdSlider->SetValue( gradient_threshold * 100 );
  stringstream t;
  t << gradient_threshold;
  GradientThresholdText->SetValue( wxString( t.str().c_str(),wxConvUTF8  ) );
}

void MedX3DDemoEdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll( wxScrollEvent& event )
{
  int threshold = event.GetPosition();
  edge_enhancement_style->gradientThreshold->setValue( threshold / 100.f );
  stringstream s;
  s << threshold / 100.f; 
  GradientThresholdText->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoEdgeEnhancementVolumeStyleOptions::OnEdgeColorChanged( wxColourPickerEvent& event )
{
  wxColour c = event.GetColour();
  edge_enhancement_style->edgeColor->setValue( RGB( c.Red() / 255.f,
                                                    c.Green() / 255.f,
                                                    c.Blue() / 255.f ) );
  wxGetApp().style_dialog->setStyleModified( true );
}
