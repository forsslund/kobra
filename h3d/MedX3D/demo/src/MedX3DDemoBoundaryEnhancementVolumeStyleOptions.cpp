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
/// \file MedX3DDemoBoundaryEnhancementVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoBoundaryEnhancementVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoBoundaryEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

using namespace H3D;

MedX3DDemoBoundaryEnhancementVolumeStyleOptions::MedX3DDemoBoundaryEnhancementVolumeStyleOptions( wxWindow* parent,
                                                                                                  BoundaryEnhancementVolumeStyle *style )
:
  BoundaryEnhancementVolumeStyleOptions( parent ),
  boundary_enhancement_style( style )
{
  
  H3DFloat retained = 
    boundary_enhancement_style->retainedOpacity->getValue();
  H3DFloat boundary = 
    boundary_enhancement_style->boundaryOpacity->getValue();
  H3DFloat factor = 
    boundary_enhancement_style->opacityFactor->getValue();

  stringstream bo;
  bo << boundary;
  BoundaryOpacityText->SetValue( wxString( bo.str().c_str(),wxConvUTF8  ) );

  stringstream of;
  of << factor;
  OpacityFactorText->SetValue( wxString( of.str().c_str(),wxConvUTF8  ) );

  RetainedOpacitySlider->SetValue( retained * 100 );
  stringstream t;
  t << retained;
  RetainedOpacityText->SetValue( wxString( t.str().c_str(),wxConvUTF8  ) );
}

void MedX3DDemoBoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll( wxScrollEvent& event )
{
  int retained = event.GetPosition();
  boundary_enhancement_style->retainedOpacity->setValue( retained / 100.f );
  stringstream s;
  s << retained / 100.f; 
  RetainedOpacityText->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoBoundaryEnhancementVolumeStyleOptions::OnBoundaryOpacityText( wxCommandEvent& /*event*/ )
{
  
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoBoundaryEnhancementVolumeStyleOptions::OnOpacityFactorText( wxCommandEvent& /*event*/ )
{
  wxString value = OpacityFactorText->GetValue();
  wxGetApp().style_dialog->setStyleModified( true );
}


