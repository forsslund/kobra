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
/// \file MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoApp.h"

using namespace H3D;

MedX3DDemoSilhouetteEnhancementVolumeStyleOptions::MedX3DDemoSilhouetteEnhancementVolumeStyleOptions( wxWindow* parent,
                                                                                                      SilhouetteEnhancementVolumeStyle *style )
:
  SilhouetteEnhancementVolumeStyleOptions( parent ),
  silhouette_style( style )
{

  H3DFloat sharpness = silhouette_style->silhouetteSharpness->getValue(); 
  H3DFloat boundary  = silhouette_style->silhouetteBoundaryOpacity->getValue(); 
  H3DFloat retained  = silhouette_style->silhouetteRetainedOpacity->getValue(); 

  stringstream s;
  s << sharpness;
  SharpnessText->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) );

  stringstream b;
  b << boundary;
  BoundaryOpacityText->SetValue( wxString( b.str().c_str(),wxConvUTF8  ) );


  RetainedOpacitySlider->SetValue( retained * 100 );
  stringstream r;
  r << retained;
  RetainedOpacityText->SetValue( wxString( r.str().c_str(),wxConvUTF8  ) );
}

void MedX3DDemoSilhouetteEnhancementVolumeStyleOptions::OnBoundaryOpacityChanged( wxCommandEvent& event ) {
  float v = atof( event.GetString().mb_str() );
  silhouette_style->silhouetteBoundaryOpacity->setValue( v );
  stringstream b;
  b << v;
  BoundaryOpacityText->SetValue( wxString( b.str().c_str(),wxConvUTF8  ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoSilhouetteEnhancementVolumeStyleOptions::OnSharpnessChanged( wxCommandEvent& event ) {
  float v = atof( event.GetString().mb_str() );
  silhouette_style->silhouetteSharpness->setValue( v );
  stringstream b;
  b << v;
  SharpnessText->SetValue( wxString( b.str().c_str(),wxConvUTF8  ) );
  wxGetApp().style_dialog->setStyleModified( true );
}

void MedX3DDemoSilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll( wxScrollEvent& event ) {
  int retained = event.GetPosition();
  silhouette_style->silhouetteRetainedOpacity->setValue( retained / 100.f );
  stringstream s;
  s << retained / 100.f; 
  RetainedOpacityText->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) ); 
  wxGetApp().style_dialog->setStyleModified( true );

}
