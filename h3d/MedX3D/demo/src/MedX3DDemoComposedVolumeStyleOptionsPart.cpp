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
/// \file MedX3DDemoComposedVolumeStyleOptionsPart.cpp
/// \brief CPP file for MedX3DDemoComposedVolumeStyleOptionsPart.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoComposedVolumeStyleOptionsPart.h"
#include "MedX3DDemoComposedVolumeStyleOptions.h"
#include "MedX3DDemoOpacityMapVolumeStyleOptions.h"
#include "MedX3DDemoBoundaryEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoBlendedVolumeStyleOptions.h"
#include "MedX3DDemoCartoonVolumeStyleOptions.h"
#include "MedX3DDemoEdgeEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoShadedVolumeStyleOptions.h"
#include "MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoToneMappedVolumeStyleOptions.h"
#include <wx/choicdlg.h>

using namespace H3D;

MedX3DDemoComposedVolumeStyleOptionsPart::MedX3DDemoComposedVolumeStyleOptionsPart( wxWindow* parent,
                                                                                    X3DComposableVolumeRenderStyleNode* style )
:
ComposedVolumeStyleOptionsPart( parent )
{
  if( style ) {
    OptionsSizer->Clear( true );
    OptionsSizer->Add( createPanelForStyle( this, style ), 0, wxEXPAND  );
    OptionsSizer->Layout();
  }

  FlexSizer->Layout();
  this->Layout();
}

void MedX3DDemoComposedVolumeStyleOptionsPart::OnAddStyleButton( wxCommandEvent& /*event*/ )
{
  wxString choices[] = { wxT("OpacityMapVolumeStyle" ),
                       wxT("BlendedVolumeStyle" ),
                         wxT("BoundaryEnhancementVolumeStyle" ),
                         wxT("CartoonVolumeStyle" ),
                         wxT("EdgeEnhancementVolumeStyle" ),
                         wxT("ShadedVolumeStyle" ),
                         wxT("SilhouetteEnhancementVolumeStyle" ),
                         wxT("ToneMappedVolumeStyle") };
  wxSingleChoiceDialog *type_choice = new wxSingleChoiceDialog( this, 
                                                                wxT( "Choose the style type to add" ),
                                                                wxT( "" ),
                                                                7,
                                                                choices );
  if( type_choice->ShowModal() == wxID_OK ) {
    wxString name = type_choice->GetStringSelection();
    X3DComposableVolumeRenderStyleNode *n = 
      static_cast< X3DComposableVolumeRenderStyleNode * > (H3DNodeDatabase::createNode( string( name.mb_str() ) ) );
    MedX3DDemoComposedVolumeStyleOptions *parent = 
      static_cast< MedX3DDemoComposedVolumeStyleOptions * >( GetParent() );
    parent->insertStyle( n, parent->getStyleOptionPosition( this ) + 1 );  
  }                                                             
}

void MedX3DDemoComposedVolumeStyleOptionsPart::OnRemoveStyleButton( wxCommandEvent& /*event*/ )
{
   MedX3DDemoComposedVolumeStyleOptions *parent = 
      static_cast< MedX3DDemoComposedVolumeStyleOptions * >( GetParent() );
   parent->removeStyleOption( this  );  
}

wxPanel *MedX3DDemoComposedVolumeStyleOptionsPart::createPanelForStyle( wxWindow *parent,
                                                                        X3DComposableVolumeRenderStyleNode *s ) {
  wxPanel *new_options = NULL;

  // Add new style options
  if( OpacityMapVolumeStyle *opacity_style = 
        dynamic_cast< OpacityMapVolumeStyle * >( s ) ) {
    new_options =  
      new MedX3DDemoOpacityMapVolumeStyleOptions( parent, opacity_style );
  } else if( BoundaryEnhancementVolumeStyle *style = 
        dynamic_cast< BoundaryEnhancementVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoBoundaryEnhancementVolumeStyleOptions( parent, style );  
  } else if( BlendedVolumeStyle *style = 
        dynamic_cast< BlendedVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoBlendedVolumeStyleOptions( parent, style );    
  } else if( CartoonVolumeStyle *cartoon_style = 
        dynamic_cast< CartoonVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoCartoonVolumeStyleOptions( parent, cartoon_style );    
  } else if( EdgeEnhancementVolumeStyle *style = 
        dynamic_cast< EdgeEnhancementVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoEdgeEnhancementVolumeStyleOptions( parent, style );    
  } else if( ShadedVolumeStyle *style = 
        dynamic_cast< ShadedVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoShadedVolumeStyleOptions( parent, style );    
  } else if( SilhouetteEnhancementVolumeStyle *style = 
        dynamic_cast< SilhouetteEnhancementVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoSilhouetteEnhancementVolumeStyleOptions( parent, style );    
  } else if( ToneMappedVolumeStyle *tm_style = 
        dynamic_cast< ToneMappedVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoToneMappedVolumeStyleOptions( parent, tm_style );    
  }
  return new_options;
}
