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
/// \file MedX3DDemoStyleDialog.cpp
/// \brief CPP file for MedX3DDemoStyleDialog.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoStyleDialog.h"
#include "MedX3DDemoApp.h"
#include "MedX3DDemoBlendedVolumeStyleOptions.h"
#include "MedX3DDemoOpacityMapVolumeStyleOptions.h"
#include "MedX3DDemoBoundaryEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoComposedVolumeStyleOptions.h"
#include "MedX3DDemoCartoonVolumeStyleOptions.h"
#include "MedX3DDemoEdgeEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoMIPVolumeStyleOptions.h"
#include "MedX3DDemoShadedVolumeStyleOptions.h"
#include "MedX3DDemoSilhouetteEnhancementVolumeStyleOptions.h"
#include "MedX3DDemoToneMappedVolumeStyleOptions.h"

#include <wx/config.h>

using namespace H3D;

MedX3DDemoStyleDialog::MedX3DDemoStyleDialog( wxWindow* parent )
:
  StyleDialog( parent ),
  current_render_style_sizer( NULL )
{
  initializeAvailableRenderStyleList();
}

void MedX3DDemoStyleDialog::OnRenderStyleChoice( wxCommandEvent& event )
{
  if( !doSaveStyleChangesDialog() ) return;

  wxString choice = StyleNameChoice->GetStringSelection();
  if( wxGetApp().style_nodes.find( choice ) == wxGetApp().style_nodes.end() ) return;
  X3DVolumeRenderStyleNode *s = wxGetApp().style_nodes[choice].get();
  
  setStyleOptions( s );
  setStyleModified( false );
  selected_style = choice;
}


void MedX3DDemoStyleDialog::initializeAvailableRenderStyleList() {
  for( MedX3DDemoApp::StyleNodeMap::iterator i = wxGetApp().style_nodes.begin();
       i != wxGetApp().style_nodes.end(); ++i ) {
    StyleNameChoice->Append( (*i).first );
  }
}

void MedX3DDemoStyleDialog::OnNewStyleButton( wxCommandEvent& event ) {
  if( !doSaveStyleChangesDialog() ) return;

  wxTextEntryDialog *enterTextDialog( new wxTextEntryDialog ( this,
                                                              wxT("Enter new style name") ) );
 
  // Open an volume data file
  if (enterTextDialog->ShowModal() == wxID_OK) {
    wxString name = enterTextDialog->GetValue();

    // make sure the name entered is a unique name
    if( wxGetApp().style_nodes.find( name ) != wxGetApp().style_nodes.end() ) {
      wxMessageBox( wxT( "Invalid name. Name already exists" ) );
      return;
    }
    
    OpacityMapVolumeStyle *s = new OpacityMapVolumeStyle;
    wxGetApp().style_nodes[ name ].reset( s );
    StyleNameChoice->Append( name );
    StyleNameChoice->SetStringSelection( name );
    selected_style = name;
    setStyleOptions( s );
    wxGetApp().saveRenderStyle( name, s );
    setStyleModified( false );
    wxGetApp().main_dialog->addRenderStyle( name );
  }
  
}

void MedX3DDemoStyleDialog::OnDeleteStyleButton( wxCommandEvent& event ) {
  wxString name_choice = StyleNameChoice->GetStringSelection();
  StyleNameChoice->Delete( StyleNameChoice->GetSelection() );
  
  wxGetApp().main_dialog->deleteRenderStyle( name_choice );
  wxGetApp().style_nodes.erase( name_choice );

  StyleNameChoice->SetSelection( 0 );
  selected_style = StyleNameChoice->GetStringSelection();
  setStyleOptions( wxGetApp().style_nodes[ StyleNameChoice->GetStringSelection() ].get() ); 
  setStyleModified( false );
  wxGetApp().deleteRenderStyle( name_choice );

  
  
}

void MedX3DDemoStyleDialog::OnSaveStyleButton( wxCommandEvent& event ) {
  wxString name_choice = StyleNameChoice->GetStringSelection();
  if( name_choice != wxT( "" ) ) {
    X3DVolumeRenderStyleNode *s = wxGetApp().style_nodes[ name_choice ].get();
    wxGetApp().saveRenderStyle( name_choice, s );
    setStyleModified( false );
  }
}

void MedX3DDemoStyleDialog::OnStyleTypeChoice( wxCommandEvent& event ) {
  wxString name_choice = StyleNameChoice->GetStringSelection();
  wxString type_choice = StyleTypeChoice->GetStringSelection();

  // the style does not exist
  if( wxGetApp().style_nodes.find( name_choice ) == wxGetApp().style_nodes.end() ) return;

  X3DVolumeRenderStyleNode *s = wxGetApp().style_nodes[name_choice].get();
  
  // Add new style options
  if( type_choice == wxT( "OpacityMapVolumeStyle" ) && 
      !dynamic_cast< OpacityMapVolumeStyle * > ( s ) ) {
    OpacityMapVolumeStyle *opacity_style = new OpacityMapVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( opacity_style );
    setStyleOptions( opacity_style );
    setStyleModified( true );
  } else if( type_choice == wxT( "BoundaryEnhancementVolumeStyle" ) && 
             !dynamic_cast< BoundaryEnhancementVolumeStyle * >( s ) ) {
    BoundaryEnhancementVolumeStyle *style = new BoundaryEnhancementVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( style );
    setStyleOptions( style );
    setStyleModified( true );
  } else if( type_choice == wxT( "CartoonVolumeStyle" ) && 
             !dynamic_cast< CartoonVolumeStyle * >( s ) ) {
    CartoonVolumeStyle *cartoon_style = new CartoonVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( cartoon_style );
    setStyleOptions( cartoon_style );
    setStyleModified( true );
  } else if( type_choice == wxT( "ComposedVolumeStyle" ) && 
             !dynamic_cast< ComposedVolumeStyle * >( s ) ) {
    ComposedVolumeStyle *composed_style = new ComposedVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( composed_style );
    setStyleOptions( composed_style );
    setStyleModified( true );
  } else if( type_choice == wxT( "EdgeEnhancementVolumeStyle" ) && 
             !dynamic_cast< EdgeEnhancementVolumeStyle * >( s ) ) {
    EdgeEnhancementVolumeStyle *style = new EdgeEnhancementVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( style );
    setStyleOptions( style );
    setStyleModified( true );
  } else if( type_choice == wxT( "ProjectionVolumeStyle" ) && 
             !dynamic_cast< ProjectionVolumeStyle * >( s ) ) {
    ProjectionVolumeStyle *style = new ProjectionVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( style );
    setStyleOptions( style );
    setStyleModified( true );
  }else if( type_choice == wxT( "BlendedVolumeStyle" ) && 
             !dynamic_cast< BlendedVolumeStyle * >( s ) ) {
    BlendedVolumeStyle *style = new BlendedVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( style );
    setStyleOptions( style );
    setStyleModified( true );
  } else if( type_choice == wxT( "ShadedVolumeStyle" ) && 
             !dynamic_cast< ShadedVolumeStyle * >( s ) ) {
    ShadedVolumeStyle *style = new ShadedVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( style );
    setStyleOptions( style );
    setStyleModified( true );
  } else if( type_choice == wxT( "SilhouetteEnhancementVolumeStyle" ) && 
             !dynamic_cast< SilhouetteEnhancementVolumeStyle * >( s ) ) {
    SilhouetteEnhancementVolumeStyle *style = 
      new SilhouetteEnhancementVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( style );
    setStyleOptions( style );
    setStyleModified( true );
  } else if(  type_choice == wxT( "ToneMappedVolumeStyle" ) && 
              !dynamic_cast< ToneMappedVolumeStyle * >( s ) ) {
    ToneMappedVolumeStyle *tm_style = new ToneMappedVolumeStyle;
    wxGetApp().style_nodes[name_choice].reset( tm_style );
    setStyleOptions( tm_style );
    setStyleModified( true );
  }

  wxGetApp().main_dialog->updateSelectedRenderStyle();
}

void MedX3DDemoStyleDialog::setStyleOptions( X3DVolumeRenderStyleNode *s ) {
  wxPanel *new_options = NULL;

  // Add new style options
  if( OpacityMapVolumeStyle *opacity_style = 
        dynamic_cast< OpacityMapVolumeStyle * >( s ) ) {
    new_options =  
      new MedX3DDemoOpacityMapVolumeStyleOptions( this, opacity_style );
    StyleTypeChoice->SetStringSelection( wxT( "OpacityMapVolumeStyle" ) );
     
  } else if( BoundaryEnhancementVolumeStyle *style = 
        dynamic_cast< BoundaryEnhancementVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoBoundaryEnhancementVolumeStyleOptions( this, style );    
     StyleTypeChoice->SetStringSelection( wxT( "BoundaryEnhancementVolumeStyle"  ));  
  } else if( CartoonVolumeStyle *cartoon_style = 
        dynamic_cast< CartoonVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoCartoonVolumeStyleOptions( this, cartoon_style );    
     StyleTypeChoice->SetStringSelection( wxT( "CartoonVolumeStyle") );  
  } else if( ComposedVolumeStyle *composed_style = 
        dynamic_cast< ComposedVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoComposedVolumeStyleOptions( this, composed_style );    
     StyleTypeChoice->SetStringSelection( wxT( "ComposedVolumeStyle" ) );  
  } else if( EdgeEnhancementVolumeStyle *style = 
        dynamic_cast< EdgeEnhancementVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoEdgeEnhancementVolumeStyleOptions( this, style );    
     StyleTypeChoice->SetStringSelection( wxT( "EdgeEnhancementVolumeStyle" ) );  
  } else if( ProjectionVolumeStyle *style = 
        dynamic_cast< ProjectionVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoMIPVolumeStyleOptions( this, style );    
     StyleTypeChoice->SetStringSelection( wxT( "ProjectionVolumeStyle") );  
   } else if( BlendedVolumeStyle *style = 
        dynamic_cast< BlendedVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoBlendedVolumeStyleOptions( this, style );    
     StyleTypeChoice->SetStringSelection( wxT( "BlendedVolumeStyle") );  
  } else if( ShadedVolumeStyle *style = 
        dynamic_cast< ShadedVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoShadedVolumeStyleOptions( this, style );    
     StyleTypeChoice->SetStringSelection( wxT( "ShadedVolumeStyle" ) );  
  } else if( SilhouetteEnhancementVolumeStyle *style = 
        dynamic_cast< SilhouetteEnhancementVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoSilhouetteEnhancementVolumeStyleOptions( this, style );    
     StyleTypeChoice->SetStringSelection( wxT( "SilhouetteEnhancementVolumeStyle" ) );  
  } else if( ToneMappedVolumeStyle *tm_style = 
        dynamic_cast< ToneMappedVolumeStyle * >( s ) ) {
     new_options = 
       new MedX3DDemoToneMappedVolumeStyleOptions( this, tm_style );    
     StyleTypeChoice->SetStringSelection( wxT( "ToneMappedVolumeStyle"  ) );  
  }
 
  if( new_options ) {
    // Remove previous style options
    if( current_render_style_sizer  ) {
      StyleDialogMainSizer->Detach( current_render_style_sizer );
      delete current_render_style_sizer;
    }

    // Add the new style options to sizer
    StyleDialogMainSizer->Add( new_options, 0, wxEXPAND  );      

    current_render_style_sizer = new_options;
  }


 
  this->Layout();

}


bool MedX3DDemoStyleDialog::isStyleModified() {
  return SaveStyleButton->IsEnabled();
}

void MedX3DDemoStyleDialog::setStyleModified( bool modified ) {
  if( modified ) SaveStyleButton->Enable();
  else SaveStyleButton->Disable();
}

bool MedX3DDemoStyleDialog::doSaveStyleChangesDialog() {
  if( !isStyleModified() ) return true;

  wxMessageDialog *saveStyleDialog( new wxMessageDialog ( this,
                                                          wxT("You have unsaved style changes. Do you want to save?"), 
                                                          wxT( ""),
                                                          wxYES_NO ) );
 
  int result = saveStyleDialog->ShowModal();
  wxString name_choice = getSelectedStyle();

  // Open an volume data file
  if (  result== wxID_YES ) {
    // check that style exists
    if( wxGetApp().style_nodes.find( name_choice ) == wxGetApp().style_nodes.end() ) return true;
    X3DVolumeRenderStyleNode *s = wxGetApp().style_nodes[name_choice].get();
    wxGetApp().saveRenderStyle( name_choice, s );
  } else if( result == wxID_NO ) {
    wxGetApp().style_nodes[ name_choice ].reset( wxGetApp().loadRenderStyle( name_choice ) );
    wxGetApp().main_dialog->updateSelectedRenderStyle();
    return true;
  } else if( result == wxID_CANCEL ) {
    return false;
  }
  
  return true;
}

bool MedX3DDemoStyleDialog::setSelectedStyleName( const wxString &name ) {
  bool exists = StyleNameChoice->SetStringSelection( name );

  if( exists ) {
    X3DVolumeRenderStyleNode *s = wxGetApp().style_nodes[name].get();
    setStyleOptions( s );
    setStyleModified( false );
    selected_style = name;
  }
  return exists;
}
