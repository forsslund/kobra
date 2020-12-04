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
/// \file MedX3DDemoComposedVolumeStyleOptions.cpp
/// \brief CPP file for MedX3DDemoComposedVolumeStyleOptions.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoComposedVolumeStyleOptions.h"
#include "MedX3DDemoComposedVolumeStyleOptionsPart.h"
#include "MedX3DDemoApp.h"

using namespace H3D;

MedX3DDemoComposedVolumeStyleOptions::MedX3DDemoComposedVolumeStyleOptions( wxWindow* parent,
                                                                            ComposedVolumeStyle *style )
:
  ComposedVolumeStyleOptions( parent ),
  composed_style( style )
{
  for( ComposedVolumeStyle::MFComposableVolumeRenderStyleNode::const_iterator i = composed_style->renderStyle->begin();
       i != composed_style->renderStyle->end(); ++i ) {
    wxPanel *panel = 
      new MedX3DDemoComposedVolumeStyleOptionsPart( this, 
                                                    static_cast< X3DComposableVolumeRenderStyleNode *>(*i) );
    StylePartsSizer->Add( panel, 0, wxEXPAND  );  
    style_options.push_back( panel );
  }

  // if there is no style at all, add an empty one with just the plus/minus buttons.
  if( composed_style->renderStyle->empty() ) {
    wxPanel *panel = new MedX3DDemoComposedVolumeStyleOptionsPart( this, NULL );
    StylePartsSizer->Add( panel,0, wxEXPAND  );  
    style_options.push_back( panel );
  }
  
  this->Layout();
}

void MedX3DDemoComposedVolumeStyleOptions::removeStyleOption( wxPanel *p ) {
  int pos = this->getStyleOptionPosition( p );
  
  // if we have only the empty panel when no styles, don't do anything
  if( pos == -1 ||
      (!style_options.empty() && composed_style->renderStyle->empty() )) {
    return;
  }

  style_options.erase( style_options.begin() + pos );
  composed_style->renderStyle->erase( pos );
  StylePartsSizer->Detach( p );
  delete p;
  
  // if there is no style at all, add an empty one with just the plus/minus buttons.
  if( composed_style->renderStyle->empty() ) {
    wxPanel *panel = new MedX3DDemoComposedVolumeStyleOptionsPart( this, NULL );
    StylePartsSizer->Add( panel,0, wxEXPAND  );  
    style_options.push_back( panel );
  }
   
  StylePartsSizer->Layout();
  this->Layout();   
  wxGetApp().style_dialog->StyleDialogMainSizer->Layout();
  wxGetApp().style_dialog->Layout(); 
  
  wxGetApp().style_dialog->setStyleModified( true ); 
} 

// Add the style to the composable node adding a gui as well. The 
// position is the position to put it in. If -1 it is added to the end.
void MedX3DDemoComposedVolumeStyleOptions::insertStyle( H3D::X3DComposableVolumeRenderStyleNode *s, 
                                                        int pos ) {
  // if we have only the empty panel when no styles, remove it
  if( !style_options.empty() && composed_style->renderStyle->empty() ) {
    style_options.clear();
    StylePartsSizer->Clear( true );
  }

  // at to end of list
  if( pos == -1 || pos >= static_cast<int>(composed_style->renderStyle->size()) ) {

    wxPanel *panel = new MedX3DDemoComposedVolumeStyleOptionsPart( this, s );
    style_options.push_back( panel );
    composed_style->renderStyle->push_back( s );
    StylePartsSizer->Add( panel, 0, wxEXPAND );
    panel->Layout();
  } else {
    wxPanel *panel = new MedX3DDemoComposedVolumeStyleOptionsPart( this, s );
    style_options.insert( style_options.begin() + pos , panel );
    composed_style->renderStyle->insert( pos, s );
    StylePartsSizer->Insert( pos, panel, 0, wxEXPAND );
    panel->Layout();
  }
  StylePartsSizer->Layout();
  this->Layout();  
  wxGetApp().style_dialog->StyleDialogMainSizer->Layout();
  wxGetApp().style_dialog->Layout();

  wxGetApp().style_dialog->setStyleModified( true ); 
}
  
// Get the position of the panel given in the style_options vector. 
// -1 if it does not exist.
int MedX3DDemoComposedVolumeStyleOptions::getStyleOptionPosition( wxPanel *p ) {
  int counter = 0;

  for( vector< wxPanel * >::iterator i = style_options.begin();
       i != style_options.end(); ++i, ++counter ) {
    if( (*i) == p ) return counter;
  }

  return -1;
}
