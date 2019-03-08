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
/// \file MedX3DDemoLoadRawImageDialog.cpp
/// \brief CPP file for MedX3DDemoLoadRawImageDialog.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoLoadRawImageDialog.h"

using namespace H3D;

MedX3DDemoLoadRawImageDialog::MedX3DDemoLoadRawImageDialog( wxWindow* parent )
:
LoadRawImageDialog( parent )
{

}

void MedX3DDemoLoadRawImageDialog::OnOKButtonPressed( wxCommandEvent& event )
{
  EndModal(  wxID_OK );

}

void MedX3DDemoLoadRawImageDialog::OnCancelButtonPressed( wxCommandEvent& event )
{
  EndModal( wxID_CANCEL);
}

H3D::RawImageLoader *MedX3DDemoLoadRawImageDialog::getNewFileReader() {
 RawImageLoader *loader = new RawImageLoader;
 loader->width->setValue( WidthSpin->GetValue() );
 loader->height->setValue( HeightSpin->GetValue());
 loader->depth->setValue( DepthSpin->GetValue() );
 loader->bitsPerPixel->setValue( BitsPerPixelSpin->GetValue() );

 wxString pixel_type = PixelTypeChoice->GetStringSelection();
 loader->pixelType->setValue( string( pixel_type.mb_str() ) );

 wxString pixel_component_type = PixelComponentTypeChoice->GetStringSelection();
 loader->pixelComponentType->setValue( string( pixel_component_type.mb_str() ) );

 float x_size = atof (PixelSizeXText->GetValue().mb_str() );
 float y_size = atof (PixelSizeYText->GetValue().mb_str() );
 float z_size = atof (PixelSizeZText->GetValue().mb_str() );
      
 loader->pixelSize->setValue( Vec3f( x_size, y_size, z_size ) );
 return loader;
}
