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
/// \file MedX3DDemoMainDialog.cpp
/// \brief CPP file for MedX3DDemoMainDialog.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "MedX3DDemoMainDialog.h"
#include "MedX3DDemoShadedVolumeStyleOptions.h"
#include "MedX3DDemoOpacityMapVolumeStyleOptions.h"
#include "MedX3DDemoLoadRawImageDialog.h"
#include "MedX3DDemoApp.h"
#include <H3D/MedX3D/VolumeData.h>
#include <H3D/Background.h>
#include <H3D/X3D.h>
#include <H3D/NrrdImageLoader.h>

#include <fstream>

// default ranges
#define DEFAULT_32_UNSIGNED_MIN 2147482649  // 2^31 + 1 + -1000 
#define DEFAULT_32_UNSIGNED_MAX 2147488649  // 2^31 + 1 + 5000
#define DEFAULT_32_SIGNED_MIN -1000
#define DEFAULT_32_SIGNED_MAX 5000
#define DEFAULT_16_UNSIGNED_MIN 31769       // 2^15 + 1 + -1000
#define DEFAULT_16_UNSIGNED_MAX 37769       // 2^15 + 1 + 5000
#define DEFAULT_16_SIGNED_MIN -1000
#define DEFAULT_16_SIGNED_MAX 5000
#define DEFAULT_8_UNSIGNED_MIN -128
#define DEFAULT_8_UNSIGNED_MAX 127
#define DEFAULT_8_SIGNED_MIN 0
#define DEFAULT_8_SIGNED_MAX 255



const std::string x3d_file_prefix = 
  "<?xml version=\"1.0\" encoding=\"utf-8\"?> \n"
  "<X3D profile=\'MedX3D\' version=\'1.0\'> \n"
  "  <head> \n"
  "    <meta name=\'title\' content=\'MedX3D example\'/> \n"
  "    <meta name=\'description\' content=\'MedX3D example. Generated with MedX3DDemo program from SenseGraphics AB, http://www.sensegraphics.com\'/> \n"
  "    <meta name=\'author\' content=\'SenseGraphics AB, 2006-2019\'/> \n"
  "    <meta name=\'generator\' content=\'MedX3DDemo, http://www.h3d.org\'/> \n"
  "  </head> \n"
  "  <Scene> \n";

const std::string x3d_file_postfix = 
  "  </Scene> \n"
  "</X3D> \n";

template< class A >
void findMinMax( void *orig_data, 
                 unsigned int size,
                 A &min_value,
                 A &max_value ) {
  A *d = (A*) orig_data;
  A min_v, max_v;
  if( size == 0 ) return;

  min_v = max_v = d[0];
  for( unsigned int i = 1; i < size; ++i ) {
    if( d[i] < min_v ) min_v = d[i];
  if( d[i] > max_v ) max_v = d[i];
  }
  min_value = min_v;
  max_value = max_v;
}

using namespace H3D;

MedX3DDemoMainDialog::MedX3DDemoMainDialog( wxWindow* parent )
:
  MainDialog( parent ),
  current_render_style_sizer( NULL ),
  iso_surface_styles( 10, wxT("None" ) ),
  segment_styles( 1, make_pair( wxT("None" ), true ) )
{

  // initialize the menues of available render styles
  initializeAvailableRenderStyleList();

  // hide information about loaded volume data since we have none
  // loaded yet.
  showVolumeInfo( false );
  
  // VolumeData node is used by default, so disable the other ones
  wxGetApp().disableAllWidgetsInSizer( SegmentedDataNodeSizer );
  wxGetApp().disableAllWidgetsInSizer( IsoSurfacesNodeSizer );

  // make sure the the segment choice list is the right size
  updateSegmentRenderStyles();

}

void MedX3DDemoMainDialog::OnIdle( wxIdleEvent& /*event*/ )
{
}


void MedX3DDemoMainDialog::OnLoadButton( wxCommandEvent& /*event*/ )
{
  wxFileDialog *openFileDialog( new wxFileDialog ( this,
                                                   wxT("Open file"),
                                                   wxT(""),
                                                   wxT(""),
                                                   wxT("*.*"),
                                                   wxFD_OPEN,
                                                   wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    wxGetApp().loadVolumeData( string( openFileDialog->GetPath().mb_str() ) );
  }
}

void MedX3DDemoMainDialog::OnLoadRawButton( wxCommandEvent& /*event*/ )
{
  wxFileDialog *openFileDialog( new wxFileDialog ( this,
                                                   wxT("Open file"),
                                                   wxT(""),
                                                   wxT(""),
                                                   wxT("*.*"),
                                                   wxFD_OPEN,
                                                   wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    auto_ptr< MedX3DDemoLoadRawImageDialog > raw_image_dialog(
      new MedX3DDemoLoadRawImageDialog( this ) );
    if( raw_image_dialog->ShowModal() == wxID_OK ) { 
      wxGetApp().loadVolumeData( string( openFileDialog->GetPath().mb_str()),
                                 raw_image_dialog->getNewFileReader() );
    }
  }
}

void MedX3DDemoMainDialog::OnSaveAsNrrdButton( wxCommandEvent& /*event*/ )
{
  wxFileDialog *saveFileDialog( new wxFileDialog ( this,
                                                   wxT("Save file"),
                                                   wxT(""),
                                                   wxT(""),
                                                   wxT("*.nrrd"),
                                                   wxFD_SAVE,
                                                   wxDefaultPosition) );
  if (saveFileDialog->ShowModal() == wxID_OK) {
    string filename(saveFileDialog->GetPath().mb_str());
    X3DVolumeNode *vn = wxGetApp().getVolumeDataNode();
    X3DTexture3DNode *tex = vn->voxels->getValue();
    Image *image = NULL;
    if( tex ) image = tex->image->getValue();
#ifdef HAVE_TEEM
    if( !image || H3DUtil::saveImageAsNrrdFile( filename, image ) != 0 ) {
      wxMessageBox( wxT("Could not save Nrrd data file"), 
                    wxT("Error"), wxOK | wxICON_EXCLAMATION);
    }
#else
    wxMessageBox( wxT("Could not save Nrrd data file, nrrd file support are disabled in H3DUtil."), 
                    wxT("Error"), wxOK | wxICON_EXCLAMATION);
#endif
  }
}


void MedX3DDemoMainDialog::showVolumeInfo( bool v ) {
  // When the density data volume info is shown the save buttons also
  // needs to be set.
  if( v ) {
    DensityDataStaticBoxSizer->Show( DensityDataInfoSizer, true );
  } else {
    DensityDataStaticBoxSizer->Hide( DensityDataInfoSizer, true );
  }
  Layout();
}

void MedX3DDemoMainDialog::OnSaveAsX3DButton( wxCommandEvent& /*event*/ ) {
   auto_ptr< wxFileDialog > saveFileDialog( new wxFileDialog ( this,
                                                               wxT("Save file"),
                                                               wxT(""),
                                                               wxT(""),
                                                               wxT("*.x3d"),
                                                               wxFD_SAVE,
                                                               wxDefaultPosition) );
 
  // Choose an file to save to.
  if (saveFileDialog->ShowModal() == wxID_OK) {
    wxString path = saveFileDialog->GetPath();
    std::ofstream os( path.mb_str() );
    if( os.bad() ) {
      os.close();
      wxMessageBox( wxT("Could not open selected file: " ) + path );
    }
    
    Field *matrix_field = wxGetApp().main_scene_def_nodes->getNode( "TRANSFORM" )->getField( "matrix" );
    Matrix4f transform = static_cast< SFMatrix4f * >( matrix_field )->getValue();
    Vec3f scale = transform.getScalePart();

    os << x3d_file_prefix;
    os << "    <Group>" << endl;
    os << "       ";
    X3D::writeNodeAsX3D( os, wxGetApp().main_scene_def_nodes->getNode( "VP" ) );
    X3D::writeNodeAsX3D( os, wxGetApp().main_scene_def_nodes->getNode( "BG" ) );
    os << "       <Transform scale=\"" << scale << "\">" << endl;
    os << "          ";
    X3D::writeNodeAsX3D( os, wxGetApp().getVolumeDataNode() );
    os << "       </Transform>" << endl;
    os << "    </Group>" << endl;
    os << x3d_file_postfix;
    os.close();
  }
}

void MedX3DDemoMainDialog::showStyleOptions( wxSizer *sizer, bool v ) {
  // When the density data volume info is shown the save buttons also
  // needs to be set.
  if( v ) {
    VolumeDataSizer->Show( sizer, true );
  } else {
    VolumeDataSizer->Hide( sizer, true );
  }
  Layout();
}

void MedX3DDemoMainDialog::setDensityDataVolumeInfo( Image *i ) {
  stringstream s;
  s << i->width() << "x" << i->height() << "x" << i->depth();
  DensityDataDimensionsText->SetLabel(wxString(s.str().c_str(),wxConvUTF8));
  
  stringstream vs;
  Vec3f voxel_size = i->pixelSize();
  vs << voxel_size.x << "x" << voxel_size.y << "x" << voxel_size.z;
  DensityDataVoxelSizeText->SetLabel(wxString(vs.str().c_str(),wxConvUTF8));
  
  Image::PixelType pixel_type = i->pixelType();
  if( pixel_type == Image::LUMINANCE )
    DensityDataComponentsText->SetLabel( wxT("LUMINANCE" ) );
  else if( pixel_type == Image::LUMINANCE_ALPHA )
    DensityDataComponentsText->SetLabel( wxT("LUMINANCE_ALPHA" ) );
  else if( pixel_type == Image::RGB )
    DensityDataComponentsText->SetLabel( wxT("RGB" ) );
  else if( pixel_type == Image::RGBA )
    DensityDataComponentsText->SetLabel( wxT("RGBA" ) );
  else if( pixel_type == Image::BGR )
    DensityDataComponentsText->SetLabel( wxT("BGR" ) );
  else if( pixel_type == Image::BGRA )
    DensityDataComponentsText->SetLabel( wxT("BGRA" ) );
  else if( pixel_type == Image::VEC3 )
    DensityDataComponentsText->SetLabel( wxT("VEC3" ) );
  else
    DensityDataComponentsText->SetLabel( wxT("UNKNOWN" ) );
  
  Image::PixelComponentType pixel_component_type = i->pixelComponentType();
  if( pixel_component_type == Image::SIGNED ) 
    DensityDataTypeText->SetLabel( wxT("SIGNED" ) );
  else if( pixel_component_type == Image::UNSIGNED ) 
    DensityDataTypeText->SetLabel( wxT("UNSIGNED" ) );
  else if( pixel_component_type == Image::RATIONAL ) 
    DensityDataTypeText->SetLabel( wxT("RATIONAL" ) );
  else 
    DensityDataTypeText->SetLabel( wxT("UNKNOWN" ) );

  stringstream bpp;
  bpp << i->bitsPerPixel(); 
  DensityDataBitsPerVoxelText->SetLabel(wxString(bpp.str().c_str(),wxConvUTF8));

  unsigned int data_size = i->width() * i->height() * i->depth();

  // calculate max and min values
  stringstream max_stream, min_stream;

  Vec2d range;
  if( i->pixelType() == Image::LUMINANCE ) {
    Image::PixelComponentType type = i->pixelComponentType();
    if( type ==Image::UNSIGNED ) {
        switch( i->bitsPerPixel() ) {
          case 8: {
            unsigned char min_v, max_v;
            findMinMax< unsigned char >( i->getImageData(), data_size, min_v, max_v ); 
            min_stream << (unsigned int) min_v;
            max_stream << (unsigned int) max_v;
            range.x = DEFAULT_8_UNSIGNED_MIN;
            range.y = DEFAULT_8_UNSIGNED_MAX;
            break;
          }
          case 16: {
            unsigned short min_v, max_v;
            findMinMax< unsigned short >( i->getImageData(),  data_size, min_v, max_v  );
            min_stream << min_v;
            max_stream << max_v;
            range.x = DEFAULT_16_UNSIGNED_MIN;
            range.y = DEFAULT_16_UNSIGNED_MAX;
            break;
          }
          case 32: {
            unsigned int min_v, max_v;
            findMinMax< unsigned int >( i->getImageData(),  data_size, min_v, max_v  ); 
            min_stream << min_v;
            max_stream << max_v;
            range.x = DEFAULT_32_UNSIGNED_MIN;
            range.y = DEFAULT_32_UNSIGNED_MAX;
            break;   
          }
        default: 
          break;
        }
      } else if( type == Image::SIGNED ) {
        switch( i->bitsPerPixel() ) {
        case 8: {  
          char min_v, max_v;
          findMinMax< char >( i->getImageData(), data_size, min_v, max_v  ); 
          min_stream << (int)min_v;
          max_stream << (int)max_v;
          range.x = DEFAULT_8_SIGNED_MIN;
          range.y = DEFAULT_8_SIGNED_MAX;
          break;
        }
        case 16: {
          short min_v, max_v;
          findMinMax< short >( i->getImageData(), data_size, min_v, max_v  ); 
          min_stream << min_v;
          max_stream << max_v;
          range.x = DEFAULT_16_SIGNED_MIN;
          range.y = DEFAULT_16_SIGNED_MAX;
          break;
        }
        case 32: { 
          int min_v, max_v;
          findMinMax< int >( i->getImageData(), data_size, min_v, max_v  ); 
          min_stream << min_v;
          max_stream << max_v;
          range.x = DEFAULT_32_SIGNED_MIN;
          range.y = DEFAULT_32_SIGNED_MAX;
          break;
        }
        default: 
          break; 
        }
    } 
  } 

  DensityDataMinValueText->SetLabel(wxString(min_stream.str().c_str(),wxConvUTF8));
  DensityDataMaxValueText->SetLabel(wxString(max_stream.str().c_str(),wxConvUTF8));

  /*
  // set the range limit values
  stringstream min_limit, max_limit;
  if( density_volume.get() ) {
    Image *density_image = density_volume->getValue()->image->getValue();
    if( density_image ) {
      if( density_image->pixelComponentType() == Image::UNSIGNED ) {
        min_limit << "0 (" << DensityDataMinValueText->GetLabel() << ")";
        max_limit << H3DPow( 2.0, (int)density_image->bitsPerPixel()) - 1 << " (" 
                  << DensityDataMaxValueText->GetLabel()<< ")";
      } else {
       min_limit << "-" << H3DPow( 2.0, (int)(density_image->bitsPerPixel() - 1) ) << " (" << DensityDataMinValueText->GetLabel() << ")";
        max_limit << H3DPow( 2.0, (int)(density_image->bitsPerPixel() - 1 ) ) - 1 << " (" 
                  << DensityDataMaxValueText->GetLabel()<< ")";
      }
    }
  }
  */
}

void MedX3DDemoMainDialog::updateSegmentRenderStyles() {

  // resize the segment selector to current size
  int selected = SegmentChoice->GetSelection();
  SegmentChoice->Clear();
  for( unsigned int i = 0; i < segment_styles.size(); ++i ) {
    stringstream s;
    s << i;
    SegmentChoice->Append( wxString(s.str().c_str(),wxConvUTF8) );
  }

  if( selected < (int) SegmentChoice->GetCount() ) {
    SegmentChoice->SetSelection( selected );
  } else {
    SegmentChoice->SetSelection( 0 );
  }

 // get the main volume data node.
  SegmentedVolumeData *volume_data = 
    dynamic_cast< SegmentedVolumeData * >( wxGetApp().getVolumeDataNode() );
  if( !volume_data ) return;

  bool found_first = false;
  volume_data->renderStyle->resize( segment_styles.size() );
  volume_data->segmentEnabled->clear();

  // search backwards in the selected styles to find the first which is 
  // not enabled. From that point add all styles earlier in the list to the 
  // the render styles will have its enabled status set in segmentEnabled.
  // Segments are enabled by default. Add all render styles as well.
  for( int i = segment_styles.size() - 1; i >= 0; --i ) {
    const wxString &name = segment_styles[i].first;
    bool enabled = segment_styles[i].second;
    if( !found_first ) {
      if( !enabled ) {
        found_first = true;
        volume_data->segmentEnabled->resize( i + 1 );
        volume_data->segmentEnabled->setValue( i, enabled );
      }
      
    } else {
      volume_data->segmentEnabled->setValue( i, enabled );
    }
    
    volume_data->renderStyle->setValue( i, 
                                        wxGetApp().style_nodes[ name ].get() );    
  }

}

void MedX3DDemoMainDialog::updateIsoSurfaceRenderStyles() {
 // get the main volume data node.
 ISOSurfaceVolumeData *volume_data = 
    dynamic_cast< ISOSurfaceVolumeData * >( wxGetApp().getVolumeDataNode() );;
  if( !volume_data ) return;
  
  bool found_first = false;
  
  // search backwards in the selected styles to find the first which is not
  // "None". From that point add all styles earlier in the list to the 
  // the render styles used.
  for( int i = iso_surface_styles.size() - 1; i >= 0; --i ) {
    const wxString &name = iso_surface_styles[i]; 
    if( !found_first ) {
      if( name != wxT("None" ) ) {
        found_first = true;
        volume_data->renderStyle->resize( i + 1 );
      } else {
        continue;
      }
    } 
    
    volume_data->renderStyle->setValue( i, wxGetApp().style_nodes[ name ].get() );    
  }
}

void MedX3DDemoMainDialog::updateSelectedRenderStyle() {
 wxString selected_style = VolumeDataRenderStyleChoice->GetStringSelection();

 // get the main volume data node.
 VolumeData *volume_data = 
   dynamic_cast< VolumeData * >( wxGetApp().getVolumeDataNode() );

 if( volume_data ) {
   if( selected_style.IsSameAs( wxT( "None" ) ) ) {
     // nothing
     volume_data->renderStyle->setValue( NULL );
   } else { 
     volume_data->renderStyle->setValue( wxGetApp().style_nodes[ selected_style] );
   }
 }
}

void MedX3DDemoMainDialog::OnRenderStyleChoice( wxCommandEvent& /*event*/ ) {
  updateSelectedRenderStyle();
}


void MedX3DDemoMainDialog::OnDataFilterChoice( wxCommandEvent& event ) {
  // get the main volume data node.
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  volume_data->filterType->setValue( string(event.GetString().mb_str() ) );
}

void MedX3DDemoMainDialog::OnRendererChoice( wxCommandEvent& /*event*/ ) {
  // get the main volume data node.
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();

  bool use_slice = RendererChoice->GetSelection() == 1;
  volume_data->useSlicing->setValue( use_slice );
  if( use_slice ) {
    volume_data->rayStep->setValue( 1.f /getNrSlices() );
  } else {
    volume_data->rayStep->setValue( getRayStep() );
  }
}

void MedX3DDemoMainDialog::OnBackgroundColorChanged( wxColourPickerEvent& event ) {
  // get the main volume data node.
  Background *background;
  wxGetApp().main_scene_def_nodes->getNode( "BG", background );
  if( background ) {
    wxColour c = event.GetColour();
    background->skyColor->clear();
    background->skyColor->push_back( RGB( c.Red()   / 255.f,
                                          c.Green() / 255.f,
                                          c.Blue()  / 255.f ) );
  }
}

void MedX3DDemoMainDialog::OnRayStep( wxCommandEvent& event ) {
  setRayStep( atof( event.GetString().mb_str() ) );
}

void MedX3DDemoMainDialog::OnUseEmptySpaceSkipping( wxCommandEvent& event ) {
  // get the main volume data node.
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  if( volume_data ) {
    volume_data->useEmptySpaceSkipping->setValue( event.IsChecked() );
  }
}

void MedX3DDemoMainDialog::OnShowNonEmptySpace( wxCommandEvent& event ) {
  // get the main volume data node.
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  if( volume_data ) {
    volume_data->showNonEmptySpace->setValue( event.IsChecked() );
  }
}

void MedX3DDemoMainDialog::OnEmptySpaceResolution( wxCommandEvent& event ) {
  // get the main volume data node.
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  if( volume_data ) {
    volume_data->emptySpaceSkippingRes->setValue( atoi( event.GetString().mb_str() ) );
  }
}

void MedX3DDemoMainDialog::OnStopRaysAtGeom( wxCommandEvent& event ) {
  // get the main volume data node.
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  if( volume_data ) {
    volume_data->stopRaysAtGeometries->setValue( event.IsChecked() );
  }
}

void MedX3DDemoMainDialog::OnUseStochasticJittering( wxCommandEvent& event ) {
  // get the main volume data node.
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  if( volume_data ) {
    volume_data->useStochasticJittering->setValue( event.IsChecked() );
  }
}

void MedX3DDemoMainDialog::OnNrSlices( wxCommandEvent& event ) {
  setNrSlices( atoi( event.GetString().mb_str() ) );
}


void MedX3DDemoMainDialog::OnShowStyleEditor( wxCommandEvent& /*event*/ ) {
  wxString name = VolumeDataRenderStyleChoice->GetStringSelection();
  wxGetApp().style_dialog->setSelectedStyleName( name ); 
  wxGetApp().style_dialog->Show();
}

void MedX3DDemoMainDialog::initializeAvailableRenderStyleList() {
  VolumeDataRenderStyleChoice->Append( wxT("None") );
  SegmentRenderStyleChoice->Append( wxT("None") );
  IsoSurfaceRenderStyleChoice->Append( wxT("None") );
  
  for( MedX3DDemoApp::StyleNodeMap::iterator i = wxGetApp().style_nodes.begin();
       i != wxGetApp().style_nodes.end(); ++i ) {
    VolumeDataRenderStyleChoice->Append( (*i).first );
    SegmentRenderStyleChoice->Append( (*i).first );
    IsoSurfaceRenderStyleChoice->Append( (*i).first );
  }

  VolumeDataRenderStyleChoice->SetSelection( 0 );
  SegmentRenderStyleChoice->SetSelection( 0 );
  IsoSurfaceRenderStyleChoice->SetSelection( 0 );
}

void MedX3DDemoMainDialog::deleteRenderStyle( const wxString &name ) {
   // volume data render style choice
   int current_selection = VolumeDataRenderStyleChoice->GetSelection();
   int pos = VolumeDataRenderStyleChoice->FindString( name );
   if( pos != wxNOT_FOUND ) {
     VolumeDataRenderStyleChoice->Delete( pos );
     if( pos == current_selection ) {
       VolumeDataRenderStyleChoice->SetSelection( 0 );
       updateSelectedRenderStyle();
     } else if( current_selection < pos ) {
       VolumeDataRenderStyleChoice->SetSelection( pos );
     } else {
       VolumeDataRenderStyleChoice->SetSelection( pos -1 );
     }
   }
   
   // segmented volume data render style choice
   current_selection = SegmentRenderStyleChoice->GetSelection();
   pos = SegmentRenderStyleChoice->FindString( name );
   if( pos != wxNOT_FOUND ) {
     SegmentRenderStyleChoice->Delete( pos );
     if( pos == current_selection ) {
       SegmentRenderStyleChoice->SetSelection( 0 );
       updateSegmentRenderStyles();
     } else if( current_selection < pos ) {
       SegmentRenderStyleChoice->SetSelection( pos );
     } else {
       SegmentRenderStyleChoice->SetSelection( pos -1 );
     }
   }
   
   // iso surface data render style choice
   current_selection = IsoSurfaceRenderStyleChoice->GetSelection();
   pos = IsoSurfaceRenderStyleChoice->FindString( name );
   if( pos != wxNOT_FOUND ) {
     IsoSurfaceRenderStyleChoice->Delete( pos );
     if( pos == current_selection ) {
       IsoSurfaceRenderStyleChoice->SetSelection( 0 );
       updateIsoSurfaceRenderStyles();
     } else if( current_selection < pos ) {
       IsoSurfaceRenderStyleChoice->SetSelection( pos );
     } else {
       IsoSurfaceRenderStyleChoice->SetSelection( pos -1 );
     }
   }
}

void MedX3DDemoMainDialog::addRenderStyle( const wxString &name ) {
 VolumeDataRenderStyleChoice->Append( name );
 SegmentRenderStyleChoice->Append( name );
 IsoSurfaceRenderStyleChoice->Append( name );
}


float MedX3DDemoMainDialog::getRayStep() {
  wxString step = RayStepText->GetValue();
  return atof( step.mb_str() );
}

void MedX3DDemoMainDialog::setRayStep( float step ) {
  stringstream s;
  s << step;
  RayStepText->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) );
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  if( !volume_data->useSlicing->getValue() ) {
    volume_data->rayStep->setValue( step );
  }
}

unsigned int MedX3DDemoMainDialog::getNrSlices() {
  wxString nr = NrSlicesText->GetValue();
  return atoi( nr.mb_str() );
}

void MedX3DDemoMainDialog::setNrSlices( unsigned nr_slices ) {
  stringstream s;
  s << nr_slices;
  NrSlicesText->SetValue( wxString( s.str().c_str(),wxConvUTF8  ) );
  X3DVolumeNode *volume_data = wxGetApp().getVolumeDataNode();
  if( volume_data->useSlicing->getValue() ) {
    volume_data->rayStep->setValue( 1.0 / nr_slices );
  }
}


void MedX3DDemoMainDialog::OnVolumeNodeChoice( wxCommandEvent& event ) {
  wxString choice = event.GetString();
  if( choice == wxT( "Normal" ) ) {
    wxGetApp().enableAllWidgetsInSizer( VolumeDataNodeSizer );
    wxGetApp().disableAllWidgetsInSizer( SegmentedDataNodeSizer );
    wxGetApp().disableAllWidgetsInSizer( IsoSurfacesNodeSizer );
    X3DVolumeNode *v = newVolumeDataFromOptions();
    copyCommonParameters( v, wxGetApp().getVolumeDataNode() );
    wxGetApp().setVolumeDataNode( v );
  } else if( choice == wxT( "Segmented" ) ) {
    wxGetApp().disableAllWidgetsInSizer( VolumeDataNodeSizer );
    wxGetApp().enableAllWidgetsInSizer( SegmentedDataNodeSizer );
    wxGetApp().disableAllWidgetsInSizer( IsoSurfacesNodeSizer );
    X3DVolumeNode *v = newSegmentedVolumeDataFromOptions();
    copyCommonParameters( v, wxGetApp().getVolumeDataNode() );
    wxGetApp().setVolumeDataNode( v );
  } else if( choice == wxT( "Iso surfaces" ) ) {
    wxGetApp().disableAllWidgetsInSizer( VolumeDataNodeSizer );
    wxGetApp().disableAllWidgetsInSizer( SegmentedDataNodeSizer );
    wxGetApp().enableAllWidgetsInSizer( IsoSurfacesNodeSizer );
    X3DVolumeNode *v = newIsoSurfaceVolumeDataFromOptions();
    copyCommonParameters( v, wxGetApp().getVolumeDataNode() );
    wxGetApp().setVolumeDataNode( v );
  }
}


VolumeData *MedX3DDemoMainDialog::newVolumeDataFromOptions() {
  VolumeData *volume_data = new VolumeData;

  wxString selected_style = VolumeDataRenderStyleChoice->GetStringSelection();

  if( selected_style.IsSameAs( wxT( "None" ) ) ) {
    // nothing
    volume_data->renderStyle->setValue( NULL );
  } else { 
    volume_data->renderStyle->setValue( wxGetApp().style_nodes[ selected_style] );
  }

  return volume_data;
}

SegmentedVolumeData *MedX3DDemoMainDialog::newSegmentedVolumeDataFromOptions() {
  SegmentedVolumeData *data = new SegmentedVolumeData;
  return data;
}

ISOSurfaceVolumeData *MedX3DDemoMainDialog::newIsoSurfaceVolumeDataFromOptions() {
  ISOSurfaceVolumeData *data = new ISOSurfaceVolumeData;
  return data;
}

void MedX3DDemoMainDialog::copyCommonParameters( H3D::X3DVolumeNode *dest, 
                                                 H3D::X3DVolumeNode *src ) {
  dest->voxels->setValue( src->voxels->getValue() );
  dest->dimensions->setValue( src->dimensions->getValue() );
  dest->stopRaysAtGeometries->setValue( src->stopRaysAtGeometries->getValue() );
  dest->useSlicing->setValue( src->useSlicing->getValue() );
  dest->useEmptySpaceSkipping->setValue( src->useEmptySpaceSkipping->getValue() );
  dest->emptySpaceSkippingRes->setValue( src->emptySpaceSkippingRes->getValue() );
  dest->showNonEmptySpace->setValue( src->showNonEmptySpace->getValue() );
  dest->filterType->setValue( src->filterType->getValue() );
  dest->useStochasticJittering->setValue( src->useStochasticJittering->getValue() );
}

void MedX3DDemoMainDialog::OnLoadSegmentDataButton( wxCommandEvent& /*event*/ ) {
  auto_ptr< wxFileDialog > openFileDialog( new wxFileDialog ( this,
                                                              wxT("Open file"),
                                                              wxT(""),
                                                              wxT(""),
                                                              wxT("*.*"),
                                                              wxFD_OPEN,
                                                              wxDefaultPosition) );
 
  // Open an volume data file
  if (openFileDialog->ShowModal() == wxID_OK) {
    SegmentedVolumeData *volume_data = 
      dynamic_cast< SegmentedVolumeData * >( wxGetApp().getVolumeDataNode() );
    if( !volume_data ) return;


    AutoRef< Image3DTexture > tex( new Image3DTexture );

    tex->url->push_back( string( openFileDialog->GetPath().mb_str() ) ); 
    tex->textureProperties->setValue( 
       X3D::createX3DNodeFromString( "<TextureProperties boundaryModeS=\"CLAMP_TO_EDGE\" \
                                                         boundaryModeT=\"CLAMP_TO_EDGE\" \
                                                         boundaryModeR=\"CLAMP_TO_EDGE\" \
                                                         minificationFilter=\"NEAREST_PIXEL\" \
                                                         magnificationFilter=\"NEAREST_PIXEL\" \
                                                         textureCompression=\"DEFAULT\" />" ) );
    Image *i = tex->image->getValue();
    if( !i ) {
      wxMessageBox( wxT( "Could not read image" ) );
      return;
    }

    LoadedSegmentDataText->SetLabel( openFileDialog->GetPath() );

    // get the max value from the image
    unsigned long max_value = 0;
    
    unsigned int data_size = i->width() * i->height() * i->depth();

    if( i->pixelType() == Image::LUMINANCE ) {
      Image::PixelComponentType type = i->pixelComponentType();
      if( type ==Image::UNSIGNED ) {
        switch( i->bitsPerPixel() ) {
        case 8: {
          unsigned char min_v, max_v;
          findMinMax< unsigned char >( i->getImageData(), data_size, min_v, max_v ); 
          max_value =  max_v;
          break;
        }
        case 16: {
          unsigned short min_v, max_v;
          findMinMax< unsigned short >( i->getImageData(),  data_size, min_v, max_v  );
          max_value =  max_v;
          break;
        }
        case 32: {
          unsigned int min_v, max_v;
          findMinMax< unsigned int >( i->getImageData(),  data_size, min_v, max_v  ); 
          max_value =  max_v;
          break;   
        }
        default: 
          break;
        }
      } else if( type == Image::SIGNED ) {
        switch( i->bitsPerPixel() ) {
        case 8: {  
          char min_v, max_v;
          findMinMax< char >( i->getImageData(), data_size, min_v, max_v  ); 
          max_value =  H3DMax( max_v, (char) 0 );
          break;
        }
        case 16: {
          short min_v, max_v;
          findMinMax< short >( i->getImageData(), data_size, min_v, max_v  ); 
          max_value =  H3DMax( max_v, (short) 0 );
          break;
        }
        case 32: { 
          int min_v, max_v;
          findMinMax< int >( i->getImageData(), data_size, min_v, max_v  ); 
          max_value =  H3DMax( max_v, (int) 0 );
          break;
        }
        default: 
          break; 
        }
      } 
    }
    segment_styles.resize( max_value + 1, make_pair( wxT( "None" ), true ) );
    volume_data->segmentIdentifiers->setValue( tex );
    updateSegmentRenderStyles();
  }
}

void MedX3DDemoMainDialog::OnSegmentChoice( wxCommandEvent& event ) {
  wxString choice = event.GetString();
  int i = atoi( choice.mb_str() );
  SegmentRenderStyleChoice->SetStringSelection( segment_styles[i].first );
  SegmentEnabledCheck->SetValue( segment_styles[i].second );
}

void MedX3DDemoMainDialog::OnSegmentStyleChoice( wxCommandEvent& event ) {
  wxString choice = event.GetString();
  int i =  atoi( SegmentChoice->GetStringSelection().mb_str() );
  segment_styles[i].first = choice;
  updateSegmentRenderStyles();
}

void MedX3DDemoMainDialog::OnSegmentEnabledCheck( wxCommandEvent& event ) {
  int i = atoi( SegmentChoice->GetStringSelection().mb_str() );
  segment_styles[i].second = event.IsChecked();
  updateSegmentRenderStyles();
}

void MedX3DDemoMainDialog::OnIsoValueChange( wxCommandEvent& event ) {
  // get the main volume data node.
  ISOSurfaceVolumeData *volume_data = 
    dynamic_cast< ISOSurfaceVolumeData * >( wxGetApp().getVolumeDataNode() );;
  if( volume_data ) {
    wxString value = event.GetString();
    try {
      volume_data->surfaceValues->setValueFromString( string( value.mb_str() ) );
    } catch (X3D::Convert::X3DFieldConversionError & ) {
      IsoValuesText->SetValue( wxString( volume_data->surfaceValues->getValueAsString().c_str(),wxConvUTF8  ) );  
    }
  }
}

void MedX3DDemoMainDialog::OnIsoSurfaceChoice( wxCommandEvent& event ) {
  wxString choice = event.GetString();
  int i = atoi( choice.mb_str() );
  IsoSurfaceRenderStyleChoice->SetStringSelection( iso_surface_styles[i] );
}

void MedX3DDemoMainDialog::OnIsoSurfaceStyleChoice( wxCommandEvent& event ) {
  wxString choice = event.GetString();
  int i = IsoSurfaceChoice->GetSelection();
  iso_surface_styles[i] = choice;
  updateIsoSurfaceRenderStyles();
}

void MedX3DDemoMainDialog::OnContourStepSizeChange( wxCommandEvent& event ) {
  // get the main volume data node.
  ISOSurfaceVolumeData *volume_data = 
    dynamic_cast< ISOSurfaceVolumeData * >( wxGetApp().getVolumeDataNode() );;
  if( volume_data ) {
    wxString value = event.GetString();
    try {
      volume_data->contourStepSize->setValueFromString( string( value.mb_str() ) );
    } catch (X3D::Convert::X3DFieldConversionError & ) {
      ContourStepSizeText->SetValue( wxString( volume_data->contourStepSize->getValueAsString().c_str(),wxConvUTF8  ) );  
    }
  }
}

void MedX3DDemoMainDialog::OnSurfaceToleranceChange( wxCommandEvent& event ) {
// get the main volume data node.
  ISOSurfaceVolumeData *volume_data = 
    dynamic_cast< ISOSurfaceVolumeData * >( wxGetApp().getVolumeDataNode() );;
  if( volume_data ) {
    wxString value = event.GetString();
    try {
      volume_data->surfaceTolerance->setValueFromString( string( value.mb_str()  ));
    } catch (X3D::Convert::X3DFieldConversionError & ) {
      SurfaceToleranceText->SetValue( wxString( volume_data->surfaceTolerance->getValueAsString().c_str(),wxConvUTF8  ) );  
    }
  }
}

void MedX3DDemoMainDialog::OnCloseDialog( wxCloseEvent& /*event*/ ) {
  MedX3DDemoMainFrame * parent =
    static_cast< MedX3DDemoMainFrame * >( GetParent() );
  parent->VolumeStyleMenuItem->Check( false );
  Hide();
}

