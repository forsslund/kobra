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
/// \file MedX3DDemoApp.cpp
/// \brief CPP file for MedX3DDemoApp.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include <wx/config.h>

#include "MedX3DDemoApp.h"
#include "MedX3DDemoMainFrame.h"
#include "WxWidgetsWindow.h"
#include <H3D/Group.h>
#include <H3D/X3D.h>
#include <H3D/H3DHapticsDevice.h>
#include <H3D/SFFloat.h>
#include <H3D/Image3DTexture.h>
#include <H3D/ImageTexture.h>
#include <H3D/Material.h>

#include <H3D/X3DTextureNode.h>
#include <H3DUtil/ResourceResolver.h>

#include <H3D/MedX3D/VolumeData.h>
#include <H3D/MedX3D/OpacityMapVolumeStyle.h>
#include <H3D/MedX3D/BoundaryEnhancementVolumeStyle.h>
#include <H3D/MedX3D/CartoonVolumeStyle.h>
#include <H3D/MedX3D/ComposedVolumeStyle.h>
#include <H3D/MedX3D/EdgeEnhancementVolumeStyle.h>
#include <H3D/MedX3D/ProjectionVolumeStyle.h>
#include <H3D/MedX3D/BlendedVolumeStyle.h>
#include <H3D/MedX3D/ShadedVolumeStyle.h>
#include <H3D/MedX3D/SilhouetteEnhancementVolumeStyle.h>
#include <H3D/MedX3D/ToneMappedVolumeStyle.h>
#include <H3D/MedX3D/WindowFunctionTexture.h>

using namespace H3D;

string default_tex_prop = "<TextureProperties boundaryModeS=\"CLAMP_TO_EDGE\" \
                                              boundaryModeT=\"CLAMP_TO_EDGE\" \
                                              boundaryModeR=\"CLAMP_TO_EDGE\" \
                                             minificationFilter=\"AVG_PIXEL\" \
                                         magnificationFilter=\"AVG_PIXEL\" />";
// <NavigationInfo headlight=\"false \"/> 
// <DirectionalLight direction=\"-1 0 0\" /> 
string main_skeleton =  " \
<Collision enabled=\"FALSE\" > \
  <Viewpoint DEF=\"VP\" position=\"0 0 0.6\" /> \
  \
  <Background DEF=\"BG\" skyColor=\"0 0 0\" /> \
  <GlobalSettings > \
    <GraphicsCachingOptions useCaching=\"false\" /> \
  </GlobalSettings > \
  <FitToBoxTransform DEF=\"TRANSFORM\" boxCenter=\"0 0 0\" boxSize=\"0.15 0.15 0.15\"\
                     uniformScalingOnly=\"true\" >\
    <!-- The volume --> \
    <Group DEF=\"VOLUME_GROUP\" /> \
  </FitToBoxTransform> \
  \
 \
</Collision>";

/*******************Required Class***********************/

H3D_API_EXCEPTION( QuitAPIException );

class QuitAPIField: public AutoUpdate< SFString > {
  virtual void update() {
    string s = static_cast< SFString * >(routes_in[0])->getValue();
    if( s[0] == 27 ) { 
      throw QuitAPIException();
    }
  }
};

H3D::AutoRef< Scene > MedX3DDemoApp::h3d_scene;
H3D::AutoRef< WxWidgetsWindow > MedX3DDemoApp::h3d_window;
auto_ptr< X3D::DEFNodes > MedX3DDemoApp::main_scene_def_nodes;

IMPLEMENT_APP(MedX3DDemoApp)

MedX3DDemoApp::MedX3DDemoApp()
{
  H3D::X3DTextureNode::load_images_in_separate_thread = false;
}

MedX3DDemoApp::~MedX3DDemoApp()
{
}

bool MedX3DDemoApp::OnInit()
{
  SetVendorName(_T("SenseGraphics AB"));
  SetAppName(_T("MedX3DDemo"));

  string urn_config_file = "index.urn";
  char *buffer = getenv( "H3D_URN_CONFIG_FILE" );
  if( buffer ) urn_config_file = buffer;
  else if( ( buffer = getenv( "H3D_ROOT" ) ) ) {
    urn_config_file = buffer;
    urn_config_file += "/index.urn";
  }
  ResourceResolver::setURNResolver( 
    new URNResolver( urn_config_file ) );

  initializeFirstUse();
  initializeAvailableRenderStyleList();

  main_frame = new MedX3DDemoMainFrame( (wxWindow*)NULL );
  h3d_window.reset( new H3D::WxWidgetsWindow( main_frame ) );

  console_dialog = new WxConsoleDialog(main_frame, wxID_ANY, wxT("Console"), 
                                       wxDefaultPosition, wxDefaultSize,
                                       wxDEFAULT_DIALOG_STYLE);

  style_dialog = new MedX3DDemoStyleDialog( main_frame );

  try {
    // init H3D stuff
    int width, height;
    main_frame->GetClientSize(&width, &height);
    h3d_window->width->setValue(width);
    h3d_window->height->setValue(height);
    
    h3d_scene.reset( new H3D::Scene );
    h3d_scene->window->push_back( h3d_window.get() );
    
    main_scene_def_nodes.reset( new H3D::X3D::DEFNodes );
    h3d_scene->sceneRoot->setValue( H3D::X3D::createX3DFromString( main_skeleton, 
                                                                main_scene_def_nodes.get() ) );
  
    main_dialog = new MedX3DDemoMainDialog( main_frame );
    //main_dialog->Show();
    
    /*    unsigned char *data = new unsigned char[256*3];
    color_table.reset( new H3DUtil::PixelImage( 256,1,1,24,
                                                H3DUtil::PixelImage::RGB, 
                                                H3DUtil::PixelImage::UNSIGNED, 
                                                data ) );
    */
    main_frame->Show();
    SetTopWindow( main_frame );
        
    // set initial value of iso value.
    /*Node *mc = MedX3DDemoApp::main_scene_def_nodes->getNode( "MC" );
    H3D::SFFloat *iso_value = static_cast< H3D::SFFloat * >(mc->getField( "isovalue" ) );
    float v = atof( main_dialog->IsoValueText->GetValue().mb_str() );
    iso_value->setValue( v );
    */
     setVolumeDataNode( new VolumeData );
  } 
  catch (const Exception::H3DException &e) {
    Console(4) << e << endl;
  }
  return true;
}

bool MedX3DDemoApp::loadVolumeData( const string &filename,
                                    H3DImageLoaderNode *loader ) {
  try {
    Console(3) << "Loading " << filename << endl;
    X3DVolumeNode *volume = getVolumeDataNode();

    Image3DTexture *tex = new Image3DTexture;

    tex->url->push_back( filename ); 
    tex->textureProperties->setValue( 
       X3D::createX3DNodeFromString( default_tex_prop ) );

    if( loader ) tex->imageLoader->push_back( loader );

    Image *image = tex->image->getValue();
    if( image ) {
      const Vec3f &pixel_size = image->pixelSize();
      volume->dimensions->setValue( Vec3f( pixel_size.x * image->width(),
                                           pixel_size.y * image->height(),
                                           pixel_size.z * image->depth() ) );
      main_dialog->DensityDataLoadedFileText->SetLabel( wxString(filename.c_str(),wxConvUTF8 ));
      main_dialog->setDensityDataVolumeInfo( image );
      main_dialog->showVolumeInfo( true );
      main_dialog->SaveAsNrrdButton->Enable();
    } else {
      main_dialog->DensityDataLoadedFileText->SetLabel( wxT("None") );
      main_dialog->showVolumeInfo( false );
      if( filename != "" )
        wxMessageBox( wxT("Unable to read file format"),
                      wxT("Error"), wxOK | wxICON_EXCLAMATION);
      Console(3) << "loadVolumeImageFile(): No image, no dimensions" << endl;
    }
    volume->voxels->setValue( tex );
  }
  catch (const Exception::H3DException &e) {
    wxMessageBox(wxString(e.message.c_str(),wxConvUTF8 ),
                 wxT("Error"), 
                 wxOK | wxICON_EXCLAMATION);
    return false;
  }
  return true;
}


void MedX3DDemoApp::initializeAvailableRenderStyleList() {
  wxConfigBase *config = wxConfigBase::Get();
  wxString p = wxT("/Styles");
  config->SetPath( p );
  
  wxString style;
  long index = 0;
  bool g = config->GetFirstGroup( style, index );
  vector< wxString > blended_styles;
  while( g ) {
    style_nodes[ style ].reset( loadRenderStyle( style) ); 
    if( dynamic_cast< BlendedVolumeStyle *>(style_nodes[style].get()) ) {
       blended_styles.push_back( style );
  }
  config->SetPath( p );
    g = config->GetNextGroup( style, index );
  }

  // reload all BlendedVolumeStyles since they require all other styles to 
  // be loaded in order for the renderStyle field to be set up correctly
  for( unsigned int i = 0; i < blended_styles.size(); ++i )  {
    style_nodes[ blended_styles[i] ].reset( loadRenderStyle( blended_styles[i]) ); 
  }
}


X3DVolumeRenderStyleNode *MedX3DDemoApp::loadRenderStyle( const wxString &name,
                                                          const wxString &root ) { 
  wxConfigBase *config = wxConfigBase::Get();
  wxString old_path = config->GetPath();
  wxString p = root + wxT("/") + name;

  if( !config->Exists( p ) ) return NULL;
  
  config->SetPath( p );

  wxString style_type;
  config->Read( wxT( "styleType" ), &style_type );

  
  if( style_type == wxT("OpacityMapVolumeStyle" ) ) {
    OpacityMapVolumeStyle *s = new OpacityMapVolumeStyle;
    wxString type;
    config->Read( wxT( "type" ), &type );
    s->type->setValue( string( type.mb_str() ) );

    wxString tf_type;
    config->Read( wxT( "transferFunctionType" ), &tf_type );

    if( tf_type == wxT( "file" ) ) {
      Image3DTexture *t = new Image3DTexture;
      wxString url;
      config->Read( wxT( "fileUrl" ), &url );
      t->url->push_back( string( url.mb_str() ) );
      s->transferFunction->setValue( t );
    } else if( tf_type == wxT( "window" ) ) {
      WindowFunctionTexture *t = new WindowFunctionTexture;
      long width = 256, center = 128;
      config->Read( wxT( "windowWidth"  ), &width );
      config->Read( wxT( "windowCenter" ), &center );
      t->windowWidth->setValue( width );
      t->windowCenter->setValue( center );
      s->transferFunction->setValue(t );
    }

    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "BoundaryEnhancementVolumeStyle" ) ) {
    BoundaryEnhancementVolumeStyle *s = new BoundaryEnhancementVolumeStyle;
  
    double retained, boundary, factor;
    
    config->Read( wxT( "retainedOpacity" ),   &retained );
    config->Read( wxT( "boundaryOpacity" ), &boundary );
    config->Read( wxT( "opacityFactor" ),  &factor );

    s->retainedOpacity->setValue( retained );
    s->boundaryOpacity->setValue( boundary );
    s->opacityFactor->setValue( factor );
    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "CartoonVolumeStyle" ) ) {
    CartoonVolumeStyle *s = new CartoonVolumeStyle;
  
    double pr, pg, pb, pa;
    double ored, og, ob, oa;
    
    long color_steps = 4;
    
    config->Read( wxT( "parallelColor_Red" ),   &pr );
    config->Read( wxT( "parallelColor_Green" ), &pg );
    config->Read( wxT( "parallelColor_Blue" ),  &pb );
    config->Read( wxT( "parallelColor_Alpha" ),  &pa );
    
    config->Read( wxT( "orthogonalColor_Red" ),   &ored );
    config->Read( wxT( "orthogonalColor_Green" ), &og );
    config->Read( wxT( "orthogonalColor_Blue" ),  &ob );
    config->Read( wxT( "orthogonalColor_Alpha" ),  &oa );
    
    config->Read( wxT( "colorSteps" ), &color_steps );
    
    s->parallelColor->setValue( RGBA( pr, pg, pb, pa ) );
    s->orthogonalColor->setValue( RGBA( ored, og, ob, oa ) );
    s->colorSteps->setValue( color_steps );
    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "ComposedVolumeStyle" ) ) {
    ComposedVolumeStyle *s = new ComposedVolumeStyle;
    unsigned int counter = 0;
    
    wxString c;
    c << counter;
    
    while( config->HasGroup( c ) ) {
      s->renderStyle->push_back( loadRenderStyle( c ,
                                                  config->GetPath() ) );
      ++counter;
      c.Clear();
      c << counter;
    }
    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "EdgeEnhancementVolumeStyle" ) ) {
    EdgeEnhancementVolumeStyle *s = new EdgeEnhancementVolumeStyle;
  
    double threshold;
    double cr, cg, cb;
    
    config->Read( wxT( "gradientThreshold" ), &threshold );
    config->Read( wxT( "edgeColor_Red" ), &cr );
    config->Read( wxT( "edgeColor_Green" ), &cg );
    config->Read( wxT( "edgeColor_Blue" ), &cb );

    s->gradientThreshold->setValue( threshold );
    s->edgeColor->setValue( RGB( cr, cg, cb ) );
    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "ProjectionVolumeStyle" ) ) {
    ProjectionVolumeStyle *s = new ProjectionVolumeStyle;
  
    double threshold;
    config->Read( wxT( "intensityThreshold" ), &threshold );
  wxString type;
    config->Read( wxT( "type" ), &type );

    s->intensityThreshold->setValue( threshold );
    s->type->setValue( string( type.mb_str() ) );
    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "BlendedVolumeStyle" ) ) {
    BlendedVolumeStyle *s = new BlendedVolumeStyle;
  
    double weight_constant_1, weight_constant_2;
    config->Read( wxT( "weightConstant1" ), &weight_constant_1 );
    config->Read( wxT( "weightConstant2" ), &weight_constant_2 );
    wxString weight_function_1, weight_function_2;
    config->Read( wxT( "weightFunction1" ), &weight_function_1 );
    config->Read( wxT( "weightFunction2" ), &weight_function_2 );
    wxString table_1_url, table_2_url, voxels_url;
    config->Read( wxT( "table1Url" ), &table_1_url );
    config->Read( wxT( "table2Url" ), &table_2_url );
    config->Read( wxT( "voxelsUrl" ), &voxels_url );
    wxString render_style;
    config->Read( wxT( "renderStyle" ), &render_style );

    StyleNodeMap::iterator pos = wxGetApp().style_nodes.find( render_style );
    if( pos != wxGetApp().style_nodes.end() ) {
      s->renderStyle->setValue( (*pos).second.get() ); 
    }
  
    s->weightFunction1->setValue( string( weight_function_1.mb_str() ) );
    s->weightFunction2->setValue( string( weight_function_2.mb_str() ) );
    s->weightConstant1->setValue( weight_constant_1 );
    s->weightConstant2->setValue( weight_constant_2 );

    if( table_1_url != wxT("None" ) ) {
      ImageTexture *t = new ImageTexture;
      t->url->push_back( string( table_1_url.mb_str() ) );
      s->weightTransferFunction1->setValue( t );
    }

    if( table_2_url != wxT("None" ) ) {
      ImageTexture *t = new ImageTexture;
      t->url->push_back( string( table_2_url.mb_str() ) );
      s->weightTransferFunction2->setValue( t );
    }

    if( voxels_url != wxT("None" ) ) {
      Image3DTexture *t = new Image3DTexture;
      t->url->push_back( string( voxels_url.mb_str() ) );
      s->voxels->setValue( t );
    }

    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "ShadedVolumeStyle" ) ) {
    ShadedVolumeStyle *s = new ShadedVolumeStyle;
  
    double transparency, ambient, shininess;
    double dr, dg, db;
    double sr, sg, sb;
    double er, eg, eb;
    bool lighting, shadows, use_material;
    
    config->Read( wxT( "diffuseColor_Red" ), &dr );
    config->Read( wxT( "diffuseColor_Green" ), &dg );
    config->Read( wxT( "diffuseColor_Blue" ), &db );

    config->Read( wxT( "specularColor_Red" ), &sr );
    config->Read( wxT( "specularColor_Green" ), &sg );
    config->Read( wxT( "specularColor_Blue" ), &sb );

    config->Read( wxT( "emissiveColor_Red" ), &er );
    config->Read( wxT( "emissiveColor_Green" ), &eg );
    config->Read( wxT( "emissiveColor_Blue" ), &eb );

    config->Read( wxT( "transparency" ), &transparency );
    config->Read( wxT( "ambientIntensity" ), &ambient );
    config->Read( wxT( "shininess" ), &shininess );
    config->Read( wxT( "lighting" ), &lighting );
    config->Read( wxT( "shadows" ), &shadows );
  config->Read( wxT( "useMaterial" ), &use_material );
    
  if( use_material ) {
      Material *m = new Material;
      s->material->setValue( m );
      m->diffuseColor->setValue( RGB( dr, dg, db ) );
      m->specularColor->setValue( RGB( sr, sg, sb ) );
      m->emissiveColor->setValue( RGB( er, eg, eb ) );
      m->shininess->setValue( shininess );
      m->ambientIntensity->setValue( ambient );
  }

    s->lighting->setValue( lighting );
    s->shadows->setValue( shadows );
         
    config->SetPath( old_path );
    return s;
  }  else if( style_type == wxT( "SilhouetteEnhancementVolumeStyle" ) ) {
    SilhouetteEnhancementVolumeStyle *s = new SilhouetteEnhancementVolumeStyle;
  
    double retained, boundary, sharpness;
    
    config->Read( wxT( "retainedOpacity" ),   &retained );
    config->Read( wxT( "boundaryOpacity" ), &boundary );
    config->Read( wxT( "sharpness" ),  &sharpness );

    s->silhouetteRetainedOpacity->setValue( retained );
    s->silhouetteBoundaryOpacity->setValue( boundary );
    s->silhouetteSharpness->setValue( sharpness );
     
    config->SetPath( old_path );
    return s;
  } else if( style_type == wxT( "ToneMappedVolumeStyle" ) ) {
    ToneMappedVolumeStyle *s = new ToneMappedVolumeStyle;
  
    double cr, cg, cb, ca;
    double wr, wg, wb, wa;
    
    config->Read( wxT( "coolColor_Red" ),   &cr );
    config->Read( wxT( "coolColor_Green" ), &cg );
    config->Read( wxT( "coolColor_Blue" ),  &cb );
    config->Read( wxT( "coolColor_Alpha" ), &ca );
    
    config->Read( wxT( "warmColor_Red" ),   &wr );
    config->Read( wxT( "warmColor_Green" ), &wg );
    config->Read( wxT( "warmColor_Blue" ),  &wb );
    config->Read( wxT( "warmColor_Alpha" ), &wa );
    
    s->coolColor->setValue( RGBA( cr, cg, cb, ca ) );
    s->warmColor->setValue( RGBA( wr, wg, wb, wa ) );
    
    config->SetPath( old_path );
    return s;
  }
  
  return NULL;  
}

bool MedX3DDemoApp::deleteRenderStyle( const wxString &name ) {
  wxConfigBase *config = wxConfigBase::Get();
  wxString p = wxT("/Styles");
  config->SetPath( p );
  return config->DeleteGroup( name );
}

bool MedX3DDemoApp::saveRenderStyle( const wxString &name, 
                                     X3DVolumeRenderStyleNode *s,
                                     const wxString &root ) {
  wxConfigBase *config = wxConfigBase::Get();
  wxString old_path = config->GetPath();
  wxString p = root + wxT("/") + name;
  config->SetPath( p );
  
  if( OpacityMapVolumeStyle *opacity_style = 
    dynamic_cast< OpacityMapVolumeStyle * >( s ) ) {
    const string &type = opacity_style->type->getValue();
    config->Write( wxT( "styleType" ), wxT("OpacityMapVolumeStyle" ) );
    config->Write( wxT( "type" ),
                   wxString( type.c_str(), wxConvUTF8 ) );

    X3DTextureNode *t = opacity_style->transferFunction->getValue();
    
    if( ImageTexture *t2d = dynamic_cast< ImageTexture * >( t ) ) {
      const vector< string > &urls = t2d->url->getValue();
      if( urls.size() > 0 ) {
        config->Write( wxT( "transferFunctionType" ), wxT( "file" ) );
        config->Write( wxT( "fileUrl" ),
                       wxString( urls[0].c_str(), wxConvUTF8 ) );
      }
    } else if ( Image3DTexture *t3d = dynamic_cast< Image3DTexture * >( t ) ) {
      const vector< string > &urls = t3d->url->getValue();
      config->Write( wxT( "transferFunctionType" ), wxT( "file" ) );
      config->Write( wxT( "fileUrl" ),
                     wxString( urls[0].c_str(), wxConvUTF8 ) );
    } else if( WindowFunctionTexture *wt = 
               dynamic_cast< WindowFunctionTexture * >( t ) ) {
      int width  =  wt->windowWidth->getValue();
      int center =  wt->windowCenter->getValue();
      config->Write( wxT( "transferFunctionType" ), wxT( "window" ) );
      config->Write( wxT( "windowWidth" ), width );
      config->Write( wxT( "windowCenter" ), center );
    }
  } else if( BoundaryEnhancementVolumeStyle *style = 
    dynamic_cast< BoundaryEnhancementVolumeStyle * >( s ) ) {
    H3DFloat retained = style->retainedOpacity->getValue();
    H3DFloat boundary = style->boundaryOpacity->getValue();
    H3DFloat factor = style->opacityFactor->getValue();

    config->Write( wxT( "styleType" ), wxT("BoundaryEnhancementVolumeStyle" ) );
    config->Write( wxT( "retainedOpacity" ), retained );
    config->Write( wxT( "boundaryOpacity" ), boundary );
    config->Write( wxT( "opacityFactor" ),  factor );
  } else if( ComposedVolumeStyle *style = 
    dynamic_cast< ComposedVolumeStyle * >( s ) ) {
    config->Write( wxT( "styleType" ), wxT("ComposedVolumeStyle" ) );
    unsigned int counter = 0;

    for( ComposedVolumeStyle::MFComposableVolumeRenderStyleNode::const_iterator i = style->renderStyle->begin();
         i != style->renderStyle->end(); ++i, ++counter ) {
      wxString n;
      n << counter;
      saveRenderStyle( n, static_cast< X3DVolumeRenderStyleNode * >(*i), config->GetPath() );
    }
  } else if( CartoonVolumeStyle *cartoon_style = 
    dynamic_cast< CartoonVolumeStyle * >( s ) ) {
    const RGBA &par_color = cartoon_style->parallelColor->getValue();
    const RGBA &orth_color = cartoon_style->orthogonalColor->getValue();
    config->Write( wxT( "styleType" ), wxT("CartoonVolumeStyle" ) );
    
    config->Write( wxT( "parallelColor_Red" ),   par_color.r );
    config->Write( wxT( "parallelColor_Green" ), par_color.g );
    config->Write( wxT( "parallelColor_Blue" ),  par_color.b );
    
    config->Write( wxT( "orthogonalColor_Red" ),   orth_color.r );
    config->Write( wxT( "orthogonalColor_Green" ), orth_color.g );
    config->Write( wxT( "orthogonalColor_Blue" ),  orth_color.b );
    
    config->Write( wxT( "colorSteps" ), cartoon_style->colorSteps->getValue() );
    
  } else if( EdgeEnhancementVolumeStyle *style = 
    dynamic_cast< EdgeEnhancementVolumeStyle * >( s ) ) {
    const RGB &edge_color = style->edgeColor->getValue();
    H3DFloat threshold = style->gradientThreshold->getValue();

    config->Write( wxT( "styleType" ), wxT("EdgeEnhancementVolumeStyle" ) );
    config->Write( wxT( "gradientThreshold" ), threshold );
    config->Write( wxT( "edgeColor_Red" ), edge_color.r );
    config->Write( wxT( "edgeColor_Green" ), edge_color.g );
    config->Write( wxT( "edgeColor_Blue" ), edge_color.b );
  } else if( ProjectionVolumeStyle *style = 
    dynamic_cast< ProjectionVolumeStyle * >( s ) ) {
    H3DFloat threshold = style->intensityThreshold->getValue();
  string type = style->type->getValue();
    config->Write( wxT( "styleType" ), wxT("ProjectionVolumeStyle" ) );
    config->Write( wxT( "intensityThreshold" ), threshold );
  config->Write( wxT( "type" ), wxString( type.c_str(), wxConvUTF8 ) );
  }  else if( BlendedVolumeStyle *style = 
    dynamic_cast< BlendedVolumeStyle * >( s ) ) {
    H3DFloat weight_constant_1 = style->weightConstant1->getValue();
    H3DFloat weight_constant_2 = style->weightConstant2->getValue();
    wxString weight_function_1( style->weightFunction1->getValue().c_str(),
                                wxConvUTF8 );
    wxString weight_function_2( style->weightFunction2->getValue().c_str(),
                                wxConvUTF8 );

    X3DTexture2DNode *t1 = style->weightTransferFunction1->getValue();
    X3DTexture2DNode *t2 = style->weightTransferFunction2->getValue();
    X3DTexture3DNode *voxels = style->voxels->getValue();
  X3DComposableVolumeRenderStyleNode *blend_style = style->renderStyle->getValue();

  wxString blend_style_name = wxT( "None" );
  for( MedX3DDemoApp::StyleNodeMap::iterator i = wxGetApp().style_nodes.begin();
         i != wxGetApp().style_nodes.end(); ++i ) {
      if( blend_style == (*i).second.get() ) {
        blend_style_name = (*i).first;
      }
    }

    wxString weight_table_1_url = wxT("None");
    if( ImageTexture *t = dynamic_cast< ImageTexture * >( t1 ) ) {
      const vector< string > &urls = t->url->getValue();
      weight_table_1_url = wxString( urls[0].c_str(), wxConvUTF8 );
    }

    wxString weight_table_2_url = wxT("None");
    if( ImageTexture *t = dynamic_cast< ImageTexture * >( t2 ) ) {
      const vector< string > &urls = t->url->getValue();
      weight_table_2_url = wxString( urls[0].c_str(), wxConvUTF8 );
    }

    wxString voxels_url = wxT("None");
    if( Image3DTexture *t = dynamic_cast< Image3DTexture * >( voxels ) ) {
      const vector< string > &urls = t->url->getValue();
      voxels_url = wxString( urls[0].c_str(), wxConvUTF8 );
    }

    config->Write( wxT( "styleType" ), wxT("BlendedVolumeStyle" ) );
    config->Write( wxT( "weightConstant1" ), weight_constant_1 );
    config->Write( wxT( "weightConstant2" ), weight_constant_2 );
    config->Write( wxT( "weightFunction1" ), weight_function_1 );
    config->Write( wxT( "weightFunction2" ), weight_function_2 );
    config->Write( wxT( "table1Url" ), weight_table_1_url );
    config->Write( wxT( "table2Url" ), weight_table_2_url );
    config->Write( wxT( "voxelsUrl" ), voxels_url );
  config->Write( wxT( "renderStyle" ), blend_style_name );
  } else if( ShadedVolumeStyle *style = 
    dynamic_cast< ShadedVolumeStyle * >( s ) ) {
    AutoRef< Material > m( static_cast< Material * >( style->material->getValue() ) );
    if(!m.get()) m.reset( new Material );
    const RGB &diffuse_color = m->diffuseColor->getValue();
    const RGB &emissive_color = m->emissiveColor->getValue();
    const RGB &specular_color = m->specularColor->getValue();
    H3DFloat transparency = m->transparency->getValue();
    H3DFloat shininess = m->shininess->getValue();
    H3DFloat ambient = m->ambientIntensity->getValue();
    bool lighting = style->lighting->getValue();
    bool shadows = style->shadows->getValue();
  bool use_material = style->material->getValue() != NULL;

    config->Write( wxT( "styleType" ), wxT("ShadedVolumeStyle" ) );
    
    config->Write( wxT( "diffuseColor_Red" ), diffuse_color.r );
    config->Write( wxT( "diffuseColor_Green" ), diffuse_color.g );
    config->Write( wxT( "diffuseColor_Blue" ), diffuse_color.b );
    
    config->Write( wxT( "emissiveColor_Red" ), emissive_color.r );
    config->Write( wxT( "emissiveColor_Green" ), emissive_color.g );
    config->Write( wxT( "emissiveColor_Blue" ), emissive_color.b );

    config->Write( wxT( "specularColor_Red" ), specular_color.r );
    config->Write( wxT( "specularColor_Green" ), specular_color.g );
    config->Write( wxT( "specularColor_Blue" ), specular_color.b );

    config->Write( wxT( "transparency" ), transparency );
    config->Write( wxT( "shininess" ), shininess );
    config->Write( wxT( "ambientIntensity" ), ambient );
    config->Write( wxT( "lighting" ), lighting );
    config->Write( wxT( "shadows" ), shadows );
  config->Write( wxT( "useMaterial" ), use_material );

  } else if( SilhouetteEnhancementVolumeStyle *style = 
    dynamic_cast< SilhouetteEnhancementVolumeStyle * >( s ) ) {
    H3DFloat retained = style->silhouetteRetainedOpacity->getValue();
    H3DFloat boundary = style->silhouetteBoundaryOpacity->getValue();
    H3DFloat sharpness = style->silhouetteSharpness->getValue();

    config->Write( wxT( "styleType" ), wxT("SilhouetteEnhancementVolumeStyle" ) );
    config->Write( wxT( "retainedOpacity" ), retained );
    config->Write( wxT( "boundaryOpacity" ), boundary );
    config->Write( wxT( "sharpness" ),  sharpness );
  } else if( ToneMappedVolumeStyle *style = 
    dynamic_cast< ToneMappedVolumeStyle * >( s ) ) {
    const RGBA &cool_color = style->coolColor->getValue();
    const RGBA &warm_color = style->warmColor->getValue();
    config->Write( wxT( "styleType" ), wxT("ToneMappedVolumeStyle" ) );
    
    config->Write( wxT( "coolColor_Red" ), cool_color.r );
    config->Write( wxT( "coolColor_Green" ), cool_color.g );
    config->Write( wxT( "coolColor_Blue" ), cool_color.b );
    
    config->Write( wxT( "warmColor_Red" ), warm_color.r );
    config->Write( wxT( "warmColor_Green" ), warm_color.g );
    config->Write( wxT( "warmColor_Blue" ), warm_color.b );
  }
    
  config->SetPath( old_path);
  return true;  
}

void MedX3DDemoApp::initializeFirstUse() {

  wxConfigBase *config = wxConfigBase::Get();
  wxString p = wxT("/");
  config->SetPath( p );

  bool init_done;
  config->Read( wxT( "initDone" ), &init_done, false );
  if( !init_done ) {  
    AutoRef< OpacityMapVolumeStyle > opacity_style( new OpacityMapVolumeStyle );
    AutoRef< BoundaryEnhancementVolumeStyle > boundary_enhancement_style( new BoundaryEnhancementVolumeStyle );
    AutoRef< CartoonVolumeStyle > cartoon_style( new CartoonVolumeStyle );
    AutoRef< EdgeEnhancementVolumeStyle > edge_enhancement_style( new EdgeEnhancementVolumeStyle );
    AutoRef< ProjectionVolumeStyle > mip_style( new ProjectionVolumeStyle );
    AutoRef< BlendedVolumeStyle > blended_style( new BlendedVolumeStyle );
    AutoRef< ShadedVolumeStyle > shaded_style( new ShadedVolumeStyle );
    AutoRef< SilhouetteEnhancementVolumeStyle > silhouette_enhancement_style( new SilhouetteEnhancementVolumeStyle );
    AutoRef< ToneMappedVolumeStyle > tone_mapped_style( new ToneMappedVolumeStyle );
    
    saveRenderStyle( wxT( "Opacity map" ), opacity_style.get() );
    saveRenderStyle( wxT( "Boundary enhancement" ), 
         boundary_enhancement_style.get() );
    saveRenderStyle( wxT( "Cartoon" ), cartoon_style.get() );
    saveRenderStyle( wxT( "Edge enhancement" ), edge_enhancement_style.get() );
    saveRenderStyle( wxT( "Projection" ), mip_style.get() );
    saveRenderStyle( wxT( "Blended" ), blended_style.get() );
    saveRenderStyle( wxT( "Shaded" ), shaded_style.get() );
    saveRenderStyle( wxT( "Silhouette enhancement" ), 
         silhouette_enhancement_style.get() );
    saveRenderStyle( wxT( "Tone mapped" ), tone_mapped_style.get() );

    config->SetPath( wxT("/") );
    config->Write( wxT( "initDone" ), true );
  }
}

void MedX3DDemoApp::enableAllWidgetsInSizer( wxSizer *s ) {
  const wxSizerItemList& items = s->GetChildren();   
  size_t nr_widgets = items.GetCount();
  for( size_t i = 0; i < nr_widgets; ++i ) {
    wxSizerItem *item = items.Item( i )->GetData();
    if( item->IsSizer() ) {
      enableAllWidgetsInSizer( item->GetSizer() );
    } else if( item->IsWindow() ) {
      item->GetWindow()->Enable();
    }
  }
}

void MedX3DDemoApp::disableAllWidgetsInSizer( wxSizer *s ) {
  const wxSizerItemList& items = s->GetChildren();   
  size_t nr_widgets = items.GetCount();
  for( size_t i = 0; i < nr_widgets; ++i ) {
    wxSizerItem *item = items.Item(i)->GetData();
    if( item->IsSizer() ) {
      disableAllWidgetsInSizer( item->GetSizer() );
    } else if( item->IsWindow() ) {
      item->GetWindow()->Disable();
    }
  }
}

void MedX3DDemoApp::setVolumeDataNode( H3D::X3DVolumeNode *n ) {
  Group *g;
  main_scene_def_nodes->getNode( "VOLUME_GROUP", g );
  g->children->clear();
  g->children->push_back( n );
  volume_node.reset( n );
}
