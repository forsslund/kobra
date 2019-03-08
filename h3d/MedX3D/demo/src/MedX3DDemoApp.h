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
/// \file MedX3DDemoApp.h
/// \brief Header file for MedX3DDemoApp.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MEDX3DDEMOAPP_H
#define __MEDX3DDEMOAPP_H

#include <wx/wx.h>
#include <H3D/Scene.h>
#include "WxWidgetsWindow.h"
#include <H3DUtil/Image.h>
#include <H3D/X3D.h>
#include <H3D/H3DImageLoaderNode.h>
#include <H3D/MedX3D/X3DVolumeNode.h>
#include "WxConsoleDialog.h"
#include "MedX3DDemoMainFrame.h"
#include "MedX3DDemoMainDialog.h"
#include "MedX3DDemoStyleDialog.h"


class MedX3DDemoApp : public wxApp
{
public:
  MedX3DDemoApp();
  virtual ~MedX3DDemoApp();
  virtual bool OnInit();

  static H3D::AutoRef< H3D::Scene > h3d_scene;
  static H3D::AutoRef< H3D::WxWidgetsWindow > h3d_window;
  static std::auto_ptr< H3D::X3D::DEFNodes > main_scene_def_nodes;

  /// Load a volume data file and visualize it.
  bool loadVolumeData( const std::string &filename,
                       H3D::H3DImageLoaderNode *loader = NULL );
 
  inline H3D::X3DVolumeNode *getVolumeDataNode() {
    return volume_node.get();
  }

  void setVolumeDataNode( H3D::X3DVolumeNode *n );

  H3D::X3DVolumeRenderStyleNode *loadRenderStyle( const wxString &name,
                                                  const wxString &root = wxT( "/Styles" )) ;
  bool saveRenderStyle( const wxString &name, 
                        H3D::X3DVolumeRenderStyleNode *s,
                        const wxString &root = wxT( "/Styles" ) );
  bool deleteRenderStyle( const wxString &name );


  void disableAllWidgetsInSizer( wxSizer *s );
  void enableAllWidgetsInSizer( wxSizer *s );

  MedX3DDemoMainFrame *main_frame;
  WxConsoleDialog *console_dialog;
  MedX3DDemoMainDialog *main_dialog;
  MedX3DDemoStyleDialog *style_dialog;

  typedef std::map< wxString, 
                    H3DUtil::AutoRef< H3D::X3DVolumeRenderStyleNode > > 
  StyleNodeMap;
  
  StyleNodeMap style_nodes;

protected:
  void initializeFirstUse();
  void initializeAvailableRenderStyleList();
  H3DUtil::AutoRef< H3D::X3DVolumeNode > volume_node;
};

DECLARE_APP(MedX3DDemoApp)

#endif 
