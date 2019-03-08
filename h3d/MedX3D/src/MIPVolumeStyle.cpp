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
/// \file MIPVolumeStyle.cpp
/// \brief CPP file for MIPVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/MIPVolumeStyle.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>

using namespace H3D;
  
H3DNodeDatabase 
MIPVolumeStyle::database( "MIPVolumeStyle", 
                          &(newInstance<MIPVolumeStyle>),
                          typeid( MIPVolumeStyle ),
                          &X3DVolumeRenderStyleNode::database );

namespace MIPVolumeStyleInternals {
  FIELDDB_ELEMENT( MIPVolumeStyle, intensityThreshold, INPUT_OUTPUT )
}

MIPVolumeStyle::MIPVolumeStyle( Inst< DisplayList >_displayList, 
                                Inst< SFBool > _enabled,
                                Inst< SFNode > _metadata,
                                Inst< SFFloat > _intensityThreshold ) :
  ProjectionVolumeStyle( _displayList, _enabled, _metadata, 
                         _intensityThreshold, 0 ) {
  
  type_name = "MIPVolumeStyle";
  database.initFields( this );
  
  Console(4) << "MIPVolumeStyle has been deprecated. Use "
             << "ProjectionVolumeStyle instead." << endl;  
}




