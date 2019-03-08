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
/// \file WindowFunctionTexture.h
/// \brief Header file for WindowFunctionTexture.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __WINDOWFUNCTIONTEXTURE_H__
#define __WINDOWFUNCTIONTEXTURE_H__

#include <H3D/MedX3D/MedX3D.h>

#include <H3D/X3DTexture2DNode.h>
#include <H3D/SFInt32.h>

namespace H3D {
  /// \ingroup MedX3DNodes 
  /// \class WindowFunctionTexture
  /// \brief The WindowFunctionTexture node is basically a filter texture.
  /// It can be used to filter the values of the style in order to only use
  /// values in the range specified by the fields windowCenter and windowWidth.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/WindowFunctionTexture.x3d">OpacityMapVolumeStyle.x3d</a>
  ///     ( <a href="x3d/OpacityMapVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API WindowFunctionTexture : public X3DTexture2DNode {
  public:
    
    /// SFImage is overridden to update the value from the url 
    /// and imageLoader fields of the ImageTexture. Each url is tried
    /// with all ImageLoader and the first one that is successful is 
    /// the one that is used.
    /// routes_in[0] is the windowCenter field
    /// routes_in[1] is the windowWidth field.
    class MEDX3D_API SFImage: public TypedField< X3DTexture2DNode::SFImage,
    Types< SFInt32, SFInt32 > > {
      /// Creates an image to be used as color table.
      virtual void update();
    };

    /// Constructor.
    WindowFunctionTexture( 
                 Inst< DisplayList > _displayList  = 0,
                 Inst< SFNode      > _metadata     = 0,
                 Inst< SFImage     > _image        = 0,
                 Inst< SFTextureProperties > _textureProperties = 0,
                 Inst< SFInt32     > _windowCenter = 0,
                 Inst< SFInt32     > _windowWidth  = 0 );

    /// The center of the filter.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b> Default value:</b> 128 \n
    auto_ptr< SFInt32 > windowCenter;

    /// The width of the filter.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b> Default value:</b> 256 \n
    auto_ptr< SFInt32 > windowWidth;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  };
}

#endif
