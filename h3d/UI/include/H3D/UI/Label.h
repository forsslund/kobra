//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file Label.h
/// \brief Header file for Label.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __LABEL_H__
#define __LABEL_H__

// UI includes
#include <H3D/UI/H3DLabeledWidget.h>

// H3D includes
#include <H3D/X3DTextureNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/Box.h>
#include <H3D/SFColor.h>

namespace H3D {

  /// \ingroup UINodes
  /// \class Label
  /// \brief The Label class is just a widget with text on it.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/Label.x3d">Label.x3d</a>
  ///     ( <a href="x3d/Label.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile Label.dot
  class UI_API Label : 
    public H3DLabeledWidget {
  public:
    /// The SFTextureNode field is dependent on the displayList field
    /// of the containing X3DTextureNode node.
    typedef DependentSFNode< X3DTextureNode, 
                             FieldRef< H3DDisplayListObject,
                                       H3DDisplayListObject::DisplayList,
                                       &H3DDisplayListObject::displayList >, 
                             true >
    SFTextureNode;

    class UI_API LabelBackground: public TypedField< SFGeometryNode, SFBool > {
      virtual void update();
    };

    /// Constructor.
    Label( Inst< SFNode           > _metadata       = 0,
           Inst< SFBound          > _bound          = 0,
           Inst< DisplayList      > _displayList    = 0,
           Inst< SFString         > _tag            = 0,
           Inst< SFBool           > _enabled        = 0,
           Inst< SFVec3f          > _desiredSize    = 0,
           Inst< SFVec3f          > _actualSize     = 0,
           Inst< SFAppearanceNode > _appearance     = 0,
           Inst< SFAppearanceNode > _textAppearance = 0,
           Inst< SFLayoutInfoNode > _layoutInfo     = 0,
           Inst< MFString         > _text           = 0,
           Inst< SFFontStyleNode  > _fontStyle      = 0,
           Inst< SFColor          > _color          = 0,
           Inst< SFColor          > _textColor      = 0,
           Inst< SFTextureNode    > _texture        = 0,
           Inst< SFBool           > _background     = 0 );

    /// Override resize to not have non-uniform scaling since
    /// it can give problems for some haptic renderers.
    virtual void resize( const Vec3f &new_size );

    virtual void initialize();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// The color of the label.
    /// Depricated: please use appearance node instead
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.6 0.6 0.6 \n
    /// 
    /// \dotfile Label_color.dot
    auto_ptr< SFColor > color;

    /// The color of the text on the label.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 0 0 \n
    /// 
    /// \dotfile Label_textColor.dot
    auto_ptr< SFColor > textColor;

    /// Contains a X3DTextureNode to put on the label.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile Label_texture.dot
    auto_ptr< SFTextureNode >  texture;

    /// Determines if there should be a background(as in buttons) to
    /// the text of the label.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile Label_background.dot
    auto_ptr< SFBool >  background;

  protected:
    /// Internal field containing a geometry node.
    auto_ptr< LabelBackground > labelBackground;

    /// The geometry for this label. Needed in order to set size correctly.
    /// One could argue that scaling the transform in which this geometry
    /// reside could be used so that all labels could share the same geometry
    /// but this does not work since non uniform scaling might be incorrectly
    /// handled by some haptic renderers.
    AutoRef< Box > label_geometry;
  };
}

#endif
