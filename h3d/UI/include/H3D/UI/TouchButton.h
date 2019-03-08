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
/// \file TouchButton.h
/// \brief Header file for TouchButton.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __TOUCHBUTTON_H__
#define __TOUCHBUTTON_H__

// UI includes
#include <H3D/UI/H3DButtonNode.h>

// H3D includes
#include <H3D/X3DTextureNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/SFColor.h>

namespace H3D {

  /// \ingroup UINodes
  /// \class TouchButton
  /// \brief The TouchButton is a X3DButtonNode that changes state
  /// when it is touch by the haptics device.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/buttons.x3d">buttons.x3d</a>
  ///     ( <a href="x3d/buttons.x3d.html">Source</a> )
  ///   - <a href="../../x3d/radiobuttons.x3d">radiobuttons.x3d</a>
  ///     ( <a href="x3d/radiobuttons.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile TouchButton.dot
  class UI_API TouchButton : 
    public H3DButtonNode {
  public:

    class UI_API FinalDiffuseColor: public TypedField< SFColor,
                             Types< SFColor, SFBool > > {
      virtual void update();
    };

    /// The SFTextureNode field is dependent on the displayList field
    /// of the containing X3DTextureNode node.
    typedef DependentSFNode< X3DTextureNode, 
                             FieldRef< H3DDisplayListObject,
                                       H3DDisplayListObject::DisplayList,
                                       &H3DDisplayListObject::displayList >, 
                             true >
    SFTextureNode;

    /// Constructor.
    TouchButton( Inst< SFNode           > _metadata       = 0,
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
                 Inst< SFBool           > _isPressed      = 0,
                 Inst< ButtonState      > _state          = 0,
                 Inst< SFString         > _buttonMode     = 0,
                 Inst< SFColor          > _color          = 0,
                 Inst< SFColor          > _textColor      = 0,
                 Inst< SFTextureNode    > _texture        = 0,
                 Inst< SFBool           > _isOver         = 0 );

    /// Override resize to not have non-uniform scaling since
    /// it can give problems for some haptic renderers.
    virtual void resize( const Vec3f &new_size );

    virtual void initialize();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// The color of the touch button.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.6 0.6 0.6 \n
    /// 
    /// \dotfile TouchButton_color.dot
    auto_ptr< SFColor > color;

    /// The color of the text on the button.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 0 0 \n
    /// 
    /// \dotfile TouchButton_textColor.dot
    auto_ptr< SFColor > textColor;

    /// Contains a X3DTextureNode to put on the button.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile TouchButton_texture.dot
    auto_ptr< SFTextureNode >  texture;

    /// True if the pointing device is pointing towards the button
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile TouchButton_isOver.dot
    auto_ptr < SFBool > isOver;

  protected:
    /// Field that composes the diffuse color for the touch button
    /// depending on color and state fields.
    /// C++ only field.
    auto_ptr< FinalDiffuseColor > finalDiffuseColor;

    // Perform || operator for the values in all fields routed to it.
    class UI_API OrField: public TypedField< SFBool, void,
      AnyNumber< Any< MFBool, SFBool > > > {

      virtual void update(){ 
        value = false;

        for( FieldVector::iterator i = routes_in.begin();
             i != routes_in.end(); ++i ) {
          if( MFBool *mfbool = dynamic_cast< MFBool * >( *i ) ) {
            const vector<bool> &v = mfbool->getValue();
            for( vector<bool>::const_iterator j = v.begin();
                 j != v.end();
                 ++j ) {
              value = value || *j;
            }
          } else if( SFBool *sfbool = dynamic_cast< SFBool * >( *i ) ) {
            bool b = sfbool->getValue();
            value = value || b;
          }
          // if a value has been true we do not have to check more
          if( value ) return;
        }
      }
    };

    // Used internally to set some properties. Moved here from source file
    // for automatic cleanup.
    auto_ptr< OrField > orField;
  };
}

#endif
