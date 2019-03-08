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
/// \file SliderBar.h
/// \brief Header file for SliderBar.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SLIDERBAR_H__
#define __SLIDERBAR_H__

// UI includes
#include <H3D/UI/H3DLabeledWidget.h>

// H3D includes
#include <H3D/X3DTextureNode.h>
#include <H3D/SFVec2f.h>
#include <H3D/SFFloat.h>
#include <H3D/SFColor.h>

        #include<iostream>


namespace H3D {

  /// \ingroup UINodes
  /// \class SliderBar
  /// The SliderBar widget allows a user to control a floating point 
  /// value by a slider bar. The value is controlled by touching and moving
  /// along the surface of the SliderBar.
  /// 
  /// The value of the SliderBar is available through the 'value' field.
  /// The valueRange field specifies the range of the values available to
  /// the SliderBar, e.g. -1, 1 will mean that the value of the SliderBar
  /// will vary between -1 and 1.
  /// The 'stepLength' field determines in what increments the slider value
  /// will update, e.g. a stepLength of 0.1 and valueRange of 0, 0.5 means that
  /// the possible slider values are 0, 0.1, 0.2, 0.3, 0.4 and 0.5. The slider
  /// value will not be anything else.
  /// The 'markerColor' field is the color of the marker that shows where
  /// the slider value is at the moment.
  /// The 'horizontal' fíeld determines if the marker on the slider should
  /// move horizontally or vertically.
  /// The 'color' and 'textColor' field specifies the color of the SliderBar
  /// and the text on the SliderBar respectively.
  /// If 'enabled' field set to False, the value will never be changed.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/SliderBar.x3d">SliderBar.x3d</a>
  ///     ( <a href="x3d/SliderBar.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SliderBar.dot
  class UI_API SliderBar : 
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

    /// The SFValue class calculates the value of the slider bar.
    ///
    /// routes_in[0] is the isTouched field of the widget geometry.
    /// routes_in[1] is the contactPoint field of the widget geometry.
    /// routes_in[2] is the valueRange field.
    /// routes_in[3] is the stepLength field.
    /// routes_in[4] is the horizontal field.
    /// routes_in[5] is the isActive field of the touch sensor.
    /// routes_in[6] is the hitPoint_changed field of the touch sensor.
    /// routes_in[7] is the enabled field.
    class UI_API SFValue: public TypedField< SFFloat, 
      Types< MFBool, MFVec3f, SFVec2f, SFFloat, SFBool, SFBool, SFVec3f, SFBool > > {
      virtual void update();
    };

  protected:
    /// The MarkerTranslation field calculates the translation of
    /// the marker depending on the current value of the sliderbar.
    /// 
    /// routes_in[0] is the value field.
    /// routes_in[1] is the valueRange field.
    /// routes_in[2] is the horizontal field.
    /// routes_in[3] is the size field of the box geometry for this widget.
    /// This is needed to handle scaling correctly.
    class UI_API MarkerTranslation: 
      public TypedField< SFVec3f,
                         Types< SFValue, SFVec2f, SFBool, SFVec3f > > {
      virtual void update();
    };

    /// The MarkerRotation field calculates the translation of
    /// the marker depending if the slider bar is horizontal or not.
    /// 
    /// routes_in[0] is the horizontal field.
    class UI_API MarkerRotation: 
      public TypedField< SFRotation, SFBool > {
      virtual void update();
    };

    class NotField: public SFBool {
      virtual void update(){ 
        value = ! value;
      }
    };

  public:
    /// Constructor.
    SliderBar( Inst< SFNode          > _metadata        = 0,
               Inst< SFBound         > _bound           = 0,
               Inst< DisplayList     > _displayList     = 0,
               Inst< SFString        > _tag             = 0,
               Inst< SFBool          > _enabled         = 0,
               Inst< SFVec3f         > _desiredSize     = 0,
               Inst< SFVec3f         > _actualSize      = 0,
               Inst< SFAppearanceNode > _appearance     = 0,
               Inst< SFAppearanceNode > _textAppearance = 0,
               Inst< SFLayoutInfoNode > _layoutInfo     = 0,
               Inst< MFString        > _text            = 0,
               Inst< SFFontStyleNode > _fontStyle       = 0,
               Inst< SFValue         > _value           = 0,
               Inst< SFVec2f         > _valueRange      = 0,
               Inst< SFFloat         > _stepLength      = 0,
               Inst< SFColor         > _markerColor     = 0,
               Inst< SFBool          > _horizontal      = 0,
               Inst< SFColor         > _color           = 0,
               Inst< SFColor         > _textColor       = 0,
               Inst< SFTextureNode   > _texture         = 0,
               Inst< SFBool          > _isActive        = 0 );

    /// Override resize to avoid non-uniform scaling since this might cause
    /// trouble for some haptic renderers.
    virtual void resize( const Vec3f &new_size );

    virtual void initialize();

    /// The value of the slider bar. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile SliderBar_value.dot
    auto_ptr< SFFloat > value;

    /// The range of values the slider bar spans. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec2f( 0, 1 ) \n
    /// 
    /// \dotfile SliderBar_valueRange.dot
    auto_ptr< SFVec2f > valueRange;

    /// The resolution of the slider bar. Values will only update
    /// with the current step length. A stepLength of 0 will 
    /// always update to the most exact value available.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile SliderBar_stepLength.dot
    auto_ptr< SFFloat > stepLength;

    /// The color of the marker on the slider bar.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> RGB( 1, 1, 0 ) \n
    /// 
    /// \dotfile SliderBar_markerColor.dot
    auto_ptr< SFColor > markerColor;

    /// Determines if the slider should be in the horizontal or vertical
    /// direction.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> true \n
    /// 
    /// \dotfile SliderBar_horizontal.dot
    auto_ptr< SFBool > horizontal;

    /// The color of the slider bar.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> RGB( 0.6, 0.6, 0.6 ) \n
    /// 
    /// \dotfile SliderBar_color.dot
    auto_ptr< SFColor > color;

    /// The color of the text on the slider bar.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 0 0 \n
    /// 
    /// \dotfile TouchButton_textColor.dot
    auto_ptr< SFColor > textColor;

    /// Contains a X3DTextureNode to put on the SliderBar.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile SliderBar_texture.dot
    auto_ptr< SFTextureNode >  texture;

    /// If the widget is currently active
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile SliderBar_isActive.dot
    auto_ptr<SFBool> isActive;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:
    /// Internal field for calculating the translation of the marker.
    auto_ptr< MarkerTranslation > markerTranslation;

    /// Internal field for calculating the rotation of the marker.
    auto_ptr< MarkerRotation > markerRotation;

    /// Internal reference to the transform in which the marker resides.
    AutoRef< Transform > marker_transform;

     /// Perform || operator for the values in all fields routed to it.
    class OrField: public TypedField< SFBool, void,
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

    /// Field that returns true if any of the fields routed to it is true.
    /// C++ only field.
    auto_ptr<OrField> orField;


  };
}

#endif
