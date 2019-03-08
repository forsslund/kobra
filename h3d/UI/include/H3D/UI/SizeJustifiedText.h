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
/// \file SizeJustifiedText.h
/// \brief Header file for SizeJustifiedText, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SIZEJUSTIFIEDTEXT_H__
#define __SIZEJUSTIFIEDTEXT_H__

// UI includes
#include <H3D/UI/UI.h>

// H3D includes
#include <H3D/Text.h>
#include <H3D/SFVec2f.h>

namespace H3D {

  /// \ingroup UINodes
  /// \class SizeJustifiedText
  /// \brief The SizeJustifiedText node specifies a two-sided, flat SizeJustifiedText string object
  /// positioned in the Z=0 plane of the local coordinate system based on
  /// values defined in the fontStyle field (see FontStyle). 
  ///
  /// Text nodes may contain multiple text strings specified using the
  /// UTF-8 encoding. The text strings are stored in the order in which the
  /// text mode characters are to be produced as defined by the parameters
  /// in the FontStyle node.
  ///
  /// The text strings are contained in the string field. The fontStyle
  /// field contains one FontStyle node that specifies the font size, font
  /// family and style, direction of the text strings, and any specific
  /// language rendering techniques used for the text. 
  ///
  /// The maxExtent field limits and compresses all of the text strings if
  /// the length of the maximum string is longer than the maximum extent, as
  /// measured in the local coordinate system. If the text string with the
  /// maximum length is shorter than the maxExtent, then there is no
  /// compressing. The maximum extent is measured horizontally for
  /// horizontal text (FontStyle node: horizontal=TRUE) and vertically for
  /// vertical text (FontStyle node: horizontal=FALSE). The maxExtent field
  /// shall be greater than or equal to zero.
  ///
  /// The length field contains an MFFloat value that specifies the length
  /// of each text string in the local coordinate system. If the string is
  /// too short, it is stretched (either by scaling the text or by adding
  /// space between the characters). If the string is too long, it is
  /// compressed (either by scaling the text or by subtracting space between
  /// the characters). If a length value is missing (for example, if there
  /// are four strings but only three length values), the missing values are
  /// considered to be 0. The length field shall be greater than or equal to
  /// zero. 
  ///
  /// Specifying a value of 0 for both the maxExtent and length fields
  /// indicates that the string may be any length. 
  /// 
  /// The solid field determines whether one or both sides of each polygon
  /// shall be displayed. If solid is FALSE, each polygon is visible
  /// regardless of the viewing direction and if it is TRUE back face culling
  /// is performed to only show the front face of the polygons.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/SizeJustifiedText.x3d">SizeJustifiedText.x3d</a>
  ///     ( <a href="x3d/SizeJustifiedText.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SizeJustifiedText.dot
  class  UI_API SizeJustifiedText : public Text {
  protected:
    typedef TypedSFNode< X3DFontStyleNode > SFFontStyleNode;

    /// The SFBound field is specialized to update itself from the
    /// values in the fields of the SizeJustifiedText node.
    /// 
    /// routes_in[0] is the fontStyle field.
    /// routes_in[1] is the length field.
    /// routes_in[2] is the maxExtent field.
    /// routes_in[3] is the string field.
    class  UI_API SFBound: 
      public TypedField< Text::SFBound,
      Types< SFFontStyleNode, MFFloat, SFFloat, MFString, SFVec2f > > {
      /// Updates to a BoxBound containing the text.
      virtual void update();
    }; 

  public:
    
    /// Constructor.
    SizeJustifiedText( Inst< SFNode         > _metadata     = 0,
                       Inst< SFBound         > _bound       = 0,
                       Inst< DisplayList     > _displayList = 0,
                       Inst< SFFontStyleNode > _fontStyle   = 0,
                       Inst< MFFloat         > _length      = 0,
                       Inst< SFFloat         > _maxExtent   = 0,
                       Inst< MFString        > _string      = 0,
                       Inst< MFVec2f         > _lineBounds  = 0,
                       Inst< SFVec3f         > _origin      = 0,
                       Inst< SFVec2f         > _textBounds  = 0,
                       Inst< SFBool          > _solid       = 0,
                       Inst< SFVec2f         > _size        = 0,
                       Inst< SFVec3f         > _textStartPos = 0 );

    /// The size of the text in meters.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1 1 \n
    /// <b>Valid range:</b> >=0
    /// 
    /// \dotfile SizeJustifiedText_size.dot
    auto_ptr< SFVec2f  > size;

    /// The start position of the text.
    ///
    /// <b>Access type:</b> OutputOnly \n
    /// <b>Default value:</b> 0 0 0 \n
    /// <b>Valid range:</b> >=0
    /// 
    /// \dotfile SizeJustifiedText_textStartPos.dot
    auto_ptr< SFVec3f > textStartPos;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    Vec3f getMinorJustification( X3DFontStyleNode *font );

    Vec3f getMajorJustification( X3DFontStyleNode *font );

  protected:
    /// Justify the text in the minor alignment by translating it in the 
    /// way described in the FontStyle node.
    virtual void justifyMinor( const vector< string > &text,
                               X3DFontStyleNode *font );

    /// Justify the line of text in the major alignment by translating
    /// it in the way described in the FontStyle node.
    virtual void justifyLine( const string& text,
                              X3DFontStyleNode *font );
  };
}

#endif
