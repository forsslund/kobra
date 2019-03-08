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
/// \file MIPVolumeStyle.h
/// \brief Header file for MIPVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MIPVOLUMESTYLE_H__
#define __MIPVOLUMESTYLE_H__

#include <H3D/MedX3D/ProjectionVolumeStyle.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/DependentNodeFields.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class MIPVolumeStyle
  /// The Maximum Intensity Projection (MIP) volume style uses the voxel data
  /// directly to generate output colour based on the maximum and minimum
  /// values of voxel data along the viewing rays from the eye point.
  /// This rendering style also includes the option to use the extended form of
  /// Local Maximum Intensity Projection.
  ///
  /// The output colour is determined by projecting rays into the voxel data
  /// from the viewer location and finding the maximum voxel value found along
  /// that ray. If the intensityThreshold value is non-zero then rendering will
  /// use the first maximum value encountered that exceeds the threshold rather
  /// than the maximum found along the entire ray.
  ///
  /// Since the output of this node is intensity values, all colour components
  /// will have the same value. The intensity is derived from the average of
  /// all colour components of the voxel data (though typical usage will only
  /// use single component textures). The Alpha channel is passed through as-is
  /// from the underlying data. If there is no alpha channel, then assume a
  /// value of 1.
  ///
  /// Cg = maxk=0..N(sk)
  /// Og = Ov
  ///
  /// where
  /// - sk is sample k encountered traversing the ray from the eye point into
  /// the object.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/MIPVolumeStyle.x3d">MIPVolumeStyle.x3d</a>
  ///     ( <a href="x3d/MIPVolumeStyle.x3d.html">Source</a> )
  /// \deprecated Deprecated and will be removed in the future.
  class MEDX3D_API MIPVolumeStyle : public ProjectionVolumeStyle {
  public:
    /// The SFTextureNode field is dependent on the displayList field
    /// of the containing X3DTextureNode node.
    typedef DependentSFNode< X3DTextureNode, 
                             FieldRef< H3DDisplayListObject,
                                       H3DDisplayListObject::DisplayList,
                                       &H3DDisplayListObject::displayList >, 
                             true >
    SFTextureNode;

    /// Field that contains the default transfer function
    struct DefaultTransferFunction : public SFTextureNode {
      virtual void update();
    };
    
    /// Constructor.
    MIPVolumeStyle( Inst< DisplayList >   _displayList = 0,
                    Inst< SFBool >        _enabled     = 0,
                    Inst< SFNode >        _metadata    = 0,
                    Inst< SFFloat >       _intensityThreshold = 0);

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
