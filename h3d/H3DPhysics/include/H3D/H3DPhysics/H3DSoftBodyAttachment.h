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
/// \file H3DSoftBodyAttachment.h
/// \brief Header file for H3DSoftBodyAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DSOFTBODYATTACHMENT__
#define __H3DSOFTBODYATTACHMENT__

#include <H3D/H3DPhysics/H3DAttachmentNode.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// An H3DSoftBodyAttachment allows vertices of an H3DSoftBodyNode to be attached to
  /// another H3DSoftBodyNode.
  ///
  /// \par Internal routes:
  /// \dotfile H3DSoftBodyAttachment.dot
  class H3DPHYS_API H3DSoftBodyAttachment : public H3DAttachmentNode {
  public:

    /// Constructor.
    H3DSoftBodyAttachment ( 
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DSoftBodyNode > _body1 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFH3DSoftBodyNode > _body2 = 0,
      Inst< TrackedMFInt32 > _index = 0,
      Inst< TrackedMFInt32 > _index2 = 0 );

    /// Indices of coordinates in body2 which will be attached
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile H3DSoftBodyAttachment_index2.dot
    auto_ptr < TrackedMFInt32 > index2;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of 
    /// PhysicsEngineParameters::H3DSoftBodyAttachmentParameters with the values updated with
    /// corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

  };
}

#endif
