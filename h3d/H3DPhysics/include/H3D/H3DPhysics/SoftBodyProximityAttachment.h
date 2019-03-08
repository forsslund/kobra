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
/// \file SoftBodyProximityAttachment.h
/// \brief Header file for SoftBodyProximityAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SOFTBODYPROXIMITYATTACHMENT__
#define __SOFTBODYPROXIMITYATTACHMENT__

#include <H3D/H3DPhysics/SoftBodyAttachment.h>
#include <H3D/X3DComposedGeometryNode.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// A SoftBodyProximityAttachment allows two soft bodies to be attached to one another
  /// based on the proximity of their surface geometries. Soft body coordinates will be connected
  /// only if those coordinates are close to a pair of surface geometry coordinates, one from each
  /// soft body, that are also close to each other. The result is that the soft bodies are connected
  /// in areas where their surfaces are close to each other.
  ///
  /// The attachment generates the coordIndices and coordIndices2 fields, based on the state of body1 and
  /// body2, when the attachment is initialized, when it is first traversed.
  ///
  /// With complex geometries initialization can take some time, so it may be useful to use this
  /// attachment node to pre-generate coordIndices and coordIndices2 fields for use with a regular
  /// SoftBodyAttachmentNode.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/SoftBodyProximityAttachment.x3d">SoftBodyProximityAttachment.x3d</a>
  ///     ( <a href="examples/SoftBodyProximityAttachment.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SoftBodyProximityAttachment.dot
  class H3DPHYS_API SoftBodyProximityAttachment : public SoftBodyAttachment {
    public:
      typedef TypedMFNode < X3DComposedGeometryNode > SFX3DComposedGeometry;

    /// Constructor.
    SoftBodyProximityAttachment (
              Inst< SFNode > _metadata = 0,
              Inst< ValueUpdater > _valueUpdater = 0,
              Inst< SFH3DSoftBodyNode > _body1 = 0,
              Inst< MFString     > _forceOutput = 0,
              Inst< MFEngineOptions > _engineOptions = 0,
              Inst< SFH3DSoftBodyNode > _body2 = 0,
              Inst< TrackedMFInt32 > _index = 0,
              Inst< TrackedMFInt32 > _index2 = 0,
              Inst< SFFloat > _surfaceProximity = 0,
              Inst< SFFloat > _surfaceSoftBodyProximity = 0,
              Inst< SFFloat > _softBodyProximity = 0,
              Inst< SFX3DComposedGeometry > _surfaceGeometry1 = 0,
              Inst< SFX3DComposedGeometry > _surfaceGeometry2 = 0
      );

    /// Initialize the attachment with the specified physics thread.
    /// Adds the attachment to the simulation.
    ///
    /// Overridden to evaluate attachment parameters and generate indices for attachment
    ///
    virtual bool initializeConstraint( PhysicsEngineThread& pt );

    /// The maximum distance between points on the surface geometries that will result in attachment
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.001 \n
    /// 
    /// \dotfile SoftBodyProximityAttachment_surfaceProximity.dot
    auto_ptr < SFFloat > surfaceProximity;

    /// The maximum distance between points identified on the surfaces in close proximity and points in
    /// the soft body simulation geometry that will result in attachment
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.001 \n
    /// 
    /// \dotfile SoftBodyProximityAttachment_surfaceSoftBodyProximity.dot
    auto_ptr < SFFloat > surfaceSoftBodyProximity;

    /// The maximum distance between two points, one in body1 and the other in body2, that will result in attachment
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.001 \n
    /// 
    /// \dotfile SoftBodyProximityAttachment_softBodyProximity.dot
    auto_ptr < SFFloat > softBodyProximity;

    /// An optional list of surface geometries associated with body1. If the list is empty, then
    /// the surfaceGeometry field of body1 is used instead.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile SoftBodyProximityAttachment_surfaceGeometry1.dot
    auto_ptr < SFX3DComposedGeometry > surfaceGeometry1;

    /// An optional list of surface geometries associated with body2. If the list is empty, then
    /// the surfaceGeometry field of body2 is used instead.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile SoftBodyProximityAttachment_surfaceGeometry2.dot
    auto_ptr < SFX3DComposedGeometry > surfaceGeometry2;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  
  protected:

    /// Generate the indices fields from the current state of the soft bodies
    void generateConnectedIndices ();
  };
}

#endif
