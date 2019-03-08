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
/// \file H3DDeviceSoftBodyModifierNode.h
/// \brief Header file for H3DDeviceSoftBodyModifierNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DDEVICESOFTBODYMODIFIERNODE__
#define __H3DDEVICESOFTBODYMODIFIERNODE__

#include <H3D/SFInt32.h>
#include <H3D/X3DGeometryNode.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DBodyModifierNode.h>

namespace H3D{  

  namespace PhysicsEngineParameters {
    struct H3DSoftBodyNodeParameters;
  }

  /// \ingroup AbstractNodes SoftBody
  /// Abstract base node for nodes used to define how a haptic device will affect 
  /// an H3DSoftBodyNode.
  ///
  /// \par Internal routes:
  /// \dotfile H3DDeviceSoftBodyModifierNode.dot
  class H3DPHYS_API H3DDeviceSoftBodyModifierNode : public H3DBodyModifierNode {
  public:

    typedef TypedSFNode< X3DGeometryNode > SFGeometryNode;

    /// Constructor.
    H3DDeviceSoftBodyModifierNode (
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFGeometryNode > _hapticGeometry = 0,
      Inst< SFInt32 > _deviceIndex = 0 );

    /// Traverse the scene graph
    ///
    /// Saves the transformation hierachy for use by the physics thread.
    virtual void traverseSG ( TraverseInfo& ti );

    /// The device that this modifier is linked to.
    ///
    /// If -1, then the position of the deformer in the array of deformers will
    /// be used as the device index instead.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile H3DDeviceSoftBodyModifierNode_deviceIndex.dot
    auto_ptr < SFInt32 > deviceIndex;

    /// If specified, this should be the geometry used for haptic rendering of
    /// the soft body. The deformer may use information about the contact
    /// between the device and this geometry to calculate the soft body's
    /// deformation.
    /// 
    /// If this field is NULL, then the surfaceGeometry field of the soft body
    /// is used instead. It is only required to set this field if the
    /// surfaceGeometry of the soft body is not the
    /// surface that is used for haptic rendering.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DDeviceSoftBodyModifierNode_hapticGeometry.dot
    auto_ptr < SFGeometryNode > hapticGeometry;

    H3DInt32 getDeviceIndex () {
      return devIndex;
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Virtual function that returns a new instance of a subclass of
    /// PhysicsEngineParameters::ModifierParameters that describes the 
    /// modifier. This should be overridden in each subclass of H3DBodyModifierNode
    /// so that it returns the parameters for the modifier that is implemented.
    /// If all_params is true then it returns all field values regardless of whether
    /// the values have changed
    virtual PhysicsEngineParameters::ModifierParameters *getModifierParameters( bool all_params = false );

    /// Matrix used to save the inverse of the transformation hierachy
    /// which may then be accessed from the physics thread
    Matrix4f accumulatedInverse;

    H3DInt32 devIndex;
  };
}

#endif
