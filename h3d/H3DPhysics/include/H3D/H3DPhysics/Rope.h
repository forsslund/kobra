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
/// \file Rope.h
/// \brief Header file for Rope, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ROPE__
#define __ROPE__

#include <H3D/H3DPhysics/H3DSoftBodyNode.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/IndexedLineSet.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// A rope is a deformable soft body described by a linear chain of linked vertices
  ///
  /// A Rope node is a concrete type of H3DSoftBodyNode where the geometry of the soft body
  /// is defined using an IndexedLineSet.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/Rope.x3d">Rope.x3d</a>
  ///     ( <a href="examples/Rope.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile Rope.dot
  class H3DPHYS_API Rope : public H3DSoftBodyNode {
  public:

    typedef SpecializedSFNode < IndexedLineSet, SFX3DGeometryNode > SFIndexedLineSet;

    /// Constructor.
    Rope (
      Inst< SFNode > _metadata = 0,
      Inst< SFMatrix4f > _transform = 0,
      Inst< SFH3DPhysicsMaterialNode > _material = 0,
      Inst< SFIndexedLineSet > _geometry = 0,
      Inst< MFX3DGeometryNode > _surfaceGeometry = 0,
      Inst< MFX3DNBodyCollidableNode > _collisionGeometry = 0,
      Inst< MFH3DGeometryMapping > _surfaceMapping = 0,
      Inst< MFH3DGeometryMapping > _collisionMapping = 0,
      Inst< SFH3DDeformationStrategyNode > _deformationStrategy = 0,
      Inst< MFH3DSoftBodyOutputNode > _output = 0,      
      Inst< LinkGeometry > _linkSurfaceGeometry= 0,
      Inst< LinkGeometry > _linkCollisionGeometry= 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< MFEngineOptions > _engineOptions = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new concrete instance of H3DSoftBodyNodeParameters appropriate for this subtype of H3DSoftBodyNode
    ///
    /// For this node it returns new instance of RopeParameters
    virtual PhysicsEngineParameters::RopeParameters* createSoftBodyParameters ();

    /// Returns a new concrete instance of H3DSoftBodyNodeParameters appropriate for this subtype of H3DSoftBodyNode
    /// and populated with values that reflect the current state of this node
    ///
    /// For this node it returns new instance of RopeParameters
    virtual PhysicsEngineParameters::RopeParameters* getSoftBodyParameters( bool all_params = false );

    /// Applies the parameters specified by params to the output fields of this node
    virtual void setSoftBodyParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// Subclasses should implement this function to use the current H3DGeometryMapping node to link the geometry
    /// to the surface geometry.
    ///
    /// This step is defered to subclasses as it depends on the type of X3DGeometryNode used by the soft body.
    virtual void linkGeometries (const X3DGeometryNode &sourceGeometry, X3DGeometryNodeList &linkingGeometries,
      const H3DGeometryMappingVector &mappings );

    /// Subclasses should implement this function to use the current H3DGeometryMapping node to update the
    /// surface geometry based on the soft body geometry.
    ///
    /// This step is defered to subclasses as it depends on the type of X3DGeometryNode used by the soft body.
    virtual void updateSurfaceGeometry ();

    /// Subclasses should implement this function to use the current H3DGeometryMapping node to update the
    /// collision geometry based on the soft body geometry.
    ///
    /// This step is defered to subclasses as it depends on the type of X3DGeometryNode used by the soft body.
    virtual void updateCollisionGeometry ();

  };
}
#endif
