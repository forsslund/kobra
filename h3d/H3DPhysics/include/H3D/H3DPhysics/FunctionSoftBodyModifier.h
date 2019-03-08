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
/// \file FunctionSoftBodyModifier.h
/// \brief Header file for FunctionSoftBodyModifier, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __FUNCTIONSOFTBODYMODIFIER__
#define __FUNCTIONSOFTBODYMODIFIER__

#include <H3D/H3DFunctionNode.h>
#include <H3D/H3DPhysics/H3DAdjacencySoftBodyDeformer.h>
#include <HAPI/HAPIFunctionObject.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// A node used to define how a haptic device will affect an H3DSoftBodyNode.
  /// The FunctionSoftBodyModifier node contains a user defined H3DFunctionNode
  /// that is used to calculate the forces for apply to vertices of the soft body
  /// as a function of their distance from the point of contact with the haptic 
  /// device.
  ///
  /// For example, an instance of GaussianFunction node can be specified as the 
  /// distanceToForce field, and its amplitude and width fields used to control
  /// the characteristics of haptic interaction with the soft body.
  ///
  /// A different instance of FunctionSoftBodyModifier may be used for each haptic
  /// device and each soft body, or instances may be shared. 
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/Cloth.x3d">Cloth.x3d</a>
  ///     ( <a href="examples/Cloth.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile FunctionSoftBodyModifier.dot
  class H3DPHYS_API FunctionSoftBodyModifier : public H3DAdjacencySoftBodyDeformer {
  public:

    typedef TypedSFNode < H3DFunctionNode > SFFunction;

    class H3DPHYS_API SavePose : public AutoUpdate < OnNewValueSField < SFBool > > {
    protected:
      virtual void onNewValue ( const bool& v );
    };

    /// Constructor.
    FunctionSoftBodyModifier (
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFGeometryNode > _hapticGeometry = 0,
      Inst< SFInt32 > _deviceIndex = 0,
      Inst< SFInt32 > _maxAdjacencyDistance = 0,
      Inst< SFFunction > _distanceToForce = 0,
      Inst< SFFloat > _maxDisplacement = 0,
      Inst< SFFloat > _fadeDistance = 0,
      Inst< SavePose > _savePose = 0 );

    /// Traverse the scene graph
    virtual void traverseSG ( TraverseInfo& ti );

    /// Calculate forces for deformation
    virtual void calculateForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams,
      HAPI::HAPIHapticsDevice& hd );

    /// A function node describing the magnitude of force on a vertex given the distance 
    /// between the vertex and the contact point
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile FunctionSoftBodyModifier_distanceToForce.dot
    auto_ptr < SFFunction > distanceToForce;

    /// The desired maximum amount by which the deformer will displace vertices from their initial position
    ///
    /// Beyond this maximum, the force will be faded out over a distance defined by the field fadeDistance,
    /// until at a displacement of maxDisplacement+fadeDistance, the applied force will be zero.
    /// 
    /// A value of -1 means no such maximum is enforced
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile FunctionSoftBodyModifier_maxDisplacement.dot
    auto_ptr < SFFloat > maxDisplacement;

    /// Distance over which to fade out vertex force to zero, after exceeding maxDisplacement
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.001 \n
    /// 
    /// \dotfile FunctionSoftBodyModifier_fadeDistance.dot
    auto_ptr < SFFloat > fadeDistance;

    /// When an event is received by this field, the current state of the soft body is used as the 
    /// new rest position, with regard to calculating the maximum allowable displacement of a vertex.
    ///
    /// If this field is never set, the state of the soft body when the deformer is first used is taken as the
    /// rest position.
    ///
    /// <b>Access type:</b> inputOnly \n
    /// <b>Default value:</b> false \n
    /// 
    /// \dotfile FunctionSoftBodyModifier_savePose.dot
    auto_ptr < SavePose > savePose;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Apply interaction forces to the specified vertex using the distanceToForce function
    ///
    void applyForceToVertex ( size_t vertexIndex, PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams, Vec3f position, Vec3f force );

    void updatePose ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams );

    /// Distance to force function as a HAPI function object
    auto_ptr<HAPI::HAPIFunctionObject> hapiDistanceToForce;

    PhysicsEngineParameters::H3DSoftBodyNodeParameters::CoordList pose;

    bool updatePoseNow;
    
    H3DFloat fade_distance;

    H3DFloat max_displacement;

    H3DFloat gaussian_center;
    H3DFloat gaussian_width;
    H3DFloat gaussian_amplitude;
  };
}

#endif
