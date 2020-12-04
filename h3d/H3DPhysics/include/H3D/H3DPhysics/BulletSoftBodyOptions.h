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
/// \file BulletSoftBodyOptions.h
/// \brief Header file for BulletSoftBodyOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __BULLETSOFTBODYOPTIONS__
#define __BULLETSOFTBODYOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/H3DPhysics/FieldTemplates.h>
#include <H3D/SFInt32.h>
#include <H3D/MFString.h>
#include <H3D/SFBool.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup Bullet SoftBody
    /// Structure describing the state of a BulletSoftBodyOptions node
    /// to be passed to the physics simulation thread
    struct BulletSoftBodyParameters : public EngineOptionParameters {

      /// Constructor
      BulletSoftBodyParameters () : 
        collisionMargin ( 0.01f ),
        pIterations ( 1 ),
        dIterations ( 0 ),
        cIterations ( 4 ),
        collision_bit_mask ( CLUSTER_RIGIDSOFT | CLUSTER_SOFTSOFT ),
        nrClusters ( 64 ),
        bendingContraintDistance ( 2 ),
        enablePerEdgeStiffness ( false ),
        enableFastEdgeBuilding ( true ),
        softRigidClusterHardness ( 0.1f ),
        softRigidClusterImpulseSplit ( 0.5f ),
        softKineticClusterHardness ( 1.0f ),
        softKineticClusterImpulseSplit ( 0.5f ),
        softRigidHardness ( 1.0f ),
        softKineticHardness ( 0.1f ),
        collisionGroup( 0 ),
        collidesWith( 0 ),
        poseMatchingVolume( false ),
        poseMatchingFrame( false ),
        poseMatchingCoefficient( 0 ) {}

      // 'set' functions

      /// Set the bullet collision shape margin
      void setCollisionMargin ( H3DFloat _collisionMargin ) {
        update_bit_mask|= COLLISION_MARGIN;
        collisionMargin= _collisionMargin;
      }

      /// Set the number of position solver iterations
      void setPIterations ( H3DInt32 _pIterations ) {
        update_bit_mask|= PITERATIONS;
        pIterations= _pIterations;
      }

      /// Set the number of drift solver iterations
      void setDIterations ( H3DInt32 _dIterations ) {
        update_bit_mask|= DITERATIONS;
        dIterations= _dIterations;
      }

      /// Set the number of cluster iterations
      void setCIterations ( H3DInt32 _cIterations ) {
        update_bit_mask|= CITERATIONS;
        cIterations= _cIterations;
      }

      /// Set the options for collision detection for this soft body
      void setCollisionOptions ( unsigned int _collisionFlags ) {
        update_bit_mask|= COLLISION_OPTIONS;
        collision_bit_mask= _collisionFlags;
      }

      /// Set the number of clusters to generate
      void setNrClusters ( H3DInt32 _nrClusters ) {
        update_bit_mask|= NR_CLUSTERS;
        nrClusters= _nrClusters;
      }

      /// Set the distance used when generating contraints
      void setBendingContraintDistance( H3DInt32 _bendingContraintDistance ) {
        update_bit_mask|= BENDING_CONSTRAINT_DISTANCE;
        bendingContraintDistance= _bendingContraintDistance;
      }

      /// Enable or disable per edge stiffnesses
      void setEnablePerEdgeStiffness ( bool _enable ) {
        update_bit_mask|= ENABLE_PER_EDGE_STIFFNESS;
        enablePerEdgeStiffness= _enable;
      }

      /// Enable or disable fast edge building
      void setEnableFastEdgeBuilding ( bool _enable  ) {
        update_bit_mask|= ENABLE_FAST_EDGE_BUILDING;
        enableFastEdgeBuilding= _enable;
      }

      /// Set hardness of soft-rigid cluster collisions.
      void setSoftRigidClusterHardness ( H3DFloat _softRigidClusterHardness ) {
        update_bit_mask|= SOFT_RIGID_CLUSTER_HARDNESS;
        softRigidClusterHardness= _softRigidClusterHardness;
      }

      /// Set impulse split for soft-rigid cluster collisions.
      void setSoftRigidClusterImpulseSplit ( H3DFloat _softRigidClusterImpulseSplit ) {
        update_bit_mask|= SOFT_RIGID_CLUSTER_IMPULSE_SPLIT;
        softRigidClusterImpulseSplit= _softRigidClusterImpulseSplit;
      }

      /// Set hardness of soft-(kinematic rigid) cluster collisions.
      void setSoftKineticClusterHardness ( H3DFloat _softKineticClusterHardness ) {
        update_bit_mask|= SOFT_KINETIC_CLUSTER_HARDNESS;
        softKineticClusterHardness= _softKineticClusterHardness;
      }

      /// Set impulse split for soft-(kinematic rigid) cluster collisions.
      void setSoftKineticClusterImpulseSplit ( H3DFloat _softKineticClusterImpulseSplit ) {
        update_bit_mask|= SOFT_KINETIC_CLUSTER_IMPULSE_SPLIT;
        softKineticClusterImpulseSplit= _softKineticClusterImpulseSplit;
      }

      /// Set the hardness for contacts between softbodies and rigid bodies.
      void setSoftRigidHardness( H3DFloat _softRigidHardness ) {
        update_bit_mask |= SOFT_RIGID_HARDNESS;
        softRigidHardness = _softRigidHardness;
      }

      /// Set the kinetic hardness for contact with softbodies.
      void setSoftKineticHardness( H3DFloat _softKineticHardness ) {
        update_bit_mask |= SOFT_KINETIC_HARDNESS;
        softKineticHardness = _softKineticHardness;
      }

      /// Set which collision group this soft body belongs to.
      void setCollisionGroup( H3DInt32 _collisionGroup ) {
        update_bit_mask |= COLLISION_GROUP;
        collisionGroup = _collisionGroup;
      }

      /// Set which groups this soft body collides with.
      void setCollidesWith( H3DInt32 _collidesWith ) {
        update_bit_mask |= COLLIDES_WITH;
        collidesWith = _collidesWith;
      }

      /// Set the pose matching coefficient.
      void setPoseMatchingCoefficient( H3DFloat _poseMatchingCoefficient ) {
        update_bit_mask |= POSE_MATCHING_COEFFICIENT;
        poseMatchingCoefficient = _poseMatchingCoefficient;
      }

      /// Set the pose matching volume.
      void setPoseMatchingVolume( bool _enable ) {
        update_bit_mask |= POSE_MATCHING_VOLUME;
        poseMatchingVolume = _enable;
      }

      /// Set the pose matching frame.
      void setPoseMatchingFrame( bool _enable ) {
        update_bit_mask |= POSE_MATCHING_FRAME;
        poseMatchingFrame = _enable;
      }

      // 'get' functions

      /// Get the bullet collision shape margin
      H3DFloat getCollisionMargin () {
        return collisionMargin;
      }

      /// Get the number of position solver iterations
      H3DInt32 getPIterations () {
        return pIterations;
      }

      /// Get the number of drift solver iterations
      H3DInt32 getDIterations () {
        return dIterations;
      }

      /// Get the number of cluster iterations
      H3DInt32 getCIterations () {
        return cIterations;
      }

      /// Get the options for collision detection for this soft body
      unsigned int getCollisionOptions () {
        return collision_bit_mask;
      }

      /// Get the number of clusters to generate
      H3DInt32 getNrClusters () {
        return nrClusters;
      }

      /// Get the distance used when generating contraints
      H3DInt32 getBendingContraintDistance() {
        return bendingContraintDistance;
      }

      /// Is per edge stiffness enabled?
      bool getEnablePerEdgeStiffness () {
        return enablePerEdgeStiffness;
      }

      /// Is fast edge building enabled?
      bool getEnableFastEdgeBuilding () {
        return enableFastEdgeBuilding;
      }

      /// Hardness of soft-rigid cluster collisions.
      H3DFloat getSoftRigidClusterHardness () {
        return softRigidClusterHardness;
      }

      /// Impulse split for soft-rigid cluster collisions.
      H3DFloat getSoftRigidClusterImpulseSplit () {
        return softRigidClusterImpulseSplit;
      }

      /// Hardness of soft-(kinematic rigid) cluster collisions.
      H3DFloat getSoftKineticClusterHardness () {
        return softKineticClusterHardness;
      }

      /// Impulse split for soft-(kinematic rigid) cluster collisions.
      H3DFloat getSoftKineticClusterImpulseSplit () {
        return softKineticClusterImpulseSplit;
      }

      /// Get the hardness for contacts between softbodies and rigid bodies.
      H3DFloat getSoftRigidHardness() {
        return softRigidHardness;
      }

      /// Get the kinetic hardness for contact with softbodies.
      H3DFloat getSoftKineticHardness() {
        return softKineticHardness;
      }

      /// Get which collision group this soft body belongs to.
      H3DInt32 getCollisionGroup() {
        return collisionGroup;
      }

      /// Get which groups this soft body collides with.
      H3DInt32 getCollidesWith() {
        return collidesWith;
      }

      /// Get the pose matching coefficient.
      H3DFloat getPoseMatchingCoefficient() {
        return poseMatchingCoefficient;
      }

      /// Get the pose matching volume.
      bool getPoseMatchingVolume() {
        return poseMatchingVolume;
      }

      /// Get the pose matching frame.
      bool getPoseMatchingFrame() {
        return poseMatchingFrame;
      }

      // 'have' functions

      /// Returns true if the bullet collision shape margin has been specified
      bool haveCollisionMargin () {
        return (update_bit_mask & COLLISION_MARGIN) != 0;
      }

      /// Returns true if the number of position solver iterations has been specified
      bool havePIterations () {
        return (update_bit_mask & PITERATIONS) != 0;
      }

      /// Returns true if the number of drift solver iterations has been specified
      bool haveDIterations () {
        return (update_bit_mask & DITERATIONS) != 0;
      }

      /// Returns true if the number of drift cluster has been specified
      bool haveCIterations () {
        return (update_bit_mask & CITERATIONS) != 0;
      }

      /// Returns true if the options for collision detection for this soft body have been specified
      bool haveCollisionOptions () {
        return (update_bit_mask & COLLISION_OPTIONS) != 0;
      }

      /// Returns true if the number of clusters to generate has been specified
      bool haveNrClusters () {
        return (update_bit_mask & NR_CLUSTERS) != 0;
      }

      /// Returns true if the distance used when generating contraints has been specified
      bool haveBendingContraintDistance() {
        return (update_bit_mask & BENDING_CONSTRAINT_DISTANCE) != 0;
      }

      /// Returns true if the option to enable or disable per edge stiffness has been specified
      bool haveEnablePerEdgeStiffness () {
        return (update_bit_mask & ENABLE_PER_EDGE_STIFFNESS) != 0;
      }

      /// Returns true if the option to enable or disable fast edge building has been specified
      bool haveEnableFastEdgeBuilding() {
        return (update_bit_mask & ENABLE_FAST_EDGE_BUILDING) != 0;
      }

      /// Returns true if hardness of soft-rigid cluster collisions has been specified
      bool haveSoftRigidClusterHardness () {
        return (update_bit_mask & SOFT_RIGID_CLUSTER_HARDNESS) != 0;
      }

      /// Returns true if impulse split for soft-rigid cluster collisions has been specified
      bool haveSoftRigidClusterImpulseSplit () {
        return (update_bit_mask & SOFT_RIGID_CLUSTER_IMPULSE_SPLIT) != 0;
      }

      /// Returns true if hardness of soft-(kinematic rigid) cluster collisions has been specified
      bool haveSoftKineticClusterHardness () {
        return (update_bit_mask & SOFT_KINETIC_CLUSTER_HARDNESS) != 0;
      }

      /// Returns true if impulse split for soft-(kinematic rigid) cluster collisions has been specified
      bool haveSoftKineticClusterImpulseSplit () {
        return (update_bit_mask & SOFT_KINETIC_CLUSTER_IMPULSE_SPLIT) != 0;
      }

      /// Returns true if hardness for contacts between softbodies and rigid bodies has been specified.
      bool haveSoftRigidHardness() {
        return (update_bit_mask & SOFT_RIGID_HARDNESS) != 0;
      }

      /// Returns true if kinetic hardness for contact with softbodies has been specified.
      bool haveSoftKineticHardness() {
        return (update_bit_mask & SOFT_KINETIC_HARDNESS) != 0;
      }

      /// Returns true if which collision groups this soft body belongs to has been specified.
      bool haveCollisionGroup() {
        return (update_bit_mask & COLLISION_GROUP) != 0;
      }

      /// Returns true if which groups this soft body collides with has been specified.
      bool haveCollidesWith() {
        return (update_bit_mask & COLLIDES_WITH) != 0;
      }

      /// Returns true if pose matching coefficient has been specified.
      bool havePoseMatchingCoefficient() {
        return (update_bit_mask & POSE_MATCHING_COEFFICIENT) != 0;
      }

      /// Returns true if pose matching volume has been specified.
      bool havePoseMatchingVolume() {
        return (update_bit_mask & POSE_MATCHING_VOLUME) != 0;
      }

      /// Returns true if pose matching frame has been specified.
      bool havePoseMatchingFrame() {
        return (update_bit_mask & POSE_MATCHING_FRAME) != 0;
      }
      
      // collision options for bit mask
      static const unsigned int CLUSTER_RIGIDSOFT = 0x0001;
      static const unsigned int SDF_RIGIDSOFT = 0x0002;
      static const unsigned int VERTEXFACE_SOFTSOFT = 0x0004;
      static const unsigned int CLUSTER_SOFTSOFT = 0x0008;
      static const unsigned int CLUSTER_SELF = 0x0010;

    protected:
      // update bit mask flags
      static const unsigned int COLLISION_MARGIN                   = 0x00001;
      static const unsigned int PITERATIONS                        = 0x00002;
      static const unsigned int DITERATIONS                        = 0x00004;
      static const unsigned int CITERATIONS                        = 0x00008;
      static const unsigned int COLLISION_OPTIONS                  = 0x00010;
      static const unsigned int NR_CLUSTERS                        = 0x00020;
      static const unsigned int BENDING_CONSTRAINT_DISTANCE        = 0x00040;
      static const unsigned int ENABLE_PER_EDGE_STIFFNESS          = 0x00080;
      static const unsigned int ENABLE_FAST_EDGE_BUILDING          = 0x00100;
      static const unsigned int SOFT_RIGID_CLUSTER_HARDNESS        = 0x00200;
      static const unsigned int SOFT_RIGID_CLUSTER_IMPULSE_SPLIT   = 0x00400;
      static const unsigned int SOFT_KINETIC_CLUSTER_HARDNESS      = 0x00800;
      static const unsigned int SOFT_KINETIC_CLUSTER_IMPULSE_SPLIT = 0x01000;
      static const unsigned int SOFT_RIGID_HARDNESS                = 0x02000;
      static const unsigned int SOFT_KINETIC_HARDNESS              = 0x04000;
      static const unsigned int COLLISION_GROUP                    = 0x08000;
      static const unsigned int COLLIDES_WITH                      = 0x10000;
      static const unsigned int POSE_MATCHING_COEFFICIENT          = 0x20000;
      static const unsigned int POSE_MATCHING_VOLUME               = 0x40000;
      static const unsigned int POSE_MATCHING_FRAME                = 0x80000;

      /// The bullet collision shape margin
      H3DFloat collisionMargin;

      /// Number of positions solver iterations
      H3DInt32 pIterations;

      /// Number of drift solver iterations
      H3DInt32 dIterations;

      /// Number of cluster iterations
      H3DInt32 cIterations;

      /// Bitmask for collision options
      unsigned int collision_bit_mask;

      /// The number of clusters to generate 
      H3DInt32 nrClusters;

      /// The distance used when generating bending constraints
      H3DInt32 bendingContraintDistance;

      /// If enabled, it is possible to set the stiffness of the soft body per edge using
      /// the edgeStiffness of H3DSoftBodyNode.
      bool enablePerEdgeStiffness;

      /// If enabled, optimisations to the time complexity of adding edges to the
      /// soft body are enabled. 
      bool enableFastEdgeBuilding;

      /// Hardness of soft-rigid cluster collisions.
      H3DFloat softRigidClusterHardness;

      /// Impulse split for soft-rigid cluster collisions.
      H3DFloat softRigidClusterImpulseSplit;

      /// Hardness of soft-(kinematic rigid) cluster collisions.
      H3DFloat softKineticClusterHardness;

      /// Impulse split for soft-(kinematic rigid) cluster collisions.
      H3DFloat softKineticClusterImpulseSplit;

      /// Hardness for contacts between softbodies and rigid bodies
      H3DFloat softRigidHardness;

      /// Kinetic hardness for contact with softbodies.
      H3DFloat softKineticHardness;

      /// Specifies which collision groups this soft body belongs to.
      H3DInt32 collisionGroup;
      /// Specifies which groups this soft body collides with.
      H3DInt32 collidesWith;

      /// Pose matching coefficient.
      H3DFloat poseMatchingCoefficient;
      /// Controls if pose matching volume should be used.
      bool poseMatchingVolume;
      /// Controls if pose matching frame should be used.
      bool poseMatchingFrame;
    };
  }

  /// \ingroup Bullet SoftBody
  /// Node used to specify options relating to an H3DSoftBodyNode that are specific to
  /// the Bullet physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an H3DSoftBodyNode node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/BulletOptions.x3d">BulletOptions.x3d</a>
  ///     ( <a href="examples/BulletOptions.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile BulletSoftBodyOptions.dot
  class H3DPHYS_API BulletSoftBodyOptions : public H3DEngineOptions {
  public:
    typedef EnumMField < MFString > MFCollisionOptions;

    /// Constructor.
    BulletSoftBodyOptions (
      Inst< SFNode  > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFFloat > _collisionMargin = 0,
      Inst< SFInt32 > _pIterations = 0,
      Inst< SFInt32 > _dIterations = 0,
      Inst< SFInt32 > _cIterations = 0,
      Inst< MFCollisionOptions > _collisionOptions = 0,
      Inst< SFInt32 > _nrClusters= 0,
      Inst< SFInt32 > _bendingContraintDistance= 0,
      Inst< SFBool > _enablePerEdgeStiffness= 0,
      Inst< SFBool > _enableFastEdgeBuilding= 0,
      Inst< SFFloat > _softRigidClusterHardness= 0,
      Inst< SFFloat > _softRigidClusterImpulseSplit= 0,
      Inst< SFFloat > _softKineticClusterHardness= 0,
      Inst< SFFloat > _softKineticClusterImpulseSplit= 0,
      Inst< SFFloat > _softRigidHardness = 0,
      Inst< SFFloat > _softKineticHardness = 0,
      Inst< SFInt32 > _collisionGroup = 0,
      Inst< SFInt32 > _collidesWith = 0,
      Inst< SFFloat > _poseMatchingCoefficient = 0,
      Inst< SFBool > _poseMatchingVolume = 0,
      Inst< SFBool > _poseMatchingFrame = 0 );

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "Bullet"
    virtual string getEngine () {
      return "Bullet";
    }

    /// The margin of the underlying bullet collision shape
    ///
    /// A larger value will help prevent interpenetration, but may result in
    /// unrealistically large separation of resting objects. The value is
    /// scaled by the worldScale field of the BulletWorldOptions node if specified.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.01 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_collisionMargin.dot
    auto_ptr < SFFloat > collisionMargin;

    /// The number of position solver iterations
    /// 
    /// A larger value should produce more accurate collision detection and help prevent fall-through
    /// during rigid-soft body interactions. However, a higher value will reduce the simulation loop rate
    /// and may result in instability. 
    ///
    /// The value of this field will also affect the characteristics of the soft body. A higher value
    /// will result in a 'stiffer' shape, while a lower value will produce a 'softer' shape for the
    /// same H3DSoftBodyNode stiffness parameters.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_pIterations.dot
    auto_ptr < SFInt32 > pIterations;

    /// The number of drift solver iterations
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_dIterations.dot
    auto_ptr < SFInt32 > dIterations;

    /// The number of cluster iterations
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 4 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_cIterations.dot
    auto_ptr < SFInt32 > cIterations;

    /// A number of strings that can be used to control some implementation specific collision 
    /// detection options used for this soft body.
    /// 
    /// The type of collision detection to use for rigid-soft and soft-soft collisions may be specified:
    /// Rigid-soft: SDF_RIGIDSOFT
    ///             CLUSTER_RIGIDSOFT
    ///               
    /// Soft-soft:  VERTEXFACE_SOFTSOFT
    ///             CLUSTER_SOFTSOFT
    /// Self:       CLUSTER_SELF
    ///
    /// Only one option for rigid-soft and one for soft-soft needs to be specified.
    ///
    /// If no option is given for a specific collision combination, then that collision is disabled
    ///
    /// Bullet implementation note: only self-collision for Cluster, not Vertex-Face yet
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> ["SDF_RIGIDSOFT"] \n
    /// <b>Valid values:</b> "CLUSTER_RIGIDSOFT", "SDF_RIGIDSOFT", "VERTEXFACE_SOFTSOFT", "CLUSTER_SOFTSOFT", "CLUSTER_SELF"
    /// 
    /// \dotfile BulletSoftBodyOptions_collisionOptions.dot
    auto_ptr < MFCollisionOptions > collisionOptions;

    /// The number of clusters to generate 
    ///
    /// Extract from Bullet code documentation: 
    /// Generate clusters (K-mean)
    /// generateClusters with k=0 will create a convex cluster for each tetrahedron or triangle
    /// otherwise an approximation will be used (better performance)
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 64 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_nrClusters.dot
    auto_ptr < SFInt32 > nrClusters;

    /// The distance used when generating bending constraints
    ///
    /// Extract from Bullet code documentation: 
    /// Generate bending constraints based on distance in the adjency graph
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 2 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_bendingContraintDistance.dot
    auto_ptr < SFInt32 > bendingContraintDistance;

    /// If enabled, it is possible to set the stiffness of the soft body per edge using
    /// the edgeStiffness of H3DSoftBodyNode.
    ///
    /// Setting the stiffness per edge requires additional pre-processing and so can
    /// increase load time for large soft body models. Therfore, this option should 
    /// be disabled if you do not plan to set the soft body's stiffness per edge.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> FALSE \n
    ///
    auto_ptr < SFBool > enablePerEdgeStiffness;

    /// If enabled, optimisations to the time complexity of adding edges to the
    /// soft body are enabled. 
    ///
    /// Without this option enabled edge load time is O(n^2), with the optimisations
    /// it is O(n)
    ///
    /// The improvement in time complexity is achieved at the cost of memory.
    /// The optimisation requires additional memory during initialization proportional to n^2 where n is
    /// the number of nodes/vertices in the soft body.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> TRUE \n
    ///
    auto_ptr < SFBool > enableFastEdgeBuilding;

    /// Hardness of soft-rigid cluster collisions.
    ///
    /// Only affects CLUSTER_RIGIDSOFT, see collisionOptions
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> 0.1 \n
    ///
    auto_ptr < SFFloat > softRigidClusterHardness;

    /// Impulse split for soft-rigid cluster collisions.
    ///
    /// Only affects CLUSTER_RIGIDSOFT, see collisionOptions
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> 0.5 \n
    ///
    auto_ptr < SFFloat > softRigidClusterImpulseSplit;

    /// Hardness of soft-(kinematic rigid) cluster collisions.
    ///
    /// Only affects CLUSTER_RIGIDSOFT, see collisionOptions
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> 1.0 \n
    ///
    auto_ptr < SFFloat > softKineticClusterHardness;

    /// Impulse split for soft-(kinematic rigid) cluster collisions.
    ///
    /// Only affects CLUSTER_RIGIDSOFT, see collisionOptions
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> 0.5 \n
    ///
    auto_ptr < SFFloat > softKineticClusterImpulseSplit;

    /// Hardness of soft-rigid (non-cluster) collisions.
    ///
    /// Only affects SDF_RIGIDSOFT, see collisionOptions
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> 1.0 \n
    ///
    auto_ptr < SFFloat > softRigidHardness;

    /// Hardness of soft-(kinematic-rigid) (non-cluster) collisions.
    ///
    /// Only affects SDF_RIGIDSOFT, see collisionOptions
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default:</b> 0.1 \n
    ///
    auto_ptr < SFFloat > softKineticHardness;

    /// The collisionGroup field is treated as a bit mask which specify the
    /// collision group for a body. All bodies that belong to the
    /// same group should use the same number here. 0 Means that 
    /// the body is not part of a group and in that case collidesWith
    /// will be ignored and the body will behave as if no
    /// BulletSoftBodyOptions was provided.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_collisionGroup.dot
    auto_ptr < SFInt32 > collisionGroup;

    /// The collidesWith field is treated as a bit mask in which each
    /// bit with a value of 1 represents a collisionGroup that a
    /// body using this BulletSoftBodyOptions node will collide with.
    /// If for example collisionGroup = 1 and collidesWith = 1
    /// then all bodies that use this BulletRigidBodyOptions node will
    /// collide with each other.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_collidesWith.dot
    auto_ptr < SFInt32 > collidesWith;

    /// Pose matching coefficient
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile BulletSoftBodyOptions_poseMatchingCoefficient.dot
    auto_ptr < SFFloat > poseMatchingCoefficient;

    /// Enable volume pose matching
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> false \n
    /// 
    /// \dotfile BulletSoftBodyOptions_poseMatchingVolume.dot
    auto_ptr < SFBool > poseMatchingVolume;

    /// Enable frame pose matching
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> false \n
    /// 
    /// \dotfile BulletSoftBodyOptions_poseMatchingFrame.dot
    auto_ptr < SFBool > poseMatchingFrame;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
  };
}
#endif
