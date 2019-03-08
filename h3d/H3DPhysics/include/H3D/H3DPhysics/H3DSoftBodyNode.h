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
/// \file H3DSoftBodyNode.h
/// \brief Header file for H3DSoftBodyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DSOFTBODYNODE__
#define __H3DSOFTBODYNODE__

#include <H3D/H3DPhysics/H3DBodyNode.h>

#include <H3D/H3DPhysics/SoftBodyParameters.h>
#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/H3DPhysics/H3DGeometryMapping.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>
#include <H3D/H3DPhysics/H3DDeformationStrategyNode.h>
#include <H3D/H3DPhysics/H3DSoftBodyOutputNode.h>
#include <H3D/H3DPhysics/H3DPhysicsMaterialNode.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/X3DComposedGeometryNode.h>
#include <H3D/X3DGeometryNode.h>
#include <H3D/SFMatrix4f.h>
#include <H3D/MFFloat.h>

namespace H3D{  

  /// \ingroup AbstractNodes SoftBody
  /// Abstract base node for a varoius different types of soft bodies.
  ///
  /// Subclasses differ, for example, in terms of how the geometry field is
  /// interpreted to construct the geometry of the soft body. In the case of
  /// the Cloth node the geometry field contains an IndexedTriangleSet describing
  /// the surface of the soft body, whereas in a SoftBody node the geometry field
  /// contains an IndexedTetraSet describing the volume of the SoftBody.
  ///
  /// \par Internal routes:
  /// \dotfile H3DSoftBodyNode.dot
  class H3DPHYS_API H3DSoftBodyNode : public H3DBodyNode {
  public:

    typedef TypedSFNode < X3DGeometryNode > SFX3DGeometryNode;
    typedef TypedMFNode < X3DGeometryNode > MFX3DGeometryNode;
    typedef TypedMFNode < X3DNBodyCollidableNode > MFX3DNBodyCollidableNode;
    typedef TypedMFNode < H3DGeometryMapping > MFH3DGeometryMapping;

    typedef vector < H3DGeometryMapping*      > H3DGeometryMappingVector;
    typedef vector < X3DGeometryNode*         > X3DGeometryNodeList;    
    typedef vector < X3DComposedGeometryNode* > X3DComposedGeometryNodeVector;
    typedef vector < X3DNBodyCollidableNode*  > X3DNBodyCollidableNodeList; 

    /// The SFH3DPhysicsMaterialNode is dependent on the
    /// materialChanged field of the contained H3DPhysicsMaterialNode.
    typedef  DependentSFNode< FieldRef<H3DPhysicsMaterialNode,
      Field,
      &H3DPhysicsMaterialNode::materialChanged > > 
      SFH3DPhysicsMaterialNode;

    /// The MFH3DSoftBodyOutputNode is dependent on the
    /// valueUpdater fields of the contained H3DSoftBodyOutputNodes.
    typedef  DependentMFNode< 
      H3DSoftBodyOutputNode,
      FieldRef<
        H3DSoftBodyOutputNode,
        H3DSoftBodyOutputNode::ValueUpdater,
        &H3DSoftBodyOutputNode::valueUpdater > > 
      MFH3DSoftBodyOutputNode;

    /// The SFH3DDeformationStrategyNode is dependent on the
    /// strategyChanged field of the contained H3DDeformationStrategyNode.
    typedef  DependentSFNode< FieldRef<H3DDeformationStrategyNode,
      Field,
      &H3DDeformationStrategyNode::strategyChanged > > 
      SFH3DDeformationStrategyNode;

    typedef MFH3DEngineOptions < H3DSoftBodyNode > MFEngineOptions;
    friend class MFH3DEngineOptions < H3DSoftBodyNode >;

    /// The ValueUpdater field is used to update values in the
    /// SoftBodyPhysicsEngineThread according to changes of fields in the
    /// H3DSoftBodyNode.
    class H3DPHYS_API ValueUpdater: 
      public EventCollectingField< PeriodicUpdate< Field > > {
    protected:
      virtual void update();
    };

    /// A fields used to link the surface geometry to the underlying 
    /// soft body geometry when the either changes.
    class H3DPHYS_API LinkGeometry : public Field {
    protected:
      virtual void update ();
    };

    /// Constructor.
    H3DSoftBodyNode(
      Inst< SFNode > _metadata = 0,
      Inst< SFMatrix4f > _transform = 0,
      Inst< SFH3DPhysicsMaterialNode > _physicsMaterial = 0,
      Inst< SFX3DGeometryNode > _geometry = 0,
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

    /// Traverse the scene graph.
    virtual void traverseSG(H3D::TraverseInfo &ti);

    /// Returns the default xml containerField attribute value.
    /// For this node it is "softBodies".
    virtual string defaultXMLContainerField() {
      return "softBodies";
    }

    /// Initialize the soft body for the given SoftBodyPhysicsThread. I.e. 
    /// create a new soft body in the physics engine with the parameters
    /// of the soft body fields. Returns 0 on success.
    virtual bool initializeBody( PhysicsEngineThread& pt );

    /// Deletes this rigid body node from the given H3DRigidBodyPhysicsEngine.
    virtual bool deleteBody();

  protected:
    /// A field used to link the surface geometry to the underlying 
    /// soft body geometry when either changes.
    /// C++ access only field.
    ///
    /// \dotfile H3DSoftBodyNode_linkSurfaceGeometry.dot
    auto_ptr< LinkGeometry > linkSurfaceGeometry;

    /// A fields used to link the collision geometry to the underlying 
    /// soft body geometry when the either changes.
    /// C++ access only field.
    ///
    /// \dotfile H3DSoftBodyNode_linkCollisionGeometry.dot
    auto_ptr< LinkGeometry > linkCollisionGeometry;

    /// The valueUpdater field is used to update values in the
    /// SoftBodyPhysicsEngine according to changes of fields in the
    /// H3DSoftBodyNode node.
    /// C++ access only field.
    ///
    /// \dotfile H3DSoftBodyNode_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater;

    /// A field used to collect events that require the geometry of
    /// the soft body to be re-applied in the simulation thread
    /// C++ access only field.
    ///
    /// \dotfile H3DSoftBodyNode_geometryChanged.dot
    auto_ptr < Field > geometryChanged;

    /// The list of surface geometries.
    X3DGeometryNodeList sgList;

    /// The list of collidable nodes.
    X3DNBodyCollidableNodeList cnList;

  public:

    /// The transformation applied to the soft body
    ///
    /// When the geometry is specified, the coordinates of the geometry will be 
    /// transformed by this value before being sent to the physics engine.
    ///
    /// A soft body has no persistent overall position, just a collection
    /// of vertex positions. Therefore this field is not updated by the
    /// simulation.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> SFMatrix4f () \n
    /// 
    /// \dotfile H3DSoftBodyNode_transform.dot
    auto_ptr < SFMatrix4f > transform;

    /// The H3DPhysicsMaterialNode including different types of
    /// H3DPhysicsMaterialPropertyNodes like mass, damping, elasticity
    /// etc.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_physicsMaterial.dot
    auto_ptr < SFH3DPhysicsMaterialNode > physicsMaterial;

    /// The geometry used as input to construct the soft body and then modified
    /// by the simulation to reflect the current state of the soft body.
    ///
    /// The allowable types of geometry may vary depending on the subtype of soft body
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_geometry.dot
    auto_ptr < SFX3DGeometryNode > geometry;

    /// The geometry used to render the surface of the soft body.
    ///
    /// The allowable types of geometry may vary depending on the subtype of soft body
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_surfaceGeometry.dot
    auto_ptr < MFX3DGeometryNode > surfaceGeometry;

    /// The geometry used for collision detection.
    ///
    /// The allowable types of geometry may vary depending on the subtype of soft body
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_collisionGeometry.dot
    auto_ptr < MFX3DNBodyCollidableNode > collisionGeometry;

    /// An H3DGeometryMapping node which defines a mapping from the geometry field
    /// to the surfaceGeometry field.
    ///
    /// This allows deformations of the soft body geometry to be reflected by the 
    /// surface geometry, even though they might not share the same coordinates.
    ///
    /// Thus two different resolutions of mesh may be used for the surface and 
    /// soft body geometry.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_surfaceMapping.dot
    auto_ptr < MFH3DGeometryMapping > surfaceMapping;

    /// An H3DGeometryMapping node which defines a mapping from the geometry field
    /// to the collisionGeometry field.
    ///
    /// This allows the use of a different geometry than the geometry field for
    /// collision detection, even though they might not share the same coordinates.
    ///
    /// Thus two different resolutions of mesh may be used for the surface and 
    /// soft body geometry.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_collisionMapping.dot
    auto_ptr < MFH3DGeometryMapping > collisionMapping;

    /// An H3DDeformationStrategyNode defines the algorithm used to evaluate the
    /// deformation behviour such as mass-spring, Finite Element, chain-mail etc.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_deformationStrategy.dot
    auto_ptr < SFH3DDeformationStrategyNode > deformationStrategy;

    /// An H3DSoftBodyOutputNode is used to transfer data per unit element
    /// (vertex, edge, volume etc.) to the scenegraph as an ouput.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile H3DSoftBodyNode_output.dot
    auto_ptr < MFH3DSoftBodyOutputNode > output;

    /// Options relating to this soft body node that are physics engine specific.
    ///
    /// Any number of H3DEngineOptions nodes may be specified. The first one
    /// that relates to the current physics engine implementation will be used.
    ///
    /// Options nodes relating to physics engine implementations other than that
    /// currently in use will be ignored.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile H3DSoftBodyNode_engineOptions.dot
    auto_ptr< MFEngineOptions > engineOptions;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new concrete instance of H3DSoftBodyNodeParameters appropriate for this subtype of H3DSoftBodyNode
    virtual PhysicsEngineParameters::H3DSoftBodyNodeParameters* createSoftBodyParameters ()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// Returns a new concrete instance of H3DSoftBodyNodeParameters appropriate for this subtype of H3DSoftBodyNode
    /// and populated with values that reflect the current state of this node.
    virtual PhysicsEngineParameters::H3DSoftBodyNodeParameters* getSoftBodyParameters( bool all_params = false );

    /// Applies the parameters specified by params to the output fields of this node
    virtual void setSoftBodyParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// Subclasses should implement this function to use the current H3DGeometryMapping node to link the geometry
    /// to the surface geometry or collision geometry.
    ///
    /// This step is defered to subclasses as it depends on the type of X3DGeometryNode used by the soft body.
    virtual void linkGeometries (const X3DGeometryNode &sourceGeometry, X3DGeometryNodeList &linkingGeometries,
      const H3DGeometryMappingVector &mappings )
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    {}
#endif

    /// Subclasses should implement this function to use the current H3DGeometryMapping node to update the
    /// surface geometry based on the soft body geometry.
    ///
    /// This step is defered to subclasses as it depends on the type of X3DGeometryNode used by the soft body.
    virtual void updateSurfaceGeometry ()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    {}
#endif

    /// Subclasses should implement this function to use the current H3DGeometryMapping node to update the
    /// collision geometry based on the soft body geometry.
    ///
    /// This step is defered to subclasses as it depends on the type of X3DGeometryNode used by the soft body.
    virtual void updateCollisionGeometry ()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    {}
#endif

    /// Accumulate all user defined vertex forces and add to the specified H3DSoftBodyNodeParameters
    void addVertexForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// Helper function which returns the transformation of the soft body based on the current position, orientation and scale
    inline Matrix4f getTransform () {
      return transform->getValue();
    }

    /// Helper function which transforms a points in local coordinates of the soft body to world coords
    vector<Vec3f> transformCoords ( const vector<Vec3f>& coords );

    /// Variables used to create all parameters of the node incase the whole
    /// node is changed.
    H3DPhysicsMaterialNode *previousPhysicsMaterial;
    H3DDeformationStrategyNode *previousDeformationStrategy;
  };
}
#endif
