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
/// \file H3DBodyModifierNode.h
/// \brief Header file for H3DBodyModifierNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DBODYMODIFIERNODE__
#define __H3DBODYMODIFIERNODE__

#include <H3D/H3DPhysics/H3DBodyInteractorNode.h>
#include <H3D/H3DPhysics/H3DEngineOptions.h>

namespace H3D{  

  /// Abstract base class for nodes used to interact with body such as
  /// vertex forces, haptic device modifiers etc.
  ///
  /// \par Internal routes:
  /// \dotfile H3DBodyModifierNode.dot
  class H3DPHYS_API H3DBodyModifierNode : public H3DBodyInteractorNode {
  public:

    typedef MFH3DEngineOptions < H3DBodyModifierNode > MFEngineOptions;
    friend class MFH3DEngineOptions < H3DBodyModifierNode >;

    /// The ValueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// H3DBodyConstraintNode.
    class H3DPHYS_API ValueUpdater: 
      public EventCollectingField< PeriodicUpdate< Field > > {
    protected:
      virtual void update();
    };

    /// Constructor.
    H3DBodyModifierNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,      
      Inst< MFEngineOptions > _engineOptions = 0 );

    /// Destructor
    virtual ~H3DBodyModifierNode();

    /// Options relating to this node that are physics engine specific.
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
    /// \dotfile H3DBodyModifierNode_engineOptions.dot
    auto_ptr< MFEngineOptions > engineOptions;

    /// Returns the default xml containerField attribute value.
    /// For this node it is "deformers".
    ///
    virtual string defaultXMLContainerField() {
      return "modifiers";
    }

    /// Initialize the modifier for the given PhysicsEngineThread. I.e. 
    /// create a new modifier in the physics engine with the parameters
    /// of the modifier fields. Returns 0 on success.
    bool initializeModifier( H3D::PhysicsEngineThread *pt );

    /// Deletes this modifier node from the given PhysicsEngineThread.
    bool deleteModifier();

    /// Traverse the scene graph.
    virtual void traverseSG(H3D::TraverseInfo &ti);

    /// Get the H3DModifierId for this node created by the PhysicsEngineThread.
    /// This is only valid if initializeModifier has been called before.
    inline H3DModifierId  getModifierId() {
      return modifier_id;
    }

    /// Calculate forces for deformation
    virtual void calculateForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams,
      HAPI::HAPIHapticsDevice& hd )
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    {}
#endif

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// The valueUpdater field is used to update values in the
    /// PhysicsEngine according to changes of fields in the
    /// H3DBodyConstraintNode node.
    /// C++ only field.
    ///
    /// \dotfile H3DBodyModifierNode_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater;

    /// Virtual function that returns a new instance of a subclass of
    /// PhysicsEngineParameters::ModifierParameters that describes the 
    /// modifier. This should be overridden in each subclass of H3DBodyModifierNode
    /// so that it returns the parameters for the modifier that is implemented.
    /// If all_params is true then it returns all field values regardless of whether
    /// the values have changed
    virtual PhysicsEngineParameters::ModifierParameters *getModifierParameters( bool all_params = false );

    /// Returns a new concrete instance of ModifierParameters appropriate for this subtype of H3DBodyModifierNode
    virtual PhysicsEngineParameters::ModifierParameters* createModifierParameters ();

    /// The ID of this modifier
    H3DModifierId modifier_id;

    /// A lock used to provide thread-safety for variables shared between
    /// the graphics and the physics thread
    MutexLock traverseLock;

  };
}
#endif
