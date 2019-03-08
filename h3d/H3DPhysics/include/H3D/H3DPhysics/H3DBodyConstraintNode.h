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
/// \file H3DBodyConstraintNode.h
/// \brief Header file for H3DBodyConstraintNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DBODYCONSTRAINTNODE__
#define __H3DBODYCONSTRAINTNODE__

#include <H3D/H3DPhysics/H3DBodyInteractorNode.h>
#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/MFString.h>

namespace H3D{

  /// Abstract base class for nodes used to interact with body such as
  /// constraints, attachments, joints etc.
  ///
  /// The forceOutput field is used to control which output fields are to
  /// be generated for the next frame. In physics models, the amount of data
  /// that can be generated per frame can be quite extensive, particularly
  /// in complex models with a large number of joints. A typical application
  /// will need only a few of them, if any at all. This field is used to 
  /// control which of those outputs the author requires to be generated. 
  /// The values of the array are to describe the names, exactly, of the
  /// output field(s) that are to be updated at the start of the next frame. 
  /// Two special values are defined: "ALL" and "NONE". If "ALL" is specified
  /// anywhere in the array, all fields are to be updated. If "NONE" is 
  /// specified, no updates are performed. If the list of values is empty,
  /// it shall be treated as if "NONE" were specified. Other values provided
  /// in addition to "NONE" shall be ignored.
  ///
  /// \par Internal routes:
  /// \dotfile H3DBodyConstraintNode.dot
  class H3DPHYS_API H3DBodyConstraintNode : public H3DBodyInteractorNode {
  public:

    typedef MFH3DEngineOptions < H3DBodyConstraintNode > MFEngineOptions;
    friend class MFH3DEngineOptions < H3DBodyConstraintNode >;

    /// The ValueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// H3DBodyConstraintNode.
    class H3DPHYS_API ValueUpdater: 
      public EventCollectingField< PeriodicUpdate< Field > > {
    protected:
      virtual void update();
    };

    /// Constructor.
    H3DBodyConstraintNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,      
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0 );

    /// Destructor
    virtual ~H3DBodyConstraintNode();

    /// The forceOutput field is used to control which output fields are to
    /// be generated for the next frame. In physics models, the amount of data
    /// that can be generated per frame can be quite extensive, particularly
    /// in complex models with a large number of interactors. A typical application
    /// will need only a few of them, if any at all. This field is used to 
    /// control which of those outputs the author requires to be generated. 
    /// The values of the array are to describe the names, exactly, of the
    /// output field(s) that are to be updated at the start of the next frame. 
    /// Two special values are defined: "ALL" and "NONE". If "ALL" is specified
    /// anywhere in the array, all fields are to be updated. If "NONE" is 
    /// specified, no updates are performed. If the list of values is empty,
    /// it shall be treated as if "NONE" were specified. Other values provided
    /// in addition to "NONE" shall be ignored.
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile H3DBodyConstraintNode_forceOutput.dot
    auto_ptr< MFString > forceOutput;

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
    /// \dotfile H3DBodyConstraintNode_engineOptions.dot
    auto_ptr< MFEngineOptions > engineOptions;

    /// Returns the default xml containerField attribute value.
    /// For this node it is "constraints".
    ///
    virtual string defaultXMLContainerField() {
      return "constraints";
    }

    /// Initialize the constraint for the given PhysicsEngineThread. I.e. 
    /// create a new constraint in the physics engine with the parameters
    /// of the constraint fields. Returns 0 on success.
    virtual bool initializeConstraint( H3D::PhysicsEngineThread& pt );

    /// Deletes this constraint node from the given PhysicsEngineThread.
    bool deleteConstraint();

    /// Traverse the scene graph.
    virtual void traverseSG(H3D::TraverseInfo &ti);

    /// Get the H3DConstraintId for this node created by the PhysicsEngineThread.
    /// This is only valid if initializeConstraint has been called before.
    inline H3DConstraintId  getConstraintId() {
      return constraint_id;
    }

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// The valueUpdater field is used to update values in the
    /// PhysicsEngine according to changes of fields in the
    /// H3DBodyConstraintNode node.
    /// C++ only field.
    ///
    /// \dotfile H3DBodyConstraintNode_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater;

    /// Virtual function that returns a new instance of a subclass of
    /// PhysicsEngineParameters::ConstraintParameters that describes the 
    /// constraint. This should be overridden in each subclass of H3DBodyConstraintNode
    /// so that it returns the parameters for the constraint that is implemented.
    /// If all_params is true then it returns all field values regardless of whether
    /// the values have changed
    virtual PhysicsEngineParameters::ConstraintParameters *getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ()
#ifndef H3D_GENERATE_DOTROUTE_FILES
                                        = 0;
#else
    { return NULL; }
#endif

    /// The ID of this constraint
    H3DConstraintId constraint_id;

  };
}
#endif
