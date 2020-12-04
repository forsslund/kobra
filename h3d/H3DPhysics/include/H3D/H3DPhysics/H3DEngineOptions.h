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
/// \file H3DEngineOptions.h
/// \brief Header file for H3DEngineOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DENGINEOPTIONS__
#define __H3DENGINEOPTIONS__

#include <H3D/H3DPhysics/PhysicsEngineParameters.h>
#include <H3D/X3DNode.h>
#include <H3D/FieldTemplates.h>

namespace H3D{

  /// \ingroup AbstractNodes
  /// Abstract base node for options that are specific to a particular physics engine implementation.
  /// Subclasses of this node type enable parameters unique to a particular physics engine to be specified.
  ///
  /// Subclasses of H3DEngineOptions node specific to each physics engine implementation can be
  /// added to the engineOptions field of various H3DPhysics nodes. The physics engine
  /// will look for the first H3DEngineOptions in the engineOptions field that is applicable and apply
  /// those options to the node containing the engineOptions field. Options relating to other 
  /// physics engines will be ignored.
  class H3DPHYS_API H3DEngineOptions : public X3DNode {
  public:

    /// A field type used to collect changes to fields of the H3DEngineOptions
    ///
    /// The parent node will request an instance of EngineOptionParameters by calling
    /// getParameters(), which will be added to the physics engine struct for that node.
    class H3DPHYS_API ValueUpdater : 
      public EventCollectingField < Field > {
    public:
      ValueUpdater() : allParams( false ) {}

      virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );
    protected:
      virtual void update();

      AutoRef < PhysicsEngineParameters::EngineOptionParameters > params;
      bool allParams;
    };

    /// Constructor.
    H3DEngineOptions(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater  > _valueUpdater = 0
      );

    /// Returns the default xml containerField attribute value.
    ///
    /// For this node it is "engineOptions".
    virtual string defaultXMLContainerField() {
      return "engineOptions";
    }

    /// Returns the string identifier of the physics engine that these options relate to.
    virtual string getEngine ()= 0;

    /// A field type used to collect changes to fields of the H3DEngineOptions
    /// Only accessible from C++.
    auto_ptr< ValueUpdater > valueUpdater;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false ) {
      return NULL;
    }
  };

  /// A field template for nodes that contain H3DEngineOption nodes
  ///
  /// Handles setting up routes to the node's valueUpdater field in order to trigger an update
  /// when H3DEngineOptions fields are modified.
  ///
  /// Provides a look-up function to retrieve an options node for a specific physics engine
  template < typename NodeType >
  class H3DPHYS_API MFH3DEngineOptions: public TypedMFNode< H3DEngineOptions > {
  public:

    /// Returns the options node relevent to the specified physics engine.
    /// Returns NULL if node is specified.
    H3DEngineOptions* getOptions ( string engine ) {
      const NodeVector& options= getValue();
      for ( const_iterator i= options.begin(); i != options.end(); ++i ) {
        H3DEngineOptions* o= static_cast<H3DEngineOptions*>(*i);
        if ( o->getEngine() == engine ) {
          return o;
        }
      }

      return NULL;
    }

  protected:
    virtual void onAdd ( Node* n ) {
      TypedMFNode< H3DEngineOptions >::onAdd ( n );
      NodeType* node= static_cast<NodeType*>(getOwner());
      H3DEngineOptions* options= static_cast<H3DEngineOptions*>(n);
      options->valueUpdater->route ( node->valueUpdater );
    }

    virtual void onRemove ( Node* n ) {
      NodeType* node= static_cast<NodeType*>(getOwner());
      H3DEngineOptions* options= static_cast<H3DEngineOptions*>(n);
      options->valueUpdater->unroute ( node->valueUpdater );
      TypedMFNode< H3DEngineOptions >::onRemove ( n );
    }
  };

}
#endif
