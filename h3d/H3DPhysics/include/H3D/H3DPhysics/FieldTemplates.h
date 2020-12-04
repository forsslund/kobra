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
/// \file H3DPhysics/FieldTemplates.h
/// \brief Additional field templates for physics implementation
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FIELDTEMPLATES_RBP__
#define __FIELDTEMPLATES_RBP__
#include <H3D/SFNode.h>
#include <H3D/MFNode.h>
#include <H3D/H3DPhysics/PythonMethods.h>

namespace H3D {

  H3D_VALUE_EXCEPTION( string, InvalidEnumValue );

  /// This template allows a TypedSFNode field of a node to have its type
  /// restriction further specialized by a subclass node.
  ///
  /// e.g. In the Cloth node the geometry field must be an IndexedTriangleSet
  /// But in the SoftBody node that field must be of type IndexedTetraSet
  template < class SpecializationT, class ParentT >
  class SpecializedSFNode : public ParentT {
    virtual void onAdd ( Node* n ) {
      if( !dynamic_cast< SpecializationT * >( n ) ) {
        Node *pi = this->getPrototypeNode( n );
        if( !dynamic_cast< SpecializationT * >( pi ) ) {
          stringstream s;
          s << "Expecting " << typeid( SpecializationT ).name();
          throw InvalidNodeType( n->getTypeName(),
                                 s.str(),
                                 H3D_FULL_LOCATION );
        }
      }
      ParentT::onAdd( n );
    }
  };

  /// This template allows a TypedMFNode field of a node to have its type
  /// restriction further specialized by a subclass node.
  ///
  /// e.g. In the Cloth node the geometry field must be an IndexedTriangleSet
  /// But in the SoftBody node that field must be of type IndexedTetraSet
  template < class SpecializationT, class ParentT >
  class SpecializedMFNode : public ParentT {
    virtual void onAdd ( Node* n ) {
      if( !dynamic_cast< SpecializationT * >( n ) ) {
        Node *pi = this->getPrototypeNode( n );
        if( !dynamic_cast< SpecializationT * >( pi ) ) {
          stringstream s;
          s << "Expecting " << typeid( SpecializationT ).name();
          throw InvalidNodeType( n->getTypeName(),
                                 s.str(),
                                 H3D_FULL_LOCATION );
        }
      }
      ParentT::onAdd( n );
    }
  };

  /// This field allows the removal of a specialization from a field previously specialized
  /// with SpecializedMFNode. 
  ///
  /// e.g. This is used by PhysicsBodyCollection to make the joints field take any kind of 
  /// H3DBodyConstraintNode, rather than only H3DJointNode.
  template < class SpecializationT, class ParentT, class BaseT >
  class GeneralizedMFNode : public ParentT {
    virtual void onAdd ( Node* n ) {
      if( !dynamic_cast< SpecializationT * >( n ) ) {
        Node *pi = this->getPrototypeNode( n );
        if( !dynamic_cast< SpecializationT * >( pi ) ) {
          stringstream s;
          s << "Expecting " << typeid( SpecializationT ).name();
          throw InvalidNodeType( n->getTypeName(),
                                 s.str(),
                                 H3D_FULL_LOCATION );
        }
      }
      BaseT::onAdd( n );
    }
  };

  /// MField template used to check for invalid values in an MField used as an enumeration.
  /// e.g. collisionOptions, where possible values are "CLUSTER_RIGIDSOFT", "SDF_RIGIDSOFT" etc.
  /// Check current values by calling validate(). An exception is raised if an invalid value is found.
  template < class MFieldType >
  class EnumMField : public MFieldType {
  public:

    void addValidValue ( typename MFieldType::value_type _value ) {
      this->validValues.push_back ( _value );
    }

    /// Check current values, an exception is raised if an invalid value is found.
    void validate () {
      for ( typename MFieldType::vector_type::iterator i= this->value.begin(); i != this->value.end(); ++i ) {
        if ( find ( this->validValues.begin(), this->validValues.end(), *i ) == this->validValues.end() ) {
          stringstream ss;
          ss << "Invalid enum value: " << *i << ". Must be one of: ";
          for ( typename MFieldType::vector_type::iterator j= this->validValues.begin(); j != this->validValues.end(); ++j ) {
            ss << *j << " ";
          }
          throw InvalidEnumValue( this->getFullName(),
                                 ss.str(),
                                 H3D_FULL_LOCATION );
        }
      }
    }

  protected:
    typename MFieldType::vector_type validValues;
  };

  /// An base class for TrackedMField which provides an
  /// interface to access changes made to the field value.
  template < class BaseMFieldType >
  class TrackedMFieldBase : public BaseMFieldType {
  public:
    typedef std::vector<H3DInt32> IntVector;

    /// Types of changes to a vector that can be tracked
    enum EditType { Edit_Insert, Edit_Update, Edit_Erase };

    /// Represents a tracked change to the vector
    struct Edit {

      /// Construct an insert, erase or update edit
      Edit ( 
        EditType _type,
        const IntVector& _index,
        const typename BaseMFieldType::vector_type& _value,
        size_t _size= 0 ) :
         type ( _type ), index ( _index ), value ( _value ), size ( _size ) {
        if ( size == 0 ) size= value.size();
      }

      /// Construct an edit for a consecutive range of indices
      Edit ( 
        EditType _type,
        size_t _index,
        const typename BaseMFieldType::vector_type& _value,
        size_t _size= 0 ) :
         type ( _type ), value ( _value ), size ( _size ) {
        if ( size == 0 ) size= value.size();
        index.resize ( size );
        for ( size_t i= 0; i < index.size(); ++i ) {
          index[i]= static_cast<int>(_index+i);
        }
      }

      /// The type of edit
      EditType type;

      /// Indices affected
      IntVector index;

      /// Number of elements affected
      size_t size;

      /// The new values (update, insert only)
      typename BaseMFieldType::vector_type value;
    };

    /// A list of edits
    typedef std::vector < Edit > EditVector;

    /// Constructor
    TrackedMFieldBase () {
      // Initialize the H3DPhysics python module
      // This provides a Python interface to this tracked field type
      H3DPhysicsPythonInterface::getInstance();
    }

    /// Provide a name for the field type
    ///
    /// The Python module uses this to create the Python version of the class
    virtual string getTypeName() { return "Tracked" + BaseMFieldType::getTypeName(); }

    // Get a list of changes make to the vector since the last call to resetTracking()
    EditVector getEdits () {
      return edits;
    }

    // Empty the list of changes
    void resetTracking () {
      edits.clear();
    }

    /// Returns true if any changes have been tracked since
    /// the last call to resetTracking ()
    bool haveTrackedChanges () {
      return !edits.empty();
    }

  protected:
    /// List of edits to the vector since last resetTracking()
    EditVector edits;
  };

  /// A template which adds tracking to an MFField. With tracking the field knows
  /// what values have been inserted, updated or erased since the last call to resetTracking().
  ///
  /// Using this information a physics engine implementation can optimize the way it handles
  /// changes in the field value. For example, if only one new link in a SoftBodyAttachment is
  /// added, there is no need to rebuild all other links.
  ///
  /// In order to track changes to the field value, the caller must use the *Tracked() functions
  /// to edit the field. Changes are still possible using the standard MFField functions, but will
  /// not be recorded and will trigger a full update.
  ///
  /// *Tracked() functions should not be combined with standard MFField modify function in the same
  /// scene-graph loop. This will cause incorrect results in the physics engine.
  /// 
  template < class BaseMFieldType >
  class TrackedMField : public TrackedMFieldBase < BaseMFieldType > {
  public:

    /// Inserts one or more new value(s) into the field at specified location(s) and remembers the change
    ///
    /// \param indices The indices after which the new values will be inserted. Each index in the list
    ///                corresponds to the value in the same place in the values list. The indices are into the
    ///                updated array, so each new index must take into account insertions that have already
    ///                happened since the last call to resetTracking(), including those made in this call.
    /// \param values  The values to insert into the field.
    /// \param id Id of the owner of the field (a node).
    void insertTracked ( const typename TrackedMFieldBase<BaseMFieldType>::IntVector& indices, const typename BaseMFieldType::vector_type& values, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();

      size_t index;
      for ( size_t i= 0; i < indices.size() && i < values.size(); ++i ) {
        index= indices[i];
        if ( /*index >= 0 &&*/ index < this->value.size() ) {
          this->value.insert( this->value.begin()+index, values[i] );
        }
      }

      this->startEvent();

      // Remember the edit
      this->edits.push_back ( 
        typename TrackedMFieldBase<BaseMFieldType>::Edit ( TrackedMFieldBase<BaseMFieldType>::Edit_Insert, indices, values ) );
    }

    /// Insert one or more value(s) at a specific position in the vector and remember this change
    ///
    /// \param pos Values will be inserted so that the first new value will have index pos
    /// \param values The value(s) to insert
    /// \param id Id of the owner of the field (a node).
    void insertRangeTracked ( typename BaseMFieldType::const_iterator pos, const typename BaseMFieldType::vector_type& values, int id = 0 ) {
      size_t start= pos-this->begin();
      
      // Remember the edit
      this->edits.push_back ( 
        typename TrackedMFieldBase<BaseMFieldType>::Edit ( TrackedMFieldBase<BaseMFieldType>::Edit_Insert, start, values ) );
      
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();
      this->value.insert( this->value.begin()+start, values.begin(), values.end() );
      this->startEvent();
    }
  
    /// Assign new values for a consecutive range of values in the vector and remember this change
    ///
    /// \param pos Iterator pointing to the first value in the vector to update
    /// \param values List of new values to assign
    /// \param id Id of the owner of the field (a node).
    void updateRangeTracked ( typename BaseMFieldType::const_iterator pos, const typename BaseMFieldType::vector_type& values, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();
      size_t start= pos-this->value.begin();
      for ( size_t i= 0; i < values.size() && start+i < this->value.size(); ++i ) {
        this->value[start+i]= values[i];
      }
      this->startEvent();

      // Remember the edit
      this->edits.push_back ( 
        typename TrackedMFieldBase<BaseMFieldType>::Edit ( TrackedMFieldBase<BaseMFieldType>::Edit_Update, start, values ) );
    }

    /// Updates one or more values in the field and remembers the change
    ///
    /// \param indices The indices of values to update. Each index in the list
    ///                corresponds to the value in the same place in the values list.
    /// \param values  The new values to assign to the indexed elements.
    /// \param id Id of the owner of the field (a node).
    void updateTracked ( const typename TrackedMFieldBase<BaseMFieldType>::IntVector& indices, const typename BaseMFieldType::vector_type& values, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();

      size_t index;
      for ( size_t i= 0; i < indices.size() && i < values.size(); ++i ) {
        index= indices[i];
        if ( /*index >= 0 && */index < this->value.size() ) {
          this->value[index]= values[i];
        }
      }

      this->startEvent();

      // Remember the edit
      this->edits.push_back ( 
        typename TrackedMFieldBase<BaseMFieldType>::Edit ( TrackedMFieldBase<BaseMFieldType>::Edit_Update, indices, values ) );
    }

    /// Remove a range of values from the vector and remember this change
    ///
    /// \param pos Iterator pointing to the first value in the vector to remove
    /// \param size The number of values to remove
    /// \param id Id of the owner of the field (a node).
    void eraseRangeTracked ( typename BaseMFieldType::const_iterator pos, size_t size, int id = 0 ) {
      size_t start= pos-this->begin();

      // Remember the edit
      this->edits.push_back ( 
        typename TrackedMFieldBase<BaseMFieldType>::Edit ( TrackedMFieldBase<BaseMFieldType>::Edit_Erase, start, typename BaseMFieldType::vector_type(), size ) );

      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();
      this->value.erase ( this->value.begin()+start, this->value.begin()+start+size );
      this->startEvent();
    }
    
    /// Erase one or more values from the field and remember this change
    ///
    /// \param indices The indices of elements to erase. The indices are into the
    ///                state of the array before this call is made. I.e., the indices
    ///                supplied should not take into account elements removed during
    ///                this call.
    /// \param id Id of the owner of the field (a node).
    void eraseTracked ( const typename TrackedMFieldBase<BaseMFieldType>::IntVector& indices, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();

      typename TrackedMFieldBase<BaseMFieldType>::IntVector sorted_indices ( indices );
      std::sort ( sorted_indices.begin(), sorted_indices.end() );
      size_t index;
      size_t nrRemoved= 0;
      for ( size_t i= 0; i < sorted_indices.size(); ++i ) {
        index= sorted_indices[i]-nrRemoved;
        if ( /*index >= 0 && */index < this->value.size() ) {
          this->value.erase ( this->value.begin()+index );
          ++nrRemoved;
        }
      }

      this->startEvent();

      // Remember the edit
      this->edits.push_back ( 
        typename TrackedMFieldBase<BaseMFieldType>::Edit ( TrackedMFieldBase<BaseMFieldType>::Edit_Erase, indices, typename BaseMFieldType::vector_type() ) );
    }

    // Some convenience functions for singular insert, update and delete operations
    
    /// Insert a single value into the vector at the specified location and remember the change.
    /// \param pos position of insertion.
    /// \param _value Value to insert.
    /// \param id Id of the owner of the field (a node).
    void insertTracked ( typename BaseMFieldType::const_iterator pos, const typename BaseMFieldType::value_type& _value, int id = 0 ) {
      insertRangeTracked ( pos, typename BaseMFieldType::vector_type ( 1, _value ), id );
    }

    /// Append a single value to the end of the vector and remember the change
    /// \param _value Value to insert.
    /// \param id Id of the owner of the field (a node).
    void pushBackTracked ( const typename BaseMFieldType::value_type& _value, int id = 0 ) {
      insertRangeTracked ( this->value.end(), typename BaseMFieldType::vector_type ( 1, _value ), id );
    }

    /// Update a single value in the vector and remember the change
    /// \param pos position of insertion.
    /// \param _value Value to insert.
    /// \param id Id of the owner of the field (a node).
    void updateTracked ( typename BaseMFieldType::const_iterator pos, const typename BaseMFieldType::value_type& _value, int id = 0 ) {
      updateRangeTracked ( pos, typename BaseMFieldType::vector_type ( 1, _value ), id );
    }

    /// Delete a single value from the vector and remember the change
    /// \param pos position of deletion.
    /// \param id Id of the owner of the field (a node).
    void eraseTracked ( typename BaseMFieldType::const_iterator pos, int id = 0 ) {
      eraseRangeTracked ( pos, 1, id );
    }
  };

  /// A specialization of the TrackedMField template for MFNode
  ///
  /// This is required because the MFNode interface is different from
  /// other MFField classes.
  ///
  template <>
  class TrackedMField < MFNode >: public TrackedMFieldBase < MFNode > {
  public:

    /// Inserts one or more new value(s) into the field at specified location(s) and remembers the change
    ///
    /// \param indices The indices after which the new values will be inserted. Each index in the list
    ///                corresponds to the value in the same place in the values list. The indices are into the
    ///                updated array, so each new index must take into account insertions that have already
    ///                happened since the last call to resetTracking(), including those made in this call.
    /// \param values  The values to insert into the field.
    /// \param id Id of the owner of the field (a node).
    void insertTracked ( const IntVector& indices, const std::vector<Node*>& values, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();

      size_t index;
      for ( size_t i= 0; i < indices.size() && i < values.size(); ++i ) {
        index= indices[i];
        if ( /*index >= 0 && */index < this->value.size() ) {
          this->value.insert( static_cast<unsigned int>(index), values[i] );
        }
      }

      this->startEvent();

      // Remember the edit
      MFNode::vector_type temp;
      temp= values;
      this->edits.push_back ( 
        TrackedMFieldBase<MFNode>::Edit ( TrackedMFieldBase<MFNode>::Edit_Insert, indices, temp ) );
    }

    /// Insert one or more value(s) at a specific position in the vector and remember this change
    ///
    /// \param pos Values will be inserted so that the first new value will have index pos
    /// \param values The value(s) to insert
    /// \param id Id of the owner of the field (a node).
    void insertRangeTracked ( MFNode::const_iterator pos, const std::vector<Node*>& values, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();
      size_t start= pos-this->value.begin();
      for ( MFNode::vector_type::const_reverse_iterator i= values.rbegin(); i != values.rend(); ++i ) {
        this->value.insert( static_cast<unsigned int>(start), *i );
      }
      this->startEvent();
      
      // Remember the edit
      MFNode::vector_type temp;
      temp= values;
      this->edits.push_back ( 
        TrackedMFieldBase<MFNode>::Edit ( TrackedMFieldBase<MFNode>::Edit_Insert, start, temp ) );
    }
    
    /// Updates one or more values in the field and remembers the change
    ///
    /// \param indices The indices of values to update. Each index in the list
    ///                corresponds to the value in the same place in the values list.
    /// \param values  The new values to assign to the indexed elements.
    /// \param id Id of the owner of the field (a node).
    void updateTracked ( const IntVector& indices, const std::vector<Node*>& values, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();

      size_t index;
      for ( size_t i= 0; i < indices.size() && i < values.size(); ++i ) {
        index= indices[i];
        if ( /*index >= 0 && */index < this->value.size() ) {
          this->value.set ( index, values[i] );
        }
      }

      this->startEvent();

      // Remember the edit
      MFNode::vector_type temp;
      temp= values;
      this->edits.push_back ( 
        TrackedMFieldBase<MFNode>::Edit ( TrackedMFieldBase<MFNode>::Edit_Update, indices, temp ) );
    }

    /// Assign new values for a consecutive range of values in the vector and remember this change
    ///
    /// \param pos Iterator pointing to the first value in the vector to update
    /// \param values List of new values to assign
    /// \param id Id of the owner of the field (a node).
    void updateRangeTracked ( MFNode::const_iterator pos, const std::vector<Node*>& values, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();
      size_t start= pos-this->value.begin();
      for ( size_t i= 0; i < values.size() && start+i < this->value.size(); ++i ) {
        this->value.set ( start+i, values[i] );
      }
      this->startEvent();

      // Remember the edit
      MFNode::vector_type temp;
      temp= values;
      this->edits.push_back ( 
        TrackedMFieldBase<MFNode>::Edit ( TrackedMFieldBase<MFNode>::Edit_Update, start, temp ) );
    }

    /// Erase one or more values from the field and remember this change
    ///
    /// \param indices The indices of elements to erase. The indices are into the
    ///                state of the array before this call is made. I.e., the indices
    ///                supplied should not take into account elements removed during
    ///                this call.
    /// \param id Id of the owner of the field (a node).
    void eraseTracked ( const IntVector& indices, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();

      IntVector sorted_indices ( indices );
      std::sort ( sorted_indices.begin(), sorted_indices.end() );
      size_t index;
      size_t nrRemoved= 0;
      for ( size_t i= 0; i < sorted_indices.size(); ++i ) {
        index= sorted_indices[i]-nrRemoved;
        if ( /*index >= 0 && */index < this->value.size() ) {
          this->value.erase ( static_cast<unsigned int>(index) );
          ++nrRemoved;
        }
      }

      this->startEvent();

      // Remember the edit
      this->edits.push_back ( 
        TrackedMFieldBase<MFNode>::Edit ( TrackedMFieldBase<MFNode>::Edit_Erase, indices, MFNode::vector_type() ) );
    }

    /// Remove a range of values from the vector and remember this change
    ///
    /// \param pos Iterator pointing to the first value in the vector to remove
    /// \param size The number of values to remove
    /// \param id Id of the owner of the field (a node).
    void eraseRangeTracked ( MFNode::const_iterator pos, size_t size, int id = 0 ) {
      // Update the vector
      this->checkAccessTypeSet( id );
      this->upToDate();
      size_t start= pos-this->value.begin();
      for ( size_t i= 0; i < size; ++i ) {
        this->value.erase ( static_cast<unsigned int>(start) );
      }
      this->startEvent();

      // Remember the edit
      this->edits.push_back ( 
        TrackedMFieldBase<MFNode>::Edit ( TrackedMFieldBase<MFNode>::Edit_Erase, start, MFNode::vector_type(), size ) );
    }

    /// Insert a single value into the vector at the specified location and remember the change
    /// \param pos position of insertion.
    /// \param _value Value to insert.
    /// \param id Id of the owner of the field (a node).
    void insertTracked ( MFNode::const_iterator pos, const MFNode::value_type& _value, int id = 0 ) {
      this->insertRangeTracked ( pos, std::vector<Node*> ( 1, _value ), id );
    }

    /// Append a single value to the end of the vector and remember the change
    /// \param _value Value to insert.
    /// \param id Id of the owner of the field (a node).
    void pushBackTracked ( const MFNode::value_type& _value, int id = 0 ) {
      this->insertRangeTracked ( this->value.end(), std::vector<Node*> ( 1, _value ), id );
    }

    /// Update a single value in the vector and remember the change
    /// \param pos position of update.
    /// \param _value Value to insert.
    /// \param id Id of the owner of the field (a node).
    void updateTracked ( MFNode::const_iterator pos, const MFNode::value_type& _value, int id = 0 ) {
      this->updateRangeTracked ( pos, std::vector<Node*> ( 1, _value ), id );
    }

    /// Delete a single value from the vector and remember the change
    /// \param pos position of deletion.
    /// \param id Id of the owner of the field (a node).
    void eraseTracked ( MFNode::const_iterator pos, int id = 0 ) {
      this->eraseRangeTracked ( pos, 1, id );
    }
  };
}
#endif
