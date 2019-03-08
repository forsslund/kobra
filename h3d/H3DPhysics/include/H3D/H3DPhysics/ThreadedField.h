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
/// \file H3DPhysics/ThreadedField.h
/// \brief Threaded field types
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __THREADEDFIELD_RBP__
#define __THREADEDFIELD_RBP__
#include <H3D/PeriodicUpdate.h>
#include <H3D/MField.h>

namespace H3D {
  
  /// An abstract base field template which allows field type to be used in non-main threads
  /// The field value can be set, and accessed from a non-main thread. Simplified
  /// routing between multiple fields of this type is supported.
  ///
  /// The following cases are supported:
  ///  * Setting the field value from any thread, using setValue()
  ///  * Routing FROM the field to any other field
  ///  * Routing TO the field from any other field
  ///  * Getting the field value from the main thread using getValue()
  ///  * Getting the field value from any thread using getValueRT()
  ///

  template < class BaseField, typename ts_value_type >
  class ThreadedFieldBase : public PeriodicUpdate < BaseField > {
  public:
    typedef PeriodicUpdate< BaseField > TField;

    ThreadedFieldBase():
      ts_route_in( NULL ),
      ts_event_pending( false ),
      update_value_from_base( false ) {
    }

    virtual ts_value_type getValueRT() {
      this->mutex.lock();
      ts_value_type tmp_value = this->ts_value;
      ThreadedFieldBase< BaseField, ts_value_type >* tmp_route_in = this->ts_route_in;
      this->mutex.unlock();

      if( tmp_route_in ) {
        return tmp_route_in->getValueRT();
      }
      return tmp_value;
    }

    virtual void setValue( const ts_value_type& _value, int _id = 0 ) {
      bool in_main = H3DUtil::ThreadBase::inMainThread();
      if( in_main ) {
        TField::setValue( _value, _id );
      } else {
        this->ts_event_pending = true;
      }
      this->mutex.lock();
      this->ts_value = _value;
      this->mutex.unlock();
    }

    virtual void upToDate() {
      assert( H3DUtil::ThreadBase::inMainThread() );
      TField::upToDate();

      this->mutex.lock();
      // update value from the base field
      // ignores events from thread
      if( update_value_from_base ) {
        this->ts_value = TField::value;
        update_value_from_base = false;
      }
      // update value from the thread
      bool tmp_ts_event_pending = this->ts_event_pending;
      this->ts_event_pending = false;
      ts_value_type tmp_value = this->ts_value;

      this->mutex.unlock();

      // If the field value was set from non-main thread, then trigger
      // an event to update values in the main thread
      if( tmp_ts_event_pending ) {
        TField::value = tmp_value;
        // reset the event pointer since we want to ignore any pending
        // events when the field is set to a new value.
        this->event.ptr = NULL;
        // generate an event.
        this->startEvent();
      }
    }

    //This is to update the value of the field from a non threaded field
    // behaviour similar to auto update of a non threaded field
    virtual void propagateEvent( Field::Event e ) {
      TField::propagateEvent( e );

      // NOTE: prevent double-propagation from occuring, in case the route in has a python
      // update() override, which will call setValue() internally
      if( !update_value_from_base ) {
        update_value_from_base = true;
        this->upToDate();
      }
    }

  protected:

    virtual void routeFrom( Field* f, int id = 0 ) {
      assert( H3DUtil::ThreadBase::inMainThread() );

      TField::routeFrom( f, id );

      if( ThreadedFieldBase< BaseField, ts_value_type >* tf = dynamic_cast < ThreadedFieldBase< BaseField, ts_value_type >* > (f) ) {
        this->mutex.lock();
        this->ts_route_in = tf;
        this->mutex.unlock();
      }
    }

    virtual void unrouteFrom( Field* f ) {
      assert( H3DUtil::ThreadBase::inMainThread() );

      TField::unrouteFrom( f );

      this->mutex.lock();
      if( f == this->ts_route_in ) {
        this->ts_route_in = NULL;
      }
      this->mutex.unlock();
    }

    MutexLock mutex;
    ts_value_type ts_value;
    ThreadedFieldBase< BaseField, ts_value_type >* ts_route_in;
    bool ts_event_pending;
    bool update_value_from_base;
  };

  /// A field template which allows an SFField type to be used in non-main threads
  template < class BaseField >
  class ThreadedField : public ThreadedFieldBase < BaseField , typename BaseField::value_type  > {
  public:
    ThreadedField() {}
  };

  /// A field template which allows an MFField type to be used in non-main threads
  /// implements valuebyindex get/set functions
  template< class BaseField >
  class ThreadedMField : public ThreadedFieldBase< BaseField, typename BaseField::vector_type > {
  public:
    typedef ThreadedFieldBase< BaseField, typename BaseField::vector_type > TField;

    ThreadedMField() {}

    virtual typename TField::value_type getValueByIndexRT( typename TField::size_type i ) {
      this->mutex.lock();
      typename BaseField::vector_type tmp_value = this->ts_value;
      ThreadedMField< BaseField >* tmp_route_in = dynamic_cast <ThreadedMField< BaseField >*>( this->ts_route_in );
      this->mutex.unlock();

      if( tmp_route_in ) {
        return tmp_route_in->getValueByIndexRT( i );
      }

      // if the size is greater throw a warning
      if( i >= tmp_value.size() ){
        H3DUtil::Console(LogLevel::Error) << " ThreadedMField trying to access value out of bounds" << std::endl;
        return typename TField::value_type();
      }
      return tmp_value[i];
    }

    virtual void setValue( const typename BaseField::vector_type& _value, int _id = 0 ) {
      TField::setValue( _value, _id);
    }

    virtual void setValue( typename TField::size_type i, const typename TField::value_type& val, int id = 0 ) {
      bool in_main= H3DUtil::ThreadBase::inMainThread();
      if ( in_main ) {
        BaseField::setValue ( i, val, id );
      } else {
        this->ts_event_pending= true;
      }

      this->mutex.lock();
      if ( i < this->ts_value.size() ) {
        this->ts_value[i] = val;
      }
      else {
        H3DUtil::Console(LogLevel::Error) << " ThreadedMField trying to set value greater than size" << std::endl;
        this->ts_event_pending = false;
      }
      this->mutex.unlock();
    }
  };

}
#endif
