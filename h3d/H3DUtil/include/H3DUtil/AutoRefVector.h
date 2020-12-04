//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3DUtil.
//
//    H3DUtil is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3DUtil is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3DUtil; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file AutoRefVector.h
/// Header file for AutoRefVector class.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __AUTOREFVECTOR_H__
#define __AUTOREFVECTOR_H__

#include <H3DUtil/H3DUtil.h>
#include <vector>
#include <algorithm>

#include <H3DUtil/AutoRef.h>

#ifdef H3D_REFERENCE_COUNT_DEBUG
#include <set>
#include <string>
#include <H3DUtil/RefCountedClass.h>
#define AUTOREFVECTOR_DEBUG_NAME(ref_ptr, debug_name) (ref_ptr).name = debug_name;
#else
#define AUTOREFVECTOR_DEBUG_NAME(ref_ptr, name) 
#endif


namespace H3DUtil {

#ifdef H3D_REFERENCE_COUNT_DEBUG
/// Base class that adds functionality needed for reference count
/// debugging.
class H3DUTIL_API AutoRefVectorBase {
  public:
    AutoRefVectorBase() {
      name = "Unknown";
      auto_ref_vectors_lock.lock();
      auto_ref_vectors.insert( this );
      auto_ref_vectors_lock.unlock();    }

    virtual ~AutoRefVectorBase() {
      auto_ref_vectors_lock.lock();
      auto_ref_vectors.erase( this );
      auto_ref_vectors_lock.unlock();
    }
    
    /// Adds the objects encapsulated in this instance to the given vector.
    /// Warning: reference count will not go up for the objects.
    virtual void getContent( std::vector< RefCountedClass *>&/*nodes*/) {}

    // lock for use for access to the auto_ref_vectors member.
    static MutexLock auto_ref_vectors_lock;
    
    // set of all AutoRefVectorBase instances created. All access to this
    // member must be using the auto_ref_vectors_lock member to gain access.
    static std::set< AutoRefVectorBase * > auto_ref_vectors;

    // Name of this AutoRefVector instance used in debugging messages.
    std::string name;
  };
#endif

  /// This class is similar to the AutoRef class in the vector elements
  /// are RefCountedClass * or pointers to subclasses of RefCountedClass. Reference counting 
  /// will be upheld on all RefCountedClasses in the vector.
  /// 
  template< class RefCountedClassType >
  class AutoRefVector : private std::vector<RefCountedClassType*>
#ifdef H3D_REFERENCE_COUNT_DEBUG
    ,public AutoRefVectorBase
#endif
  {
  public:
    /// The type of the RefCountedClassType, stored in the vector.
    typedef typename std::vector<RefCountedClassType*>::value_type value_type;
    /// Pointer to RefCountedClassType.
    typedef typename std::vector<RefCountedClassType*>::pointer pointer;
    /// Const reference to RefCountedClassType.
    typedef typename std::vector<RefCountedClassType*>::const_reference const_reference;
    /// An unsigned integral type.
    typedef typename std::vector<RefCountedClassType*>::size_type size_type;
    /// A signed integral type.
    typedef typename std::vector<RefCountedClassType*>::difference_type difference_type; 
    /// Const iterator used to iterate through a vector.
    typedef typename std::vector<RefCountedClassType*>::const_iterator const_iterator;
    /// Iterator used to iterate backwards through a vector.
    typedef typename std::vector<RefCountedClassType*>::const_reverse_iterator 
    const_reverse_iterator;

    /// Creates an empty vector.
    inline AutoRefVector() {}

    /// Copy constructor from a vector class.
    inline AutoRefVector( const std::vector<RefCountedClassType *> &v ) :
      std::vector<RefCountedClassType*>( v ) {
      refAll();
    }

    /// Copy constructor
    inline AutoRefVector( const AutoRefVector<RefCountedClassType> &v ) :
      std::vector<RefCountedClassType*>( v ) {
#ifdef H3D_REFERENCE_COUNT_DEBUG
      name = "Copy of " + v.name;
#endif
      refAll();
    }

    /// Creates a vector with n elements.
    inline AutoRefVector( size_type n ):
      std::vector< RefCountedClassType * >( n ) {}

    /// Destructor.
    inline virtual ~AutoRefVector() {
      clear();
    }

    /// Assignement operator.
    inline AutoRefVector<RefCountedClassType> 
    &operator=( const AutoRefVector<RefCountedClassType> &v ) {
      if( this != &v ) {
        unrefAll();      
        std::vector<RefCountedClassType*>::operator=( v );
        refAll();
      }
      return *this;
    }

    /// Assignement operator.
    inline AutoRefVector<RefCountedClassType> &operator=(
                                       const std::vector<RefCountedClassType *> &v ) {
      // temporarily add an extra reference to the refcounted class in v so
      // they are not accidentally removed in unrefAll()
      for( typename std::vector< RefCountedClassType * >::const_iterator i = v.begin();
           i != v.end();
           ++i ) 
        if(*i) (*i)->ref();
      unrefAll();
      std::vector<RefCountedClassType*>::operator=( v );
      refAll();

      // remove the temporary references.
      for( typename std::vector< RefCountedClassType * >::const_iterator i = v.begin();
           i != v.end();
           ++i ) 
        if(*i) (*i)->unref();
      return *this;
    }

    /// Returns a const_iterator pointing to the beginning of the vector.
    inline const_iterator begin() const { 
      return std::vector<RefCountedClassType*>::begin();
    }
      
    /// Returns a const_iterator pointing to the end of the vector.
    inline const_iterator end() const { return std::vector<RefCountedClassType*>::end(); }

        
    /// Returns a const_reverse_iterator pointing to the beginning of the
    /// reversed vector.
    inline const_reverse_iterator rbegin() const { 
      return std::vector<RefCountedClassType*>::rbegin();
    }
      
    /// Returns a const_reverse_iterator pointing to the end of the reversed 
    /// vector.
    inline const_reverse_iterator rend() const { 
      return std::vector<RefCountedClassType*>::rend(); 
    }

    /// Returns the size of the vector.
    inline size_type size() const { 
      return std::vector<RefCountedClassType*>::size(); 
    }

    /// Returns the largest possible size of the vector.
    inline size_type max_size() const {
      return std::vector<RefCountedClassType*>::max_size();
    }
        
    /// Number of elements for which memory has been allocated. capacity() 
    /// is always greater than or equal to size().
    inline size_type capacity() const { 
      return std::vector<RefCountedClassType*>::capacity(); 
    }
        
    /// Swaps the contents of two vectors.
    inline void swap( AutoRefVector<RefCountedClassType> &x ) {
      std::vector<RefCountedClassType*>::swap( x );
    }

    /// Swaps the contents of two vectors.
    inline void swap( std::vector<RefCountedClassType*> &x ) {
      unrefAll();
      std::vector<RefCountedClassType*>::swap( x );
      refAll();
    }
        
    /// A request for allocation of additional memory. If s is less than
    /// or equal to capacity(), this call has no effect. 
    /// Otherwise, it is a request for allocation of additional memory. 
    /// If the request is successful, then capacity() is greater than or 
    /// equal to s; otherwise, capacity() is unchanged. In either case, 
    /// size() is unchanged.
    /// 
    inline void reserve( size_t s ) { std::vector<RefCountedClassType*>::reserve( s ); }

    /// Inserts or erases elements at the end such that the size becomes n.
    /// Note: when n is bigger than original size, the newly added value will
    /// point to the same location, unless you want that, otherwise do not specify
    /// value t (will use NULL then) and always set the value manually after resized.
    /// It is expected that t has a reference of at least 1 when this function is called.
    inline virtual void resize( size_t n, RefCountedClassType * t = NULL ) {
      if( size() > n ) {
        for( size_t i = n; i < size(); ++i )
          unref( std::vector<RefCountedClassType*>::operator[]( i ) );
      }
      if( size() < n ) {
        for( size_t j = 0; j < n-size(); j++ ) {
          ref( t );
        }
      }
      std::vector<RefCountedClassType*>::resize( n, t );
    }

    /// Inserts or erases elements at the end such that the size becomes n.
    /// Note: when n is bigger than original size, the newly added value will
    /// point to the same location.
    void resize( size_t n, AutoRef< RefCountedClassType > t ) {
      if( size() > n ) {
        for( size_t i = n; i < size(); ++i )
          unref( std::vector<RefCountedClassType*>::operator[]( i ) );
      }
      if( size() < n ) {
        for( size_t j = 0; j < n - size(); j++ ) {
          ref( t.get() );
        }
      }
      std::vector<RefCountedClassType*>::resize( n, t.get() );
    }

    /// true if the vector's size is 0.
    inline bool empty() const { return std::vector<RefCountedClassType*>::empty(); }

    /// Returns the n'th element. We return a const_reference so that
    /// the values of the vector only can be changed using member 
    /// functions. To change the value of a specific index use
    /// the set( index, value ) function.
    inline const_reference operator[](size_type n) const {
      return std::vector<RefCountedClassType*>::operator[]( n );
    }

    /// Set value at index i to v.
    inline void set( size_type i, const value_type &v ) {
      if( v != std::vector<RefCountedClassType*>::operator[]( i ) ) {
        unref( std::vector<RefCountedClassType*>::operator[]( i ) );
        ref( v );
        std::vector<RefCountedClassType*>::operator[]( i ) = v;
      }
    }

    /// Returns the first element.
    inline const_reference front() const { return std::vector<RefCountedClassType*>::front();}

    /// Returns the last element.
    inline const_reference back() const { return std::vector<RefCountedClassType*>::back(); }

    /// Inserts a new element at the end.
    inline void push_back( const value_type &x ) {
      ref( x );
      std::vector< RefCountedClassType * >::push_back( x );
    }

    /// Removed the last element.
    void pop_back() {
      unref( back() );
      std::vector< RefCountedClassType * >::pop_back();
    }
        
    /// Erases all of the elements.
    inline void clear() {
      unrefAll();
      std::vector<RefCountedClassType*>::clear();
    }

    /// Erase the first element equal to a.
    inline virtual void erase( RefCountedClassType *a ) {
      typename std::vector<RefCountedClassType * >::iterator i = 
        std::find( std::vector<RefCountedClassType*>::begin(), 
                   std::vector<RefCountedClassType*>::end(), 
                   a );
      if( i != end() ) {
        unref( *i );
        std::vector<RefCountedClassType*>::erase( i );
      } 
    }

    /// Insert an element before the index given by pos.
    inline virtual void insert(unsigned int pos,
                               const value_type & x) {
      ref( x );
      std::vector< RefCountedClassType*>::insert( std::vector< RefCountedClassType*>::begin() + pos, x );
    }

    /// Removes the element at the index pos.
    inline virtual void erase(unsigned int pos ) {
      // nop if pos is outside range.
      if( pos >= size() ) return;

      unref( std::vector<RefCountedClassType*>::operator[]( pos ) );
      std::vector< RefCountedClassType*>::erase( std::vector< RefCountedClassType*>::begin() + pos );
    }

#ifdef H3D_REFERENCE_COUNT_DEBUG
    virtual void getContent( std::vector< RefCountedClass *>&r) {
      r.insert( r.end(), begin(), end() );
    }
#endif

  protected:
    /// Virtual function that is called when a RefCountedClassType is added to 
    /// the vector.
    inline virtual void ref( RefCountedClassType *n ) const {
      if( n ) {
        n->ref();
      }
    }

    /// Virtual function that is called when a RefCountedClassType is removed from
    /// the vector.
    inline virtual void unref( RefCountedClassType *n ) const {
      if( n ) {
        n->unref();
      }
    }

    /// Call ref () on all values in the vector.
    inline void refAll() const {
      for( const_iterator i = std::vector<RefCountedClassType*>::begin(); 
           i != std::vector<RefCountedClassType*>::end(); ++i ) 
        ref( *i );
    }

    /// Call unref () on all values in the vector.
    inline void unrefAll() const {
      for( const_iterator i = std::vector<RefCountedClassType*>::begin(); 
           i != std::vector<RefCountedClassType*>::end(); ++i ) 
        unref( *i );
    }
  };
}
    
#endif
