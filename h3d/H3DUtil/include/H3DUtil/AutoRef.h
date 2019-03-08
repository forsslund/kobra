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
/// \file AutoRef.h
/// Header file for AutoRef class.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __AUTOREF_H__
#define __AUTOREF_H__
#include <stddef.h>
#include <set>

#include <H3DUtil/H3DUtil.h>
#include <H3DUtil/RefCountedClass.h>

//#include <boost/stacktrace.hpp>

#ifdef H3D_REFERENCE_COUNT_DEBUG
#define AUTOREF_DEBUG_NAME(ref_ptr, debug_name) (ref_ptr).name = debug_name;
#else
#define AUTOREF_DEBUG_NAME(ref_ptr, name) 
#endif

namespace H3DUtil {
    
#ifdef H3D_REFERENCE_COUNT_DEBUG
  /// Base class that adds functionallity needed for reference count
  /// debugging.
  class H3DUTIL_API AutoRefBase {
  public:
    AutoRefBase() {
      // for debugging when tracing down where an Unknown AutoRef is located
      //std::stringstream s;
      //s << boost::stacktrace::stacktrace();
      //name = s.str(); 
      name = "Unknown";
      auto_refs_lock.lock();
      auto_refs.insert( this );
      auto_refs_lock.unlock();
    }

    virtual ~AutoRefBase() {
      auto_refs_lock.lock();
      auto_refs.erase( this );
      auto_refs_lock.unlock();
    }

    /// Returns the encapsulated object. Warning: reference count will not go up for the object.
    virtual H3DUtil::RefCountedClass *getContent() {
      return NULL;
    }

    // lock for use for access to the auto_refs member.
    static MutexLock auto_refs_lock;

    // set of all AutoRefBase instances created. All access to this
    // member must be using the auto_refs_lock member to gain access.
    static std::set< AutoRefBase * > auto_refs;

    // Name of this AutoRef instance used in debugging messages.
    std::string name;
  };
#endif
  
  /// The AutoRef class is similar to the auto_ptr class, but it requires
  /// that the pointer to RefCountedClass or a subclass of RefCountedClass. It will keep a 
  /// reference to the RefCountedClass pointer it encapsulates, i.e. ref() will be
  /// called. When destructed the AutoRef does not delete the pointer, 
  /// but instead unreferences the RefCountedClass (if this causes the reference count
  /// for the RefCountedClass to go down to 0 then it will be deleted though).
  ///
  template < class RefCountedClassType> 
  class AutoRef 
#ifdef H3D_REFERENCE_COUNT_DEBUG
  : public AutoRefBase
#endif
  {
  public: 
    /// The type of the encapsulated RefCountedClass.
    typedef RefCountedClassType element_type; 
        
    /// Constructor.   
    explicit AutoRef (RefCountedClassType* n = NULL) throw() :
      ref_counted_class_ptr( n ) {
      ref( ref_counted_class_ptr );
    }

    /// Copy constructor.
    AutoRef(const AutoRef<RefCountedClassType>&ar) throw () :
      ref_counted_class_ptr( NULL ){
#ifdef H3D_REFERENCE_COUNT_DEBUG
      name = "Copy of " + ar.name;
#endif
      reset( ar.get() );
    }
            
    /// Copy constructor from other type of AutoRef.
    template <class Y> 
    AutoRef(const AutoRef<Y>&ar) throw() :
      ref_counted_class_ptr( NULL ){
#ifdef H3D_REFERENCE_COUNT_DEBUG
      name = "Copy of " + ar.name;
#endif
      reset( ar.get() );
    }
        
    /// Assignment operator.
    AutoRef<RefCountedClassType>& operator=(const AutoRef<RefCountedClassType>&ar) throw() {
      reset( ar.ref_counted_class_ptr );
      return *this;
    }

    /// Assignment operator for other type of AutoRef.
    template <class Y> 
    AutoRef<RefCountedClassType>& operator= (const AutoRef<Y>& ar) throw() {
      reset( ar.ref_counted_class_ptr );
      return *this; 
    } 
    virtual ~AutoRef() throw() {
      unref( ref_counted_class_ptr );
    } 
        
    /// Returns what the encapsulated RefCountedClass * points to.  
    RefCountedClassType& operator* () const throw() {
      return *ref_counted_class_ptr;
    } 

    /// Returns the encapsulated RefCountedClass pointer.
    RefCountedClassType* operator-> () const throw() {
      return ref_counted_class_ptr;
    } 

    /// Returns the encapsulated RefCountedClass pointer.
    RefCountedClassType* get () const throw() {
      return ref_counted_class_ptr;
    }

    /// Change the RefCountedClass pointer that is encapsulated. Will cause an
    /// unref on the current RefCountedClass * and a ref on the new.
    /// \param p The new RefCountedClass pointer to encapsulate.
    void reset(RefCountedClassType* p = 0) throw() {
      if( p != ref_counted_class_ptr ) {
        // Increase ref_count to not get early deletion of RefCountedClass
        RefCountedClassType *old_p = ref_counted_class_ptr;
        if( old_p ) old_p->ref();

        // These two have to be in this order, then unless
        // the above ref is done the ptr might disappear to early.
        if ( ref_counted_class_ptr ) unref( ref_counted_class_ptr );
        ref_counted_class_ptr = p;
        if ( ref_counted_class_ptr ) ref( ref_counted_class_ptr );

        // unref to even out the previous ref
        if( old_p ) old_p->unref();
      }
    }
#ifdef H3D_REFERENCE_COUNT_DEBUG
    virtual H3DUtil::RefCountedClass *getContent() {
      return ref_counted_class_ptr;
    }
#endif

  protected:
    /// This function is called when a RefCountedClass * is to be held by the AutoRef.
    /// It increments the reference counter of the RefCountedClass by calling the 
    /// ref() function. Subclasses can override this function in order to 
    /// get specialized behaviour.
    /// \param n The RefCountedClass that is to be held by the AutoRef
    ///
    inline virtual void ref( RefCountedClassType *n ) {
      if( n )
        n->ref();
    }

    /// This function is called when a RefCountedClass * is released by the AutoRef. 
    /// It decrements the reference counter of the RefCountedClass by calling the 
    /// unref() function. Subclasses can override this function in order to 
    /// get specialized behaviour.
    /// \param n The RefCountedClass being released by the AutoRef.
    /// 
    inline virtual void unref( RefCountedClassType *n ) {
      if( n )
        n->unref();
    }
        
    RefCountedClassType *ref_counted_class_ptr;

  };
}

#endif
