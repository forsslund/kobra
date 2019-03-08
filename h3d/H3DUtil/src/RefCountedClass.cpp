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
/// \file RefCountedClass.cpp
/// \brief .cpp file for RefCountedClass class.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3DUtil/RefCountedClass.h>

using namespace H3DUtil;

RefCountedClass::RefCountedClass( ):
      ref_count( 0 ),
      name( "" ),
      type_name( "RefCountedClass" ),
      is_initialized( false ),
      manual_initialize( false ),
      ref_count_lock_pointer(0),
      unref_callbacks() {
}

RefCountedClass::RefCountedClass( bool _use_lock ):
      ref_count( 0 ),
      name( "" ),
      type_name( "RefCountedClass" ),
      is_initialized( false ),
      manual_initialize( false ),
      ref_count_lock_pointer(NULL),
      unref_callbacks() {
  if( _use_lock ) {
    ref_count_lock_pointer = new MutexLock();
  }
}

RefCountedClass::~RefCountedClass() {
#ifdef REF_COUNT_DEBUG
  Console(LogLevel::Debug) << "~RefCountedClass: " << this << endl;
#endif
  if( ref_count_lock_pointer ) {
    delete ref_count_lock_pointer;
    ref_count_lock_pointer = NULL;
  }
  unref_callbacks.clear();
}

void RefCountedClass::ref() {
  bool locked = false;
  if( ref_count_lock_pointer ) {
    ref_count_lock_pointer->lock();
    locked = true;
  }
  ++ref_count;
#ifdef REF_COUNT_DEBUG
  Console(LogLevel::Debug) << "Ref " << getName() << " " << this << ": " 
    << ref_count << endl;
#endif
  if( !manual_initialize && ref_count == 1 ) {
    initialize();
  }
  if( locked && ref_count_lock_pointer )
    ref_count_lock_pointer->unlock();
}

void RefCountedClass::unref() {
  for ( UnrefCallbacks::const_iterator it = unref_callbacks.begin(); it != unref_callbacks.end(); ++it ) {
    it->first( this, it->second );
  }
  if( ref_count_lock_pointer )
    ref_count_lock_pointer->lock();
  --ref_count;
#ifdef REF_COUNT_DEBUG
  Console(LogLevel::Debug) << "Unref " << getName() << " " << this << ": " 
    << ref_count << endl;
#endif
  if( ref_count == 0 ) {
    if( ref_count_lock_pointer )
      ref_count_lock_pointer->unlock();
    delete this;
  }
  else {
    if( ref_count_lock_pointer )
      ref_count_lock_pointer->unlock();
  }
}

int H3DUtil::RefCountedClass::addUnrefCallback( void( *func )( RefCountedClass *, void * ), void * args ) {
  unref_callbacks.push_back( std::make_pair( func, args ) );
  return 0;
}

int H3DUtil::RefCountedClass::removeUnrefCallback( void( *func )( RefCountedClass *, void * ), void * args ) {
  UnrefCallbacks::iterator i =
    std::find( unref_callbacks.begin(), unref_callbacks.end(),
               std::make_pair( func, args ) );
  if ( i == unref_callbacks.end() ) {
    return -1;
  } else {
    unref_callbacks.erase( i );
    return 0;
  }
}
