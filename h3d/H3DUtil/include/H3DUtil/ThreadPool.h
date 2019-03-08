/////////////////////////////////////////////////////////////////////////////
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
/// \file ThreadPool.h
/// \brief Header file for ThreadPool. Collection of threads.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DUTIL_THREADPOOL_H__
#define __H3DUTIL_THREADPOOL_H__

#include <H3DUtil/Threads.h>
#include <H3DUtil/AutoPtrVector.h>

namespace H3DUtil {

  /// The ThreadPool class manages of pool of worker threads that can be used
  /// to perform calculations in parallell. There is a significant overhead
  /// in creating and destroying new threads, so this class keeps a number 
  /// of threads alive and reuse them for new function calls. The caller can
  /// choose to synchronize with the function throug a ThreadPool::SyncObject
  /// instance.
  ///
  /// <b>Example:</b>
  /// \code 
  /// // function to execute.
  /// void f( void * ) { .. }
  /// // synchronization object.
  /// ThreadPool::SyncObject sync_object;
  /// // Add two calls to the function to be executed in parallell using
  /// // the same SyncObject.
  /// ThreadPool::pool.executeFunction( f, NULL, &sync_object );
  /// ThreadPool::pool.executeFunction( f, NULL, &sync_object );
  /// // wait for both function to finish.
  /// sync_object.wait()
  /// \endcode
class H3DUTIL_API ThreadPool {
protected:
  // forward declaration.
  struct ThreadData;
  public:

  /// Destructor.
  ~ThreadPool() {
    threads.clear();
  }

  /// The SyncObject class is used to synchronize and wait for a function
  /// call to be finished in the thread pool. The object can be used by 
  /// many functions so that one can send of multiple functions to be executed 
  /// in parallell and wait for all to be finished.
  class H3DUTIL_API SyncObject {
  public:
    /// Destructor. Deletes all ConditionLocks in the signal_locks list.
    ~SyncObject();

    /// Wait for all functions sharing this SyncObject to complete.
    void wait();

    friend struct ThreadPool::ThreadData;
  protected:
    /// Container type for all locks used for signalling. The 
    typedef std::list< std::pair< H3DUtil::ConditionLock *, bool > > SignalTypeContainer;

    SignalTypeContainer::iterator addSignalLock();
    SignalTypeContainer signal_locks;
    friend class ThreadPool;
  };

  /// Constructor.
  ThreadPool( unsigned int nr_threads) : 
    next_thread( 0 ) {
    resize( nr_threads );
    }

  /// Make sure that the pool contains at least the number of threads 
  /// specified.
  void reserve( unsigned int nr_threads );

  /// Set the number of threads that should be in the pool.
  void resize( unsigned int nr_threads );

  /// Returns the number of threads in the pool.
  unsigned int size();

  /// A global thread pool object that can be used by any class.
  static ThreadPool global_pool;

  /// The type for functions to be executed on the threads.
  typedef void * (*FunctionType)(void *data); 

  /// Execute the function. 
  /// \param func The function to execute.
  /// \param args The argument to the function.
  /// \param so Optional argument to provide a SyncObject for the caller
  /// or the executeFunction to know when the function is finished.
  void executeFunction( FunctionType func,
                        void *args = NULL,
                        SyncObject *so = NULL );

 protected:
  friend struct ThreadData;

  /// Container class for all data needed for a function call in the 
  /// thread.
  struct H3DUTIL_API ThreadData {

    /// Constructor. 
    ThreadData( FunctionType _func,
                void *_args,
                SyncObject *so ) {
      update( _func, _args, so );
    }

    /// Update the data.
    void update( FunctionType _func,
                 void *_args,
                 SyncObject *so ) {
      func = _func;
      args = _args;
      have_signal = so != NULL;
      if( so ) signal = so->addSignalLock();
  }

    FunctionType func;
    void *args;
    bool have_signal;
    SyncObject::SignalTypeContainer::iterator signal;

  };

  /// Callback function to run in pool thread.
  static H3DUtil::PeriodicThread::CallbackCode threadCB( void *data );

  /// Find the thread to use for next callback, prioritizing threads with current low work load.
  H3DUtil::PeriodicThread *findBestThread();

  /// The threads in the pool.
  H3DUtil::AutoPtrVector< H3DUtil::PeriodicThread > threads;

  /// The data for each thread in threads
  std::vector< ThreadData > thread_data;

  /// The index of the thread to put the next executeFunction call on.
  unsigned int next_thread;
};

}
#endif
