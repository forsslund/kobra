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
/// \file Threads.h
/// \brief Header file for thread handling functions.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __THREADS_H__
#define __THREADS_H__

#include <H3DUtil/H3DUtil.h>
#include <list>
#ifdef THREAD_LOCK_DEBUG
#if defined(_MSC_VER) && _MSC_FULL_VER < 150030729
#undef THREAD_LOCK_DEBUG
#else
#include <unordered_map>
#endif
#endif
#include <vector>
#include <set>
#include <string>
#include <algorithm>
#if defined( _MSC_VER ) && ( _MSC_VER >= 1900 ) && defined( PTHREAD_W32_LEGACY_VERSION )
#define HAVE_STRUCT_TIMESPEC 1
#endif
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <stdexcept>

#ifdef H3D_WINDOWS
#define DEFAULT_THREAD_PRIORITY THREAD_PRIORITY_NORMAL
#else
#define DEFAULT_THREAD_PRIORITY 0
#endif // H3D_WINDOWS

namespace H3DUtil {
#define THREADSPECIFICCONTAINERMAXTHREAD 4
  /// Mutual exclusion lock class for synchronisation between threads. 
  /// Most common usage is to make sure that only one thread enters a 
  /// "critical section" at a time. E.g. if both threads uses the same 
  /// variable we must put a lock around the access to make sure that
  /// two threads does not access it at once.
  class H3DUTIL_API MutexLock {
  public:
    /// Constructor.
    MutexLock();
    
    /// Destructor.
    ~MutexLock();

    /// Locks the mutex. If already locked, waits until it is unlocked and
    /// then locks it.
    void lock();

    /// Unlocks the mutex.
    void unlock();

    /// Try to lock the mutex, if the lock is not available false is returned.
    bool tryLock();
        
  protected:
    pthread_mutex_t mutex;
  };

  /// Read-write lock that allows only one writing thread or one or
  /// more reading threads on the same lock.
  class H3DUTIL_API ReadWriteLock {
  public:
    /// Constructor.
    ReadWriteLock();
    
    /// Destructor.
    ~ReadWriteLock();
    
    /// Applies a read lock. If already write locked, waits until it
    /// is unlocked and then locks it.
    void readLock();
    
    /// Applies a write lock. If already read or write locked, waits
    /// until it is unlocked and then locks it.
    void writeLock();
    
    /// Applies a read lock if it is not already write locked.
    bool tryReadLock();
    
    /// Applies a write lock if it is not already read or write
    /// locked.
    bool tryWriteLock();
    
    /// Unlocks first any write locks and then read locks.
    void unlock();
    
  protected:
    pthread_rwlock_t rwlock;
  };


  /// The ConditionLock is a little more advanced version of MutexLock in that
  /// it can wait for an arbitrary action in the other thread.
  /// 
  /// Thread 1        
  /// l.lock()
  /// l.wait()  - lock will be released and the thread will wait for a signal
  ///             from another thread before continuing. When it continues the
  ///             lock is aquired again.
  /// l.unlock()
  ///
  /// Thread 2
  /// l.lock()
  /// do some stuff
  /// l.signal() - wake the waiting thread  
  /// l.unlock()
  class H3DUTIL_API ConditionLock: public MutexLock {
  public:
    /// Constructor.
    ConditionLock();
    
    /// Destructor.
    ~ConditionLock();

    /// Wait for the conditional to get a signal. The lock will be released
    /// while waiting and reaquired when a signal is received.
    void wait();

    /// Wait for the conditional to get a signal, but only wait a
    /// certain time. If the time exceeds the specified time false
    /// is returned. If signal is received true is returned.
    bool timedWait( unsigned int ms );

    /// Wakes up at least one thread blocked on this condition lock.
    void signal();

    /// This wakes up all of the threads blocked on the condition lock.
    void broadcast();
        
  protected:
    pthread_cond_t cond; 
  };

  /// A container which stores a variable that has a different value for 
  /// each thread that accesses it
  ///
  /// - Assignment to type T is overridden to store a value
  /// - Type conversion to type T is overridden to read a value
  ///
  /// \note A limited number of ThreadSpecificStorage instances are permitted
  /// per process (see PTHREAD_KEYS_MAX). Use ThreadSpecificContainer to avoid this restriction.
  ///
  /// \par Example:
  /// \code{.cpp}
  /// ThreadSpecificStorage< int > tss_int;  // create thread specific storage for an int
  /// tss_int = 1                            // write "1" to thread specific storage
  /// int local_int = tss_int                // read from thread specific storage
  /// \endcode
  ///
  template < typename T >
  class ThreadSpecificStorage {
  public:
    /// Constructor
    ThreadSpecificStorage() {
      int success = pthread_key_create( &key, destructor );
      if( success != 0 ) {
        switch( success ) {
        case EAGAIN:
          throw std::runtime_error( 
            "Unable to create thread specific storage key: Insufficient resources or PTHREAD_KEYS_MAX exceeded!" );
        case ENOMEM:
          throw std::runtime_error( "Unable to create thread specific storage key: Insufficient memory!" );
        default:
          throw std::runtime_error( "Unable to create thread specific storage key: Unknown error!" );
        }
      }
    }

    /// Destructor
    virtual ~ThreadSpecificStorage() {
      pthread_key_delete( key );
    }

    /// Write a value to thread specific storage
    ThreadSpecificStorage< T >& operator=( const T& _value ) {
      initStorage();

      T* value = static_cast< T* >( pthread_getspecific( key ) );
      *value = _value;
      return *this;
    }

    /// Read a value from thread specific storage
    operator T& () {
      initStorage();

      T* value = static_cast< T* >( pthread_getspecific( key ) );
      return *value;
    }

  protected:
    /// Initialize thread specific storage
    void initStorage() {
      if( !pthread_getspecific( key ) ) {
        T* value = new T();
        pthread_setspecific( key, value );
      }
    }

    /// Destroy thread specific storage
    static void destructor( void* data ) {
      T* value = static_cast< T* >( data );
      delete value;
    }

    pthread_key_t key;
  };

  /// A generic scoped lock template
  template< typename Lockable >
  class ScopedLockBase {
  public:
    /// Constructor. Locks the object.
    ScopedLockBase( Lockable& _lockable, void(Lockable::*_lock)(), void(Lockable::*_unlock)() ) :
      lockable( _lockable ), unlock( _unlock ) {
      (lockable.*_lock)();
    }

    /// Destructor. Releases the object.
    ~ScopedLockBase() {
      (lockable.*unlock)();
    }

    Lockable& lockable;
    void(Lockable::*unlock)();
  };
  
  /// A helper class to lock a mutex (or any object having lock()/unlock() methods) for the current scope
  ///
  /// The lock is obtained when the ScopedLock is created and released when it is destroyed.
  /// This simplifies releasing the lock when returning early or raising exceptions.
  ///
  /// \par Example:
  /// \code{.cpp}
  /// void myFunction() {
  ///   ScopedLock< MutexLock > lock( my_mutex );   // lock obtained
  ///   ...
  ///   if( ... ) {
  ///     return;                                   // lock released
  ///   }
  /// }                                             // lock released
  /// \endcode
  template< typename Lockable >
  class ScopedLock : public ScopedLockBase< Lockable > {
  public:
    ScopedLock( Lockable& _lockable ) :
      ScopedLockBase< Lockable >( _lockable, &Lockable::lock, &Lockable::unlock ) {}
  };

  /// A scoped write lock compatible with a ReadWriteLock
  template< typename Lockable >
  class ScopedLockWrite : public ScopedLockBase< Lockable > {
  public:
    ScopedLockWrite( Lockable& _lockable ) : 
      ScopedLockBase< Lockable >( _lockable, &Lockable::writeLock, &Lockable::unlock ) {}
  };

  /// A scoped read lock compatible with a ReadWriteLock
  template< typename Lockable >
  class ScopedLockRead : public ScopedLockBase< Lockable > {
  public:
    ScopedLockRead( Lockable& _lockable ) : 
      ScopedLockBase< Lockable >( _lockable, &Lockable::readLock, &Lockable::unlock ) {}
  };

  /// Internal non-templated base for ThreadSpecificContainer
  class H3DUTIL_API ThreadSpecificContainerBase {
  protected:
    /// Protected constructor, use ThreadSpecificContainer instead
    ThreadSpecificContainerBase() {}

    /// Stores information about a thread necessary to access its data from a ThreadSpecificContainer
    ///
    /// Internally we use ThreadSpecificStorage to store a unique index for each thread.
    /// The index is assigned on demand when a new thread accesses the data via \c getInstance().
    class H3DUTIL_API ThreadInfo {
    protected:
      ThreadInfo();

    public:
      ~ThreadInfo();
      friend class ThreadSpecificStorage< ThreadInfo >;
      static ThreadInfo& getInstance();

      /// The index used to access data specific to this thread
      std::size_t index;

    protected:
      static MutexLock mutex;
      static std::size_t acquireThreadIndex();
      static std::set<std::size_t> used_thread_indexes;
    };
  };

  /// Similar to ThreadSpecificStorage, this class stores a value that is different for each thread.
  ///
  /// Unlike ThreadSpecificStorage there is no restriction on the number of instances of 
  /// ThreadSpecificContainer used in a process. However, the maximum number of threads that will access 
  /// it must be specified at compile time as \c MaxThreads.
  ///
  /// \c MaxThreads refers to all types of ThreadSpecificContainer not only the instantiated template.
  ///
  template < typename T, std::size_t MaxThreads = THREADSPECIFICCONTAINERMAXTHREAD >
  class ThreadSpecificContainer : public ThreadSpecificContainerBase {
  public:
    /// Constructor
    /// \param default_value An initial value. By default value-initialization is used.
    ThreadSpecificContainer( const T& default_value = T() ) {
      std::fill_n( value, MaxThreads, default_value );
    }

    /// Write a value to thread specific storage
    ThreadSpecificContainer< T >& operator=( const T& _value ) {
      ThreadInfo& ti = ThreadInfo::getInstance();
      assert( ti.index < MaxThreads );
      value[ ti.index ] = _value;
      return *this;
    }

    /// Read a value from thread specific storage
    operator T& () {
      ThreadInfo& ti = ThreadInfo::getInstance();
      assert( ti.index < MaxThreads );
      return value[ ti.index ];
    }

    T value[ MaxThreads ];
  };

  /// The abstract base class for threads.
  class H3DUTIL_API ThreadBase {
  public:
    /// Constructor
    ThreadBase():
      name( "Unnamed" ), valid( false ) {
      current_threads.push_back( this );
    }


    /// Destructor.
    virtual ~ThreadBase() {
   
      std::vector< ThreadBase *>::iterator i = std::find( current_threads.begin(), current_threads.end(), this ); 
      if( i != current_threads.end() ) {
        current_threads.erase( i );
      }

#ifdef THREAD_LOCK_DEBUG
      thread_lock_info.erase( getThreadId() );
#endif
    }

    /// Used to specify thread priority in a platform independent way.
    enum Priority {
      LOW_PRIORITY      = -2,
      NORMAL_PRIORITY   =  0,
      HIGH_PRIORITY     =  2,
      REALTIME_PRIORITY =  5
    };

    typedef pthread_t ThreadId;

    /// Returns the id of the thread this function is called in.
    static ThreadId getCurrentThreadId();

    /// Returns the thread this function is called in.
    static ThreadBase *getCurrentThread();

    /// Returns the id of the main thread.
    static ThreadId getMainThreadId() {
      return main_thread_id;
    }
    
    /// Returns true if the call was made from the main thread.
    static bool inMainThread();

    /// Returns the thread id for this thread.
    inline ThreadId getThreadId() { return thread_id; }

    /// Only for Windows Visual Studio users. Sets the name of the 
    /// thread, specified by id, as it appears in the Visual Studio debugger.
    static void setThreadName( ThreadId id, const std::string &name );

#ifdef THREAD_LOCK_DEBUG
    struct ThreadLockInfo {
      ThreadLockInfo():
        thread( NULL ),
        total_run_time( 0 ), 
        total_lock_time( 0 ),
        period_start_time( -1 ),
        period_lock_time( 0 ){
      }
    
      ThreadBase *thread;
      double total_lock_time;
      double total_run_time;
      double period_start_time;
      double period_lock_time;
    };
  

        struct equal_to_pthread
                : public std::binary_function<ThreadId, ThreadId, bool>
        {       // functor for operator==
        bool operator()(const ThreadId& _Left, const ThreadId& _Right) const
                {       // apply operator== to operands
                return pthread_equal( _Left, _Right ) != 0;
                }
        };

struct hash_pthread
                : public std::unary_function<ThreadId, size_t>
        {       // functor for operator==
        bool operator()(const ThreadId& _Left) const
                {       // apply operator== to operands
#ifdef H3D_WINDOWS
                return _Left.x!=0;
#else
    return _Left!=0;
#endif
                }
        };
    typedef std::unordered_map< ThreadId, ThreadLockInfo, hash_pthread, equal_to_pthread > ThreadLockInfoMap;
    static ThreadLockInfoMap thread_lock_info;
#endif
    /// The threads currently in use.
    static std::vector< ThreadBase * > current_threads;

    // Returns the thread with the given id, if such an instance exists. Returns NULL if not.
    // A return of NULL means that no thread by that id has been created with the threadBase
    // class, but such a thread might still exist on the system and been created by other means.
    static ThreadBase *getThreadById( ThreadId id );

    /// Only for Windows Visual Studio users. Sets the name of the 
    /// thread as it appears in the Visual Studio debugger.
    void setThreadName( const std::string &name );

    /// Only for Windows Visual Studio users. Gets the name of the 
    /// thread as it appears in the Visual Studio debugger.
    const std::string &getThreadName();

    /// Check if the thread is correctly created. It is the children class's
    /// responsibility to fill in the correct value for the valid variable.
    bool isValid() { return valid; }

  protected:
    /// the id of the thread.
    ThreadId thread_id;

    /// The name of the thread.
    std::string name;
    
    /// The id of the main thread.
    static ThreadId main_thread_id;

    /// value to indicate validity
    bool valid;
  };

  /// The abstract base class for threads that have a main loop and that allow
  /// you to add callback functions to be run in that loop.
  class H3DUTIL_API PeriodicThreadBase: public ThreadBase {
  public:
    /// Return code for callback functions. 
    typedef enum {
      /// The callback is done and should not be called any more.
      CALLBACK_DONE,
      /// The callback should be rescheduled and called the next loop 
      /// again.
      CALLBACK_CONTINUE
    } CallbackCode;

    /// Constructor.
    PeriodicThreadBase();

    /// Callback function type.
    typedef CallbackCode (*CallbackFunc)(void *data); 

    /// Add a callback function to be executed in this thread. The calling
    /// thread will wait until the callback function has returned before 
    /// continuing.
    virtual void synchronousCallback( CallbackFunc func, void *data ) = 0;

    /// Add a callback function to be executed in this thread. The calling
    /// thread will continue executing after adding the callback and will 
    /// not wait for the callback function to execute.
    /// Returns a handle to the callback that can be used to remove
    /// the callback.
    virtual int asynchronousCallback( CallbackFunc func, void *data ) = 0;

    /// Attempts to remove a callback. returns true if succeded. returns
    /// false if the callback does not exist. This function should be handled
    /// with care. It can remove the wrong callback if the callback that
    /// returned the callback_handle id is removed and a new callback is added.
    /// Callbacks are removed if they return CALLBACK_DONE or a call to this
    /// function is made.
    virtual bool removeAsynchronousCallback( int callback_handle ) = 0;

  protected:
    // internal function used to generate id for each callback.
    inline int genCallbackId() {
      if( free_ids.empty() ) {
        return next_id++;
      } else {
        int id = free_ids.back();
        free_ids.pop_back();
        return id;
      }
    }

    // the next id to use.
    int next_id;

    // if an id have been used and is freed it is stored here.
    std::list< int > free_ids;
  };

  /// The interface base class for all threads that are used for haptics
  /// devices.
  class H3DUTIL_API HapticThreadBase {
  public:
    /// Constructor.
    HapticThreadBase();
    
    /// Destructor.
    virtual ~HapticThreadBase();

    /// Add a callback function that is to be executed when all the haptic
    /// threads have been synchronised, so it will be a thread safe 
    /// callback between all haptic threads.
    static void synchronousHapticCB( PeriodicThreadBase::CallbackFunc func, 
                                     void *data );

    /// Returns true if the call was made from within a HapticThreadBase
    /// thread.
    static bool inHapticThread();
  protected:
    // Callback function for synchronising all haptic threads.
    static PeriodicThreadBase::CallbackCode sync_haptics( void * );

    // The haptic threads that have been created.
    static std::vector< HapticThreadBase * > threads;

    // Lock used to get the haptic threads to wait for the callback 
    // function to finish.
    static ConditionLock haptic_lock; 

    // Lock used to get the thread calling the synchronousHapticCB
    // function to wait for all the haptic threads to synchronize.
    static ConditionLock sg_lock; 

    // Counter used when synchronizing haptic threads. It tells how
    // many more haptic threads that are left to synchronize.
    static int haptic_threads_left;
  };

  /// The SimpleThread class creates a new thread to run a function. The
  /// thread is run until the function returns.
  class H3DUTIL_API SimpleThread : public ThreadBase {

  public:

    /// Function pointer returning void pointer and taking void pointer as argument
    typedef void* (*FunctionPtr)(void*);

    /// Constructor.
    /// \param func The function to run in the thread. 
    /// \param args Arguments to the function in argument func.
    /// \param thread_priority The priority of the thread.
    SimpleThread( FunctionPtr func,
                  void *args = NULL,
                  Priority thread_priority = NORMAL_PRIORITY );

    /// Wait for thread to complete.
    /// Returns 0 on success.
    int join();

    /// Destructor.
    virtual ~SimpleThread();
  }; 

  /// The PeriodicThread class is used to create new threads and provides an interface
  /// to add callback functions to be executed in the new thread that can be 
  /// used by other threads.
  class H3DUTIL_API PeriodicThread : public PeriodicThreadBase {
  public:
    /// Constructor.
    /// \param thread_priority The priority of the thread.
    /// \param thread_frequency The frequence of the thread loop. -1 means
    /// run as fast as possible.
    /// The system will try to match the frequency as close as possible but
    /// the actual frequency is dependent on the frequency of the timer on the 
    /// system. E.g on a Windows system the multimedia timers are used for
    /// synchronization. When run at its highest frequency this will have a clock
    /// cycle of 0.976 ms. This means that the highest frequence we can get is
    /// 1024. Since we only can get an event from the timer once for each ms, the
    /// possible frequences are 1024/x, where x is the number of milliseconds to run
    /// each loop in the thread, i.e. 1024, 512, 342, 256, 205 and so on.
    PeriodicThread( PeriodicThreadBase::Priority 
                      thread_priority = NORMAL_PRIORITY,
                    int thread_frequency = -1 );
    
    /// Deprecated.
    PeriodicThread( int thread_priority,
                    int thread_frequency = -1 );
    
    /// Destructor.
    virtual ~PeriodicThread();
    
    /// Add a callback function to be executed in this thread. The calling
    /// thread will wait until the callback function has returned before 
    /// continuing.
    virtual void synchronousCallback( CallbackFunc func, void *data );

    /// An alternative implementation of synchronousCallback() that reduces the 
    /// likelihood of blocking the calling thread for a long time. By using a separate callback queue 
    /// for synchronous callback, synchronous callback does not need to fight 
    /// with asynchronous callback for trying to hold the locks. When that happens,
    /// the calling thread can get blocked for long time (thread starvation). 
    /// In general, in comparing to synchronousCallback callbacks from this function
    /// can get executed earlier hence blocking the calling thread for shorter period.
    /// NOTE: if thread has a negative frequency, this separate queue won't work due
    /// to risk of stalling, and it will fall back to use the normal synchronousCallback function internally
    virtual void synchronousCallbackDedicatedQueue( CallbackFunc func, void *data );

    /// Add a callback function to be executed in this thread. The calling
    /// thread will continue executing after adding the callback and will 
    /// not wait for the callback function to execute.
    /// Returns a handle to the callback that can be used to remove
    /// the callback.
    /// When subclassing this function all callbacks should be
    /// added to the callbacks_added variable and use the callback_added_lock
    /// around the commands if frequency of the thread if above 0. Otherwise 
    /// the calling thread might have to be synchronized with the
    /// PeriodicThread to add the callbacks. If frequency is below 0 then this
    /// approach can not be used because the thread might be waiting for new
    /// callbacks and there is no safe way to check if the callback_lock is
    /// waiting for new callbacks or waiting in the synchronousCallback
    /// function.
    virtual int asynchronousCallback( CallbackFunc func, void *data );

    /// Add several asynchronous callbacks at once in order to minimize
    /// synchronization time in transferring the callbacks.
    /// The InputIterator must be an iterator where *i is of 
    /// type pair< CallbackFunc, void * >
    /// No handles to the functions will be given so the functions them 
    /// self must finish with CALLBACK_DONE in order for them to be removed.
    template< class InputIterator >
    void asynchronousCallbacks( InputIterator begin,
                                InputIterator end ) {
      bool use_callback_lock = false;
      callbacks_added_lock.lock();  // Have to get added_lock in order to call genCallbackId in a thread safe way
      for( InputIterator i = begin; i != end; ++i ) {
        int cb_id = genCallbackId();
        // add the new callback
        callbacks_added.push_back( make_pair( cb_id, *i ) );
      }
      if( frequency < 0 && callbacks.size() == 0 ) // The size of callbacks is never modified outside callbacks_added_lock so we check size here.
        use_callback_lock = true;
      callbacks_added_lock.unlock();

      if( use_callback_lock ) {
        callback_lock.lock();
        if( callbacks.size() == 0 ) callback_lock.signal(); // callback_lock.signal have to be done between callback_lock pairs in order to get well defined behaviour.
        callback_lock.unlock();
      }
    }

    /// Attempts to remove a callback. returns true if succeded. returns
    /// false if the callback does not exist. This function should be handled
    /// with care. It can remove the wrong callback if the callback that
    /// returned the callback_handle id is removed and a new callback is added.
    /// Callbacks are removed if they return CALLBACK_DONE or a call to this
    /// function is made.
    virtual bool removeAsynchronousCallback( int callback_handle );

    /// Remove all callbacks.
    virtual void clearAllCallbacks();

    /// Exit the thread_func. Will not destroy the PeriodicThread instance.
    inline void exitThread() {
      thread_func_is_running = false;
    }

  protected:
    // The function that handles callbacks. Is also the main function that
    // is run in the thread.
    static void *thread_func( void * );

    typedef std::list< std::pair< int, std::pair< CallbackFunc, void * > > >
      CallbackList;
    // A list of the callback functions to run.
    CallbackList callbacks;
    
    // A lock for synchronizing changes to the callbacks member.
    ConditionLock callback_lock;

    // A list of the callback functions to add to the callbacks variable
    // when the callback_lock is released. 
    CallbackList callbacks_added;

    // A lock used when modifying callbacks_added variable.
    MutexLock callbacks_added_lock;

    // A function that transfers the content of callbacks_added to callbacks.
    // DO NOT use this function anywhere unless you really know what you are
    // doing. It assumes that the callback_lock is locked when used.
    inline void transferCallbackList() {
      callbacks_added_lock.lock();
      callbacks.insert( callbacks.end(), callbacks_added.begin(), callbacks_added.end() );
      callbacks_added.clear();
      callbacks_added_lock.unlock();
    }
    typedef std::list< std::pair< CallbackFunc, void * > > SyncedCallbacklist;
    // a dedicated callback queue for synchronous callbacks
    SyncedCallbacklist synced_callbacks;

    // A lock for synchronizing changes to the synced_callbacks member
    ConditionLock synced_callback_lock;


    /// The priority of the thread.
    Priority priority;

    /// Thre frequency of the thread. -1 means run as fast as possible.
    int frequency;

    /// Flag used to exit the thread by making thread_func return when
    /// set to false.
    bool thread_func_is_running;

  };

  /// HapticThread is a thread class that should be used by haptics devices
  /// when creating threads. It is the same as PeriodicThread, but also inherits
  /// from HapticThreadBase to make it aware that it is a haptic thread.
  class H3DUTIL_API HapticThread : public HapticThreadBase,
                                public PeriodicThread {
  public:
    /// Constructor.
    HapticThread( Priority thread_priority = NORMAL_PRIORITY,
                  int thread_frequency = -1 ):
      PeriodicThread( thread_priority, thread_frequency ) {
    }
    /// Deprecated.
    HapticThread( int thread_priority,
                  int thread_frequency = -1 ):
      PeriodicThread( thread_priority, thread_frequency ) {
    }
  };
}

#endif




