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
/// \file Threads.cpp
/// \brief cpp file for thread handling functions.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
#define _WIN32_WINNT 0x0500
#include <windows.h>
#endif

#include <algorithm>
#include <vector>
#include <iostream>
using namespace std;
#include <H3DUtil/Threads.h>
#ifndef H3D_WINDOWS
#include <unistd.h>
#endif
#include <errno.h>

#ifdef MACOSX
#include <mach/mach_init.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>
#endif

#ifdef HAVE_NVIDIATX
#include <nvToolsExt.h>
#endif

#include <H3DUtil/Console.h>


using namespace H3DUtil;
using namespace std;

#ifdef THREAD_LOCK_DEBUG
 ThreadBase::ThreadLockInfoMap ThreadBase::thread_lock_info;
#endif

vector<ThreadBase *> ThreadBase::current_threads;

MutexLock ThreadSpecificContainerBase::ThreadInfo::mutex;
std::size_t ThreadSpecificContainerBase::ThreadInfo::next_thread_index = 0;

// Constructor.
MutexLock::MutexLock() {
  pthread_mutex_init( &mutex, NULL );
}
 
// Destructor.
MutexLock::~MutexLock() {
  pthread_mutex_destroy( &mutex );
}

// Locks the mutex.
void MutexLock::lock() {
#ifdef THREAD_LOCK_DEBUG
  TimeStamp before;
#endif
  
  pthread_mutex_lock( &mutex );

#ifdef THREAD_LOCK_DEBUG
  double t = TimeStamp() - before;
  ThreadBase::ThreadId thread_id = ThreadBase::getCurrentThreadId();
  ThreadBase::ThreadLockInfo &info = ThreadBase::thread_lock_info[thread_id];
  info.total_lock_time += t;
  info.period_lock_time += t;
#endif
}

// Unlocks the mutex.
void MutexLock::unlock() {
  pthread_mutex_unlock( &mutex );
}

// Try to lock the mutex, if the lock is not available false is returned.
bool MutexLock::tryLock() {
  return pthread_mutex_trylock( &mutex ) != EBUSY;
}

ReadWriteLock::ReadWriteLock() {
  pthread_rwlock_init( &rwlock, NULL );
}

ReadWriteLock::~ReadWriteLock() {
  pthread_rwlock_destroy(&rwlock);
}

void ReadWriteLock::readLock() {
  pthread_rwlock_rdlock(&rwlock);
}

void ReadWriteLock::writeLock() {
  pthread_rwlock_wrlock(&rwlock);
}

bool ReadWriteLock::tryReadLock() {
  return pthread_rwlock_tryrdlock(&rwlock) != EBUSY;
}

bool ReadWriteLock::tryWriteLock() {
  return pthread_rwlock_trywrlock(&rwlock) != EBUSY;
}

void ReadWriteLock::unlock() {
  pthread_rwlock_unlock(&rwlock);
}

// Constructor.
ConditionLock::ConditionLock() {
  pthread_cond_init( &cond, NULL );
}
    
// Destructor.
ConditionLock::~ConditionLock() {
  pthread_cond_destroy( &cond );
}

// Wait for the conditional to get a signal.
void ConditionLock::wait() {
 #ifdef THREAD_LOCK_DEBUG
  TimeStamp before;
#endif

  pthread_cond_wait( &cond, &mutex );

#ifdef THREAD_LOCK_DEBUG
  double t = TimeStamp() - before;
  ThreadBase::ThreadId thread_id = ThreadBase::getCurrentThreadId();
  ThreadBase::ThreadLockInfo &info = ThreadBase::thread_lock_info[thread_id];
  info.total_lock_time += t;
  info.period_lock_time += t;
#endif
}

// Wait for the conditional to get a signal, but only wait a
// certain time. If the time exceeds the specified time false
// is returned. If signal is received true is returned.
bool ConditionLock::timedWait( unsigned int millisecs ) {
  long secs, nanosecs;
  const long NANOSEC_PER_MILLISEC = (long)1e6;
  const long NANOSEC_PER_SEC = (long)1e9;

  /* get current system time and add millisecs */

#ifdef H3D_WINDOWS
  _timeb sys_time;
  _ftime(&sys_time);
  secs = (long)sys_time.time;
  nanosecs = ( (millisecs + sys_time.millitm)) * NANOSEC_PER_MILLISEC;
#else
  timeval sys_time;
  gettimeofday( &sys_time, NULL );
  secs = sys_time.tv_sec;
  nanosecs = ((millisecs + sys_time.tv_usec * (long)1e-3)) *
    NANOSEC_PER_MILLISEC;
#endif // H3D_WINDOWS
  
  if (nanosecs >= NANOSEC_PER_SEC) {
      secs = secs + nanosecs / NANOSEC_PER_SEC;
      nanosecs %= NANOSEC_PER_SEC;
  } 

  timespec time;
  time.tv_nsec = nanosecs;
  time.tv_sec = secs;

  return pthread_cond_timedwait( &cond, &mutex, &time ) != ETIMEDOUT;
}

// Wakes up at least one thread blocked on this condition lock.
void ConditionLock::signal() {
  pthread_cond_signal( &cond ); 
}

// This wakes up all of the threads blocked on the condition lock.
void ConditionLock::broadcast() {
  pthread_cond_broadcast( &cond );
}

void *PeriodicThread::thread_func( void * _data ) {
  PeriodicThread *thread = static_cast< PeriodicThread * >( _data );
  pthread_setcanceltype( PTHREAD_CANCEL_ASYNCHRONOUS, NULL );
#ifdef MACOSX
  // set thread priority
  struct thread_time_constraint_policy ttcpolicy;
  int ret;
  ttcpolicy.period=      100000;
  ttcpolicy.computation=  20000;
  ttcpolicy.constraint=  100000;
  ttcpolicy.preemptible=  1;
  if (( ret=thread_policy_set(
          mach_thread_self(),
          THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&ttcpolicy,
          THREAD_TIME_CONSTRAINT_POLICY_COUNT)) != KERN_SUCCESS ) {
    H3DUtil::Console(LogLevel::Error) << "Threads: set_realtime() failed" << endl;
  }
#endif
  
#ifdef H3D_WINDOWS
  HANDLE hTimer = NULL;
  LARGE_INTEGER liDueTime;
  // add some time to compensate for the time spend between waking up
  // and setting the new timer.
  liDueTime.QuadPart=(LONGLONG)-(1e7 / thread->frequency ) + 400;
  if( thread->frequency > 0 ) {

    // set the multimedia timer frequency to 1 ms
    timeBeginPeriod(1); 
    
    // Create a waitable timer.
    hTimer = CreateWaitableTimer(NULL, TRUE, NULL );
    if (!hTimer) {
      H3DUtil::Console(LogLevel::Error) << "CreateWaitableTimer failed (%d)" << endl 
                         << GetLastError() << endl;
      timeEndPeriod(1); 
      return NULL;
    }
  
    if (!SetWaitableTimer(hTimer, &liDueTime, 0, NULL, NULL, 0)) {
      H3DUtil::Console(LogLevel::Error) << "SetWaitableTimer failed (%d)\n"
        << GetLastError() << endl;
      timeEndPeriod(1); 
      return NULL;
    }
  }

#else
  TimeStamp last_time;
#endif // H3D_WINDOWS

  while( thread->thread_func_is_running ) {
#ifdef THREAD_LOCK_DEBUG
    TimeStamp current_time;
    ThreadBase::ThreadLockInfo &info = ThreadBase::thread_lock_info[thread->getThreadId()];
    if( info.period_start_time != -1 ) {
      info.total_run_time += current_time - info.period_start_time;
    }
#endif
    if( thread->frequency > 0 ) {
#ifdef H3D_WINDOWS
      if (WaitForSingleObject(hTimer, INFINITE) != WAIT_OBJECT_0)
        printf("WaitForSingleObject failed (%d)\n", GetLastError());

      // Set a timer to wait for.
      if (!SetWaitableTimer(hTimer, &liDueTime, 0, NULL, NULL, 0)) {
        H3DUtil::Console(LogLevel::Error) << "SetWaitableTimer failed (%d)\n"
          << GetLastError() << endl;
        return NULL;
      }
#else
      TimeStamp a;
      TimeStamp current_time;
      double dt =  current_time - last_time;
      double delay = 1.0 / thread->frequency - dt;
      if( delay > 0 ) {
         usleep( 1e6 * delay );
      }
      last_time = TimeStamp();
#endif //H3D_WINDOWS
    }

#ifdef THREAD_LOCK_DEBUG
    info.period_start_time = TimeStamp();
#endif

    vector< PeriodicThread::CallbackList::iterator > to_remove;
    thread->callback_lock.lock();

    // At the beginning of every haptic thread cycle, add more callbacks if 
    // there are any. So the added call back do not need to wait one cycle to get
    // added , and at least another cycle to actually get executed in worst case.
    // This will make the synchronousHapticCB runs faster as that function will wait
    // for sync_haptics callback to be actually executed in haptic thread. However,
    // even the callback adding in the very beginning of the cycle, in worst case, 
    // the callback may still miss its first chance to get added 
    // and wait one cycle more to be added and executed. So depends on how fast haptic
    // thread runs and how many haptic thread in total, synchronousHapticCB may 
    // becomes relatively slow.
    
    // if no more callbacks wait for a callback to be added in order to
    // avoid spending time doing no useful operations in the thread.
    thread->transferCallbackList();
    if( thread->frequency < 0 ) {
      // Check thread_func_is_running in order to avoid potential hangs
      // if the user clears the callbacks list and then destroys the thread
      // class. In that case the wait statement can be reached after
      // signal in ~PeriodicThread().
      if( thread->callbacks.size() == 0 && thread->thread_func_is_running ) {
        thread->callback_lock.wait();
        thread->transferCallbackList();
      }
    }

    for( PeriodicThread::CallbackList::iterator i = thread->callbacks.begin();
         i != thread->callbacks.end(); ++i ) {
      PeriodicThread::CallbackCode c = ( (*i).second ).first(
        ( (*i).second ).second );
      if( c == PeriodicThread::CALLBACK_DONE ) {
        to_remove.push_back( i );
      }
    }

    thread->callbacks_added_lock.lock();
    // remove all callbacks that returned CALLBACK_DONE.
    for( vector< PeriodicThread::CallbackList::iterator >::iterator i = 
           to_remove.begin();
         i != to_remove.end(); ++i ) {
      thread->free_ids.push_back( (*(*i) ).first );
      thread->callbacks.erase( *i );
    }
    thread->callbacks_added_lock.unlock();

    thread->callback_lock.signal();

    thread->callback_lock.unlock();

    sched_yield();
#ifndef H3D_WINDOWS
    // According to documentation usleep(0) should not do anything so it should
    // not do any harm to add this code. The reason to add it is that
    // sched_yield does not seem to do what we expect it to
    // do on linux ubuntu. It should yield to other threads of same or
    // higher priority but even if there are threads waiting to obtain
    // a lock that lock is in some cases never obtained when sched_yield
    // is used. Therefore usleep(0) is added because it must do something
    // since the lock is suddenly obtained.
    usleep(0);
#endif // H3D_WINDOWS
  }
  return 0;
} 

ThreadBase::ThreadId ThreadBase::main_thread_id =
  ThreadBase::getCurrentThreadId();
ConditionLock HapticThreadBase::haptic_lock; 
ConditionLock HapticThreadBase::sg_lock; 
int HapticThreadBase::haptic_threads_left = -1;
std::vector< HapticThreadBase * > HapticThreadBase::threads;

namespace ThreadsInternal {
  // callback functions
  PeriodicThreadBase::CallbackCode doNothing( void * ) { 
    return PeriodicThreadBase::CALLBACK_DONE; 
  }
}

PeriodicThreadBase::PeriodicThreadBase() : next_id(0) {}

PeriodicThread::PeriodicThread( int _thread_priority,
                                int _thread_frequency ):
  frequency( _thread_frequency ),
  thread_func_is_running( true ) {
#ifdef H3D_WINDOWS
  priority = _thread_priority == THREAD_PRIORITY_LOWEST ? LOW_PRIORITY :
             _thread_priority == THREAD_PRIORITY_NORMAL ? NORMAL_PRIORITY :
             _thread_priority == THREAD_PRIORITY_ABOVE_NORMAL ? HIGH_PRIORITY :
             REALTIME_PRIORITY;
  int policy = SCHED_OTHER;
#else
  // This is actually not really correct but is put here anyways since priority
  // should be set to something. In old code 20 was used for HIGH_PRIORITY
  // numbers.
  priority = _thread_priority < 20 ? NORMAL_PRIORITY :
             _thread_priority < 99 ? HIGH_PRIORITY :
             REALTIME_PRIORITY;
  int policy =
    priority > NORMAL_PRIORITY ?
    SCHED_FIFO : SCHED_OTHER;
#endif //H3D_WINDOWS

  pthread_attr_t attr;
  sched_param p;
  p.sched_priority = _thread_priority;
  pthread_attr_init( &attr );
  pthread_attr_setschedparam( &attr, &p );
  pthread_attr_setschedpolicy( &attr, policy );
  pthread_create( &thread_id, &attr, thread_func, this ); 
  pthread_attr_destroy( &attr );
}

PeriodicThread::PeriodicThread( Priority _thread_priority,
                                int _thread_frequency ):
  priority( _thread_priority ),
  frequency( _thread_frequency ),
  thread_func_is_running( true ) {
  
  pthread_attr_t attr;
  pthread_attr_init( &attr );
  
  sched_param p;
  pthread_attr_getschedparam( &attr, &p );
  
#ifdef H3D_WINDOWS
  
  p.sched_priority =
    priority == LOW_PRIORITY    ? THREAD_PRIORITY_LOWEST :
    priority == NORMAL_PRIORITY ? THREAD_PRIORITY_NORMAL :
    priority == HIGH_PRIORITY   ? THREAD_PRIORITY_ABOVE_NORMAL :
    THREAD_PRIORITY_HIGHEST;
  
  int policy = SCHED_OTHER;
  
#else
  
  // There is no fine grain control of priority in pthreads unless you
  // have a real-time thread, which you typically don't want if you're
  // not after really high priority. That means that you need
  // superuser rights to apply anything other than normal priority and
  // if you do you will need a real-time scheduler. I.e. to simulate
  // priorities with pthreads "high" and "realtime" priority are using
  // real-time scheduler with normal and high priority and anything
  // below uses normal priority with normal scheduler.
  
  p.sched_priority =
    priority == REALTIME_PRIORITY ? 99 : 0;
  
  int policy =
    priority > NORMAL_PRIORITY ?
    SCHED_FIFO : SCHED_OTHER;
  
#endif // H3D_WINDOWS
  
  pthread_attr_setschedparam( &attr, &p );
  pthread_attr_setschedpolicy( &attr, policy );
  pthread_create( &thread_id, &attr, thread_func, this ); 
  pthread_attr_destroy( &attr );
}

PeriodicThread::~PeriodicThread() {
  ThreadId this_thread = getCurrentThreadId();
  
  if( !pthread_equal( this_thread, thread_id ) ) {
    callback_lock.lock();
    callbacks_added_lock.lock();
    // No need to free ids here since they are local to the PeriodicThread
    // and this is the PeriodicThread destructor.
    callbacks_added.clear();
    callbacks.clear();
    callbacks_added_lock.unlock();
    exitThread();
    callback_lock.signal();
    callback_lock.unlock();
    pthread_join( thread_id, NULL );
  } else {
    pthread_exit(0);
  }
}

SimpleThread::SimpleThread( FunctionPtr func,
                            void *args,
                            Priority thread_priority ){
  pthread_attr_t attr;
  sched_param p;
  
#ifdef H3D_WINDOWS
  
  p.sched_priority =
    thread_priority == LOW_PRIORITY    ? THREAD_PRIORITY_LOWEST :
    thread_priority == NORMAL_PRIORITY ? THREAD_PRIORITY_NORMAL :
    thread_priority == HIGH_PRIORITY   ? THREAD_PRIORITY_ABOVE_NORMAL :
    THREAD_PRIORITY_HIGHEST;
  
  int policy = SCHED_OTHER;
  
#else
  
  // There is no fine grain control of priority in pthreads unless you
  // have a real-time thread, which you typically don't want if you're
  // not after really high priority. That means that you need
  // superuser rights to apply anything other than normal priority and
  // if you do you will need a real-time scheduler. I.e. to simulate
  // priorities with pthreads "high" and "realtime" priority are using
  // real-time scheduler with normal and high priority and anything
  // below uses normal priority with normal scheduler.
  
  p.sched_priority =
    thread_priority == REALTIME_PRIORITY ? 99 : 0;
  
  int policy =
    thread_priority > NORMAL_PRIORITY ?
    SCHED_FIFO : SCHED_OTHER;
  
#endif // H3D_WINDOWS

  pthread_attr_init( &attr );
  pthread_attr_setschedparam( &attr, &p );
  pthread_attr_setschedpolicy( &attr, policy );
  valid = (pthread_create( &thread_id, &attr, func, args ) == 0);
  pthread_attr_destroy( &attr );
}

int SimpleThread::join() {
  return pthread_join( thread_id, NULL );
}

SimpleThread::~SimpleThread() {
  if( !valid ) {// if not valid, do not need to kill the thread
    Console( LogLevel::Info ) << "no need to kill invalid thread: " << name << std::endl;
    return;
  }
  ThreadId this_thread = getCurrentThreadId();
  if( !pthread_equal( this_thread, thread_id ) ) {
    pthread_cancel( thread_id );  
    //pthread_join( thread_id, NULL );
  } else {
    pthread_exit(0);
  }
}

void PeriodicThread::synchronousCallback( CallbackFunc func, void *data ) {
  callback_lock.lock();
  // signal the thread that a new callback is available if it is waiting for one.
  if( frequency < 0 && callbacks.size() == 0 ) callback_lock.signal();
  // add the new callback.
  callbacks_added_lock.lock();  // Have to get added_lock in order to call genCallbackId in a thread safe way
  callbacks.push_back( make_pair( genCallbackId(), make_pair( func, data ) ) );
  callbacks_added_lock.unlock();
  // wait for the callback to be done.
  callback_lock.wait();
  callback_lock.unlock();
}

int PeriodicThread::asynchronousCallback( CallbackFunc func, void *data ) {
  int cb_id = -1;
  bool use_callback_lock = false;

  callbacks_added_lock.lock();
  cb_id = genCallbackId();
  // add the new callback
  callbacks_added.push_back( make_pair( cb_id, make_pair( func, data ) ) );
  if( frequency < 0 && callbacks.size() == 0 )
    use_callback_lock = true;
  callbacks_added_lock.unlock();

  if( use_callback_lock ) {
    callback_lock.lock();
    // signal the thread that a new callback is available if it is waiting for
    // one.
    if( callbacks.size() == 0 ) callback_lock.signal();
    callback_lock.unlock();
  }
  return cb_id;
}

bool PeriodicThread::removeAsynchronousCallback( int callback_handle ) {
  callback_lock.lock();
  callbacks_added_lock.lock();
  // For threads with a low frequency it could be that the callback is
  // in callbacks_added, therefore lock both locks and then go through
  // both lists.
  for( CallbackList::iterator i = callbacks_added.begin();
       i != callbacks_added.end(); ++i ) {
  if( (*i).first == callback_handle ) {
      // Add callback_handle integer to the free_ids list in order
      // to reuse id later.
      free_ids.push_back( callback_handle );
      callbacks_added.erase( i );
      callbacks_added_lock.unlock();
      callback_lock.unlock();
      return true;
    }
  }

  for( CallbackList::iterator i = callbacks.begin();
       i != callbacks.end(); ++i ) {
    if( (*i).first == callback_handle ) {
      // Add callback_handle integer to the free_ids list in order
      // to reuse id later.
      free_ids.push_back( callback_handle );
      callbacks.erase( i );
      callbacks_added_lock.unlock();
      callback_lock.unlock();
      return true;
    }
  }
  callbacks_added_lock.unlock();
  callback_lock.unlock();
  return false;
}

// Remove all callbacks.
void PeriodicThread::clearAllCallbacks() {
  callback_lock.lock();
  callbacks_added_lock.lock();
  // For threads with a low frequency it could be that the callback is
  // in callbacks_added, therefore lock both locks and then go through
  // both lists.

  // All the ids should be added to the free_ids list in
  // order to be reused later.
  for( CallbackList::iterator i = callbacks_added.begin();
       i != callbacks_added.end(); ++i ) {
    free_ids.push_back( (*i).first );
  }
  callbacks_added.clear();

  // All the ids should be added to the free_ids list in
  // order to be reused later.
  for( CallbackList::iterator i = callbacks.begin();
       i != callbacks.end(); ++i ) {
    free_ids.push_back( (*i).first );
  }
  callbacks.clear();
  callbacks_added_lock.unlock();
  callback_lock.unlock();
}


HapticThreadBase::HapticThreadBase() {
  sg_lock.lock();
  threads.push_back( this );
  sg_lock.unlock();

}

HapticThreadBase::~HapticThreadBase() {
  sg_lock.lock();
  vector< HapticThreadBase *>::iterator i = 
    std::find( threads.begin(), 
               threads.end(), 
               this );
  if( i != threads.end() ) {
    threads.erase( i );
  }
  sg_lock.unlock();
}


bool HapticThreadBase::inHapticThread() {
  PeriodicThread::ThreadId id = PeriodicThread::getCurrentThreadId();
  sg_lock.lock();
  for( vector< HapticThreadBase *>::const_iterator i = threads.begin();
       i != threads.end(); ++i ) {
    ThreadBase *thread = dynamic_cast< ThreadBase * >( *i );
    if( thread && pthread_equal( thread->getThreadId(), id ) ) {
      sg_lock.unlock();
      return true;
    }
  }
  sg_lock.unlock();
  return false;
}

PeriodicThreadBase::CallbackCode HapticThreadBase::sync_haptics( void * ) {
  sg_lock.lock();
  --haptic_threads_left;
  if( haptic_threads_left == 0 ) {
    sg_lock.signal();
    sg_lock.wait();
    sg_lock.unlock();
    haptic_lock.lock();
    haptic_lock.broadcast();
    haptic_lock.unlock();
  } else {
    sg_lock.unlock();
    haptic_lock.lock();
    haptic_lock.wait();
    haptic_lock.unlock();
  }
  
  return PeriodicThreadBase::CALLBACK_DONE;
}
void HapticThreadBase::synchronousHapticCB(
                        PeriodicThreadBase::CallbackFunc func, 
                        void *data ) {
  // add empty callbacks in order to make sure that the haptics threads 
  // have released the callback_lock since last call to sync_haptics.
  for( vector< HapticThreadBase *>::iterator i = threads.begin();
       i != threads.end(); ++i ) {
    PeriodicThreadBase *thread = dynamic_cast< PeriodicThreadBase * >( *i );
    if( thread ) {
      thread->asynchronousCallback( ThreadsInternal::doNothing, NULL );

    }
  }
  sg_lock.lock(); 
  haptic_threads_left = (int) threads.size();
  if( haptic_threads_left > 0 ) {
    for( vector< HapticThreadBase *>::iterator i = threads.begin();
         i != threads.end(); ++i ) {
      PeriodicThreadBase *thread = dynamic_cast< PeriodicThreadBase * >( *i );
      if( thread ) {
        thread->asynchronousCallback( sync_haptics, NULL );
      }
    }
    sg_lock.wait();
    func( data );
    sg_lock.signal();

  } else {
    func( data );
  }
  sg_lock.unlock();
  sched_yield();
}

ThreadBase::ThreadId ThreadBase::getCurrentThreadId() {
  return pthread_self();
} 

ThreadBase *ThreadBase::getThreadById( ThreadId id ) {
  for( size_t i = 0; i < current_threads.size(); ++i ) {
    ThreadBase *thread = current_threads[i];
    if( pthread_equal( thread->getThreadId(), id ) ) {
      return thread;
    }
  }
  return NULL;
}

ThreadBase *ThreadBase::getCurrentThread() {
  ThreadBase::ThreadId id = getCurrentThreadId();
  for( std::vector< ThreadBase *>::iterator i = current_threads.begin();
       i != current_threads.end(); ++i ) {
    if( pthread_equal( id, (*i)->getCurrentThreadId() ) != 0 ) {
      return *i;
    }
  }
  
  return NULL;
} 

#ifdef _MSC_VER
#define MS_VC_EXCEPTION 0x406D1388

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO {
  DWORD dwType; // Must be 0x1000.
  LPCSTR szName; // Pointer to name (in user addr space).
  DWORD dwThreadID; // Thread ID (-1=caller thread).
  DWORD dwFlags; // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)
#endif

void ThreadBase::setThreadName( ThreadId thread_id, const string &name ) {
#if ( (PTW32_VERSION_MAJOR >= 3)  || ( ( PTW32_VERSION_MAJOR == 2 ) && ( PTW32_VERSION_MINOR >= 11 ) ) )
  pthread_setname_np( thread_id,  name.c_str() );
#endif

#ifdef _MSC_VER
 THREADNAME_INFO info;
 info.dwType = 0x1000;
 info.szName = name.c_str();
 info.dwThreadID = ((DWORD *)thread_id.p)[0];
 info.dwFlags = 0;

 __try {
   RaiseException( MS_VC_EXCEPTION, 0, 
       sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info );
 }
 __except(EXCEPTION_EXECUTE_HANDLER)
 {
 }
#ifdef HAVE_NVIDIATX
  nvtxNameOsThread(info.dwThreadID, name.c_str());
#endif
#endif

}

void ThreadBase::setThreadName( const string &_name ) {
  name = _name;
  ThreadBase::setThreadName( thread_id, _name );
}

const std::string &ThreadBase::getThreadName() {
  return name;
}
// Returns true if the call was made from the main thread.
bool ThreadBase::inMainThread() {
  return pthread_equal( main_thread_id, getCurrentThreadId() ) != 0;
} 

ThreadSpecificContainerBase::ThreadInfo::ThreadInfo() {
  ScopedLock< MutexLock > lock( mutex );
  index = next_thread_index++;
}

ThreadSpecificContainerBase::ThreadInfo& ThreadSpecificContainerBase::ThreadInfo::getInstance() {
  static ThreadSpecificStorage< ThreadInfo > thread_info;
  return thread_info;
}
