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
/// \file ThreadPool.cpp
/// \brief CPP file for ThreadPool, collection of threads.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3DUtil/ThreadPool.h>
#include <H3DUtil/TimeStamp.h>
#include <iostream>
#include <sstream>

using namespace H3DUtil;

ThreadPool ThreadPool::global_pool( 0 );

void ThreadPool::reserve( unsigned int nr_threads ) {
  if( size() < nr_threads )
    resize( nr_threads );
}

void ThreadPool::resize( unsigned int nr_threads ) {
  size_t current_nr_threads = threads.size();
  if( nr_threads > current_nr_threads ) {
    for( size_t i = current_nr_threads; i < nr_threads; i++ ) {
      PeriodicThread *thread = new PeriodicThread;
      std::stringstream s;
      s << "ThreadPool thread " << i;
      thread->setThreadName( s.str() );
      threads.push_back( thread );
    }
  } else if( nr_threads < current_nr_threads ) {
    for( size_t i = current_nr_threads; i > nr_threads; i-- )
      threads.pop_back();
  }
  
  next_thread = 0;
}

unsigned int ThreadPool::size() {
  return static_cast<unsigned int>(threads.size());
}

PeriodicThread *ThreadPool::findBestThread() {
  PeriodicThread *pt = threads[next_thread];
  next_thread = (next_thread + 1) % threads.size();
  return pt;
}

void ThreadPool::executeFunction( FunctionType func,
                                  void *args,
                                  SyncObject *so ) {
  ThreadData *td = new ThreadData( func, args, so );
  PeriodicThread *thread = findBestThread();
  std::string a = thread->getThreadName();
  thread->asynchronousCallback( threadCB, td );
}

ThreadPool::SyncObject::~SyncObject() {
  for( SignalTypeContainer::iterator i = signal_locks.begin(); i != signal_locks.end(); i++ ) {
    delete (*i).first;
  }
}


ThreadPool::SyncObject::SignalTypeContainer::iterator ThreadPool::SyncObject::addSignalLock() {
  signal_locks.push_front( std::make_pair(new ConditionLock, false) );
  return signal_locks.begin();
}

void ThreadPool::SyncObject::wait() {
  for( SignalTypeContainer::iterator i = signal_locks.begin(); i != signal_locks.end(); i++ ) {
    (*i).first->lock();
    if( !(*i).second ) (*i).first->wait();
    (*i).first->unlock();
  }
}



PeriodicThread::CallbackCode ThreadPool::threadCB( void *d ) {
  ThreadData *data = static_cast< ThreadData * >( d );
  
  // execute function
  data->func( data->args );
  
  if( data->have_signal ) {
    (*data->signal).first->lock();
    (*data->signal).second = true;
    (*data->signal).first->signal();
    (*data->signal).first->unlock();
  } 

  delete data;
  return PeriodicThread::CALLBACK_DONE;
}


