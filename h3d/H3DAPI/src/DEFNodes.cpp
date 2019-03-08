//////////////////////////////////////////////////////////////////////////////
//    Copyright 2018-2019, SenseGraphics AB
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
/// \file DEFNodes.cpp
/// \brief CPP file for DEFNodes
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/DEFNodes.h>

using namespace H3D;

#ifdef H3D_REFERENCE_COUNT_DEBUG
std::set< X3D::DEFNodesBase * > X3D::DEFNodesBase::DEF_nodes;
MutexLock X3D::DEFNodesBase::DEF_nodes_lock;

//#include <boost/stacktrace.hpp>


X3D::DEFNodesBase::DEFNodesBase() {
  // for debugging when tracing down where an Unknown AutoRef is located
  //  std::stringstream s;
  // s << boost::stacktrace::stacktrace();
  // name = s.str(); 
  name = "Unknown";
  DEF_nodes_lock.lock();
  DEF_nodes.insert(this);
  DEF_nodes_lock.unlock();
}

X3D::DEFNodesBase::~DEFNodesBase() {
  DEF_nodes_lock.lock();
  DEF_nodes.erase(this);
  DEF_nodes_lock.unlock();
}

#endif

X3D::DEFNodes::~DEFNodes() {
  for (iterator i = map< const string, Node * >::begin();
    i != map< const string, Node * >::end();
    ++i) {
    (*i).second->unref();
  }
}
