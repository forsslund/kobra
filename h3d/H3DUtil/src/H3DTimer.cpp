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
/// \file H3DTimer.cpp
/// \brief Source file for H3DTimer class.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3DUtil/H3DTimer.h>

using namespace H3DUtil;

#ifdef HAVE_NVIDIATX
#include <nvToolsExt.h>
#include <map>

namespace H3DUtil {
  namespace NSight {
    struct Attributes {
      uint32_t color;
      uint32_t category_id;
    };
    std::map< std::string, Attributes > group_attributes;
  }
}

void H3DUtil::NSight::pushNSightMarker( const std::string & idStr, const std::string &group ) {
  if( group_attributes.empty() ) {
    group_attributes["GENERAL"] = { 0xFF880000, 0 };
    group_attributes["PARSING"] = { 0xFF880088, 1 };
    group_attributes["SHADER"] = { 0xFF000088, 2 };
    group_attributes["PYTHON"] = { 0xFF888800,3 };
    group_attributes["FBO"] = { 0xFF008800,4 };
    group_attributes["GEOMETRY"] = { 0xFF008888,5 };
    group_attributes["H3D"] = { 0xFF000088,6 };
    nvtxNameCategory( 0, "GENERAL" );
    nvtxNameCategory( 1, "PARSING" );
    nvtxNameCategory( 2, "SHADER" );
    nvtxNameCategory( 3, "PYTHON" );
    nvtxNameCategory( 4, "FBO" );
    nvtxNameCategory( 5, "GEOMETRY" );
    // category H3D is reserved to provide a set of nvtx range which does not change too often
    // to make the performance analysis comparable in longer time.
    nvtxNameCategory( 6, "H3D" );
  }
  auto i = group_attributes.find( group );
  uint32_t color;

  if( i == group_attributes.end() ) {
    color = 255 << 24 | (rand() % 200) << 16 | ((rand() % 200) << 8) | ((rand() % 200));
    group_attributes[group].color = color;
  } else {
    color = (*i).second.color;
  }

  nvtxEventAttributes_t initAttrib = { 0 };
  initAttrib.version = NVTX_VERSION;
  initAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  initAttrib.color = color;
  initAttrib.colorType = NVTX_COLOR_ARGB;
  initAttrib.message.ascii = idStr.c_str();
  initAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  initAttrib.category = (*i).second.category_id;
  nvtxRangePushEx( &initAttrib );
}

void H3DUtil::NSight::popNSightMarker() {
  nvtxRangePop();
}

#endif // HAVE_NVIDIATX

#ifdef HAVE_PROFILER
#include <sofa_h3d/helper/AdvancedTimer.h>

void H3DTimer::stepBegin(const std::string & idStr, const std::string &group) {
#ifdef HAVE_NVIDIATX
  NSight::pushNSightMarker(idStr, group );
#endif
  sofa::helper::AdvancedTimer::stepBegin( idStr );
}

void H3DTimer::stepNext(const std::string & idStr, const std::string & next, const std::string &next_group ) {
  H3DTimer::stepEnd( idStr );
  H3DTimer::stepBegin( next );
}

void H3DTimer::stepEnd  (const std::string &idStr) {
#ifdef HAVE_NVIDIATX
  NSight::popNSightMarker();
#endif
  sofa::helper::AdvancedTimer::stepEnd( idStr );
}

void H3DTimer::begin(const std::string & id) {
  sofa::helper::AdvancedTimer::begin(id);
}

void H3DTimer::end(const std::string & id, std::stringstream& profiledResult) {
  sofa::helper::AdvancedTimer::end(id,profiledResult);
}

bool H3DTimer::isEnabled(const std::string & id) {
  return sofa::helper::AdvancedTimer::isEnabled(id);
}

void H3DTimer::setEnabled(const std::string & id, bool val) {
  sofa::helper::AdvancedTimer::setEnabled(id, val);
}


void H3DTimer::setInterval(const std::string & id, int val) {
  sofa::helper::AdvancedTimer::setInterval(id, val);
}
    
int H3DTimer::getInterval(const std::string & id) {
   return sofa::helper::AdvancedTimer::getInterval(id);
}



#endif
