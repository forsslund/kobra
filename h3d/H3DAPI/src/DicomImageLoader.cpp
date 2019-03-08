//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file DicomImageLoader.cpp
/// \brief CPP file for DicomImageLoader, a image loader using the DicomImage
/// library to read images.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/DicomImageLoader.h>
#include <H3DUtil/DicomImage.h>
#include <H3D/DDSImageLoader.h>

#ifdef HAVE_DCMTK
#include <dcmtk/dcmdata/dcmetinf.h>
#include <dcmtk/dcmdata/dcdatset.h>
#include <dcmtk/dcmdata/dcdeftag.h>
#include <dcmtk/ofstd/ofconsol.h>

// if on 32 bit windows
#ifndef H3D_WINDOWS 
#include <dirent.h>
#endif

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase DicomImageLoader::database( "DicomImageLoader", 
                                           &(newInstance<DicomImageLoader>), 
                                           typeid( DicomImageLoader ) );

namespace DicomImageLoaderInternals {
  FIELDDB_ELEMENT( DicomImageLoader, loadSingleFile, INITIALIZE_ONLY )
}

H3DImageLoaderNode::FileReaderRegistration 
DicomImageLoader::reader_registration(
                            "DicomImageLoader",
                            &(newImageLoaderNode< DicomImageLoader >),
                            &DicomImageLoader::supportsFileType 
                            );


DicomImageLoader::DicomImageLoader():
  loadSingleFile( new SFBool ) {

  type_name = "DicomImageLoader";
  database.initFields( this );
  
  loadSingleFile->setValue( false );
}

// DCMTK version 3.6.1 adds namespace and puts log4cplus in the namespace.
// So the simplest fix is simply to add the empty namespace here and then use it.
namespace dcmtk { 
}

bool DicomImageLoader::supportsFileType( const string &url ) {
  using namespace dcmtk;
  // DicomImageLoader try to load some DDS image, then it hangs, so stop
  // using DicomImageLoader to load it when DDSImageLoader can do it
  if( DDSImageLoader::supportsFileType(url) ) {
    return false;
  }
  // set the dcmtk console to write to the H3D console
  ofConsole.setCerr( &H3DUtil::Console );
  ofConsole.setCout( &H3DUtil::Console );

  // temporarily shut down console to avoid warning messages from
  // dcmtk while checking if supported.
  H3DUtil::Console.disable();
  log4cplus::Logger log = log4cplus::Logger::getRoot();
  log4cplus::LogLevel old_level = log.getLogLevel();
  // make DCMTK less likely to send out messages to console which is disabled
  // Note: better to competely disable it if it is possible
  log.setLogLevel( OFLogger::FATAL_LOG_LEVEL );


  DcmFileFormat fileformat;
  OFCondition status = fileformat.loadFile( url.c_str(), EXS_Unknown,
    EGL_noChange, 0 );
  // restore console output level.
  H3DUtil::Console.enable();
  log.setLogLevel( old_level );
  return status.good();
}

H3D::Image *DicomImageLoader::loadImage( const string &url ) {
  return H3DUtil::loadDicomFile( url, loadSingleFile->getValue() );
}

#endif //HAVE_DCMTK
