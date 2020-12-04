//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2020, SenseGraphics AB
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
/// \file ResourceResolver.cpp
/// \brief cpp file for ResourceResolver.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3DUtil/ResourceResolver.h>
#include <H3DUtil/Console.h>

#include <sys/stat.h>

#ifndef H3D_WINDOWS
#include <unistd.h>
#endif

#ifndef S_ISREG
#define S_ISREG(x) (((x) & S_IFMT) == S_IFREG)
#endif

#ifndef S_ISDIR
#define S_ISDIR(x) (((x) & S_IFMT) == S_IFDIR)
#endif

using namespace H3DUtil;

std::string ResourceResolver::baseURL( "" );
ResourceResolver::TmpFileNameList ResourceResolver::tmp_files;
MutexLock ResourceResolver::tmp_files_lock;

std::string ResourceResolver::resolveURLAs( 
  const std::string &urn,
  bool *is_tmp_file,
  bool folder,
  bool return_contents,
  const std::string& _base_url,
  bool try_relative_path ) {

  if( urn == "" ) return "";
  std::string filename = urn;
  if( urn_resolver().get() ) {
    filename = urn_resolver()->resolveURN( urn );
  }

  // first try as relative path
  if( try_relative_path ) {
    std::string cur_base_url = _base_url.empty() ? baseURL : _base_url;
    if( cur_base_url != "" ) {
      std::string full_url = cur_base_url + filename;

      // Only return file contents if a resolver explicitly supports it
      // i.e. if it implements resolveURLAsStringInternal()
      if( !return_contents ) {
        // if is a local file, just return the file name
        struct stat file_info;
        int file_status = ::stat( full_url.c_str(), &file_info );
        if( file_status == 0 && (folder ?
          S_ISDIR( file_info.st_mode ) :
          S_ISREG( file_info.st_mode )) != 0 ) {
          if( is_tmp_file ) *is_tmp_file = false;
          return full_url;
        }
      }

      // otherwise try the resolvers.
      for( AutoPtrVector< ResourceResolver >::iterator i = resolvers().begin();
        i != resolvers().end(); ++i ) {
        if( !return_contents ) {
          std::string resolved_name = (*i)->resolveURLAsTmpFile( full_url );
          if( resolved_name != "" ) {
            if( is_tmp_file ) *is_tmp_file = true;
            return resolved_name;
          }
        } else {
          std::string contents = (*i)->resolveURLAsStringInternal( full_url );
          if( contents != "" ) {
            if( is_tmp_file ) *is_tmp_file = false;
            return contents;
          }
        }
      }
    }
  }
  
  // try as absolute path
  
  // if is a local file, just return the file name
  if ( !return_contents ) {
    struct stat file_info;
    int file_status = ::stat(filename.c_str(),&file_info);
    if( file_status == 0 && ( folder ?
                              S_ISDIR(file_info.st_mode) :
                              S_ISREG(file_info.st_mode) ) != 0 ) {
      if( is_tmp_file ) *is_tmp_file = false;
      return filename;
    }
  }
  
  // otherwise try the resolvers.
  for( AutoPtrVector< ResourceResolver >::iterator i = resolvers().begin();
       i != resolvers().end(); ++i ) {
    if ( !return_contents ) {
      std::string resolved_name = (*i)->resolveURLAsTmpFile( filename );
      if( resolved_name != "" ) {
        if( is_tmp_file ) *is_tmp_file = true;
        return resolved_name;
      }
    } else {
      std::string contents = (*i)->resolveURLAsStringInternal( filename );
      if( contents != "" ) {
        if( is_tmp_file ) *is_tmp_file = false;
        return contents;
      }
    }
  }
  if( is_tmp_file ) *is_tmp_file = false;
  return "";
}

std::string ResourceResolver::getTmpFileName() {
#ifdef H3D_WINDOWS
  // gets the temp path env string (no guarantee it's a valid path).
  TCHAR temp_path_buffer[MAX_PATH];
  DWORD path_result = GetTempPath( MAX_PATH, temp_path_buffer );
  if( path_result > MAX_PATH || path_result == 0 ) {
    // an error occurred
    Console( LogLevel::Error ) <<
      "ResourceResolver::getTmpFileName(): Failed to get temp directory!" << std::endl;
    return "";
  }

  // generates a temporary file name
  //
  // NOTE: This also creates an empty file and releases the handle to it
  // which prevents the race condition associated with _tempnam() and
  // similar, which only generates the filename
  TCHAR temp_file_name[MAX_PATH];
  UINT temp_file_result = GetTempFileName( 
    temp_path_buffer,   // directory for tmp files
    TEXT( "h3d" ),      // temp file name prefix (max 3 characters used)
    0,                  // create unique name
    temp_file_name );   // buffer for name
  if( temp_file_result == 0 ) {
    // an error occurred
    Console( LogLevel::Error ) <<
      "ResourceResolver::getTmpFileName(): Failed to create temp file!" << std::endl;
    return "";
  }

  tmp_files_lock.lock();
  tmp_files.push_back( temp_file_name );
  tmp_files_lock.unlock();
  
  return temp_file_name;
#else
  // check the environment variable for the tmp directory
  const char* tmp_dir = getenv( "TMPDIR" );
  if( !tmp_dir ) {
    // fallback to standard location
    tmp_dir = "/tmp";
  }

  // a template for the chosen filename 
  const char* tmp_template = "/tmph3d_XXXXXX";

  // generate the full template path
  char* template_path = new char [ strlen(tmp_dir) + strlen(tmp_template) + 1 ];
  strcpy( template_path, tmp_dir );
  strcat( template_path, tmp_template );

  // generate a unique name and create an empty file in one atomic step
  int fd = mkstemp( template_path );
  if( fd != -1 ) {
    // success, store the new filename
    std::string temp_file_name( template_path );
    delete [] template_path;

    tmp_files_lock.lock();
    tmp_files.push_back( temp_file_name );
    tmp_files_lock.unlock();

    // release the file so the caller can re-open it
    close( fd );
    
    return temp_file_name;
  } else {
    // an error occurred
    Console( LogLevel::Error ) <<
      "ResourceResolver::getTmpFileName(): Failed to create temp file!" << std::endl;
    delete [] template_path;
    return "";
  }
#endif
}

bool ResourceResolver::releaseTmpFileName( const std::string &file ) {
  tmp_files_lock.lock();
  for( std::list< std::string >::iterator i = tmp_files.begin();
       i != tmp_files.end(); ++i ) {
    if( file == (*i) ) {
      remove( (*i).c_str() );
      tmp_files.erase( i );
      tmp_files_lock.unlock();
      return true;
    }
  }
  tmp_files_lock.unlock();
  return false;
}

std::auto_ptr< URNResolver > & ResourceResolver::urn_resolver() {
  static std::auto_ptr< URNResolver > urn_resolver( NULL );
  return urn_resolver;
}
