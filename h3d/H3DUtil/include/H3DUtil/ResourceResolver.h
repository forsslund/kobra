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
/// \file ResourceResolver.h
/// \brief Header file for ResourceResolver.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __RESOURCERESOLVER_H__
#define __RESOURCERESOLVER_H__

#include <H3DUtil/H3DUtil.h>
#include <string>
#include <list>
#include <H3DUtil/URNResolver.h>
#include <H3DUtil/AutoPtrVector.h>
#include <H3DUtil/Threads.h>
#include <memory>

namespace H3DUtil {

  /// The ResourceResolver class is both a base class for all classes
  /// that resolves resource names and contains static functions for
  /// resolving them.
  class H3DUTIL_API ResourceResolver {
  public:

    /// List of tmpfile names
    class TmpFileNameList: public std::list< std::string > {
    public:
      ~TmpFileNameList() {
        for( list< std::string >::iterator i = begin();
           i != end(); ++i ) {
          std::remove( (*i).c_str() );
        }
      }
    };

    /// Destructor.
    virtual ~ResourceResolver() {}

    /// This function should be implemented by resource resolvers that
    /// extracts file data. It returns a local filename that contains
    /// the resource specified by url.
    virtual std::string resolveURLAsTmpFile( const std::string &/*url*/ ){ return ""; }

    /// This function should be implemented by resource resolvers that
    /// extracts folder tree data. It returns a local folder that contains
    /// the resource data specified by url.
    virtual std::string resolveURLAsTmpFolder( const std::string &/*url*/ ){ return ""; }

    /// Set the URNResolver to use when resolving resource.
    static void setURNResolver( URNResolver *resolver ) {
      urn_resolver().reset( resolver );
    } 

    /// Get the current URNResolver.
    static URNResolver* getURNResolver() {
      return urn_resolver().get();
    } 

    /// Add a ResourceResolver that can be used when resolving resources. 
    static void addResolver( ResourceResolver *resolver ) {
      resolvers().push_back( resolver );
    }

    /// Set the current base URL. The base URL will be used as the base
    /// when the url to resolve is a relative url.
    static void setBaseURL( const std::string &base ) {
      baseURL = base;
    }

    /// Get the current base URL,
    static const std::string & getBaseURL() {
      return baseURL;
    }

    /// Returns a local filename that contains the resource specified
    /// by urn. The boolean pointed to by the is_tmp_file argument 
    /// is set to true if the resolved file is a temporary file.
    static std::string resolveURLAsFile( const std::string &urn,
                                    bool *is_tmp_file = NULL,
                                    const std::string& _base_url = "" ){
      return resolveURLAs(urn,is_tmp_file,false,false,_base_url);
    }

    /// Returns a local filename that contains the resource specified
    /// by urn. The boolean pointed to by the is_tmp_folder argument 
    /// is set to true if the resolved file is a temporary folder.
    static std::string resolveURLAsFolder( const std::string &urn,
                                      bool *is_tmp_folder = NULL ){
      return resolveURLAs(urn,is_tmp_folder,true,false);
    }

    /// Returns a std::string containing the contents of the url resource
    ///
    /// This function should be implemented by resource resolvers that
    /// extract file data and wish to avoid the use of temporary files
    ///
    /// \param[in] urn     The URL to resolve
    /// \param[in] _base_url The base url to which relative paths are relative to.
    /// \param[in] try_relative_path If true then first try the urn as a relative path.
    static std::string resolveURLAsString ( const std::string &urn, const std::string& _base_url = "", bool try_relative_path = true ) {
      return resolveURLAs(urn,NULL,false,true,_base_url,try_relative_path );
    }

    /// Returns a new unique filename that can be used to create a temporary
    /// file. The filename should be released as soon as it is not needed
    /// any more with the releaseTmpFileName function.
    /// When calling this function the temporary file will be created to
    /// avoid race conditions with other processes creating conflicting temporary
    /// file name.
    static std::string getTmpFileName();

    /// Remove a temporary file. Returns true on success or false if the 
    /// tmpfile does not exist or have not been allocated with the 
    /// getTmpFileName function.
    static bool releaseTmpFileName( const std::string &file );

  protected:

    /// Returns a std::string containing the contents of the url resource
    ///
    /// This function should be implemented by resource resolvers that
    /// extract file data and wish to avoid the use of temporary files.
    ///
    /// \return The contents of the url or an empty std::string on failure
    ///
    virtual std::string resolveURLAsStringInternal( const std::string &/*url*/ ) { return ""; }

    /// Resolves a URN and returns either a local file name or the file
    /// contents itself.
    ///
    /// \param[in] urn              The URN to resolve.
    /// \param[out] is_tmp_file     Will be set to true if the filename returned is temporary
    ///                             the caller should then remove the file once it is no longer 
    ///                             needed.
    /// \param[in] folder           True if the resource is a folder.
    /// \param[in] return_contents  If true then the file contents is returned, 
    ///                             otherwise the local file name is returned.
    /// \param[in] _base_url The base url to which relative paths are relative to.
    /// \param[in] try_relative_path If true then first try the urn as a relative path.
    static std::string resolveURLAs( const std::string &urn,
                                     bool *is_tmp_file,
                                     bool folder,
                                     bool return_contents = false,
                                     const std::string& _base_url = "",
                                     bool try_relative_path = true );
    
    static std::auto_ptr< URNResolver > & urn_resolver();
    
    static H3DUtil::AutoPtrVector< ResourceResolver > & resolvers() {
      static H3DUtil::AutoPtrVector< ResourceResolver > resolvers;
      return resolvers;
    }
    
    static std::string baseURL;
    static TmpFileNameList tmp_files;
    static H3DUtil::MutexLock tmp_files_lock;
  };
}

#endif
