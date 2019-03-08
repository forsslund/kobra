//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
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
/// \file DDSImageLoader.h
/// \brief Header file for DDSImageLoader, an image loader for DDS format
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __DDSIMAGELOADER_H__
#define __DDSIMAGELOADER_H__

#include <H3D/H3DImageLoaderNode.h>
#include <H3DUtil/LoadImageFunctions.h>

namespace H3D {

  /// \ingroup H3DNodes
  /// \class DDSImageLoader
  /// DDSImageLoader is used to load DirectDraw Surface images (.dds).
  /// Note that not all dds formas types are supported yet.
  /// Some of the supported formats are dxt1, dxt3 and dxt5.
  /// Look in to the implementation of H3DUtil::loadDDSImage in
  /// H3DUtil::LoadImageFunctions.h to know exactly which formats are supported.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../../H3DAPI/examples/All/DDSImageLoader.x3d">DDSImageLoader.x3d</a>
  ///     ( <a href="examples/DDSImageLoader.x3d.html">Source</a> )
  class H3DAPI_API DDSImageLoader : public H3DImageLoaderNode {
  public:
    /// Constructor.
    DDSImageLoader() {
      type_name = "DDSImageLoader";
    }

    /// Load the image
    /// \param url URL to the file to load.
    virtual Image *loadImage( const string &url ) {
      return H3DUtil::loadDDSImage( url );
    }
    
    /// Load an image from an istream
    ///
    /// Does not need to be implemented for all image loaders
    ///
    /// \returns An Image * with the image data loaded from the
    /// url. Returns NULL if not implemented for image loader.
    ///
    virtual Image *loadImage( istream &is ) {
      return H3DUtil::loadDDSImage( is );
    }

    /// Returns true if the node supports the filetype of the file
    /// specified by url.
    static bool supportsFileType( const string &url );

    /// Returns true if the node supports the filetype of the file
    /// specified by the stream.
    static bool supportsStreamType( istream &is );

    static H3DNodeDatabase database;    

    /// Register this node to the H3DSoundFileNodes available.
    static FileReaderRegistration reader_registration;
  };
}

#endif
