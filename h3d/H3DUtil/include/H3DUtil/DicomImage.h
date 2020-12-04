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
/// \file DicomImage.h
/// \brief Image class for an image created from a Dicom file.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __DICOMIMAGE_H__
#define __DICOMIMAGE_H__

#include <H3DUtil/PixelImage.h>
#include <H3DUtil/Exception.h>

#ifdef HAVE_DCMTK


#ifndef H3D_WINDOWS
// for Unix platforms this has to be defined before including anything
// from dcmtk
#define HAVE_CONFIG_H
#endif // H3D_WINDOWS
// DCMTK includes
#include <dcmtk/dcmdata/dcfilefo.h> 


namespace H3DUtil {
  /// \class DicomImage
  /// This Image class uses DCMTK to create an Image from a DICOM file,
  /// providing all nesessary Image functions to use it in e.g. ImageTexture
  /// 
  class H3DUTIL_API DicomImage: public PixelImage {
  public:
    /// Throw when there was an error while loading the Dicom image.
    H3D_VALUE_EXCEPTION( std::string, CouldNotLoadDicomImage );
    /// Constructor. 
    /// \param url The url of the Dicom file.
    DicomImage( const std::string &url );

    /// Get the DcmFileFormat (see DCMTK documentation) object for the 
    /// loaded file. This allows you to access the meta info and 
    /// all the elements in Dicom datase.
    inline DcmFileFormat &getDicomFileInfo() {
      return dicom_file_info;
    }

    /// Returns true if the loaded file was a DIRFILE.
    inline bool dirfileLoaded() {
      return dir_file_info.size() > 0;
    }

    /// Get the vector of DcmFileFormat objects for each file specified
    /// in the DIRFILE, if a DIRFILE was loaded. The vector is empty if
    /// no dirfile was loaded.
    inline std::vector< DcmFileFormat > &getDirfileFileInfo() {
      return dir_file_info;
    }

    /// Convert a pixel value(range 0 to 1) to the corresponing hounsfield value
    /// from the dicom file.
    H3DFloat pixelToHounsfieldValue( H3DFloat v );

    /// Convert a hounsfield value to the corresponding pixel value(range 0 to 1)
    /// from the dicom file.
    H3DFloat hounsfieldToPixelValue( H3DFloat v );

  protected:
    /// Load the image from the given url. The url can be a DIRFILE.
    void loadImage( const std::string &url );

    /// Load the image from several urls where each url specifies
    /// a Dicom file containing a 2D-slice.
    void loadImage( const std::vector< std::string > &urls );

    /// Returns the type of image.
    virtual ImageType getImageType() const override { return ImageType::DICOMIMAGE; };

    /// The DcmFileFormat object for the currently loaded image.
    DcmFileFormat dicom_file_info;
    
    /// The DcmFileFormat object for each file entry if a DIRFILE is
    /// loaded.
    std::vector< DcmFileFormat > dir_file_info;
  };

    
}

#endif // HAVE_DCMTK

#endif


