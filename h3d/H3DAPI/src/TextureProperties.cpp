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
/// \file TextureProperties.cpp
/// \brief CPP file for TextureProperties, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/TextureProperties.h>

using namespace H3D;

H3DNodeDatabase TextureProperties::database( 
        "TextureProperties", 
        &newInstance< TextureProperties>,
        typeid( TextureProperties ),
        &X3DNode::database 
        );

namespace TexturePropertiesInternals {
  FIELDDB_ELEMENT( TextureProperties, anisotropicDegree, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, borderColor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, borderWidth, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, boundaryModeS, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, boundaryModeT, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, boundaryModeR, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, magnificationFilter, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, minificationFilter, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, textureCompression, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, texturePriority, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, generateMipMaps, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, textureTransferScale, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, textureTransferBias, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, textureCompareMode, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, textureCompareFailValue, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, textureType, INPUT_OUTPUT )
  FIELDDB_ELEMENT( TextureProperties, textureFormat, INPUT_OUTPUT )
}

TextureProperties::TextureProperties( 
                       Inst< SFNode  > _metadata             ,
                       Inst< SFFloat  > _anisotropicDegree   ,
                       Inst< SFColorRGBA > _borderColor      ,
                       Inst< SFInt32  > _borderWidth         ,
                       Inst< SFString > _boundaryModeS       ,
                       Inst< SFString > _boundaryModeT       ,
                       Inst< SFString > _boundaryModeR       ,
                       Inst< SFString  > _magnificationFilter,
                       Inst< SFString  > _minificationFilter ,
                       Inst< SFString > _textureCompression  ,
                       Inst< SFFloat > _texturePriority       ,
                       Inst< SFBool  > _generateMipMaps,
                       Inst< SFVec4f > _textureTransferScale,
                       Inst< SFVec4f > _textureTransferBias,
                       Inst< SFString > _textureCompareMode,
                       Inst< SFFloat  > _textureCompareFailValue,
                       Inst< SFString > _textureType,
                       Inst< SFString > _textureFormat ):
  X3DNode( _metadata ),
  anisotropicDegree ( _anisotropicDegree  ),
  borderColor ( _borderColor  ),
  borderWidth( _borderWidth ),
  boundaryModeS( _boundaryModeS ),
  boundaryModeT( _boundaryModeT ),
  boundaryModeR( _boundaryModeR ),
  magnificationFilter( _magnificationFilter ),
  minificationFilter( _minificationFilter ),
  textureCompression( _textureCompression ),
  texturePriority( _texturePriority ),
  generateMipMaps( _generateMipMaps ),
  textureTransferScale( _textureTransferScale ),
  textureTransferBias( _textureTransferBias ),
  propertyChanged( new Field ),
  textureCompareMode( _textureCompareMode ),
  textureCompareFailValue( _textureCompareFailValue ),
  textureType( _textureType ),
  textureFormat( _textureFormat ) {
  type_name = "TextureProperties";

  database.initFields( this );

  anisotropicDegree->setValue( 1.0f );
  borderColor->setValue( RGBA( 0, 0, 0, 0 ) );
  borderWidth->setValue( 0 );
  boundaryModeS->addValidValue( "REPEAT" );
  boundaryModeS->addValidValue( "CLAMP" );
  boundaryModeS->addValidValue( "CLAMP_TO_EDGE" );
  boundaryModeS->addValidValue( "CLAMP_TO_BOUNDARY" );
  boundaryModeS->addValidValue( "MIRRORED_REPEAT" );
  boundaryModeS->setValue( "REPEAT" );
  boundaryModeT->addValidValue( "REPEAT" );
  boundaryModeT->addValidValue( "CLAMP" );
  boundaryModeT->addValidValue( "CLAMP_TO_EDGE" );
  boundaryModeT->addValidValue( "CLAMP_TO_BOUNDARY" );
  boundaryModeT->addValidValue( "MIRRORED_REPEAT" );
  boundaryModeT->setValue( "REPEAT" );
  boundaryModeR->addValidValue( "REPEAT" );
  boundaryModeR->addValidValue( "CLAMP" );
  boundaryModeR->addValidValue( "CLAMP_TO_EDGE" );
  boundaryModeR->addValidValue( "CLAMP_TO_BOUNDARY" );
  boundaryModeR->addValidValue( "MIRRORED_REPEAT" );
  boundaryModeR->setValue( "REPEAT" );
  magnificationFilter->addValidValue( "FASTEST" );
  magnificationFilter->addValidValue( "NEAREST_PIXEL" );
  magnificationFilter->addValidValue( "AVG_PIXEL" );
  magnificationFilter->addValidValue( "DEFAULT" );
  magnificationFilter->addValidValue( "NICEST" );
  magnificationFilter->setValue( "FASTEST" );
  minificationFilter->addValidValue( "FASTEST" );
  minificationFilter->addValidValue( "NEAREST_PIXEL" );
  minificationFilter->addValidValue( "AVG_PIXEL" );
  minificationFilter->addValidValue( "DEFAULT" );
  minificationFilter->addValidValue( "NICEST" );
  minificationFilter->addValidValue( "AVG_PIXEL_AVG_MIPMAP" );
  minificationFilter->addValidValue( "AVG_PIXEL_NEAREST_MIPMAP" );
  minificationFilter->addValidValue( "NEAREST_PIXEL_AVG_MIPMAP" );
  minificationFilter->addValidValue( "NEAREST_PIXEL_NEAREST_MIPMAP" );
  minificationFilter->setValue( "FASTEST" );
  textureCompression->addValidValue( "DEFAULT" );
  textureCompression->addValidValue( "FASTEST" );
  textureCompression->addValidValue( "HIGH" );
  textureCompression->addValidValue( "MEDIUM" );
  textureCompression->addValidValue( "NICEST" );
  textureCompression->addValidValue( "LOW" );
  textureCompression->addValidValue( "BC1" );
  textureCompression->addValidValue( "BC2" );
  textureCompression->addValidValue( "BC3" );
  textureCompression->addValidValue( "BC4" );
  textureCompression->addValidValue( "BC5" );
  textureCompression->addValidValue( "BC6" );
  textureCompression->addValidValue( "BC7" );
  textureCompression->addValidValue( "NONE" );
  textureCompression->setValue( "FASTEST" );
  texturePriority->setValue( 1.0f );
  generateMipMaps->setValue( false );
  textureTransferScale->setValue( Vec4f( 1, 1, 1, 1 ) );
  textureTransferBias->setValue( Vec4f( 0, 0, 0, 0 ) );
  textureCompareMode->addValidValue( "NONE" );
  textureCompareMode->addValidValue( "GEQUAL" );
  textureCompareMode->addValidValue( "LEQUAL" );
  textureCompareMode->setValue( "NONE" );
  textureCompareFailValue->setValue( 0 );
  textureType->addValidValue( "NORMAL" );
  textureType->addValidValue( "2D_RECTANGLE" );
  textureType->addValidValue( "2D_ARRAY" );
  textureType->setValue( "NORMAL" );
  textureFormat->addValidValue( "NORMAL" );
  textureFormat->addValidValue( "INTEGER" );
  textureFormat->addValidValue( "FLOAT" );
  textureFormat->addValidValue( "SRGB" );
  textureFormat->setValue( "NORMAL" );

  propertyChanged->setName( "propertyChanged" );
  anisotropicDegree->route( propertyChanged );
  borderColor->route( propertyChanged );
  borderWidth->route( propertyChanged );
  boundaryModeS->route( propertyChanged );
  boundaryModeT->route( propertyChanged );
  boundaryModeR->route( propertyChanged );
  magnificationFilter->route( propertyChanged );
  minificationFilter->route( propertyChanged );
  textureCompression->route( propertyChanged );
  texturePriority->route( propertyChanged );
  generateMipMaps->route( propertyChanged );
  textureTransferScale->route( propertyChanged );
  textureTransferBias->route( propertyChanged );
  textureCompareMode->route( propertyChanged );
  textureCompareFailValue->route( propertyChanged );
  textureType->route( propertyChanged );
  textureFormat->route( propertyChanged );
}

void TextureProperties::renderTextureProperties( GLenum texture_target, bool texture_provided_mip_maps ) {
  // anisotropicDegree
  H3DFloat anisotropic = anisotropicDegree->getValue();
  if( anisotropic < 1 ) {
    Console(LogLevel::Warning) << "Warning: Invalid anisotropicDegree \"" << anisotropic 
               << "\". Must be greater that 1.0 (in " << getName()
               << ")" << endl;
    
  } else {
    if( GLEW_EXT_texture_filter_anisotropic ) {
      glTexParameterf(texture_target, 
                      GL_TEXTURE_MAX_ANISOTROPY_EXT, anisotropic );
    }
  }
  // border color
  const RGBA &border_color = borderColor->getValue();
  GLfloat c[4];
  c[0] = border_color.r;
  c[1] = border_color.g;
  c[2] = border_color.b;
  c[3] = border_color.a;
  glTexParameterfv( texture_target, GL_TEXTURE_BORDER_COLOR, c );

  // boundary modes

  // S
  const string &s_mode = boundaryModeS->getValue();
  if( s_mode == "CLAMP" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_S, GL_CLAMP );
  } else if( s_mode == "CLAMP_TO_EDGE" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
  } else if( s_mode == "CLAMP_TO_BOUNDARY" ) {
    if( GLEW_ARB_texture_border_clamp ) {
      glTexParameteri( texture_target, GL_TEXTURE_WRAP_S, 
                       GL_CLAMP_TO_BORDER );
    } else {
      Console(LogLevel::Warning) << "Warning: MIRRORED_REPEAT boundary mode not "
                 << "supported by your graphics card (in " << getName()
                 << ")" << endl;
    }
  } else if( s_mode == "MIRRORED_REPEAT" ) {
    if( GLEW_ARB_texture_mirrored_repeat ) {
      glTexParameteri( texture_target, GL_TEXTURE_WRAP_S, 
                       GL_MIRRORED_REPEAT_ARB );
    } else {
      Console(LogLevel::Warning) << "Warning: MIRRORED_REPEAT boundary mode not "
                 << "supported by your graphics card (in " << getName()
                 << ")" << endl;
    }
  } else if( s_mode == "REPEAT" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_S, GL_REPEAT );
  } else {
    Console(LogLevel::Warning) << "Warning: Invalid boundary mode \"" << s_mode 
               << "\" in TextureProperties "
               << " node for texture node(" << getName() << ")." << endl; 
  }

  // T
  const string &t_mode = boundaryModeT->getValue();
  if( t_mode == "CLAMP" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_T, GL_CLAMP );
  } else if( t_mode == "CLAMP_TO_EDGE" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
  } else if( t_mode == "CLAMP_TO_BOUNDARY" ) {
    if( GLEW_ARB_texture_border_clamp ) {
      glTexParameteri( texture_target, GL_TEXTURE_WRAP_T, 
                       GL_CLAMP_TO_BORDER );
    } else {
      Console(LogLevel::Warning) << "Warning: MIRRORED_REPEAT boundary mode not "
                 << "supported by your graphics card (in " << getName()
                 << ")" << endl;
    }
  } else if( t_mode == "MIRRORED_REPEAT" ) {
    if( GLEW_ARB_texture_mirrored_repeat ) {
      glTexParameteri( texture_target, GL_TEXTURE_WRAP_T, 
                       GL_MIRRORED_REPEAT_ARB );
    } else {
      Console(LogLevel::Warning) << "Warning: MIRRORED_REPEAT boundary mode not "
                 << "supported by your graphics card (in " << getName()
                 << ")" << endl;
    }
  } else if( t_mode == "REPEAT" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_T, GL_REPEAT );
  } else {
    Console(LogLevel::Warning) << "Warning: Invalid boundary mode \"" << t_mode 
               << "\" in TextureProperties "
               << " node for texture node(" << getName() << ")." << endl; 
  }


  // R
  const string &r_mode = boundaryModeR->getValue();
  if( r_mode == "CLAMP" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_R, GL_CLAMP );
  } else if( r_mode == "CLAMP_TO_EDGE" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE );
  } else if( r_mode == "CLAMP_TO_BOUNDARY" ) {
    if( GLEW_ARB_texture_border_clamp ) {
      glTexParameteri( texture_target, GL_TEXTURE_WRAP_R, 
                       GL_CLAMP_TO_BORDER );
    } else {
      Console(LogLevel::Warning) << "Warning: MIRRORED_REPEAT boundary mode not "
                 << "supported by your graphics card (in " << getName()
                 << ")" << endl;
    }
  } else if( r_mode == "MIRRORED_REPEAT" ) {
    if( GLEW_ARB_texture_mirrored_repeat ) {
      glTexParameteri( texture_target, GL_TEXTURE_WRAP_R, 
                       GL_MIRRORED_REPEAT_ARB );
    } else {
      Console(LogLevel::Warning) << "Warning: MIRRORED_REPEAT boundary mode not "
                 << "supported by your graphics card (in " << getName()
                 << ")" << endl;
    }
  } else if( r_mode == "REPEAT" ) {
    glTexParameteri( texture_target, GL_TEXTURE_WRAP_R, GL_REPEAT );
  } else {
    Console(LogLevel::Warning) << "Warning: Invalid boundary mode \"" << r_mode 
               << "\" in TextureProperties "
               << " node for texture node(" << getName() << ")." << endl; 
  }

  // magnification filter
  const string &mag_filter = 
    magnificationFilter->getValue();

  if( mag_filter == "NEAREST_PIXEL" ||
      mag_filter == "FASTEST" )
    glTexParameteri( texture_target, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
  else if( mag_filter == "AVG_PIXEL" ||
           mag_filter == "DEFAULT" ||
           mag_filter == "NICEST" ) {
    glTexParameteri( texture_target, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  } else  {
    Console(LogLevel::Warning) << "Warning: Invalid magnification filter \"" << mag_filter 
               << "\" in TextureProperties "
               << " node for texture node(" << getName() << ")." << endl; 
  }

  // minification filter
  const string &min_filter = 
    minificationFilter->getValue();

  if( min_filter == "NEAREST_PIXEL" ||
      min_filter == "FASTEST" )
    glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
  else if( min_filter == "AVG_PIXEL" ||
           min_filter == "DEFAULT" ||
           min_filter == "NICEST" ) {
    glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  } else if( min_filter == "AVG_PIXEL_AVG_MIPMAP") {
    if( texture_provided_mip_maps ) {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER,
                       GL_LINEAR_MIPMAP_LINEAR );
    } else {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    }
  } else if( min_filter == "AVG_PIXEL_NEAREST_MIPMAP") {
    if( texture_provided_mip_maps ) {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER,
                       GL_LINEAR_MIPMAP_NEAREST );
    } else {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    }
  } else if( min_filter == "NEAREST_PIXEL_AVG_MIPMAP") {
    if( texture_provided_mip_maps ) {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER,
                       GL_NEAREST_MIPMAP_LINEAR );
    } else {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    }
  } else if( min_filter == "NEAREST_PIXEL_NEAREST_MIPMAP") {
    if( texture_provided_mip_maps ) {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER, 
                       GL_NEAREST_MIPMAP_NEAREST );
    } else {
      glTexParameteri( texture_target, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    }
  } else {
    Console(LogLevel::Warning) << "Warning: Invalid minification filter \"" << min_filter 
               << "\" in TextureProperties "
               << " node for texture node(" << getName() << ")." << endl; 
  }

  // priority
  H3DFloat priority = texturePriority->getValue();
  if( priority < 0 || priority > 1 ) {
    Console(LogLevel::Warning) << "Warning: Invalid texturePriority \"" << priority
               << "\". Must be in range [0, 1] (in " << getName()
               << ")" << endl;
        
  } else {
    glTexParameterf(texture_target, 
                    GL_TEXTURE_PRIORITY, priority );
  }

  // compression
  if( GLEW_ARB_texture_compression ) {
    // compression
    const string &compression = 
      textureCompression->getValue();

    if( compression == "DEFAULT" ) {
      glHint( GL_TEXTURE_COMPRESSION_HINT_ARB, GL_DONT_CARE );
    } else if( compression == "FASTEST" ||
               compression == "HIGH" ||
               compression == "MEDIUM" ) {
      glHint( GL_TEXTURE_COMPRESSION_HINT_ARB, GL_FASTEST );
    } else if( compression == "NICEST" ||
               compression == "LOW"  ) {
      glHint( GL_TEXTURE_COMPRESSION_HINT_ARB, GL_NICEST );
    }
  }

  // textureCompareMode
  if( GLEW_ARB_shadow ) {
    const string &compare_mode = textureCompareMode->getValue();
    if( compare_mode == "LEQUAL" ) {
      glTexParameteri( texture_target, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB );
      glTexParameteri( texture_target, GL_TEXTURE_COMPARE_FUNC_ARB, GL_LEQUAL );
    } else if( compare_mode == "GEQUAL" ) {
      glTexParameteri( texture_target, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB );
      glTexParameteri( texture_target, GL_TEXTURE_COMPARE_FUNC_ARB, GL_GEQUAL );
    } else {
      if( compare_mode != "NONE" ) {
        Console(LogLevel::Warning) << "Warning: Invalid textureCompareMode: " << compare_mode 
                   << " in TextureProperties node for texture node(" << getName() << ")." << endl; 
      }
      glTexParameteri( texture_target, GL_TEXTURE_COMPARE_MODE_ARB, GL_NONE );
    }
  } else {
    Console(LogLevel::Warning) << "Warning: textureCompareMode is not supported by your graphics card. " 
               << "Requires the ARB_shadow extension (in TextureProperties node). " << endl;
  }

  GLfloat fail_value = textureCompareFailValue->getValue();
  // textureCompareFailValue
  if( GLEW_ARB_shadow_ambient ) {
    glTexParameterf( texture_target, GL_TEXTURE_COMPARE_FAIL_VALUE_ARB, fail_value );
  } else {
    if( fail_value != 0 ) {
      Console(LogLevel::Warning) << "Warning: textureCompareFailValue is not supported by your graphics card. " 
                 << "Requires the ARB_shadow_ambient extension (in TextureProperties node). " << endl;
    }
  }

}

bool TextureProperties::glInternalFormat( Image *image, GLint &internal_format ) {
  string texture_format = textureFormat->getValue();
  if( texture_format != "NORMAL" ) {
    if( GLEW_EXT_texture_integer && texture_format == "INTEGER" ) {
      switch( image->pixelType() ) {
        // Do we need to check for RATIONAL pixelComponentType here or can that be
        // mixed without problem
        // as long as pixel format is correctly set. Have to test later.
        case Image::LUMINANCE:
          switch( image->pixelComponentType() ) {
            case Image::UNSIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:  internal_format = GL_LUMINANCE8UI_EXT; return true;
                case 16: internal_format = GL_LUMINANCE16UI_EXT; return true;
                case 32: internal_format = GL_LUMINANCE32UI_EXT; return true;
                default: return false;
              }
            case Image::SIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:  internal_format = GL_LUMINANCE8I_EXT; return true;
                case 16: internal_format = GL_LUMINANCE16I_EXT; return true;
                case 32: internal_format = GL_LUMINANCE32I_EXT; return true;
                default: return false;
              }
            default: return false;
          }
        case Image::LUMINANCE_ALPHA:
          switch( image->pixelComponentType() ) {
            case Image::UNSIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:
                  internal_format = GL_LUMINANCE_ALPHA8UI_EXT; return true;
                case 16:
                  internal_format = GL_LUMINANCE_ALPHA16UI_EXT; return true;
                case 32:
                  internal_format = GL_LUMINANCE_ALPHA32UI_EXT; return true;
                default: return false;
              }
            case Image::SIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:
                  internal_format = GL_LUMINANCE_ALPHA8I_EXT; return true;
                case 16:
                  internal_format = GL_LUMINANCE_ALPHA16I_EXT; return true;
                case 32:
                  internal_format = GL_LUMINANCE_ALPHA32I_EXT; return true;
                default: return false;
              }
            default: return false;
          }
        case Image::RGB:
        case Image::BGR:
          switch( image->pixelComponentType() ) {
            case Image::UNSIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:  internal_format = GL_RGB8UI_EXT; return true;
                case 16: internal_format = GL_RGB16UI_EXT; return true;
                case 32: internal_format = GL_RGB32UI_EXT; return true;
                default: return false;
              }
            case Image::SIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:  internal_format = GL_RGB8I_EXT; return true;
                case 16: internal_format = GL_RGB16I_EXT; return true;
                case 32: internal_format = GL_RGB32I_EXT; return true;
                default: return false;
              }
            default: return false;
          }
        case Image::RGBA:
        case Image::BGRA:
          switch( image->pixelComponentType() ) {
            case Image::UNSIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:  internal_format = GL_RGBA8UI_EXT; return true;
                case 16: internal_format = GL_RGBA16UI_EXT; return true;
                case 32: internal_format = GL_RGBA32UI_EXT; return true;
                default: return false;
              }
            case Image::SIGNED:
              switch( image->bitsPerPixel() ) {
                case 8:  internal_format = GL_RGBA8I_EXT; break;
                case 16: internal_format = GL_RGBA16I_EXT; break;
                case 32: internal_format = GL_RGBA32I_EXT; break;
                default: return false;
              }
            default: return false;
          }
        default: return false;
      }
    } else if( GLEW_ARB_texture_float && texture_format == "FLOAT" ) {
      switch( image->pixelType() ) {
        // Do we need to check for RATIONAL pixelComponentType here or can that be
        // mixed without problem
        // as long as pixel format is correctly set. Have to test later.
        case Image::LUMINANCE:
          switch( image->pixelComponentType() ) {
            case Image::RATIONAL:
              switch( image->bitsPerPixel() ) {
                case 16: internal_format = GL_LUMINANCE16F_ARB; return true;
                case 32: internal_format = GL_LUMINANCE32F_ARB; return true;
                default: return false;
              }
            default: return false;
          }
        case Image::LUMINANCE_ALPHA:
          switch( image->pixelComponentType() ) {
            case Image::RATIONAL:
              switch( image->bitsPerPixel() ) {
                case 16: internal_format = GL_LUMINANCE_ALPHA16F_ARB; break;
                case 32: internal_format = GL_LUMINANCE_ALPHA32F_ARB; break;
                default: return false;
              }
            default: return false;
          }
        case Image::RGB:
        case Image::BGR:
          switch( image->pixelComponentType() ) {
            case Image::RATIONAL:
              switch( image->bitsPerPixel() ) {
                case 16: internal_format = GL_RGB16F_ARB; break;
                case 32: internal_format = GL_RGB32F_ARB; break;
                default: return false;
              }
            default: return false;
          }
        case Image::RGBA:
        case Image::BGRA:
          switch( image->pixelComponentType() ) {
            case Image::RATIONAL:
              switch( image->bitsPerPixel() ) {
                case 16: internal_format = GL_RGBA16F_ARB; break;
                case 32: internal_format = GL_RGBA32F_ARB; break;
                default: return false;
              }
            default: return false;
          }
        default: return false;
      }
#ifdef GL_EXT_texture_sRGB
    } else if( GLEW_EXT_texture_sRGB && texture_format == "SRGB" ) {
      if( image->compressionType() == Image::NO_COMPRESSION ) {
        switch( image->pixelType() ) {
        case Image::RGB:
        case Image::BGR:
          switch( image->pixelComponentType() ) {
          case Image::UNSIGNED:
            switch( image->bitsPerPixel() ) {
            case 24:  internal_format = GL_SRGB8_EXT;  return true;
            default: return false;
            }
          default: return false;
          }
        case Image::RGBA:
        case Image::BGRA:
          switch( image->pixelComponentType() ) {
          case Image::UNSIGNED:
            switch( image->bitsPerPixel() ) {
            case 32:  internal_format = GL_SRGB8_ALPHA8_EXT; return true;
            default: return false;
            }
          default: return false;
          }
        default: return false;
        }
      } else {
        // Compressed sRGB modes

        switch( image->compressionType() ) {

#ifdef GL_EXT_texture_compression_s3tc
        case Image::BC1:
          if( GLEW_EXT_texture_compression_s3tc ) {
            switch( image->pixelType() ) {
            case Image::RGB:
              internal_format = GL_COMPRESSED_SRGB_S3TC_DXT1_EXT; return true;
            case Image::RGBA:
              internal_format = GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT; return true;
            default:
              return false;
            }
          }
          break;

        case Image::BC2:
          if( GLEW_EXT_texture_compression_s3tc ) {
            internal_format = GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT; return true;
          }
          break;

        case Image::BC3:
          if( GLEW_EXT_texture_compression_s3tc ) {
            internal_format = GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT; return true;
          }
          break;
#endif
        default:
          return false;
        }
      }
    }
#else
    }
#endif
  }
  return false;
}

bool TextureProperties::glPixelFormat( Image *image, GLenum &pixel_format ) {
  if( GLEW_EXT_texture_integer && textureFormat->getValue() == "INTEGER" ) {
    switch( image->pixelType() ) {
      case Image::LUMINANCE:
        pixel_format = GL_LUMINANCE_INTEGER_EXT;
        switch( image->bitsPerPixel() ) {
          case 8:
          case 16:
          case 32: return true;
          default: return false;
        }
      case Image::LUMINANCE_ALPHA:
        pixel_format = GL_LUMINANCE_ALPHA_INTEGER_EXT;
        switch( image->bitsPerPixel() ) {
          case 8:
          case 16:
          case 32: return true;
          default: return false;
        }
      case Image::RGB:
      case Image::BGR:
        pixel_format = GL_RGB_INTEGER_EXT;
        switch( image->bitsPerPixel() ) {
          case 8:
          case 16:
          case 32: return true;
          default: return false;
        }
      case Image::RGBA:
      case Image::BGRA:
        pixel_format = GL_RGBA_INTEGER_EXT;
        switch( image->bitsPerPixel() ) {
          case 8:
          case 16:
          case 32: return true;
          default: return false;
        }
      default: return false;
    }
  }
  return false;
}

bool H3D::TextureProperties::match( const TextureProperties* tp ) {
  // check equalities
  if( !tp ) {
    return false;
  }
  if( anisotropicDegree->getValue() != tp->anisotropicDegree->getValue() ) {
    return false;
  }
  if( borderColor->getValue() != tp->borderColor->getValue() ) {
    return false;
  }
  if( borderWidth->getValue() != tp->borderWidth->getValue() ) {
    return false;
  }
  if( boundaryModeS->getValue() != tp->boundaryModeS->getValue() ) {
    return false;
  }
  if( boundaryModeT->getValue() != tp->boundaryModeT->getValue() ) {
    return false;
  }
  if( boundaryModeR->getValue() != tp->boundaryModeR->getValue() ) {
    return false;
  }
  if( magnificationFilter->getValue() != tp->magnificationFilter->getValue() ) {
    return false;
  }
  if( minificationFilter->getValue() != tp->minificationFilter->getValue() ) {
    return false;
  }
  if( textureCompression->getValue() != tp->textureCompression->getValue() ) {
    return false;
  }
  if( texturePriority->getValue() != tp->texturePriority->getValue() ) {
    return false;
  }
  if( generateMipMaps->getValue() != tp->generateMipMaps->getValue() ) {
    return false;
  }
  if( textureTransferScale->getValue() != tp->textureTransferScale->getValue() ) {
    return false;
  }
  if( textureTransferBias->getValue() != tp->textureTransferBias->getValue() ) {
    return false;
  }
  if( textureCompareMode->getValue() != tp->textureCompareMode->getValue() ) {
    return false;
  }
  if( textureCompareFailValue->getValue() != tp->textureCompareFailValue->getValue() ) {
    return false;
  }
  if( textureType->getValue() != tp->textureType->getValue() ) {
    return false;
  }
  if( textureFormat->getValue() != tp->textureFormat->getValue() ) {
    return false;
  }
  return true;
}


