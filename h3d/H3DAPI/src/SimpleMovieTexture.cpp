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
/// \file SimpleMovieTexture.cpp
/// \brief CPP file for SimpleMovieTexture.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/SimpleMovieTexture.h>

using namespace H3D;

H3DNodeDatabase SimpleMovieTexture::database( 
        "SimpleMovieTexture", 
        &(newInstance< SimpleMovieTexture > ),
        typeid( SimpleMovieTexture ),
        &H3DVideoTextureNode::database 
        );

namespace SimpleMovieTextureInternals {
  FIELDDB_ELEMENT( SimpleMovieTexture, play, INPUT_ONLY )
  FIELDDB_ELEMENT( SimpleMovieTexture, stop, INPUT_ONLY )
  FIELDDB_ELEMENT( SimpleMovieTexture, pause, INPUT_ONLY )
  FIELDDB_ELEMENT( SimpleMovieTexture, duration, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SimpleMovieTexture, rate, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SimpleMovieTexture, videoWidth, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SimpleMovieTexture, videoHeight, OUTPUT_ONLY )
  FIELDDB_ELEMENT( SimpleMovieTexture, loop, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SimpleMovieTexture, playAudio, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SimpleMovieTexture, url, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SimpleMovieTexture, elapsedTime, INPUT_OUTPUT )
}


SimpleMovieTexture::SimpleMovieTexture( 
                                   Inst< DisplayList > _displayList,
                                   Inst< SFNode  > _metadata,
                                   Inst< SFBool  > _repeatS,
                                   Inst< SFBool  > _repeatT,
                                   Inst< SFBool  > _scaleToP2,
                                   Inst< SFImage > _image,
                                   Inst< SFTextureProperties > _textureProperties,
                                   Inst< SFBool > _play,
                                   Inst< SFBool  > _stop,
                                   Inst< SFBool > _pause,
                                   Inst< SFTime      > _duration,
                                   Inst< SFFloat     > _rate,
                                   Inst< SFBool      > _playAudio,
                                   Inst< SFBool      > _loop,
                                   Inst< SFInt32     > _width,
                                   Inst< SFInt32     > _height,
                                   Inst< MFString >  _url,
                                   Inst< SFTime   > _elapsedTime ):
  H3DVideoTextureNode( _displayList, _metadata, _repeatS, _repeatT, _scaleToP2,
                       _image, _textureProperties ),
  X3DUrlObject( _url ),
  play( _play ),
  stop( _stop ),
  pause( _pause ),
  duration( _duration ),
  rate( _rate ),
  playAudio( _playAudio ),
  loop( _loop ),
  videoWidth( _width ),
  videoHeight( _height ),
  elapsedTime( _elapsedTime ),
  fieldUpdater( new FieldUpdater ),
  decoderManager( new DecoderManager ) {
  type_name = "SimpleMovieTexture";
  database.initFields( this );
  
  decoderManager->setOwner( this );
  decoderManager->setName( "decoderManager" );
  fieldUpdater->setOwner( this );
  fieldUpdater->setName( "fieldUpdater" );

  rate->setValue( 1, id );
  playAudio->setValue( true, id );
  loop->setValue( false, id );
  videoWidth->setValue( 0, id );
  videoHeight->setValue( 0, id );
  duration->setValue( 0, id );
  elapsedTime->setValue( 0, id );

  play->routeNoEvent( decoderManager, id );
  stop->routeNoEvent( decoderManager, id );
  pause->routeNoEvent( decoderManager, id );
  loop->routeNoEvent( decoderManager, id );
  url->routeNoEvent( decoderManager, id );
  rate->route( decoderManager, id );
  elapsedTime->route( decoderManager, id );

  Scene::time->route( fieldUpdater );
}

void SimpleMovieTexture::FieldUpdater::update() {
  SimpleMovieTexture *tex = static_cast< SimpleMovieTexture * >( getOwner() );
  H3DTime elapsed_time = 0;
  if( tex->decoder.get() ) {
    elapsed_time = tex->decoder->getPosition();
  }
  if( tex->elapsedTime->getValue() != elapsed_time ) {
    // removing route while setting value to avoid decoderManager to update
    tex->elapsedTime->unroute( tex->decoderManager );
    tex->elapsedTime->setValue( elapsed_time, tex->id );
    tex->elapsedTime->routeNoEvent( tex->decoderManager );
  }
}

void SimpleMovieTexture::DecoderManager::update() {
  SimpleMovieTexture *tex = static_cast< SimpleMovieTexture * >( getOwner() );
  if( event.ptr == routes_in[0] ) {
    // play
    if( static_cast< SFBool * >( routes_in[0] )->getValue( tex->id ) ) {
      if( tex->decoder.get() )
        tex->decoder->startPlaying();
    }
  } else if( event.ptr == routes_in[1] ) {
    // stop
    if( static_cast< SFBool * >( routes_in[1] )->getValue( tex->id ) ) {
      if( tex->decoder.get() )
        tex->decoder->stopPlaying();
    }
  } else if( event.ptr == routes_in[2] ) {
    // pause
    if( static_cast< SFBool * >( routes_in[2] )->getValue( tex->id ) ) {
      if( tex->decoder.get() )
        tex->decoder->pausePlaying();
    }
  } else if( event.ptr == routes_in[3] ) {
    // loop
    if( tex->decoder.get() )
      tex->decoder->setLooping( static_cast< SFBool * >( routes_in[3] )->getValue( tex->id ) );
  } else if( event.ptr == routes_in[4] ) {
    // url
    MFString *urls = static_cast< MFString * >( routes_in[4] );
    for( MFString::const_iterator i = urls->begin(); i != urls->end(); ++i ) {
      string _url = tex->resolveURLAsFile( *i );
      H3DVideoClipDecoderNode *_decoder = 
        H3DVideoClipDecoderNode::getSupportedDecoder( _url );
      if( _decoder ) {
        if( _decoder->loadClip( _url ) ) {
          tex->decoder.reset( _decoder );
          tex->setURLUsed( *i );
          tex->videoWidth->setValue( _decoder->getFrameWidth(), tex->id );
          tex->videoHeight->setValue( _decoder->getFrameHeight(), tex->id );
          tex->duration->setValue( _decoder->getDuration(), tex->id );
          return;
        }
      } else {
        tex->decoder.reset( NULL );
        tex->videoWidth->setValue( 0, tex->id );
        tex->videoHeight->setValue( 0, tex->id );
        tex->duration->setValue( 0, tex->id );
      }
    }

    Console(LogLevel::Error) << "Warning: None of the urls in SimpleMovieTexture with url [";
    for( MFString::const_iterator i = urls->begin(); i != urls->end(); ++i ) {  
      Console(LogLevel::Error) << " \"" << *i << "\"";
    }
    Console(LogLevel::Error) << "] could be loaded. Either they don't exist or the file format "
             << "is not supported by any H3DVideoClipDecoderNodes that is available "
               << "(in " << getOwner()->getName() << ")" << endl;

    tex->setURLUsed( "" );
  } else if( event.ptr == routes_in[5] ) {
    // rate
    H3DFloat _rate =static_cast< SFFloat * >( routes_in[5] )->getValue( tex->id );
    if( tex->decoder.get() ) {
      if(! tex->decoder->setRate( _rate ) ) {
        Console(LogLevel::Warning) << "Warning: Unable to set rate to " << _rate
                   << ". Rate not supported by decoder ( " 
                   << tex->decoder->getName() << " in " 
                   << tex->getName() << endl;
      }
    }
  } else if( event.ptr == routes_in[6] ) {
    // position
    H3DTime elapsed_time =static_cast< SFTime * >( routes_in[6] )->getValue( tex->id );
    if( tex->decoder.get() ) {
      tex->decoder->setPosition( elapsed_time );
    }
  }
}
  
