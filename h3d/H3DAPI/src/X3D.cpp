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
/// \file X3D.cpp
/// \brief CPP file for functions needed when parsing X3D code.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/X3D.h>

#ifdef HAVE_XERCES
#include <H3D/X3DSAX2Handlers.h>
#include <H3D/IStreamInputSource.h>
#endif

#include <H3DUtil/ResourceResolver.h>
#include <H3D/VrmlParser.h>
#include <H3D/X3DGeometryNode.h>
#include <sstream>

#ifdef HAVE_ZLIB
#include <zlib.h>

#define ZIP_MAGIC_NR 0x8B1F

#endif

using namespace H3D;

Group* X3D::createX3DFromString( const string &str,
                                 DEFNodes *dn,
                                 DEFNodes *exported_nodes,
                                 PrototypeVector *prototypes  ) {
  if ( isVRML( str ) )
    return createVRMLFromString( str, dn, exported_nodes, prototypes );
  Group *g = new Group;
  try{
    AutoRef< Node > n = createX3DNodeFromString(str, dn,
      exported_nodes,
      prototypes);
    if (n.get()) {
      g->children->push_back(n.get());
    }
  }
  catch (...){
    delete g;
    g = NULL;
    throw;
  }
  return g;
}

Group* X3D::createX3DFromURL( const string &url,
                              DEFNodes *dn,
                              DEFNodes *exported_nodes,
                              PrototypeVector *prototypes,
                              bool change_base_path_during_parsing ) {
  H3DTIMER_BEGIN( "createX3DFromURL (" + url + ")", "PARSING" ) 
  // First try to resolve the url to file contents and load via string buffer
  // Otherwise fallback on using temp files
  string url_contents= ResourceResolver::resolveURLAsString ( url );
  if ( url_contents != "" ) {
    // Ensure that we set the base URL even when returning file contents
    string old_base = ResourceResolver::getBaseURL();
    string::size_type to = url.find_last_of( "/\\" );
    if( to != string::npos ) {
      string base = url.substr( 0, to + 1 );
      string new_base = old_base + base;
      if( ResourceResolver::getURNResolver() ) {
        base = ResourceResolver::getURNResolver()->resolveURN( base );
        if( base.size() > 1 && base[1] == ':' ) {
          new_base = base;
        }
      }
      ResourceResolver::setBaseURL( new_base );
    }

    Group* g= createX3DFromString ( url_contents, dn, exported_nodes, prototypes );

    ResourceResolver::setBaseURL( old_base );
    H3DTIMER_END( "createX3DFromURL (" + url + ")" ) 
    return g;
  }

  bool is_tmp_file;
  string resolved_url = ResourceResolver::resolveURLAsFile( url, 
    &is_tmp_file );

  string::size_type pos = resolved_url.find_last_of( "/\\" );
  string path = resolved_url.substr( 0, pos + 1 );
  if( is_tmp_file ) {
    pos = url.find_last_of( "/\\" );
    path = url.substr( 0, pos + 1 );
  }
  string old_base = ResourceResolver::getBaseURL();

#ifdef H3D_WINDOWS
  // needed when running H3DAPI as a plugin to a web-browser
  // and if url points to a local file.
  // Needed for plugin for IE
  string remove_string = "file:///";
  if( path.find( remove_string ) == 0 ) {
    path = path.substr( remove_string.size(), pos + 1 );
  }

  // Needed for plugin for Netscape ( tested on Opera )
  remove_string = "file://localhost/";
  if( path.find( remove_string ) == 0 ) {
    path = path.substr( remove_string.size(), pos + 1 );
  }
#endif // H3D_WINDOWS

  if( change_base_path_during_parsing )
    ResourceResolver::setBaseURL( path ); 

#ifdef HAVE_XERCES
  auto_ptr< SAX2XMLReader > parser( getNewXMLParser() );
  X3DSAX2Handlers handler( dn, exported_nodes, prototypes );
  parser->setContentHandler(&handler);
  parser->setErrorHandler(&handler); 
  parser->setLexicalHandler( &handler );
#endif

  if( resolved_url == "" ) {
#ifdef HAVE_XERCES
    parser->parse( url.c_str() );
#endif

  } else {

#ifdef HAVE_ZLIB
    // check if zip-file
    ifstream ifs( resolved_url.c_str(), ios::binary );
    unsigned short magic_nr;
    ifs.read( (char *)&magic_nr, sizeof( unsigned short ) );
    bool file_exists = ifs.good();
    ifs.close();

    // then unpack it
    if( file_exists && magic_nr == ZIP_MAGIC_NR ) {
      string tmp_file = ResourceResolver::getTmpFileName();
      if( tmp_file != "" ) { 
        gzFile in  = gzopen( resolved_url.c_str(),"rb");
        ofstream ofs( tmp_file.c_str(), ios::binary );
        if( in && ofs.good() ) {
          char buf[ 16384 ];  
          int len = 0;
          while( (len = gzread( in, buf, sizeof(buf))) != 0 ) {
            ofs.write( buf, len );
          }
          gzclose(in);
          ofs.close();
          if( is_tmp_file ) 
            ResourceResolver::releaseTmpFileName( resolved_url );
          resolved_url = tmp_file;
          is_tmp_file = true;
        }
      }
    }
#endif


    ifstream istest( resolved_url.c_str() );
    if ( isVRML( istest ) ) {
      istest.close();
      Group *g = createVRMLFromURL( resolved_url, dn,
                                    exported_nodes, prototypes,
                                    change_base_path_during_parsing );
      if( is_tmp_file ) 
        ResourceResolver::releaseTmpFileName( resolved_url );
      ResourceResolver::setBaseURL( old_base );
      H3DTIMER_END( "createX3DFromURL (" + url + ")" ) 
      return g;
    }
    istest.close();

    #ifdef HAVE_XERCES
    XERCES_CPP_NAMESPACE_USE
    // else...
    ifstream is( resolved_url.c_str() );
    unique_ptr< XMLCh > url_ch(new XMLCh[url.size() + 1]);
    for(unsigned int i = 0; i < url.size(); ++i) {
      url_ch.get()[i] = url[i];
    }
    url_ch.get()[url.size()] = '\0';
    try {
      parser->parse(IStreamInputSource(is, url_ch.get()));
    } catch(...) {
      H3DTIMER_END("createX3DFromURL (" + url + ")")
        throw;
    }
    is.close();
#endif   
  }
  if( is_tmp_file ) 
    ResourceResolver::releaseTmpFileName( resolved_url );
  ResourceResolver::setBaseURL( old_base );

#ifdef HAVE_XERCES
  Group *g = new Group;
  try{
    AutoRef< Node > n = handler.getResultingNode();
    if (n.get()) {
      g->children->push_back(n.get());
    }
    H3DTIMER_END("createX3DFromURL (" + url + ")")
  }
  catch (...){
    delete g;
    g = NULL;
    throw;
  }
  return g;
#else
  Console(LogLevel::Warning) << "H3D API compiled without HAVE_XERCES flag. X3D-XML files "
       << "are not supported" << endl;
  return 0;
#endif
}

Group* X3D::createX3DFromStream( istream &is, 
                                 DEFNodes *dn,
                                 DEFNodes *exported_nodes,
                                 PrototypeVector *prototypes,
                                 const string &system_id ) {
  Group *g = new Group;
  try{
    AutoRef< Node > n = createX3DNodeFromStream(is, dn, exported_nodes,
      prototypes, system_id);
    if (n.get()) {
      g->children->push_back(n.get());
    }
  }
  catch (...){
    delete g;
    g = NULL;
    throw;
  }
  return g;
}


AutoRef< Node > X3D::createX3DNodeFromString( const string &str,
                                              DEFNodes *dn,
                                              DEFNodes *exported_nodes,
                                              PrototypeVector *prototypes ) {
  if ( isVRML( str ) )
    return createVRMLNodeFromString( str, dn, exported_nodes, prototypes );
  else {
#ifdef HAVE_XERCES
    H3DTIMER_BEGIN( "createX3DNodeFromString", "PARSING" ) 
    auto_ptr< SAX2XMLReader > parser( getNewXMLParser() );
    X3DSAX2Handlers handler( dn, exported_nodes, prototypes );
    stringstream s;
    s << str;
    parser->setContentHandler(&handler);
    parser->setErrorHandler(&handler); 
    parser->setLexicalHandler( &handler );
    parser->parse( IStreamInputSource( s, (const XMLCh*)L"<string input>" ) );
    H3DTIMER_END( "createX3DNodeFromString") 
    return handler.getResultingNode();
#else
  Console(LogLevel::Warning) << "H3D API compiled without HAVE_XERCES flag. X3D-XML files "
       << "are not supported" << endl;
  return AutoRef< Node >(NULL);
#endif
  }
}

AutoRef< Node > X3D::createX3DNodeFromURL( const string &url,
                                           DEFNodes *dn,
                                           DEFNodes *exported_nodes,
                                           PrototypeVector *prototypes,
                                           bool change_base_path_during_parsing ) {
  H3DTIMER_BEGIN( "createX3DNodeFromURL(" + url + ")", "PARSING" ) 
  // First try to resolve the url to file contents and load via string buffer
  // Otherwise fallback on using temp files
  string url_contents= ResourceResolver::resolveURLAsString ( url );
  if ( url_contents != "" ) {
    // Ensure that we set the base URL even when returning file contents
    string old_base = ResourceResolver::getBaseURL();
    string::size_type to = url.find_last_of( "/\\" );
    if ( to != string::npos ) {
      string base = url.substr ( 0, to+1 );
      string new_base = old_base + base;
      if( ResourceResolver::getURNResolver() ) {
        base = ResourceResolver::getURNResolver()->resolveURN( base );
        if( base.size() > 1 && base[1] == ':' ) {
          new_base = base;
        }
      }
      ResourceResolver::setBaseURL( new_base );
    }
      
     AutoRef< Node > n = createX3DNodeFromString ( url_contents, dn, exported_nodes, prototypes );
    
    ResourceResolver::setBaseURL( old_base );
    H3DTIMER_END( "createX3DNodeFromURL(" + url + ")" )
    return n;
  }

  bool is_tmp_file;
  string resolved_url = ResourceResolver::resolveURLAsFile( url, 
                                                            &is_tmp_file );
  
  string::size_type pos = resolved_url.find_last_of( "/\\" );
  string path = resolved_url.substr( 0, pos + 1 );
  if( is_tmp_file ) {
    pos = url.find_last_of( "/\\" );
    path = url.substr( 0, pos + 1 );
  }
  string old_base = ResourceResolver::getBaseURL();

#ifdef H3D_WINDOWS
  // needed when running H3DAPI as a plugin to a web-browser
  // and if url points to a local file.
  // Needed for plugin for IE
  string remove_string = "file:///";
  if( path.find( remove_string ) == 0 ) {
    path = path.substr( remove_string.size(), pos + 1 );
  }

  // Needed for plugin for Netscape ( tested on Opera )
  remove_string = "file://localhost/";
  if( path.find( remove_string ) == 0 ) {
    path = path.substr( remove_string.size(), pos + 1 );
  }
#endif // H3D_WINDOWS

  if( change_base_path_during_parsing )
    ResourceResolver::setBaseURL( path ); 

#ifdef HAVE_XERCES
  auto_ptr< SAX2XMLReader > parser( getNewXMLParser() );
  X3DSAX2Handlers handler( dn, exported_nodes, prototypes );
  parser->setContentHandler(&handler);
  parser->setErrorHandler(&handler); 
  parser->setLexicalHandler( &handler );
#endif

  if( resolved_url == "" ) {
#ifdef HAVE_XERCES
    parser->parse( url.c_str() );
#endif

  } else {
 
#ifdef HAVE_ZLIB
    // check if zip-file
    ifstream ifs( resolved_url.c_str(), ios::binary );
    unsigned short magic_nr;
    ifs.read( (char *)&magic_nr, sizeof( unsigned short ) );
    bool file_exists = ifs.good();
    ifs.close();

    // then unpack it
    if( file_exists && magic_nr == ZIP_MAGIC_NR ) {
      string tmp_file = ResourceResolver::getTmpFileName();
      if( tmp_file != "" ) { 
        gzFile in  = gzopen( resolved_url.c_str(),"rb");
        ofstream ofs( tmp_file.c_str(), ios::binary );
        if( in && ofs.good() ) {
          char buf[ 16384 ];  
          int len = 0;
          while( (len = gzread( in, buf, sizeof(buf))) != 0 ) {
            ofs.write( buf, len );
          }
          gzclose(in);
          ofs.close();
          if( is_tmp_file ) 
            ResourceResolver::releaseTmpFileName( resolved_url );
          resolved_url = tmp_file;
          is_tmp_file = true;
        }
      }
    }
#endif


    ifstream istest( resolved_url.c_str() );
    if ( isVRML( istest ) ) {
      istest.close();
      // reset base to what it was before since createVRMLNodeFromURL
      // will to its own resolving.
      ResourceResolver::setBaseURL( old_base );
      AutoRef< Node > n = createVRMLNodeFromURL( resolved_url, dn,
                                                 exported_nodes, prototypes,
                                                 change_base_path_during_parsing );
      if( is_tmp_file ) 
        ResourceResolver::releaseTmpFileName( resolved_url );
      ResourceResolver::setBaseURL( old_base );
      H3DTIMER_END( "createX3DNodeFromURL(" + url + ")" )
      return n;
    }
    istest.close();

#ifdef HAVE_XERCES
    XERCES_CPP_NAMESPACE_USE
    // else...
    ifstream is( resolved_url.c_str() );
    unique_ptr< XMLCh > url_ch(new XMLCh[url.size() + 1]);
    for(size_t i = 0; i < url.size(); ++i) {
      url_ch.get()[i] = url[i];
    }
    url_ch.get()[url.size()] = '\0';
    parser->parse(IStreamInputSource(is, url_ch.get()));
    is.close();
#endif   
  }
  if( is_tmp_file ) 
    ResourceResolver::releaseTmpFileName( resolved_url );
  ResourceResolver::setBaseURL( old_base );
  
  H3DTIMER_END( "createX3DNodeFromURL(" + url + ")" )
 
#ifdef HAVE_XERCES
  return handler.getResultingNode();
#else
  Console(LogLevel::Warning) << "H3D API compiled without HAVE_XERCES flag. X3D-XML files "
       << "are not supported" << endl;
  return AutoRef< Node >(NULL);
#endif
}

AutoRef< Node > X3D::createX3DNodeFromStream( istream &is, 
                                              DEFNodes *dn,
                                              DEFNodes *exported_nodes,
                                              PrototypeVector *prototypes,
                                              const string & system_id ) {
#ifdef HAVE_XERCES
  auto_ptr< SAX2XMLReader > parser( getNewXMLParser() );
  X3DSAX2Handlers handler( dn, exported_nodes, prototypes );
  parser->setContentHandler(&handler);
  parser->setErrorHandler(&handler);
  parser->setLexicalHandler( &handler ); 
  unique_ptr<XMLCh> system_id_ch(new XMLCh[system_id.size() + 1]);
  for(size_t i = 0; i < system_id.size(); ++i) {
    system_id_ch.get()[i] = system_id[i];
  }
  system_id_ch.get()[system_id.size()] = '\0';
  parser->parse(IStreamInputSource(is, system_id_ch.get()));
  return handler.getResultingNode();
#else
  Console(LogLevel::Warning) << "H3D API compiled without HAVE_XERCES flag. X3D-XML files "
       << "are not supported" << endl;
  return AutoRef< Node >( NULL );
#endif
}

void X3D::writeNodeAsX3D( ostream& os, 
        Node *node, 
        const string &container_field ) {
  std::set< Node *> visited_nodes;
  writeNodeAsX3DHelp( os, node, container_field, "", 
                      visited_nodes, 0 );
}

void X3D::writeNodeAsVRML( ostream& os, 
                           Node *node ) {
  std::set< Node *> visited_nodes;
  // write header
  os << "#VRML V2.0 utf8" << endl;

  // write node
  writeNodeAsX3DHelp( os, node, "", "", 
                      visited_nodes, 1 );
}

#define X3D_OUTPUT 0
#define VRML_OUTPUT 1

void X3D::writeNodeAsX3DHelp( ostream& os, 
                              Node *node, 
                              const string& container_field,
                              const string & prefix,
                              std::set< Node * > &visited_nodes,
                              unsigned int output_type ) {
  if( !node ) {
    return;
  }

  
  H3DNodeDatabase *db = H3DNodeDatabase::lookupNodeInstance( node );
  string node_name = node->getTypeName();

  // construct a unique name for this node.
  stringstream s;
  s << "NODE_" << (H3DPtrUint) node;
  string name = node->hasName() ? node->getName() : s.str();
  
  // the following nodes have changed name in the X3D spec. Use the 
  // one from the new specification instead.
  if( node_name == "Image3DTexture" ) node_name = "ImageTexture3D";
  else if( node_name == "Pixel3DTexture" ) node_name = "PixelTexture3D";
  else if( node_name == "Composed3DTexture" ) node_name = "ComposedTexture3D";   
  set< Node * >::iterator i = visited_nodes.find( node );    
  bool use_node = i != visited_nodes.end();
     
  if( output_type == X3D_OUTPUT ) {
    os << prefix << "<" << node_name << " ";
    // only use container field if it is needed.
    if( node->defaultXMLContainerField() != container_field && 
        container_field != "" ) {
      os << "containerField=\"" << container_field << "\" ";
    }
    // handle DEF/USE cases
    if( !use_node ) {
      os << "DEF=\"" << name << "\" ";
      visited_nodes.insert( node );
    } else {
      os << "USE=\"" << name << "\" ";
    }
  } else if( output_type == VRML_OUTPUT ) {
    if( !use_node ) {
      os << prefix << "DEF " << name << " " << node_name << " {" << endl;
      visited_nodes.insert( node );
    } else {
      os << prefix << "USE " << name << endl;
    }
  }

  string new_prefix = prefix + "  "; 

  vector< pair< string, SFNode * > > sf_nodes;
  vector< pair< string, MFNode * > > mf_nodes;
  vector< pair< string, SFNode * > > dyn_sf_nodes;
  vector< pair< string, MFNode * > > dyn_mf_nodes;
  vector< pair< string, ParsableField * > > dyn_parse_fields;

  //only set fields if it is not a node that is USEd
  if( !use_node ) {
    H3DDynamicFieldsObject * dyn_f_obj =
      dynamic_cast< H3DDynamicFieldsObject * >(node);
    AutoRef< Node > default_value_node( H3DNodeDatabase::createNode( node_name ) );
    if( default_value_node.get() ) {
      for( H3DNodeDatabase::FieldDBConstIterator k = db->fieldDBBegin();
           k != db->fieldDBEnd(); ++k ) {
        Field *f = node->getField( *k );
        // In the case of dynamic fields a field in the field database might not
        // exist for this node, in that case just check next.
        if( !f )
          continue;
        Field::AccessType access_type = f->getAccessType();
        // check if field is a dynamic field.
        bool dynamic_field = false;
        H3DDynamicFieldsObject::field_iterator j;
        if( dyn_f_obj ) {
          for( j = dyn_f_obj->firstField();
               j != dyn_f_obj->endField(); ++j )
            if( (*j) == f ) {
              dynamic_field = true;
              break;
            }
        }

        if( dynamic_field ) {
          // Dynamic fields are added to other vectors.
          if( MFNode *mf_node = dynamic_cast< MFNode * >( f ) ) {
            dyn_mf_nodes.push_back( make_pair( (*j)->getName(), mf_node ) );
          } else if( SFNode *sf_node = dynamic_cast< SFNode * >( f ) ) {
            dyn_sf_nodes.push_back( make_pair( (*j)->getName(), sf_node ) );
          } else if( ParsableField *p_field = 
                     dynamic_cast< ParsableField * >( f ) ) {
            dyn_parse_fields.push_back( make_pair( (*j)->getName(), p_field ) );
          }
        } else if( access_type != Field::INPUT_ONLY &&
                   access_type != Field::OUTPUT_ONLY ) {
          if( MFNode *mf_node = dynamic_cast< MFNode * >( f ) ) {
            if( mf_node->size() > 0 )
              mf_nodes.push_back( make_pair( *k, mf_node ) );
          } else if( SFNode *sf_node = dynamic_cast< SFNode * >( f ) ) {
            if( sf_node->getValue() )
              sf_nodes.push_back( make_pair( *k, sf_node ) );
          } else if( ParsableField *p_field = 
                     dynamic_cast< ParsableField * >( f ) ){
            // only add value if it is different from the default value.
            ParsableField *default_field = dynamic_cast< ParsableField * >(
                 default_value_node->getField( *k ) );
            string v = p_field->getValueAsString();
            if( default_field && default_field->getValueAsString() != v ) {
              if( output_type == X3D_OUTPUT ) {
                os << *k << "=\'" << v << "\' ";
              } else if( output_type == VRML_OUTPUT ) {
                // VRML versions of true and false are upper case.
                if( dynamic_cast< MFBool * >( default_field ) || 
                    dynamic_cast< SFBool * >( default_field ) ){
                  for (size_t l=0; l<v.length(); ++l) {
                    v[l]=toupper(v[l]);
                  }
                }

                if( dynamic_cast< MFieldClass * >( default_field ) ) { 
                  os << new_prefix << *k << " [" << v << "]" << endl;
                } else if( dynamic_cast< SFString *>( default_field ) ) {
                  os << new_prefix << *k << " \"" << v << "\"" << endl;
                } else {
                  os << new_prefix << *k << " " << v << endl;
                }
              }
            }
          }
        }
      }
    }
  }

  
  if( sf_nodes.empty() && mf_nodes.empty() &&
      dyn_sf_nodes.empty() && dyn_mf_nodes.empty() &&
      dyn_parse_fields.empty() ) {
    if( output_type == X3D_OUTPUT ) {
      // if no nodes as children, end tag on same line
      os << " />" << endl;
    } else if( output_type == VRML_OUTPUT ) {
      if( !use_node ) {
        os << prefix << "}" << endl;
      }
    }
  } else {
    if( output_type == X3D_OUTPUT ) {
      os << ">" << endl;
    }

    // Handle dynamic parsable fields
    for( unsigned int k = 0; k < dyn_parse_fields.size(); ++k ) {
      if( output_type == X3D_OUTPUT ) {
        Field::AccessType access_type =
          dyn_parse_fields[k].second->getAccessType();
        os << "<field name=\"" << dyn_parse_fields[k].first << "\" type=\""
           << dyn_parse_fields[k].second->getTypeName()
           << "\" value=\"" << dyn_parse_fields[k].second->getValueAsString()
           << "\" accessType=\"";
        if( access_type == Field::INPUT_OUTPUT )
          os << "inputOutput";
        else if( access_type == Field::INPUT_ONLY )
          os << "inputOnly";
        else if( access_type == Field::OUTPUT_ONLY )
          os << "ouputOnly";
        else if( access_type == Field::INITIALIZE_ONLY )
          os << "initializeOnly";
        os << "\"/>" << endl;
      } else if( output_type == VRML_OUTPUT ) {
        string v = dyn_parse_fields[k].second->getValueAsString();

        // VRML versions of true and false are upper case.
        if( dynamic_cast< MFBool * >( dyn_parse_fields[k].second ) ||
            dynamic_cast< SFBool * >( dyn_parse_fields[k].second ) ){
          for (size_t j=0; j<v.length(); ++j) {
            v[j]=toupper(v[j]);
          }
        }

        os << new_prefix << "field "
                         << dyn_parse_fields[k].second->getTypeName();

        if( dynamic_cast< MFieldClass * >( dyn_parse_fields[k].second ) ) {
          os << " [" << v << "]" << endl;
        } else if( dynamic_cast< SFString *>( dyn_parse_fields[k].second ) ) {
          os << " \"" << v << "\"" << endl;
        } else {
          os << " " << v << endl;
        }
      }
    }

    // process child nodes
    for( unsigned int k = 0; k < sf_nodes.size(); ++k ) {
      if( output_type == VRML_OUTPUT ) {
        os << new_prefix << sf_nodes[k].first << " ";
      }

      X3D::writeNodeAsX3DHelp( os, 
             sf_nodes[k].second->getValue(), 
             sf_nodes[k].first,
             new_prefix,
             visited_nodes,
           output_type);
    }

    for( unsigned int k = 0; k < mf_nodes.size(); ++k ) {
      if( output_type == VRML_OUTPUT ) {
         os << new_prefix << mf_nodes[k].first << " ["<< endl;
      }

      for( MFNode::const_iterator n = mf_nodes[k].second->begin();
           n != mf_nodes[k].second->end(); ++n ) {
        X3D::writeNodeAsX3DHelp( os, 
         *n, 
         mf_nodes[k].first, 
         new_prefix + "  ", 
         visited_nodes,
         output_type );  
      }

      if( output_type == VRML_OUTPUT ) {
          os << new_prefix << "]" << endl;
      } 
    }

    // Handle dynamic sf node fields, might not work when exporting
    // to wrl because I do not know the syntax when not using the
    // USE attribute, or if it is even possible.
    for( unsigned int k = 0; k < dyn_sf_nodes.size(); ++k ) {
      if( output_type == X3D_OUTPUT ) {
        Field::AccessType access_type =
          dyn_sf_nodes[k].second->getAccessType();
        os << "<field name=\"" << dyn_sf_nodes[k].first << "\" type=\""
           << dyn_sf_nodes[k].second->getTypeName() << "\" accessType=\"";
        if( access_type == Field::INPUT_OUTPUT )
          os << "inputOutput";
        else if( access_type == Field::INPUT_ONLY )
          os << "inputOnly";
        else if( access_type == Field::OUTPUT_ONLY )
          os << "ouputOnly";
        else if( access_type == Field::INITIALIZE_ONLY )
          os << "initializeOnly";
        os << "\" >" << endl;
      } else if( output_type == VRML_OUTPUT ) {
        os << new_prefix << "exposedField "
           << dyn_sf_nodes[k].second->getTypeName();
      }

      X3D::writeNodeAsX3DHelp( os, 
             dyn_sf_nodes[k].second->getValue(), 
             "",
             new_prefix,
             visited_nodes,
           output_type );

      if( output_type == X3D_OUTPUT ) {
        os << "</field>" << endl;
      }
    }

    // Handle dynamic mf node fields, might not work when exporting
    // to wrl because I do not know the syntax when not using the
    // USE attribute, or if it is even possible.
    for( unsigned int k = 0; k < dyn_mf_nodes.size(); ++k ) {
      if( output_type == X3D_OUTPUT ) {
        Field::AccessType access_type =
          dyn_mf_nodes[k].second->getAccessType();
        os << "<field name=\"" << dyn_mf_nodes[k].first
           << "\" type=\"MFNode\" accessType=\"";
        if( access_type == Field::INPUT_OUTPUT )
          os << "inputOutput";
        else if( access_type == Field::INPUT_ONLY )
          os << "inputOnly";
        else if( access_type == Field::OUTPUT_ONLY )
          os << "ouputOnly";
        else if( access_type == Field::INITIALIZE_ONLY )
          os << "initializeOnly";
        os << "\"/>" << endl;
      } else if( output_type == VRML_OUTPUT ) {
         os << new_prefix << "exposedField "
           << dyn_sf_nodes[k].second->getTypeName() << "[";
      }

      for( MFNode::const_iterator n = dyn_mf_nodes[k].second->begin();
           n != dyn_mf_nodes[k].second->end(); ++n ) {
        X3D::writeNodeAsX3DHelp( os, 
         *n, 
         "", 
         new_prefix + "  ",
         visited_nodes,
         output_type );
      }

      if( output_type == X3D_OUTPUT )
        os << "</field>" << endl;
      else if( output_type == VRML_OUTPUT ) {
          os << new_prefix << "]" << endl;
      } 
    }

    if( output_type == X3D_OUTPUT ) {
      // end tag
      os << prefix << "</" << node_name << ">" << endl;
    } else if( output_type == VRML_OUTPUT ) {
      if( !use_node )
         os << prefix << "}" << endl;
    }
  }
}


/// Write the triangles rendered by the geometry node as STL to
/// the given ostream.
H3DAPI_API void X3D::writeGeometryAsSTL( ostream &os,
                                    X3DGeometryNode *geom,
                                    const string &name,
                                    bool use_binary_format ) {
  vector< HAPI::Collision::Triangle > tris;
  tris.reserve( geom->nrTriangles() );
  geom->boundTree->getValue()->getAllTriangles( tris );

  if( use_binary_format ) {
    unsigned char header[80];
    unsigned int triangles = geom->nrTriangles();
    unsigned short abc = 0;
  
    os.write( (char*)header, sizeof(header) );
    os.write( (char*)&triangles, sizeof(unsigned int) );
  
    for( vector< HAPI::Collision::Triangle >::iterator i = tris.begin();
         i != tris.end(); ++i ) {
    
      float normal[3] = { (float) (*i).normal.x, 
                          (float) (*i).normal.y, 
                          (float) (*i).normal.z };
    
      float a[3]      = { (float)(*i).a.x, (float)(*i).a.y, (float)(*i).a.z };
      float b[3]      = { (float)(*i).b.x, (float)(*i).b.y, (float)(*i).b.z };
      float c[3]      = { (float)(*i).c.x, (float)(*i).c.y, (float)(*i).c.z };

      os.write( (char*)& normal, sizeof(float) * 3 );
      os.write( (char*)& a, sizeof(float) * 3 );
      os.write( (char*)& b, sizeof(float) * 3 );
      os.write( (char*)& c, sizeof(float) * 3 );
    
      os.write( (char*)&abc, sizeof(unsigned short) );
    }
  } else {

    os << "solid " << name << endl;

    for( vector< HAPI::Collision::Triangle >::iterator i = tris.begin();
         i != tris.end(); ++i ) {
      os << "  facet normal " << (*i).normal << endl;
      os << "    outer loop" << endl;
      os << "      vertex " << (*i).a << endl;
      os << "      vertex " << (*i).b << endl;
      os << "      vertex " << (*i).c << endl;
      os << "    endloop" << endl;
      os << "  endfacet" << endl;
    }
  
    os << "endsolid " << name << endl;
  }
}
