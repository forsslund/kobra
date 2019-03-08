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
/// \file ShaderPart.cpp
/// \brief CPP file for ShaderPart.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ShaderPart.h>
#include <H3D/ResourceResolver.h>
#include <H3D/GlobalSettings.h>
#include <H3D/ComposedShader.h>

#include <fstream>


using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase ShaderPart::database( 
                                   "ShaderPart", 
                                   &(newInstance<ShaderPart>), 
                                   typeid( ShaderPart ),
                                   &X3DNode::database );

namespace ShaderPartInternals {
  FIELDDB_ELEMENT( ShaderPart, url, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShaderPart, type, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( ShaderPart, forceReload, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShaderPart, shaderString, INPUT_OUTPUT )
}

namespace {
  const int pre_processor_max_recurse_depth= 32;
  const std::string include_marker= "#pragma h3dapi include";
}

ShaderPart::ShaderPart( Inst< SFNode         > _metadata,
                        Inst< MFString       > _url ,
                        Inst< SFString       > _type,
                        Inst< SFShaderString > _shaderString,
                        Inst< SFBool         > _forceReload):
  X3DNode( _metadata ),
  X3DUrlObject( _url ),
  type( _type ),
  shaderString( _shaderString ),
  forceReload( _forceReload ),
  shader_handle( 0 ),
  debug_options_previous( NULL ),
  parent_composed_shader( NULL ){
  type_name = "ShaderPart";
  database.initFields( this );

  addInlinePrefix( "glsl" );

  type->addValidValue( "VERTEX" );
  type->addValidValue( "FRAGMENT" );
  type->addValidValue( "GEOMETRY" );
  type->addValidValue( "TESS_CONTROL" );
  type->addValidValue( "TESS_EVALUATION" );
  type->setValue( "VERTEX", id );
  url->route( shaderString );
  forceReload->route( shaderString );
}


GLhandleARB ShaderPart::compileShader() {
  if( shaderString->isUpToDate() ) {
    return shader_handle;
  } else {
    return compileShaderPart();
  }
}

GLhandleARB ShaderPart::compileShaderPart(){

    //PROFILE_START("shaderpart: compile");
    if( shader_handle ) {
      glDeleteObjectARB( shader_handle );
      shader_handle = 0;
    }

    const string &s = shaderString->getValue();
    if( s == "" ) return 0;

    const string &shader_type = type->getValue();
    if( shader_type == "FRAGMENT" ) {
      shader_handle = glCreateShaderObjectARB( GL_FRAGMENT_SHADER_ARB );
    } else if( shader_type == "VERTEX" ) {
      shader_handle = glCreateShaderObjectARB( GL_VERTEX_SHADER_ARB );
    } else if( shader_type == "GEOMETRY" ) {
      if( !GLEW_EXT_geometry_shader4 ) {
        Console(LogLevel::Error) << "Warning: Geometry shaders not supported by your graphics card. ShaderPart with type \"GEOMETRY\" will be ignored." << endl;
        return 0;
      }
      shader_handle = glCreateShaderObjectARB( GL_GEOMETRY_SHADER_EXT );
    }  else if( shader_type == "TESS_CONTROL" ) {
      if( !GLEW_ARB_tessellation_shader ) {
        Console(LogLevel::Error) << "Warning: Tesselation shaders not supported by your graphics card. ShaderPart with type \"TESS_CONTROL\" will be ignored." << endl;
        return 0;
      }
      shader_handle = glCreateShaderObjectARB( GL_TESS_CONTROL_SHADER );
    }  else if( shader_type == "TESS_EVALUATION" ) {
      if( !GLEW_ARB_tessellation_shader ) {
        Console(LogLevel::Error) << "Warning: Tesselation shaders not supported by your graphics card. ShaderPart with type \"TESS_EVALUATION\" will be ignored." << endl;
        return 0;
      }
      shader_handle = glCreateShaderObjectARB( GL_TESS_EVALUATION_SHADER );
    } else {
      shader_handle = 0;
      Console(LogLevel::Warning) << "Warning: Unsupported shader type \"" << shader_type
        << "\" in ShaderPart node. Must be either \"FRAGMENT\"," 
        << "\"VERTEX\", \"GEOMETRY\", \"TESS_CONTROL\" "
        << " or \"TESS_EVALUATION." << endl;
      return shader_handle;
    }

    const char * shader_string = s.c_str();
    glShaderSourceARB( shader_handle, 1, &shader_string, NULL );
    glCompileShaderARB( shader_handle );

    GLint compile_success;
    glGetObjectParameterivARB( shader_handle,
      GL_OBJECT_COMPILE_STATUS_ARB,
      &compile_success );
    int print_error = 0;
    if( compile_success == GL_FALSE ) print_error = 1;
    else if( printShaderLog() ) print_error = 2;
    if( print_error != 0 ) {
      GLint nr_characters;
      glGetObjectParameterivARB( shader_handle,
        GL_OBJECT_INFO_LOG_LENGTH_ARB,
        &nr_characters );
      if( nr_characters > 1 ) {
        GLcharARB *log = new GLcharARB[nr_characters];
        glGetInfoLogARB( shader_handle,
          nr_characters,
          NULL,
          log );
        if( print_error == 1 ) Console(LogLevel::Warning) << "Warning: Error w";
        else Console(LogLevel::Warning) << "Warning: W";
        Console(LogLevel::Warning) << "hen compiling shader source of \"" 
          << getName() << "\" node (" << url_used 
          << ") in "<<(getParentComposedShader()==NULL?"Undefined shader":getParentComposedShader()->getFullName())
          <<"." << endl << log << endl;

        if( print_error == 1 ) {
          glDeleteObjectARB( shader_handle );
          shader_handle = 0;
        }
        delete [] log;
      }
    }
    //PROFILE_END();
    return shader_handle;
}

std::string ShaderPart::shaderStringFromURL ( const std::string& shader_url ) {
  // First try to resolve the url to file contents and load via string buffer
  // Otherwise fallback on using temp files
  string url_contents= resolveURLAsString ( shader_url );
  if ( url_contents != "" ) {
    return url_contents;
  }

  bool is_tmp_file;
  string _url = resolveURLAsFile( shader_url, &is_tmp_file );
  if( _url != "" ) {
    ifstream is( _url.c_str(), ifstream::binary );
    if( is.good() ) {
      std::streamsize length;
      char * buffer;
        
      // get length of file:
      is.seekg (0, ios::end);
      length = is.tellg();
      is.seekg (0, ios::beg);
        
      // allocate memory:
      buffer = new char [(unsigned int)length + 1];
      // read data as a block:
      is.read (buffer,length);
      length = is.gcount();
      is.close();
      if( is_tmp_file ) ResourceResolver::releaseTmpFileName( _url );
      buffer[length] = '\0';
      string value= string( buffer );
      delete [] buffer;
      //PROFILE_END();
      return value;
    }
    is.close();
    if( is_tmp_file ) ResourceResolver::releaseTmpFileName( _url );
  }

  return "";
}

std::string ShaderPart::preProcess ( const std::string& input, const std::string& _url, int depth ) {
  // Check if there is need to process input at all
  // FTM, since preProcess only process include_marker, so only include_marker need to be checked
  if( depth == 0 && (input.find( include_marker ) == std::string::npos) ) {
    return input;
  }
  // Catch infinite recursion
  if ( depth > pre_processor_max_recurse_depth ) {
    Console(LogLevel::Error) << "Warning: ShaderPart: " << getName() << ": Maximum recursion depth reached in pre-processing. Could be recursive include." << endl;
    return input;
  }

  string output;
  stringstream ss ( input );
  string line;
  while ( getline ( ss, line ) ) {

    // #include statements
    size_t i= line.find ( include_marker );
    if ( i != string::npos ) {
      string path= line.substr ( i+include_marker.length() );

      // Strip white space and quotes
      size_t a, b;
      for ( a= 0; a < path.size() && (isspace ( path[a] ) || path[a] == '\"'); ++a );
      for ( b= 1; b <= path.size()  && (isspace ( path[path.size() - b] ) || path[path.size() - b] == '\"'); ++b );
      path= path.substr ( a, path.size()-b-a+1 );

      // change url resolver base
      string old_base= getURLBase ();
      string base= _url.substr( 0, _url.find_last_of( "/\\" ) + 1 );

      // if the url is relative this should succeed
      setURLBase ( old_base + base );
      string include_source= shaderStringFromURL ( path );
      if ( include_source == "" ) {
        // if the url is absolute this should succeed
        setURLBase ( base );
        include_source= shaderStringFromURL ( path );
      }

      if ( include_source != "" ) {
        include_source= preProcess ( include_source, path, depth+1 );
        output+= include_source + "\n";
      } else {
        Console(LogLevel::Error) << "Warning: ShaderPart: " << getName() << ": Could not include shader file: " << path << " in " << getURLBase() << "." << endl;
      }

      setURLBase ( old_base );
    } else {
      output+= line + "\n";
    }
  }
  return output;
}

bool ShaderPart::isCompiled () {
  return shaderString->isUpToDate();
}

bool ShaderPart::SFShaderString::doFullRebuild() {
  ShaderPart *shader_part = static_cast<ShaderPart *>(getOwner());
  return hasCausedEvent(shader_part->url) || hasCausedEvent(shader_part->forceReload);
}

void ShaderPart::SFShaderString::update() {
  //PROFILE_START("shaderpart: update");

  ShaderPart *shader_part = static_cast<ShaderPart *>(getOwner());
  ComposedShader *parent = shader_part->getParentComposedShader();

  bool full_rebuild = doFullRebuild();
#ifdef DEBUG_COMPILE_LOG
  Console(LogLevel::Error) << "Rebuilding(" << shader_part->type->getValue() << "). Reason: " << event.ptr->getName() << endl;
#endif
  if(full_rebuild) {
    MFString *urls = static_cast<MFString *>(routes_in[0]);
    bool found_url = false;
    for(MFString::const_iterator i = urls->begin(); i != urls->end(); ++i) {
      string include_source = shader_part->shaderStringFromURL(*i);
      if(include_source != "") {
        value = include_source;
        shader_part->setURLUsed(*i);
        found_url = true;
        break;
      }
    }

    if(!found_url) {
      Console(LogLevel::Error) << "None of the urls in ShaderPart: " << this->getFullName() << " with url [";
      for(MFString::const_iterator i = urls->begin(); i != urls->end(); ++i) {
        Console(LogLevel::Error) << " \"" << *i << "\"";
      }
      Console(LogLevel::Error) << "] could be loaded." << endl;
      shader_part->setURLUsed("");
      value = "";
    }
  }

  if(full_rebuild ||
    (parent && hasCausedEvent(parent->shaderConstants))) {
      shader_part->updateShaderConstantValues(value, full_rebuild);
  }

  if(shader_part->type->getValue() == "GEOMETRY"||shader_part->type->getValue() == "FRAGMENT") {
    shader_part->updateSinglePassStereoValues(value);
  }

  if(value != "") {
    value = shader_part->preProcess(value, shader_part->getURLUsed());
  }

  //PROFILE_END();
}

bool H3D::ShaderPart::SFShaderString::modifyShaderConstants(Field* field) {
  ShaderPart* shader_part = static_cast<ShaderPart *>(getOwner());

  if(shader_part) {
    // See if the constant can be found in the shader part
    size_t location = value.find(" " + field->getName() + " ");

    // If a match is found, we set its value
    if(location != std::string::npos) {

      std::string field_name;
      std::string field_type;
      std::string field_value;

      if(shader_part->getConstantVariableString(field, field_name, field_type, field_value)) {
        shader_part->replaceString(value, field_type + field_name + " =", ";", field_value);
        return true;
      }
    }
  }

  return false;
}

X3DUrlObject::LoadStatus ShaderPart::loadStatus() {
  if( url_used != "" ) return X3DUrlObject::LOADED;
  else return X3DUrlObject::FAILED;
}

bool ShaderPart::printShaderLog() {
  DebugOptions *debug_options = NULL;
  GlobalSettings *default_settings = GlobalSettings::getActive();
  if( default_settings&&default_settings->optionNodesUpdated() ) {
    default_settings->getOptionNode( debug_options );
    if( debug_options ) {// update debug options
      debug_options_previous = debug_options;
      return debug_options->printShaderWarnings->getValue();
    }else{
      // global setting change in last frame, but no debug options in it now
      debug_options_previous = NULL;
      return false;
    }
  }
  else if( default_settings ) { 
    // global setting option node exist but not updated
    if( debug_options_previous!=NULL ) {
      return debug_options_previous->printShaderWarnings->getValue();
    }else{
      return false;
    }
  }else{
    // no global settings at all now
    debug_options_previous = NULL;
    return false;
  }
}

void ShaderPart::initialize() {
  X3DNode::initialize();
  GlobalSettings *default_settings = GlobalSettings::getActive();
  if( default_settings ) default_settings->getOptionNode( debug_options_previous );
  // set up a route so that shader string is rebuilt if singlePassStereo 
  // is changed
  if(type->getValue() == "GEOMETRY"||type->getValue()=="FRAGMENT" ) {
    Scene *scene = Scene::scenes.size() > 0?*Scene::scenes.begin():NULL;
    if(scene) {
      H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue()[0]);
      window->singlePassStereo->route(shaderString);
    }
  }

}

bool ShaderPart::getConstantVariableString(Field* const variable,
  std::string& out_name, std::string& out_type, std::string& out_field_value) {
    std::string value_string;
    X3DTypes::X3DType x3d_type = variable->getX3DType();

    out_name = variable->getName();

    switch(x3d_type) {
    case X3DTypes::SFSTRING:
      {
        SFFloat *f = static_cast<SFFloat*>(variable);
        out_type = "#define ";
        out_field_value = f->getValueAsString();
        break;
      }
    case X3DTypes::SFFLOAT:
      {
        SFFloat *f = static_cast<SFFloat*>(variable);
        out_type = "const float ";
        out_field_value = f->getValueAsString();
        out_field_value = " float("+out_field_value+")";
        break;
      }
    case X3DTypes::SFDOUBLE:
      {
        SFDouble *f = static_cast<SFDouble*>(variable);
        out_type = "const double ";
        out_field_value = f->getValueAsString();
        break;
      }
    case X3DTypes::SFBOOL:
      {
        SFBool *f = static_cast<SFBool*>(variable);
        out_type = "const bool ";
        out_field_value = f->getValueAsString();
        break;
      }
    case X3DTypes::SFINT32:
      {
        SFInt32 *f = static_cast<SFInt32*>(variable);
        out_type = "const int ";
        out_field_value = f->getValueAsString();
        break;
      }
    case X3DTypes::SFVEC2F:{
        SFVec2f *f = static_cast<SFVec2f*>(variable);
        std::stringstream ss;
        ss<<"vec2("<<f->getValue().x<<","<<
          f->getValue().y<<")";
        out_type = "const vec2 ";
        out_field_value = ss.str();
        break;
      } 
    case X3DTypes::SFVEC3F:
      {
        SFVec3f *f = static_cast<SFVec3f*>(variable);
        std::stringstream ss;
        ss << "vec3(" << f->getValue().x << "," <<
          f->getValue().y << "," <<
          f->getValue().z << ")";

        out_type = "const vec3 ";
        out_field_value = ss.str();
        break;
      }
    case X3DTypes::SFVEC4F:
      {
        SFVec4f *f = static_cast<SFVec4f*>(variable);
        std::stringstream ss;
        ss << "vec4(" << f->getValue().x << "," << f->getValue().y << "," <<
          f->getValue().z << "," << f->getValue().w << ")";

        out_type = "const vec4 ";
        out_field_value = ss.str();
        break;
      }
    case X3DTypes::SFCOLOR:
      {
        SFColor *f = static_cast<SFColor*>(variable);
        std::stringstream ss;
        ss << "vec3(" << f->getValue().r << "," <<
          f->getValue().g << "," <<
          f->getValue().b << ")";

        out_type = "const vec3 ";
        out_field_value = ss.str();
        break;
      }
    case X3DTypes::SFCOLORRGBA:
      {
        SFColorRGBA *f = static_cast<SFColorRGBA*>(variable);
        std::stringstream ss;
        ss << "vec4(" << f->getValue().r << "," << f->getValue().g << ","
          << f->getValue().b << "," << f->getValue().a << ")";

        out_field_value = ss.str();
        out_type = "const vec4 ";
        break;
      }
    default:
    {
      Console( LogLevel::Error )<<"shader constant type is not supported for "<<out_name<<endl;
      return false;
    }
    }

    return true;
}


bool H3D::ShaderPart::modifyShaderConstants(Field* field) {
  return shaderString->modifyShaderConstants(field);
}

bool H3D::ShaderPart::replaceString(std::string &shader_string,
  const std::string& string_start,
  const std::string& string_end,
  const std::string& to_insert,
  const std::string& conditional_string /* = "" */) {

    bool success = false;

    // Find start in shader string
    std::size_t found_start = shader_string.find(string_start);


    // If conditional_string is defined, we only want to insert the to_insert
    // string if we first find conditional_string.
    if(!conditional_string.empty()) {
      std::size_t conditional_found = shader_string.find(conditional_string);

      // The conditional string was not found, and as such we set the start to
      // npos to make the entire find&replace fail.
      if(conditional_found == std::string::npos) {
        found_start = std::string::npos;
      }
    }

    if(found_start != std::string::npos) {
      // Find end part and the nearest newline. Then compare the two to 
      // make sure we don't overstep any newline boundaries.
      std::size_t found_end = shader_string.find(string_end, found_start + string_start.length());
      std::size_t found_newline = shader_string.find('\n', found_start + string_start.length());

      // If everything went fine then we replace.
      if(found_end != std::string::npos && found_end <= found_newline) {
        shader_string.replace(found_start + string_start.length(),
          found_end - found_start - string_start.length(),
          to_insert);
        success = true;
      }
    }

    return success;
}


void ShaderPart::updateShaderConstantValues(std::string &shader_string, bool update_all_values) {
  ComposedShader *parent = getParentComposedShader();
  if(!parent) {
    return;
  }

  ShaderConstants *constants_node = parent->shaderConstants->getValue();
  if(constants_node) {
    for(ShaderConstants::field_iterator it = constants_node->firstField(); it != constants_node->endField(); ++it) {
      Field *field = *it;

      if(constants_node->displayList->hasCausedEvent(field) || update_all_values) {
        modifyShaderConstants(field);
      }
    }
  }
}


void ShaderPart::updateSinglePassStereoValues(std::string &shader_string) {
  Scene *scene = Scene::scenes.size() > 0 ? *Scene::scenes.begin() : NULL;
  bool new_value = false;

  if(scene) {
    H3DWindowNode* window = static_cast<H3DWindowNode*>(scene->window->getValue()[0]);
    new_value = window->singlePassStereo->getValue();
  }

  const std::string value_string = new_value?"1":"0";
  const std::string sps_supported_str = "#define SPS_SUPPORTED";
  const std::string sps_toggle_str = "#define SPS_ENABLED ";
  bool success = replaceString(shader_string,
    sps_toggle_str,
    "\n",
    value_string,
    sps_supported_str);
}


void ShaderPart::setParentComposedShader(ComposedShader *s) {
  if(parent_composed_shader) {
    parent_composed_shader->shaderConstants->unroute(shaderString);
  }

  parent_composed_shader = s;

  if(s) {
    parent_composed_shader->shaderConstants->route(shaderString);
  }
}

