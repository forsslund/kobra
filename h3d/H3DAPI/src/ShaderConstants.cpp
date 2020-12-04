//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    Any use, or distribution, of this file without permission from the
//    copyright holders is strictly prohibited. Please contact SenseGraphics,
//    www.sensegraphics.com, for more information.
//
//
/// \file ShaderConstants.cpp
/// \brief CPP file for ShaderConstants.
///
//
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/ShaderConstants.h>
//#include <H3D/ShaderPart.h>
#include <H3D/SFFloat.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase ShaderConstants::database("ShaderConstants",
  &(newInstance< ShaderConstants >),
  typeid(ShaderConstants),
  &Node::database);

// field definitions
namespace ShaderConstantsInternals {
}

ShaderConstants::ShaderConstants( Inst< DisplayList > _displaylist ) :
  Node(), 
  H3DDynamicFieldsObject(),
  H3DDisplayListObject( _displaylist ) {
    type_name = "ShaderConstants";
    database.initFields(this);
}

bool ShaderConstants::addField( const std::string& _name, 
                                const Field::AccessType& access, Field* field ) {

  bool success = H3DDynamicFieldsObject::addField( _name, access, field );
  if( !success ) {
    Console( LogLevel::Warning ) << "Warning: ShaderConstants: " << this->getName()
      << ". field " << _name << " is already added once. " << std::endl;
    return false;
  }
  field->route( displayList );

  return success;
}

bool ShaderConstants::addFieldNoEvent( const std::string &_name,
                                       const Field::AccessType &access, Field *field ) {

  bool success = H3DDynamicFieldsObject::addField( _name, access, field );
  if( !success ) {
    Console( LogLevel::Debug ) << "Warning: ShaderConstants: " << this->getName()
      << ". field " << _name << " is already added once. " << std::endl;
    return false;
  }
  field->routeNoEvent( displayList );
  return success;
}

bool ShaderConstants::removeField(const std::string& _name) {
  Field* field = getField(_name);

  if(field) {
    getField(_name)->unroute(displayList);
    return H3DDynamicFieldsObject::removeField(_name);
  }

  return true;
}
