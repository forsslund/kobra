//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    Any use, or distribution, of this file without permission from the
//    copyright holders is strictly prohibited. Please contact SenseGraphics,
//    www.sensegraphics.com, for more information.
//
//
/// \file ShaderConstants.h
/// \brief Header file for ShaderConstants.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DSHADERCONSTANTS_H__
#define __H3DSHADERCONSTANTS_H__

#include <H3D/Node.h>
#include <H3D/Field.h>
#include <H3D/H3DDynamicFieldsObject.h>
#include <H3D/FieldTemplates.h>
#include <H3D/H3DDisplayListObject.h>
#include <map>

namespace H3D {

  /// \ingroup H3DNodes
  /// \brief The ShaderConstants node is a node will be used to gather all static
  /// shader inputs to be used in the shader. Whenever any parameter value is 
  /// modified, the related shaders shader string will be changed accordingly which
  /// will also cause the shader to be recompiled
  class H3DAPI_API ShaderConstants : 
    public Node, 
    public H3DDynamicFieldsObject,
    public H3DDisplayListObject {
  public:
    ShaderConstants(  Inst< DisplayList > _displaylist = 0 );

    virtual std::string defaultXMLContainerField() {
      return "shaderConstants";
    }

    /// override the addField function, to collect all field added and filter out
    /// duplications. 
    virtual bool addField( const std::string &name,
      const Field::AccessType &access, Field *field );

    // add field , but do not trigger event
    virtual bool addFieldNoEvent( const std::string &name,
      const Field::AccessType &access, Field *field );

    /// Remove a field from the Node.
    /// \param _name The name of the field to remove.
    /// \returns true on success false otherwise.
    virtual bool removeField (const std::string& _name);

    static H3DNodeDatabase database;
  };
}

#endif
