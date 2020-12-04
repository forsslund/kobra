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
/// \file H3DVolumeRendererNode.h
/// \brief Header file for H3DVolumeRendererNode.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DVOLUMERENDERERNODE_H__
#define __H3DVOLUMERENDERERNODE_H__

#include <H3D/X3DNode.h>
#include <H3D/ComposedShader.h>
#include <H3D/MFString.h>
#include <H3D/MedX3D/MedX3D.h>

namespace H3D {
  // forward declaration
  class X3DVolumeNode;

  /// \ingroup AbstractNodes
  /// \class H3DVolumeRendererNode
  /// \brief This abstract node type is the base type for nodes
  /// that implement volume rendering algorithms.
  ///
  /// 
  class MEDX3D_API H3DVolumeRendererNode : public X3DNode {
  public:
    
    /// Constructor.
    H3DVolumeRendererNode( Inst< SFNode>  _metadata = 0 );

    /// Returns the default xml containerField attribute value.
    /// For H3DVolumeRendererNode it is "renderer".
    virtual string defaultXMLContainerField() {
      return "renderer";
    }

    /// Traverse the scene graph.
    virtual void traverseSG( X3DVolumeNode *, TraverseInfo & ) {}

    /// Virtual function to build the shader to use for a single volume.
    virtual void buildShader( X3DVolumeNode * ) {}

    /// Virtual function to render a single volume with the algorithm
    /// the node defines.
    virtual void render( X3DVolumeNode * ) {}

    /// This function will be called once for each X3DVolumeNode the
    /// node is being used. The call is made when it is added to the
    /// renderer field in the X3DVolumeNode.
    virtual void addVolume( X3DVolumeNode *volume );

    /// This function will be called when this node is being removed
    /// from the renderer field of an X3DVolumeNode. 
    virtual bool removeVolume( X3DVolumeNode *volume );

    // add field as uniform to fragment shader and the corresponding code
    static string addUniformToFragmentShader( ComposedShader *shader,
                const string &name,
                const string &glsl_type,
                const Field::AccessType &access,
                Field *field,
                int array_type = -1,
                bool delete_unadded_field = true );

    /// This field generates an event when any of a H3DVolumeRendererNode
    /// parameters change. All sub-classes should route their fields
    /// to this one.
    auto_ptr< Field > paramsChanged;

    /// Returns the if the nr_enabled_lights constant 
    /// or getEnabledLights macro is to be used in the shader.
    virtual bool requiresEnabledLights() { return false; }

  protected:
    /// String containing the glsl code for common functions used
    /// by renderers such as all the rendering style functions.
    static const string style_function;

    /// Struct to group together all shader parameters that
    /// are required for each volume it is being used for.
    struct RendererShaderInfo {
      /// Constructor.
      RendererShaderInfo();

      /// the GLSL shader for this render style 
      auto_ptr<ComposedShader> shader;
    
      /// the GLSL code string for this render style
      auto_ptr<MFString> fragmentShaderString;
    };

    
    typedef std::map< X3DVolumeNode *, RendererShaderInfo * > VolumeShaderMap;

    /// Map from X3DVolumeNode to the shader information used for that
    /// node. Each X3DVolumeNode that uses the same H3DVolumeRendererNode
    /// has its own shader in this map.
    VolumeShaderMap volume_shaders;

    // read shader string from file
    string readShaderFromFile( const string &filename );
    
    // write shader string to file
    void writeShaderToFile( const string &shaderstring, 
                            const string &filename);

    // insert fragment shader code before string before
    bool insertFragmentShaderCode( MFString *fragmentShaderString,
                                   const string &add_before, 
                                   const string &code_to_add);

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
