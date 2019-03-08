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
/// \file MultiVolumeRayCaster.h
/// \brief Header file for MultiVolumeRayCaster.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MULTIVOLUMERAYCASTER_H__
#define __MULTIVOLUMERAYCASTER_H__

#include <H3D/X3DTexture3DNode.h>
#include <H3D/X3DTexture2DNode.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/FrameBufferTextureGenerator.h>
#include <H3D/MFMatrix4f.h>
#include <H3D/MedX3D/H3DVolumeRendererNode.h>


namespace H3D {

  /// \ingroup Nodes
  /// \class MultiVolumeRayCaster
  /// \brief The MultiVolumeRayCaster node implements GPU-based ray casting volume
  /// rendering. It is used in X3DVolumeNodes to specify the rendering
  /// algorithm to use.
  ///
  /// The stopRaysAtGeometries field determines if a ray should be stopped
  /// when reaching a position where a previously rendered geometry is. If 
  /// false the ray will continue through and accumulate values from behind 
  /// the geometry. 
  ///
  /// The useStochasticJittering field controls whether stochastic jittering
  /// should be used or not. Stochastic jittering
  /// is used to hide wood-grain artifacts by adding small offsets to the 
  /// sampling positions of rays in the viewing direction. It will reduce 
  /// regular patterns but add more noise to the result.
  /// 
  /// The rayStep field determines the step length to use between each
  /// sample taken along the ray in the ray caster.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/MultiVolumeRayCaster.x3d">MultiVolumeRayCaster.x3d</a>
  ///     ( <a href="x3d/MultiVolumeRayCaster.x3d.html">Source</a> )
  class MEDX3D_API MultiVolumeRayCaster : public H3DVolumeRendererNode {
  public:
    
        
    typedef TypedSFNode< X3DTexture2DNode > SFTexture2DNode;

    typedef DependentSFNode< X3DTexture3DNode,
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;

    class MEDX3D_API SFFloatNoZero : public SFFloat {
    public:
      virtual void setValue( const H3DFloat &v, int _id = 0 ) {
        if( v != 0 )
          SFFloat::setValue( v, _id );
      }

    protected:
      virtual void update() {
        H3DFloat value_before = value;
        SFFloat::update();
        if( value == 0 )
          value = value_before;
      }
    };

    /// Constructor.
    MultiVolumeRayCaster( Inst< SFNode  > _metadata              = 0,
                          Inst< SFFloatNoZero > _rayStep         = 0,
                          Inst< SFBool > _stopRaysAtGeometries   = 0,
                          Inst< SFBool > _useStochasticJittering = 0
        );
    
    /// Do volume rendering for a single volume.
    virtual void render( X3DVolumeNode *);

    /// Do ray casting shader for a single volume.
    virtual void buildShader( X3DVolumeNode * );

    /// Add uniform values needed to the ray casting shader.
    string addUniforms(); 

    virtual void traverseSG( X3DVolumeNode *volume, TraverseInfo &ti );

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster. This code is initialization
    /// code and will be run before the raycasting loop (PRE-LOOP)
    virtual string getShaderInitCode( );

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster. This code will be run 
    /// after the raycasting loop (POST-LOOP)
    virtual string getShaderPostCode();

    /// Virtual function modify the start position and the direction of 
    /// the ray(RAY-INITIALIZATION)
    string getRayInitializationCode();

    /// Returns the if the nr_enabled_lights constant 
    /// or getEnabledLights macro is to be used in the shader.
    virtual bool requiresEnabledLights();

    /// This function will be called when this node is being removed
    /// from the renderer field of an X3DVolumeNode. 
    virtual bool removeVolume( X3DVolumeNode *volume );

    /// The field containing the SFFloat field for the ray step size.
    /// The rayStep is in global coordinates.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.01 \n
    /// <b>Allowed values:</b> (-inf, 0) and ( 0, inf )\n
    auto_ptr< SFFloatNoZero > rayStep;

    /// The stopRaysAtGeometries field determines if a ray should be stopped
    /// when reaching a position where a previously rendered geometry is. If 
    /// false the ray will continue through and accumulate values from behind 
    /// the geometry. 
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> true \n
    auto_ptr< SFBool > stopRaysAtGeometries;

    /// If true, stochastic jittering will be used. Stochastic jittering
    /// is used to hide wood-grain artifacts by adding small offsets to the 
    /// sampling positions of rays in the viewing direction. It will reduce 
    /// regular patterns but add more noise to the result.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> false \n
    auto_ptr< SFBool > useStochasticJittering;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    typedef TypedSFNode< X3DTextureNode > SFGeneratedTextureNode;
  protected:

    /// Draw the bounding volume, used as proxy geometry for shader raycasting.
    void drawVolumeBox( const Vec3f &size );

    /// Draw a box for each volume in current_loop_volumes.
    void drawVolumeBoxes( ComposedShader *shader );

    /// Generate a texture for use with stochastic jittering.
    X3DTexture2DNode * generateStochasticJitteringTexture( unsigned int width, 
                                                           unsigned int height );

    /// A texture used for stochastic jittering when the useStochasticJittering
    /// field is set to true.
    auto_ptr< SFTexture2DNode > stochasticJitteringTexture;
    
    /// The dimensions of the stochasticJitteringTexture to generate.
    auto_ptr< SFInt32 > stochasticJitteringTextureDimension; 

    /// Contains a FrameBufferTexture for use when the stopRaysAtGeometries
    /// field is set to true.
    auto_ptr< SFNode > depthBufferTexture;

    auto_ptr< MFMatrix4f > texToLocalSpace; 
    auto_ptr< MFMatrix4f > localToTexSpace;
    auto_ptr< MFMatrix4f > localToGlobalSpace; 
    auto_ptr< MFMatrix4f > globalToLocalSpace; 
    auto_ptr< SFInt32 > volumeIndex;

    /// Internally set to true if we want the depthTexture to be ignored
    /// in getShaderCode even if stopRaysAtGeometries. We need to do this
    /// when traversing rays from other points than the viewer, in e.g.
    /// ShadedVolumeStyle with shadows.
    bool ignore_depth_texture;

    /// Specialized field that rebuilds the shader when it updates
    struct RebuildShader : 
      public TypedField< PeriodicUpdate< SFBool >, void, 
      AnyNumber< Field > > {
      
      virtual void update() { 
        MultiVolumeRayCaster *vn = 
          static_cast< MultiVolumeRayCaster * >( getOwner() );

        for( MultiVolumeRayCaster::VolumeShaderMap::iterator i = vn->volume_shaders.begin();
       i != vn->volume_shaders.end(); ++i ) {
          vn->buildShader( (*i).first ); 
    break;
         }
      }
    };

    /// When an event is received by this field the raycaster shader
    /// will be rebuilt.
    auto_ptr< RebuildShader > rebuildShader;

    /// Callback functions used to clear the 
    static Scene::CallbackCode resetVolumesCallback( void *data );

    static void checkIfVolumesChangedCallback( TraverseInfo &ti, void *data );

    vector< pair< X3DVolumeNode *, Matrix4f > > last_loop_volumes;
    vector< pair< X3DVolumeNode *, Matrix4f > > current_loop_volumes;

    AutoRef< FrameBufferTextureGenerator > depth_texture_generator;

    /// Callback function to the depth_peel_texture_generators that 
    /// sets up culling of front or back faces of the geometries 
    /// rendered.
    static void renderDepthPeelCallback( FrameBufferTextureGenerator *, int i, void *); 

  };
}

#endif
