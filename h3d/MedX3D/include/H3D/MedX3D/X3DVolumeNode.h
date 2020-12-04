//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//    Slice based rendering code Copyright 2003-2005, Karljohan Lundin
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
/// \file X3DVolumeNode.h
/// \brief Header file for X3DVolumeNode, X3D abstract volume scene-graph 
/// node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __X3DVOLUMENODE_H__
#define __X3DVOLUMENODE_H__

#include <H3D/MedX3D/MedX3D.h>

#include <H3D/X3DChildNode.h>
#include <H3D/X3DBoundedObject.h>
#include <H3D/H3DDisplayListObject.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/X3DTexture2DNode.h>
#include <H3D/SFMatrix4f.h>
#include <H3D/SFFloat.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFInt32.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/ComposedShader.h>
#include <H3D/MedX3D/H3DVolumeRendererNode.h>

namespace H3D {
  
  /// \ingroup AbstractNodes
  /// \class X3DVolumeNode
  /// This abstract node type is the base type for 
  /// all node types that describe volumetric data to be rendered. It sits at
  /// the same level as the polygonal X3DShapeNode within the scene graph
  /// structure, but defines volumetric data rather polygons.
  ///
  /// The dimensions field specifies the dimensions of this geometry in the
  /// local coordinate space using standard X3D units. It is assumed the
  /// volume is centered around the local origin. If the bounding box size is
  /// set, it will typically be the same size as the dimensions.
  ///
  /// Note: The following fields are H3D extensions to the X3D-specification:
  ///
  /// The surfaceNormals field contains normals from the current voxel data 
  /// using gradients and creates an X3DTexture3DNode with the normals.
  ///
  /// The renderer field contains a H3DVolumeRendererNode that defines
  /// the volume rendering technique to use and set its properties. 
  /// By default a RayCaster renderer is used.
  ///
  /// The filterType field determines what filter type to use when calculating
  /// the value at a sample point. 
  /// Valid values are:
  /// - "DEFAULT" - use the filtering type from the 3d texture node.
  /// - "NEAREST" - use nearest voxel value
  /// - "LINEAR" - use tri linear interpolation
  /// - "CUBIC_B_SPLINE" - use cubic b-spline filter(smoothing filter)
  /// - "CATMULL_ROM" - use cubic Catmull-Rom filter
  class MEDX3D_API X3DVolumeNode : 
    public X3DChildNode, 
    public X3DBoundedObject,
    public H3DDisplayListObject {
    
  public:

    typedef DependentSFNode< H3DVolumeRendererNode,
                             FieldRef<H3DVolumeRendererNode,
                                      Field,
                                      &H3D::H3DVolumeRendererNode::paramsChanged>,
                             true > SFVolumeRendererNodeBase;

    /// SFVolumeRendererNode field is specialized to call addVolume/removeVolume
    /// on the contained H3DVolumeRendererNode that is added to it.
    class MEDX3D_API SFVolumeRendererNode: public SFVolumeRendererNodeBase {
    public:
      virtual ~SFVolumeRendererNode() {
        this->value = NULL;
      }
    protected:
      virtual void onAdd( Node *n );
      virtual void onRemove( Node *n );
    };

    typedef DependentSFNode< X3DTexture3DNode,
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;
    
    /// Field for automatically computing surface normals from the voxels field
    /// routes_in[0] is the voxels field
    /// value is the surface normals
    struct MEDX3D_API SurfaceNormals : public SFTexture3DNode {
      virtual void update();
    };

    /// The UseSlicingField is specialized to change between a RayCaster
    /// or SliceRenderer instance in the renderer field when the useSlicing
    /// value is changed. This is only here for backwards compatibility
    /// reasons of the useSlicing field. The useSlicing field is deprecated
    /// so it will be removed in the future.
    class MEDX3D_API UseSlicingField : 
    public AutoUpdate< OnValueChangeSField< SFBool > > {
    protected:
      virtual void onValueChange( const bool &v );
    };
   
    /// Field that updates dimensions, texture matrices, and ray step
    /// routes_in[0] is the voxels field
    /// value is the dimensions
    struct MEDX3D_API UpdateDimensions : 
      public TypedField<PeriodicUpdate<SFVec3f>, SFVec3f> {
      virtual void update();
    };

    /// Field that calculates a 2-component 3D texture representing
    /// a subdivision of the volume data where the 2 components
    /// in each voxel is the minimum and maximum scalar value of
    /// voxels. Used in empty space skipping.
    ///
    /// routes_in[0] is the voxels field.
    /// routes_in[1] is the emptySpaceSkippingRes field
    class MEDX3D_API SFEmptySpaceMinMaxTexture3D: 
          public TypedField< TypedSFNode< X3DTexture3DNode >,
      Types< SFTexture3DNode, SFInt32 > > {
      virtual void update();
    };

    /// Field that calculates a 2D texture depending on current
    /// transfer function/render style where the value at
    /// tex coord s and t is 0 if all alpha values in the 
    /// range s(min) to t(max) (after applying transfer function)
    /// 0 and 1 otherwise.
    ///
    /// All fields that causes the transfer function/render styles
    /// to change should be routed here to cause an update to the
    /// classification texture.
    class MEDX3D_API SFEmptySpaceClassificationTexture2D: 
            public TypedField< TypedSFNode< X3DTexture2DNode >, void,
          AnyNumber< Field > > {
      virtual void update();
    };
    
    /// Bound is updated from the dimensions field
    struct MEDX3D_API SFBound : public TypedField<H3DBoundedObject::SFBound, SFVec3f> {
      virtual void update() {
        Vec3f size = static_cast<SFVec3f*>(routes_in[0])->getValue();
        BoxBound *bb = new BoxBound;
        bb->size->setValue( size );
        value = bb;
      }
    };

    /// Specialized field that rebuilds the shader when it updates
    struct MEDX3D_API RebuildShader : 
      public TypedField< PeriodicUpdate< SFBool >, void, 
      AnyNumber< Field > > {
      
      virtual void update() { 
        X3DVolumeNode *vn = 
          static_cast< X3DVolumeNode * >( getOwner() );
        vn->buildShader(); 
      }
    };

    /// Field that calculates a texture of pre-calcualted vales of the 
    /// h and g functions used in filtering(see e.g. First Third-Order
    /// Textre Filtering by Christian Sigg and Markus Hadwiger).
    /// routes_in[0] is the voxels field.
    /// routes_in[1] is the filterType field
    class MEDX3D_API SFFilterKernelTexture: 
          public TypedField< TypedSFNode< X3DTexture2DNode >,
      Types< SFTexture3DNode, SFString > > {
      virtual void update();
    protected:
      /// Calculate the weights for a cubic B-spline at the value x.
      void cubicBSplineWeights( H3DFloat x, 
                                H3DFloat &w0,
                                H3DFloat &w1,
                                H3DFloat &w2,
                                H3DFloat &w3 );

      /// Calculate the weights for a Catmull-Rom spline at the value x.
      void catmullRomSplineWeights( H3DFloat x, 
                                    H3DFloat &w0,
                                    H3DFloat &w1,
                                    H3DFloat &w2,
                                    H3DFloat &w3 );

      

    };

    /// Constructor.
    X3DVolumeNode( Inst< DisplayList > _displayList = 0, 
                   Inst< SFVec3f >     _dimensions  = 0,
                   Inst< SFNode  >     _metadata    = 0,
                   Inst< SFBound >     _bound       = 0,
                   Inst< SFTexture3DNode > _voxels      = 0, // obs SF
                   Inst< SurfaceNormals > _surfaceNormals = 0,
                   Inst< SFVec3f >     _bboxCenter  = 0,
                   Inst< SFVec3f >     _bboxSize    = 0,
                   Inst< SFString >    _filterType  = 0,
                   Inst< SFFilterKernelTexture > _filterTexture = 0,
                   Inst< SFInt32 > _emptySpaceSkippingRes = 0,
                   Inst< SFEmptySpaceMinMaxTexture3D > _emptySpaceMinMaxTexture = 0,
                   Inst< SFEmptySpaceClassificationTexture2D > _emptySpaceClassificationTexture = 0,
                   Inst< SFVolumeRendererNode > _renderer = 0
                   
                   );
    
    /// Sets up the bound field using the bboxCenter and bboxSize fields.
    /// If bboxSize is (-1, -1, -1) the bound will be a BoxBound
    /// with center in (0,0,0) and size equal to dimensions.
    /// Otherwise it will be a BoxBound with center
    /// and origin determined by the bboxCenter and bboxOrigin fields.
    virtual void initialize();
    
    /// Render the volume using OpenGL.
    virtual void render();

    /// This function will be called once per scene-graph to possibly update 
    /// the value of uniform fields. Normally only calls updateUniformFields
    /// on all volume render styles in the node.
    virtual void updateUniformFields() {}

    /////////
    // Following is a number of virtual functions for constructing the 
    // final shader

    /// Function for adding all the uniform variables to use in the shader.
    /// This is done by adding a field to the ComposedShader that is used,
    /// easiest done with the addUniformToFragmentShader help function,
    /// and returning a string of glsl uniform declarations to be used
    /// in the shader.
    virtual string addUniforms( ComposedShader *shader );

    /// Function for getting the color from the original data set.
    /// After the code returned by this function the vec4 glsl variable 
    /// 'orig_sample_color' should be set for the color of the original
    /// data set at texture coordinate "r0".
    virtual string getOrigSampleColor();

    /// Virtual function for adding glsl fragment shader code to calculate
    /// color and opacity at a sample point within the raycaster loop.
    /// After the code returned by this function the glsl variable 
    /// 'sample_color' should be set for the color of the sample. See
    /// shader source traverseRay function to see what parameters are 
    /// available(INSIDE-LOOP).
    virtual string getShaderCode();

    /// Virtual function for adding glsl fragment shader code to calculate
    /// opacity at a sample point within the raycaster loop.
    /// After the code returned by this function the glsl variable 
    /// 'sample_color' should be set for the color of the sample. See
    /// shader source traverseRay function to see what parameters are 
    /// available(INSIDE-LOOP).
    virtual string getShaderCodeOpacityOnly();

    /// Virtual function for adding function to the fragment shader that
    /// can be called in other of the shader build functions.
    virtual string getShaderFunctions();

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster. This code is initialization
    /// code and will be run before the raycasting loop (PRE-LOOP)
    virtual string getShaderInitCode() { return ""; }

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster. This code will be run 
    /// after the raycasting loop (POST-LOOP)
    virtual string getShaderPostCode() { return ""; }

    /// Virtual function to specify the compositing code, i.e. how the
    /// current sample color should be composited with the previous one.
    /// 'sample_color' will be the current sample color and opacity and
    /// 'rr' contains the previously accumulated values. After this code
    /// has been run the 'rr' variable should contain the composited
    /// color.
    virtual string getShaderCompositingCode();

    /// Virtual function modify the start position and the direction of 
    /// the ray(RAY-INITIALIZATION)
    virtual string getRayInitializationCode();

    /// Set the blend mode to use in the slice based renderer.
    virtual void setSliceRenderBlendMode() {
      glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    }

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    virtual bool requiresDefaultNormals() { return false; }

    /// Returns true if the styles requries the nr_enabled_lights constant
    /// and getEnabledLights macro in the shader.
    virtual bool requiresEnabledLights() { return false; }

    /// Detect intersection between a line segment and the VolumeData.
    /// \param from The start of the line segment.
    /// \param to The end of the line segment.
    /// \param result Contains info about the closest intersection for every
    /// object that intersects the line.
    /// \returns true if intersected, false otherwise.
    /// NOTE: Will not return correct normal and textureCoordinate at
    /// the intersection point.
    virtual bool lineIntersect( 
      const Vec3f &from,
      const Vec3f &to,
      LineIntersectResult &result );

    /// Find closest point on this geometry to point p.
    /// \param p The point to find the closest point to.
    /// \param result A struct containing various results of closest
    /// points such as which geometries the closest points where
    /// detected on.
    /// NOTE: Will not return correct normal and textureCoordinate at
    /// the closest point.
    virtual void closestPoint( const Vec3f &p,
                               NodeIntersectResult &result );

    /// Detect collision between a moving sphere and the geometry.
    /// \param radius The radius of the sphere
    /// \param from The start position of the sphere
    /// \param to The end position of the sphere.
    /// \param result A struct containing various results of intersections
    /// such as which geometries intersected the moving sphere.
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersect( H3DFloat radius,
                                        const Vec3f &from, 
                                        const Vec3f &to,
                                        NodeIntersectResult &result );
    
    /// Traverse the scenegraph.
    virtual void traverseSG( TraverseInfo &ti );
    
    /// Dimensions of the volume.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1 1 1 \n
    /// <b>Valid range:</b> [0-infinity)
    auto_ptr< SFVec3f > dimensions;

    /// The field containing the volume data in 3D textures.
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< SFTexture3DNode > voxels; // obs SF

    /// The field containing the automatically computed gradients
    /// for the voxel data set to be used for rendering styles that
    /// need a normal in its calculation.
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< SurfaceNormals > surfaceNormals;
    
    /// \deprecated Use a RayCaster in the renderer field and specify 
    /// its options instead
    /// The field containing the SFFloat field for the ray step size.
    /// The field is now only accessible on C++ level.
    auto_ptr< SFFloat > rayStep;
    
    /// \deprecated Use renderer field to specify renderer instead
    /// The field containing the SFBool field for using the 
    /// 3D texture slicing technique instead of raycasting
    /// The field is now only accessible on C++ level.
    auto_ptr< UseSlicingField > useSlicing;

    /// The filtering type to use for the texture data.
    /// Valid values are:
    /// - "DEFAULT" - use the filtering type from the 3d
    /// texture node.
    /// - "LINEAR" - use tri linear interpolation
    /// - "CUBIC_B_SPLINE" - use cubic b-spline filter(smoothing filter)
    /// - "CATMULL_ROM" - use cubic Catmull-Rom filter
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> DEFAULT \n
    auto_ptr< SFString > filterType;

    /// Texture used for fast calculations of cubic filter types. It
    /// is automatically calculated based on the filterType used.
    ///
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFFilterKernelTexture > filterTexture;
    
    /// \deprecated Use a RayCaster in the renderer field and specify 
    /// its options instead
    /// The stopRaysAtGeometries field determines if a ray should be stopped
    /// when reaching a position where a previously rendered geometry is. If 
    /// false the ray will continue through and accumulate values from behind 
    /// the geometry. 
    /// The field is now only accessible on C++ level.
    auto_ptr< SFBool > stopRaysAtGeometries;

    /// \deprecated Use a RayCaster in the renderer field and specify 
    /// its options instead
    /// Determines if we should use empty space skipping. Empty space skipping
    /// is a ray caster optimization method that can improve performance if
    /// a big part of the volume is just empty space(after applying styles).
    /// The volume is divided into a a number of subvolume(decided by the 
    /// emptySpaceSkippingRes field) and for each subvolume we pre-calculate
    /// if the volume is empty or not. If it is the ray-caster can increase 
    /// its step length in order to skip the empty parts. The method involves
    /// an extra texture lookup though, so in order for it to improve performace
    /// many sub-volumes have to be empty or it might actually decrease 
    /// performance.
    /// The field is now only accessible on C++ level.
    auto_ptr< SFBool > useEmptySpaceSkipping;

    /// The number of sub-volumes to divide the volume data into when
    /// useEmptySpaceSkipping is true.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 8 \n
    auto_ptr< SFInt32 > emptySpaceSkippingRes;

    /// \deprecated Use a RayCaster in the renderer field and specify 
    /// its options instead
    /// If set to true, the sub-volumes that are non-empty(in the sense
    /// for use in empty space skipping) are drawn as wireframe boxes.
    /// The field is now only accessible on C++ level.
    auto_ptr< SFBool  > showNonEmptySpace;

    /// Automatically calculated texture for use in empty space skipping.
    /// Contains a 2-component 3D texture representing
    /// a subdivision of the volume data where the 2 components
    /// in each voxel is the minimum and maximum scalar value of
    /// voxels.
    /// 
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFEmptySpaceMinMaxTexture3D > emptySpaceMinMaxTexture;

    /// Automatically calculated texture for use in empty space skipping.
    /// Contains a 2D texture depending on current
    /// transfer function/render style where the value at
    /// tex coord s and t is 0 if all alpha values in the 
    /// range s(min) to t(max) (after applying transfer function)
    /// 0 and 1 otherwise.
    ///
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFEmptySpaceClassificationTexture2D > emptySpaceClassificationTexture;    

    /// \deprecated Use a RayCaster in the renderer field and specify 
    /// its options instead
    /// If true, stochastic jittering will be used. Stochastic jittering
    /// is used to hide wood-grain artifacts by adding small offsets to the 
    /// sampling positions of rays in the viewing direction. It will reduce 
    /// regular patterns but add more noise to the result.
    /// The field is now only accessible on C++ level.
    auto_ptr< SFBool > useStochasticJittering;

    /// The H3DVolumeRendererNode to use for rendering the volume.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> RayCaster \n
    auto_ptr< SFVolumeRendererNode > renderer;

    /// volume coordinates to texture coordinates
    /// Only accessable in C++
    auto_ptr<SFMatrix4f> textureMatrix;
    
    /// texture coordinates to volume coordinates
    /// Only accessable in C++
    auto_ptr<SFMatrix4f> textureMatrixInverse;

    /// field that updates the dimensions and the texture matrices from 
    /// the voxels field
    /// Only accessable in C++
    auto_ptr<UpdateDimensions> updateDimensions;

    /// When an event is received by this field the raycaster shader
    /// will be rebuilt.
    auto_ptr< RebuildShader > rebuildShader;

    /// If this field generates an event the shader will be rebuilt.
    auto_ptr< Field > forceRebuildShader;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// The uniqueShaderName function is used to generate a name unique for
    /// this instance of the shader generator node class. This should to 
    /// generate names for all uniform and varying variables used
    /// in the node in order to avoid name clashes between other nodes
    /// or multiple instances of the same node type.
    ///
    /// \param base_name The original name of a variable.
    /// \return A representation of the name unique for this node.
    virtual string uniqueShaderName( const string &base_name );

  protected:

    ////////////////////
    // Help functions and fields for the raycaster implementation
    
    // add field as uniform to fragment shader and the corresponding code
    static string addUniformToFragmentShader( ComposedShader *shader,
                                              const string &name,
                                              const string &glsl_type,
                                              const Field::AccessType &access,
                                              Field *field,
                                              int array_type = -1,
                                              bool delete_unadded_field = true );

    /// Returns true if the current renderer is a RayCaster instance.
    bool usingRayCaster();

    /// build shader
    virtual void buildShader();

    /// Returns a string with glsl code for front-to-back composition of
    /// assiciated colors(i.e. where the color value is already pre-multiplied
    /// with the alpha value)
    string frontToBackCompositionAssociated();

    /// Returns a string with glsl code for front-to-back composition of
    /// non-assiciated colors.
    string frontToBackCompositionNonAssociated();

    /// Returns a string with glsl code for MIP composition.
    string MIPComposition();

    /// Virtual function used by the 
    /// SFEmptySpaceClassificationTexture2D::update to create a 
    /// 2d texture with values representing if a range of values
    /// contain empt space or not. The function returns
    /// true if the value range min_v to max_v can be empty, i.e. there
    /// exists a value in the range that causes the alpha value to be 0.
    virtual bool isEmptySpace( H3DFloat /*min_v*/, H3DFloat max_v ) {
      return max_v == 0;
    }

    /// Test if segment intersects the VolumeData and in that case return point
    /// of intersection
    bool lineSegmentIntersect( Vec3f from, Vec3f to, Vec3f _center,
                               Vec3f _size, Vec3f &q );

    /// This field is put between 
    /// surfaceNormals and the shader defaultNormal field and is needed
    /// to get the correct texture values in the shader.
    /// surfaceNormals->surfaceNormals_glsl->ComposedShader.defaultNormals
    auto_ptr< SFTexture3DNode > surfaceNormals_glsl;

    /// The index of the lights that were enabled during the last call to
    /// render().
    vector< unsigned int > enabled_lights;
   
  };
  
  // Duplicates the input field and routes it to the output.
  // Use this to add the render style's auto_ptr<field> to the shader.
  template<class T>
  T* copyAndRouteField(T *input) {
    T* f = new T;
    input->route( f );
    return f;
  }
  template<class T>
  T* copyAndRouteField(const auto_ptr<T> &input) {
    T* f = new T;
    input->route( f );
    return f;
  }

}

#endif
