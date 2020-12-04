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
/// \file MarchingCubes.h
/// \brief Header file for MarchingCubes, MedX3D scene graph node.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MARCHINGCUBES_H__
#define __MARCHINGCUBES_H__

#include <H3D/MedX3D/MedX3D.h>
#include <H3D/X3DGeometryNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/SFInt32.h>
#include <H3D/MFString.h>
#include <H3D/Scene.h>
#include <H3D/TextureProperties.h>
#include <H3D/X3DVertexAttributeNode.h>

#define GRADIENTS_ON_THE_FLY

namespace H3D {

  /// \ingroup Geometries
  /// \class MarchingCubes
  /// \brief The MarchingCubes geometry is a node for generating an 
  /// triangle-mesh iso-surface from a volume data set and an iso-value.
  ///
  /// This node currently only works for LUMINANCE textures as
  /// input to the voxels field. MarchingCubes will not work for textures in
  /// which any of the texture dimensions contains less than 2 pixels.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/MarchingCubes.x3d">MarchingCubes.x3d</a>
  ///     ( <a href="x3d/MarchingCubes.x3d.html">Source</a> )
  class MEDX3D_API MarchingCubes : public X3DGeometryNode {
  public:

      typedef enum {
        DISPLAY_LISTS = 0,
        VERTEX_ARRAYS,
        VERTEX_BUFFER_OBJECTS
      } RenderMode;

    class MEDX3D_API FVAContainer {
      public:
      FVAContainer() : vbo_id( NULL ), nr_components( 0 ), needs_update( true ) {}
      void updateFVAVertexBufferObject();
      void render( GLint attrib_index, const RenderMode &render_mode );
      void disable( GLint attrib_index );

      string attrib_name;
      // The index for the vertex buffer object
      GLuint *vbo_id;
      vector< GLfloat > data;
      int nr_components;
      bool needs_update;
    };

    /// Used as a stand in for AABBTree that is not a full BinaryBoundTree but instead using and OctTree
    /// structure in order to mimic the structure used by this MarchingCubes implementation.
    /// The octtree leaves however contains a HAPI::Collision::AABBTree which contains all the
    /// triangles for that leaf of the otttree.
    class MEDX3D_API AABBOctTreeBBTreeLeaf : public HAPI::Collision::AABBTree {
     public:
      AABBOctTreeBBTreeLeaf() : HAPI::Collision::AABBTree() {}

      /// Get constraint planes of the shape. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const HAPI::Vec3 &point,
                                   HAPI::Constraints &constraints,
                                   HAPI::Collision::FaceType face = HAPI::Collision::FRONT_AND_BACK,
                                   HAPI::HAPIFloat radius = -1 );

      /// Adds the triangles found in the tree, that are within the distance 
      /// radius from p, to the triangles vector.
      virtual void getTrianglesWithinRadius( const HAPI::Vec3 &p,
                                             HAPI::HAPIFloat radius,
                                             std::vector< HAPI::Collision::Triangle > &triangles);

      /// Adds the triangles, lines and points found in the tree, that are 
      /// within the distance radius from p, to their respective vector.
      virtual void getPrimitivesWithinRadius( const HAPI::Vec3 &p,
                                             HAPI::HAPIFloat radius,
                                             std::vector< HAPI::Collision::Triangle > &triangles,
                                             std::vector< HAPI::Collision::LineSegment > &lines,
                                             std::vector< HAPI::Collision::Point > &points );

      /// Adds the triangles that are intersected by the volume swept by a
      /// sphere when moving from "from" to "to".
      virtual void getTrianglesIntersectedByMovingSphere(
                    HAPI::HAPIFloat radius,
                    HAPI::Vec3 from,
                    HAPI::Vec3 to,
                    std::vector< HAPI::Collision::Triangle > &triangles);

      /// Adds the triangles, lines and points that are intersected by 
      /// the volume swept by a sphere when moving from "from" to "to".
      virtual void getPrimitivesIntersectedByMovingSphere(
                    HAPI::HAPIFloat radius,
                    HAPI::Vec3 from,
                    HAPI::Vec3 to,
                    std::vector< HAPI::Collision::Triangle > &triangles,
                    std::vector< HAPI::Collision::LineSegment > &lines,
                    std::vector< HAPI::Collision::Point > &points );

      /// Render the objects in the leafs. The caller of the function need to
      /// set up OpenGL state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Render the outlines of the tree at a given tree depth for debugging 
      /// purposes.
      virtual void renderBounds( int depth );

      /// Detect collision between a line segment and the object. Will check
      /// for collision between the triangles, lines or points contained in the
      /// leaves of the tree and the line segment.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if line
      /// intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const HAPI::Vec3 &from, 
                                  const HAPI::Vec3 &to,
                                  HAPI::Collision::IntersectionInfo &result,
                                  HAPI::Collision::FaceType face = HAPI::Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPI::HAPIFloat radius,
                                          const HAPI::Vec3 &from, 
                                          const HAPI::Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPI::HAPIFloat radius,
                                          const HAPI::Vec3 &from, 
                                          const HAPI::Vec3 &to,
                                          HAPI::Collision::IntersectionInfo &result);

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const HAPI::Vec3 &p,
                                 HAPI::Vec3 &closest_point,
                                 HAPI::Vec3 &normal,
                                 HAPI::Vec3 &tex_coord );

      /// The closest point on the bound to the given point. If tree is a leaf,
      /// the closest point to the triangles, lines or points in the leaf is
      /// returned. To know the closest point to the primitives in the bound,
      /// use closestPoint.
      virtual HAPI::Vec3 boundClosestPoint( const HAPI::Vec3 &p ) {
        if( bound.get() )
          return bound->boundClosestPoint( p );
        else {
          HAPI::Vec3 cp, tmp;
          closestPoint( p, cp, tmp, tmp );
          return cp;
        }
      }

      /// Add all triangles in the tree to the given vector.
      virtual void getAllTriangles( std::vector< HAPI::Collision::Triangle > &triangles );

      /// Add all triangles, lines and points in the tree to the given vector.
      virtual void getAllPrimitives( std::vector< HAPI::Collision::Triangle > &triangles,
                                     std::vector< HAPI::Collision::LineSegment > &lines,
                                     std::vector< HAPI::Collision::Point > &points );

      AutoRefVector< AABBOctTreeBBTreeLeaf > children;
    };

    /// The OctTreeNode class defines a tree structure for voxel space 
    /// partitioning where each node is has 8 children dividing the 
    /// space it contains.
    /// Each node contains information about which voxels it contains,
    /// what triangles has been generated for the node( if leaf), etc.
    /// The structure is used in order to only regenerate local triangles
    /// when doing dynamic updates to the volume data instead of having to 
    /// regenerate triangles for the entire dataset.
    struct MEDX3D_API OctTreeNode : public RefCountedClass {
      /// Constructor.
      OctTreeNode( int _x_min, int _y_min, int _z_min, int _x_max, int _y_max, int _z_max ):
        parent( NULL ),
        next( NULL ),
        value_min(0),
        value_max( 1000 ),
        x_min( _x_min ),
        x_max( _x_max ),
        y_min( _y_min ),
        y_max( _y_max ),
        z_min( _z_min ),
        z_max( _z_max ),
        cache( 0 ),
        vbo_id( NULL ),
        rebuild_vbo( true ) {
        for( int i = 0; i < 8; ++i ) {
          children[i] = NULL;
        }
      }

      ~OctTreeNode();
      /// Returns true if this node is a leaf node.
      inline bool isLeaf() { return children[0] == NULL; }

      /// The parent of the node.
      OctTreeNode *parent;
      
      /// If this is a leaf node next points to the next node in the leaf.
      OctTreeNode *next;
      
      /// The children nodes of the OctTreeNode.
      OctTreeNode *children[8];

      /// The minimum voxel value of any voxel within this node.
      H3DFloat value_min;

      /// The maximum voxel value of any voxel within this node.
      H3DFloat value_max;

      /// The minimum voxel index value in the x-direction of this node.
      int x_min;
      /// The maximum voxel index value in the x-direction of this node.
      int x_max;
      /// The minimum voxel index value in the y-direction of this node.
      int y_min;
      /// The maximum voxel index value in the y-direction of this node.
      int y_max;
      /// The minimum voxel index value in the z-direction of this node.
      int z_min;
      /// The maximum voxel index value in the z-direction of this node.
      int z_max;

      /// The OpenGL display list used for rendering the triangles generated by
      /// the marching cubes algorithm for this node. 0 if no such display list
      /// has been generated.
      GLuint cache;

      /// The vertices generated by the marching cubes algorithm for this node.
      /// Empty if not leaf node.
      vector<Vec3f> vvertex;

      /// The normals generated by the marching cubes algorithm for this node.
      /// Empty if not leaf node.
      vector<Vec3f> vnormal;

      /// An index list for the vertices/normals generated by the Marching cubes algorithm
      vector< unsigned int > vindex;

      typedef enum {
        GRADIENT = 0,
        NORMALS_PER_FACE
      } NormalRenderMode;

      /// Render this node using OpenGL. If a leaf node a display list will
      /// be generated to render the triangles for this node the next time.
      void render( NormalRenderMode normal_render_mode, RenderMode render_mode, const map< string, GLint > &attribute_location_map );

      /// Subdivide the current node depth number of times. All previous
      /// children will be removed.
      void subdivide( int depth );

      /// Breaks the cache which will force a rerender of the entire tree.
      void breakCache();

      /// Returns the number of triangles used for rendering the oct tree;
      int nrTriangles();

      AutoRef< AABBOctTreeBBTreeLeaf > bound_tree;

      // id for vertex buffer object rendering.
      GLuint *vbo_id;
      bool rebuild_vbo;

      vector< FVAContainer > float_vertex_attributes;
      map< unsigned int, vector< vector< H3DFloat > > > vindex_to_fva_value_map;

      void calculateNewFVA( const vector< unsigned int > &_vindices, vector< vector< H3DFloat > > &fva );
      void clearFVA();
    };
  

    /// The SFOctTree class is a field class that updates its internal 
    /// oct tree and marching cubes triangles according to changes in
    /// volume data and iso value.
    ///
    /// routes_in[0] is the isovalue   (SFFloat)
    /// routes_in[1] is the voxel data (SFTexture3DNode)
    class MEDX3D_API SFOctTree: public RefCountSField< OctTreeNode > {
    public:
      /// Constructor.
      SFOctTree() : 
        data_matrix( 0 ),
        x_points( 0 ),
        y_points( 0 ),
        z_points( 0 ),
        function_count( 0 ),
        update_thread( new PeriodicThread() ),
        currently_used_image( NULL ) {
        update_thread->setThreadName( "MarchingCubes update thread" );
      }

      ~SFOctTree();

      /// Structure for transfering data to separate thread to calculate
      /// new triangles if thread mode is used.
      struct MEDX3D_API UpdateData {
        float iso_value;
        unsigned int x_min, y_min, z_min, x_max, y_max, z_max;
        AutoRef< MarchingCubes > mc;
        OctTreeNode *oct_tree_leaf;
        HAPI::Vec3 (*texture_coordinate_generator_func)( const Vec3f &c, void *data );
        vector< H3DFloat > texture_coordinate_generator_data;
      };
      
      /// Structure for transferring triangles back when new triangles
      /// have been calculated in separate thread.
      struct MEDX3D_API TransferData {
        auto_ptr< vector<Vec3f > > vertices;
        auto_ptr< vector<Vec3f > > normals;
        auto_ptr< vector< unsigned int > > vindices;
        AutoRef< MarchingCubes > mc;
        OctTreeNode *oct_tree_leaf;
        AutoRef< HAPI::Collision::AABBTree > triangle_collision_tree;
        vector< vector< H3DFloat > > float_vertex_attributes;
      };
      
      /// Update the oct tree value_min and value_max values according
      /// to the current values of the voxel values in the data_matrix
      /// member.
      void updateMinMaxValues( OctTreeNode *t);
      
      /// Recalculates all the triangles in the oct tree using the 
      /// marching cubes algorithm using the voxel values in the data_matrix
      /// member.
      /// 
      /// \param tree The oct tree which we should update with new triangle
      /// info.
      /// \param iso_value The iso-value of the iso-surface of which to
      /// extract triangles.
      /// \param update_data If the update_data paramater is given the 
      /// function will not update the triangle data directly. Instead it will
      /// collect a set of functions and parameters that have to be called in
      /// order for the update to occur. This can be used to e.g. call these
      /// functions in different threads in order to speed up computation.
      /// \param texture_coordinate_generator_data Contains data used for
      /// texture coordinate generation. The data is assumed to be the 12
      /// values supplied to a TextureCoordinateGenerator if the MATRIX
      /// generation mode is used.
      void updateMCTriangles( OctTreeNode *tree,
                              H3DFloat iso_value, 
                              vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > *update_data,
                              vector< H3DFloat > &texture_coordinate_generator_data );


      /// Recalculates all the triangles within a subvolume in the oct tree
      /// using the marching cubes algorithm using the voxel values in the
      /// data_matrix member.
      /// 
      /// \param tree The oct tree which we should update with new triangle
      /// info.
      /// \param iso_value The iso-value of the iso-surface of which to extract
      /// triangles.
      /// \param volume_min_x The voxel with the lowest index in x direction of
      /// the subvolume.
      /// \param volume_max_x The voxel with the highest index in x direction
      /// of the subvolume.
      /// \param volume_min_y The voxel with the lowest index in y direction of
      /// the subvolume.
      /// \param volume_max_y The voxel with the highest index in y direction
      /// of the subvolume.
      /// \param volume_min_z The voxel with the lowest index in z direction of
      /// the subvolume.
      /// \param volume_max_z The voxel with the highest index in z direction
      /// of the subvolume.
      /// \param update_data If the update_data paramater is given the function
      /// will not update the triangle data directly. Instead it will collect a
      /// set of functions and parameters that have to be called in order for
      /// the update to occur. This can be used to e.g. call these functions in
      /// different threads in order to speed up computation.
      /// \param texture_coordinate_generator_data Contains data used for
      /// texture coordinate generation. The data is assumed to be the 12
      /// values supplied to a TextureCoordinateGenerator if the MATRIX
      /// generation mode is used.
      void updateMCTrianglesInVolume(  OctTreeNode *tree,
                                       H3DFloat iso_value,
                                       int volume_min_x,
                                       int volume_min_y,
                                       int volume_min_z,
                                       int volume_max_x,
                                       int volume_max_y,
                                       int volume_max_z,
                                       vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > *update_data,
                                       vector< H3DFloat > &texture_coordinate_generator_data );

      /// This function generates triangles using the marching cubes algorithm 
      /// for a specified subvolume of data.
      /// 
      /// \param iso_value The iso-value of the iso-surface of which to extract
      /// triangles.
      /// \param x_min The voxel with the lowest index in x direction of the
      /// subvolume.
      /// \param x_max The voxel with the highest index in x direction of the
      /// subvolume.
      /// \param y_min The voxel with the lowest index in y direction of the
      /// subvolume.
      /// \param y_max The voxel with the highest index in y direction of the
      /// subvolume.
      /// \param z_min The voxel with the lowest index in z direction of the
      /// subvolume.
      /// \param z_max The voxel with the highest index in z direction of the
      /// subvolume.
      /// \param vertices Generated vertices of the triangles.
      /// \param normals Generated normals of the triangles.
      /// \param indices Index of each vertex in vertices
      /// \param aabb_tree A collision tree for the triangles in this OctTreeNode.
      /// \param texture_coordinate_generator_func A function to generate texture coordinates for
      /// each point.
      /// \param texture_coordinate_generator_data User specified data sent to function.
      void vMarchingCubes( H3DFloat iso_value, 
                           unsigned int x_min, unsigned int x_max, 
                           unsigned int y_min, unsigned int y_max, 
                           unsigned int z_min, unsigned int z_max, 
                           vector< Vec3f > &vertices,
                           vector< Vec3f > &normals,
                           vector< unsigned int > &indices,
                           HAPI::Collision::AABBTree **aabb_tree,
                           HAPI::Vec3 (*texture_coordinate_generator_func)( const Vec3f &c, void *data ) = NULL,
                           void * texture_coordinate_generator_data = NULL );

      /// Create the local copy of the data that is maintained for both speed
      /// and for thread consistency when using different threads. It will
      /// update the data_matrix, voxel_size and ?_points members.
      void buildDataMatrix( Image *i, TextureProperties *tp );

      /// Update the local copy of the data that is maintained for both speed
      /// and for thread consistency when using different threads. It will
      /// update the data_matrix, voxel_size and ?_points members in the
      /// subvolume defined.
      void updateDataMatrix( Image *i,
                             int volume_min_x,
                             int volume_min_y,
                             int volume_min_z,
                             int volume_max_x,
                             int volume_max_y,
                             int volume_max_z,
                             TextureProperties *tp );
      
      /// Update the gradients in the subvolume specified by the function
      /// parameters. Updates will be done directly in the gradients parameter
      /// and it is assumed that the vector is of the correct size before a
      /// call to this function.
      virtual  void updateGradients(int x_min, int x_max, 
                                   int y_min, int y_max, 
                                   int z_min, int z_max,
                                   vector< Vec3f > &gradients );
      
      /// Returns the value of the voxel at the specified index.
      inline H3DFloat getVoxelValue( unsigned int x, 
                                     unsigned int y, 
                                     unsigned int z ) {
        //MarchingCubes *mc = static_cast< MarchingCubes * >( getOwner() );
        //return mc->voxels->getValue()->
        //         image->getValue()->getPixel( x, y, z ).r;
        return data_matrix[ ( z*y_points + y ) * x_points +x ];
      }

      /// Returns the gradient of the voxel at the specified index.
      inline Vec3f getGradient( unsigned int x, 
                                unsigned int y, 
                                unsigned int z ) {
#ifdef GRADIENTS_ON_THE_FLY
        Vec3f gradient;

        if (x==0)  {
          gradient.x=(getVoxelValue(x+1,y,z)-getVoxelValue(x,y,z))
            /voxel_size.x;          
        } else if (x==x_points-1) {
          gradient.x=(getVoxelValue(x,y,z)-getVoxelValue(x-1,y,z))
            /voxel_size.x;
        } else {
          gradient.x=(getVoxelValue(x+1,y,z)-getVoxelValue(x-1,y,z))
            /(2*voxel_size.x);
        }
        
        if (y==0)  {
          gradient.y=(getVoxelValue(x,y+1,z)-getVoxelValue(x,y,z))
            /voxel_size.y;
        } else if (y==y_points-1) {
          gradient.y=(getVoxelValue(x,y,z)-getVoxelValue(x,y-1,z))
            /voxel_size.y;
        } else {
          gradient.y=(getVoxelValue(x,y+1,z)-getVoxelValue(x,y-1,z))
            /(2*voxel_size.y);
        }
        
        if (z==0) {
          gradient.z=(getVoxelValue(x,y,z+1)-getVoxelValue(x,y,z))
            /voxel_size.z;
        } else if (z==z_points-1) {
          gradient.z=(getVoxelValue(x,y,z)-getVoxelValue(x,y,z-1))
            /voxel_size.z;
        } else{
          gradient.z=(getVoxelValue(x,y,z+1)-getVoxelValue(x,y,z-1))
            /(2*voxel_size.z);
        }
        
        return gradient;
#else
        return gradients[ ( z*y_points + y ) * x_points +x ];
#endif
      }
      
      /// The field update function to update the SFOctTree field.
      virtual void update(); 

      // Returns The root node of the OctTree
      OctTreeNode* getRootNode() { return value.get(); }

      // Writes the octTree as an indexedTriangleSet.
      void writeAsITS( const string &url );

    protected:
      /// This function generates triangles using the marching cubes algorithm 
      /// for one cube
      /// 
      /// \param iso_value The iso-value of the iso-surface of which to extract
      /// triangles.
      /// \param iX The cube index in the x direction.
      /// \param iY The cube index in the y direction.
      /// \param iZ The cube index in the z direction.
      /// \param vertices Generated vertices of the triangles.
      /// \param normals Generated normals of the triangles.
      /// \param indices A list of indices for each vertex.
      void vMarchingCube( H3DFloat iso_value, 
                          unsigned int iX,
                          unsigned int iY,
                          unsigned int iZ,
                          vector< Vec3f > &vertices,
                          vector< Vec3f > &normals,
                          vector< unsigned int > &indices );

      /// Finds the approximate point of intersection of the surface
      /// between two points with the values fValue1 and fValue2
      float fGetOffset(float fValue1, float fValue2, float fValueDesired);

      /// The volume data. Each value is a value between 0 and 1. Dimensions of
      /// the data can be found in x_points, y_points and z_points,
      float *data_matrix;

      /// The size in metres of each voxel of the data in data_matrix.
      Vec3f voxel_size;

      /// The x-dimension of the data in data_matrix.
      unsigned int x_points;
      /// The y-dimension of the data in data_matrix.
      unsigned int y_points;
      /// The z-dimension of the data in data_matrix.
      unsigned int z_points;

      /// The gradients in each point in data_matrix
      /// ( when calculated by grad() )
      vector< Vec3f > gradients;

      /// the number of functions currently executing updates in separate
      /// threads.
      unsigned int function_count;
      
      /// Callback function for calculating new triangles in separate threads.
      static PeriodicThread::CallbackCode calculateNewTriangles( void * data );
      
      /// Callback function for transfering triangle updates after done in
      /// separate thread.
      static Scene::CallbackCode transferUpdatedTriangles( void *data );
      
      // thread used for updating the marching cubes triangles.
      auto_ptr< PeriodicThread > update_thread;

      inline unsigned int cubeIndex2VertexIndex( const int &i, const int &j, const int &k, int edge_number ) {
        unsigned int first_value = edgeVerticesConnection[edge_number][0];
        unsigned int offset = 0;
        if( first_value > 0 )
          offset = (x_points + 1) * (y_points + 2) * (z_points + 2);
        if( first_value > 1 )
          offset += (x_points + 2) * (y_points + 1) * (z_points + 2);
        return offset + ( ( i + edgeVerticesConnection[edge_number][1] ) +
                                                                              ( x_points + edgeVerticesVoxelSizeDiff[edge_number][0] ) *
                                                                                ( ( j + edgeVerticesConnection[edge_number][2] ) +
                                                                                ( y_points + edgeVerticesVoxelSizeDiff[edge_number][1] ) *
                                                                                ( k + edgeVerticesConnection[edge_number][3] ) ) );
      }

      H3DUtil::Image *currently_used_image;
    }; //end of  class SFOctTree

    typedef DependentSFNode<X3DTexture3DNode,
                            FieldRef<H3DDisplayListObject,
                                     H3DDisplayListObject::DisplayList,
                                     &H3D::H3DDisplayListObject::displayList>,
                            true > SFTexture3DNode;


    /// The SFBound class is specialized to update its Bound from the 
    /// voxel data routed to it.
    ///
    /// routes_in[0] is the voxels (SFTexture3DNode)
    class MEDX3D_API SFBound: public TypedField< X3DGeometryNode::SFBound,
                   SFTexture3DNode > {
      virtual void update();
    };


    /// Wrapper class to translate string to enum
    class MEDX3D_API SFNormalRenderMode : public OnValueChangeSField< SFString > {
    public:
      SFNormalRenderMode() : normal_render_mode( OctTreeNode::GRADIENT ) {}
      inline OctTreeNode::NormalRenderMode getNormalRenderMode() {
        getValue();
        return normal_render_mode;
      }
    protected:
      virtual void onValueChange( const std::string &new_value );

      OctTreeNode::NormalRenderMode normal_render_mode;
    };

    class MEDX3D_API SFWriteMCAsITS : public OnNewValueSField< SFString > {
    protected:
      virtual void onNewValue( const std::string &new_value ) {
        MarchingCubes * mc = static_cast< MarchingCubes * >(getOwner());
        mc->octTree->writeAsITS( new_value );
      }
    };

    /// The SFTextureCoordinteNode is dependent on the propertyChanged 
    /// field of the contained X3DTextureCoordinateNode.
    typedef DependentSFNode< 
                X3DTextureCoordinateNode,
                FieldRef< X3DGeometricPropertyNode,
                          Field,
                          &X3DTextureCoordinateNode::propertyChanged > >
    SFTextureCoordinateNode;

    /// The SFBoundTree constructs a BinaryBoundTree that can be used
    /// for collision detection as well as fast collection of haptic
    /// primitives (triangles, points, lines).
    class MEDX3D_API SFBoundTree: public X3DGeometryNode::SFBoundTree {
      virtual void update();
    };

    typedef TypedMFNode< X3DVertexAttributeNode > MFVertexAttributeNode;

    /// Constructor.
    MarchingCubes( Inst< SFNode  > _metadata        = 0,
                   Inst< SFBound > _bound           = 0,
                   Inst< DisplayList > _displayList = 0,
                   Inst< SFFloat  >  _isovalue      = 0,
                   Inst< SFOctTree > _octTree       = 0,
                   Inst< SFTexture3DNode > _voxels  = 0,
                   Inst< SFNormalRenderMode > _normalRenderMode = 0,
                   Inst< SFWriteMCAsITS > _writeMarchingCubesAsITS = 0,
                   Inst< SFTextureCoordinateNode > _texCoord = 0,
                   Inst< SFBoundTree > _boundTree = 0,
                   Inst< MFVertexAttributeNode > _attrib     = 0,
                   Inst< SFBool > _trianglesBuilt = 0,
                   Inst< SFBool > _alwaysGenerateOriginalFVA = 0,
                   Inst< SFBool > _voxelsAllowedToBeReinitialized = 0 );

    /// Render the node in OpenGL.
    virtual void render();

    /// Returns the number of triangles generated by the marching cubes 
    /// algorithm.
    virtual int nrTriangles() {
      OctTreeNode *oct_tree = octTree->getValue();
      if( oct_tree ) {
        return oct_tree->nrTriangles();
      } else {
        return 0;
      }
    }

    /// The iso-value of the iso-surface for which we should extract triangles.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 10
    auto_ptr< SFFloat >  isovalue;

    /// The voxel data to extract iso surfaces in.
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< SFTexture3DNode > voxels;

    /// The internal octTree structure used for dynamic updates of the iso
    /// surface.
    auto_ptr< SFOctTree> octTree;

    /// Indicates which normal mode to use when calculating normals.
    /// GRADIENT - Normals calculated by using the voxel value gradient
    /// and interpolating.
    /// NORMALS_PER_FACE - Each triangle has only one normal which is
    /// perpendicular to the surface of the triangle.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Valid values:</b> GRADIENT, NORMALS_PER_FACE
    /// <b>Default value:</b> GRADIENT
    auto_ptr< SFNormalRenderMode > normalRenderMode;

    /// If value is set then tries to write the current MarchingCubes as IndexedTriangleSet
    /// to the given url.
    ///
    /// <b>Access type:</b> inputOutput
    auto_ptr< SFWriteMCAsITS > writeMarchingCubesAsITS;

    /// Contains an X3DTextureCoordinateNode which texture coordinates are applied
    /// to the MarchingCubes.
    /// 
    /// <b>Access type:</b> inputOutput \n
    auto_ptr< SFTextureCoordinateNode > texCoord;

    /// If the attrib field is not empty it shall contain a list of
    /// X3DVertexAttributeNode instances with per-vertex attribute
    /// information for programmable shaders.
    /// This field will only be used for a total rebuild of MarchingCubes and not
    /// when simply editing the texture given in the voxels field. It is used to
    /// initialize per vertex information and and additional attribute called
    /// "originalMCPoint" will also be created which contains 1 for original points
    /// and 0 for anything created if texture was edited.
    /// 
    /// <b>Access type:</b> inputOutput \n
    auto_ptr< MFVertexAttributeNode > attrib;

    /// Set to true when the triangles have been built the first time.
    /// 
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFBool > trianglesBuilt;

    /// If true then the FloatVertexAttribute that contains original points
    /// are always generated regardless of the value in attrib.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> false \n
    auto_ptr< SFBool > alwaysGenerateOriginalFVA;

    /// If true then voxels are allowed to be reinitialized because of perhaps
    /// displayList changes to the contained texture or something like that.
    /// If not then it will only be changed by function count or if new image value
    /// differs from old.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> false \n
    auto_ptr< SFBool > voxelsAllowedToBeReinitialized;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// a2iVertexOffset lists the positions (int), relative to vertex0,
    /// of each of the 8 vertices of a cube
    static const int a2iVertexOffset[8][3];
        
    /// a2fVertexOffset lists the positions (float), relative to vertex0,
    /// of each of the 8 vertices of a cube
    static const float a2fVertexOffset[8][3];
    
    /// a2iEdgeConnection lists the index of the endpoint vertices for each of
    /// the 12 edges of the cube
    static const int a2iEdgeConnection[12][2];
    
    /// a2fEdgeDirection lists the direction vector (vertex1-vertex0) for each
    /// edge in the cube
    static const float a2fEdgeDirection[12][3];

    /// For each of the possible vertex states listed in aiCubeEdgeFlags
    /// there is a specific triangulation of the edge intersection points.
    /// a2iTriangleConnectionTable lists all of them in the form of
    /// 0-5 edge triples with the list terminated by the invalid value -1.
    /// For example: a2iTriangleConnectionTable[3] list the 2 triangles 
    /// formed when corner[0] and corner[1] are inside of the surface, but
    /// the rest of the cube is not.
    static const int a2iTriangleConnectionTable[256][16];

    /// For any edge, if one vertex is inside of the surface and the other
    /// is outside of the surface then the edge intersects the surface.
    /// For each of the 8 vertices of the cube can be two possible states :
    /// either inside or outside of the surface.
    /// For any cube the are 2^8=256 possible sets of vertex states.
    /// This table lists the edges intersected by the surface for all 256
    /// possible vertex states. There are 12 edges. For each entry in the 
    /// table, if edge bit n is intersected, then bit n is set to 1
    static const int aiCubeEdgeFlags[256];

    /// edgeVerticesConnection lists transformations from cube index
    /// (index in allcubes) to edge index (index in allvertices) for each
    /// edge belonging to a cube. That is, if cube position is i,j,k then
    /// for edge 0 the vertex for that edge is in
    /// allvertices[edgeVerticesConnection[0][0]][index] where
    /// index is a transformation from ( i + edgeVerticesConnection[0][1],
    /// j + edgeVerticesConnection[0][2], k + edgeVerticesConnection[0][3] )
    /// to a single index.
    /// See cubeIndex2VertexIterator to know how it is used.
    static const int edgeVerticesConnection[12][4];

    /// edgeVerticesVoxelSizeDiff lists size additions from x_points, y_points
    /// z_points for each edge. Used in order to calculate a single index
    /// from 3 indices when accessing allvertices. The dimensions for
    /// edge 0 is x_points + edgeVerticesVoxelSizeDiff[0][0],
    /// y_points + edgeVerticesVoxelSizeDiff[0][1],
    /// z_points + edgeVerticesVoxelSizeDiff[0][2].
    /// See cubeIndex2VertexIterator to know how it is used.
    static const int edgeVerticesVoxelSizeDiff[12][3];

    // flag used to know if a full rerender is needed
    OctTreeNode::NormalRenderMode previous_normal_render_mode;
  public:
    /// Start texture coordinate generation. Texture coordinates will be
    /// generated for all texture units used by the currently active texture.
    /// If tex_coord is NULL then texture coordinates will be generated 
    /// from the bounding box of the geometry as defined in the X3D 
    /// specification. If tex_coord is not NULL then we use the 
    /// TextureCoordinateGenerator to define the texture coordinate generation.
    virtual void startTexGen( X3DTextureCoordinateNode *tex_coord );

    /// Stop texture coordinate generation.
    virtual void stopTexGen( X3DTextureCoordinateNode *tex_coord );

    /// Gets the matrix that transforms from object space to texture
    /// coordinate space in the default case when no texCoord node
    /// is specified. 
    virtual Matrix4f getDefaultTexGenMatrix();

    /// Contains the name of the float vertex attribute that indicates whether a
    /// point is original or changed after initial generation.
    static string original_mc_point_name;
  }; 
}

#endif
