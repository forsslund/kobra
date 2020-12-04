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
/// \file PhysXCollidableOptions.h
/// \brief Header file for PhysXCollidableOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSXCOLLIDABLEOPTIONS__
#define __PHYSXCOLLIDABLEOPTIONS__

#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3D/SFInt32.h>
#include <H3D/MFString.h>

namespace H3D{

  namespace PhysicsEngineParameters {

    /// \ingroup PhysX
    /// Structure describing the state of a PhysXCollidableOptions node
    /// to be passed to the physics simulation thread
    struct PhysXCollidableParameters : public EngineOptionParameters {

      /// Constructor
      PhysXCollidableParameters ( string _url_base = "" ) :
        convex ( true ),
        convexDecomposition ( false ),
        compacityWeight ( 0.0001f ),
        volumeWeight ( 0.0f ),
        scaleFactor ( 1000.0f ),
        nrClusters ( 2 ),
        nrVerticesPerCH ( 100 ),
        concavity ( 100.0f ),
        addExtraDistPoints ( true ),
        nrTargetTrianglesDecimatedMesh ( 2000 ),
        addFacesPoints ( true ),
        connectDist ( 30.0f ),
        smallClusterThreshold ( 0.25f ),
        url_base( _url_base ),
        restOffset( 0.0f ),
        contactOffset( 0.0f ),
        suppressDisabledContacts( true ),
        setFlagsForAll( false ) {
        
          contactPairFlags.push_back( "eCONTACT_DEFAULT" );
          contactPairFlags.push_back( "eNOTIFY_CONTACT_POINTS" );
          contactPairFlags.push_back( "eNOTIFY_TOUCH_PERSISTS" );
        
        }

      // 'set' functions

      void setConvex ( bool _convex ) {
        update_bit_mask|= CONVEX;
        convex= _convex;
      }

      void setConvexDecomposition ( bool _convexDecomposition ) {
        update_bit_mask|= CONVEX_DECOMPOSITION;
        convexDecomposition= _convexDecomposition;
      }

      void setCompacityWeight ( H3DFloat _compacityWeight ) {
        update_bit_mask|= COMPACITY_WEIGHT;
        compacityWeight= _compacityWeight;
      }

      void setVolumeWeight ( H3DFloat _volumeWeight ) {
        update_bit_mask|= VOLUME_WEIGHT;
        volumeWeight= _volumeWeight;
      }

      void setScaleFactor ( H3DFloat _scaleFactor ) {
        update_bit_mask|= SCALE_FACTOR;
        scaleFactor= _scaleFactor;
      }

      void setNrClusters ( H3DInt32 _nrClusters ) {
        update_bit_mask|= NR_CLUSTERS;
        nrClusters= _nrClusters;
      }

      void setNrVerticesPerCH ( H3DInt32 _nrVerticesPerCH ) {
        update_bit_mask|= NR_VERTICES_PER_CH;
        nrVerticesPerCH= _nrVerticesPerCH;
      }

      void setConcavity ( H3DFloat _concavity ) {
        update_bit_mask|= CONCAVITY;
        concavity= _concavity;
      }

      void setAddExtraDistPoints ( bool _addExtraDistPoints ) {
        update_bit_mask|= ADD_EXTRA_DIST_POINTS;
        addExtraDistPoints= _addExtraDistPoints;
      }

      void setNrTargetTrianglesDecimatedMesh ( H3DInt32 _nrTargetTrianglesDecimatedMesh ) {
        update_bit_mask|= NR_TARGET_TRIANGLES_DECIMATED_MESH;
        nrTargetTrianglesDecimatedMesh= _nrTargetTrianglesDecimatedMesh;
      }

      void setAddFacesPoints ( bool _addFacesPoints ) {
        update_bit_mask|= ADD_FACES_POINTS;
        addFacesPoints= _addFacesPoints;
      }

      void setConnectDist ( H3DFloat _connectDist ) {
        update_bit_mask|= CONNECT_DIST;
        connectDist= _connectDist;
      }

      void setSmallClusterThreshold ( H3DFloat _smallClusterThreshold ) {
        update_bit_mask|= SMALL_CLUSTER_THRESHOLD;
        smallClusterThreshold= _smallClusterThreshold;
      }

      void setCookedFilename ( const std::string& _cookedFilename) {
        update_bit_mask|= COOKED_FILENAME;
        cookedFilename= _cookedFilename;
      }

      void setSaveConvexDecomposition ( const std::string& _saveConvexDecomposition ) {
        update_bit_mask|= SAVE_CONVEX_DECOMPOSITION;
        saveConvexDecomposition= _saveConvexDecomposition;
      }

      void setRestOffset ( H3DDouble _restOffset ) {
        update_bit_mask|= REST_OFFSET;
        restOffset = _restOffset;
      }

      void setContactOffset ( H3DDouble _contactOffset ) {
        update_bit_mask|= CONTACT_OFFSET;
        contactOffset = _contactOffset;
      }

      void setSuppressDisabledContacts ( bool _supress ) {
        update_bit_mask|= SUPPRESSDISABLEDCONTACTS;
        suppressDisabledContacts= _supress;
      }

      void setSetFlagsForAll ( bool _setall ) {
        update_bit_mask|= SETFLAGSFORALL;
        setFlagsForAll= _setall;
      }

      void setContactPairFlags( const std::vector< std::string > &_flags ) {
        update_bit_mask |= CONTACTPAIRFLAGS; 
        contactPairFlags = _flags ;
      }

      // 'get' functions

      bool getConvex () {
        return convex;
      }

      bool getConvexDecomposition () {
        return convexDecomposition;
      }

      H3DFloat getCompacityWeight () {
        return compacityWeight;
      }

      H3DFloat getVolumeWeight () {
        return volumeWeight;
      }

      H3DFloat getScaleFactor () {
        return scaleFactor;
      }

      H3DInt32 getNrClusters () {
        return nrClusters;
      }

      H3DInt32 getNrVerticesPerCH () {
        return nrVerticesPerCH;
      }

      H3DFloat getConcavity () {
        return concavity;
      }

      bool getAddExtraDistPoints () {
        return addExtraDistPoints;
      }

      H3DInt32 getNrTargetTrianglesDecimatedMesh () {
        return nrTargetTrianglesDecimatedMesh;
      }

      bool getAddFacesPoints () {
        return addFacesPoints;
      }

      H3DFloat getConnectDist () {
        return connectDist;
      }

      H3DFloat getSmallClusterThreshold () {
        return smallClusterThreshold;
      }

      std::string getCookedFilename () {
        return cookedFilename;
      }

      std::string getSaveConvexDecomposition () {
        return saveConvexDecomposition;
      }

      std::string getBaseURL() {
        return url_base;
      }

      H3DDouble getRestOffset () {
        return restOffset;
      }

      H3DDouble getContactOffset () {
        return contactOffset;
      }

      bool getSuppressDisabledContacts () {
        return suppressDisabledContacts;
      }

      bool getSetFlagsForAll() {
        return setFlagsForAll;
      }

      inline std::vector<std::string>& getContactPairFlags() {
        return contactPairFlags;
      }
      
      // 'have' functions

      bool haveConvex () {
        return (update_bit_mask & CONVEX) != 0;
      }

      bool haveConvexDecomposition () {
        return (update_bit_mask & CONVEX_DECOMPOSITION) != 0;
      }

      bool haveCompacityWeight () {
        return (update_bit_mask & COMPACITY_WEIGHT) != 0;
      }

      bool haveVolumeWeight () {
        return (update_bit_mask & VOLUME_WEIGHT) != 0;
      }

      bool haveScaleFactor () {
        return (update_bit_mask & SCALE_FACTOR) != 0;
      }

      bool haveNrClusters () {
        return (update_bit_mask & NR_CLUSTERS) != 0;
      }

      bool haveNrVerticesPerCH () {
        return (update_bit_mask & NR_VERTICES_PER_CH) != 0;
      }

      bool haveConcavity () {
        return (update_bit_mask & CONCAVITY) != 0;
      }

      bool haveAddExtraDistPoints () {
        return (update_bit_mask & ADD_EXTRA_DIST_POINTS) != 0;
      }

      bool haveNrTargetTrianglesDecimatedMesh () {
        return (update_bit_mask & NR_TARGET_TRIANGLES_DECIMATED_MESH) != 0;
      }

      bool haveAddFacesPoints () {
        return (update_bit_mask & ADD_FACES_POINTS) != 0;
      }

      bool haveConnectDist () {
        return (update_bit_mask & CONNECT_DIST) != 0;
      }

      bool haveSmallClusterThreshold () {
        return (update_bit_mask & SMALL_CLUSTER_THRESHOLD) != 0;
      }

      bool haveCookedFilename () {
        return (update_bit_mask & COOKED_FILENAME) != 0;
      }

      bool haveSaveConvexDecomposition () {
        return (update_bit_mask & SAVE_CONVEX_DECOMPOSITION) != 0;
      }

      bool haveRestOffset () {
        return (update_bit_mask & REST_OFFSET) != 0;
      }

      bool haveContactOffset () {
        return (update_bit_mask & CONTACT_OFFSET) != 0;
      }

      bool haveSuppressDisabledContacts () {
        return (update_bit_mask & SUPPRESSDISABLEDCONTACTS) != 0;
      }

      bool haveSetFlagsForAll() {
        return (update_bit_mask & SETFLAGSFORALL) != 0;
      }

      bool haveContactPairFlags() {
        return (update_bit_mask & CONTACTPAIRFLAGS ) != 0;
      }


    protected:
      // update bit mask flags
      static const unsigned int CONVEX                             = 0x0001;
      static const unsigned int CONVEX_DECOMPOSITION               = 0x0002;
      static const unsigned int COMPACITY_WEIGHT                   = 0x0004;
      static const unsigned int VOLUME_WEIGHT                      = 0x0008;
      static const unsigned int SCALE_FACTOR                       = 0x0010;
      static const unsigned int NR_CLUSTERS                        = 0x0020;
      static const unsigned int NR_VERTICES_PER_CH                 = 0x0040;
      static const unsigned int CONCAVITY                          = 0x0080;
      static const unsigned int ADD_EXTRA_DIST_POINTS              = 0x0100;
      static const unsigned int NR_TARGET_TRIANGLES_DECIMATED_MESH = 0x0200;
      static const unsigned int ADD_FACES_POINTS                   = 0x0400;
      static const unsigned int CONNECT_DIST                       = 0x0800;
      static const unsigned int SMALL_CLUSTER_THRESHOLD            = 0x1000;
      static const unsigned int COOKED_FILENAME                    = 0x2000;
      static const unsigned int SAVE_CONVEX_DECOMPOSITION          = 0x4000;
      static const unsigned long int REST_OFFSET               = 0x00010000;
      static const unsigned long int CONTACT_OFFSET            = 0x00020000;
      static const unsigned long int SUPPRESSDISABLEDCONTACTS  = 0x00040000;
      static const unsigned long int SETFLAGSFORALL            = 0x00080000;
      static const unsigned long int CONTACTPAIRFLAGS          = 0x00100000;
      

      bool convex;

      bool convexDecomposition;

      H3DFloat compacityWeight;

      H3DFloat volumeWeight;

      H3DFloat scaleFactor;

      H3DInt32 nrClusters;

      H3DInt32 nrVerticesPerCH;

      H3DFloat concavity;

      bool addExtraDistPoints;

      H3DInt32 nrTargetTrianglesDecimatedMesh;

      bool addFacesPoints;

      H3DFloat connectDist;

      H3DFloat smallClusterThreshold;

      H3DDouble restOffset;

      H3DDouble contactOffset;

      bool suppressDisabledContacts;

      bool setFlagsForAll;

      /// Path to load/save a cached version of the cooked collision mesh
      /// 
      /// If the filename exists the mesh is loaded, otherwise it is cooked and saved
      /// to this location.
      ///
      /// If empty, the collision mesh is always recomputed.
      ///
      std::string cookedFilename;

      /// For debugging, a path to save the result of convex decomposition as wrl format.
      ///
      /// If empty, it is not saved.
      ///
      std::string saveConvexDecomposition;

      /// check physx doc
      std::vector< std::string > contactPairFlags;

      string url_base;
    };
  }

  /// \ingroup PhysX
  /// Node used to specify options relating to a Collidable that are specific to
  /// the PhysX physics engine implementation.
  ///
  /// The options node should be added to the engineOptions field of an H3DSoftBodyNode node.
  /// These options will be ignored by other physics engine implementations.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/PhysXCollidableOptions.x3d">PhysXCollidableOptions.x3d</a>
  ///     ( <a href="examples/PhysXCollidableOptions.x3d.html">Source</a> )
  ///   - <a href="../../examples/RigidBody/PhysXCollidableOptions_Letters.x3d">PhysXCollidableOptions_Letters.x3d</a>
  ///     ( <a href="examples/PhysXCollidableOptions_Letters.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysXCollidableOptions.dot
  class H3DPHYS_API PhysXCollidableOptions : public H3DEngineOptions {
  public:

    /// Constructor.
    PhysXCollidableOptions (
      Inst < SFNode > _metadata = 0,
      Inst < ValueUpdater > _valueUpdater = 0,
      Inst < SFBool  > _convex = 0,
      Inst < SFBool  > _convexDecomposition = 0,
      Inst < SFFloat > _compacityWeight = 0,
      Inst < SFFloat > _volumeWeight = 0,
      Inst < SFFloat > _scaleFactor = 0,
      Inst < SFInt32 > _nrClusters = 0,
      Inst < SFInt32 > _nrVerticesPerCH = 0,
      Inst < SFFloat > _concavity = 0,
      Inst < SFBool  > _addExtraDistPoints = 0,
      Inst < SFInt32 > _nrTargetTrianglesDecimatedMesh = 0,
      Inst < SFBool  > _addFacesPoints = 0,
      Inst < SFFloat > _connectDist = 0,
      Inst < SFFloat > _smallClusterThreshold = 0,
      Inst < SFString > _cookedFilename = 0,
      Inst < SFString > _saveConvexDecomposition = 0,
      Inst < SFDouble > _restOffset = 0,
      Inst < SFDouble > _contactOffset = 0,
      Inst < SFBool  > _setFlagsForAll = 0,
      Inst < SFBool  > _suppressDisabledContacts = 0,
      Inst < MFString > _contactShaderPairFlags = 0 );

    /// Returns the string identifier of the physics engine that these options relate to.
    /// In the case of this node, this function returns "PhysX"
    virtual string getEngine () {
#ifdef HAVE_PHYSX4
      return "PhysX4";
#else
      return "PhysX3";
#endif
    }

    /// Set to true if the shape should be considered convex
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> true
    ///
    /// \dotfile PhysXCollidableOptions_convex.dot
    auto_ptr < SFBool > convex;

    /// Set to true if the shape should be decomposed into convex parts
    /// using the HACD library (if present).
    ///
    /// This is required for non-static concave shapes to behave correctly.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> false
    ///
    /// \dotfile PhysXCollidableOptions_convexDecomposition.dot
    auto_ptr < SFBool > convexDecomposition;

    /// compacityWeight parameter for HACD
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.0001f
    ///
    /// \dotfile PhysXCollidableOptions_compacityWeight.dot
    auto_ptr < SFFloat > compacityWeight;

    /// volumeWeight parameter for HACD
    ///
    /// Hint: Higher numbers help to preserve (not fill in) holes
    ///       but result in more convex parts.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.0f
    ///
    /// \dotfile PhysXCollidableOptions_volumeWeight.dot
    auto_ptr < SFFloat > volumeWeight;

    /// scaleFactor parameter for HACD
    ///
    /// Scale factor applied to the mesh before processing in HACD
    ///
    /// Hint: Lower number = less accurate, fewer convex parts
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 1000.0f
    ///
    /// \dotfile PhysXCollidableOptions_scaleFactor.dot
    auto_ptr < SFFloat > scaleFactor;

    /// nrClusters parameter for HACD
    ///
    /// The minimum number of convex parts to generate.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 2
    ///
    /// \dotfile PhysXCollidableOptions_nrClusters.dot
    auto_ptr < SFInt32 > nrClusters;

    /// nrVerticesPerCH parameter for HACD
    ///
    /// The maximum number of vertices for each generated convex-hull.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 100
    ///
    /// \dotfile PhysXCollidableOptions_nrVerticesPerCH.dot
    auto_ptr < SFInt32 > nrVerticesPerCH;

    /// concavity parameter for HACD
    ///
    /// The maximum allowed concavity.
    ///
    /// Hint: Lower numbers = more accurate, more convex parts
    ///       Higher numbers = less accurate, less convex parts
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 100.0f
    ///
    /// \dotfile PhysXCollidableOptions_concavity.dot
    auto_ptr < SFFloat > concavity;

    /// addExtraDistPoints parameter for HACD
    ///
    /// Specifies whether extra points should be added when computing the concavity.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> true
    ///
    /// \dotfile PhysXCollidableOptions_addExtraDistPoints.dot
    auto_ptr < SFBool > addExtraDistPoints;

    /// nrTargetTrianglesDecimatedMesh parameter for HACD
    ///
    /// The targeted number of triangles of the decimated mesh
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 2000
    ///
    /// \dotfile PhysXCollidableOptions_nrTargetTrianglesDecimatedMesh.dot
    auto_ptr < SFInt32 > nrTargetTrianglesDecimatedMesh;

    /// addFacesPoints parameter for HACD
    ///
    /// Specifies whether faces points should be added when computing the concavity.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> true
    ///
    /// \dotfile PhysXCollidableOptions_addFacesPoints.dot
    auto_ptr < SFBool > addFacesPoints;

    /// connectDist parameter for HACD
    ///
    /// The maximum allowed distance to get CCs connected.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 30.0f
    ///
    /// \dotfile PhysXCollidableOptions_connectDist.dot
    auto_ptr < SFFloat > connectDist;

    /// smallClusterThreshold parameter for HACD
    ///
    /// The threshold to detect small clusters. The threshold is expressed 
    /// as a percentage of the total mesh surface
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.25f
    ///
    /// \dotfile PhysXCollidableOptions_smallClusterThreshold.dot
    auto_ptr < SFFloat > smallClusterThreshold;

    /// Path to load/save a cached version of the cooked collision mesh
    /// 
    /// If the filename exists the mesh is loaded, otherwise it is cooked and saved
    /// to this location.
    ///
    /// If empty, the collision mesh is always recomputed.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> ""
    ///
    /// \dotfile PhysXCollidableOptions_cookedFilename.dot
    auto_ptr < SFString > cookedFilename;

    /// For debugging, a path to save the result of convex decomposition as wrl format.
    ///
    /// If empty, it is not saved.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> ""
    ///
    /// \dotfile PhysXCollidableOptions_saveConvexDecomposition.dot
    auto_ptr < SFString > saveConvexDecomposition;

    ///  The distance where the maximium force due to contact is applied
    ///  to the collidable. The contact starts applying force to the collidable
    ///  starting from contactOffset and gradually increases until restOffset.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.0f
    ///
    /// \dotfile PhysXCollidableOptions_restOffset.dot
    auto_ptr < SFDouble > restOffset;

    ///  The distance where the collidable will start creating contacts
    ///  and a force is started to be applied gradually. Must be greater
    ///  then restOffset.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.02f
    ///
    /// \dotfile PhysXCollidableOptions_contactOffset.dot
    auto_ptr < SFDouble > contactOffset;

    ///  If true, Sets the pairFlag values to both enabled and disabled collidable contacts
    ///  Check PhysXCallbacks.cpp details.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> false
    ///
    /// \dotfile PhysXCollidableOptions_setFlagsForAll.dot
    auto_ptr < SFBool > setFlagsForAll;

    ///  If true, disabled collidable contacts are suppressed else reported.
    ///  Check PhysXCallbacks.cpp details.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> true
    ///
    /// \dotfile PhysXCollidableOptions_suppressDisabledContacts.dot
    auto_ptr < SFBool > suppressDisabledContacts;

    /// The pairFlag values to be set in physXFilterShader
    /// Check PhysX website for details and possible values.
    /// There is an additional value called CONTACT_MODE_ALL which is basically
    /// like the contactMode="All" of CollisionCollection but for this collidable only.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> "eCONTACT_DEFAULT" "eNOTIFY_CONTACT_POINTS" "eNOTIFY_TOUCH_PERSISTS"
    ///
    /// \dotfile PhysXCollidableOptions_contactShaderPairFlags.dot
    auto_ptr < MFString > contactShaderPairFlags;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new instance of EngineOptionParameters describing the current state of the node fields
    virtual PhysicsEngineParameters::EngineOptionParameters* getParameters( bool all_params = false );

    string base_url;
  public:
    /// Initialize function.
    virtual void initialize();
  };
}
#endif
