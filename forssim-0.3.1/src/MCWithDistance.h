//////////////////////////////////////////////////////////////////////////////
//
/// Author: Sudanthi Wijewickrema
/// 		Dept. of Otolaryngology
/// 		University of Melbourne
///
//////////////////////////////////////////////////////////////////////////////

#ifndef MCWITHDISTANCE_H__
#define MCWITHDISTANCE_H__

#include "MCWithRange.h"

using namespace H3D;

namespace FS {

  class MCWithDistance : public MCWithRange {
	  public:

	    // This is the structure that stores the minimum distance from a haptics
	    // device to the surface rendered by MarchingCubes. It also stores the
	    // point closest to the haptics device on the surface. Note that currently
	    // only the distance from one haptics device is considered. If more devcies
	    // are connected and the distance info for those are also required, minDist
	    // and closestPoint fields can be made into vectors.
	    struct minDistStruct {
			public:
				float minDist;
				Vec3f closestPoint;
		};

		// Constructor.
		MCWithDistance(Inst< SFNode  > _metadata        = 0,
					   Inst< SFBound > _bound           = 0,
					   Inst< DisplayList > _displayList = 0,
					   Inst< SFIso  >  _isovalue        = 0,
					   Inst< SFOctTree > _octTree       = 0,
					   Inst< SFTexture3DNode > _voxels  = 0,
					   Inst< SFFloat > _isoStart 		= 0,
					   Inst< SFFloat > _isoEnd		 	= 0,
					   Inst< SFInt32 > _segNo			= 0,
					   Inst< SFInt32 > _segStart		= 0,
					   Inst< SFInt32 > _segEnd			= 0,
					   Inst< SFMaterialSegmentationNode > _segmentModel = 0,
					   Inst< SFSmoothingMask > _smoothingMask = 0,
					   Inst< SFTexture3DNode > _texture3D = 0,
					   Inst< SFMaterialNode > _material = 0,
					   Inst< SFTexture3DNode > _hilight = 0,
					   Inst< SFTexture3DNode > _cryoColours = 0,
					   Inst< SFBool > _generateSurface = 0,
					   Inst< MFVec3f > _surfaceMesh = 0,
					   Inst< MFVec3f > _surfaceNormals = 0,
					   Inst< SFBool > _toggleHilight = 0,
					   Inst< MFColor > _hilightColours = 0,
					   Inst< SFBool > _sequentialHilightMode = 0,
					   Inst< SFInt32 > _currentHilightSegment = 0,
					   Inst< SFBool > _isClose 			= 0,
					   Inst< SFFloat > _distThresh 		= 0,
					   Inst< SFFloat > _distFromDevice 	= 0,
					   Inst< SFVec3f > _closestPoint 	= 0,
					   Inst< SFInt32 > _skipFactor		= 0,
					   Inst< SFFloat > _burrRadius		= 0,
					   Inst< SFString > _displayName 	= 0,
					   Inst< SFVec3f > _position		= 0);

		// Override the travserseSG function of MarchingCubes so that at each scene graph traversal
		// the distance is calculated
		virtual void traverseSG(TraverseInfo &ti);

		// Retrieve the minimum distance from pt to the surface rendered by MarchingCubes
		void getMinimumDistance(Vec3f pt, minDistStruct &result);

		// Has the device come within the distance threshold. Only updated when the threshold
		// is crossed (on the way in or out)
		auto_ptr< SFBool > isClose;

		// What is the minimum distance (in meters) that we let the device come to the structure.
		auto_ptr< SFFloat > distThresh;

		// Minimum distance (in meters) of the surface from the device.
		auto_ptr< SFFloat > distFromDevice;

		// Point on the surface closest to the haptic device. Note that this is in world coordinates.
		// If there has been some transformation of the structure, this point will reflect that change
		auto_ptr< SFVec3f > closestPoint;

		// The number of triangles to be skipped when calculating the distance. Should be a number greater
		// than or equal to 0. If you want to speed up the calculations at the risk of compromising accuracy,
		// set this to a value greater than 0. The higher this number the less accurate the results.
		auto_ptr< SFInt32 > skipFactor;

		// Burr radius of the haptics device. The tracker position will only tell u where the drill ends.
		// Therefore, in the distance calculations, we need to account for the burrRadius as well.
		auto_ptr< SFFloat > burrRadius;

		// Name of the structure that can be used for display purposes
		auto_ptr< SFString > displayName;

		// Position of the point you want to find the distance to.
		auto_ptr< SFVec3f > position;

		/// The H3DNodeDatabase for this node.
		static H3DNodeDatabase database;

	  protected:

		// Calculate the minimum distance of all the child nodes of the given octtree node
		// from a given point to the rendered surface.
		void getMinimumDistanceSqr(OctTreeNode *octTreeNode, Vec3f pt, minDistStruct &result);

  }; 
}

#endif
