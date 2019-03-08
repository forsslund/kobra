//////////////////////////////////////////////////////////////////////////////
//
/// Author: Sudanthi Wijewickrema
/// 		Dept. of Otolaryngology
/// 		University of Melbourne
///
//////////////////////////////////////////////////////////////////////////////

#include "MCWithDistance.h"
#include "VolumeModel.h"
#include <H3D/H3DHapticsDevice.h>
#include <H3D/Image3DTexture.h>
#include <limits>

using namespace FS;
using namespace std;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase MCWithDistance::database( "MCWithDistance",
                                         &(newInstance<MCWithDistance>),
                                         typeid( MCWithDistance ),
                                         &MCWithRange::database );

// Define the fields that can be accessed
namespace MCWithDistanceInternals {
  FIELDDB_ELEMENT( MCWithDistance, isClose, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithDistance, distThresh, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithDistance, distFromDevice, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithDistance, closestPoint, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithDistance, skipFactor, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithDistance, burrRadius, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithDistance, displayName, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithDistance, position, INPUT_OUTPUT );
}

// Constructor
MCWithDistance::MCWithDistance( Inst< SFNode    > _metadata,
                              Inst< SFBound     > _bound,
                              Inst< DisplayList   > _displayList,
                              Inst< SFIso       > _isovalue,
                              Inst< SFOctTree     > _octTree,
                              Inst< SFTexture3DNode > _voxels,
                              Inst< SFFloat > _isoStart,
                              Inst< SFFloat > _isoEnd,
                              Inst< SFInt32 > _segNo,
                              Inst< SFInt32 > _segStart,
                              Inst< SFInt32 > _segEnd,
                              Inst< SFMaterialSegmentationNode > _segmentModel,
                              Inst< SFSmoothingMask > _smoothingMask,
                              Inst< SFTexture3DNode > _texture3D,
                              Inst< SFMaterialNode > _material,
                              Inst< SFTexture3DNode > _hilight,
                              Inst< SFTexture3DNode > _cryoColours,
                              Inst< SFBool > _generateSurface,
                              Inst< MFVec3f > _surfaceMesh,
                              Inst< MFVec3f > _surfaceNormals,
                              Inst< SFBool > _toggleHilight,
                              Inst< MFColor > _hilightColours,
                              Inst< SFBool > _sequentialHilightMode,
                              Inst< SFInt32 > _currentHilightSegment,
                              Inst< SFBool > _isClose,
                              Inst< SFFloat > _distThresh,
                              Inst< SFFloat > _distFromDevice,
                              Inst< SFVec3f > _closestPoint,
                              Inst< SFInt32 > _skipFactor,
                              Inst< SFFloat > _burrRadius,
                              Inst< SFString > _displayName,
                              Inst< SFVec3f > _position):
  MCWithRange( _metadata, _bound, _displayList, _isovalue, _octTree, _voxels, _isoStart, _isoEnd, _segNo, 
		  _segStart, _segEnd, _segmentModel, _smoothingMask, _texture3D, _material, _hilight, _cryoColours,
		  _generateSurface, _surfaceMesh, _surfaceNormals, _toggleHilight, _hilightColours, _sequentialHilightMode,
		  _currentHilightSegment),
  isClose( _isClose ),
  distThresh( _distThresh),
  distFromDevice(_distFromDevice),
  closestPoint(_closestPoint),
  skipFactor(_skipFactor),
  burrRadius(_burrRadius),
  displayName(_displayName),
  position(_position){

  type_name = "MCWithDistance";
  database.initFields( this ); 

  isClose->setValue(false);
  distThresh->setValue(0.01f);
  float max = std::numeric_limits<float>::max();
  distFromDevice->setValue(max);
  closestPoint->setValue(Vec3f(0, 0, 0));
  skipFactor->setValue(0);
  burrRadius->setValue(0.0);
  displayName->setValue("");
  position->setValue(Vec3f(0, 0, 0));


  isClose->route( displayList );
  distThresh->route( displayList );
  distFromDevice->route( displayList );
  closestPoint->route( displayList );
  skipFactor->route( displayList );
  burrRadius->route( displayList );
  displayName->route( displayList );
  position->route( displayList );

  //cout << "MCWithDistance constructed" << endl;
}

// Override the scene graph traversal function to calculate the distance at each
// traversal
void MCWithDistance::traverseSG( TraverseInfo &ti ){

	// Let MarchingCubes do its thing
	MCWithRange::traverseSG(ti);

	// Only consider the first haptics device. Can be modified later to include all
	// available devices if necessary. Will have to change the fields of the minDistStruct
	// to carry vectors of distance and closest points if this change is introduced.
//	H3DHapticsDevice * hd = ti.getHapticsDevice(0);

	// Get position with respect to the tracker coordinates
	const Vec3f &pos = ti.getAccInverseMatrix() * position->getValue();

	// Create a structure for the results
	minDistStruct result = minDistStruct();
	result.minDist = numeric_limits<float>::max();
	result.closestPoint = Vec3f(0, 0, 0);

	// Call the minimum distance for pos
	getMinimumDistance(pos, result);

	// Convert results back to world coordinates. Leave the distance as it is (coordinate
	// transformations shouldn't affect the distance)
	float dist = result.minDist;
	Vec3f projPt = ti.getAccForwardMatrix() * result.closestPoint;

	distFromDevice->setValue(dist);
	closestPoint->setValue(projPt);

	bool isInsideThresh =  dist < distThresh->getValue();
	bool prevInsideThresh = isClose->getValue();

	// Only set isClose value if moving in or out of the threshold
	if (isInsideThresh && !prevInsideThresh){
		isClose->setValue(true);
		//cout << "Moved inside threshold. distance = " << dist << endl;
	}
	else if (!isInsideThresh && prevInsideThresh){
		isClose->setValue(false);
		//cout << "Moved outside threshold. distance = " << dist << endl;
	}
}

// Get the minimum distance and the closest point from a given point to the surface
// rendered by MarchingCubes.
void MCWithDistance::getMinimumDistance(Vec3f pt, minDistStruct &result) {

	// Get the octTree structure from MarchingCubes where the vertices are stored
	OctTreeNode *octTreeRoot = octTree->getValue();

	// Get square of minimum distance and closest point for the root node. It will
	// recurse through the tree and update result as it goes
	getMinimumDistanceSqr(octTreeRoot, pt, result);

	// Get distance from square of distance and remove burr radius
	result.minDist = sqrt(result.minDist);
	result.minDist = result.minDist - burrRadius->getValue();

	// account for small variations in the burr size etc.
	if (result.minDist < 0) {
		result.minDist = 0;
	}
}

// Get the minimum distance (squared) and closest point from a given point to the faces of a surface
// stored in an octtree node. Checks all children of the node recursively. Considers the normal case
// of 3 distinct vertices defining a face, and also degenerate cases where the vertices form a line or
// a point. The results are returned in the structure result. Note that the square of the distance is
// used here to avoid calculating the square root for each distance.
void MCWithDistance::getMinimumDistanceSqr(OctTreeNode *octTreeNode, Vec3f pt, minDistStruct &result) {

	// only a leaf node stores vertices in the octtree
	if ( octTreeNode->isLeaf() ) {

		float dist;
		Vec3f projPt;
		unsigned int skipDist = (skipFactor->getValue() + 1) * 3;

		// The faces are stored in an array with 3 consecutive vertices representing
		// a triangle.
        int noTriangles = int(ceil((float) octTreeNode->vvertex.size() / skipDist));

		// Define zero limit
		float eps = numeric_limits<float>::epsilon();

		// Consider a face (ie. 3 vertices) of the rendered surface at a time
		for ( int i=0; i<noTriangles; i++ ) {

			int j = i*skipDist;

			Vec3f pt1 = octTreeNode->vvertex[j];
			Vec3f pt2 = octTreeNode->vvertex[j+1];
			Vec3f pt3 = octTreeNode->vvertex[j+2];

			// Get vectors going from point one to the other two vertices and haptics location
			Vec3f vec1 = pt2 - pt1;
			Vec3f vec2 = pt3 - pt1;
			Vec3f vec = pt - pt1;

			// All 3 vertices are coincident (ie. a point)
			if ( vec1.lengthSqr() <= eps && vec2.lengthSqr() <= eps ) {

				// projPt is any of the 3 points in the vertex
				projPt = pt1;
				dist = (pt - projPt).lengthSqr();

			// The 3 vertices are colinear (ie. a line)
			} else if ((vec1.crossProduct(vec2)).lengthSqr() <= eps) {

				// If pt1 is in the middle, swap points and update vectors
				if (vec1.dotProduct(vec2) < 0) {
					Vec3f tempPt = pt1;
					pt1 = pt2;
					pt2 = tempPt;
					vec1 = pt2 - pt1;
					vec2 = pt3 - pt1;
					vec = pt - pt1;
				}

				// If vec2 is longer, use that in the calculations so that we can
				// easily figure out if projPt is within the line segment
				if (vec1.lengthSqr() < vec2.lengthSqr()) {
					vec1 = vec2;
				}

				// Get any point on line:
				// projPt = pt1 + lamblda*vec1
				// Use the fact that (projPt - pt) is normal to the line:
				// (projPt - pt).vec1 = 0
				// Solving the above equations, we get lambda as:
				float lambda = vec1.dotProduct(vec) / vec1.lengthSqr();

				// Calculate distance if projPt is inside line segment
				if (lambda >= 0 && lambda <= 1) {
					projPt = pt1 + lambda*vec1;
					dist = (pt - projPt).lengthSqr();
				} else {
					dist = numeric_limits<float>::max();
				}

			// The 3 vertices are distinct and not colinear (ie. a triangle)
			} else {

				// Get any point on the plane formed by the 3 points:
				// projPt = pt1 + lambda1*vec1 + lambda2*vec2
				// Use the fact that (projPt - pt) is normal to the plane:
				// (projPt - pt).vec1 = 0, (projPt - pt).vec2 = 0
				// solving for the 3 equations above gives us:
				// [a, b; b, c] * [lambda1; lambda2] = [d; e];
				float a = vec1.lengthSqr();
				float b = vec1.dotProduct(vec2);
				float c = vec2.lengthSqr();

				float d = vec1.dotProduct(vec);
				float e = vec2.dotProduct(vec);

				float denom = b*b - a*c;

				// get scalars that define the projPt on the plane formed by the triangle
				float lambda1 = (b*e - c*d) / denom;
				float lambda2 = (b*d - a*e) / denom;

				// calculate distance if projPt is inside the triangle
				if (lambda1 >= 0 && lambda2 >= 0 && lambda1 <= 1 && lambda2 <= 1 && (lambda1 + lambda2) <= 1) {
					projPt = pt1 + lambda1*vec1 + lambda2*vec2;
					dist = (pt - projPt).lengthSqr();
				} else {
					dist = numeric_limits<float>::max();
				}
			}

			// only update the result structure if the distance in this iteration is less than the one
			// stored there
			if (dist < result.minDist) {
				result.minDist = dist;
				result.closestPoint = projPt;
			}
		}

	// if not a leaf node, traverse all its children
    } else {
    	for( int i = 0; i < 8; i++ ) {
    		getMinimumDistanceSqr(octTreeNode->children[i], pt, result);
    	}
    }

}
