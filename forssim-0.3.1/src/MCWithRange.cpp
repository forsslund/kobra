//////////////////////////////////////////////////////////////////////////////
//
/// MCWithRange.cpp is a modified version of MarchingCubes.cpp in MedX3D by
/// Copyright 2004-2008, SenseGraphics AB
///
/// Modified by: Sudanthi Wijewickrema
/// 		Dept. of Otolaryngology
/// 		University of Melbourne
//
//
//////////////////////////////////////////////////////////////////////////////

#include "MCWithRange.h"
#include <H3D/H3DHapticsDevice.h>
#include <assert.h>
#include <math.h>
#include <limits>
#ifdef Q_OS_WIN
#include <windows.h>
#else
#include <unistd.h>
#endif

using namespace FS;

#define TREEDEPTH 3
#define _USE_MATH_DEFINES

// a2iVertexOffset lists the positions (int), relative to vertex0,
// of each of the 8 vertices of a cube
const int MCWithRange::a2iVertexOffset[8][3] =  {
  {0, 0, 0},{1, 0, 0},{1, 1, 0},{0, 1, 0},
  {0, 0, 1},{1, 0, 1},{1, 1, 1},{0, 1, 1}
};

// a2fVertexOffset lists the positions (float), relative to vertex0,
// of each of the 8 vertices of a cube
const float MCWithRange::a2fVertexOffset[8][3] = {
  {0.0, 0.0, 0.0},{1.0, 0.0, 0.0},{1.0, 1.0, 0.0},{0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0},{1.0, 0.0, 1.0},{1.0, 1.0, 1.0},{0.0, 1.0, 1.0}
};

// a2iEdgeConnection lists the index of the endpoint vertices for each of
// the 12 edges of the cube
const int MCWithRange::a2iEdgeConnection[12][2] = {
  {0,1}, {1,2}, {2,3}, {3,0},
  {4,5}, {5,6}, {6,7}, {7,4},
  {0,4}, {1,5}, {2,6}, {3,7}
};

// a2fEdgeDirection lists the direction vector (vertex1-vertex0) for each
// edge in the cube
const float MCWithRange::a2fEdgeDirection[12][3] = {
  {1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{-1.0, 0.0, 0.0},{0.0, -1.0, 0.0},
  {1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{-1.0, 0.0, 0.0},{0.0, -1.0, 0.0},
  {0.0, 0.0, 1.0},{0.0, 0.0, 1.0},{ 0.0, 0.0, 1.0},{0.0,  0.0, 1.0}
};

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase MCWithRange::database( "MCWithRange",
                                         &(newInstance<MCWithRange>),
                                         typeid( MCWithRange ),
                                         &X3DGeometryNode::database );

namespace MCWithRangeInternals {
  FIELDDB_ELEMENT( MCWithRange, isovalue, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, voxels, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, isoStart, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, isoEnd, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, segNo, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, segStart, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, segEnd, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, materialSegmentationModel, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, smoothingMask, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, texture3D, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, material, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, hilight, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, cryoColours, INPUT_OUTPUT );
  FIELDDB_ELEMENT( MCWithRange, generateSurface, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, surfaceMesh, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, surfaceNormals, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, toggleHilight, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, hilightColours, INPUT_OUTPUT);
  FIELDDB_ELEMENT( MCWithRange, sequentialHilightMode, INITIALIZE_ONLY); 
  FIELDDB_ELEMENT( MCWithRange, currentHilightSegment, INPUT_OUTPUT);
}


MCWithRange::MCWithRange( Inst< SFNode    > _metadata,
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
                              Inst< SFMaterialSegmentationNode  > _segmentModel,
                              Inst<SFSmoothingMask > _smoothingMask,
                              Inst<SFTexture3DNode> _texture3D,
                              Inst<SFMaterialNode> _material, 
                              Inst< SFTexture3DNode > _hilight,
                              Inst< SFTexture3DNode > _cryoColours,
                              Inst< SFBool > _generateSurface,
                              Inst< MFVec3f> _surfaceMesh,
                              Inst< MFVec3f> _surfaceNormals,
                              Inst< SFBool > _toggleHilight,
                              Inst< MFColor > _hilightColours,
                              Inst< SFBool > _sequentialHilightMode,
                              Inst< SFInt32 > _currentHilightSegment):
  X3DGeometryNode( _metadata, _bound, _displayList ), 
  isovalue(_isovalue),
  octTree(_octTree),
  voxels( _voxels ),
  isoStart(_isoStart),
  isoEnd(_isoEnd),
  segNo(_segNo),
  segStart(_segStart),
  segEnd(_segEnd),
  materialSegmentationModel(_segmentModel),
  smoothingMask(_smoothingMask),
  texture3D(_texture3D), 
  material(_material), 
  hilight(_hilight),
  cryoColours(_cryoColours),
  generateSurface(_generateSurface),
  surfaceMesh(_surfaceMesh),
  surfaceNormals(_surfaceNormals),
  toggleHilight(_toggleHilight),
  hilightColours(_hilightColours),
  sequentialHilightMode(_sequentialHilightMode),
  currentHilightSegment(_currentHilightSegment){

  cout << "Constructing Marching Cubes" << endl;

  type_name = "MCWithRange";
  database.initFields( this ); 
  octTree->setOwner( this ); 
  octTree->setName( "octTree" );

  // Initialize to negative values so that we can easily identify
  // if they were set or not
  isoStart->setValue(-1);
  isoEnd->setValue(-1);
  segNo->setValue(-1);;
  segStart->setValue(-1);
  segEnd->setValue(-1);

  isovalue->setValue(10);
  sequentialHilightMode->setValue(false);
  toggleHilight->setValue(false);
  currentHilightSegment->setValue(1);

  isoStart->route(isovalue);
  isoEnd->route(isovalue);
  materialSegmentationModel->route(isovalue);
  segNo->route( isovalue );
  segStart->route( isovalue );
  segEnd->route( isovalue );

  isovalue->route( octTree );
  voxels->route( octTree );
  isoStart->route( octTree );
  isoEnd->route( octTree );
  materialSegmentationModel->route(octTree);
  smoothingMask->route(octTree);
  texture3D->route(octTree);
  hilight->route(octTree);
  material->route(octTree);
  cryoColours->route(octTree);
  generateSurface->route(octTree);
  toggleHilight->route(octTree);
  hilightColours->route(octTree);
  sequentialHilightMode->route(octTree);
  currentHilightSegment->route(octTree);

  isovalue->route( displayList );
  voxels->route( displayList );
  isoStart->route( displayList );
  isoEnd->route( displayList );
  segNo->route( displayList );
  segStart->route( displayList );
  segEnd->route( displayList );
  materialSegmentationModel->route(displayList);
  smoothingMask->route(displayList);
  texture3D->route(displayList);
  material->route( displayList);
  hilight->route(displayList);
  cryoColours->route(displayList);
  generateSurface->route(displayList);
  surfaceMesh->route(displayList);
  surfaceNormals->route(displayList);
  toggleHilight->route(displayList);
  hilightColours->route(displayList);
  sequentialHilightMode->route(displayList);
  currentHilightSegment->route(displayList);

  voxels->route( bound );
  // If cacheMode is on then the render function will generate
  // displayList errors because glNewList is called between glNewList
  // and glEndList pairs.
  displayList->setCacheMode( H3DDisplayListObject::DisplayList::OFF );
}


void MCWithRange::SFOctTree::updateMinMaxValues( OctTreeNode *tree ) {

	if ( tree->isLeaf() ) {
    // Base case, our node is a leaf.

    // Find the lowest and highest value within the volume of this leaf.
    for( uint iZ = tree->z_min; iZ < tree->z_max; ++iZ ) {
      for( uint iY = tree->y_min; iY < tree->y_max; ++iY ) {
        for( uint iX = tree->x_min; iX < tree->x_max; ++iX ) {
          
		  H3DFloat voxelValue = getVoxelValue(iX,iY,iZ);
          // Update the min/max values
          if ( voxelValue < tree->value_min ) {
            tree->value_min = voxelValue;
          }
          if ( voxelValue > tree->value_max ) {
            tree->value_max = voxelValue;
          }
        }
      }
    }
  } else {
    for( uint i=0; i<8; ++i ) {
      updateMinMaxValues( tree->children[i] );
    
      if ( tree->children[i]->value_min < tree->value_min )
        tree->value_min = tree->children[i]->value_min;
      if ( tree->children[i]->value_max > tree->value_max )
        tree->value_max = tree->children[i]->value_max;
    }
  }
}


void MCWithRange::render(){
  OctTreeNode *oct_tree = octTree->getValue();
  if( oct_tree ) {
	  oct_tree->render();
  }
}


#undef max


void MCWithRange::SFIso::update() {

	H3DFloat isoStart = static_cast<SFFloat*>(routes_in[0])->getValue();
	H3DFloat isoEnd = static_cast<SFFloat*>(routes_in[1])->getValue();
	MaterialSegmentationModel *segmentModel = static_cast<SFMaterialSegmentationNode*>(routes_in[2])->getValue();
	H3DInt32 segNo = static_cast<SFInt32*>(routes_in[3])->getValue();
	H3DInt32 segStart = static_cast<SFInt32*>(routes_in[4])->getValue();
	H3DInt32 segEnd = static_cast<SFInt32*>(routes_in[5])->getValue();

	// We need atleast one of the values to be valid (air is not considered to be a valid segment)
	if (isoStart <= 0 && isoEnd <= 0 && segNo <= 0 && segStart <= 0 && segEnd <= 0)
		return;

	// We need a valid segmentModel to be able to set the iso
	if (segmentModel == NULL)
		return;

	// We need atleast one valid segment
	MaterialSegmentationModel::RangeStruct range = MaterialSegmentationModel::RangeStruct();
    segmentModel->getSegmentRange((uint)(1), range);
	if (range.min < 0)
		return;

	// delta is a small value used when segStart is used as the start of the range. Since the start of the segment
	// returned from segmentModel is effectively the end of the previous segment, we need to add delta to the
	// start of the range. This should be smaller than the smallest data value possible.
    float delta = 1.0f / (float(numeric_limits<int>::max()) + 1.0f);

	// Get MCWithRange object
	MCWithRange *parentObj = static_cast<MCWithRange*>(owner);
	parentObj->setMaxSegmentDataVal(segmentModel->getMaxSegmentValue());

	// Clear existing data
	parentObj->setRangeStart(-1);
	parentObj->setRangeEnd(-1);
	parentObj->setIsoMid(-1);

	// Here we consider the different combinations of values that we can have and calculate the range values
	// (values outside of which will be set to zero), mid point (to fold the distribution around) and isovalue
	// accordingly. The calculations for some cases will be the same, but they are kept separate for clarity.
	// Note that the priority of the field combinations affect the calculateions. ie. if a combination of higher
	// priority is given, no combinations of lower priority will be checked (even if the combination of higher
	// priority is invalid, and therefore, isovalue has not been set). Following are the priorities:
	// 1) segNo
	// 2) segStart & segEnd
	// 3) segStart & isoEnd
	// 4) segStart
	// 5) isoStart & segEnd
	// 6) isoStart & isoEnd
	// 7) isoStart
	// 8) segEnd
	// 9) isoEnd

	// 1) if a segNo is given ignore everything else
	if (segNo > 0) {
		MaterialSegmentationModel::RangeStruct segmentRange = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(segNo, segmentRange);

		if (segmentRange.min >= 0) {
			parentObj->setRangeStart(segmentRange.min);
			parentObj->setRangeEnd(segmentRange.max);

			// segmentRange.min will be 0 after converting to range coordinates. Therefore, anything greater
			// than zero (or greater or equal to delta) should be rendered.
			value = delta;
		}
		return;
	}

	// 2) If a segStart and segEnd combination is given
	if (segStart > 0 && segEnd > 0) {

		if (segStart <= segEnd){

			MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
			segmentModel->getSegmentRange(segStart, segmentRange1);

			MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
			segmentModel->getSegmentRange(segEnd, segmentRange2);

			if (segmentRange1.min >= 0 && segmentRange2.min >= 0) {

				parentObj->setRangeStart(segmentRange1.min);
				parentObj->setRangeEnd(segmentRange2.max);

				// segmentRange1.min will be 0 after converting to range coordinates. Therefore, anything greater
				// than zero (or greater or equal to delta) should be rendered.
				value = delta;
			}
		}

		return;
	}

	// 3) If a segStart and isoEnd combination is given
	if (segStart > 0 && isoEnd > 0) {

		MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(segStart, segmentRange1);

		MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(isoEnd, segmentRange2);

		if (segmentRange1.min >= 0 && segmentRange2.min >= 0 && segmentRange1.min <= segmentRange2.min) {

			parentObj->setRangeStart(segmentRange1.min);
			parentObj->setRangeEnd(segmentRange2.max);

			// In range coordinates
			isoEnd = (isoEnd - segmentRange1.min) / (segmentRange2.max - segmentRange1.min);

			// An end is given (other than the end of the range). Therefore
			// we need to fold the distribution around the mid point
			float isoMid = (delta + isoEnd) / 2;
			parentObj->setIsoMid(isoMid);

			float maxVal = max(isoMid, (1 - isoMid));
			float isoThresh = (isoEnd - delta) / 2;

			// Convert to coordinates after folding, spreading across the range, and inverting
			value = 1 - isoThresh  / maxVal;
		}

		return;
	}

	// 4) If only segStart is given (without a matching segEnd or isoEnd)
	if (segStart > 0) {

		MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(segStart, segmentRange1);

		// End of the range should be the end of the last segment
		int lastSeg = segmentModel->getNoOfSegments() - 1;
		MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(lastSeg, segmentRange2);

		if (segmentRange1.min >= 0 && segmentRange2.min >= 0 && segmentRange1.min <= segmentRange2.min) {

			parentObj->setRangeStart(segmentRange1.min);
			parentObj->setRangeEnd(segmentRange2.max);

			// segmentRange1.min will be 0 after converting to range coordinates. Therefore, anything greater
			// than zero (or greater or equal to delta) should be rendered.
			value = delta;
		}

		return;
	}

	// 5) If an isoStart and segEnd combination is given
	if (isoStart > 0 && segEnd > 0) {

		MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(isoStart, segmentRange1);

		MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(segEnd, segmentRange2);

		if (segmentRange1.min >= 0 && segmentRange2.min >= 0 && segmentRange1.min <= segmentRange2.min) {

			parentObj->setRangeStart(segmentRange1.min);
			parentObj->setRangeEnd(segmentRange2.max);

			// isovalue is isoStart in range coordinates
			value = (isoStart - segmentRange1.min) / (segmentRange2.max - segmentRange1.min);
		}

		return;
	}

	// 6) If isoStart and isoEnd are given
	if (isoStart > 0 && isoEnd > 0) {

		if (isoStart < isoEnd) {

			MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
			segmentModel->getSegmentRange(isoStart, segmentRange1);

			MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
			segmentModel->getSegmentRange(isoEnd, segmentRange2);

			if (segmentRange1.min >= 0 && segmentRange2.min >= 0) {

				parentObj->setRangeStart(segmentRange1.min);
				parentObj->setRangeEnd(segmentRange2.max);

				// In range coordinates
				isoStart = (isoStart - segmentRange1.min) / (segmentRange2.max - segmentRange1.min);
				isoEnd = (isoEnd - segmentRange1.min) / (segmentRange2.max - segmentRange1.min);

				// An end is given (other than the end of the range). Therefore
				// we need to fold the distribution around the mid point
				float isoMid = (isoStart + isoEnd) / 2;
				parentObj->setIsoMid(isoMid);

				float maxVal = max(isoMid, (1 - isoMid));
				float isoThresh = (isoEnd - isoStart) / 2;

				// Convert to coordinates after folding, spreading across the range, and inverting
				value = 1 - isoThresh  / maxVal;
			}
		}

		return;
	}

	// 7) If only isoStart is given (without a matching segEnd or isoEnd)
	if (isoStart > 0) {

		MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(isoStart, segmentRange1);

		// End of the range should be the end of the last segment
		int lastSeg = segmentModel->getNoOfSegments() - 1;
		MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(lastSeg, segmentRange2);

		if (segmentRange1.min >= 0 && segmentRange2.min >= 0 && segmentRange1.min <= segmentRange2.min) {

			parentObj->setRangeStart(segmentRange1.min);
			parentObj->setRangeEnd(segmentRange2.max);

			// isovalue is isoStart in range coordinates
			value = (isoStart - segmentRange1.min) / (segmentRange2.max - segmentRange1.min);
		}

		return;
	}

	// 8) If only segEnd is given without a matching segStart or isoStart
	if (segEnd > 0) {

		// Start should be the start of the first segment (after air)
		MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(1, segmentRange1);

		MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(segEnd, segmentRange2);

		if (segmentRange1.min >= 0 && segmentRange2.min >= 0 && segmentRange1.min <= segmentRange2.min) {

			parentObj->setRangeStart(segmentRange1.min);
			parentObj->setRangeEnd(segmentRange2.min);

			// segmentRange1.min will be 0 after converting to range coordinates. Therefore, anything greater
			// than zero (or greater or equal to delta) should be rendered.
			value = delta;
		}

		return;
	}

	// 9) If only isoEnd is given without a matching segStart or isoStart
	if (isoEnd > 0) {

		// Start should be the start of the first segment (after air)
		MaterialSegmentationModel::RangeStruct segmentRange1 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(1, segmentRange1);

		MaterialSegmentationModel::RangeStruct segmentRange2 = MaterialSegmentationModel::RangeStruct();
		segmentModel->getSegmentRange(isoEnd, segmentRange2);

		if (segmentRange1.min >= 0 && segmentRange2.min >= 0 && segmentRange1.min <= segmentRange2.min) {

			parentObj->setRangeStart(segmentRange1.min);
			parentObj->setRangeEnd(segmentRange2.min);

			// In range coordinates
			isoEnd = (isoEnd - segmentRange1.min) / (segmentRange2.max - segmentRange1.min);

			float isoMid = (delta + isoEnd) / 2;
			parentObj->setIsoMid(isoMid);

			float maxVal = max(isoMid, (1 - isoMid));
			float isoThresh = (isoEnd - delta) / 2;

			// Convert to coordinates after folding, spreading across the range, and inverting
			value = 1 - isoThresh  / maxVal;
		}


		return;
	}
}

template< class A >
void buildNormalizedData( float *normalized_data, 
						  float * unsmoothedData,
                          void *orig_data,
                          uint width,
                          uint height,
                          uint depth,
                          uint volume_min_x,
                          uint volume_min_y,
                          uint volume_min_z,
                          uint volume_max_x,
                          uint volume_max_y,
                          uint volume_max_z,
                          float scale,
                          float bias,
                          float rangeStart,
                          float rangeEnd,
                          float isoMid,
                          float maxSegmentDataVal,
                          SmoothingMask * smoothingMask,
                          DataBounds &dataBounds) {

	A *d = (A*) orig_data;

	// Check if we are in the initial rendering (when the full cube is sent to be rendered) or if part of
	// the cube us sent to be rendered after drilling.
    bool fullCube = false;
    if ((volume_max_x - volume_min_x + 1) == width && (volume_max_y - volume_min_y + 1) == height && (volume_max_z - volume_min_z + 1) == depth)
		fullCube = true;
   
	// If a valid max segment value is given, we need to convert it to the range of the normalized data
	float maxDataTypeVal = float(numeric_limits<A>::max());
	//if (maxSegmentDataVal > 0) maxSegmentDataVal = (maxSegmentDataVal / maxDataTypeVal) * scale + bias;
	
	// Initialize the min max data. These are used to determine whether the changed area of the image includes
	// data for the range of values under consideration
    uint minDataX = volume_max_x;
    uint minDataY = volume_max_y;
    uint minDataZ = volume_max_z;
    uint maxDataX = volume_min_x;
    uint maxDataY = volume_min_y;
    uint maxDataZ = volume_min_z;

	// Calculate the new position of isoMid relative to the start and end of the segment
	float maxVal = max(isoMid, (1 - isoMid));

	float data;
	uint i, iNorm, iMask, iSmooth;
 
    for (uint z = volume_min_z; z<=volume_max_z; ++z) {
        for (uint y = volume_min_y; y<=volume_max_y; ++y) {
            for (uint x = volume_min_x; x<=volume_max_x; ++x) {

				i = ( ( z * height + y ) * width + x );

				// If the segment data max is not the same as the data type max, we need to convert to the range
				// of the data indicated in the segment model
				if (maxSegmentDataVal > 0)
					data = (((float)d[ i ]) / maxSegmentDataVal) * scale + bias;
				else 
					data = (((float)d[ i ]) / maxDataTypeVal) * scale + bias;
						
				// if data is out of range, assign min or max values to it
				if (data < 0) 
					data = 0;
				if (data > 1) 
					data = 1.0;	
				
				// When using maxSegmentDataVal > 0, some segments don't seem to get rendered. The sleep seems to 
				// force the caluclations to be performed before rendering. It only needs to be done when rendering 
				// the full surface (ie. not each time when material is removed).
                if (fullCube){
#ifdef Q_OS_WIN
                                   //Sleep(0.000001); // Only takes integer ms
#else
                                   //sleep(0.000001);
#endif
                }

				// A valid data range has been set. this indicates that the data for the structure under consideration
				// does not span the full range [0 1]
				if (rangeStart >= 0 && rangeEnd > 0) {

					// Remove anything that doesn't belong to the range. this is done so that the data outside our
					// valid range doesn't mess up the surface intersection calculations
					if (data <= rangeStart || data > rangeEnd) {
						data = 0;
					} else {
                        // Spread out the data belonging to the segment
                        data = (data - rangeStart) / (rangeEnd - rangeStart);
  					}

					// A valid isoMid value has been set. this indicates that the data should be folded
					// around isoMid
					if (isoMid > 0) {

						// Get the distance around isoMid
						data = isoMid - data;
						if (data < 0) data = data * (-1);

						// Spread the data across the full range [0 1] and invert, so that data on isoMid falls
						// on 1 and data farthest to isoMid falls on 0
						data = 1 - data / maxVal;
					}
				}
				
				if ((fullCube && data > 0) || (!fullCube && unsmoothedData[i] > 0)) {
					if (minDataX > x) minDataX = x;
					if (minDataY > y) minDataY = y;
					if (minDataZ > z) minDataZ = z;
					if (maxDataX < x) maxDataX = x;
					if (maxDataY < y) maxDataY = y;
					if (maxDataZ < z) maxDataZ = z;
				}
					
				// update matrices. normalized_data updated here in case a smoothing mask is not defined. it will be 
				// updated later when smoothing
				normalized_data[i] = data;
				unsmoothedData[i] = data;
			}
		}
	}

	// No smoothing required if no valid mask is given
	if (smoothingMask == NULL || smoothingMask->getMask() == NULL)
		return;
	
	// Get the bounding box of the data in our range
    int xLength = maxDataX - minDataX + 1;
    int yLength = maxDataY - minDataY + 1;
    int zLength = maxDataZ - minDataZ + 1;

	// The area under consideration doesn't have any data falling into the right partition
    if (xLength <= 0 || yLength <= 0 || zLength <= 0)
        return;

    /*
	double time1, time2;
    time1 = fullCube ? double(H3D::TimeStamp()) : 0.0;
    */
        
	// Get smoothing mask data
	float * mask = smoothingMask->getMask();
	uint maskSize = smoothingMask->getMaskSize();
	float maskSum = smoothingMask->getMaskSum();
    int maskRad = (maskSize - 1) / 2;
    
    // set data bounds if initial rendering
	if (fullCube) {
		
        dataBounds.xMin = float(minDataX); //TODO: Invetigate why float?
        dataBounds.yMin = float(minDataY);
        dataBounds.zMin = float(minDataZ);
        dataBounds.xMax = float(maxDataX);
        dataBounds.yMax = float(maxDataY);
        dataBounds.zMax = float(maxDataZ);
	}
    
    // add padding
	int padding = maskRad;
	minDataX = (uint)max((int)minDataX - padding, 0);
	minDataY = (uint)max((int)minDataY - padding, 0);
	minDataZ = (uint)max((int)minDataZ - padding, 0);
	maxDataX = (uint)min((int)maxDataX + padding, (int)width - 1);
	maxDataY = (uint)min((int)maxDataY + padding, (int)height - 1);
	maxDataZ = (uint)min((int)maxDataZ + padding, (int)depth - 1);
	xLength = maxDataX - minDataX + 1;;
	yLength = maxDataY - minDataY + 1;;
	zLength = maxDataZ - minDataZ + 1;;

    float * smoothDataArray = new float[xLength * yLength * zLength]();
	int xNorm, yNorm, zNorm, xMask, yMask, zMask;

	// For all points in the given area
    for (int z = 0; z < zLength; ++z) {
        for (int y = 0; y < yLength; ++y) {
            for (int x = 0; x < xLength; ++x) {
				iSmooth = ( ( z * yLength + y ) * xLength + x );

				// Get coordinates in the normalized_data (and unsmoothedData) matrix
				xNorm = x + minDataX;
				yNorm = y + minDataY;
				zNorm = z + minDataZ;

				uint iNorm = ( ( zNorm * height + yNorm ) * width + xNorm );

				float sum = 0;
				float usedMaskSum = maskSum;

				// Place the middle of the mask on the point under consideration and perform
				// convolution operation
				for (int z1 = zNorm - maskRad; z1 <= zNorm + maskRad; ++z1) {
					for (int y1 = yNorm - maskRad; y1 <= yNorm + maskRad; ++y1) {
						for (int x1 = xNorm - maskRad; x1 <= xNorm + maskRad; ++x1) {

							// Get mask coordinates
							xMask = x1 - xNorm + maskRad;
							yMask = y1 - yNorm + maskRad;
							zMask = z1 - zNorm + maskRad;
							iMask = (zMask * maskSize * maskSize + yMask * maskSize + xMask);

							// If the coordinates of the point fall inside the data cube, multiply by mask value
							// and add to the sum. Otherwise, remove the mask value from the sum so that it doesn't
							// get used in the calculation.
							if (z1 >= 0 && z1 < int(depth) && y1 >= 0 && y1 < int(height) && x1 >= 0 && x1 < int(width) ) {
								iNorm = ( ( z1 * height + y1 ) * width + x1 );
								sum = sum + unsmoothedData[iNorm] * mask[iMask];
							} else {
								usedMaskSum = usedMaskSum - mask[iMask];
							}
						}
					}
				}

				// usedMaskSum should never be 0 as we always place the center of the mask on a valid voxel.
				// This is just a precaution in case something weird has happened
				if (usedMaskSum > 0) smoothDataArray[iSmooth] = sum / usedMaskSum;
			}
		}
	}

	// Assign the smooth data values to normalized_data to be used in the rendering
    for (int z = 0; z < zLength; ++z) {
        for (int y = 0; y < yLength; ++y) {
            for (int x = 0; x < xLength; ++x) {

				xNorm = x + minDataX;
				yNorm = y + minDataY;
				zNorm = z + minDataZ;

				iNorm = ( ( zNorm * height + yNorm ) * width + xNorm );
				iSmooth = ( ( z * yLength + y ) * xLength + x );

                normalized_data[iNorm] = smoothDataArray[iSmooth];
			}
		}
	}

	delete [] smoothDataArray;

	/*
	// if initial rendering
	if (fullCube) {
		time2 = H3D::TimeStamp();
		cout << "Time taken for smoothing = " << time2 - time1 << endl;
	}
	*/
}


void MCWithRange::SFOctTree::updateDataMatrix( Image *image,
                                                uint volume_min_x,
                                                uint volume_min_y,
                                                uint volume_min_z,
                                                uint volume_max_x,
                                                uint volume_max_y,
                                                uint volume_max_z,
                                                TextureProperties *tp) {

  if( image ) {

    SmoothingMask *smoothingMask = static_cast<SFSmoothingMask*>(routes_in[5])->getValue();

	// Get range info from parent
	MCWithRange *parentObj = static_cast<MCWithRange*>(owner);
	float rangeStart = parentObj->getRangeStart();
	float rangeEnd = parentObj->getRangeEnd();
	float isoMid = parentObj->getIsoMid();
	float maxSegDataVal = parentObj->getMaxSegmentDataVal();
	
    uint d = image->depth();
    uint h = image->height();
    uint w = image->width();

    float bias = 0;
    float scale = 1;

    if( tp ) {
      bias = tp->textureTransferBias->getValue().x;
      scale = tp->textureTransferScale->getValue().x;
    }
 
    if( image->pixelType() == Image::LUMINANCE ) {
      Image::PixelComponentType type = image->pixelComponentType();
      if( type ==Image::UNSIGNED ) {
        switch( image->bitsPerPixel() ) {
        case 8:
        	buildNormalizedData< unsigned char >( data_matrix, unsmoothedDataMatrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias, rangeStart, rangeEnd, isoMid, maxSegDataVal, smoothingMask, dataBounds );
        	break;
        case 16:
        	buildNormalizedData< unsigned short >( data_matrix, unsmoothedDataMatrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias, rangeStart, rangeEnd, isoMid, maxSegDataVal, smoothingMask, dataBounds );
        	break;
        case 32: 
        	buildNormalizedData< uint >( data_matrix, unsmoothedDataMatrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias, rangeStart, rangeEnd, isoMid, maxSegDataVal, smoothingMask, dataBounds );
        	break;
        default: 
          Console(4) << "Warning: MCWithRange::buildDataMatrix failed." << endl;
        }
      } else if( type == Image::SIGNED ) {
        switch( image->bitsPerPixel() ) {
        case 8:  
        	buildNormalizedData< char >( data_matrix, unsmoothedDataMatrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias, rangeStart, rangeEnd, isoMid, maxSegDataVal, smoothingMask, dataBounds );
        	break;
        case 16:
        	buildNormalizedData< short >( data_matrix, unsmoothedDataMatrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias, rangeStart, rangeEnd, isoMid, maxSegDataVal, smoothingMask, dataBounds );
        	break;
        case 32: 
        	buildNormalizedData< int >( data_matrix, unsmoothedDataMatrix, image->getImageData(), w, h, d, volume_min_x, volume_min_y, volume_min_z, volume_max_x, volume_max_y, volume_max_z, scale, bias, rangeStart, rangeEnd, isoMid, maxSegDataVal, smoothingMask, dataBounds );
        	break;
        default: 
          Console(4) << "Warning: MCWithRange::buildDataMatrix failed." << endl;
        }
      } else if( type == Image::RATIONAL ) {
        float *d = (float * )image->getImageData();
        
        for (uint z = volume_min_z; z<volume_max_z; ++z) {
          for (uint y = volume_min_y; y<volume_max_y; ++y) {
            for (uint x = volume_min_x; x<volume_max_x; ++x) {
              uint i = ( ( z * h + y ) * w + x );
              data_matrix[i] =   d[ i ] * scale + bias;
              if( data_matrix[i] < 0 ) data_matrix[i] = 0;
            }
          }
        }
      } else {
        Console(4) << "Warning: MCWithRange::buildDataMatrix failed." << endl;
      }
    } else {
      Console(4) << "Warning: MCWithRange::buildDataMatrix failed." << endl;
    }      
  }
}


void MCWithRange::SFOctTree::buildDataMatrix( Image *image, TextureProperties * tp ) {
  if( image ) {
    uint d = image->depth();
    uint h = image->height();
    uint w = image->width();

    uint data_matrix_size = x_points * y_points * z_points;
    data_matrix= new float[ data_matrix_size ];
    unsmoothedDataMatrix= new float[ data_matrix_size ];
    dataBounds = DataBounds();
    updateDataMatrix( image, 0, 0, 0, w-1, h-1, d-1, tp );
  }
}
 

void MCWithRange::SFOctTree::buildTextureMatrix(Image *textureImage) {
	if (textureImage) {	
		
		cout << "Loading texture matrix\n";
		
		unsigned char *texData = (unsigned char*) textureImage->getImageData();	
		uint bits = textureImage->bitsPerPixel();
		
		// for now only works for luminance where there is only one channel
		float maxVal = float(pow(2.0, (int) bits) - 1.0); 
		
		if (texData) {
			texSizeZ = textureImage->depth();
			texSizeY = textureImage->height();
			texSizeX = textureImage->width();
					
			textureMatrix = new float[texSizeX * texSizeY * texSizeZ];
			for (uint z = 0; z < texSizeZ; ++z) {
				for (uint y = 0; y < texSizeY; ++y) {
					for (uint x = 0; x < texSizeX; ++x) {
						uint i = ((z * texSizeY + y) * texSizeX + x);
						textureMatrix[i] = float(texData[i]) / maxVal;
						
						if (textureMatrix[i] < 0 || textureMatrix[i] > 1)
							cout << textureMatrix[i] << endl;
					}
				}
			}
		}
	}
}

// Builds hilight matrix and gives each value appearing there a unique identifier starting from 1.
// This ensures that the colours given in hilightColours vector are assigned to each value in hilightImage
// in ascending order
void MCWithRange::SFOctTree::buildHilightMatrix(Image *hilightImage) {
	
	unsigned char *hilightData = (unsigned char*) hilightImage->getImageData();
	vector<uint> uniqueHilightVals; 
	int uniqueIdx = -1;
		
	if (hilightData) {
			
		hilightSizeX = hilightImage->width();
		hilightSizeY = hilightImage->height();
		hilightSizeZ = hilightImage->depth();
		
		// check for sizes 
		if (x_points != hilightSizeX || y_points != hilightSizeY || z_points != hilightSizeZ) {
			cout << "Highlight matrix size should match that of the data matrix!\n";
			return;
		}
		
		cout << "Loading highlight matrix\n";
			
		uint hilightMatrixSize = hilightSizeX * hilightSizeY * hilightSizeZ;
		hilightMatrix = new uint[hilightMatrixSize];
		for (uint z = 0; z < hilightSizeZ; ++z) {
			for (uint y = 0; y < hilightSizeY; ++y) {
				for (uint x = 0; x < hilightSizeX; ++x) {
					uint i = ((z * hilightSizeY + y) * hilightSizeX + x);
					uint hilightVal = (uint) hilightData[i];
					hilightMatrix[i] = hilightVal;
					
					// for valid hilight values
					if (hilightVal > 0) {
						
						// see if value has already been added
						uniqueIdx = -1;
						for (uint idx = 0; idx < uniqueHilightVals.size(); idx++) {
							if (uniqueHilightVals[idx] == hilightVal) {
								uniqueIdx = idx;
							}
						}
							
						// if value not found in the list
						if (uniqueIdx < 0) {
							uniqueHilightVals.push_back(hilightVal);
						}
					}
				}
			}
		}
		
		// sort unique hilight values
		vector<uint> sortedHilightVals = uniqueHilightVals;
		std::sort (sortedHilightVals.begin(), sortedHilightVals.end());
		
		// we assume that hilight values can only go from 0-255 (with 0 being no hilight and 1-255 being valid colours)
		// this indexing method makes it faster to set the rank of the hilight value
		uint hilightValIndices[256] = {0}; 
		
		for (uint idx = 0; idx < sortedHilightVals.size(); idx++) {
			hilightValIndices[sortedHilightVals.at(idx)] = idx + 1;
		}
		
		// assign sorted ranks to the hilight matrix
		for (uint idx = 0; idx < hilightMatrixSize; idx++) {
			uint currVal = hilightMatrix[idx];
			
			if (currVal > 0) {
				hilightMatrix[idx] = hilightValIndices[currVal];
			}
		}
	}
}


// Set the hilight colour per value in hilightData
void MCWithRange::SFOctTree::setHilightColours(vector<RGB> hilightCols) {
	hilightColours = hilightCols;
}

// Build the matrix that hold colour per voxel. Note that cryoColourImage should be RGB
void MCWithRange::SFOctTree::buildCryoColourMatrix(Image *cryoColourImage) {
	unsigned char *cryoColourData = (unsigned char*) cryoColourImage->getImageData();
	
	if (cryoColourData) {
		cryoColoursizeZ = cryoColourImage->depth();
		cryoColoursizeY = cryoColourImage->height();
		cryoColoursizeX = cryoColourImage->width();
		
		// check for sizes 
		if (x_points != cryoColoursizeX || y_points != cryoColoursizeY || z_points != cryoColoursizeZ) {
			cout << "Colour matrix size should match that of the data matrix!\n";
			return;
		}
		
		cout << "Loading colour matrix\n";
		cryoColourMatrix = new RGB[cryoColoursizeX * cryoColoursizeY * cryoColoursizeZ];
		unsigned short numChannels = (unsigned short) cryoColourImage->bitsPerPixel() / 8;
		
		for (uint z = 0; z < cryoColoursizeZ; ++z) {
			for (uint y = 0; y < cryoColoursizeY; ++y) {
				for (uint x = 0; x < cryoColoursizeX; ++x) {
					
					// index in colour data matrix and start index of cryoColourData  
					uint i = ((z * cryoColoursizeY + y) * cryoColoursizeX + x);
					uint j = i * numChannels;
					
					// RGB channel values
                    float r = float(cryoColourData[j]) / 255.0f;
                    float g = float(cryoColourData[j+1]) / 255.0f;
                    float b = float(cryoColourData[j+2]) / 255.0f;
					RGB colour = RGB(r, g, b);
					cryoColourMatrix[i] = colour;
				}
			}
		}
	} 
}

// return intensity of voxel  given indices
float MCWithRange::SFOctTree::getTextureValue(uint x, uint y, uint z ) {
	x = x % texSizeX;
	y = y % texSizeY;
	z = z % texSizeZ;
    return (textureMatrix ? textureMatrix[ ( z*texSizeY + y ) * texSizeX + x ] : 1.0f);
}


// return hilight value of voxel  given indices
uint MCWithRange::SFOctTree::getHilightValue( uint x, uint y, uint z ) {
	uint res = 0;
	if (hilightMatrix && x >= 0 && x < hilightSizeX && y >= 0 && y < hilightSizeY && z >= 0 && z < hilightSizeZ)
		res = hilightMatrix[ ( z*hilightSizeY + y ) * hilightSizeX + x ];
	
	return res;
}

// return colour of voxel  given indices
RGB MCWithRange::SFOctTree::getCryoColourValue( uint x, uint y, uint z ) {
	RGB colour = RGB(0.0, 0.0, 0.0);
	if (cryoColourMatrix && x >= 0 && x < cryoColoursizeX && y >= 0 && y < cryoColoursizeY && z >= 0 && z < cryoColoursizeZ)
		colour =  cryoColourMatrix[ ( z*cryoColoursizeY + y ) * cryoColoursizeX + x ];
	
	return colour;
}
	  
 
void MCWithRange::SFOctTree::update() {
	H3DFloat iso_value = static_cast<SFFloat*>(routes_in[0])->getValue(); 
	X3DTexture3DNode *texture = static_cast< SFTexture3DNode * >( routes_in[1])->getValue();
	X3DTexture3DNode *texture3D = static_cast<SFTexture3DNode *>(routes_in[6])->getValue();
	X3DTexture3DNode *hilight = static_cast<SFTexture3DNode *>(routes_in[7])->getValue();
    //Material *material = static_cast<SFMaterialNode *>(routes_in[8])->getValue();
	X3DTexture3DNode *cryoColours = static_cast<SFTexture3DNode *>(routes_in[9])->getValue();
	bool generateSurface = static_cast<SFBool*>(routes_in[10])->getValue();	
    //bool toggleHilight = static_cast<SFBool*>(routes_in[11])->getValue();
	vector<RGB> hilightCols = static_cast<MFColor*>(routes_in[12])->getValue();
    //bool seqHilightMode = static_cast<SFBool*>(routes_in[13])->getValue();
    //uint hilightSegment = static_cast<SFInt32*>(routes_in[14])->getValue();
	  
	H3D::Image *image = NULL;
	TextureProperties *texture_properties = NULL;
	bool materialUpdated = false;
	bool hilightToggled = false;

	if( texture ) {
		image = texture->image->getValue();
		texture_properties = texture->textureProperties->getValue();
	}
	
	if( !image || image->depth() == 0 || image->height() == 0 || image->width() == 0 ) {
		value = NULL;
		return;
	}
  
	// material changed after the initial update
	if (event.ptr == routes_in[8] && value.get() != NULL) {
		materialUpdated = true;
	}
	
	// hilight toggled after the initial update
	if (event.ptr == routes_in[11] && value.get() != NULL) {
		hilightToggled = true;
	}
	
	// first time - build all matrices and generate triangles for full volume
	if( function_count == 0 && ( !texture->image->imageChanged() || data_matrix==0 ) && !materialUpdated && !hilightToggled) {
   
		if ( data_matrix != NULL ) delete [] data_matrix;
		if ( unsmoothedDataMatrix != NULL ) delete [] unsmoothedDataMatrix;
		if ( textureMatrix != NULL ) delete[] textureMatrix;
		if ( hilightMatrix != NULL ) delete[] hilightMatrix;
		if ( cryoColourMatrix != NULL ) delete[] cryoColourMatrix;
    
		x_points = y_points = z_points = 0;

		Vec3f size( 1, 1, 1 );

		if( image ) {
		  x_points = image->width();
		  y_points = image->height();
		  z_points = image->depth();
				
		  // MarchingCubes can not be applied if any of the dimensions is
		  // less than 2.
		  if( x_points < 2 || y_points < 2 || z_points < 2 ) {
			value = NULL;
			return;
		  }
				
		  voxel_size = image->pixelSize();
		  size = Vec3f( x_points * voxel_size.x,
						y_points * voxel_size.y,
						z_points * voxel_size.z );
		  buildDataMatrix( image, texture_properties );
								
		  if (hilight) {
			buildHilightMatrix(hilight->image->getValue());
		  }
		  
		  if (!hilightCols.empty()) {
				setHilightColours(hilightCols);
		  }
		 
		  if (texture3D) {
			H3D::Image *texImage = texture3D->image->getValue();
			buildTextureMatrix(texImage);
		  }
				
		  if (cryoColours) {
			buildCryoColourMatrix(cryoColours->image->getValue());
		  }
	} 
    	
	#ifndef GRADIENTS_ON_THE_FLY
    gradients.resize( x_points * y_points * z_points );

    updateGradients( 0, x_points, 0, y_points, 0, z_points,
                     gradients );
	#endif

    // create tree structure....
    value = new OctTreeNode;
    value->x_min = 0;
    value->x_max = x_points-1;
    value->y_min = 0;
    value->y_max = y_points-1;
    value->z_min = 0;
    value->z_max = z_points-1;

    value->subdivide( TREEDEPTH );

    updateMinMaxValues( value.get() );

    TimeStamp t;

    vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > data;

    updateMCTriangles( value.get(),
                       iso_value,
                       &data );

    update_thread->asynchronousCallbacks( data.begin(), data.end() );
		function_count += (uint) data.size();

    old_value = iso_value;
  
	// update coming in from ADrillable node or some other update 
  } else {
	  	  
		// define update bounds
		uint xMin, yMin, zMin, xMax, yMax, zMax;
		
		// update coming from a change in material
		if (materialUpdated || hilightToggled) {
			xMin = (uint)dataBounds.xMin;
			yMin = (uint)dataBounds.yMin;
			zMin = (uint)dataBounds.zMin;
			xMax = (uint)dataBounds.xMax;
			yMax = (uint)dataBounds.yMax;
			zMax = (uint)dataBounds.zMax;
		} else {
			xMin = texture->image->xMin();
			yMin = texture->image->yMin(); 
			zMin = texture->image->zMin();
			xMax = texture->image->xMax();
			yMax = texture->image->yMax();
			zMax = texture->image->zMax();
		}
			 	  
    // TODO: if function_count > 0 accumulate changes for later update
		if( function_count == 0 || materialUpdated || hilightToggled) {
			
      TimeStamp t;
			updateDataMatrix( image, xMin, yMin, zMin, xMax, yMax, zMax, texture_properties );
			
			#ifndef GRADIENTS_ON_THE_FLY
            // TODO: change in updateGradients) : <=
				updateGradients( xMin,  xMax + 1, yMin, yMax + 1, zMin,  zMax + 1, gradients );
			#endif    

            vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > data;

			updateMCTrianglesInVolume( value.get(),iso_value, xMin, yMin, zMin, xMax, yMax, zMax, &data);
			
			function_count += (uint) data.size();
            update_thread->asynchronousCallbacks( data.begin(), data.end() );
    } 
  }
  
  // only create surface mesh and corresponding normals when asked to do so
	if (event.ptr == routes_in[10] && generateSurface) 
	  createSurfaceMesh(value.get());
  
}

// Generate surface mesh and normals, and set the values of the correponding
// variables in MCWithRange. This will only happen when a request is sent
// through generateSurface boolean
void MCWithRange::SFOctTree::createSurfaceMesh(OctTreeNode *tree){
	if( tree ) {
		MCWithRange *parentObj = static_cast<MCWithRange*>(owner);
				
		vector< Vec3f > *vertices = new vector< Vec3f >();
		vector< Vec3f > *normals = new vector< Vec3f >(); 
		vector< Vec4f > *colours = new vector< Vec4f >();

		// calculate the triangle mesh and normals anew for full volume
		vMarchingCubes( parentObj->isovalue->getValue(),
						0, x_points-1,
						0, y_points-1,
						0, z_points-1,
						*vertices, *normals, *colours );
		
		// clear existing data
		parentObj->surfaceMesh->clear();
		parentObj->surfaceNormals->clear();
		
		// set to MC variables
		parentObj->surfaceMesh->setValue(*vertices);
		parentObj->surfaceNormals->setValue(*normals);
	}
}

void MCWithRange::SFOctTree::updateGradients(   uint x_min, uint x_max,
                                                uint y_min, uint y_max,
                                                uint z_min, uint z_max,
                                                vector< Vec3f > &gradients ) {

  for ( uint z=z_min; z<z_max; ++z) {
    for (uint y=y_min; y<y_max; ++y) {
      for (uint x = x_min; x<x_max; ++x) {
        int index = ( z*y_points + y ) * x_points +x;
        Vec3f &gradient = gradients[index];

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
      }
    }
  }
}

void MCWithRange::SFOctTree::updateMCTriangles( OctTreeNode *tree,
                                                  H3DFloat iso_value, 
  vector< pair< PeriodicThread::CallbackFunc, UpdateData * > > *update_data) {

  if ( tree->isLeaf() ) {
    // this is a leaf node
    
    // if the isovalue is within this leaf generate the triangles for it.
    if ( ( iso_value <= tree->value_max &&
           iso_value >= tree->value_min ) ) {
    	
      if( !update_data ) {
		// clean out the vertex list
		tree->vvertex.clear();
		tree->vnormal.clear();
		tree->vcolour.clear();
		
		if( tree->cache != 0 ) {
		  glDeleteLists( tree->cache, 1 );
		  tree->cache = 0;
		}
		vMarchingCubes( iso_value, 
						tree->x_min, tree->x_max,
						tree->y_min, tree->y_max,
						tree->z_min, tree->z_max,
						tree->vvertex, tree->vnormal, tree->vcolour);
      } else {
        UpdateData *data = new UpdateData;
        data->iso_value = iso_value;
        data->x_min = tree->x_min;
        data->y_min = tree->y_min;
        data->z_min = tree->z_min;
        data->x_max = tree->x_max;
        data->y_max = tree->y_max;
        data->z_max = tree->z_max;
        data->mc.reset( static_cast< MCWithRange * >( getOwner() ) );
        data->oct_tree_leaf = tree;

        // make_pair does not work for VS 2010, so do it like this.
        pair< PeriodicThread::CallbackFunc, 
              UpdateData * > fd_pair( calculateNewTriangles, data ); 
        update_data->push_back( fd_pair );
      }
    } else {
      // clean out the vertex list
      tree->vvertex.clear();
      tree->vnormal.clear();
      tree->vcolour.clear();
    }

  } else {
    for( int i=0; i < 8; ++i )
      updateMCTriangles( tree->children[i],
                         iso_value, 
                         update_data);
  }
}

void MCWithRange::SFOctTree::updateMCTrianglesInVolume(
                               OctTreeNode *tree,
                               H3DFloat iso_value, 
                               uint volume_min_x,
                               uint volume_min_y,
                               uint volume_min_z,
                               uint volume_max_x,
                               uint volume_max_y,
                               uint volume_max_z,
                               vector< pair< PeriodicThread::CallbackFunc, 
                               UpdateData * > > *update_data) {

  
  if( (volume_max_x < tree->x_min && volume_min_x < tree->x_min ) ||
      (volume_max_x > tree->x_max && volume_min_x > tree->x_max ) ||
      (volume_max_y < tree->y_min && volume_min_y < tree->y_min ) ||
      (volume_max_y > tree->y_max && volume_min_y > tree->y_max ) ||
      (volume_max_z < tree->z_min && volume_min_z < tree->z_min ) ||
      (volume_max_z > tree->z_max && volume_min_z > tree->z_max ) ) {
    // updated volume does not intersect the volume this subtree occupies
    // so do nothing
    return;
  }

  if ( tree->isLeaf() ) {
    // this is a leaf node
    updateMCTriangles( tree, iso_value, 
                       update_data );
  } else {
    for( int i=0; i < 8; ++i )
      updateMCTrianglesInVolume( tree->children[i],
                                 iso_value, 
                                 volume_min_x,
                                 volume_min_y,
                                 volume_min_z,
                                 volume_max_x,
                                 volume_max_y,
                                 volume_max_z,
                                 update_data );
  }
}


// vMarchingCubes iterates over the (entire) part of the dataset that is
// inside the bounding box.
void MCWithRange::SFOctTree::vMarchingCubes( H3DFloat iso_value,
                                               uint x_min,
                                               uint x_max, 
                                               uint y_min,
                                               uint y_max, 
                                               uint z_min,
                                               uint z_max, 
                                               vector< Vec3f > &vertices,
                                               vector< Vec3f > &normals,
                                               vector< Vec4f > &colours) {

	Material *m = static_cast<SFMaterialNode *>(routes_in[8])->getValue();
	bool toggleHilight = static_cast<SFBool*>(routes_in[11])->getValue(); 
	bool sequentialHilightMode = static_cast<SFBool*>(routes_in[13])->getValue();
	uint hilightSegment = static_cast<SFInt32*>(routes_in[14])->getValue();
	
	RGB diffuseColor;
	H3DFloat trans;
	
	if (m) {
		diffuseColor = m->diffuseColor->getValue();
		trans = 1 - m->transparency->getValue();
	} else {
		diffuseColor = RGB(1.0, 1.0, 1.0);
		trans = 1;
	}
  	
	for(uint iZ = z_min; iZ <= (z_max<z_points-1?z_max:z_max-1); ++iZ) {  
		for(uint iY = y_min; iY <= (y_max<y_points-1?y_max:y_max-1); ++iY){
			for( uint iX = x_min; iX <= (x_max<x_points-1?x_max:x_max-1); ++iX) {

				int iCorner, iVertex,  iEdge, iTriangle, iFlagIndex, iEdgeFlags;
				H3DFloat fOffset, afCubeValue[8];
				Vec3f afCubeGrad[8];
				Vec4f afColorValue[8];
				
                uint x[8], y[8], z[8];
				x[0] = iX + 0; y[0] = iY + 0; z[0] = iZ + 0;
				x[1] = iX + 1; y[1] = iY + 0, z[1] = iZ + 0;
				x[2] = iX + 1; y[2] = iY + 1, z[2] = iZ + 0;
				x[3] = iX + 0; y[3] = iY + 1, z[3] = iZ + 0;
				x[4] = iX + 0; y[4] = iY + 0, z[4] = iZ + 1;
				x[5] = iX + 1; y[5] = iY + 0, z[5] = iZ + 1;
				x[6] = iX + 1; y[6] = iY + 1, z[6] = iZ + 1;
				x[7] = iX + 0; y[7] = iY + 1, z[7] = iZ + 1;
        
				iFlagIndex = 0;
				for (int i = 0; i < 8; ++i) {
					H3DFloat voxelValue = getVoxelValue(x[i], y[i], z[i]);
					afCubeValue[i] = voxelValue;
					if (voxelValue < iso_value){
							iFlagIndex |= 1 << i;
					}
						
					// add texture and hilights
					if (textureMatrix || hilightMatrix || cryoColourMatrix) {
						H3DFloat intensity = 1;
						RGB colour;
						if (textureMatrix) {
							intensity = getTextureValue(x[i], y[i], z[i]);	
						}
						
						if (cryoColourMatrix) {
							colour = getCryoColourValue(x[i], y[i], z[i]);
						} else {
							colour = diffuseColor;
						}
						
						if (toggleHilight && hilightMatrix) {
							
						uint hilightVal = getHilightValue(x[i], y[i], z[i]);
							
						if (hilightVal > 0) {
									
							// if sequential mode we should only show segments less than or equal to the current segment
							if (!sequentialHilightMode || (sequentialHilightMode && hilightVal <= hilightSegment)){
							
							//if no colours have been given, use default
							if (hilightColours.empty()) {
								colour = defaultHilightColour;
							} else {
			
								// if sequential, use first colour in list
								if (sequentialHilightMode) {
									colour = (RGB) hilightColours.at(0);
								} else {
				
									// if we have more hilight groups than hilight colours, set the colour to the last in the list
									if (hilightVal > hilightColours.size()) {
                                        hilightVal = uint(hilightColours.size());
									}
									colour = (RGB) hilightColours.at(hilightVal - 1);
								}
							}
						}
					} 
				}
						
				// change intensity of colours
				float red = colour.r * intensity;
				float green = colour.g * intensity;
				float blue = colour.b * intensity;
				
				afColorValue[i] = Vec4f(red, green, blue, trans);
			}
		}

        //Find which edges are intersected by the surface
        iEdgeFlags = aiCubeEdgeFlags[iFlagIndex];         
        
				// If any edge is intersected by the surface.
        if ( iEdgeFlags ) {
          Vec3f asEdgeVertex[12];
          Vec3f asEdgeNorm[12];
					Vec4f asEdgeColour[12];
          
          // get the gradients for each corner
          for(iVertex = 0; iVertex < 8; ++iVertex) {
            afCubeGrad[iVertex] = -getGradient( iX + a2iVertexOffset[iVertex][0],
                                                iY + a2iVertexOffset[iVertex][1], 
                                                iZ + a2iVertexOffset[iVertex][2]); 
          }
        
          // Find the point of intersection of the surface with each edge
          // Then find the normal to the surface at those points using
          // gradients and interpolation of corner values.
          for(iEdge = 0; iEdge < 12; ++iEdge) {
            
            //if there is an intersection on this edge
            if(iEdgeFlags & (1<<iEdge)) {

              int edge_p0 = a2iEdgeConnection[iEdge][0];
              int edge_p1 = a2iEdgeConnection[iEdge][1];

			  fOffset = fGetOffset(afCubeValue[ edge_p0 ], afCubeValue[ edge_p1 ], iso_value);

              // the intersection point on the edge.
			  asEdgeVertex[iEdge] = Vec3f( 
									  ( iX +  ( a2fVertexOffset[ edge_p0 ][0]  +  
									  fOffset * a2fEdgeDirection[iEdge][0])  - 0.5f * (x_points-1) ) * voxel_size.x ,
									  ( iY + (a2fVertexOffset[ edge_p0 ][1]  +  
									  fOffset * a2fEdgeDirection[iEdge][1]) - 0.5f * (y_points-1) ) * voxel_size.y ,
									  ( iZ + (a2fVertexOffset[ edge_p0 ][2]  +  
									  fOffset * a2fEdgeDirection[iEdge][2]) - 0.5f * (z_points-1) ) * voxel_size.z);

			  // calculate by linear interpolation at the intersection point
              // gradient (x,y,z)
              // Get the normalised gradient for the intersection point
			  asEdgeNorm[iEdge] = afCubeGrad[ edge_p0 ] + fOffset*( afCubeGrad[ edge_p1 ] - afCubeGrad[ edge_p0 ] );
              asEdgeNorm[iEdge].normalizeSafe();
							
			  if (textureMatrix || hilightMatrix || cryoColourMatrix) {
				asEdgeColour[iEdge] = afColorValue[edge_p0] + fOffset * (afColorValue[edge_p1] - afColorValue[edge_p0]);
			  }
            }
          }
        
          // Draw the triangles that were found.
          // There can be up to five per cube
          for(iTriangle = 0; iTriangle < 5; ++iTriangle) {
            if(a2iTriangleConnectionTable[iFlagIndex][3*iTriangle] < 0) {
              break;
            }
											
            for(iCorner = 0; iCorner < 3; ++iCorner) {            
			  iVertex = a2iTriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];
              vertices.push_back(asEdgeVertex[iVertex]);
              normals.push_back(asEdgeNorm[iVertex] );
              
			  if (textureMatrix || hilightMatrix || cryoColourMatrix) {
			     colours.push_back(asEdgeColour[iVertex]);
			  }
            }
          }
        }
      }
    }
  }
}


MCWithRange::SFOctTree::~SFOctTree() {

	if (data_matrix != NULL) delete [] data_matrix;
	if (unsmoothedDataMatrix != NULL) delete [] unsmoothedDataMatrix;
	if (textureMatrix != NULL) delete[] textureMatrix;
	if (hilightMatrix != NULL) delete [] hilightMatrix;
	if (cryoColourMatrix != NULL) delete [] cryoColourMatrix;
}

MCWithRange::OctTreeNode::~OctTreeNode(){

	if ( isLeaf() ) {
		// if a leaf node, just remove the vectors
		vvertex.clear();
		vnormal.clear();
		vcolour.clear();
	} else {
		// if not a leaf node, delete all its children
		for( int i=0; i < 8; ++i ) {
			delete children[i];
			children[i] = NULL;
		}
	}

	parent = NULL;
	next = NULL;
}

void MCWithRange::OctTreeNode::render(  ) {

  if ( cache != 0 ) {
    glCallList( cache );
  } else if ( isLeaf() ) {
    cache = glGenLists(1);
    glNewList( cache, GL_COMPILE_AND_EXECUTE );
    
    if ( vvertex.size() > 0 ) {
		if (vcolour.size() > 0) {		
			glEnable(GL_COLOR_MATERIAL);
			glEnable(GL_BLEND);
			glEnable(GL_LIGHTING);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			
			glColorPointer(4, GL_FLOAT, 0, &(*(vcolour.begin())));
			glEnableClientState(GL_COLOR_ARRAY);
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
		} 
      
      glVertexPointer(3, GL_FLOAT, 0, &(*(vvertex.begin())) );
      glNormalPointer(GL_FLOAT, 0, &(*(vnormal.begin())) );
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_NORMAL_ARRAY);
      glDrawArrays(GL_TRIANGLES, 0, (GLsizei) vvertex.size());
      glDisableClientState(GL_VERTEX_ARRAY);
      glDisableClientState(GL_NORMAL_ARRAY);
      
		if (vcolour.size() > 0) {
			glDisableClientState(GL_COLOR_ARRAY);
			glDisable(GL_COLOR_MATERIAL);
			glDisable(GL_BLEND);
		
			// reset material attributes set in Material Node. This seems to stop some
			// strange things from happening
			GLfloat glMat[4];
			
			glGetMaterialfv( GL_BACK, GL_DIFFUSE, glMat );
			glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, glMat );
			
			glGetMaterialfv( GL_BACK, GL_AMBIENT, glMat );
			glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, glMat );
			
			glGetMaterialfv( GL_BACK, GL_SPECULAR, glMat );
			glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, glMat );
			
			glGetMaterialfv( GL_BACK, GL_SHININESS, glMat );
			glMaterialfv( GL_FRONT_AND_BACK, GL_SHININESS, glMat );
			
			glGetMaterialfv( GL_BACK, GL_EMISSION, glMat );
			glMaterialfv( GL_FRONT_AND_BACK, GL_EMISSION, glMat );	
		}
    }
    glEndList();
  } else {
    for( int i=0; i<8; ++i )
      children[i]->render();
  }
}

void MCWithRange::OctTreeNode::subdivide( int tree_depth ) {

  if ( tree_depth != 0 ) {
    int half_x = (int)floor(float(x_max-x_min)/2);
    int half_y = (int)floor(float(y_max-y_min)/2);
    int half_z = (int)floor(float(z_max-z_min)/2);

    if( half_x == 0 && half_y == 0 && half_z == 0 )
      return;

    for( int i=0; i<2; ++i ) {
      for( int j=0; j<2; ++j ) {
        for( int k=0; k<2; ++k ) {
          int index = i * 4 + j*2 + k;
          children[index] = new OctTreeNode;
          children[index]->parent = this;

          // x
          if( half_x == 0 ) {
            children[index]->x_min = x_min;
            children[index]->x_max = x_max;
          } else if ( i==0 ) {
            children[index]->x_min = x_min;
            children[index]->x_max = x_max - half_x - 1;
          } else {
            children[index]->x_min = x_max - half_x;
            children[index]->x_max = x_max;
          }

          // y
          if( half_y == 0 ) {
            children[index]->y_min = y_min;
            children[index]->y_max = y_max;
          } else if ( j==0 ) {
            children[index]->y_min = y_min;
            children[index]->y_max = y_max - half_y - 1;
          } else {
            children[index]->y_min = y_max - half_y;
            children[index]->y_max = y_max;
          }

          // z
          if( half_z == 0 ) {
            children[index]->z_min = z_min;
            children[index]->z_max = z_max;
          } else if ( k==0 ) {
            children[index]->z_min = z_min;
            children[index]->z_max = z_max - half_z - 1;
          } else {
            children[index]->z_min = z_max - half_z;
            children[index]->z_max = z_max;
          }
          
          children[index]->subdivide( tree_depth-1 );

          if ( children[index]->value_min < value_min )
            value_min = children[index]->value_min;
          if ( children[index]->value_max > value_max )
            value_max = children[index]->value_max;
        }
      }
    }
  }
}


void MCWithRange::SFBound::update() {
  X3DTexture3DNode *tex = 
    static_cast<SFTexture3DNode*>(routes_in[0])->getValue();
  H3D::Image *image = NULL;
  if( tex ) image = tex->image->getValue();
  BoxBound *bb; 
  BoxBound *b =  dynamic_cast< BoxBound * >( value.get() );
  if( b ) bb = b;
  else bb = new BoxBound;
  
  if( image ) {
    Vec3f voxel_size = image->pixelSize();
    bb->size->setValue( Vec3f( voxel_size.x * image->width(),
                               voxel_size.y * image->height(),
                               voxel_size.z * image->depth() ) );
  } else {
    bb->size->setValue( Vec3f( 0, 0, 0 ) );
  }
  value = bb;
}

// fGetOffset finds the approximate point of intersection of the surface
// between two points with the values fValue1 and fValue2
float MCWithRange::SFOctTree::fGetOffset( float fValue1,
                                            float fValue2, 
                                            float fValueDesired) {
  float fDelta = fValue2 - fValue1;

  if(fDelta == 0.0) {
    return 0.5;
  }
  return (fValueDesired - fValue1)/fDelta;
}


// For any edge, if one vertex is inside of the surface and the other is
// outside of the surface then the edge intersects the surface
// For each of the 8 vertices of the cube can be two possible states :
// either inside or outside of the surface
// For any cube the are 2^8=256 possible sets of vertex states
// This table lists the edges intersected by the surface for all 256 possible
// vertex states There are 12 edges. For each entry in the table, if edge #n is
// intersected, then bit #n is set to 1
const int MCWithRange::aiCubeEdgeFlags[256]=
  {
    0x000, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00, 
    0x190, 0x099, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90, 
    0x230, 0x339, 0x033, 0x13a, 0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30, 
    0x3a0, 0x2a9, 0x1a3, 0x0aa, 0x7a6, 0x6af, 0x5a5, 0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 
    0x460, 0x569, 0x663, 0x76a, 0x066, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60, 
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0x0ff, 0x3f5, 0x2fc, 0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0, 
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x055, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950, 
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0x0cc, 0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0, 
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0x0cc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0, 
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x055, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650, 
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0x0ff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0, 
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f, 0x066, 0x76a, 0x663, 0x569, 0x460, 
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0x0aa, 0x1a3, 0x2a9, 0x3a0, 
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x033, 0x339, 0x230, 
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x099, 0x190, 
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x000
  };



// For each of the possible vertex states listed in aiCubeEdgeFlags there is
// a specific triangulation of the edge intersection points. 
// a2iTriangleConnectionTable lists all of them in the form of 0-5 edge triples
// with the list terminated by the invalid value -1. For example:
// a2iTriangleConnectionTable[3] list the 2 triangles formed when corner[0] 
// and corner[1] are inside of the surface, but the rest of the cube is not.

// lists the edges that are intersected for all 256 combinations of intersections
const int MCWithRange::a2iTriangleConnectionTable[256][16] =
  {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
  };

PeriodicThread::CallbackCode 
MCWithRange::SFOctTree::calculateNewTriangles( void * d ) {
  UpdateData *data = static_cast< UpdateData * >( d );

  vector < Vec3f > *vertices = new vector< Vec3f >();
  vector < Vec3f > *normals = new vector< Vec3f >();
  vector < Vec4f > *colours = new vector<Vec4f>();

  vertices->reserve( data->oct_tree_leaf->vvertex.size() );
  normals->reserve( data->oct_tree_leaf->vnormal.size() );
  colours->reserve(data->oct_tree_leaf->vcolour.size());

  data->mc->octTree->vMarchingCubes( data->iso_value,
                                     data->x_min, data->x_max,
                                     data->y_min, data->y_max,
                                     data->z_min, data->z_max,
									 *vertices, *normals, *colours );

  TransferData *transfer_data = new TransferData;

  transfer_data->vertices.reset( vertices );
  transfer_data->normals.reset( normals );
  transfer_data->colours.reset(colours);
  transfer_data->mc.reset( data->mc.get() );
  transfer_data->oct_tree_leaf = data->oct_tree_leaf ; 

  delete data;

  Scene::addCallback( transferUpdatedTriangles, transfer_data );
  return PeriodicThread::CALLBACK_DONE;
}

Scene::CallbackCode
  MCWithRange::SFOctTree::transferUpdatedTriangles( void *d ) {
  TransferData *data = static_cast< TransferData * >( d );

  data->oct_tree_leaf->vvertex.swap( *data->vertices );
  data->oct_tree_leaf->vnormal.swap( *data->normals );
  data->oct_tree_leaf->vcolour.swap(*data->colours);

  if( data->oct_tree_leaf->cache != 0 ) {
    glDeleteLists( data->oct_tree_leaf->cache, 1 );
    data->oct_tree_leaf->cache = 0; 
  }
  --(data->mc->octTree->function_count);
  if( data->mc->octTree->function_count == 0)
    data->mc->displayList->breakCache();

  delete data;

  return Scene::CALLBACK_DONE;
}
