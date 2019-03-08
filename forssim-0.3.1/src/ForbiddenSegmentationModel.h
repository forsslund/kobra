/**
 * ForbiddenSegmentationModel.h
 *
 *  Created on: Mar 23, 2009
 *      Author: barrygr
 */

#ifndef FORBIDDENSEGMENTATIONMODEL_H_
#define FORBIDDENSEGMENTATIONMODEL_H_

#include <H3D/X3DChildNode.h>
#include <H3D/MFString.h>
#include <H3D/MFInt32.h>
#include "MFUnsignedInt8.h"

using namespace std;
using namespace H3D;

namespace FS
{


/**
 * \brief Defines a groups of segments in which the user is not supposed to bore
 */
class ForbiddenSegmentationModel : public H3D::X3DChildNode
{
public:


	/**
	 * Constructor
	 * \param segmentNameField A list of Forbidden Segment Names, INITIALIZE_ONLY Field
	 * \param noOfVoxelsBoredByUser The amount bored from each segment by the user, OUTPUT_ONLY Field
	 */
	ForbiddenSegmentationModel(
			H3D::Inst< H3D::MFString > 				segmentNameField = 0,
			H3D::Inst< H3D::MFInt32 > 				noOfVoxelsBoredByUser = 0
	);

	virtual ~ForbiddenSegmentationModel();

	void initialize();

	/**
	 * Increment by one the number of voxels drilled by the user for a particular segment
	 * \param segmentId The segment in question
	 */
	void incrementNoOfVoxelsBoredByUser(const unsigned char& segmentId);
	
	/**
	 * Decrement by one the number of voxels drilled by the user for a particular segment
	 * \param segmentId The segment in question
	 */
	void decrementNoOfVoxelsBoredByUser(const unsigned char& segmentId);

	/**
	 * The H3DNodeDatabase for this node.
	 */
	static H3D::H3DNodeDatabase database;

	/**
	 * The list of Forbidden Segment Names
	 */
	auto_ptr< H3D::MFString >		segmentNameField;

	/**
	 * The number of voxels drilled by the user for each forbidden segment with the undefined segment value
	 * stored as the first value in the noOfVoxelsBoredByUser field
	 */
	auto_ptr< H3D::MFInt32 >		noOfVoxelsBoredByUser;

};
}

#endif /* FORBIDDENSEGMENTATIONMODEL_H_ */
