/**
 * MaterialSegmentModel.h
 *
 *  Created on: Dec 15, 2008
 *      Author: parhe and barrygr
 */

#ifndef MATERIALSEGMENTMODEL_H_
#define MATERIALSEGMENTMODEL_H_

#include <H3D/X3DChildNode.h>
#include <H3D/SFBool.h>
#include <H3D/SFFloat.h>
#include <H3D/SFInt32.h>
#include <H3D/MFInt32.h>
#include <H3D/MFFloat.h>
#include <H3D/MFString.h>
#include <H3D/SFString.h>
#include "SFUnsignedInt8.h"
#include "MFUnsignedInt8.h"
#include <stdexcept>

using namespace std;
using namespace H3D;

namespace FS
{

#define AIR_SEGMENT_ID 0
#define AIR_SEGMENT_NAME "Air"
#define AIR_SEGMENT_HARDNESS 0.0

/**
 * \brief Stores the properties of all the required material segments.
 * Each type of material is given its own id with AIR_SEGMENT_ID defining that of Air. Additional
 * materials are defined from x3d. Examples of materials include those of Bone and Tooth Enamel
 */
class MaterialSegmentationModel: public H3D::X3DChildNode
{
public:

	/**
	 * Constructor
	 * \param segmentIdField A list of Segment Ids
	 * \param segmentNameField A list of Segment Names
	 * \param segmentHardnessField A list of Segment Hardnesses
	 */
	MaterialSegmentationModel(
			H3D::Inst< H3D::MFString > 				segmentNameField = 0,
			H3D::Inst< H3D::MFFloat > 				segmentHardnessField = 0,
			H3D::Inst< H3D::MFFloat > 				segmentForceRateField = 0,
			H3D::Inst< H3D::MFInt32 > 				noOfVoxelsBoredByUser = 0,
			H3D::Inst< H3D::MFFloat > 				segmentMaxValueField = 0
	);

	~MaterialSegmentationModel();

	/**
	 * Initialize the SegmentModel Node
	 */
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
	 * The Segment Names
	 */
	auto_ptr< H3D::MFString > 				segmentNameField;

	/**
	 * The Hardness of each of these segments
	 */
	auto_ptr< H3D::MFFloat > 				segmentHardnessField;

	/**
	 * The force rate of each of these segments
	 */
	auto_ptr< H3D::MFFloat > 				segmentForceRateField;


	/**
	 * The number of voxels drilled by the user for each forbidden segment with the undefined segment value
	 * stored as the first value in the noOfVoxelsBoredByUser field
	 */
	auto_ptr< H3D::MFInt32 >		noOfVoxelsBoredByUser;

	/**
	 * Maximum data values that each segment can have. This should correspond to the maximum values of each
	 * structure that the nrrd file has. If used in drilling, make sure that there is enough separation
	 * between the max data values of each segment (since the material remaining is converted back to the
	 * original data value in the nrrd file). The larger the separation, the smoother the drilling.
	 */
	auto_ptr< H3D::MFFloat > 		segmentMaxValueField;

	struct RangeStruct {
		public:
			float min;
			float max;
	};

	/**
	 * Set the hardness of a particular material
	 * \param segmentId The segment id
	 * \param hardness The hardness to set to
	 */
	void setHardness(const unsigned int segmentId, const float& hardness);

	/**
	 * Get the hardness of a particular material
	 * \param segmentId The segment id
	 * \return The hardness
	 */
    float getHardness(const unsigned int segmentId);

	/**
	 * Get the force rate of a particular material
	 * \param segmentId The segment id
	 * \return The hardness
	 */
    float getForceRate(const unsigned int segmentId);

	/**
	 * Get the name of a segment from its id
	 * \param segmentId The segment id
	 * \return The segment's name
	 */
    string getName(const unsigned int segmentId);

	/**
	 * The number of segments defined by the Segment Model including the Air Segment
	 */
    unsigned int getNoOfSegments();

	/**
	 * Return the list of Segment Ids associated with the Segment Model. The Air segment is the first segment in this list
	 * \return The Segment Ids
	 */
    std::vector<unsigned char> getSegmentIds();

	/**
	 * Return start and end values of the segment. The range start is exclusive and the range end is inclusive.
	 * ie segmentRange = (segmentStart segmentEnd]. If the value doesn't belong to a valid segment, return [-1 -1]
	 */
    void getSegmentRange(unsigned int segNo, RangeStruct& range);
    void getSegmentRange(int segNo, RangeStruct& range) { getSegmentRange((unsigned int)segNo, range); } //TODO: Hax

	/**
	 * Return start and end values of the segment that a given value falls into. The range start is exclusive and the range end is inclusive.
	 * ie segmentRange = (segmentStart segmentEnd]. If the value doesn't belong to a valid segment, return [-1 -1]
	 */
	void getSegmentRange(float val, RangeStruct& range);

	/**
	 * Return the maximum value in segmentMaxValueField
	 */
	float getMaxSegmentValue();


	class MaterialSegmentationModelException: public runtime_error
	{
	public:

		/**
		 * Constructor
		 */
		MaterialSegmentationModelException(const string& what) :
			runtime_error("Exception within SegmentModel: " + what)
			{
			}
	};


protected:
     /**
	 * Maximum value in segmentMaxValueField
	 */
	float maxSegmentValue;
  

};
}

#endif /* MATERIALSEGMENTMODEL_H_ */
