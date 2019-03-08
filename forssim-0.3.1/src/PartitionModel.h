/**
 * PartitionModel.h
 *
 *  Created on: Jan 13, 2009
 *      Author: barrygr
 */

#ifndef PARTITIONMODEL_H_
#define PARTITIONMODEL_H_

#include <H3D/X3DChildNode.h>
#include <H3D/MFString.h>
#include "SFUnsignedInt8.h"
#include <H3D/SFBool.h>
#include <H3D/SFFloat.h>
#include <H3D/SFInt32.h>
#include <H3D/MFFloat.h>
#include "MFUnsignedInt8.h"
#include <stdexcept>
using namespace std;
using namespace H3D;

namespace FS
{

#define UNDEFINED_PARTITION_ID 0	// Voxels not belonging to any partition
#define UNDEFINED_PARTITION_NAME "Undefined Partition"

/**
 * \brief Stores the properties of all the required partitions. Partitions are parts of a tooth,
 * such as the root or crown, each of which may well be composed of a number of different materials
 */
class PartitionModel: public H3D::X3DChildNode
{

public:

	/**
	 * Constructor
	 * \param partitionIdField A list of Partition Ids
	 * \param partitionNameField A list of Partition Names
	 */
	PartitionModel(
			H3D::Inst< MFUnsignedInt8 >		 		partitionIdField = 0,
			H3D::Inst< H3D::MFString > 				partitionNameField = 0
	);

	virtual ~PartitionModel()
	{
		cout<<"Partition Model Destroyed"<<endl;
	}

	/**
	 * Initialize the Partition Model Node
	 */
	void initialize();

	/**
	 * The H3DNodeDatabase for this node.
	 */
	static H3D::H3DNodeDatabase database;

	/**
	 * The Partition Ids
	 */
	auto_ptr< MFUnsignedInt8 > 				partitionIdField;

	/**
	 * The Partition Names
	 */
	auto_ptr< H3D::MFString > 				partitionNameField;

	/**
	 * Get the name of a partition via its id
	 * \param partitionID The partition's Id
	 */
	const string getName(const unsigned char& partitionID);

	/**
	 * Get the id of a partition via its name
	 * \param partitionName The partition's Name
	 */
    unsigned char getId(const string& partitionName);

	/**
	 * Return the list of Partition Ids associated with the Partition Model. The undefined partition is the first partition in this list
	 */
    std::vector<unsigned char> getPartitionIds();

	/**
	 * \brief Exception class for Partition Model
	 */
	class PartitionModelException: public runtime_error
	{
	public:

		/**
		 * Constructor
		 */
		PartitionModelException(const string& what) :
			runtime_error("Exception within PartitionModel: " + what)
			{
			}
	};

private:

    unsigned char getPartitionIdFieldIndex(const unsigned char& segmentId);
    unsigned char getPartitionNameFieldIndex(const string& partitionName);
    bool existsInPartitionIdField(const unsigned char& segmentId);
    bool existsInPartitionNameField(const string& segmentName);
};

}

#endif /* PARTITIONMODEL_H_ */
