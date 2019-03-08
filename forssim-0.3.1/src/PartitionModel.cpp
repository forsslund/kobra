/**
 * PartitionModel.cpp
 *
 *  Created on: Jan 13, 2009
 *      Author: barrygr
 */

#include "PartitionModel.h"

using namespace FS;

/// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase PartitionModel::database("PartitionModel",
		&(newInstance<PartitionModel>),
		typeid( PartitionModel ),
		&X3DChildNode::database );

namespace PartitionModelInternals
{
H3D::FIELDDB_ELEMENT( PartitionModel, partitionIdField, 			INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( PartitionModel, partitionNameField, 			INPUT_OUTPUT );
}

PartitionModel::PartitionModel(H3D::Inst< MFUnsignedInt8  > partitionIdField,
		H3D::Inst< H3D::MFString  > partitionNameField)
:partitionIdField(partitionIdField),
partitionNameField(partitionNameField)
{
	cout << "Partition Model Constructing"<<endl;

	type_name = "PartitionModel";
	database.initFields( this );

	cout << "Partition Model Constructed"<<endl;
}

void PartitionModel::initialize()
{
	cout<<"Partition Model Initializing"<<endl;

	// check if the number of partitions defined within the partitionIdField is greater than zero
	if(!(partitionIdField->getValue()).size())
		throw PartitionModelException("partitionIdField should contain one or more values");

	// UNDEFINED_PARTITION_ID is reserved
	if(existsInPartitionIdField(UNDEFINED_PARTITION_ID))
	{
		stringstream failStringStream;
		failStringStream << "The Id: " << (int)UNDEFINED_PARTITION_ID <<" is reserved for " << UNDEFINED_PARTITION_NAME;
		throw PartitionModelException(failStringStream.str());
	}

	if(partitionIdField->getSize() != partitionNameField->getSize())
		throw PartitionModelException("partitionNameField should contain the same number of values as partitionIdField");

	// UNDEFINED_PARTITION_NAME is reserved
	if(existsInPartitionNameField(UNDEFINED_PARTITION_NAME))
	{
		stringstream failStringStream;
		failStringStream << "The Name: " << UNDEFINED_PARTITION_NAME <<" is reserved";
		throw PartitionModelException(failStringStream.str());
	}

	X3DChildNode::initialize();

	cout<<"Partition Model Initialized"<<endl;
}

const string PartitionModel::getName(const unsigned char& partitionId)
{
	if(partitionId==UNDEFINED_PARTITION_ID)
		return UNDEFINED_PARTITION_NAME;
	else
		return partitionNameField->getValueByIndex(getPartitionIdFieldIndex(partitionId));
}

unsigned char PartitionModel::getId(const string& partitionName)
{
	if(partitionName==UNDEFINED_PARTITION_NAME)
		return UNDEFINED_PARTITION_ID;
	else
		return partitionIdField->getValueByIndex(getPartitionNameFieldIndex(partitionName));
}

std::vector<unsigned char> PartitionModel::getPartitionIds()
{
	vector<unsigned char> partitionIds;

	partitionIds.push_back(UNDEFINED_PARTITION_ID);
	vector<unsigned char>::const_iterator i;

	for(i=partitionIdField->getValue().begin();i<partitionIdField->getValue().end();i++)
		partitionIds.push_back(*i);

	return partitionIds;
}

unsigned char PartitionModel::getPartitionIdFieldIndex(const unsigned char& partitionId)
{
	MFUnsignedInt8::const_iterator partitionIdFieldIterator;
	unsigned int index;

	for(partitionIdFieldIterator=partitionIdField->begin(), index = 0;
		partitionIdFieldIterator<partitionIdField->end();
		partitionIdFieldIterator++, index++)
	{
		if(*partitionIdFieldIterator==partitionId)
			return index;
	}

	stringstream failStringStream;
	failStringStream << "Can't get the index for the Partition with Id: " << (int)partitionId <<" since it is not defined in partitionIdField";
	throw PartitionModelException(failStringStream.str());
}

unsigned char PartitionModel::getPartitionNameFieldIndex(const string& partitionName)
{
	MFString::const_iterator partitionNameFieldIterator;
	unsigned int index;

	for(partitionNameFieldIterator=partitionNameField->begin(), index = 0;
		partitionNameFieldIterator<partitionNameField->end();
		partitionNameFieldIterator++, index++)
	{
		if(*partitionNameFieldIterator==partitionName)
			return index;
	}

	stringstream failStringStream;
	failStringStream << "Can't get the index for the Partition with Name: " << partitionName <<" since it is not defined in partitionNameField";
	throw PartitionModelException(failStringStream.str());
}

bool PartitionModel::existsInPartitionIdField(const unsigned char& partitionId)
{
	MFUnsignedInt8::const_iterator partitionIdFieldIterator;

	for(partitionIdFieldIterator=partitionIdField->begin();
		partitionIdFieldIterator<partitionIdField->end();
		partitionIdFieldIterator++)
	{
		if(*partitionIdFieldIterator==partitionId)
			return true;
	}

	return false;
}

bool PartitionModel::existsInPartitionNameField(const string& partitionName)
{
	MFString::const_iterator partitionNameFieldIterator;

	for(partitionNameFieldIterator=partitionNameField->begin();
		partitionNameFieldIterator<partitionNameField->end();
		partitionNameFieldIterator++)
	{
		if(*partitionNameFieldIterator==partitionName)
			return true;
	}

	return false;
}

