#include "MaterialSegmentationModel.h"

using namespace FS;

/// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase MaterialSegmentationModel::database("MaterialSegmentationModel",
		&(newInstance<MaterialSegmentationModel>),
		typeid( MaterialSegmentationModel ),
		&X3DChildNode::database );

namespace MaterialSegmentationModelInternals
{
	H3D::FIELDDB_ELEMENT( MaterialSegmentationModel, segmentNameField, 				INPUT_OUTPUT );
	H3D::FIELDDB_ELEMENT( MaterialSegmentationModel, segmentHardnessField, 		INPUT_OUTPUT );
	H3D::FIELDDB_ELEMENT( MaterialSegmentationModel, segmentForceRateField, 		INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT(MaterialSegmentationModel, noOfVoxelsBoredByUser, INPUT_OUTPUT);
	H3D::FIELDDB_ELEMENT(MaterialSegmentationModel, segmentMaxValueField, INPUT_OUTPUT);
}

MaterialSegmentationModel::MaterialSegmentationModel(
		H3D::Inst< H3D::MFString > segmentNameField,
		H3D::Inst< H3D::MFFloat  > segmentHardnessField,
		H3D::Inst< H3D::MFFloat  > segmentForceRateField,
		H3D::Inst< H3D::MFInt32  > noOfVoxelsBoredByUser,
		H3D::Inst< H3D::MFFloat  > segmentMaxValueField)
:segmentNameField(segmentNameField),
segmentHardnessField(segmentHardnessField),
segmentForceRateField(segmentForceRateField),
noOfVoxelsBoredByUser(noOfVoxelsBoredByUser),
segmentMaxValueField(segmentMaxValueField)
{
	cout << "Material Segmentation Model Constructing"<<endl;
	type_name = "MaterialSegmentationModel";
	database.initFields( this );
	cout << "Material Segmentation Model Constructed"<<endl;
}

MaterialSegmentationModel::~MaterialSegmentationModel()
{
    cout<<"Material Segmentation Model Destroyed"<<endl;
}

void MaterialSegmentationModel::initialize()
{
	cout<<"Material Segmentation Model Initializing"<<endl;

	// check if the number of segments defined within the segmentIdField is greater than zero
	if(!(segmentNameField->getValue()).size())
		throw MaterialSegmentationModelException("segmentNameField should contain one or more values");

	if(segmentHardnessField->getSize() != segmentNameField->getSize()){
		std::cout << "    asdfasdf: " << segmentHardnessField->getSize() << std::endl;
		std::cout << "    asdfasdf: " << segmentNameField->getValue()[0] << std::endl;
		throw MaterialSegmentationModelException("segmentNameField should contain the same number of values as segmentHardnessField!");
	}

	// initialize the force rate vector and set the default values to 0
	if(segmentForceRateField->getSize() != segmentNameField->getSize()){
		segmentForceRateField->clear();
		vector<float> values;
        for(unsigned int i=0;i<segmentNameField->getSize();i++)
		{
			values.push_back(0.0);
		}
		segmentForceRateField->setValue(values);
	}

	maxSegmentValue = -1.0;

	// if segmentMaxValueField is not the right size, initialise to -1 to indicate there's been an error
	if(segmentMaxValueField->getSize() != segmentNameField->getSize()){
		cout << "Invalid values given for maxValue field in the Segmentation Model" << endl;

		segmentMaxValueField->clear();
		vector<float> values;
        for(unsigned int i=0;i<segmentNameField->getSize();i++)
		{
			values.push_back(-1.0);
		}
		segmentMaxValueField->setValue(values);
	} else { // normalize the values
		vector<float> values = segmentMaxValueField->getValue();
        for(unsigned int i=0;i<segmentMaxValueField->getSize();i++) {
			if (maxSegmentValue < values[i])
				maxSegmentValue = values[i];
		}

		if (maxSegmentValue > 0) {
            for(unsigned int i=0;i<segmentMaxValueField->getSize();i++) 	{
				values[i] = values[i] / maxSegmentValue;
			}
			segmentMaxValueField->clear();
			segmentMaxValueField->setValue(values);
		}

	}

	// initialize the noOfVoxelsBoredByUser list with zeros
    typedef unsigned int uint;
    unsigned int size = uint((segmentNameField->getValue()).size());
	H3DInt32* data = new H3DInt32[size];
	for(unsigned int i = 0; i< size;i++)
		data[i] = 0;
	noOfVoxelsBoredByUser->setValueFromVoidPtr(data,size,size*sizeof(H3DInt32),this->id);
	delete[] data;

	X3DChildNode::initialize();


	cout<<"Material Segmentation Model Initialized"<<endl;
}




void MaterialSegmentationModel::incrementNoOfVoxelsBoredByUser(const unsigned char& segmentId)
{
	if(getNoOfSegments()>=segmentId)
	{
		unsigned int currentValue = noOfVoxelsBoredByUser->getValue(this->id)[segmentId];
		noOfVoxelsBoredByUser->setValue(segmentId, ++currentValue,this->id);
	}
	else
	{
		throw MaterialSegmentationModelException("MaterialSegmentationModel::incrementNoOfVoxelseBoredByUser  segmentId out of bounds");
	}

}

void MaterialSegmentationModel::decrementNoOfVoxelsBoredByUser(const unsigned char& segmentId)
{
	if(getNoOfSegments()>=segmentId)
	{
		unsigned int currentValue = noOfVoxelsBoredByUser->getValue(this->id)[segmentId];
		noOfVoxelsBoredByUser->setValue(segmentId, --currentValue, this->id);
	}
	else
	{
		throw MaterialSegmentationModelException("MaterialSegmentationModel::decrementNoOfVoxelseBoredByUser  segmentId out of bounds");
	}

}


void MaterialSegmentationModel::setHardness(const unsigned int segmentId, const float& hardness)
{
	if(getNoOfSegments()>=segmentId)
	{
		segmentHardnessField->setValue(segmentId, hardness);
		cout<<"Material hardness for segment with Id: " << (int)segmentId <<" set to "<< hardness<<endl;
	}
	else
	{
		throw MaterialSegmentationModelException("getHardness segmentId out of bounds");
	}
}

float MaterialSegmentationModel::getHardness(const unsigned int segmentId)
{
	if(getNoOfSegments()>=segmentId)
	{
		return segmentHardnessField->getValue()[segmentId];
	}
	else
	{
		throw MaterialSegmentationModelException("getHardness segmentId out of bounds");
	}
}

float MaterialSegmentationModel::getForceRate(const unsigned int segmentId)
{
	if(getNoOfSegments()>=segmentId)
	{
		return segmentForceRateField->getValue()[segmentId];
	}
	else
	{
		throw MaterialSegmentationModelException("getForceField segmentId out of bounds");
	}
}

string MaterialSegmentationModel::getName(const unsigned int segmentId)
{
	if(getNoOfSegments()>=segmentId)
	{
		return segmentNameField->getValue()[segmentId];
	}
	else
	{
		throw MaterialSegmentationModelException("getHardness segmentId out of bounds");
	}
}



std::vector<unsigned char> MaterialSegmentationModel::getSegmentIds()
{
	vector<unsigned char> segmentIds;

    for(unsigned int i=0;i < getNoOfSegments();i++)
		segmentIds.push_back(i);
	return segmentIds;
}

unsigned int MaterialSegmentationModel::getNoOfSegments()
{
    typedef unsigned int uint;
    return uint(segmentNameField->getSize());	// add one for air
}

void MaterialSegmentationModel::getSegmentRange(float val, RangeStruct& range)
{
	int segmentNo = -1;
	vector<float> segmentMaxVals = segmentMaxValueField->getValue();

    for(unsigned int i=0;i<segmentMaxValueField->getSize();i++) {
		if (val <=  segmentMaxVals[i]) {
			segmentNo = i;
			break;
		}
	}

	if (segmentNo > 0) {
		range.min = segmentMaxVals[segmentNo - 1];
		range.max = segmentMaxVals[segmentNo];
	} else {
		range.min = -1;
		range.max = -1;
	}
}

void MaterialSegmentationModel::getSegmentRange(unsigned int segNo, RangeStruct& range) {
    vector<float> segmentMaxVals = segmentMaxValueField->getValue();

	if (segNo > 0 && segNo < segmentMaxVals.size()) {
		range.min = segmentMaxVals[segNo - 1];
		range.max = segmentMaxVals[segNo];
	} else {
		range.min = -1;
		range.max = -1;
	}
}

float MaterialSegmentationModel::getMaxSegmentValue() {
	return maxSegmentValue;
}
