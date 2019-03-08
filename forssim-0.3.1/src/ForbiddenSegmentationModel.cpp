/**
 * ForbiddenSegmentationModel.cpp
 *
 *  Originally created on: Mar 23, 2009
 *      Author: barrygr
 *  Extremely simplified by Jonas, Aug 10, 2015.
 */

#include "ForbiddenSegmentationModel.h"

using namespace FS;

// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase ForbiddenSegmentationModel::database(
		"ForbiddenSegmentationModel",
		&(newInstance<ForbiddenSegmentationModel>),
		typeid(ForbiddenSegmentationModel),
		&X3DChildNode::database);

namespace ForbiddenSegmentationModelInternals
{
H3D::FIELDDB_ELEMENT(ForbiddenSegmentationModel, segmentNameField, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(ForbiddenSegmentationModel, noOfVoxelsBoredByUser, OUTPUT_ONLY);
}

ForbiddenSegmentationModel::ForbiddenSegmentationModel(
		H3D::Inst< H3D::MFString > 				segmentNameField,
		H3D::Inst< H3D::MFInt32 > 				noOfVoxelsBoredByUser)
:segmentNameField(segmentNameField),
noOfVoxelsBoredByUser(noOfVoxelsBoredByUser)
{
	cout << "Forbidden Segmentation Model Node Constructing"<<endl;

	type_name = "ForbiddenSegmentationModel";
	database.initFields( this );       

	cout << "Forbidden Segmentation Model Node Constructed"<<endl<<endl;
}

void ForbiddenSegmentationModel::initialize()
{
	cout<<"Forbidden Segmentation Model Initializing"<<endl;

    // If list of named segments is provided, initialize each to 0
    std::vector<H3DInt32> v = noOfVoxelsBoredByUser->getValue();
    while(segmentNameField->getSize() > v.size()) v.push_back(0);
    noOfVoxelsBoredByUser->setValue(v,this->id);

	X3DChildNode::initialize();

	cout<<"Forbidden Segmentation Model Initialized"<<endl;
}

ForbiddenSegmentationModel::~ForbiddenSegmentationModel()
{
	cout<<"Forbidden Segmentation Model Detroyed"<<endl;
}

void ForbiddenSegmentationModel::incrementNoOfVoxelsBoredByUser(const unsigned char& segmentId)
{    
    if(segmentId==0) return;
    std::vector<H3DInt32> v = noOfVoxelsBoredByUser->getValue();
    while(segmentId > v.size()) v.push_back(0);
    ++v[segmentId-1];
    noOfVoxelsBoredByUser->setValue(v,this->id);
    while(noOfVoxelsBoredByUser->getSize() > segmentNameField->getSize()){
        std::vector<string> vs = segmentNameField->getValue();
        vs.push_back("unnamed forbidden");
        segmentNameField->setValue(vs,this->id);
    }
}

void ForbiddenSegmentationModel::decrementNoOfVoxelsBoredByUser(const unsigned char& segmentId)
{
    if(segmentId==0) return;
    std::vector<H3DInt32> v = noOfVoxelsBoredByUser->getValue();
    while(segmentId > v.size()) v.push_back(0);
    --v[segmentId-1];
    noOfVoxelsBoredByUser->setValue(v,this->id);
    while(noOfVoxelsBoredByUser->getSize() > segmentNameField->getSize()){
        std::vector<string> vs = segmentNameField->getValue();
        vs.push_back("unnamed forbidden");
        segmentNameField->setValue(vs,this->id);
    }
}


