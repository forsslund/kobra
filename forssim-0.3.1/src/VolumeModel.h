//////////////////////////////////////////////////////////////////////////////
//    Copyright 2008-2017 Forsslund Systems AB
//
//    ADrillForce is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    ADrillForce is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with ADrillForce; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    If you wish to use the code in a proprietary project, please contact
//    jonas@forsslundsystems.se for more information.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _VOLUMEMODEL_H
#define	_VOLUMEMODEL_H

#include <H3D/X3DChildNode.h>
#include <H3D/H3DImageObject.h>
#include <H3D/MFInt32.h>
#include <H3D/MFFloat.h>
#include <H3D/MFBool.h>
#include <H3D/SFVec3f.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/Image3DTexture.h>
#include <H3DUtil/Image.h>
#include <H3DUtil/Vec3f.h>
#include <H3DUtil/TimeStamp.h>
#include <HAPI/HAPIForceEffect.h>
#include "MFUnsignedInt8.h"
#include "MFUnsignedInt8FromNrrdFile.h"
#include "MFDoubleFromNrrdFile.h"
#include "MFieldFromNrrdFile.h"
#include "MaterialSegmentationModel.h"
#include "ForbiddenSegmentationModel.h"
#include "PartitionModel.h"
#include "Vec3i.h"
#include <map>
#include <assert.h>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <ctime>
namespace FS
{

/**
 * Represents the volume within which we bore plus its associated properties. These properties
 * include for example the material to which each voxel belongs.
 */
class VolumeModel: public H3D::X3DChildNode
{
public:
	
	/// The SFTextureNode field is dependent on the displayList field
	/// of the containing X3DTextureNode node.
	typedef H3D::DependentSFNode< H3D::X3DTexture3DNode,
	H3D::FieldRef< H3D::H3DDisplayListObject,
	H3D::H3DDisplayListObject::DisplayList,
	&H3D::H3DDisplayListObject::displayList >,
	true >
	SFTexture3DNode;


    class SFStringParent:  public AutoUpdate< SFString >{
    public:
        virtual void setValue(const string &v, int id = 0  ){
            SFString::setValue(v,id);
            VolumeModel *vm = static_cast<VolumeModel*>(getOwner());
            std::cout << "(SetValue) You sent me this string: " << value << "\n";
            vm->removePartitionByName(v,0.0);
        }
    protected:
      virtual void update(){
        SFString::update();
        //VolumeModel *vm = static_cast<VolumeModel*>(getOwner());
        std::cout << "(Update) You sent me this string: " << value << "\n";
      }
    };
	

	/**
	 * Constructor
	 */
	VolumeModel(
			H3D::Inst< VolumeModel::SFTexture3DNode  >  _imageField = 0,
			H3D::Inst< MFUnsignedInt8FromNrrdFile >     _materialSegmentationField = 0,
			H3D::Inst< MFUnsignedInt8FromNrrdFile >     _forbiddenSegmentationField = 0,
			H3D::Inst< MFUnsignedInt8FromNrrdFile >     _partitionField = 0,
			H3D::Inst< MFUnsignedInt8FromNrrdFile >     _expertDrillingStepsField = 0,
			H3D::Inst< MFDoubleFromNrrdFile >           _timeStampsField = 0,
			H3D::Inst< H3D::SFNode >                    _materialSegmentationModelField = 0,
			H3D::Inst< H3D::SFNode >                    _forbiddenSegmentationModelField = 0,
            H3D::Inst< H3D::SFNode >                    _partitionModelField = 0,
            H3D::Inst< H3D::SFVec3f >                   _nrrdOffset = 0,
            H3D::Inst< H3D::MFFloat >                   _expertFraction = 0,
            H3D::Inst< VolumeModel::SFStringParent >    _doMaskPartion = 0,
            H3D::Inst< H3D::SFFloat >                   _multiplyImageVoxels = 0,
            H3D::Inst< H3D::SFBool >                    _doSaveTimestamps = 0
	);

	/**
	 * Destructor
	 */
	~VolumeModel();

	/**
	 * Volume Model Node initialization
	 */
	virtual void initialize();

	/**
	 * Traverse Scene Graph
	 */
	virtual void traverseSG( H3D::TraverseInfo &ti );
	

	/// The H3DNodeDatabase for this node.
	static H3D::H3DNodeDatabase database;

	auto_ptr< VolumeModel::SFTexture3DNode >    imageField;
	auto_ptr< MFUnsignedInt8FromNrrdFile >      materialSegmentationField;
	auto_ptr< MFUnsignedInt8FromNrrdFile >      forbiddenSegmentationField;
	auto_ptr< MFUnsignedInt8FromNrrdFile >      partitionField;
	auto_ptr< MFUnsignedInt8FromNrrdFile >      expertDrillingStepsField;
	auto_ptr< MFDoubleFromNrrdFile >            timeStampsField;
	auto_ptr< H3D::SFNode >                     materialSegmentationModelField;
	auto_ptr< H3D::SFNode >                     forbiddenSegmentationModelField;
	auto_ptr< H3D::SFNode >                     partitionModelField;
    auto_ptr< H3D::SFVec3f >                    nrrdOffset;
    auto_ptr< H3D::MFFloat >                    expertFraction;
    auto_ptr< VolumeModel::SFStringParent >     doMaskPartition;
    auto_ptr< H3D::SFFloat >                    multiplyImageVoxels;
    auto_ptr< H3D::SFBool >                     doSaveTimestamps;

	/**
	 * Get the time since the program began
	 * \return The time in seconds
	 */
	double getElapsedTime();

	/**
	 * Set the startTime to current time
	 */
	void resetStartTime();

	/**
	 * Get the image properties of a particular voxel
	 * \param index The index to the voxel indexed from lower-left-back scalar point origin
	 * \return The voxels red value, between 0.0 and 1.0
	 */
	H3D::H3DFloat getImageValue(unsigned int index);

	/**
	 * Get the image properties of a particular voxel
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \return The voxels red value, between 0.0 and 1.0
	 */
	H3D::H3DFloat getImageValue(Vec3i p);

	/**
	 * Set the image properties of a particular voxel. Also updates the screen area to be refreshed
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \param r The red component of this voxel's RGB value
	 * \param g The green component of this voxel's RGB value
	 * \param b The blue component of this voxel's RGB value
	 */
	void setImageValue(Vec3i p, float r, float g, float b);

	/**
	 * Set the image properties of a particular voxel. Only updates the screen area is specified
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \param r The red component of this voxel's RGB value
	 * \param g The green component of this voxel's RGB value
	 * \param b The blue component of this voxel's RGB value
	 * \param setScreenArea Boolean to set if the screen area should be refreshed or not
	 */
	void setImageValue(Vec3i p, float r, float g, float b, bool setScreenArea);
	
	/**
	 * Get the value stored at a particular voxel from the original image (how the image
	 * appeared when the program started)
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \return The voxels red value, between 0.0 and 1.0
	 */
	H3D::H3DFloat getOriginalImageValue(Vec3i p);


    /**
     * Get the segment Id stored at a particular voxel
     * \param index The index to the voxel indexed from lower-left-back scalar point origin
     * \return The segment Id
     */
	unsigned char getMaterialSegmentationId(unsigned int index);

    // get segment range
	MaterialSegmentationModel::RangeStruct getMCRangeForVoxelAt(Vec3i p){
        MaterialSegmentationModel::RangeStruct range;
        unsigned char segNo = getMaterialSegmentationId(p);
        MaterialSegmentationModel* materialSegmentationModel = static_cast< MaterialSegmentationModel * >(materialSegmentationModelField->getValue());
        materialSegmentationModel->getSegmentRange(segNo,range);
        return range;
    }

    /**
     * Get the segment Id stored at a particular voxel
     * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
     * \return The segment Id
     */
	unsigned char getMaterialSegmentationId(Vec3i p);

	/**
	 * Set the segment Id stored at a particular voxel
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \param value The segment Id to set to
	 */
	void setMaterialSegmentationId(Vec3i p, unsigned char value);

    /**
     * Get the partition Id stored at a particular voxel
     * \param index The index to the voxel indexed from lower-left-back scalar point origin
     * \return The partition Id
     */
	unsigned char getPartitionId(unsigned int index);

    /**
     * Get the partition Id stored at a particular voxel
     * \param p The voxels coordinate with respect to a lower-https://webmail.hiq.se/exchange/left-back scalar point origin
     * \return The partition Id
     */
	unsigned char getPartitionId(Vec3i p);

	/**
     * Get the segment Id stored at a particular voxel
     * \param index The index to the voxel indexed from lower-left-back scalar point origin
     * \return The segment Id
     */
	unsigned char getForbiddenSegmentationId(Vec3i p);


	/**
     * Get the segment Id stored at a particular voxel
     * \param index The index to the voxel indexed from lower-left-back scalar point origin
     * \return The segment Id
     */

	/**
	 * Increments the no of voxels bored by the user for the forbidden segment stored
	 * at p
	 * \param p The voxel in question
	 */
	void incrementNoOfVoxelsBoredByUserForForbiddenSegment(Vec3i p);
	
	/**
	 * Decrements the no of voxels bored by the user for the forbidden segment stored
	 * at p
	 * \param p The voxel in question
	 */
	void decrementNoOfVoxelsBoredByUserForForbiddenSegment(Vec3i p);

	/**
	 * Increments the no of voxels bored by the user for the segment stored
	 * at p
	 * \param p The voxel in question
	 */
	void incrementNoOfVoxelsBoredByUserForSegment(Vec3i p);

    void incrementNoOfVoxelsBoredByUserForExpertSegment(Vec3i p);
    long getNoOfVoxelsBoredByUserForExpertSegment(unsigned char segment){
        return expert_segment_count[segment];
    }
	
	/**
	 * Decrements the no of voxels bored by the user for the segment stored
	 * at p
	 * \param p The voxel in question
	 */
	void decrementNoOfVoxelsBoredByUserForSegment(Vec3i p);

	/**
	 * Set the partition Id stored at a particular voxel
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \param value The partition Id to set to
	 */
	void setPartitionId(Vec3i p, unsigned char value);

	/**
	 * Get the expert drilling step Id stored at a particular voxel
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \return The expert drilling step Id
	 */
	unsigned char getExpertDrillingStepId(Vec3i p);

	/**
	 * Set the expert drilling step Id stored at a particular voxel
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \param drillingStepId The expert drilling step Id
	 */
	void setExpertDrillingStepId(Vec3i p, unsigned char drillingStepId);

	/**
	 * Does this voxel belong to the drilling step with the id expertDrillingStepId
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \return answer
	 */
	bool isExpertDrillingStep(Vec3i p, unsigned char expertDrillingStepId);

	/**
	 * Is this voxel drilled by the user or not
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \return Whether this voxel has been drilled
	 */
	bool isDrilledByUser(Vec3i p);

	/**
	 * Is this voxel drilled by the user or not
	 * \param index The index to the voxel indexed from lower-left-back scalar point origin
	 * \return Whether this voxel has been drilled
	 */
	bool isDrilledByUser(unsigned int index);

	/*
	 * Helper function when debugging. Prints out all the unique drilled by values.
	 */
	void printUniqueDrilledBy();

	/**
	 * Get how much material is remaining for a particular voxel
	 * \param index The index to the voxel indexed from lower-left-back scalar point origin
	 * \return The amount of material remaining between 0.0 and 1.0
	 */
	float getMaterialRemaining(unsigned int index);

	/**
	 * Get how much material is remaining for a parSFNodeticular voxel
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \return The amount of material remaining between 0.0 and 1.0
	 */
	float getMaterialRemaining(Vec3i p);

    /**
     * Set how much material is remaining for a particular voxel
     * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
     * \param value The amount of material remaining between 0.0 and 1.0
     */
	void setMaterialRemaining(Vec3i p, float value);

    /**
     * Get the time at which a voxel was removed. A value of -1.0 represents a
     * voxel that has not been removed.
     * \param index The index to the voxel indexed from lower-left-back scalar point origin
     * \return The time in seconds that a voxel was removed relative to program start
     */
	double getTimeStamp(unsigned int i);

    /**
     * Get the time at which a voxel was removed. A value of -1.0 represents a
     * voxel that has not been removed.
     * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
     * \return The time in seconds that a voxel was removed relative to program start
     */
	double getTimeStamp(Vec3i p);

	/**
	 * Set the time at which a voxel was removed.
	 * voxel that has not been removed.
	 * \param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 * \value The time in seconds that a voxel was removed relative to program start
	 */
	void setTimeStamp(Vec3i p, double value);

	/**
	 * Save the data regarding when voxels have been removed to file.
	 */
  string saveTimeStamps(string filename="../data/timeStamps.nrrd");

	/**
	 * Save drilling statistics. Is done on Exit.
	 */
	void saveStatistics();

	/**
	 * Return the nr of voxels drilled for a specific segment ID.
	 */
	 unsigned int noOfVoxelsRemoved(unsigned int segmentId);


	/**
	 * Remove a partition giving its name
	 */
    void removePartitionByName(string partitionName, float newAmount=0.0);

	bool isInsideVolume( H3D::Vec3f c );

	/// Returns a list of 6 cartesian neighbours of given scalar id. If no neighbour
	/// exists (out of border), id will be set to -1.
	void getNeighbours(Vec3i *vector, Vec3i id);

	unsigned int getHeight();
	unsigned int getWidth();
	unsigned int getDepth();

	float getSpacing();
	float getVolumeFromVoxels(int noOfVoxels);

	std::multimap<double, Vec3i> timestamp_map;

	/**
	 * Can be used to set the timeStamp file after VolumeModel has been initialized
	 * /param the full path to the file
	 */
	void setTimeStampFile(const string& nrrdFilePath );

	/** Returns global coordinates for field or given scalar (voxel)
	 * indexed from lower left back scalar point
	 */
	H3D::Vec3f getGlobalCoordinate();

	/** Returns global coordinates for field or given scalar (voxel)
	 *indexed from lower left back scalar point
	 */
	H3D::Vec3f getGlobalCoordinate(int i);

	/** Returns global coordinates for field or given scalar (voxel)
	 *indexed from lower left back scalar point
	 *\param p The voxels coordinate with respect to a lower-left-back scalar point origin
	 */
	H3D::Vec3f getGlobalCoordinate(Vec3i p);

	/// Returns local coordinates for field or given scalar (voxel)
	/// indexed from lower left back scalar point
	H3D::Vec3f getLocalCoordinate();
	H3D::Vec3f getLocalCoordinate(int i);
	H3D::Vec3f getLocalCoordinate(Vec3i p);

	/// Returns relative coordinates for field or given scalar (voxel)
	/// indexed from lower left back scalar point
	H3D::Vec3f getRelativeCoordinate();
	H3D::Vec3f getRelativeCoordinate(int i);
	H3D::Vec3f getRelativeCoordinate(Vec3i p);

	/// The coordinates of the center of our object, in GLOBAL coordinates
	void setGlobalCoordinate(H3D::Vec3f c);

	/// The coordinates of relative (reference) point, in GLOBAL coordinates
	void setRelativeCoordinate(H3D::Vec3f c);

	Vec3i getIndexGlobal(H3D::Vec3f c);
	Vec3i getIndexGlobalWall(H3D::Vec3f c);
	Vec3i getIndexLocal(H3D::Vec3f c);
	Vec3i getIndexLocalWall(H3D::Vec3f c);
	Vec3i getIndexRelative(H3D::Vec3f c);
	Vec3i getIndexRelativeWall(H3D::Vec3f c);

	/// For safe in-boundary calculations, returns 0 if coordinate < 0, max if
	/// coordinate above max
	Vec3i wall(Vec3i);

	//Returns the maxsize of the image.
	Vec3i getMax() { return Vec3i(maxx,maxy,maxz); }
	
	// Get maximum value allowed by bits per voxel in the image 
	float getMaxImageVal();

	// Get maximum data value present in the image 
	float getMaxSegVal();
	
	// Set the area of the screen that should be updated
	void setScreenUpdateArea(Vec3i min, Vec3i max);	
	
private:
    H3DUtil::TimeStamp lastSaveTime;

	H3DUtil::Image *imageData;
	H3DUtil::RGBA *originalImageData;			/// Stores the image when it is first read in
	
	auto_ptr<MFDoubleFromNrrdFile> timeStamps;

	H3DUtil::TimeStamp *startTime; // time Volume Model Initialized


    // 2016 expert count
    int expert_segment_count[256];
    int expert_segment_max[256];

    float spacing;
	unsigned int maxx;
	unsigned int maxy;
	unsigned int maxz;

	unsigned int width;
	unsigned int height;
	unsigned int depth;
	unsigned long size;

	bool imageUpdatedOnScreen;
	int imageRegionToUpdateOnScreen_maxX;
	int imageRegionToUpdateOnScreen_maxY;
	int imageRegionToUpdateOnScreen_maxZ;
	int imageRegionToUpdateOnScreen_minX;
	int imageRegionToUpdateOnScreen_minY;
	int imageRegionToUpdateOnScreen_minZ;
	H3DUtil::TimeStamp lastUpdateTime;

	H3D::Vec3f global;
	H3D::Vec3f relative;
	H3D::Vec3f indexZeroPos;

	H3D::MFFloat materialRemainingField;
	
	
	unsigned int getIndex(Vec3i p);
	
	//static std::multimap<double, Vec3i> timestamp_map;

	/**
	 * Exception class for VolumeModel
	 */
	class VolumeModelException: public runtime_error
	{
	public:

		/**
		 * Constructor
		 */
		VolumeModelException(const string& what) :
			runtime_error("Exception within VolumeModel: " + what)
			{
			}

		VolumeModelException(const stringstream& what) :
			runtime_error("Exception within VolumeModel: " + what.str())
			{
			}
	};

    struct minmax{
        Vec3i min;
        Vec3i max;
    };
    std::map<int,minmax> partitionBox;
    minmax getPartionBox(int partitionId);
};

}

#endif	/*VOLUMEMODEL_H */

