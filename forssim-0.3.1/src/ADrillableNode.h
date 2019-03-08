//////////////////////////////////////////////////////////////////////////////
//    Copyright 2008 Forsslund Systems AB
//
//    ADrillableNode is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    ADrillableNode is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with ADrillableNode; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    If you wish to use the code in a proprietary project, please contact
//    jonas@forsslundsystems.se for more information.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _ADrillableNode_H
#define	_ADrillableNode_H
#include <H3D/H3DForceEffect.h>
#include <H3D/SFVec3f.h>
#include <H3D/MFFloat.h>
#include <H3D/MFInt32.h>
#include <H3D/SFBool.h>
#include <H3D/Node.h>
#include <map>
#include <algorithm>
#include "AHapticDrillForce.h"
#include "VolumeModel.h"
#include "MaterialSegmentationModel.h"
//#include "DrillingStepsNode.h"
#include <cmath>

namespace FS
{
/** \brief Makes objects drillable
	Makes the scene drillable, i.e voxels can be removed when coming in contact with the drill head
 */
class ADrillableNode: public H3D::X3DChildNode
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
	


	/**
	 * Constructor
	 * \param position Position of drillhead.
	 * \param cutRadius The size of the drillhead.
	 * \param drillSpeed The speed of the drill i.e. the impact the drill will have on the material
	 * \param volumeModel A pointer to the volume within which we bore plus its associated properties. These properties
     * include for example the segment to which each voxel belongs.
	 * \param segmentModel A pointer to an instance of the node containing information about each segment, e.g. hardness.
	 */
	ADrillableNode( H3D::Inst< H3D::SFVec3f > _position = 0,
			H3D::Inst< H3D::SFVec3f > _appliedForce = 0,
			H3D::Inst< H3D::SFFloat > _cutRadius = 0,
			H3D::Inst< H3D::SFFloat > _drillSpeed = 0,
			H3D::Inst< H3D::SFNode > _volumeModel = 0,
			H3D::Inst< H3D::SFNode > _segmentModel = 0,
			H3D::Inst< H3D::SFNode > _drillingStepsField = 0,
			H3D::Inst< H3D::SFFloat > _currentFPS = 0,
			H3D::Inst< H3D::SFFloat > _playbackTime = 0, //added to enable ADrillableNode to use timestamps from playback
			H3D::Inst< H3D::SFBool  > _usesMarchingCubes = 0,
			H3D::Inst< H3D::SFBool  > _hasPedal = 0,

            H3D::Inst< H3D::SFInt32 > _onlyCutSegment = 0
			);

	/**
	 * ADrillable Node initialization
	 */
	virtual void initialize();

	/**
	 * Traverse the scene graph
	 */
	virtual void traverseSG( H3D::TraverseInfo &ti );
	
	/**
	 *  function to reset drilled voxels
	 */
    //void resetDrilledVoxels();

	/**
	 * Update properties of voxels affected by the drill head such as material remaining, remove voxel if zero.
	 * Also calculate the distribution of materials which is in contact with the drill head.
	 * \param center The coordinates of the drill head
	 * \param drillImpact The impact the drill will have on the material
	 */
	void cutVolume(H3D::Vec3f center, float drillImpact);

	auto_ptr< H3D::SFVec3f > position;
	auto_ptr< H3D::SFVec3f > appliedForce;
	auto_ptr< H3D::SFFloat > cutRadius;
	auto_ptr< H3D::SFFloat > drillSpeed;
	auto_ptr< H3D::SFNode > volumeModel;
	auto_ptr< H3D::SFNode > materialSegmentationModel;
	auto_ptr< H3D::SFNode > drillingStepsField;
	auto_ptr< H3D::SFFloat > currentFPS;
	auto_ptr< H3D::SFFloat > playbackTime;
	auto_ptr< H3D::SFBool  > usesMarchingCubes;
	auto_ptr< H3D::SFBool  > hasPedal;

    auto_ptr< H3D::SFInt32 > onlyCutSegment;

	VolumeModel *volume_model;
	MaterialSegmentationModel *segment_model;

	double elapsedTime;
	double prevTime;
	H3DUtil::TimeStamp startTime;
	int frameCount;
	double ptime;
	bool firstVoxel;
	bool usingPlayback;

	/// The H3DNodeDatabase for this node.
	static H3D::H3DNodeDatabase database;
};
}

#endif	/* _ADrillableNode_H */

