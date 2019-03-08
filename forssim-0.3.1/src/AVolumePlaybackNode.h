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

#ifndef _AVolumePlaybackNode_H
#define	_AVolumePlaybackNode_H
#include <H3D/H3DForceEffect.h>
#include <H3D/SFVec3f.h>
#include <H3D/MFFloat.h>
#include <H3D/Node.h>
#include "AHapticDrillForce.h"
#include "VolumeModel.h"
#include <list>

namespace FS
{
/** \brief Node for replaying a saved scenario from start to finish
 * or by stepping forward and backwards in time.

	To be able to do a replay a scenario has to be saved to file (by pressing 'Save and Exit'). The VolumeModel
	has to know which timeStampField.nrrd file to use.
	If isPlay is set then the scenario will be played from start to beginning (set in .x3d-file or with
	'Play/Pause' button).
	To step back and forward through scenario use 'RW' and 'FFW'. To do playback without restarting .x3d file
	the user can press 'Reset' and then start stepping from beginning of saved timestamps.

 */

class AVolumePlaybackNode: public H3D::X3DChildNode
{
public:

	/**
	 * Constructor
	 * \param _playbackTimeField The current timeindex when stepping through scenario. The image is showing the status at this specified time.
	 * \param volumeModel A pointer to the volume within which we bore plus its associated properties. These properties
     * include for example the segment to which each voxel belongs.
	 * \param isPlay If true then the complete scenario will be played from start to finish. _playbackTimeField is updated with elapsedTime calculated from system realtimeclock.
	 * \param _direction When set (-1 backward, 1 forward) the playback will go 0.1s in set direction.
	 * \param _directionFastPlayback When set (-1 backward, 1 forward) the playback will go 1.0s in set direction.
	 * \param _resePlaybackTimeField If set the time and image will be reset to 0 and user can start stepping through timestamps again.
	 */
	AVolumePlaybackNode(
			H3D::Inst< H3D::SFFloat > _direction = 0,
			H3D::Inst< H3D::SFFloat > _directionFastPlayback = 0,
			H3D::Inst< H3D::SFNode  > _volumeModel = 0,
			H3D::Inst< H3D::SFBool > _isPlay = 0,
			H3D::Inst< H3D::SFBool > _resePlaybackTimeField = 0,
			H3D::Inst< H3D::SFFloat > _playbackTimeField = 0);

	/**
	 * VolumePlayback Node initialization
	 */
	virtual void initialize();

	/**
	 * Traverse Scene Graph
	 *
	 * Updates the image according to current Playback-time.
	 */
	virtual void traverseSG( H3D::TraverseInfo &ti );

	/**
	 * Set endTime to last value in timestamp map.
	 */
	void setEndTime();

	/**
	 * Returns the endTime, i.e the last value in timestamp map.
	 */
	double getEndTime();

	auto_ptr< H3D::SFFloat > direction;
	auto_ptr< H3D::SFFloat > directionFastPlayback;
	auto_ptr< H3D::SFNode > volumeModel;
	auto_ptr< H3D::SFBool > isPlay;
	auto_ptr< H3D::SFBool > resetPlaybackTimeField;
	auto_ptr< H3D::SFFloat > playbackTimeField;

	/// The H3DNodeDatabase for this node.
	static H3D::H3DNodeDatabase database;

	VolumeModel *volume_model;

	///Elapsed time since last call to traverseSG.
	double elapsedTime;

	///Current Real world time
	double realTime;

	///Real world time when traverseSG was last called
	double oldRealTime;

	///The absolute start time of the playback.
	H3DUtil::TimeStamp startTime;

	///Time set on last run of traversSG().
	float oldTime;

	bool tempStop;

	///If traverseSG() has been run one time or not
	bool firstTime;





    // hack to degrade voxels
    struct removeHack {
        Vec3i p;
        float amountLeft;
        removeHack(Vec3i p):
            p(p),amountLeft(1.0){}
    };
    typedef std::list<removeHack> removelist;
    removelist slowremovelist;




	/**
	 * Exception class for VolumePlayback
	 */
	class VolumePlaybackException: public runtime_error
	{
	public:

		/**
		 * Constructor
		 */
		VolumePlaybackException(const string& what) :
			runtime_error("Exception within VolumePlayback: " + what)
			{
			}
	};

private:
	///Last value in timestamp map
	double endTime;
};
}




#endif	/* _AVolumePlaybackNode_H */

