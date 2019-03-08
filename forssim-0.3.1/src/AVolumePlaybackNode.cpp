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
#include "AVolumePlaybackNode.h"

using namespace FS;

// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase AVolumePlaybackNode::database( "AVolumePlaybackNode",
		&(newInstance<AVolumePlaybackNode>),
		typeid( AVolumePlaybackNode ),
		&X3DChildNode::database );

namespace AVolumePlaybackNodeInternals
{
H3D::FIELDDB_ELEMENT( AVolumePlaybackNode, direction,  INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( AVolumePlaybackNode, directionFastPlayback,  INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( AVolumePlaybackNode, volumeModel, INPUT_OUTPUT);
H3D::FIELDDB_ELEMENT( AVolumePlaybackNode, isPlay,  INPUT_OUTPUT  );
H3D::FIELDDB_ELEMENT( AVolumePlaybackNode, resetPlaybackTimeField, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( AVolumePlaybackNode, playbackTimeField, INPUT_OUTPUT );
}

AVolumePlaybackNode::AVolumePlaybackNode( H3D::Inst< H3D::SFFloat > _direction,
		H3D::Inst< H3D::SFFloat > _directionFastPlayback,
		H3D::Inst< H3D::SFNode  > _volumeModel,
		H3D::Inst < H3D::SFBool > _isPlay,
		H3D::Inst< H3D::SFBool >_resetPlaybackTimeField,
		H3D::Inst< H3D::SFFloat >_playbackTimeField):
			direction( _direction ),
			directionFastPlayback( _directionFastPlayback),
			volumeModel( _volumeModel ),
			isPlay( _isPlay),
			resetPlaybackTimeField(_resetPlaybackTimeField),
			playbackTimeField( _playbackTimeField){

	type_name = "AVolumePlaybackNode";
	database.initFields( this );

	/* Default values */
	direction->setValue(0);
	directionFastPlayback->setValue(0);
	isPlay->setValue(false);
	tempStop = false;
	oldTime = 0;
	resetPlaybackTimeField->setValue(false);
	playbackTimeField->setValue(0);
}


void AVolumePlaybackNode::initialize()
{

	cout<<"Initializing Volume Playback Node"<<endl;

	volume_model = static_cast< VolumeModel * >( volumeModel->getValue() );
	if(!volume_model)
	{
		//TODO:Replace by throw
		cout << "Null pointer exception in volume model!\n";
		return;
	}

	elapsedTime=0.0;
	H3DUtil::TimeStamp ts;
	startTime = ts;
	realTime = 0.0;
	oldRealTime = 0.0;
	endTime = 0;
	firstTime = true;

	X3DChildNode::initialize();
	cout<<"Initialized Volume Playback Node"<<endl;
}


void AVolumePlaybackNode::traverseSG( H3D::TraverseInfo& )
{
	/*Not helping after removal of forsApp, cannot be sure timestamp_map has been initialized yet
	 * even if traverseSG() has started...:(
	 * Instead also check everytime endTime() is called.
	 */
	if (firstTime)
	{
		setEndTime();
		firstTime = false;
	}

	//The multimap timestamp_map is a map containing all voxels removed sorted by time.
	//This means we can update only the voxels that has changed since last time by getting
	//a subset from the map by using playbackTimeField as key.

	realTime = H3D::TimeStamp();
	elapsedTime = realTime-oldRealTime;

	//By pressing 'Reset' the user resets the playback time to 0 and can then step from the beginning again.
	if (resetPlaybackTimeField->getValue())
	{
		//Reset Image to original values and reset startTime.
		multimap<double, Vec3i>::iterator it;
		for ( it=volume_model->timestamp_map.begin() ; it != volume_model->timestamp_map.end(); it++ )
		{
			volume_model->setImageValue((*it).second,volume_model->getOriginalImageValue((*it).second),0,0);
			volume_model->setMaterialRemaining((*it).second,1.0);
		}
		setEndTime();
		playbackTimeField->setValue(0.0);
		oldTime = 0.0;
		resetPlaybackTimeField->setValue(false);
		isPlay->setValue(false);
	}

	Vec3i at;
	//This was needed so user can wind time even during play. The play will stop and then after rewind or forward
	//(change of playback-time) be started again (look for tempStop).
	if (isPlay->getValue())
	{
		if ((direction->getValue() != 0) || (directionFastPlayback->getValue() != 0))
		{
			tempStop = true;
		}
	}

	//Play the complete saved scenario.
	if (isPlay->getValue() && tempStop!=true)
	{
		if (playbackTimeField->getValue() < getEndTime())
		{
            playbackTimeField->setValue(playbackTimeField->getValue()+float(elapsedTime));
			multimap<double, Vec3i>::iterator start, stop, it;
			start = volume_model->timestamp_map.lower_bound(oldTime);
			stop = volume_model->timestamp_map.upper_bound (playbackTimeField->getValue());

			for ( it=start ; it != stop; it++ )
			{
				//Set all voxels that are "removed" at this time to 0.

                //volume_model->setImageValue((*it).second,0.0,0.0,0.0);
                //volume_model->setMaterialRemaining((*it).second,0.0);
                slowremovelist.push_back((*it).second);
			}

			//Update image.
			oldTime = playbackTimeField->getValue();
		}
		else
		{;
			isPlay->setValue(false);
		}
	}           
	//Only calculate a new image if user have moved forward or backward in time.
	else
	{
		if ((direction->getValue() != 0) || (directionFastPlayback->getValue() != 0))
		{
			//Set if playback-time should be changed fast or slow
			float timeStep = 0;
			if (direction->getValue() != 0)
			{
                timeStep = 0.1f;
			}
			else if (directionFastPlayback->getValue() != 0)
			{
                timeStep = 1.0f;
			}

			multimap<double, Vec3i>::iterator start, stop, it;
			if ((direction->getValue() == -1) || (directionFastPlayback->getValue() == -1))
			{
				if (playbackTimeField->getValue() > 0)
				{
					//Backwards in time. "Refill" drilled voxels from original image.
					playbackTimeField->setValue(playbackTimeField->getValue() - timeStep);

					if (playbackTimeField->getValue() < 0)
					{//So counter is displaying 0, and only one more voxel update will be done.
						playbackTimeField->setValue(0);
					}
					start = volume_model->timestamp_map.lower_bound(playbackTimeField->getValue());
					stop = volume_model->timestamp_map.upper_bound (oldTime);
					for ( it=start ; it != stop; it++ )
					{
						volume_model->setImageValue((*it).second,volume_model->getOriginalImageValue((*it).second),0,0);
						volume_model->setMaterialRemaining((*it).second,1.0);
					}
				}
			}
			if ((direction->getValue() == 1) || (directionFastPlayback->getValue() == 1))
			{
				if (playbackTimeField->getValue() < getEndTime())
				{
					//Forward in time. Remove drilled voxels by setting them to 0.
					playbackTimeField->setValue(playbackTimeField->getValue() + timeStep);

					start = volume_model->timestamp_map.lower_bound(0);
					stop = volume_model->timestamp_map.upper_bound (playbackTimeField->getValue());
					for ( it=start ; it != stop; it++ )
					{
						volume_model->setImageValue((*it).second,0.0,0.0,0.0);
						volume_model->setMaterialRemaining((*it).second,0.0);
					}
				}
			}
			//Update image.
			oldTime = playbackTimeField->getValue();
			direction->setValue(0);
			directionFastPlayback->setValue(0);

		}
		else
		{
			//NOP User has not moved forward or backward in time.
		}
	}





    // Do the actual degradation of voxels
    for(removelist::iterator it = slowremovelist.begin();it!=slowremovelist.end();it++){

        removeHack& r = *it;

        //if(r.amountLeft < volume_model->getMCRangeForVoxelAt(r.p).min){
        if(r.amountLeft < 0){
            slowremovelist.erase(it++);
            volume_model->setImageValue(r.p,r.amountLeft,0.0,0.0);
            volume_model->setMaterialRemaining(r.p,0.0);
            continue;
        }

        volume_model->setImageValue(r.p,r.amountLeft,0.0,0.0);
        r.amountLeft -= 0.1f;

    }

















	if (tempStop)
	{
		tempStop = false;
	}

	oldRealTime = realTime;
}

void AVolumePlaybackNode::setEndTime()
{
	multimap<double, Vec3i>::iterator it;
	if (volume_model->timestamp_map.size() > 0)
	{
		it = volume_model->timestamp_map.end();
		it--;
		endTime = (*it).first;
	}
	else
	{
		endTime = 0;
	}
}

double AVolumePlaybackNode::getEndTime()
{
	/*It seems that after the removal of forsApp, even when traverseSG() has started,
	 * we cannot be sure that the VolumeModel has been initiated and timestamp_map has
	 * been initialized...
	 */
	if (endTime == 0)
	{
		setEndTime();
	}
	return endTime;
}
