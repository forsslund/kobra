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
#include "ADrillableNode.h"

using namespace FS;
using namespace std;

H3D::H3DNodeDatabase ADrillableNode::database( "ADrillableNode",
		&(newInstance<ADrillableNode>),
		typeid( ADrillableNode ),
		&X3DChildNode::database );

namespace ADrillableNodeInternals
{
H3D::FIELDDB_ELEMENT( ADrillableNode, position, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillableNode, appliedForce, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillableNode, cutRadius,  INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillableNode, drillSpeed,  INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillableNode, volumeModel, INPUT_OUTPUT);
H3D::FIELDDB_ELEMENT( ADrillableNode, materialSegmentationModel, INPUT_OUTPUT);

H3D::FIELDDB_ELEMENT( ADrillableNode, drillingStepsField, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillableNode, currentFPS, INPUT_OUTPUT ); //this should only be OUTPUT but doesn't seem to work...?
H3D::FIELDDB_ELEMENT( ADrillableNode, playbackTime, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillableNode, usesMarchingCubes, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillableNode, hasPedal, INPUT_OUTPUT );

H3D::FIELDDB_ELEMENT( ADrillableNode, onlyCutSegment, INPUT_OUTPUT );
}


ADrillableNode::ADrillableNode(H3D::Inst< H3D::SFVec3f > _position,
		H3D::Inst< H3D::SFVec3f > _appliedForce,
		H3D::Inst< H3D::SFFloat > _cutRadius,
		H3D::Inst< H3D::SFFloat > _drillSpeed,
		H3D::Inst< H3D::SFNode  > _volumeModel,
		H3D::Inst< H3D::SFNode  > _segmentModel,

		H3D::Inst< H3D::SFNode  > _drillingStepsField,
		H3D::Inst< H3D::SFFloat > _currentFPS,
        H3D::Inst< H3D::SFFloat > _playbackTime,
		H3D::Inst< H3D::SFBool  > _usesMarchingCubes,
		H3D::Inst< H3D::SFBool  > _hasPedal,

        H3D::Inst< H3D::SFInt32 > _onlyCutSegment
):
	position( _position ),
	appliedForce(_appliedForce),
	cutRadius( _cutRadius ),
	drillSpeed( _drillSpeed ),
	volumeModel( _volumeModel ),
	materialSegmentationModel( _segmentModel ),

	drillingStepsField(_drillingStepsField),
	currentFPS(_currentFPS),
    playbackTime(_playbackTime),
	usesMarchingCubes(_usesMarchingCubes),
	hasPedal(_hasPedal),

    onlyCutSegment(_onlyCutSegment)
	{

	type_name = "ADrillableNode";
	database.initFields( this );
	
	/* Default values */
	position->setValue(H3D::Vec3f(0, 0, 0));
	appliedForce->setValue(H3D::Vec3f(0,0,0));
	cutRadius->setValue(3);
	drillSpeed->setValue(1);
    playbackTime->setValue(-1.0);
	usesMarchingCubes->setValue(false);

	onlyCutSegment->setValue(0);
	}

void ADrillableNode::initialize()
{
	cout<<"Initializing Drillable Node"<<endl;

	volume_model = static_cast< VolumeModel * >( volumeModel->getValue() );
	segment_model = static_cast< MaterialSegmentationModel * >(materialSegmentationModel->getValue());

	if(!volume_model)
	{
		cout << "Null pointer exception in volume model!\n";
		return;
	}

	if (!segment_model) {
		cout << "Null pointer exception in segmentation model!\n";
		return;
	}

        frameCount = 0;
        usingPlayback = false;
        if (playbackTime->getValue() > 0.0)
        {
           usingPlayback = true;
           ptime = playbackTime->getValue();
        }
        else
        {
          ptime = H3D::TimeStamp();
        }
        prevTime = ptime;
        
        firstVoxel = true;
        hasPedal->setValue(false);

	X3DChildNode::initialize();
	
	cout<<"Initialized Drillable Node"<<endl;
}


void ADrillableNode::traverseSG( H3D::TraverseInfo &ti )
{
        double playbackTimestamp = playbackTime->getValue();
        double currentTime;
        if (playbackTimestamp > 0.0)
        {
            if (!usingPlayback)
            {
              ptime = playbackTimestamp;
              prevTime = ptime;
              usingPlayback = true;
              cout << "***** Using playback timestamps in ADrillableNode *****" << endl;
            }
            currentTime = playbackTimestamp;
        }
        else
        {   //Get current time
            currentTime = H3D::TimeStamp();
        }
        //For FPS calculation
        frameCount += 1;
        float deltaTime = float(currentTime - ptime);


        //Time since the last time a drill was made
        float diffTime = float(currentTime-prevTime);

        //If drilling is enabled
        if( drillSpeed->getValue() > 0)
        {
            if (usingPlayback || ti.hapticsEnabled())
            {            	
            	//Due to the low resolution in the segmented volume a minimum amount of drilling is set.
                //If the amount to be drilled is less, then the algorithm waits another turn.
                if (diffTime>0.01)
                {
                        cutVolume(ti.getAccInverseMatrix()*position->getValue(),drillSpeed->getValue()*diffTime);
                        if (usingPlayback) prevTime = playbackTimestamp;
                        else prevTime = H3D::TimeStamp();
                }
            }
        }

        else
        {
            if (usingPlayback) prevTime = playbackTimestamp;
            else prevTime = H3D::TimeStamp();
        }
        //Calculate current FPS. This is displayed in GUI.
        if (deltaTime > 1.0)
        {
            currentFPS->setValue(frameCount/deltaTime);
            if (usingPlayback) ptime = playbackTimestamp;
            else ptime = H3D::TimeStamp();
            frameCount=0;
        }
}


/// Cuts the volume
/**
 *  \param center the center of the volume to cut
 *	\param drillImpact drillspeed times the time it has been applied
 */
void ADrillableNode::cutVolume(H3D::Vec3f center, float drillImpact)
{
	volume_model = static_cast< VolumeModel * >( volumeModel->getValue() );
	segment_model = static_cast< MaterialSegmentationModel * >( materialSegmentationModel->getValue() );

	double elapsedTime;
	if (usingPlayback) elapsedTime = playbackTime->getValue();
	else elapsedTime = volume_model->getElapsedTime();

	if(!volume_model)
	{
		//TODO: Replace by throw
		cout << "Null pointer exception in volume model!\n";
		return;
	}

	if(!segment_model)
	{
		//TODO: Replace by throw
		cout << "Null pointer exception in segment model!\n";
		return;
	}

	float radius = cutRadius->getValue();
	

	Vec3i at;
	Vec3i max = volume_model->getIndexLocalWall(H3D::Vec3f(center.x+radius,center.y+radius,center.z+radius));
	Vec3i min = volume_model->getIndexLocalWall(H3D::Vec3f(center.x-radius,center.y-radius,center.z-radius));
		
	float maxSegVal = volume_model->getMaxSegVal();
	float maxImageVal = volume_model->getMaxImageVal();
	float maxSegScale = maxSegVal / maxImageVal;
		
	// update the area of the screen that should be updated and set updateScreenAreaPerVoxel
	// to fasle so that the area doesn't get updated every time a voxel value is set in the image
	volume_model->setScreenUpdateArea(min, max);
	bool updateScreenAreaPerVoxel = false;
	
	for (at.x=min.x;at.x<=max.x;at.x++)
	{
		for (at.y=min.y;at.y<=max.y;at.y++)
		{
			for (at.z=min.z;at.z<=max.z;at.z++)
			{
				H3D::Vec3f gPos = volume_model->getGlobalCoordinate(at);
				H3D::Vec3f dist = gPos - center;
								
				float val = volume_model->getMaterialRemaining(at);


                // If onlyCutSegment is active (!=0), only cut those segments
                if(int onlyCut = onlyCutSegment->getValue()){

                    // Here we would continue but there might be neighbouring voxels
                    // of other material that is not visible (due to the smooth rendering)
                    // but that the user will feel anyway. So if we are neightbour to
                    // the "only material" then remove, i.e. do NOT skip cutting it

                    // Find if no neighbour is onlyCut
                    bool neighbourFound=false;
                    Vec3i vm = volume_model->getMax();
                    for(int i=std::max(0,at.x-1);i<=std::min(vm.x,at.x+1);++i){
                        for(int j=std::max(0,at.y-1);j<=std::min(vm.y,at.y+1);++j){
                            for(int k=std::max(0,at.z-1);k<=std::min(vm.z,at.z+1);++k){
                                if(onlyCut == volume_model->getMaterialSegmentationId(Vec3i(i,j,k)))
                                    neighbourFound=true;
                            }
                        }
                    }
                    if(!neighbourFound)
                        continue;
                }

				
				// only do the calculations if the voxel is not air
                if (val > 0 )
				{	
					//voxel is within the drill radius
					if ((dist.x*dist.x+dist.y*dist.y+dist.z*dist.z) <= radius*radius)
					{																
						unsigned int segmentID = volume_model->getMaterialSegmentationId(at);








						//Set this boolean to true in the X3D file to make forssim render marching cubes drilling smoothly
						//This has not been set to true by default because it does not work with direct volume rendering
                        float proximity = 1.0f;
						if (usesMarchingCubes->getValue())
                            proximity = 1.0f - (dist.x*dist.x+dist.y*dist.y+dist.z*dist.z)/(radius*radius);

						//calculate force magnitude
						float forceMag = sqrt(pow(appliedForce->getValue().x,2)+pow(appliedForce->getValue().y,2)+pow(appliedForce->getValue().z,2));
						float forceRate = segment_model->getForceRate(segmentID);
						float newValue;

						// Air wrongly defined in this case. Remove voxel.
						if (segment_model->getHardness(segmentID) < 0.000) 
						{
							newValue = 0.0;
						}
						else if (forceRate < 0.0001)
						{
							newValue = val - drillImpact * segment_model->getHardness(segmentID) * proximity;
						}
						else
						{
							float forcePerc = forceMag / forceRate;
							newValue = val - drillImpact * segment_model->getHardness(segmentID) * forcePerc * proximity;
						}
						
						// check if out of range
						if (newValue < 0) newValue = 0.0;
						if (newValue > 1) newValue = 1.0;
								
						float mcVal = 0.0;
						if (usesMarchingCubes->getValue())
						{
							if (newValue > 0)
							{
								MaterialSegmentationModel::RangeStruct segmentRange;
								segment_model->getSegmentRange((int)segmentID, segmentRange);
                                float range = segmentRange.max - segmentRange.min;
								
								mcVal = range * newValue + segmentRange.min;
								
								int mcValImage;
								float mcValRounded;
								
								if (fabs(mcVal - segmentRange.min) * maxSegVal < 0.5) 
								{
									newValue = 0.0;
									mcVal = 0;
								} else 
								{
									// The values we deal with are normalized so that the maximum value present 
									// in the data is 1. However, when it is saved in the image, it should be
									// converted so that 1 represents the maximum value allowed by the data type/
									mcVal = mcVal * maxSegScale;
									
									// Becuse the image is not saved as a float, we lose some precision due to 
									// rounding. Therefore, the material remaining and the value in image data 
									// will not match. To overcome this, we calculate the actual value set in 
									// the image and set the material remaining (newValue) accordingly. We use
									// int in the rounding to take care of all possible image data types.
									mcValImage = (int) (mcVal * maxImageVal);
									mcValRounded = ((float)mcValImage) / maxSegVal;
									newValue = (mcValRounded - segmentRange.min) / range;
									
									if (fabs(newValue) <= 0.0001) 
									{
										mcVal = 0.0;
										newValue = 0.0;	
									} 
								}
							} 		
						}	
						
					
						if (newValue == 0.0) // voxel fully drilled
						{
							if (firstVoxel)
							{ //Start the timer
								volume_model->resetStartTime();
								firstVoxel = false;
								//TODO: Will this mean that elapsedTime will be 0 for first removed voxel?
								//TODO: Is that ok or maybe resetStartTime should do startTime = currentTime-1?
																	if (usingPlayback) elapsedTime = playbackTime->getValue();
																	else elapsedTime = volume_model->getElapsedTime();
							}
							volume_model->setMaterialRemaining(at,0.0);
							volume_model->setImageValue(at,0,0,0,updateScreenAreaPerVoxel);
							volume_model->setTimeStamp(at,elapsedTime);
							volume_model->incrementNoOfVoxelsBoredByUserForForbiddenSegment(at);
							volume_model->incrementNoOfVoxelsBoredByUserForSegment(at);
                            volume_model->incrementNoOfVoxelsBoredByUserForExpertSegment(at);
                            //drillingStepsNode->setAsBored(at,elapsedTime);
                            //volume_model->updateDrilledHilightVoxels(at);

						} else // voxel partially drilled
						{
							volume_model->setMaterialRemaining(at,newValue);
							
							if (usesMarchingCubes->getValue())
								volume_model->setImageValue(at,mcVal,0,0,updateScreenAreaPerVoxel);
						}
					} 
				}
			}
		}
	}
	
}
