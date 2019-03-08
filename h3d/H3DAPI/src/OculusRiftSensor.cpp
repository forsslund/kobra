//////////////////////////////////////////////////////////////////////////////
//    Copyright 2016-2019, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file OculusRiftSensor.cpp
/// \brief CPP file for OculusRiftSensor.
///
//
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/OculusRiftSensor.h>

#include <H3D/Scene.h>

using namespace H3D;
#ifdef HAVE_LIBOVR
OculusRiftHandler * OculusRiftSensor::oculus = NULL;
#endif

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase OculusRiftSensor::database( 
                                          "OculusRiftSensor", 
                                          &(newInstance<OculusRiftSensor>),
                                          typeid( OculusRiftSensor ),
                                          &X3DSensorNode::database
                                          );

namespace OculusRiftSensorInternal {
  FIELDDB_ELEMENT( OculusRiftSensor, headOrientation, OUTPUT_ONLY )
  FIELDDB_ELEMENT( OculusRiftSensor, headPosition, OUTPUT_ONLY )
  FIELDDB_ELEMENT( OculusRiftSensor, performanceStats, INPUT_OUTPUT )
  FIELDDB_ELEMENT( OculusRiftSensor, recenterTracking, INPUT_OUTPUT )
  FIELDDB_ELEMENT( OculusRiftSensor, MSAAEnabled, INPUT_OUTPUT )
  FIELDDB_ELEMENT(OculusRiftSensor,  leftHandOrientation, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  leftHandPosition, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  leftHandTrigger, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  leftHandIndexTrigger, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  leftHandThumbstick, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  leftHandVibrationFrequency, INPUT_OUTPUT)
  FIELDDB_ELEMENT(OculusRiftSensor,  leftHandVibrationAmplitude, INPUT_OUTPUT)
  FIELDDB_ELEMENT(OculusRiftSensor,  rightHandOrientation, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  rightHandPosition, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  rightHandTrigger, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  rightHandIndexTrigger, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  rightHandThumbstick, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  rightHandVibrationFrequency, INPUT_OUTPUT)
  FIELDDB_ELEMENT(OculusRiftSensor,  rightHandVibrationAmplitude, INPUT_OUTPUT)

  FIELDDB_ELEMENT(OculusRiftSensor,  buttons, OUTPUT_ONLY)
  FIELDDB_ELEMENT(OculusRiftSensor,  touches, OUTPUT_ONLY)
}



OculusRiftSensor::OculusRiftSensor( 
              Inst< SFBool                > _enabled,
              Inst< SFNode                > _metadata,
              Inst< SFBool                > _isActive,
              Inst< SFVec3f               > _headPosition,
              Inst< SFRotation            > _headOrientation,
              Inst< PerformanceStatsField > _performanceStats,
              Inst< RecenterTrackingField > _recenterTracking,
              Inst< MSAAEnabledField      > _MSAAEnabled,
              Inst< SFVec3f               > _leftHandPosition,
              Inst< SFRotation            > _leftHandOrientation,
              Inst< SFFloat               > _leftHandTrigger,
              Inst< SFFloat               > _leftHandIndexTrigger,
              Inst< SFVec2f               > _leftHandThumbstick,
              Inst< SFFloat               > _leftHandVibrationFrequency,
              Inst< SFFloat               > _leftHandVibrationAmplitude,
              Inst< SFVec3f               > _rightHandPosition,
              Inst< SFRotation            > _rightHandOrientation,
              Inst< SFFloat               > _rightHandTrigger,
              Inst< SFFloat               > _rightHandIndexTrigger,
              Inst< SFVec2f               > _rightHandThumbstick,
              Inst< SFFloat               > _rightHandVibrationFrequency,
              Inst< SFFloat               > _rightHandVibrationAmplitude,
              Inst< SFInt32               > _buttons,
              Inst< SFInt32               > _touches  ):
  X3DSensorNode( _enabled, _metadata, _isActive ),
  headPosition( _headPosition ),
  headOrientation( _headOrientation ),
  performanceStats( _performanceStats ),
  recenterTracking( _recenterTracking ),
  MSAAEnabled( _MSAAEnabled ),
  leftHandPosition( _leftHandPosition ),
  leftHandOrientation( _leftHandOrientation ),
  leftHandTrigger( _leftHandTrigger ),
  leftHandIndexTrigger( _leftHandIndexTrigger ),
  leftHandThumbstick(_leftHandThumbstick),
  leftHandVibrationFrequency( _leftHandVibrationFrequency ),
  leftHandVibrationAmplitude( _leftHandVibrationAmplitude ),
  rightHandPosition( _rightHandPosition ),
  rightHandOrientation( _rightHandOrientation ),
  rightHandTrigger( _rightHandTrigger ),
  rightHandIndexTrigger( _rightHandIndexTrigger ),
  rightHandThumbstick(_rightHandThumbstick),
  rightHandVibrationFrequency( _rightHandVibrationFrequency ),
  rightHandVibrationAmplitude( _rightHandVibrationAmplitude ),
  buttons( _buttons ),
  touches( _touches ),
  update( new Update ) {

  type_name = "OculusRiftSensor";

  database.initFields( this );
  
  update->setOwner(this);
  Scene::time->routeNoEvent(update);
  
  MSAAEnabled->setValue(true, id);
  recenterTracking->setValue(false, id);

  performanceStats->addValidValue("OFF");
  performanceStats->addValidValue("PERF_SUMMARY");
  performanceStats->addValidValue("LATENCY_TIMING");
  performanceStats->addValidValue("APP_RENDER_TIMING");
  performanceStats->addValidValue("COMP_RENDER_TIMING");
  performanceStats->addValidValue("VERSION_INFO");

  performanceStats->setValue("OFF", id);

  leftHandVibrationFrequency->setValue( 0, id );
  leftHandVibrationAmplitude->setValue( 0, id );

  rightHandVibrationFrequency->setValue( 0, id );
  rightHandVibrationAmplitude->setValue( 0, id );
}

void OculusRiftSensor::updateValues(){
#ifdef HAVE_LIBOVR

  bool active = oculus && oculus->isInitialized();

  if( isActive->getValue() != active )
    isActive->setValue( active, id );



  if( active && enabled->getValue() ) {
    const ovrTrackingState &tracking_state = oculus->getTrackingState();
    const ovrEyeRenderDesc &left_eye_desc = oculus->getLeftEyeRenderDesc();
    const ovrEyeRenderDesc &right_eye_desc = oculus->getRightEyeRenderDesc();
    const ovrHmdDesc &hmd_desc = oculus->getHMDDesc();
    const ovrInputState &input_state = oculus->getInputState();
    
    // poses
    headPosition->setValue(Vec3f(tracking_state.HeadPose.ThePose.Position.x,
                                 tracking_state.HeadPose.ThePose.Position.y,
                                 tracking_state.HeadPose.ThePose.Position.z), id);

    headOrientation->setValue(Rotation( Quaternion( tracking_state.HeadPose.ThePose.Orientation.x,
                                                    tracking_state.HeadPose.ThePose.Orientation.y,
                                                    tracking_state.HeadPose.ThePose.Orientation.z,
                                                    tracking_state.HeadPose.ThePose.Orientation.w ) ), id );

    leftHandPosition->setValue(Vec3f(tracking_state.HandPoses[0].ThePose.Position.x,
                                     tracking_state.HandPoses[0].ThePose.Position.y, 
                                     tracking_state.HandPoses[0].ThePose.Position.z), id);

    rightHandPosition->setValue(Vec3f(tracking_state.HandPoses[1].ThePose.Position.x,
                                      tracking_state.HandPoses[1].ThePose.Position.y, 
                                      tracking_state.HandPoses[1].ThePose.Position.z), id);

    leftHandOrientation->setValue(Rotation(Quaternion(tracking_state.HandPoses[0].ThePose.Orientation.x,
                                                      tracking_state.HandPoses[0].ThePose.Orientation.y,
                                                      tracking_state.HandPoses[0].ThePose.Orientation.z,
                                                      tracking_state.HandPoses[0].ThePose.Orientation.w)), id);



    rightHandOrientation->setValue(Rotation(Quaternion(tracking_state.HandPoses[1].ThePose.Orientation.x,
                                                       tracking_state.HandPoses[1].ThePose.Orientation.y,
                                                       tracking_state.HandPoses[1].ThePose.Orientation.z,
                                                       tracking_state.HandPoses[1].ThePose.Orientation.w)), id);

    
    // buttons and triggers
    buttons->setValue( input_state.Buttons, id );
    touches->setValue( input_state.Touches, id );
    leftHandTrigger->setValue( input_state.HandTrigger[0], id );
    rightHandTrigger->setValue( input_state.HandTrigger[1], id );
    leftHandIndexTrigger->setValue( input_state.IndexTrigger[0], id );
    rightHandIndexTrigger->setValue( input_state.IndexTrigger[1], id );
    leftHandThumbstick->setValue( Vec2f( input_state.Thumbstick[0].x,
                                         input_state.Thumbstick[0].y ), id );
    rightHandThumbstick->setValue( Vec2f( input_state.Thumbstick[1].x,
                                          input_state.Thumbstick[1].y ), id );

    // vibration
    oculus->setControllerVibration( ovrControllerType_LTouch, leftHandVibrationFrequency->getValue(), leftHandVibrationAmplitude->getValue() );
    oculus->setControllerVibration( ovrControllerType_RTouch, rightHandVibrationFrequency->getValue(), rightHandVibrationAmplitude->getValue() );
 

  }
#endif
}

void OculusRiftSensor::PerformanceStatsField::onNewValue(const string &new_value) {
#ifdef HAVE_LIBOVR
  if (OculusRiftSensor::oculus) {
    if (new_value == "PERF_SUMMARY") OculusRiftSensor::oculus->showPerfStats(ovrPerfHud_PerfSummary);
    else if (new_value == "LATENCY_TIMING") OculusRiftSensor::oculus->showPerfStats(ovrPerfHud_LatencyTiming);
    else if (new_value == "APP_RENDER_TIMING") OculusRiftSensor::oculus->showPerfStats(ovrPerfHud_AppRenderTiming);
    else if (new_value == "COMP_RENDER_TIMING") OculusRiftSensor::oculus->showPerfStats(ovrPerfHud_CompRenderTiming);
    else if (new_value == "VERSION_INFO") OculusRiftSensor::oculus->showPerfStats(ovrPerfHud_VersionInfo);
    else {
      if (new_value != "OFF") {
        Console(LogLevel::Error) << "Warning: Invalid performanceStats value: \"" << new_value
          << "\". Using \"OFF\" instead(in OculusRiftSensor node). " << endl;
      }
      OculusRiftSensor::oculus->showPerfStats(ovrPerfHud_Off);
    }
  }
#endif
}

void OculusRiftSensor::RecenterTrackingField::onNewValue(const bool &new_value) {
#ifdef HAVE_LIBOVR  
  if (new_value && OculusRiftSensor::oculus) oculus->recenterTracking();
#endif
}
void OculusRiftSensor::MSAAEnabledField::onNewValue(const bool &new_value) {
#ifdef HAVE_LIBOVR
  if (OculusRiftSensor::oculus) oculus->setMSAA( new_value );
#endif
}
