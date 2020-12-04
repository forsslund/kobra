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
/// \file OculusRiftSensor.h
/// \brief Header file for OculusRiftSensor.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3D_OCULUSRIFTSENSOR_H__
#define __H3D_OCULUSRIFTSENSOR_H__


#include <H3D/X3DSensorNode.h>
#include <list>
#include <H3D/SFVec3f.h>
#include <H3D/SFFloat.h>
#include <H3D/SFInt32.h>
#include <H3D/SFRotation.h>
#include <H3DUtil/Threads.h>
#include <H3D/OculusRiftHandler.h>
#include <H3D/SFVec2f.h>

namespace H3D {

  /// \ingroup H3DNodes
  /// \class OculusRiftSensor
  /// \brief This is a X3DSensorNode for reading values from a Oculus Rift HMD.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../../H3DAPI/examples/All/OculusRiftSensor.x3d">OculusRiftSensor.x3d</a>
  ///     ( <a href="examples/OculusRiftSensor.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile OculusRiftSensor.dot
  class H3DAPI_API OculusRiftSensor : public X3DSensorNode {
  public:

  
    /// This class is an AutoUpdate class that sets the field
    /// accumulatedRotation to its input value and also sets
    /// accumulatedYaw, accumulatedPitch and accumulatedRoll to
    /// a value dependent on input value
    ///
    class H3DAPI_API RecenterTrackingField: 
      public AutoUpdate< OnNewValueSField< SFBool > > {
      
    protected:
      virtual void onNewValue(const bool &new_value);
    };

    class H3DAPI_API MSAAEnabledField :
      public AutoUpdate< OnNewValueSField< SFBool > > {

    protected:
      virtual void onNewValue(const bool &new_value);
    };

    class H3DAPI_API PerformanceStatsField :
      public AutoUpdate< OnNewValueSField< SFString > > {

    protected:
      virtual void onNewValue(const string &new_value);
    };


   
  
    /// Constructor.
    OculusRiftSensor( 
                    Inst< SFBool                > _enabled  = 0,
                    Inst< SFNode                > _metadata = 0,
                    Inst< SFBool                > _isActive = 0,
                    Inst< SFVec3f               > _headPosition = 0,
                    Inst< SFRotation            > _headOrientation = 0,
                    Inst< PerformanceStatsField > _performanceStats = 0,
                    Inst< RecenterTrackingField > _recenterTracking = 0,
                    Inst< MSAAEnabledField      > _MSAAEnabled = 0,
                    Inst< SFVec3f               > _leftHandPosition = 0,
                    Inst< SFRotation            > _leftHandOrientation = 0,
                    Inst< SFFloat               > _leftHandTrigger = 0,
                    Inst< SFFloat               > _leftHandIndexTrigger = 0,
                    Inst< SFVec2f               > _leftHandThumbstick = 0,
                    Inst< SFFloat               > _leftHandVibrationFrequency = 0,
                    Inst< SFFloat               > _leftHandVibrationAmplitude = 0,
                    Inst< SFVec3f               > _rightHandPosition = 0,
                    Inst< SFRotation            > _rightHandOrientation = 0,
                    Inst< SFFloat               > _rightHandTrigger = 0,
                    Inst< SFFloat               > _rightHandIndexTrigger = 0,
                    Inst< SFVec2f               > _rightHandThumbstick = 0,
                    Inst< SFFloat               > _rightHandVibrationFrequency = 0,
                    Inst< SFFloat               > _rightHandVibrationAmplitude = 0,
                    Inst< SFInt32               > _buttons = 0,
                    Inst< SFInt32               > _touches = 0
                    );

    ~OculusRiftSensor() {
    }

    /// Contains the current head translation as reported by the device.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_headPosition.dot
    auto_ptr< SFVec3f> headPosition;

    /// Contains the current head orientation around the x-axis as reported 
    /// by the device.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_headOrientation.dot
    auto_ptr< SFRotation> headOrientation;

    /// Makes it possible to show performance stats of the oculus.
    ///
    /// <b>Access type: </b> inputOutput \n
    /// <b>Default value: </b> "OFF" \n
    /// <b>Valid values: </b> "OFF" "PERF_SUMMARY" "LATENCY_TIMING"
    ///                       "APP_RENDER_TIMING" "COMP_RENDER_TIMING"
    ///                       "VERSION_INFO" \n
    /// 
    /// \dotfile OculusRiftSensor_performanceStats.dot
    auto_ptr< PerformanceStatsField> performanceStats;


    /// Recenter the tracking for oculus.
    ///
    /// <b>Access type: </b> inputOutput \n
    /// 
    /// \dotfile OculusRiftSensor_recenterTracking.dot
    auto_ptr< RecenterTrackingField> recenterTracking;

    /// Controls multi sample anti aliasing on the oculus.
    ///
    /// <b>Access type: </b> inputOutput \n
    /// <b>Default value: </b> true \n
    /// 
    /// \dotfile OculusRiftSensor_MSAAEnabled.dot
    auto_ptr< MSAAEnabledField > MSAAEnabled;

    /// The position of the left touch controller.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_leftHandPosition.dot
    auto_ptr< SFVec3f> leftHandPosition;

    /// The orientation of the left touch controller.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_leftHandOrientation.dot
    auto_ptr< SFRotation> leftHandOrientation;

    /// The value for the left hand trigger.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_leftHandTrigger.dot
    auto_ptr< SFFloat               > leftHandTrigger;

    /// The value for the left hand index trigger.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_leftHandIndexTrigger.dot
    auto_ptr< SFFloat               > leftHandIndexTrigger;

    /// The value for the left hand thumb stick.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_leftHandThumbstick.dot
    auto_ptr< SFVec2f               > leftHandThumbstick;

    /// Used to control the vibration frequency of the left touch controller.
    ///
    /// <b>Access type: </b> inputOutput \n
    /// <b>Default value: </b> 0 \n
    /// 
    /// \dotfile OculusRiftSensor_leftHandVibrationFrequency.dot
    auto_ptr< SFFloat               > leftHandVibrationFrequency;

    /// Used to control the vibration amplitude of the left touch controller.
    ///
    /// <b>Access type: </b> inputOutput \n
    /// <b>Default value: </b> 0 \n
    /// 
    /// \dotfile OculusRiftSensor_leftHandVibrationFrequency.dot
    auto_ptr< SFFloat               > leftHandVibrationAmplitude;

    /// The position of the right touch controller.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_rightHandPosition.dot
    auto_ptr< SFVec3f> rightHandPosition;

    /// The orientation of the right touch controller.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_rightHandOrientation.dot
    auto_ptr< SFRotation> rightHandOrientation;

    /// The value for the right hand trigger.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_rightHandTrigger.dot
    auto_ptr< SFFloat               > rightHandTrigger;

    /// The value for the right hand index trigger.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_rightHandIndexTrigger.dot
    auto_ptr< SFFloat               > rightHandIndexTrigger;

    /// The value for the right hand thumb stick.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_rightHandThumbstick.dot
    auto_ptr< SFVec2f               > rightHandThumbstick;

    /// Used to control the vibration frequency of the right touch controller.
    ///
    /// <b>Access type: </b> inputOutput \n
    /// <b>Default value: </b> 0 \n
    /// 
    /// \dotfile OculusRiftSensor_rightHandVibrationFrequency.dot
    auto_ptr< SFFloat               > rightHandVibrationFrequency;

    /// Used to control the vibration amplitude of the right touch controller.
    ///
    /// <b>Access type: </b> inputOutput \n
    /// <b>Default value: </b> 0 \n
    /// 
    /// \dotfile OculusRiftSensor_rightHandVibrationFrequency.dot
    auto_ptr< SFFloat               > rightHandVibrationAmplitude;

    /// Indicates which buttons are pressed.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_buttons.dot
    auto_ptr< SFInt32               > buttons;

    /// Setting some touched output state.
    ///
    /// <b>Access type: </b> outputOnly \n
    /// 
    /// \dotfile OculusRiftSensor_touches.dot
    auto_ptr< SFInt32 > touches;

  public:

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

#ifdef HAVE_LIBOVR
    static OculusRiftHandler *oculus;
#endif

  private:

    /// Transfers the values from the device communication thread to
    /// the scenegraph thread.
    void updateValues();
    
    struct H3DAPI_API Update
      : AutoUpdate<Field> {
      void update(){
        static_cast<OculusRiftSensor*>
          (owner)->updateValues();
      }
    };
    
    auto_ptr< Update > update;
  };
}

#endif
