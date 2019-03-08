//////////////////////////////////////////////////////////////////////////////
//    Copyright 2016-2019, SenseGraphics AB (modifications)
//    Copyright(c) 2016 Krzysztof Kondrak (original code)
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
/// \file OculusRiftHandler.h
/// \brief Header file for the Oculus Rift support code
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3D_OCULUSRIFTHANDLER_H__
#define __H3D_OCULUSRIFTHANDLER_H__

#include <H3D/Sphere.h>

#ifdef HAVE_LIBOVR
#include <OVR_CAPI_GL.h>
#include <Extras/OVR_Math.h>

/// The OculusRiftHandler class provides support for the Oculus Rift VR
/// device. It provides convenience functions to initialize and render
/// OpenGL content to the OculusRift display. It is based on the MinimumOpenGL
/// code by Krzysztof Kondrak that can be found at
/// https://github.com/kondrak/oculusvr_samples.
/// 
/// Current SDK version supported is 1.3. 
///
/// To integrate with your solution
class H3DAPI_API OculusRiftHandler {
public:
  /// Constructor.
  OculusRiftHandler();
  /// Destructor.
  ~OculusRiftHandler();

  /// \returns true if Oculus has been successfully initialized with initVR, false otherwise. 
  inline bool isInitialized() { return m_isInitialized; }

  /// Initializes the connection to the Oculus Rift.
  /// \returns true if successful, false if failed.
  bool initVR();

  /// Initializes the OpenGL buffers to use for rendering.
  /// \param window_width The width of the buffer to use (in pixels).
  /// \param window_height The height of the buffer to use (in pixels).
  /// \returns true if successful, false if failed.
  bool initVRBuffers(int window_width, int window_height);

  /// Should be called in the beginning of each frame. Saves the tracker
  /// data from the HMD to be used in the current frame.
  void  onRenderStart();

  /// Should be called just before rendering the specified eye.  
  /// Matrices set in the ooutput parameters should be used in the modelview and projection matrix
  /// for the render.
  /// \param eye_index The index of the eye (normally 0 for left and 1 for right)
  /// \param view_matrix Output for the view matrix to use for the eye.
  /// \param projection_matrix Output for the projection matrix to use for the eye.
  void onEyeRender(int eye_index, H3D::Matrix4f &view_matrix, H3D::Matrix4f &projection_matrix );

  /// Should be called when rendering for the specified eye is done.
  /// \param eye_index The index of the eye (normally 0 for left and 1 for right)
  void  onEyeRenderFinish(int eye_index);

  /// Should be called when all rendering is done to submit the current frame
  /// to the HMD display.
  void  submitFrame();

  /// Should be called after submitFrame to transfer the data to the back 
  /// buffer (this is to update the view in the normal render window)
  void  blitMirror(ovrEyeType num_eyes = ovrEye_Count, int offset = 0);   // regular OculusRiftHandler mirror view

  /// Cleanup everything. 
  void destroyVR();

  /// Recenter the HMD tracking so that the current view direction is center,
  void recenterTracking();

  void setControllerVibration( ovrControllerType type, float frequency, float amplitude );

  void  showPerfStats(ovrPerfHudMode statsMode);

  /// Gets the resolution of the HMD display. 
  const ovrSizei getResolution() const;

  inline const ovrHmdDesc &getHMDDesc() { return m_hmdDesc; }
  inline const ovrEyeRenderDesc &getLeftEyeRenderDesc() { return m_eyeRenderDesc[0]; }
  inline const ovrEyeRenderDesc &getRightEyeRenderDesc() { return m_eyeRenderDesc[1]; }
  inline const ovrTrackingState  &getTrackingState() { return m_trackingState; }
  inline double getFrameTiming() { return m_frameTiming; }
  inline const ovrInputState &getInputState() { return m_inputState; }


  /// Gets the view-projection matrix for an eye.
  H3D::Matrix4f getEyeMVPMatrix(int eye_index) const;

  /// Toggle multi-sample anti-aliasing.
  inline void  setMSAA(bool val) { m_msaaEnabled = val; }

  /// Get current status of multi-sample anti-aliasing.
  /// \returns true if enabled, false if disabled.
  inline bool  MSAAEnabled() const { return m_msaaEnabled; }

private:

    // A buffer struct used to store eye textures and framebuffers.
    // We create one instance for the left eye, one for the right eye.
    // Final rendering is done via blitting two separate frame buffers into one render target.
    struct OVRBuffer
    {  
        OVRBuffer(const ovrSession &session, int eyeIdx);
        void onRender();
        void onRenderFinish();
        void setupMSAA(); 
        void onRenderMSAA();
        void onRenderMSAAFinish();
        void destroy(const ovrSession &session);

        ovrSizei   m_eyeTextureSize;
        GLuint     m_eyeFbo      = 0;
        GLuint     m_eyeTexId    = 0;
        GLuint     m_depthBuffer = 0;

        GLuint m_msaaEyeFbo   = 0;   // framebuffer for MSAA texture
        GLuint m_eyeTexMSAA   = 0;   // color texture for MSAA
        GLuint m_depthTexMSAA = 0;   // depth texture for MSAA

        ovrTextureSwapChain m_swapTextureChain = nullptr;
    };

    // data and buffers used to render to HMD
    ovrSession        m_hmdSession;
    ovrHmdDesc        m_hmdDesc;
    ovrEyeRenderDesc  m_eyeRenderDesc[ovrEye_Count];
    ovrPosef          m_eyeRenderPose[ovrEye_Count];
    ovrVector3f       m_hmdToEyeOffset[ovrEye_Count];
    OVRBuffer        *m_eyeBuffers[ovrEye_Count];

    OVR::Matrix4f     m_projectionMatrix[ovrEye_Count];
    OVR::Matrix4f     m_eyeOrientation[ovrEye_Count];
    OVR::Matrix4f     m_eyePose[ovrEye_Count];
    

    // frame timing data and tracking info
    double            m_frameTiming;
    ovrTrackingState  m_trackingState;
    ovrInputState     m_inputState;

    // mirror texture used to render HMD view to OpenGL window
    ovrMirrorTexture     m_mirrorTexture;
    ovrMirrorTextureDesc m_mirrorDesc;

    GLuint            m_mirrorFBO;
    bool              m_msaaEnabled;
    long long         m_frameIndex;
    double            m_sensorSampleTime;
    bool              m_isInitialized;

};

#endif
#endif