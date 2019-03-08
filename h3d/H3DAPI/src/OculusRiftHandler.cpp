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
/// \file OculusRiftHandler.cpp
/// \brief CPP file for the Oculus Rift support code
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/OculusRiftHandler.h>
#include <H3D/OculusRiftSensor.h>

#ifdef HAVE_LIBOVR

#define LOG_MESSAGE_ASSERT(assert_cond, msg) if( !assert_cond) H3DUtil::Console( H3DUtil::LogLevel::Error) << msg << std::endl

OculusRiftHandler::OVRBuffer::OVRBuffer(const ovrSession &session, int eyeIdx)
{
    ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
    m_eyeTextureSize   = ovr_GetFovTextureSize(session, (ovrEyeType)eyeIdx, hmdDesc.DefaultEyeFov[eyeIdx], 1.0f);

    ovrTextureSwapChainDesc desc = {};
    desc.Type = ovrTexture_2D;
    desc.ArraySize = 1;
    desc.Width = m_eyeTextureSize.w;
    desc.Height = m_eyeTextureSize.h;
    desc.MipLevels = 1;
    desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    desc.SampleCount = 1;
    desc.StaticImage = ovrFalse;

    ovrResult result = ovr_CreateTextureSwapChainGL(session, &desc, &m_swapTextureChain);

    int textureCount = 0;
    ovr_GetTextureSwapChainLength(session, m_swapTextureChain, &textureCount);

    for (int j = 0; j < textureCount; ++j)
    {
        GLuint chainTexId;
        ovr_GetTextureSwapChainBufferGL(session, m_swapTextureChain, j, &chainTexId);
        glBindTexture(GL_TEXTURE_2D, chainTexId);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    glGenFramebuffers(1, &m_eyeFbo);

    // create depth buffer
    glGenTextures(1, &m_depthBuffer);
    glBindTexture(GL_TEXTURE_2D, m_depthBuffer);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_eyeTextureSize.w, m_eyeTextureSize.h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);

    // MSAA color texture and fbo setup
    // simply comment this line out to skip MSAA altogether
    setupMSAA();
}

void OculusRiftHandler::OVRBuffer::setupMSAA()
{
    glGenFramebuffers(1, &m_msaaEyeFbo);

    // create color MSAA texture
    int samples  = 4;
    int mipcount = 1;
 
    glGenTextures(1, &m_eyeTexMSAA);

    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, m_eyeTexMSAA);

    LOG_MESSAGE_ASSERT(!glGetError(), "OculusRiftHandler: Could not create MSAA texture");

    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_RGBA, m_eyeTextureSize.w, m_eyeTextureSize.h, false);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

    // linear filter
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_MAX_LEVEL, mipcount - 1);    

    // create MSAA depth buffer
    glGenTextures(1, &m_depthTexMSAA);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, m_depthTexMSAA);

    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_DEPTH_COMPONENT, m_eyeTextureSize.w, m_eyeTextureSize.h, false);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_MULTISAMPLE, GL_TEXTURE_MAX_LEVEL, mipcount - 1);

    LOG_MESSAGE_ASSERT(!glGetError(), "OculusRiftHandler: MSAA setup failed");
}

void OculusRiftHandler::OVRBuffer::onRenderMSAA()
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_msaaEyeFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, m_eyeTexMSAA, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D_MULTISAMPLE, m_depthTexMSAA, 0);

    glViewport(0, 0, m_eyeTextureSize.w, m_eyeTextureSize.h);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OculusRiftHandler::OVRBuffer::onRenderMSAAFinish()
{
    // blit the contents of MSAA FBO to the regular eye buffer "connected" to the HMD
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_msaaEyeFbo);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D_MULTISAMPLE, m_eyeTexMSAA, 0);
    glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);

    LOG_MESSAGE_ASSERT((glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE), "Could not complete framebuffer operation");

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_eyeFbo);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeTexId, 0);
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);

    LOG_MESSAGE_ASSERT((glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE), "Could not complete framebuffer operation");

    glBlitFramebuffer(0, 0, m_eyeTextureSize.w, m_eyeTextureSize.h,
                      0, 0, m_eyeTextureSize.w, m_eyeTextureSize.h, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


void OculusRiftHandler::OVRBuffer::onRender()
{
    // Switch to eye render target
    glBindFramebuffer(GL_FRAMEBUFFER, m_eyeFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeTexId, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depthBuffer, 0);

    glViewport(0, 0, m_eyeTextureSize.w, m_eyeTextureSize.h);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OculusRiftHandler::OVRBuffer::onRenderFinish()
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_eyeFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
}

void OculusRiftHandler::OVRBuffer::destroy(const ovrSession &session)
{
    if (glIsFramebuffer(m_eyeFbo))
        glDeleteFramebuffers(1, &m_eyeFbo);  

    if (glIsTexture(m_depthBuffer))
        glDeleteTextures(1, &m_depthBuffer);

    if (glIsFramebuffer(m_msaaEyeFbo))
        glDeleteFramebuffers(1, &m_msaaEyeFbo);

    if (glIsTexture(m_eyeTexMSAA))
        glDeleteTextures(1, &m_eyeTexMSAA);

    if (glIsTexture(m_depthTexMSAA))
        glDeleteTextures(1, &m_depthTexMSAA);

    ovr_DestroyTextureSwapChain(session, m_swapTextureChain);
}

OculusRiftHandler::OculusRiftHandler() : 
  m_hmdSession(nullptr),
  m_msaaEnabled(false),
  m_frameIndex(0),
  m_sensorSampleTime(0),
  m_isInitialized(false),
  m_eyeBuffers(),
  m_eyeRenderDesc(),
  m_eyeRenderPose(),
  m_hmdDesc(),
  m_frameTiming( 0.0 ),
  m_hmdToEyeOffset(),
  m_mirrorDesc(),
  m_mirrorFBO( 0 ),
  m_mirrorTexture( nullptr ),
  m_trackingState() {
  H3D::OculusRiftSensor::oculus = this;
}

OculusRiftHandler::~OculusRiftHandler() {
  destroyVR();
  ovr_Destroy(m_hmdSession);
  ovr_Shutdown();
  m_hmdSession = nullptr;
  if (H3D::OculusRiftSensor::oculus == this) {
    H3D::OculusRiftSensor::oculus = NULL;
  }
}

bool OculusRiftHandler::initVR()
{
    ovrResult result = ovr_Initialize(nullptr);
    ovrGraphicsLuid luid; // as of SDK 0.7.0.0 luid is not supported with OpenGL

    if (result != ovrSuccess)
    {
        LOG_MESSAGE_ASSERT(false, "Failed to initialize LibOVR");
        return false;
    }

    result = ovr_Create(&m_hmdSession, &luid);

    if (result != ovrSuccess)
    {
        LOG_MESSAGE_ASSERT(result == ovrSuccess, "Failed to create OVR device");
    }

    m_hmdDesc = ovr_GetHmdDesc(m_hmdSession);

    m_isInitialized = result == ovrSuccess;
    return m_isInitialized;
}

bool OculusRiftHandler::initVRBuffers(int windowWidth, int windowHeight)
{
    for (int eyeIdx = 0; eyeIdx < ovrEye_Count; eyeIdx++)
    {
        m_eyeBuffers[eyeIdx]    = new OVRBuffer(m_hmdSession, eyeIdx);
        m_eyeRenderDesc[eyeIdx] = ovr_GetRenderDesc(m_hmdSession, (ovrEyeType)eyeIdx, m_hmdDesc.DefaultEyeFov[eyeIdx]);
    }

    memset(&m_mirrorDesc, 0, sizeof(m_mirrorDesc));
    m_mirrorDesc.Width  = windowWidth;
    m_mirrorDesc.Height = windowHeight;
    m_mirrorDesc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

    // since SDK 0.6 we're using a mirror texture + FBO which in turn copies contents of mirror to back buffer
    ovr_CreateMirrorTextureGL(m_hmdSession, &m_mirrorDesc, &m_mirrorTexture);

    // Configure the mirror read buffer
    GLuint texId;
    ovr_GetMirrorTextureBufferGL(m_hmdSession, m_mirrorTexture, &texId);
    glGenFramebuffers(1, &m_mirrorFBO);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texId, 0);
    glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        glDeleteFramebuffers(1, &m_mirrorFBO);
        LOG_MESSAGE_ASSERT(false, "Could not initialize VR buffers!");
        return false;
    }

    return true;
}

void OculusRiftHandler::destroyVR()
{
    if (m_hmdSession)
    {
      //  delete m_debugData;
      //  delete m_cameraFrustum;
       // delete m_trackerChaperone;

      //  m_debugData     = nullptr;
      //  m_cameraFrustum = nullptr;

        if (glIsFramebuffer(m_mirrorFBO))
            glDeleteFramebuffers(1, &m_mirrorFBO);

        ovr_DestroyMirrorTexture(m_hmdSession, m_mirrorTexture);

        for (int eyeIdx = 0; eyeIdx < ovrEye_Count; eyeIdx++)
        {
            m_eyeBuffers[eyeIdx]->destroy(m_hmdSession);
            delete m_eyeBuffers[eyeIdx];
            m_eyeBuffers[eyeIdx] = nullptr;
        }
    }

    m_isInitialized = false;
}

const ovrSizei OculusRiftHandler::getResolution() const
{
    ovrSizei resolution = { m_hmdDesc.Resolution.w, m_hmdDesc.Resolution.h };
    return resolution;
}

void OculusRiftHandler::onRenderStart()
{
    m_hmdToEyeOffset[0] = m_eyeRenderDesc[0].HmdToEyeOffset;
    m_hmdToEyeOffset[1] = m_eyeRenderDesc[1].HmdToEyeOffset;

    // this data is fetched only for the debug display, no need to do this to just get the rendering work
    m_frameTiming   = ovr_GetPredictedDisplayTime(m_hmdSession, 0);

    m_trackingState = ovr_GetTrackingState(m_hmdSession, m_frameTiming, ovrTrue);
    
    // todo: check success or not
    ovr_GetInputState(m_hmdSession, ovrControllerType_Touch, &m_inputState);

    // Get both eye poses simultaneously, with IPD offset already included.
    ovr_GetEyePoses(m_hmdSession, m_frameIndex, ovrTrue, m_hmdToEyeOffset, m_eyeRenderPose, &m_sensorSampleTime);    
}


void OculusRiftHandler::onEyeRender(int eyeIndex, H3D::Matrix4f &view_matrix, H3D::Matrix4f &projection_matrix)
{
    // set the current eye texture in swap chain
    int curIndex;
    ovr_GetTextureSwapChainCurrentIndex(m_hmdSession, m_eyeBuffers[eyeIndex]->m_swapTextureChain, &curIndex);
    ovr_GetTextureSwapChainBufferGL(m_hmdSession, m_eyeBuffers[eyeIndex]->m_swapTextureChain, curIndex, &m_eyeBuffers[eyeIndex]->m_eyeTexId);

    if (m_msaaEnabled)
        m_eyeBuffers[eyeIndex]->onRenderMSAA();
    else
        m_eyeBuffers[eyeIndex]->onRender();

    m_projectionMatrix[eyeIndex] = OVR::Matrix4f(ovrMatrix4f_Projection(m_eyeRenderDesc[eyeIndex].Fov, 0.01f, 10000.0f, ovrProjection_None));
    m_eyeOrientation[eyeIndex] = OVR::Matrix4f(OVR::Quatf(m_eyeRenderPose[eyeIndex].Orientation).Inverted());
    m_eyePose[eyeIndex]        = OVR::Matrix4f::Translation(-OVR::Vector3f(m_eyeRenderPose[eyeIndex].Position));

    OVR::Matrix4f m (m_eyeOrientation[eyeIndex] * m_eyePose[eyeIndex] );
    view_matrix = H3D::Matrix4f(m.M[0][0], m.M[0][1], m.M[0][2], m.M[0][3],
                                m.M[1][0], m.M[1][1], m.M[1][2], m.M[1][3],
                                m.M[2][0], m.M[2][1], m.M[2][2], m.M[2][3],
                                m.M[3][0], m.M[3][1], m.M[3][2], m.M[3][3]);
    m = m_projectionMatrix[eyeIndex] ;
    projection_matrix = H3D::Matrix4f(m.M[0][0], m.M[0][1], m.M[0][2], m.M[0][3],
                                      m.M[1][0], m.M[1][1], m.M[1][2], m.M[1][3],
                                      m.M[2][0], m.M[2][1], m.M[2][2], m.M[2][3],
                                      m.M[3][0], m.M[3][1], m.M[3][2], m.M[3][3]);
}

void OculusRiftHandler::onEyeRenderFinish(int eyeIndex) {
    if (m_msaaEnabled)
        m_eyeBuffers[eyeIndex]->onRenderMSAAFinish();
    else
        m_eyeBuffers[eyeIndex]->onRenderFinish();

    ovr_CommitTextureSwapChain(m_hmdSession, m_eyeBuffers[eyeIndex]->m_swapTextureChain);
}

H3D::Matrix4f OculusRiftHandler::getEyeMVPMatrix(int eyeIndex) const {
    OVR::Matrix4f m = m_projectionMatrix[eyeIndex] * m_eyeOrientation[eyeIndex] * m_eyePose[eyeIndex];
    return H3D::Matrix4f(m.M[0][0], m.M[0][1], m.M[0][2], m.M[0][3],
                         m.M[1][0], m.M[1][1], m.M[1][2], m.M[1][3],
                         m.M[2][0], m.M[2][1], m.M[2][2], m.M[2][3],
                         m.M[3][0], m.M[3][1], m.M[3][2], m.M[3][3]);
}

void OculusRiftHandler::submitFrame()
{
    // set up positional data
    ovrViewScaleDesc viewScaleDesc;
    viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;
    viewScaleDesc.HmdToEyeOffset[0] = m_hmdToEyeOffset[0];
    viewScaleDesc.HmdToEyeOffset[1] = m_hmdToEyeOffset[1];

    // create the main eye layer
    ovrLayerEyeFov eyeLayer;
    eyeLayer.Header.Type  = ovrLayerType_EyeFov;
    eyeLayer.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL.

    for (int eye = 0; eye < ovrEye_Count; eye++)
    {
        eyeLayer.ColorTexture[eye] = m_eyeBuffers[eye]->m_swapTextureChain;
        eyeLayer.Viewport[eye]     = OVR::Recti(m_eyeBuffers[eye]->m_eyeTextureSize);
        eyeLayer.Fov[eye]          = m_hmdDesc.DefaultEyeFov[eye];
        eyeLayer.RenderPose[eye]   = m_eyeRenderPose[eye];
        eyeLayer.SensorSampleTime  = m_sensorSampleTime;
    }

    // append all the layers to global list
    ovrLayerHeader* layerList = &eyeLayer.Header;

    ovrResult result = ovr_SubmitFrame(m_hmdSession, m_frameIndex, nullptr, &layerList, 1);
}

void OculusRiftHandler::blitMirror(ovrEyeType numEyes, int offset)
{
    // Blit mirror texture to back buffer
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    GLint w = m_mirrorDesc.Width;
    GLint h = m_mirrorDesc.Height;

    switch (numEyes)
    {
    case ovrEye_Count:
        glBlitFramebuffer(0, h, w, 0, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        break;
    case ovrEye_Left:
        glBlitFramebuffer(0, h, w / 2, 0, offset, 0, w / 2 + offset, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        break;
    case ovrEye_Right:
        glBlitFramebuffer(w / 2, h, w, 0, offset, 0, w / 2 + offset, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        break;
    default:
        LOG_MESSAGE_ASSERT(false, "Unrecognized ovrEyeType");
    }

    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
}



void OculusRiftHandler::showPerfStats(ovrPerfHudMode statsMode) {
  ovr_SetInt(m_hmdSession, "PerfHudMode", (int)statsMode);
}

void OculusRiftHandler::recenterTracking( ){
  ovr_RecenterTrackingOrigin(m_hmdSession); 
}

void OculusRiftHandler::setControllerVibration(ovrControllerType type, float frequency, float amplitude) {
  ovr_SetControllerVibration( m_hmdSession, type, frequency, amplitude );
}


#endif