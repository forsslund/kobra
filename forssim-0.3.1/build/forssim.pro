#-------------------------------------------------
#
# Project updated 2015-06-09
#
#-------------------------------------------------

QT       -= core gui
CONFIG += c++11

TARGET = forssim
CONFIG(release, debug|release): TARGET = forssim-0.3.1
CONFIG(debug, debug|release):   TARGET = forssim-0.3.1_d

#VC++ ignore warnings
win32 {
    QMAKE_CXXFLAGS += /wd"4100"   # Formal (function) parameter not used
    QMAKE_CXXFLAGS_WARN_ON -= -w34100

    # Note: in QtCreator, in Project add build step (make) with "install"
    #       for both release and debug

    # Uncomment this line if you want the experimental webserv support
    # This requires Boost libraries. Install on Windows through
    # unzipping boost to any folder and then run from cmd.exe:
    #   bootstrap
    #   .\b2 address-model=64
    CONFIG += webserv
    BOOST = F:\boost_1_62_0

    VS_VER = 14  # Visual studio version according to H3D (Ver. 2.3->13, Trunk->14)

    dlltarget.path = F:/h3d/bin64
    INSTALLS += dlltarget
}

unix {
    CONFIG += webserv

    # Install Boost:
    # in /usr/local
    #  sudo tar -xjvf boost_1_63_0.tar.bz2 boost_1_63_0/
    #  cd boost_1_63_0/
    #
    #  sudo ./bootstrap.sh 
    #  sudo ./b2 address-model=64
}


TEMPLATE = lib

DEFINES += QTBUILD_LIBRARY

DEFINES += NOMINMAX
DEFINES += EIGEN_DONT_ALIGN_STATICALLY
DEFINES += BOOST_COROUTINES_NO_DEPRECATION_WARNING
DEFINES += BOOST_COROUTINE_NO_DEPRECATION_WARNING


win32: DEFINES += Q_OS_WIN

SOURCES += \
    ../src/WilhelmsenProjection.cpp \
    ../src/Volumes.cpp \
    ../src/VolumeModel.cpp \
    ../src/SmoothingMask.cpp \
    ../src/PedalNode.cpp \
    ../src/PartitionModel.cpp \
    ../src/MCWithRange.cpp \
    ../src/MCWithDistance.cpp \
    ../src/MaterialSegmentationModel.cpp \
    ../src/IntervalCollisionDetector.cpp \
    ../src/ForbiddenSegmentationModel.cpp \
    ../src/CHapticDrillForce.cpp \
    ../src/CDrillForce.cpp \
    ../src/AVolumePlaybackNode.cpp \
    ../src/AHapticDrillForce.cpp \
    ../src/ADrillForce.cpp \
    ../src/ADrillableNode.cpp \
    ../src/FloatSwitch.cpp \
    ../src/ForssimInfo.cpp


HEADERS += \
    ../src/WilhelmsenProjection.h \
    ../src/Volumes.h \
    ../src/VolumeModel.h \
    ../src/Vec3i.h \
    ../src/SmoothingMask.h \
    ../src/SFUnsignedInt8.h \
    ../src/PedalNode.h \
    ../src/PartitionModel.h \
    ../src/MFUnsignedInt8FromNrrdFile.h \
    ../src/MFUnsignedInt8.h \
    ../src/MFieldFromNrrdFile.h \
    ../src/MFDoubleFromNrrdFile.h \
    ../src/MCWithRange.h \
    ../src/MCWithDistance.h \
    ../src/MaterialSegmentationModel.h \
    ../src/IntervalCollisionDetector.h \
    ../src/ForbiddenSegmentationModel.h \
    ../src/CHapticDrillForce.h \
    ../src/CDrillForce.h \
    ../src/AVolumePlaybackNode.h \
    ../src/AHapticDrillForce.h \
    ../src/ADrillForce.h \
    ../src/ADrillableNode.h \
    ../src/FloatSwitch.h



# Candy mouse
#SOURCES += ../src/MouseHapticsDevice.cpp
#HEADERS += ../src/MouseHapticsDevice.h
#LIBS += -lglu32




unix: {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += ../src

win32 {
    INCLUDEPATH += f:/h3d/h3dutil/include \
                   f:/h3d/h3dapi/include  \
                   f:/h3d/hapi/include    \
                   f:/h3d/external/include/pthread \
                   f:/h3d/external/include
    LIBS += -Lf:/h3d/lib64 \
            -Lf:/h3d/external/lib64
}

CONFIG(release, debug|release){
    win32{
        LIBS += -lH3DUtil_vc$${VS_VER} -lHAPI_vc$${VS_VER} -lH3DAPI_vc$${VS_VER}
    }
    else {
        LIBS += -lh3dutil -lhapi -lh3dapi
    }
}
else:CONFIG(debug, debug|release) {
    win32{
        LIBS +=  -lH3DUtil_vc$${VS_VER}_d -lHAPI_vc$${VS_VER}_d -lH3DAPI_vc$${VS_VER}_d
    }
    else {
        LIBS += -lh3dutil_d -lhapi_d -lh3dapi_d
    }
}


win32:CONFIG(release, debug|release): LIBS += -lpthreadVC2
else:win32:CONFIG(debug, debug|release): LIBS += -lpthreadVC2

win32: LIBS += -lglew32 -lopengl32
LIBS += -lteem

DR = "debug"
CONFIG(release, debug|release): DR = release

webserv {
    VS_VER = 14
    SOURCES += ../src/Webserv.cpp
    HEADERS += ../src/Webserv.h

    win32{
        INCLUDEPATH += $${BOOST}
        A = -L$${BOOST}\bin.v2\libs
        B = \build\msvc-$${VS_VER}.0\\$${DR}\address-model-64\link-static\threading-multi
        LIBS += $${A}\system$${B}
        LIBS += $${A}\date_time$${B}
        LIBS += $${A}\regex$${B}
        LIBS += $${A}\context$${B}
        LIBS += $${A}\coroutine$${B}
        LIBS += $${A}\thread$${B}
        LIBS += $${A}\chrono$${B}
    }
    unix {
        #INCLUDEPATH += /usr/local/boost_1_63_0
	#LIBS += -L/usr/local/boost_1_63_0/stage/lib

        LIBS +=  -lboost_system -lboost_date_time -lboost_regex -lboost_context -lboost_coroutine -lboost_thread -lboost_chrono
    }
}

message("*************** FORSSIM ************* ")
message("Building: " $${DR})
message("Libs: " $${LIBS})
message("************************************* ")
