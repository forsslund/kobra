#TEMPLATE = app

TEMPLATE = lib
CONFIG += dynamiclib

CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


TARGET = haptikfabrikenapi
linux: target.path = /usr/local/lib
header_files.files = src/haptikfabrikenapi.h
linux: header_files.path = /usr/local/include
INSTALLS += target
INSTALLS += header_files


# Choose if you are using USB HID edition (currently only linux) or UDP (default) by commenting this line
#CONFIG += use_usb_hid
CONFIG += use_webserv
CONFIG += use_sensoray

# In windows, specify your boost folder
BOOST = F:\boost_1_62_0





SOURCES +=   src/kinematics.cpp \
    src/fshapticdevicethread.cpp \
    src/haptikfabrikenapi.cpp

HEADERS += src/kinematics.h \
    src/fshapticdevicethread.h \
    src/haptikfabrikenapi.h



use_usb_hid {
    SOURCES += src/fsusbhapticdevicethread.cpp \
               external/hidapi/hid.c
    HEADERS += src/fsusbhapticdevicethread.h \
               external/hidapi/hidapi.h
    DEFINES += USE_USB_HID
}

use_webserv {
    SOURCES += src/webserv.cpp
    HEADERS += src/webserv.h src/client_http.h src/server_http.h
}

use_sensoray {
    SOURCES += src/fsdaqhapticdevicethread.cpp
    HEADERS += src/fsdaqhapticdevicethread.h
    DEFINES += USE_DAQ

    unix: LIBS += -l826_64
    INCLUDEPATH += $$PWD/external/sensoray
    DEPENDPATH += $$PWD/external/sensoray
#    unix:!macx: PRE_TARGETDEPS += $$PWD/external/sensoray/sdk_826_linux_3.3.11/middleware/lib826_64.a
}


DEFINES += BOOST_COROUTINES_NO_DEPRECATION_WARNING
DEFINES += BOOST_COROUTINE_NO_DEPRECATION_WARNING

win32{
    DR = "debug"
    CONFIG(release, debug|release): DR = release
    VS_VER = 14
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
    DEFINES += UNIX
    DEFINES += LINUX
    LIBS += -lpthread
    LIBS +=  -lrt -lboost_system -lboost_date_time -lboost_regex -lboost_context -lboost_coroutine -lboost_thread -lboost_chrono

    use_usb_hid: LIBS += -lusb-1.0 -ludev
}





