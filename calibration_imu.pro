QT += core gui widgets

TARGET = calibration_imu
TEMPLATE = app

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../../devel/include

SOURCES += \
        src/main.cpp \
        src/fmain.cpp

HEADERS += \
        src/fmain.h

FORMS += \
        src/fmain.ui

DISTFILES += \
    CMakeLists.txt \
    package.xml
