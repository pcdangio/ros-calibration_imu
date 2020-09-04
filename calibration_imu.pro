QT += core gui widgets charts

TARGET = calibration_imu
TEMPLATE = app

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../../devel/include \
    ../qn_optimizer/include

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
