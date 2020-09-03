QT += core gui widgets charts

TARGET = calibration_imu
TEMPLATE = app

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../../devel/include

SOURCES += \
        src/magnetometer.cpp \
        src/main.cpp \
        src/fmain.cpp

HEADERS += \
        src/fmain.h \
        src/magnetometer.h

FORMS += \
        src/fmain.ui

DISTFILES += \
    CMakeLists.txt \
    package.xml
