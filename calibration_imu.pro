QT += core gui widgets charts

TARGET = calibration_imu
TEMPLATE = app

INCLUDEPATH += \
    /opt/ros/melodic/include \
    /opt/ros/melodic/include/ifopt \
    ../../devel/include \
    src

SOURCES += \
        src/magnetometer/optimizer/cost_objective.cpp \
        src/magnetometer/optimizer/optimizer.cpp \
        src/magnetometer/optimizer/variables_center.cpp \
        src/magnetometer/optimizer/variables_radius.cpp \
        src/magnetometer/optimizer/variables_rotation.cpp \
        src/main.cpp \
        src/fmain.cpp

HEADERS += \
        src/fmain.h \
        src/magnetometer/optimizer/cost_objective.h \
        src/magnetometer/optimizer/optimizer.h \
        src/magnetometer/optimizer/variables_center.h \
        src/magnetometer/optimizer/variables_radius.h \
        src/magnetometer/optimizer/variables_rotation.h \
        src/magnetometer/point.h

FORMS += \
        src/fmain.ui

DISTFILES += \
    CMakeLists.txt \
    package.xml
