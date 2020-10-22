QT += core gui widgets

TARGET = calibration_imu
TEMPLATE = app

INCLUDEPATH += \
    /opt/ros/melodic/include \
    /opt/ros/melodic/include/ifopt \
    ../../devel/include \
    src

DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    README.md \
    package.xml

FORMS += \
    src/accelerometer/gui/fmain.ui \
    src/magnetometer/gui/fmain.ui

HEADERS += \
    src/accelerometer/data/data_manager.h \
    src/accelerometer/gui/fmain.h \
    src/magnetometer/calibration/calibrator.h \
    src/magnetometer/calibration/cost_objective.h \
    src/magnetometer/calibration/variables_center.h \
    src/magnetometer/calibration/variables_radius.h \
    src/magnetometer/calibration/variables_rotation.h \
    src/magnetometer/data/data_interface.h \
    src/magnetometer/data/ellipsoid.h \
    src/magnetometer/geometry/ellipsoid.h \
    src/magnetometer/graph/graph.h \
    src/magnetometer/gui/fmain.h

SOURCES += \
    src/accelerometer/data/data_manager.cpp \
    src/accelerometer/gui/fmain.cpp \
    src/accelerometer/main.cpp \
    src/magnetometer/calibration/calibrator.cpp \
    src/magnetometer/calibration/cost_objective.cpp \
    src/magnetometer/calibration/variables_center.cpp \
    src/magnetometer/calibration/variables_radius.cpp \
    src/magnetometer/calibration/variables_rotation.cpp \
    src/magnetometer/data/data_interface.cpp \
    src/magnetometer/geometry/ellipsoid.cpp \
    src/magnetometer/graph/graph.cpp \
    src/magnetometer/gui/fmain.cpp \
    src/magnetometer/main.cpp
