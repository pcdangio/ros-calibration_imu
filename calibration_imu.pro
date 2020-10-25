QT += core gui widgets charts

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
    src/accelerometer/calibration/calibrator.h \
    src/accelerometer/calibration/cost_term.h \
    src/common/geometry/ellipsoid.h \
    src/common/calibration/variables_center.h \
    src/common/calibration/variables_radius.h \
    src/accelerometer/data/data_interface.h \
    src/accelerometer/graph/graph.h \
    src/accelerometer/gui/fmain.h \
    src/magnetometer/calibration/calibrator.h \
    src/magnetometer/calibration/cost_term.h \
    src/magnetometer/calibration/variables_rotation.h \
    src/magnetometer/data/data_interface.h \
    src/magnetometer/data/ellipsoid.h \
    src/magnetometer/graph/graph.h \
    src/magnetometer/gui/fmain.h

SOURCES += \
    src/accelerometer/calibration/calibrator.cpp \
    src/accelerometer/calibration/cost_term.cpp \
    src/common/geometry/ellipsoid.cpp \
    src/common/calibration/variables_center.cpp \
    src/common/calibration/variables_radius.cpp \
    src/accelerometer/data/data_interface.cpp \
    src/accelerometer/graph/graph.cpp \
    src/accelerometer/gui/fmain.cpp \
    src/accelerometer/main.cpp \
    src/magnetometer/calibration/calibrator.cpp \
    src/magnetometer/calibration/cost_term.cpp \
    src/magnetometer/calibration/variables_rotation.cpp \
    src/magnetometer/data/data_interface.cpp \
    src/magnetometer/graph/graph.cpp \
    src/magnetometer/gui/fmain.cpp \
    src/magnetometer/main.cpp
