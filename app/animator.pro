
# Only Linux

TARGET = animator
CONFIG *= qt opengl c++11
QT *= opengl xml

SOURCES += src/main.cpp \
    src/animator.cpp
HEADERS += \
    include/animator.h


RESOURCES = resources.qrc

INCLUDEPATH *= ./include
INCLUDEPATH *= ../include

# Eigen
INCLUDEPATH *= /usr/include/eigen3
# OpenMesh
LIBS *= -lOpenMeshCore
# libqglviewer
LIBS *= -lQGLViewer

QMAKE_CXXFLAGS *= -fopenmp
# QMAKE_CXXFLAGS += -DSURFMORPH_DONT_PARALLELIZE
# QMAKE_CXXFLAGS += -DSURFMORPH_PARALLELIZE_INTERPOLATION
QMAKE_LFLAGS *= -fopenmp

# Optimization
QMAKE_CXXFLAGS_RELEASE -= -O1
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE *= -O3
QMAKE_CXXFLAGS_RELEASE *= -DNDEBUG
QMAKE_CXXFLAGS_RELEASE *= -DEIGEN_NO_STATIC_ASSERT
QMAKE_CXXFLAGS_RELEASE *= -march=native
