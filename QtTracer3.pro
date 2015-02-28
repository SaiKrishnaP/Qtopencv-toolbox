#-------------------------------------------------
#
# Project created by QtCreator 2013-04-22T19:00:40
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QtTracer3
TEMPLATE = app

INCLUDEPATH = /usr/local/include

SOURCES += main.cpp\
        dialog.cpp

HEADERS  += dialog.h

QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.3

CONFIG-=app_bundle

LIBS += -lm -lopencv_core \
 -lopencv_calib3d \
 -lopencv_imgproc \
 -lopencv_photo \
 -lopencv_contrib \
 -lopencv_legacy \
 -lopencv_stitching \
 -lopencv_ml \
 -lopencv_ts \
 -lopencv_features2d \
 -lopencv_nonfree \
 -lopencv_video \
 -lopencv_flann \
 -lopencv_objdetect \
 -lopencv_videostab \
 -lopencv_gpu \
 -lopencv_highgui \
 -lopencv_video




FORMS    += dialog.ui


