#-------------------------------------------------
#
# Project created by QtCreator 2017-08-06T15:48:31
#
#-------------------------------------------------

QT       += core gui
QT       += serialport
QT       += multimedia
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Commender
TEMPLATE = app

CONFIG += console



SOURCES += main.cpp\
        mainwindow.cpp \
    puma560.cpp \
    snakerobot.cpp \
    kdlfun.cpp

HEADERS  += mainwindow.h \
    myhelper.h \
    colorsetter.h \
    kdl.h \
    models.hpp

#here must use Eigen3 but not Eigen2
INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /home/gavin/Desktop/SnakeLike/orocos_kinematics_dynamics/orocos_kdl/models

INCLUDEPATH += /usr/local/include/kdl

#you must link this lib to use some functions in KDL
LIBS += /usr/local/lib/liborocos-kdl.so


FORMS    += mainwindow.ui
