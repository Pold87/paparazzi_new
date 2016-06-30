#-------------------------------------------------
#
# Project created by QtCreator 2016-05-10T22:00:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = trexton_qt
TEMPLATE = app


INCLUDEPATH += ../../airborne/modules/particle_filter/
INCLUDEPATH += /usr/include/glib-2.0
INCLUDEPATH += /usr/lib/x86_64-linux-gnu/glib-2.0/include

# TODO: might go better with pkg-config
LIBS += -lglibivy -lglib-2.0 -lpcre -lm

QMAKE_UIC

SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h

FORMS    += mainwindow.ui


