TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += main.cpp


unix:!macx:!symbian: LIBS += -lopencv_core -lopencv_highgui

HEADERS += \
    KalmanFilter.h \
    DrawUtilities.h \
    SerialReader.h
