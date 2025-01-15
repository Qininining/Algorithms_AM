QT       += core gui serialport charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Algorithms_AM.cpp \
    AttitudeMeasurement.cpp \
    ForceSensor.cpp \
    MotionPlatform.cpp \
    PidController.cpp \
    Scanner.cpp \
    SerialCommon.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    Algorithms_AM.h \
    AttitudeMeasurement.h \
    ForceSensor.h \
    MotionPlatform.h \
    PidController.h \
    Scanner.h \
    SerialCommon.h \
    mainwindow.h

FORMS += \
    mainwindow.ui


INCLUDEPATH += \
    $$PWD/NanoDrive2.8.12/04_SDK/include \
    $$PWD/eigen-3.4.0 \
    $$PWD/SCANNERDLL/sdk240410

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32: LIBS += -L$$PWD/SCANNERDLL/sdk240410/64/ -lScanControl

INCLUDEPATH += $$PWD/SCANNERDLL/sdk240410/64
DEPENDPATH += $$PWD/SCANNERDLL/sdk240410/64

win32: LIBS += -L$$PWD/NanoDrive2.8.12/04_SDK/lib64/ -lNTControl

INCLUDEPATH += $$PWD/NanoDrive2.8.12/04_SDK/lib64
DEPENDPATH += $$PWD/NanoDrive2.8.12/04_SDK/lib64
