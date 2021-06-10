QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += console

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Json_Functions.cpp \
    main.cpp \
    pilotmanager.cpp \
    Mavlink2/Airside_Functions.cpp \
    Mavlink2/Groundside_Functions.cpp

HEADERS += \
    Json_Functions.h \
    pilotmanager.h \
    Mavlink2/Mavlink2_lib/common/common.h \
    Mavlink2/Airside_Functions.hpp \
    Mavlink2/Groundside_Functions.hpp \
    Mavlink2/Encodings.hpp

FORMS += \
    pilotmanager.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
