QT       += core gui
QT       += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += console

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    external/Mavlink2/Airside_Functions.cpp \
    external/Mavlink2/Groundside_Functions.cpp \
    src/json_functions/Json_Functions.cpp \
    src/mainwindow/mainwindow.cpp \
    src/serial/serial.cpp \
    src/main.cpp \


HEADERS += \
    external/Mavlink2/Mavlink2_lib/common/common.h \
    external/Mavlink2/Airside_Functions.hpp \
    external/Mavlink2/Groundside_Functions.hpp \
    external/Mavlink2/Encodings.hpp \
    external/json.hpp \
    src/json_functions/Json_Functions.h \
    src/mainwindow/mainwindow.h \
    src/serial/serial.h \
    src/json.hpp \

FORMS += \
    src/mainwindow/mainwindow.ui


INCLUDEPATH += \
    src/ \
    external/

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
