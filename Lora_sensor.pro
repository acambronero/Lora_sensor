QT -= gui core

CONFIG += c++11 console
CONFIG -= app_bundle

QMAKE_CXXFLAGS += -std=c++11

QMAKE_CXXFLAGS += -fpermissive

unix:LIBS += -pthread

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += /usr/include/
INCLUDEPATH += /usr/local/include/

LIBS += -L"/usr/lib"
LIBS += -lwiringPi

DEFINES += RASPI

INCLUDEPATH += \
        $$PWD/SX126x \
        $$PWD/SX126x/boards \
        $$PWD/SX126x/radio \

SOURCES += \
        SX126x/boards/mcu/espressif/timer.cpp \
        SX126x/radio/sx126x/radiohandler.cpp \
        SX126x/radio/sx126x/sx126x.cpp \
        SX126xHardware.cpp \
        datahelper.cpp \
        lorahandler.cpp \
        loratimersloop.cpp \
        main.cpp \
        monitoringhandler.cpp \
        spibase.cpp \
        spilora.cpp \
        timershandler.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    SX126x/boards/mcu/timer.h \
    SX126x/radio/radiohandler.h \
    SX126x/radio/sx126x/sx126x.h \
    SX126xHardware.h \
    datahelper.h \
    loop.h \
    lorahandler.h \
    loratimersloop.h \
    monitoringhandler.h \
    spibase.h \
    spilora.h \
    timershandler.h
