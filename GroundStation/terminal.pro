QT += widgets serialport

TARGET = terminal
TEMPLATE = app

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    settingsdialog.cpp \
    console.cpp \
    protocol_parser.cpp \
    crc32.cpp \
    utils.cpp

HEADERS += \
    mainwindow.h \
    settingsdialog.h \
    console.h \
    protocol_parser.h \
    crc32.h \
    utils.h

FORMS += \
    mainwindow.ui \
    settingsdialog.ui

RESOURCES += \
    terminal.qrc

target.path = /media/tuba/Second/physics/diplomna/GroundStation/build
INSTALLS += target
