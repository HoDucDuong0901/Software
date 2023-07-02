QT       += core gui
QT       += serialport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    qcustomplot.cpp \
    spline.cpp \
    src/Accumulator.cpp \
    src/AlbersEqualArea.cpp \
    src/AuxAngle.cpp \
    src/AuxLatitude.cpp \
    src/AzimuthalEquidistant.cpp \
    src/CassiniSoldner.cpp \
    src/CircularEngine.cpp \
    src/DAuxLatitude.cpp \
    src/DMS.cpp \
    src/DST.cpp \
    src/Ellipsoid.cpp \
    src/EllipticFunction.cpp \
    src/GARS.cpp \
    src/GeoCoords.cpp \
    src/Geocentric.cpp \
    src/Geodesic.cpp \
    src/GeodesicExact.cpp \
    src/GeodesicLine.cpp \
    src/GeodesicLineExact.cpp \
    src/Geohash.cpp \
    src/Geoid.cpp \
    src/Georef.cpp \
    src/Gnomonic.cpp \
    src/GravityCircle.cpp \
    src/GravityModel.cpp \
    src/LambertConformalConic.cpp \
    src/LocalCartesian.cpp \
    src/MGRS.cpp \
    src/MagneticCircle.cpp \
    src/MagneticModel.cpp \
    src/Math.cpp \
    src/NormalGravity.cpp \
    src/OSGB.cpp \
    src/PolarStereographic.cpp \
    src/PolygonArea.cpp \
    src/Rhumb.cpp \
    src/SphericalEngine.cpp \
    src/TransverseMercator.cpp \
    src/TransverseMercatorExact.cpp \
    src/UTMUPS.cpp \
    src/Utility.cpp \
    tinyxml2.cpp

HEADERS += \
    GeographicLib/Accumulator.hpp \
    GeographicLib/AlbersEqualArea.hpp \
    GeographicLib/AuxAngle.hpp \
    GeographicLib/AuxLatitude.hpp \
    GeographicLib/AzimuthalEquidistant.hpp \
    GeographicLib/CassiniSoldner.hpp \
    GeographicLib/CircularEngine.hpp \
    GeographicLib/Config.h \
    GeographicLib/Config.h.in \
    GeographicLib/Constants.hpp \
    GeographicLib/DAuxLatitude.hpp \
    GeographicLib/DMS.hpp \
    GeographicLib/DST.hpp \
    GeographicLib/Ellipsoid.hpp \
    GeographicLib/EllipticFunction.hpp \
    GeographicLib/GARS.hpp \
    GeographicLib/GeoCoords.hpp \
    GeographicLib/Geocentric.hpp \
    GeographicLib/Geodesic.hpp \
    GeographicLib/GeodesicExact.hpp \
    GeographicLib/GeodesicLine.hpp \
    GeographicLib/GeodesicLineExact.hpp \
    GeographicLib/Geohash.hpp \
    GeographicLib/Geoid.hpp \
    GeographicLib/Georef.hpp \
    GeographicLib/Gnomonic.hpp \
    GeographicLib/GravityCircle.hpp \
    GeographicLib/GravityModel.hpp \
    GeographicLib/LambertConformalConic.hpp \
    GeographicLib/LocalCartesian.hpp \
    GeographicLib/MGRS.hpp \
    GeographicLib/MagneticCircle.hpp \
    GeographicLib/MagneticModel.hpp \
    GeographicLib/Math.hpp \
    GeographicLib/NearestNeighbor.hpp \
    GeographicLib/NormalGravity.hpp \
    GeographicLib/OSGB.hpp \
    GeographicLib/PolarStereographic.hpp \
    GeographicLib/PolygonArea.hpp \
    GeographicLib/Rhumb.hpp \
    GeographicLib/SphericalEngine.hpp \
    GeographicLib/SphericalHarmonic.hpp \
    GeographicLib/SphericalHarmonic1.hpp \
    GeographicLib/SphericalHarmonic2.hpp \
    GeographicLib/TransverseMercator.hpp \
    GeographicLib/TransverseMercatorExact.hpp \
    GeographicLib/UTMUPS.hpp \
    GeographicLib/Utility.hpp \
    mainwindow.h \
    qcustomplot.h \
    spline.h \
    src/kissfft.hh \
    tinyxml2.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    GeographicLib/.gitignore \
    GeographicLib/CMakeLists.txt \
    src/.gitignore \
    src/CMakeLists.txt
