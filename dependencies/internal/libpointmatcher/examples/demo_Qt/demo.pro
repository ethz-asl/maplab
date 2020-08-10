# Example of a configuration file for QT Creator
#
# NOTE: you will need to adapt the paths to your own system

QT       	+= core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
TARGET    	 = LAUPointMatcher
TEMPLATE  	 = app
SOURCES  	+= main.cpp

INCLUDEPATH += 	/Users/francoispomerleau/Research/Code/libpointmatcher/pointmatcher \
                /Users/francoispomerleau/Research/Code/libnabo/ \
                /usr/local/Cellar/eigen/3.2.4/include/eigen3 \
		/usr/local/include/

CONFIG          += c++11
#QMAKE_CXXFLAGS += -mmacosx-version-min=10.7
#QMAKE_LFLAGS   += -mmacosx-version-min=10.7

LIBS     	+= 	/usr/local/lib/libboost_thread-mt.dylib \
                	/usr/local/lib/libboost_filesystem-mt.dylib \
                        /usr/local/lib/libboost_system-mt.dylib \
                        /usr/local/lib/libboost_program_options-mt.dylib \
                        /usr/local/lib/libboost_date_time-mt.dylib \
                        /usr/local/lib/libboost_chrono-mt.dylib \
                        /Users/francoispomerleau/Research/Code/libpointmatcher/build/libpointmatcher.a \
                        /Users/francoispomerleau/Research/Code/libnabo/build/libnabo.a \
                        /Users/francoispomerleau/Research/Code/libpointmatcher/build/contrib/yaml-cpp-pm/libyaml-cpp-pm.a
