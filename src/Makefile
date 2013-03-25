# -*- Mode: Makefile; tab-width: 4; indent-tabs-mode: t -*-
#------------------------------------------------------------------------------
# Copyright (c) 2013, Marcus Geelnard
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#------------------------------------------------------------------------------

OPENCTM_PATH = third_party/openctm/lib
LIBOPENCTM = $(OPENCTM_PATH)/libopenctm2.a
TINYXML2_PATH = third_party/tinyxml2

CPP = g++
CPPFLAGS = -c -std=c++0x -O3 -ffast-math -W -Wall -Wextra \
           -I. -I$(OPENCTM_PATH) -I$(TINYXML2_PATH) -DSCALAR_IS_FLOAT \
           -fopenmp
LFLAGS = -L$(OPENCTM_PATH) -fopenmp
LIBS = -lpng -lopenctm2 -lpthread -lm

DEPEND = $(CPP) -MM

# Debug flags
CPPFLAGS += -D_DEBUG -g
LFLAGS += -g

LIB_SRCS = aabb.cpp \
           camera.cpp \
           image.cpp \
           light.cpp \
           mat.cpp \
           material.cpp \
           mesh.cpp \
           mesh_data.cpp \
           object.cpp \
           scene.cpp \
           tree.cpp \
           base/perf.cpp \
           importers/openctm_importer.cpp \
           third_party/tinyxml2/tinyxml2.cpp

LIB_OBJS = aabb.o \
           camera.o \
           image.o \
           light.o \
           mat.o \
           material.o \
           mesh.o \
           mesh_data.o \
           object.o \
           openctm_importer.o \
           perf.o \
           scene.o \
           tinyxml2.o \
           tree.o

.PHONY: all clean depend

all: mageray

clean:
	rm -rf mageray mageray.o $(LIB_OBJS) make.depend
	cd third_party/openctm/lib && make -f Makefile.linux clean

mageray: mageray.o $(LIB_OBJS) $(LIBOPENCTM)
	$(CPP) -o $@ $(LFLAGS) mageray.o $(LIB_OBJS) $(LIBS)

$(LIBOPENCTM):
	cd $(OPENCTM_PATH) && $(MAKE) -f Makefile.linux libopenctm2.a

%.o: %.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

%.o: base/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

%.o: importers/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

%.o: third_party/tinyxml2/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

depend:
	$(DEPEND) $(CPPFLAGS) $(LIB_SRCS) mageray.cpp > make.depend

-include make.depend
