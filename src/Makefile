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

CPP = g++
CPPFLAGS = -c -O3 -W -Wall -std=c++0x
LFLAGS =
LIBS = -lpng

# Debug flags
CPPFLAGS += -D_DEBUG -g
LFLAGS += -g

LIB_OBJS = camera.o \
           image.o \
           light.o \
           mat.o \
           material.o \
           mesh.o \
           object.o \
           scene.o \
           tree.o

.PHONY: all clean

all: test

clean:
	rm -rf test test.o $(LIB_OBJS)

test: test.o $(LIB_OBJS)
	$(CPP) -o test $(LFLAGS) test.o $(LIB_OBJS) $(LIBS)

test.o: test.cpp vec.h mat.h image.h
	$(CPP) $(CPPFLAGS) test.cpp

camera.o: camera.cpp camera.h
	$(CPP) $(CPPFLAGS) camera.cpp

image.o: image.cpp image.h pixel.h
	$(CPP) $(CPPFLAGS) image.cpp

light.o: light.cpp light.h
	$(CPP) $(CPPFLAGS) light.cpp

mat.o: mat.cpp mat.h
	$(CPP) $(CPPFLAGS) mat.cpp

material.o: material.cpp material.h
	$(CPP) $(CPPFLAGS) material.cpp

mesh.o: mesh.cpp mesh.h
	$(CPP) $(CPPFLAGS) mesh.cpp

object.o: object.cpp object.h
	$(CPP) $(CPPFLAGS) object.cpp

scene.o: scene.cpp scene.h camera.h image.h mesh.h material.h light.h object.h
	$(CPP) $(CPPFLAGS) scene.cpp

tree.o: tree.cpp tree.h
	$(CPP) $(CPPFLAGS) tree.cpp

