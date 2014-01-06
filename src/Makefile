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

DEBUG ?= YES
OPENMP ?= YES
LIBPNG ?= YES

LIB_PATH = lib
OUT_PATH = out
OBJ_PATH = $(OUT_PATH)/obj
BIN_PATH = $(OUT_PATH)/bin

OPENCTM_PATH = third_party/openctm/lib
LIBOPENCTM = $(OPENCTM_PATH)/libopenctm2.a
TINYXML2_PATH = third_party/tinyxml2
LODEPNG_PATH = third_party/lodepng
EXPRTK_PATH = third_party/exprtk

CPP = g++
CPPFLAGS = -c -std=c++0x \
           -Ofast -funroll-loops \
           -W -Wall -Wextra \
           -I$(LIB_PATH) -I$(OPENCTM_PATH) -I$(TINYXML2_PATH) -I$(EXPRTK_PATH) \
           -DSCALAR_IS_FLOAT
LFLAGS = -L$(OPENCTM_PATH)
LIBS = -lopenctm2 -lpthread -lm

LIB_OBJS = $(OBJ_PATH)/aabb.o \
           $(OBJ_PATH)/camera.o \
           $(OBJ_PATH)/expression_parser.o \
           $(OBJ_PATH)/image.o \
           $(OBJ_PATH)/light.o \
           $(OBJ_PATH)/mat.o \
           $(OBJ_PATH)/material.o \
           $(OBJ_PATH)/mesh.o \
           $(OBJ_PATH)/mesh_data.o \
           $(OBJ_PATH)/object.o \
           $(OBJ_PATH)/openctm_importer.o \
           $(OBJ_PATH)/perf.o \
           $(OBJ_PATH)/photon_map.o \
           $(OBJ_PATH)/sampler.o \
           $(OBJ_PATH)/scene.o \
           $(OBJ_PATH)/shader.o \
           $(OBJ_PATH)/tinyxml2.o \
           $(OBJ_PATH)/tree.o \
           $(OBJ_PATH)/tracer.o

# Use libpng or lodepng?
ifeq ($(LIBPNG), YES)
CPPFLAGS += -DUSE_LIBPNG
LIBS += -lpng
else
CPPFLAGS += -DUSE_LODEPNG
CPPFLAGS += -I$(LODEPNG_PATH)
LIB_OBJS += $(OBJ_PATH)/lodepng.o
endif

# Use OpenMP?
ifeq ($(OPENMP), YES)
CPPFLAGS += -DUSE_OPENMP -fopenmp
LFLAGS += -fopenmp
endif

# Enable debugging?
ifeq ($(DEBUG), YES)
CPPFLAGS += -D_DEBUG -g
LFLAGS += -g
endif

# Automatically generate dependency files
CPPFLAGS += -MMD -MP

MAGERAY = $(BIN_PATH)/mageray

.PHONY: all clean

all: $(MAGERAY)

clean:
	rm -rf $(MAGERAY) $(OBJ_PATH)/mageray.o $(LIB_OBJS)
	cd third_party/openctm/lib && make -f Makefile.linux clean

$(MAGERAY): $(OBJ_PATH)/mageray.o $(LIB_OBJS) $(LIBOPENCTM)
	$(CPP) -o $@ $(LFLAGS) $(OBJ_PATH)/mageray.o $(LIB_OBJS) $(LIBS)

$(LIBOPENCTM):
	cd $(OPENCTM_PATH) && $(MAKE) -f Makefile.linux libopenctm2.a

$(OBJ_PATH)/%.o: %.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

$(OBJ_PATH)/%.o: $(LIB_PATH)/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

$(OBJ_PATH)/%.o: $(LIB_PATH)/base/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

$(OBJ_PATH)/%.o: $(LIB_PATH)/importers/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

$(OBJ_PATH)/%.o: third_party/tinyxml2/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

$(OBJ_PATH)/%.o: third_party/lodepng/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<

# These dependency files are generated automatically when building the object
# files.
-include $(LIB_OBJS:.o=.d) $(OBJ_PATH)/mageray.d

