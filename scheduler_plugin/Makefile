
MOSEK_LIBDIR := $(HOME)/mosek/9.2/tools/platform/linux64x86/bin/
LDFLAGS := -lboost_thread -lboost_chrono -L$(MOSEK_LIBDIR) -Wl,-rpath-link,$(MOSEK_LIBDIR) -Wl,-rpath=$(MOSEK_LIBDIR) -lmosek64 -lfusion64
CFLAGS := -I./../src/beginner_tutorials
HPP_FILES := $(shell find ../src/beginner_tutorials/ -name '*.h') $(shell find -L . -name '*.hpp')
CPP_FILES := ../src/beginner_tutorials/src/dag_controller_be.cpp ../src/beginner_tutorials/src/dag.cpp ../src/beginner_tutorials/src/dag_multi_core_approx.cpp ../src/beginner_tutorials/src/dag_mc.cpp
WARNINGS := -Wall -Wextra -Wno-unused-function
include common/common.mk
