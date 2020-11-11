CFLAGS := -I./../src/beginner_tutorials
HPP_FILES := $(shell find ../src/beginner_tutorials/ -name '*.h') $(shell find -L . -name '*.hpp')
CPP_FILES := $(shell find ../src/beginner_tutorials/src/ -name '*.cpp' \
				-not -name 'controller_node.cpp' \
				-not -name 'dag_controller_node.cpp' \
				-not -name '*subscriber.cpp' \
				-not -name 'globalcmp_td.cpp' \
				-not -name 'local_planner.cpp' \
				-not -name 'new_controller_node.cpp' \
				-not -name 'obj_detector.cpp' \
				-not -name 'obj_tracker.cpp' \
				-not -name '*publisher.cpp' \
				-not -name 'shim_freq_node.cpp')
WARNINGS := -Wall -Wextra -Wno-unsued-function
include common/common.mk
