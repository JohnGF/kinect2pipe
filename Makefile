CXX = g++
CXXFLAGS = -std=c++14 -O3 -march=native -mtune=native -Wall -Wextra
SCRIPT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

INCLUDES = -I$(SCRIPT_DIR)include \
           -I$(SCRIPT_DIR)../libfreenect2/include -I$(SCRIPT_DIR)../libfreenect2/build

LIBS = -L$(SCRIPT_DIR)lib -L$(SCRIPT_DIR)../libfreenect2/build/lib \
       -Wl,-rpath,'$$ORIGIN/../lib:$$ORIGIN/../../libfreenect2/build/lib:$$ORIGIN' \
       -lfreenect2 -lswscale -lpthread

TARGET = build/kinect2pipe

SRCS = kinect2pipe.cpp main.cpp
HEADERS = kinect2pipe.h

all: $(TARGET)

$(TARGET): $(SRCS) $(HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRCS) $(LIBS) -o $(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all clean
