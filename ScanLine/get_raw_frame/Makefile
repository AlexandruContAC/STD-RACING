PIXY2_DIR = ../pixy2

CXX      = g++
CPPFLAGS = -g -fpermissive \
           -I/usr/include/libusb-1.0 \
           -I$(PIXY2_DIR)/src/host/libpixyusb2/include \
           -I$(PIXY2_DIR)/src/host/arduino/libraries/Pixy2 \
           -I$(PIXY2_DIR)/src/common/inc
LDLIBS   = $(PIXY2_DIR)/build/libpixyusb2/libpixy2.a -lusb-1.0

SRCS = get_raw_frame.cpp
OBJS = $(subst .cpp,.o,$(SRCS))

all: get_raw_frame

clean:
	rm -f *.o get_raw_frame

get_raw_frame: $(OBJS)
	$(CXX) $(LDFLAGS) -o get_raw_frame $(OBJS) $(LDLIBS)
