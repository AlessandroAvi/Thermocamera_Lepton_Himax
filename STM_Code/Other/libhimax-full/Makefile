LIB = libhimax

OBJECTS = i2c.o hm01b0.o

DEPS = libio libmsp

override SRC_ROOT = ../../src

override CFLAGS += \
        -I../../src/include \
        -I../../src/include/$(LIB) \
        -mlarge

LFLAGS += -mlarge

include $(MAKER_ROOT)/Makefile.$(TOOLCHAIN)

