#
TARGET = de1_soc_hps_binary

#
CROSS_COMPILE = arm-linux-gnueabihf-

# cris: original next does not work"
#CFLAGS = -static -g -Wall  -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include
# solution fix from: http://www.alteraforum.com/forum/showthread.php?t=48706
HWLIBS_ROOT = ${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib
ALT_DEVICE_FAMILY ?= soc_cv_av
CFLAGS := -I$(HWLIBS_ROOT)/include -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) -D$(ALT_DEVICE_FAMILY)
#CFLAGS := -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include/$(ALT_DEVICE_FAMILY) -D$(ALT_DEVICE_FAMILY)

LDFLAGS =  -g -Wall
CC = $(CROSS_COMPILE)gcc
ARCH= arm


build: $(TARGET)
$(TARGET): main.o
	$(CC) $(LDFLAGS)   $^ -o $@  
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ 
