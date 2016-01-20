CC:=gcc
CFLAGS:= -Wall -std=gnu99
BUILDDIR:=build
LIBDIR:=../build
SAMPLE:= build/imu_gyro build/imu_accel
COMPLIBS = $(wildcard $(LIBDIR)/*.o)

clean:
	@rm -rf $(BUILDDIR)

all: $(addprefix $(BUILDDIR)/, $(patsubst %.c,%,$(wildcard ./*.c)))

$(BUILDDIR)/%:%.c
	@mkdir -p $(BUILDDIR)
	@echo $(COMPLIBS)
	$(CC) -o $@ $< -lwiringPi -I../ $(COMPLIBS) $(CFLAGS)
