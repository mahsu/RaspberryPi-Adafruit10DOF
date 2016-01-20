CC:=gcc
CFLAGS:= -Wall -std=gnu99
BUILDDIR:=build
LIBDIR:=../build

COMPLIBS = $(wildcard $(LIBDIR)/*.o)

imu_gyro: imu_gyro.c
	@mkdir -p $(BUILDDIR)
	$(CC) -o $(BUILDDIR)/$@ $< -lwiringPi -I../ $(COMPLIBS) $(CFLAGS)
