C:=gcc
CFLAGS:=-Wall -std=gnu99
BUILDDIR:=build

.PHONY: clean all prep lib

clean:
	@rm -rf $(BUILDDIR)

all: prep lib

prep:
	@mkdir -p $(BUILDDIR)

#build everything in the lib directory
lib: prep $(addprefix $(BUILDDIR)/, $(patsubst %.c,%.o,$(wildcard ./*.c)))

#autodetect lib sources
$(BUILDDIR)/%.o: %.c
	$(CC) -c -o $@ $< -lwiringPi -I. $(CFLAGS)

