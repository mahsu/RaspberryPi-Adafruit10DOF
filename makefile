C:=gcc
CFLAGS:=-Wall -std=gnu99
BUILDDIR:=build

.PHONY: clean prep lib

clean:
	@rm -rf $(BUILDDIR)

prep:
	@mkdir -p $(BUILDDIR)

#build everything in the lib directory
lib: prep $(addprefix $(BUILDDIR)/, $(patsubst %.c,%.o,$(wildcard ./*.c)))

#autodetect lib sources
$(BUILDDIR)/%.o: %.c
	$(CC) -c -o $@ $< -lwiringPi -I. $(CFLAGS)

