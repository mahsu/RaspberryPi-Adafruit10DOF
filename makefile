C:=gcc
CFLAGS:=-Wall -std=gnu99 -O3
BUILDDIR:=build

LIBNAME:=libada10dof
VERSION:=1
INSTALLDIR:=/usr/lib
INCLUDEDIR:=/usr/include
LIBFILENAME:=$(LIBNAME).so.$(VERSION)

.PHONY: clean all prep lib install uninstall

clean:
	@rm -rf $(BUILDDIR)

all: prep lib

prep:
	@mkdir -p $(BUILDDIR)

HEADERS=$(wildcard *.h)

install:
	cp $(BUILDDIR)/$(LIBFILENAME) $(INSTALLDIR)
	cp $(HEADERS) $(INCLUDEDIR)
	ln -s $(INSTALLDIR)/$(LIBFILENAME) $(INSTALLDIR)/$(LIBNAME).so

uninstall:
	rm $(INSTALLDIR)/$(LIBFILENAME)
	rm $(INSTALLDIR)/$(LIBNAME).so
	rm $(addprefix $(INCLUDEDIR)/, $(HEADERS))
	
TARGETS=$(addprefix $(BUILDDIR)/, $(patsubst %.c,%.o,$(wildcard *.c)))

#build everything in the lib directory
lib: prep $(TARGETS)	
	$(CC) -shared -Wl,-soname,$(LIBNAME).so.$(VERION) \
		-o $(BUILDDIR)/$(LIBNAME).so.$(VERSION) $(TARGETS) -lc

#autodetect lib sources
$(BUILDDIR)/%.o: %.c
	$(CC) -fPIC -g -c -o $@ $< -lwiringPi -lm -I. $(CFLAGS)
