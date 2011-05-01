DESTDIR     =
prefix      = /usr/local
exec_prefix = $(prefix)
bindir      = $(exec_prefix)/bin
sbindir     = $(exec_prefix)/sbin
datarootdir = $(prefix)/share
datadir     = $(datarootdir)
mandir      = $(datarootdir)/man
man1dir     = $(mandir)/man1

INSTALL         = install
INSTALL_PROGRAM = $(INSTALL)
INSTALL_DATA    = $(INSTALL) -m 644

###

CFLAGS += -Os -g -Wall -Wextra -pipe

TMP_WILD := $(TMP_WILD) *~ *.bak cscope.*
TMP_PAT  := $(subst *,%,$(TMP_WILD))
RELEASE  := $(shell cat release.txt)
MYNAME   := sintvert
DISTNAME  := $(MYNAME)-$(RELEASE)

PROGS := sintvert
MANS := $(addprefix man/, $(PROGS:=.1))
DESKTOPS := $(wildcard data/*.desktop)

CLEAN_FILES := $(MANS) $(PROGS)

.PHONY: clean all srcdist

all: $(PROGS) $(MANS)

sintvert: sintvert.c Makefile
	$(CC) -std=c99 -D_REENTRANT -ljack -lsndfile -lm  -lpthread -lrt $< -o $@

man/%.1: % $(filter-out $(wildcard man), man) Makefile
	help2man -N -o $@ $(abspath $<) || { $< --help || :; $< --version || :; false; }

install: all installdirs
	$(INSTALL_PROGRAM) $(PROGS) $(DESTDIR)$(bindir)
	$(INSTALL_DATA) $(MANS) $(DESTDIR)$(man1dir)
	$(INSTALL_DATA) $(DESKTOPS) $(DESTDIR)$(datadir)/applications

clean:
	set -f; for pat in $(TMP_WILD); do find . -iname $$pat -exec rm {} \; ; done; \
	rm -rf $(CLEAN_FILES)

srcdist: clean
	TD=`mktemp -d /tmp/mkdist.XXXXXX`; \
	cp -a . $$TD/$(DISTNAME); cd $$TD; \
	find $(DISTNAME) '(' -name '.git' -prune ')' -o -type f -print | \
	  tar -cvzf /tmp/$(DISTNAME).tar.gz -T -; \
	test $$TD && $(RM) -r $$TD/*; rmdir $$TD

showvars:
	@echo TMP_PAT: $(TMP_PAT)

man:
	mkdir man

installdirs: mkinstalldirs
	./mkinstalldirs $(DESTDIR)$(bindir) $(DESTDIR)$(datadir) \
	$(DESTDIR)$(mandir) $(DESTDIR)$(man1dir) $(DESTDIR)$(datadir)/applications
