# Makefile.in generated by automake 1.11.6 from Makefile.am.
# regress/gtest/Makefile.  Generated from Makefile.in by configure.

# Copyright (C) 1994, 1995, 1996, 1997, 1998, 1999, 2000, 2001, 2002,
# 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010, 2011 Free Software
# Foundation, Inc.
# This Makefile.in is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.



#
#  $Id$
#


am__make_dryrun = \
  { \
    am__dry=no; \
    case $$MAKEFLAGS in \
      *\\[\ \	]*) \
        echo 'am--echo: ; @echo "AM"  OK' | $(MAKE) -f - 2>/dev/null \
          | grep '^AM OK$$' >/dev/null || am__dry=yes;; \
      *) \
        for am__flg in $$MAKEFLAGS; do \
          case $$am__flg in \
            *=*|--*) ;; \
            *n*) am__dry=yes; break;; \
          esac; \
        done;; \
    esac; \
    test $$am__dry = yes; \
  }
pkgdatadir = $(datadir)/simulavr
pkgincludedir = $(includedir)/simulavr
pkglibdir = $(libdir)/simulavr
pkglibexecdir = $(libexecdir)/simulavr
am__cd = CDPATH="$${ZSH_VERSION+.}$(PATH_SEPARATOR)" && cd
install_sh_DATA = $(install_sh) -c -m 644
install_sh_PROGRAM = $(install_sh) -c
install_sh_SCRIPT = $(install_sh) -c
INSTALL_HEADER = $(INSTALL_DATA)
transform = $(program_transform_name)
NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
build_triplet = x86_64-unknown-linux-gnu
host_triplet = x86_64-unknown-linux-gnu
noinst_PROGRAMS = dut$(EXEEXT)
subdir = regress/gtest
DIST_COMMON = $(srcdir)/Makefile.am $(srcdir)/Makefile.in
ACLOCAL_M4 = $(top_srcdir)/aclocal.m4
am__aclocal_m4_deps = $(top_srcdir)/m4/ac_pkg_swig.m4 \
	$(top_srcdir)/m4/avr_tcl.m4 \
	$(top_srcdir)/m4/ax_prog_doxygen.m4 \
	$(top_srcdir)/m4/ax_python_module.m4 \
	$(top_srcdir)/m4/az_python.m4 $(top_srcdir)/m4/check_mingw.m4 \
	$(top_srcdir)/m4/check_msys_zlib.m4 \
	$(top_srcdir)/m4/check_winsock.m4 \
	$(top_srcdir)/m4/swig_enable_cxx.m4 $(top_srcdir)/configure.ac
am__configure_deps = $(am__aclocal_m4_deps) $(CONFIGURE_DEPENDENCIES) \
	$(ACLOCAL_M4)
mkinstalldirs = $(install_sh) -d
CONFIG_HEADER = $(top_builddir)/src/config.h
CONFIG_CLEAN_FILES =
CONFIG_CLEAN_VPATH_FILES =
PROGRAMS = $(noinst_PROGRAMS)
am__dirstamp = $(am__leading_dot)dirstamp
am__objects_1 = session_001/unittest001.$(OBJEXT) \
	session_irq_check/unittest_irq.$(OBJEXT) \
	session_io_pin/unittest_io_pin.$(OBJEXT) gtest_main.$(OBJEXT)
am__objects_2 = gtest-1.6.0/src/gtest-all.$(OBJEXT)
am_dut_OBJECTS = $(am__objects_1) $(am__objects_2)
dut_OBJECTS = $(am_dut_OBJECTS)
am__DEPENDENCIES_1 =
DEFAULT_INCLUDES = -I. -I$(top_builddir)/src
depcomp = $(SHELL) $(top_srcdir)/depcomp
am__depfiles_maybe = depfiles
am__mv = mv -f
CXXCOMPILE = $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) \
	$(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS)
LTCXXCOMPILE = $(LIBTOOL) --tag=CXX $(AM_LIBTOOLFLAGS) $(LIBTOOLFLAGS) \
	--mode=compile $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) \
	$(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS)
CXXLD = $(CXX)
CXXLINK = $(LIBTOOL) --tag=CXX $(AM_LIBTOOLFLAGS) $(LIBTOOLFLAGS) \
	--mode=link $(CXXLD) $(AM_CXXFLAGS) $(CXXFLAGS) $(AM_LDFLAGS) \
	$(LDFLAGS) -o $@
SOURCES = $(dut_SOURCES)
DIST_SOURCES = $(dut_SOURCES)
am__can_run_installinfo = \
  case $$AM_UPDATE_INFO_DIR in \
    n|no|NO) false;; \
    *) (install-info --version) >/dev/null 2>&1;; \
  esac
ETAGS = etags
CTAGS = ctags
DISTFILES = $(DIST_COMMON) $(DIST_SOURCES) $(TEXINFOS) $(EXTRA_DIST)
ACLOCAL = ${SHELL} /home/becker/async/WCET_gitlab/simulavr-mod/missing --run aclocal-1.11
AMTAR = $${TAR-tar}
AR = ar
AUTOCONF = ${SHELL} /home/becker/async/WCET_gitlab/simulavr-mod/missing --run autoconf
AUTOHEADER = ${SHELL} /home/becker/async/WCET_gitlab/simulavr-mod/missing --run autoheader
AUTOMAKE = ${SHELL} /home/becker/async/WCET_gitlab/simulavr-mod/missing --run automake-1.11
AVR_AS = avr-as
AVR_GCC = avr-gcc
AVR_GXX = avr-g++
AVR_LD = avr-ld
AVR_TCL_INCLUDE = 
AVR_TCL_LIB = 
AVR_TCL_MODULE_SUFFIX = 
AWK = gawk
CC = gcc
CCDEPMODE = depmode=gcc3
CFLAGS = -g -O2
CPP = gcc -E
CPPFLAGS = 
CXX = g++
CXXCPP = g++ -E
CXXDEPMODE = depmode=gcc3
CXXFLAGS = -g -O2
CXX_PROG = yes
CYGPATH_W = echo
DEFS = -DHAVE_CONFIG_H
DEPDIR = .deps
DISTCHECK_CONFIGURE_FLAGS = 
DLLTOOL = false
DOLLAR_SIGN = $
DOXYGEN_PAPER_SIZE = 
DSYMUTIL = 
DUMPBIN = 
DX_CONFIG = simulavr.doxy
DX_DOCDIR = doxygen-doc
DX_DOT = 
DX_DOXYGEN = 
DX_DVIPS = 
DX_EGREP = 
DX_ENV =  SRCDIR='.' PROJECT='Simulavr' DOCDIR='doxygen-doc' VERSION='1.1dev' HAVE_DOT='NO' GENERATE_MAN='NO' GENERATE_RTF='NO' GENERATE_XML='NO' GENERATE_HTMLHELP='NO' GENERATE_CHI='NO' GENERATE_HTML='NO' GENERATE_LATEX='NO'
DX_FLAG_chi = 0
DX_FLAG_chm = 0
DX_FLAG_doc = 0
DX_FLAG_dot = 0
DX_FLAG_html = 0
DX_FLAG_man = 0
DX_FLAG_pdf = 0
DX_FLAG_ps = 0
DX_FLAG_rtf = 0
DX_FLAG_xml = 0
DX_HHC = 
DX_LATEX = 
DX_MAKEINDEX = 
DX_PDFLATEX = 
DX_PERL = 
DX_PROJECT = Simulavr
ECHO_C = 
ECHO_N = -n
ECHO_T = 
EGREP = /bin/grep -E
EXEEXT = 
EXTRA_LIBS = 
FGREP = /bin/grep -F
GNUTIME = yes
GREP = /bin/grep
GTKWAVE = 
INSTALL = /usr/bin/install -c
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_PROGRAM = ${INSTALL}
INSTALL_SCRIPT = ${INSTALL}
INSTALL_STRIP_PROGRAM = $(install_sh) -c -s
IVERILOG = 
LD = /usr/bin/ld -m elf_x86_64
LDFLAGS = 
LIBOBJS = 
LIBS = 
LIBTOOL = $(SHELL) $(top_builddir)/libtool
LIBWSOCK_FLAGS = 
LIBZ_FLAGS = 
LIPO = 
LN_S = ln -s
LTLIBOBJS = 
MAINT = #
MAKEINFO = ${SHELL} /home/becker/async/WCET_gitlab/simulavr-mod/missing --run makeinfo
MAKEINFO_PROG = yes
MANIFEST_TOOL = :
MKDIR_P = /bin/mkdir -p
NM = /usr/bin/nm -B
NMEDIT = 
OBJDUMP = objdump
OBJEXT = o
OTOOL = 
OTOOL64 = 
PACKAGE = simulavr
PACKAGE_BUGREPORT = simulavr-devel@nongnu.org
PACKAGE_NAME = SimulAVR
PACKAGE_STRING = SimulAVR 1.1dev
PACKAGE_TARNAME = simulavr
PACKAGE_URL = 
PACKAGE_VERSION = 1.1dev
PATH_SEPARATOR = :
PYTHON = /usr/bin/python
PYTHON_EXECPREFIX = 
PYTHON_EXEC_PREFIX = ${exec_prefix}
PYTHON_MODULE_EXTENSION = .so
PYTHON_PLATFORM = linux2
PYTHON_PREFIX = ${prefix}
PYTHON_VERSION = 2.7
RANLIB = ranlib
SED = /bin/sed
SET_MAKE = 
SHELL = /bin/bash
STRIP = strip
SWIG = echo "Error: SWIG is not installed. You should look at http://www.swig.org" ; false -c++
SWIG_LIB = 
TCL_SHELL = /usr/bin/tclsh
TCL_WISH = /usr/bin/wish
TEX = tex
VERSION = 1.1-cache
VVP = 
abs_builddir = /home/becker/async/WCET_gitlab/simulavr-mod/regress/gtest
abs_srcdir = /home/becker/async/WCET_gitlab/simulavr-mod/regress/gtest
abs_top_builddir = /home/becker/async/WCET_gitlab/simulavr-mod
abs_top_srcdir = /home/becker/async/WCET_gitlab/simulavr-mod
ac_ct_AR = ar
ac_ct_CC = gcc
ac_ct_CXX = g++
ac_ct_DUMPBIN = 
am__include = include
am__leading_dot = .
am__quote = 
am__tar = $${TAR-tar} chof - "$$tardir"
am__untar = $${TAR-tar} xf -
bindir = ${exec_prefix}/bin
build = x86_64-unknown-linux-gnu
build_alias = 
build_cpu = x86_64
build_os = linux-gnu
build_vendor = unknown
builddir = .
datadir = ${datarootdir}
datarootdir = ${prefix}/share
docdir = ${datarootdir}/doc/${PACKAGE_TARNAME}
dvidir = ${docdir}
exec_prefix = ${prefix}
host = x86_64-unknown-linux-gnu
host_alias = 
host_cpu = x86_64
host_os = linux-gnu
host_vendor = unknown
htmldir = ${docdir}
includedir = ${prefix}/include
infodir = ${datarootdir}/info
install_sh = ${SHELL} /home/becker/async/WCET_gitlab/simulavr-mod/install-sh
libdir = ${exec_prefix}/lib
libexecdir = ${exec_prefix}/libexec
localedir = ${datarootdir}/locale
localstatedir = ${prefix}/var
mandir = ${datarootdir}/man
mkdir_p = /bin/mkdir -p
oldincludedir = /usr/include
pdfdir = ${docdir}
pkgpyexecdir = ${pyexecdir}/simulavr
pkgpythondir = ${pythondir}/simulavr
prefix = /usr/local
program_transform_name = s,x,x,
psdir = ${docdir}
pyexecdir = ${exec_prefix}/lib/python2.7/dist-packages
pythondir = ${prefix}/lib/python2.7/dist-packages
sbindir = ${exec_prefix}/sbin
sharedstatedir = ${prefix}/com
srcdir = .
sysconfdir = ${prefix}/etc
target_alias = 
top_build_prefix = ../../
top_builddir = ../..
top_srcdir = ../..

# google test library
GTEST_OBJS = gtest-1.6.0/src/gtest-all.cc
GTEST_INCLUDE = -Igtest-1.6.0/include/gtest -Igtest-1.6.0/include -Igtest-1.6.0
GTEST_CXXFLAGS = -Dprivate=public -Dprotected=public 
GTEST_EXTRA_FILES = gtest-1.6.0/src/gtest-internal-inl.h \
                    gtest-1.6.0/src/gtest-filepath.cc \
                    gtest-1.6.0/src/gtest.cc \
                    gtest-1.6.0/src/gtest-port.cc \
                    gtest-1.6.0/src/gtest-printers.cc \
                    gtest-1.6.0/src/gtest-test-part.cc \
                    gtest-1.6.0/src/gtest-death-test.cc \
                    gtest-1.6.0/src/gtest_main.cc \
                    gtest-1.6.0/src/gtest-typed-test.cc \
                    gtest-1.6.0/CONTRIBUTORS \
                    gtest-1.6.0/CHANGES \
                    gtest-1.6.0/README \
                    gtest-1.6.0/COPYING \
                    gtest-1.6.0/include/gtest/gtest-printers.h \
                    gtest-1.6.0/include/gtest/gtest-param-test.h \
                    gtest-1.6.0/include/gtest/gtest_pred_impl.h \
                    gtest-1.6.0/include/gtest/gtest-death-test.h \
                    gtest-1.6.0/include/gtest/gtest_prod.h \
                    gtest-1.6.0/include/gtest/gtest.h \
                    gtest-1.6.0/include/gtest/gtest-spi.h \
                    gtest-1.6.0/include/gtest/gtest-typed-test.h \
                    gtest-1.6.0/include/gtest/gtest-test-part.h \
                    gtest-1.6.0/include/gtest/gtest-message.h \
                    gtest-1.6.0/include/gtest/internal/gtest-filepath.h \
                    gtest-1.6.0/include/gtest/internal/gtest-string.h \
                    gtest-1.6.0/include/gtest/internal/gtest-linked_ptr.h \
                    gtest-1.6.0/include/gtest/internal/gtest-port.h \
                    gtest-1.6.0/include/gtest/internal/gtest-type-util.h \
                    gtest-1.6.0/include/gtest/internal/gtest-death-test-internal.h \
                    gtest-1.6.0/include/gtest/internal/gtest-param-util-generated.h \
                    gtest-1.6.0/include/gtest/internal/gtest-tuple.h \
                    gtest-1.6.0/include/gtest/internal/gtest-internal.h \
                    gtest-1.6.0/include/gtest/internal/gtest-param-util.h


# simulavr bindings
SIMULAVR_PATH = ../..
SIMULAVR_INCLUDE = -I$(SIMULAVR_PATH)/src
SIMULAVR_LIB = $(SIMULAVR_PATH)/src/.libs/libsim.la

# design under test settings
OBJS_UNITTEST = session_001/unittest001.cpp \
                session_irq_check/unittest_irq.cpp \
                session_io_pin/unittest_io_pin.cpp \
                gtest_main.cpp


# target sources (needed for make dist), if you change this list, you have to change OBJS_TARGET too!
OBJS_SRC = session_001/avr_code.s \
           session_irq_check/check.s \
           session_irq_check/tc1.s \
           session_irq_check/tc2.s \
           session_irq_check/tc3.s \
           session_io_pin/tc1.s


# target objects (needed for test), if you change this list, you have to change OBJS_SRC too!
OBJS_TARGET = session_001/avr_code.atmega32.o \
              session_irq_check/check.atmega32.o \
              session_irq_check/tc1.atmega32.o \
              session_irq_check/tc2.atmega32.o \
              session_irq_check/tc3.atmega32.o \
              session_io_pin/tc1.atmega128.o

AM_CXXFLAGS = $(GTEST_CXXFLAGS) $(GTEST_INCLUDE) $(SIMULAVR_INCLUDE) -g
EXTRA_DIST = $(OBJS_SRC) $(GTEST_EXTRA_FILES)
SUFFIXES = .c .s
CLEANFILES = */*.o
dut_SOURCES = $(OBJS_UNITTEST) $(GTEST_OBJS)
dut_LDADD = -lpthread $(SIMULAVR_LIB) $(LIBZ_FLAGS) $(EXTRA_LIBS)
dut_DEPENDENCIES = $(SIMULAVR_LIB)
all: all-am

.SUFFIXES:
.SUFFIXES: .c .s .cc .cpp .lo .o .obj
$(srcdir)/Makefile.in: # $(srcdir)/Makefile.am  $(am__configure_deps)
	@for dep in $?; do \
	  case '$(am__configure_deps)' in \
	    *$$dep*) \
	      ( cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh ) \
	        && { if test -f $@; then exit 0; else break; fi; }; \
	      exit 1;; \
	  esac; \
	done; \
	echo ' cd $(top_srcdir) && $(AUTOMAKE) --foreign regress/gtest/Makefile'; \
	$(am__cd) $(top_srcdir) && \
	  $(AUTOMAKE) --foreign regress/gtest/Makefile
.PRECIOUS: Makefile
Makefile: $(srcdir)/Makefile.in $(top_builddir)/config.status
	@case '$?' in \
	  *config.status*) \
	    cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh;; \
	  *) \
	    echo ' cd $(top_builddir) && $(SHELL) ./config.status $(subdir)/$@ $(am__depfiles_maybe)'; \
	    cd $(top_builddir) && $(SHELL) ./config.status $(subdir)/$@ $(am__depfiles_maybe);; \
	esac;

$(top_builddir)/config.status: $(top_srcdir)/configure $(CONFIG_STATUS_DEPENDENCIES)
	cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh

$(top_srcdir)/configure: # $(am__configure_deps)
	cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh
$(ACLOCAL_M4): # $(am__aclocal_m4_deps)
	cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh
$(am__aclocal_m4_deps):

clean-noinstPROGRAMS:
	@list='$(noinst_PROGRAMS)'; test -n "$$list" || exit 0; \
	echo " rm -f" $$list; \
	rm -f $$list || exit $$?; \
	test -n "$(EXEEXT)" || exit 0; \
	list=`for p in $$list; do echo "$$p"; done | sed 's/$(EXEEXT)$$//'`; \
	echo " rm -f" $$list; \
	rm -f $$list
session_001/$(am__dirstamp):
	@$(MKDIR_P) session_001
	@: > session_001/$(am__dirstamp)
session_001/$(DEPDIR)/$(am__dirstamp):
	@$(MKDIR_P) session_001/$(DEPDIR)
	@: > session_001/$(DEPDIR)/$(am__dirstamp)
session_001/unittest001.$(OBJEXT): session_001/$(am__dirstamp) \
	session_001/$(DEPDIR)/$(am__dirstamp)
session_irq_check/$(am__dirstamp):
	@$(MKDIR_P) session_irq_check
	@: > session_irq_check/$(am__dirstamp)
session_irq_check/$(DEPDIR)/$(am__dirstamp):
	@$(MKDIR_P) session_irq_check/$(DEPDIR)
	@: > session_irq_check/$(DEPDIR)/$(am__dirstamp)
session_irq_check/unittest_irq.$(OBJEXT):  \
	session_irq_check/$(am__dirstamp) \
	session_irq_check/$(DEPDIR)/$(am__dirstamp)
session_io_pin/$(am__dirstamp):
	@$(MKDIR_P) session_io_pin
	@: > session_io_pin/$(am__dirstamp)
session_io_pin/$(DEPDIR)/$(am__dirstamp):
	@$(MKDIR_P) session_io_pin/$(DEPDIR)
	@: > session_io_pin/$(DEPDIR)/$(am__dirstamp)
session_io_pin/unittest_io_pin.$(OBJEXT):  \
	session_io_pin/$(am__dirstamp) \
	session_io_pin/$(DEPDIR)/$(am__dirstamp)
gtest-1.6.0/src/$(am__dirstamp):
	@$(MKDIR_P) gtest-1.6.0/src
	@: > gtest-1.6.0/src/$(am__dirstamp)
gtest-1.6.0/src/$(DEPDIR)/$(am__dirstamp):
	@$(MKDIR_P) gtest-1.6.0/src/$(DEPDIR)
	@: > gtest-1.6.0/src/$(DEPDIR)/$(am__dirstamp)
gtest-1.6.0/src/gtest-all.$(OBJEXT): gtest-1.6.0/src/$(am__dirstamp) \
	gtest-1.6.0/src/$(DEPDIR)/$(am__dirstamp)
dut$(EXEEXT): $(dut_OBJECTS) $(dut_DEPENDENCIES) $(EXTRA_dut_DEPENDENCIES) 
	@rm -f dut$(EXEEXT)
	$(CXXLINK) $(dut_OBJECTS) $(dut_LDADD) $(LIBS)

mostlyclean-compile:
	-rm -f *.$(OBJEXT)
	-rm -f gtest-1.6.0/src/gtest-all.$(OBJEXT)
	-rm -f session_001/unittest001.$(OBJEXT)
	-rm -f session_io_pin/unittest_io_pin.$(OBJEXT)
	-rm -f session_irq_check/unittest_irq.$(OBJEXT)

distclean-compile:
	-rm -f *.tab.c

include ./$(DEPDIR)/gtest_main.Po
include gtest-1.6.0/src/$(DEPDIR)/gtest-all.Po
include session_001/$(DEPDIR)/unittest001.Po
include session_io_pin/$(DEPDIR)/unittest_io_pin.Po
include session_irq_check/$(DEPDIR)/unittest_irq.Po

.cc.o:
	depbase=`echo $@ | sed 's|[^/]*$$|$(DEPDIR)/&|;s|\.o$$||'`;\
	$(CXXCOMPILE) -MT $@ -MD -MP -MF $$depbase.Tpo -c -o $@ $< &&\
	$(am__mv) $$depbase.Tpo $$depbase.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CXXDEPMODE) $(depcomp) \
#	$(CXXCOMPILE) -c -o $@ $<

.cc.obj:
	depbase=`echo $@ | sed 's|[^/]*$$|$(DEPDIR)/&|;s|\.obj$$||'`;\
	$(CXXCOMPILE) -MT $@ -MD -MP -MF $$depbase.Tpo -c -o $@ `$(CYGPATH_W) '$<'` &&\
	$(am__mv) $$depbase.Tpo $$depbase.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CXXDEPMODE) $(depcomp) \
#	$(CXXCOMPILE) -c -o $@ `$(CYGPATH_W) '$<'`

.cc.lo:
	depbase=`echo $@ | sed 's|[^/]*$$|$(DEPDIR)/&|;s|\.lo$$||'`;\
	$(LTCXXCOMPILE) -MT $@ -MD -MP -MF $$depbase.Tpo -c -o $@ $< &&\
	$(am__mv) $$depbase.Tpo $$depbase.Plo
#	source='$<' object='$@' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CXXDEPMODE) $(depcomp) \
#	$(LTCXXCOMPILE) -c -o $@ $<

.cpp.o:
	depbase=`echo $@ | sed 's|[^/]*$$|$(DEPDIR)/&|;s|\.o$$||'`;\
	$(CXXCOMPILE) -MT $@ -MD -MP -MF $$depbase.Tpo -c -o $@ $< &&\
	$(am__mv) $$depbase.Tpo $$depbase.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CXXDEPMODE) $(depcomp) \
#	$(CXXCOMPILE) -c -o $@ $<

.cpp.obj:
	depbase=`echo $@ | sed 's|[^/]*$$|$(DEPDIR)/&|;s|\.obj$$||'`;\
	$(CXXCOMPILE) -MT $@ -MD -MP -MF $$depbase.Tpo -c -o $@ `$(CYGPATH_W) '$<'` &&\
	$(am__mv) $$depbase.Tpo $$depbase.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CXXDEPMODE) $(depcomp) \
#	$(CXXCOMPILE) -c -o $@ `$(CYGPATH_W) '$<'`

.cpp.lo:
	depbase=`echo $@ | sed 's|[^/]*$$|$(DEPDIR)/&|;s|\.lo$$||'`;\
	$(LTCXXCOMPILE) -MT $@ -MD -MP -MF $$depbase.Tpo -c -o $@ $< &&\
	$(am__mv) $$depbase.Tpo $$depbase.Plo
#	source='$<' object='$@' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CXXDEPMODE) $(depcomp) \
#	$(LTCXXCOMPILE) -c -o $@ $<

mostlyclean-libtool:
	-rm -f *.lo

clean-libtool:
	-rm -rf .libs _libs

ID: $(HEADERS) $(SOURCES) $(LISP) $(TAGS_FILES)
	list='$(SOURCES) $(HEADERS) $(LISP) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '{ files[$$0] = 1; nonempty = 1; } \
	      END { if (nonempty) { for (i in files) print i; }; }'`; \
	mkid -fID $$unique
tags: TAGS

TAGS:  $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	set x; \
	here=`pwd`; \
	list='$(SOURCES) $(HEADERS)  $(LISP) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '{ files[$$0] = 1; nonempty = 1; } \
	      END { if (nonempty) { for (i in files) print i; }; }'`; \
	shift; \
	if test -z "$(ETAGS_ARGS)$$*$$unique"; then :; else \
	  test -n "$$unique" || unique=$$empty_fix; \
	  if test $$# -gt 0; then \
	    $(ETAGS) $(ETAGSFLAGS) $(AM_ETAGSFLAGS) $(ETAGS_ARGS) \
	      "$$@" $$unique; \
	  else \
	    $(ETAGS) $(ETAGSFLAGS) $(AM_ETAGSFLAGS) $(ETAGS_ARGS) \
	      $$unique; \
	  fi; \
	fi
ctags: CTAGS
CTAGS:  $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	list='$(SOURCES) $(HEADERS)  $(LISP) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '{ files[$$0] = 1; nonempty = 1; } \
	      END { if (nonempty) { for (i in files) print i; }; }'`; \
	test -z "$(CTAGS_ARGS)$$unique" \
	  || $(CTAGS) $(CTAGSFLAGS) $(AM_CTAGSFLAGS) $(CTAGS_ARGS) \
	     $$unique

GTAGS:
	here=`$(am__cd) $(top_builddir) && pwd` \
	  && $(am__cd) $(top_srcdir) \
	  && gtags -i $(GTAGS_ARGS) "$$here"

distclean-tags:
	-rm -f TAGS ID GTAGS GRTAGS GSYMS GPATH tags

distdir: $(DISTFILES)
	@srcdirstrip=`echo "$(srcdir)" | sed 's/[].[^$$\\*]/\\\\&/g'`; \
	topsrcdirstrip=`echo "$(top_srcdir)" | sed 's/[].[^$$\\*]/\\\\&/g'`; \
	list='$(DISTFILES)'; \
	  dist_files=`for file in $$list; do echo $$file; done | \
	  sed -e "s|^$$srcdirstrip/||;t" \
	      -e "s|^$$topsrcdirstrip/|$(top_builddir)/|;t"`; \
	case $$dist_files in \
	  */*) $(MKDIR_P) `echo "$$dist_files" | \
			   sed '/\//!d;s|^|$(distdir)/|;s,/[^/]*$$,,' | \
			   sort -u` ;; \
	esac; \
	for file in $$dist_files; do \
	  if test -f $$file || test -d $$file; then d=.; else d=$(srcdir); fi; \
	  if test -d $$d/$$file; then \
	    dir=`echo "/$$file" | sed -e 's,/[^/]*$$,,'`; \
	    if test -d "$(distdir)/$$file"; then \
	      find "$(distdir)/$$file" -type d ! -perm -700 -exec chmod u+rwx {} \;; \
	    fi; \
	    if test -d $(srcdir)/$$file && test $$d != $(srcdir); then \
	      cp -fpR $(srcdir)/$$file "$(distdir)$$dir" || exit 1; \
	      find "$(distdir)/$$file" -type d ! -perm -700 -exec chmod u+rwx {} \;; \
	    fi; \
	    cp -fpR $$d/$$file "$(distdir)$$dir" || exit 1; \
	  else \
	    test -f "$(distdir)/$$file" \
	    || cp -p $$d/$$file "$(distdir)/$$file" \
	    || exit 1; \
	  fi; \
	done
check-am: all-am
	$(MAKE) $(AM_MAKEFLAGS) check-local
check: check-am
all-am: Makefile $(PROGRAMS)
installdirs:
install: install-am
install-exec: install-exec-am
install-data: install-data-am
uninstall: uninstall-am

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am

installcheck: installcheck-am
install-strip:
	if test -z '$(STRIP)'; then \
	  $(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	    install_sh_PROGRAM="$(INSTALL_STRIP_PROGRAM)" INSTALL_STRIP_FLAG=-s \
	      install; \
	else \
	  $(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	    install_sh_PROGRAM="$(INSTALL_STRIP_PROGRAM)" INSTALL_STRIP_FLAG=-s \
	    "INSTALL_PROGRAM_ENV=STRIPPROG='$(STRIP)'" install; \
	fi
mostlyclean-generic:

clean-generic:
	-test -z "$(CLEANFILES)" || rm -f $(CLEANFILES)

distclean-generic:
	-test -z "$(CONFIG_CLEAN_FILES)" || rm -f $(CONFIG_CLEAN_FILES)
	-test . = "$(srcdir)" || test -z "$(CONFIG_CLEAN_VPATH_FILES)" || rm -f $(CONFIG_CLEAN_VPATH_FILES)
	-rm -f gtest-1.6.0/src/$(DEPDIR)/$(am__dirstamp)
	-rm -f gtest-1.6.0/src/$(am__dirstamp)
	-rm -f session_001/$(DEPDIR)/$(am__dirstamp)
	-rm -f session_001/$(am__dirstamp)
	-rm -f session_io_pin/$(DEPDIR)/$(am__dirstamp)
	-rm -f session_io_pin/$(am__dirstamp)
	-rm -f session_irq_check/$(DEPDIR)/$(am__dirstamp)
	-rm -f session_irq_check/$(am__dirstamp)

maintainer-clean-generic:
	@echo "This command is intended for maintainers to use"
	@echo "it deletes files that may require special tools to rebuild."
clean: clean-am

clean-am: clean-generic clean-libtool clean-noinstPROGRAMS \
	mostlyclean-am

distclean: distclean-am
	-rm -rf ./$(DEPDIR) gtest-1.6.0/src/$(DEPDIR) session_001/$(DEPDIR) session_io_pin/$(DEPDIR) session_irq_check/$(DEPDIR)
	-rm -f Makefile
distclean-am: clean-am distclean-compile distclean-generic \
	distclean-tags

dvi: dvi-am

dvi-am:

html: html-am

html-am:

info: info-am

info-am:

install-data-am:

install-dvi: install-dvi-am

install-dvi-am:

install-exec-am:

install-html: install-html-am

install-html-am:

install-info: install-info-am

install-info-am:

install-man:

install-pdf: install-pdf-am

install-pdf-am:

install-ps: install-ps-am

install-ps-am:

installcheck-am:

maintainer-clean: maintainer-clean-am
	-rm -rf ./$(DEPDIR) gtest-1.6.0/src/$(DEPDIR) session_001/$(DEPDIR) session_io_pin/$(DEPDIR) session_irq_check/$(DEPDIR)
	-rm -f Makefile
maintainer-clean-am: distclean-am maintainer-clean-generic

mostlyclean: mostlyclean-am

mostlyclean-am: mostlyclean-compile mostlyclean-generic \
	mostlyclean-libtool

pdf: pdf-am

pdf-am:

ps: ps-am

ps-am:

uninstall-am:

.MAKE: check-am install-am install-strip

.PHONY: CTAGS GTAGS all all-am check check-am check-local clean \
	clean-generic clean-libtool clean-noinstPROGRAMS ctags \
	distclean distclean-compile distclean-generic \
	distclean-libtool distclean-tags distdir dvi dvi-am html \
	html-am info info-am install install-am install-data \
	install-data-am install-dvi install-dvi-am install-exec \
	install-exec-am install-html install-html-am install-info \
	install-info-am install-man install-pdf install-pdf-am \
	install-ps install-ps-am install-strip installcheck \
	installcheck-am installdirs maintainer-clean \
	maintainer-clean-generic mostlyclean mostlyclean-compile \
	mostlyclean-generic mostlyclean-libtool pdf pdf-am ps ps-am \
	tags uninstall uninstall-am


define build-asm-m32
avr-gcc -Wa,--gstabs,-D -xassembler-with-cpp -mmcu=atmega32 $< -o $@
endef

define build-c-m32
avr-gcc -mmcu=atmega32 -O2 $< -o $@
endef

define build-asm-m128
avr-gcc -Wa,--gstabs,-D -xassembler-with-cpp -mmcu=atmega128 $< -o $@
endef

session_001/avr_code.atmega32.o: session_001/avr_code.s
	$(build-asm-m32)

session_irq_check/check.atmega32.o: session_irq_check/check.s
	$(build-asm-m32)

session_irq_check/tc1.atmega32.o: session_irq_check/tc1.s
	$(build-asm-m32)

session_irq_check/tc2.atmega32.o: session_irq_check/tc2.s
	$(build-asm-m32)

session_irq_check/tc3.atmega32.o: session_irq_check/tc3.s
	$(build-asm-m32)

session_io_pin/tc1.atmega128.o: session_io_pin/tc1.s
	$(build-asm-m128)

check-local: dut $(OBJS_TARGET)
	./dut
#check-local:
#	@echo "  Configure could not find AVR cross compiling environment so gtest"
#	@echo "  can not be run."

# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
