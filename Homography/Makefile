#======================================================================== 
# Package	: Homography Library
# ----------------------------------------------------------------------  
# File: makefile
#========================================================================  


#Name of the package
PKG = Homography

# ----- Directories -----
INSTALLDIR = /usr/local
INSTALLHEADERPATH= $(INSTALLDIR)/include/
INSTALLLIBPATH= $(INSTALLDIR)/lib
INSTALLBINPATH =
INSTALLBSRCPATH = 

# ----- Doxygen documentation parameters -----
DOCNAME = Computer Vision Utilities
DOCSOURCE = *.hpp
DOCTARGET = 

# Libraries, headers, and binaries that will be installed.
LIBS = lib$(PKG).so lib$(PKG).a
HDRS = Homography.hpp HomographyUtilities.hpp
#SRC = *.cpp

# ---- compiler options ----
CC = g++
LD = g++
CFLAGS += -W -Wall -fexceptions -fno-builtin -O2 -fpic -D_REENTRANT -c
LDFLAGS = 
INCLUDEHEADERS = -I ./ -I /usr/local/include/QMath -I /usr/local/include 
INCLUDELIBS = 
OBJ = HomographyUtilities.o Homography.o 
TARGET = $(LIBS)
CLEAN = rm -rf *.o *.dat $(TARGET)


# ========== Targets ==========
targets: $(TARGET)

# ----- lib -----
lib$(PKG).a: $(OBJ)
	ar cr $@ $(OBJ)
	ranlib $@

lib$(PKG).so: $(OBJ)
	$(LD) -shared -o $@ $(OBJ)

# ----- obj -----
.cpp.o:
	$(CC) $(CFLAGS) $< $(INCLUDEHEADERS)

# ---- make rules ----
clean:
	@echo
	@echo ----- Package $(PKG), Cleaning -----
	@echo
	$(CLEAN)
	if (test -d examples) ; then (cd examples; make clean);fi

install:
	@echo
	@echo ----- Package $(PKG), Installing to $(INSTALLDIR) -----
	@echo
	if ! (test -d $(INSTALLLIBPATH)); then (mkdir $(INSTALLLIBPATH)); fi
	for i in ${LIBS}; do (cp $$i $(INSTALLLIBPATH)); done
	if ! (test -d $(INSTALLHEADERPATH)); then (mkdir $(INSTALLHEADERPATH)); fi
	if ! (test -d $(INSTALLHEADERPATH)$(PKG)); then (mkdir $(INSTALLHEADERPATH)$(PKG)); fi
	for i in ${HDRS}; do (cp $$i $(INSTALLHEADERPATH)$(PKG)); done

