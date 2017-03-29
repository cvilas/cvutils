#======================================================================== 
# Package	: Realtime Feature Tracking Library
# ----------------------------------------------------------------------  
# File: makefile
#========================================================================  


#Name of the package
PKG = FeatureTracker

# ----- Directories -----
INSTALLDIR = /usr/local
INSTALLHEADERPATH= $(INSTALLDIR)/include/
INSTALLLIBPATH= $(INSTALLDIR)/lib
INSTALLBINPATH =
INSTALLBSRCPATH = 

# ----- Doxygen documentation parameters -----
DOCNAME = Realtime Feature Tracking Library
DOCSOURCE = *.hpp
DOCTARGET = 

# Libraries, headers, and binaries that will be installed.
OS = ${shell uname}

LIBS = lib$(PKG).so lib$(PKG).a
HDRS = PXCCaptureLoop.hpp TrackerUtils.hpp FeatureTrackerKLT.hpp FeatureTrackerOCV.hpp \
       FeatureClientServer.hpp Pixmap.hpp
#SRC = *.cpp

# ---- compiler options ----
CC = g++
LD = g++
CFLAGS += -W -Wall -fexceptions -fno-builtin -O2 -fpic -D_REENTRANT
LDFLAGS = 
INCLUDEHEADERS = -I ./ -I /usr/include/SDL -I /usr/local/include -I /opt/include/SDL
INCLUDELIBS = 
OBJ = Pixmap.o TrackerUtils.o FeatureTrackerKLT.o FeatureTrackerOCV.o FeatureClientServer.o
ifeq ($(OS),QNX)
 CFLAGS += -DNTO -DQRTS
 OBJ += PXCCaptureLoop.o
 INCLUDEHEADERS += -I /usr/qrts/include -I ./pxc200/qnx
endif
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
	$(CC) $(CFLAGS) -c $< $(INCLUDEHEADERS)

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

