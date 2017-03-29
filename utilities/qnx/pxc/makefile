#======================================================================== 
# Project: OCTOR
# ---------------------------------------------------------------------- 
# Package: PXCCont
# Authors: Vilas Kumar Chitrakaran
# Start Date: Tue Sep 30 09:30:10 EDT 2003
# ----------------------------------------------------------------------  
# File: makefile
#========================================================================  

include $(QRTS)/makefiles/endusertop

include $(QRTS)/makefiles/qwidgets

ifeq ($(SYSNAME), nto)
	DEFAULTTARGET = gcc_ntox86
else
	DEFAULTTARGET = default
endif

ifeq ($(release), release)
	PHABOPT = DEBUG=
endif

BINS = pxcdisplay pxcTrigCapture

# ========== Targets ==========

targets: $(BINS)

PXCCont.o:	PXCCont.cpp PXCCont.hpp
	$(CC) $(CFLAGS) PXCCont.cpp -I ../include

pxcTrigCapture.o:	pxcTrigCapture.cpp
	$(CC) $(CFLAGS) pxcTrigCapture.cpp -I ../include -I ../../../packages/qnx/include/ -I /usr/qrts/include

pxcDisplay.o: pxcDisplay.cpp
	$(CC) $(CFLAGS) pxcDisplay.cpp -I ../../../packages/qnx/include/ -I /usr/qrts/include

pxcdisplay:	pxcDisplay.o PXCCont.o PXCCont.hpp
	$(LDC) -o pxcdisplay pxcDisplay.o PXCCont.o -L /usr/qrts/lib -L /usr/local/lib -L ../lib -lpxc200 $(QWIDGETSLIBS) -lStatus -ljpeg
	usemsg pxcdisplay pxcdisplay.use

pxcTrigCapture:	pxcTrigCapture.o
	$(LDC) -o pxcTrigCapture pxcTrigCapture.o -L /usr/qrts/lib -L /usr/local/lib -lpxc200
	usemsg pxcTrigCapture pxcTrigCapture.use

LOCALCLEAN = rm -f $(BINS) *.o *.err

include $(QRTS)/makefiles/enduserbottom
