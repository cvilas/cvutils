#==============================================================================
# Makefile
#==============================================================================

#Name of the package
PKG = 

# ---- compiler options ----
os=linux
CC = g++
LD = g++
CFLAGS += -Wall -fexceptions -fno-builtin -D_REENTRANT -O2 -fpic -c
LDFLAGS = -fexceptions -O2 -o
INCLUDEHEADERS = -I ../ -I /usr/local/include -I /usr/local/include/QMath \
                 -I /usr/qrts/include/ -I /usr/qrts/include/Homography 
INCLUDELIBS = -L ../ -L /usr/local/lib -L /usr/qrts/lib/ -lHomography \
              -lgsl -lgslcblas -lQMath -lQMathGsl -lm

OBJ = 
TARGET = decomposeHomography.t Homography.t
CLEAN = rm -rf *.o lib* *.dat $(TARGET)


# ========== Targets ==========
targets: $(TARGET)

.cpp.o:
	$(CC) $(CFLAGS) -c $< $(INCLUDEHEADERS)

# ----- decomposeHomography -----
decomposeHomography.t: decomposeHomography.t.o
	rm -f decomposeHomography.t
	$(LD) -o decomposeHomography.t decomposeHomography.t.o $(INCLUDELIBS)

# ----- Homography -----
Homography.t: Homography.t.o
	rm -f Homography.t
	$(LD) -o Homography.t Homography.t.o $(INCLUDELIBS)

clean:
	$(CLEAN)

