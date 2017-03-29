#==============================================================================
# Makefile
#==============================================================================

#Name of the package
PKG = 

# ---- compiler options ----
OS = ${shell uname}
CC = g++
LD = g++
CFLAGS += -Wall -fexceptions -fno-builtin -D_REENTRANT -O2 -fpic -c
LDFLAGS = -fexceptions -O2
INCLUDEHEADERS = -I ../ -I /usr/local/include -I /usr/include/SDL \
                 -I /opt/include -I /usr/local/include -I /usr/qrts/include \
                 -I /usr/local/include/FeatureTracker -I /usr/qrts/include/FeatureTracker \
                 -I /usr/include/ffmpeg -I /usr/qrts/include/ffmpeg
INCLUDELIBS = -L ../ -L /usr/local/lib -L /opt/lib -L /usr/qrts/lib \
	          -lklt -lFeatureTracker -lputils -lSDL -lSDL_gfx
	           

ifeq ($(OS),QNX)
 CFLAGS += -DNTO
 INCLUDEHEADERS += -I ../pxc200/qnx
 INCLUDELIBS += -L ../pxc200/qnx -L /usr/qrts/lib -L /usr/local/lib -lpxc200 -lsocket
 FFMPEGLIBS = /usr/qrts/lib/libavformat.a /usr/qrts/lib/libavcodec.a
 OPENCVLIBS = /usr/qrts/lib/libcv.a /usr/qrts/lib/libcvaux.a /usr/qrts/lib/libcxcore.a
else 
 INCLUDELIBS += 	-lpthread -lrt -ldl -lm -lz
 FFMPEGLIBS = -lavutil -lavformat -lavcodec
 OPENCVLIBS = -lcv -lcvaux -lcxcore 
endif

SRC = TrackVideoFeatures.t.cpp FeatureTrackerKLT.t.cpp FeatureServer.t.cpp \
      FeatureTrackerOCV.t.cpp FeatureClient.t.cpp SDLWindow.t.cpp Pixmap.t.cpp
ifeq ($(OS),QNX)
	SRC += PXCCaptureLoop.t.cpp tracker01.cpp
endif

OBJ = $(SRC:.cpp=.o)
TARGET = TrackVideoFeatures.t FeatureTrackerKLT.t FeatureTrackerOCV.t \
         FeatureServer.t FeatureClient.t SDLWindow.t Pixmap.t
         
ifeq ($(OS),QNX)
	TARGET += PXCCaptureLoop.t tracker01 
endif
CLEAN = rm -rf *.o lib* *.dat *.t.ppm $(TARGET)


# ========== Targets ==========
targets: $(TARGET)

# ----- obj -----
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDEHEADERS)
	
PXCCaptureLoop.t: PXCCaptureLoop.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS)

TrackVideoFeatures.t: TrackVideoFeatures.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(FFMPEGLIBS) $(OPENCVLIBS)

FeatureTrackerKLT.t: FeatureTrackerKLT.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS)

FeatureTrackerOCV.t: FeatureTrackerOCV.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS)

FeatureServer.t: FeatureServer.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS)

tracker01: tracker01.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS) 

FeatureClient.t: FeatureClient.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS)

SDLWindow.t: SDLWindow.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS)

Pixmap.t: Pixmap.t.o
	$(CC) $(LDFLAGS) $< -o $@ $(INCLUDELIBS) $(OPENCVLIBS)

clean:
	$(CLEAN)

