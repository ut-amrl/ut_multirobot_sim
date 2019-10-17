INCDIR = -I. -I../../shared/util
DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(DBG) $(OPT) $(INCDIR)
LINK   = -lm -lrt

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

all: dt

dt: dt.cpp ../../shared/util/timer.cc ../../shared/util/timer.h
	$(CPP) $(CFLAGS) -o dt dt.cpp ../../shared/util/timer.cc $(LINK)

clean:
	/bin/rm -f dt *.o output.*

clean-all: clean
	/bin/rm -f *~

test: dt
	./dt input.pbm output.pgm
	/usr/bin/qiv input.pbm output.pgm


