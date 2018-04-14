CC=gcc
CXX=g++
FL=-c -std=c99
SW=-w
BIN=../bin
CFLAGS=-I. -I/usr/include/hokuyoaist-3/ -I/usr/include/flexiport-2 -I/usr/local/include/hokuyoaist-3/ -I/usr/local/include/flexiport-2 -I../inc -O2
CXXFLAGS=$(CFLAGS)
LDFLAGS=-L/usr/local/lib/gearbox
LDLIBS=-lflexiport -lhokuyoaist -lm -lMage -lpthread -lipc -lgcm

URGserver: socketUtil.cpp URGserver.cpp
	$(CXX) -o $(BIN)/$@ $^ $(CFLAGS) $(LDFLAGS) $(LDLIBS)

URGquit: socketUtil.cpp URGquit.cpp
	$(CXX) -o $(BIN)/$@ $^ $(CFLAGS) $(LDFLAGS) $(LDLIBS)

nav: nav.c utils.c movements.c socketUtil.cpp map.c ppmIO.c particle.c test.c histogram.c
	$(CXX) $(SW) -o $(BIN)/$@ $^ $(CFLAGS) $(LDFLAGS) $(LDLIBS)

stand_alone: stand_alone.c
	$(CXX) -o $(BIN)/$@ $^ $(CFLAGS) $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o *~ URGserver URGquit
