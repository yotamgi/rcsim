CXXFLAGS = -O3 -Wall -Werror -g
LDFLAGS = -rdynamic -g $(CXXFLAGS)

# target specific settings
LDFLAGS += -lIrrlicht

all: rcsim

run: rcsim
	./rcsim

heli.o: heli.cc
	$(CXX) -c $(CXXFLAGS) heli.cc -o heli.o

main.o: main.cc
	$(CXX) -c $(CXXFLAGS) main.cc -o main.o

rcsim: main.o heli.o
	$(CXX) main.o heli.o -o rcsim $(LDFLAGS) 

clean:
	rm -f rcsim.o heli.o rcsim

.PHONY: all clean
