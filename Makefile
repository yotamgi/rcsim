CXXFLAGS = -O3 -Wall -Werror -g
LDFLAGS = -rdynamic -g $(CXXFLAGS)

# target specific settings
LDFLAGS += -lIrrlicht

all: main

run: main
	./main

heli.o: heli.cc
	$(CXX) -c $(CXXFLAGS) heli.cc -o heli.o

main.o: main.cc
	$(CXX) -c $(CXXFLAGS) main.cc -o main.o

main: main.o heli.o
	$(CXX) main.o heli.o -o main $(LDFLAGS) 

clean:
	rm -f main.o heli.o main

.PHONY: all clean
