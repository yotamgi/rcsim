CXXFLAGS = -O3 -Wall -Werror -g
LDFLAGS = -rdynamic -g $(CXXFLAGS)

# target specific settings
LDFLAGS += -lIrrlicht

all: rcsim

run: rcsim
	./rcsim

OBJS = smooth_rand.o heli.o main.o rotor_blur.o arrow.o flight_controller.o controls.o dashboard.o 

dashboard.o: dashboard.cc
	$(CXX) -c $(CXXFLAGS) dashboard.cc -o dashboard.o

controls.o: controls.cc
	$(CXX) -c $(CXXFLAGS) controls.cc -o controls.o

flight_controller.o: flight_controller.cc
	$(CXX) -c $(CXXFLAGS) flight_controller.cc -o flight_controller.o

arrow.o: arrow.cc
	$(CXX) -c $(CXXFLAGS) arrow.cc -o arrow.o

rotor_blur.o: rotor_blur.cc
	$(CXX) -c $(CXXFLAGS) rotor_blur.cc -o rotor_blur.o

smooth_rand.o: smooth_rand.cc
	$(CXX) -c $(CXXFLAGS) smooth_rand.cc -o smooth_rand.o

heli.o: heli.cc
	$(CXX) -c $(CXXFLAGS) heli.cc -o heli.o

main.o: main.cc
	$(CXX) -c $(CXXFLAGS) main.cc -o main.o

rcsim: ${OBJS}
	$(CXX) ${OBJS} -o rcsim $(LDFLAGS) 

clean:
	rm -f ${OBJS} rcsim

.PHONY: all clean
