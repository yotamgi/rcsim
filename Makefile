CXXFLAGS = -O3 -Wall -Werror -g
LDFLAGS = -rdynamic -g $(CXXFLAGS)

# target specific settings
LDFLAGS += -lIrrlicht

all: rcsim

run: rcsim
	./rcsim

OBJS = smooth_rand.o heli.o main.o rotor_blur.o arrow.o controller.o controls_view.o

controls_view.o: controls_view.cc
	$(CXX) -c $(CXXFLAGS) controls_view.cc -o controls_view.o

controller.o: controller.cc
	$(CXX) -c $(CXXFLAGS) controller.cc -o controller.o

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
