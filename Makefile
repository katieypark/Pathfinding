all: sim
sim:
	g++ -o proj3 main.cpp Robot.cpp Project3.cpp
clean:
	rm proj3
