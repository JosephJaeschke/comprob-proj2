all:
	g++ -c quaternion.cpp -o quaternion.o
	g++ piano.cpp quaternion.o -o piano
clean:
	rm quaternion.o
	rm piano
