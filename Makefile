all: Particle

Particle: Particle.o Vector3d.o
	g++ Particle.o Vector3d.o -o Particle

Particle.o: Particle.cpp Particle.h Vector3d.h Scene.h
	g++ -c Particle.cpp

Vector3d.o: Vector3d.cpp Vector3d.h
	g++ -c Vector3d.cpp

clean:
	rm -f *.o 


