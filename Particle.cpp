//  Project
// 
# include <stdio.h>
# include "Particle.h"
# include "Scene.h" 
# include "Vector3d.h"
# define ITERATIONS 50
# define DT 1
# define GRAVITY -9.8
# define G 6.67e-11


Vector3d Scene::computeSimpleForces(Particle *particle) {
   
    Vector3d force(0, 0, 0);
   
   // compute gravity and drag forces 
   if (gravity == true) {
       force.setCoords(force.X(), force.Y() + particle->mass*GRAVITY, force.Z()); 
   
       force = force + particle->drag*particle->vel*(1/particle->mass);
   }

   return force;

}


std::vector<Vector3d> Scene::computeForces() {
  std::vector<Vector3d> forces;
    
  // add force std::vector per particle
  for (int i = 0; i < particles.size(); i++) 
    forces.push_back(computeSimpleForces(particles[i]));
    
    // compute spring forces
    
    // for each spring, add to the force std::vector corresponding to the 
    // particles connected to it 
    for (int i = 0; i < springs.size(); i++) {
      Particle *p1 = springs[i]->friends[0];
      Particle *p2 = springs[i]->friends[1];
        
      Vector3d dVector = p2->pos - p1->pos;
      double d = dVector.length();
      double r = springs[i]->restLength;
      double ks = springs[i]->kSpring;
      double kd = springs[i]->kDamp;
      Vector3d v1 = p1->vel;
      Vector3d v2 = p2->vel;
        
      Vector3d f1 = (ks*(d - r) + kd*((v2 - v1).dot(dVector)))*dVector;
      Vector3d f2 = -1*f1;
        
      forces[p1->id] += f1;
      forces[p2->id] += f2;      
    }
    
    if (gravity == false) {
      // compute gravitational forces between particles
      for (int i = 0; i < particles.size(); i++) {
	    double m1 = particles[i]->mass; 
	    for (int j = 0; j < particles.size(); j++) {
	      if (i != j) {
	        double m2 = particles[j]->mass;
	        Vector3d r = particles[j]->pos - particles[i]->pos;
	        forces[i] += r*G*m1*m2/r.lsquared();
	      } 
	    } 
      }  
    } 
    
    return forces;
      
}

void Scene::forwardEulersUpdate (double dT) {

  // Initialize Vectors
  std::vector<Vector3d> forces = computeForces();

  for (int i = 0; i < particles.size(); i++) {
    // Position Update
    particles[i]->pos = particles[i]->pos + dT * particles[i]->vel;
        
    // Velocity Update
    Vector3d accel = forces[i] / particles[i]->mass;
    
    particles[i]->vel = particles[i]->vel + dT * accel;
    
  }

}

void Scene::backwardEulersUpdate (double dT) {
    
    // initialize a changeable substitute for the vector of particles
    std::vector<Particle*> standIn;
    for (int i = 0; i < particles.size(); i++) standIn.push_back(particles[i]);
    
    // compute a forward euler step to find state at t + 1
    forwardEulersUpdate(dT);
    
    // compute forces at t + 1
    std::vector<Vector3d> dVel = computeForces();
    std::vector<Vector3d> dPos;
    for (int i = 0; i < standIn.size(); i++) dPos.push_back(standIn[i]->vel);
    
    // compute acceleration, velocity change and position change in t + 1
    for (int i = 0; i < particles.size(); i++) {
        dVel[i] = dT * dVel[i] / particles[i]->mass;
        dPos[i] = dT * dPos[i];
    }
    
    //update scene velocity and position step based on original position plus vel & pos at t + 1
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->vel = particles[i]->vel + dVel[i];
        particles[i]->pos = particles[i]->pos + dPos[i];
    }
}

void Scene::midPointUpdate (double dT) {
    
    std::vector<Particle*> copy;
    for (int i = 0; i < particles.size(); i++) copy.push_back(new Particle(*particles[i]));
    
    // compute a
    std::vector<Vector3d> a1;
    for (int i = 0; i < particles.size(); i++) a1.push_back(particles[i]->vel);
    
    std::vector<Vector3d> a2 = computeForces();
    
    for (int i = 0; i < particles.size(); i++) {
        a1[i] = a1[i]*dT;
        a2[i] = a2[i]*dT/particles[i]->mass;
    }
    
    // alter copy
    for (int i = 0; i < particles.size(); i++) {
        copy[i]->pos = copy[i]->pos + (0.5*a1[i]);
        copy[i]->vel = copy[i]->vel + (0.5*a2[i]);
    }
    
    // compute b
    std::vector<Vector3d> b1;
    for (int i = 0; i < particles.size(); i++) b1.push_back(copy[i]->vel);
    
    std::vector<Vector3d> b2 = computeForces();
    
    for (int i = 0; i < particles.size(); i++) {
        b1[i] = b1[i]*dT;
        b2[i] = b2[i]*dT/particles[i]->mass;
        particles[i]->pos += b1[i];
        particles[i]->vel += b2[i];
        delete copy[i];
    }
    
}

void Scene::rK4Update (double dT) {
    
    // initialize a changeable substitute for the vector of particles
    std::vector<Particle*> standIn;
    for (int i = 0; i < particles.size(); i++) standIn.push_back(new Particle(*particles[i]));
    
    // initialize acceleration, velocity vectors for A
    std::vector<Vector3d> a1;
    for (int i = 0; i < particles.size(); i++) a1.push_back(standIn[i]->vel);
    std::vector<Vector3d> a2 = computeForces();
    
    
    // find A and update standIn
    for (int i = 0; i < standIn.size(); i++) {
        a1[i] = a1[i] * dT;
        a2[i] = dT * a2[i] / standIn[i]->mass;
        
        // find RK4 positions and velocities for B
        standIn[i]->pos = .5 * a1[i] + particles[i]->pos;
        standIn[i]->vel = .5 * a2[i] + particles[i]->vel;
    }
    
    // initialize acceleration, velocity, and position vectors for B
    std::vector<Vector3d> b1;
    for (int i = 0; i < particles.size(); i++) b1.push_back(standIn[i]->vel);
    std::vector<Vector3d> b2 = computeForces();
    
    // find B and update standIn
    for (int i = 0; i < standIn.size(); i++) {
        b1[i] = b1[i] * dT;
        b2[i] = dT * b2[i] / standIn[i]->mass;
        
        // find RK4 positions and velocities for C
        standIn[i]->pos = .5 * b1[i] + particles[i]->pos;
        standIn[i]->vel = .5 * b2[i] + particles[i]->vel;
    }
    
    // initialize acceleration, velocity, vectors for C
    std::vector<Vector3d> c1;
    for (int i = 0; i < particles.size(); i++) c1.push_back(standIn[i]->vel);
    std::vector<Vector3d> c2 = computeForces();
    
    // find C and update standIn
    for (int i = 0; i < standIn.size(); i++) {
        c1[i] = c1[i] * dT;
        c2[i] = dT * c2[i] / standIn[i]->mass;
        
        // find RK4 positions and velocities for D
        standIn[i]->pos = c1[i] + particles[i]->pos;
        standIn[i]->vel = c2[i] + particles[i]->vel;
    }
    
    // initialize acceleration, velocity, and position vectors for D
    std::vector<Vector3d> d1;
    for (int i = 0; i < particles.size(); i++) d1.push_back(standIn[i]->vel);
    std::vector<Vector3d> d2 = computeForces();
    
    // find D
    for (int i = 0; i < standIn.size(); i++) {
        d1[i] = d1[i] * dT;
        d2[i] = dT * d2[i] / standIn[i]->mass;
        
    }
    
    // update poitions and velocities in particle based on Runge-Kutta 4 formula
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->pos = particles[i]->pos + (1/6) * (2 * (b1[i] + c1[i]) +
                                                         a1[i] + d1[i]);
        
        particles[i]->vel = particles[i]->vel + (1/6) * (2 * (b2[i] + c2[i]) +
                                                         a2[i] + d2[i]);
        delete standIn[i];
    }
}

double Scene::adaptiveSolver (double dT, void (Scene::*solver) (double)) {

  // initialize variables
  double delta = 0;           // error term
  int callNum = 1;            // number of times euler is called for each scene
  double threshold = 1e-3;    // error threshold

  // the previous scenestep    
  Scene *copyThis = new Scene(*this);
  // the next scene stescenep    
  Scene *copyNext = new Scene(*this);

  Scene *finalCopy = new Scene(*this);
  do {
    // call euler with previous time step and call number
    copyThis = new Scene(*this);
    copyNext = new Scene(*this);
    for (int i = 0; i < callNum; i++) {
      (copyThis->*solver)(dT);
    }

    // half time step and double calls
    dT = dT / 2;
    callNum = callNum * 2;

    // call euler with new time step and new call number
    for (int i = 0; i < callNum; i++) {
      (copyNext->*solver)(dT);
    }

    delta = 0;
    // determine error by finding the absolute value of the length of differences
    
    for (int i = 0; i < particles.size(); i++) {
      Vector3d diff = copyThis->particles[i]->pos - copyNext->particles[i]->pos;
      delta += diff.length();
      // printf("delta is %f\n\n", delta);
    }        

    // delete if delta is not small enough and you need to start over
    if (threshold < delta) {
      delete copyNext;
    }
    delete copyThis;
  } while (threshold < delta); 

  for (int i = 0; i < this->particles.size(); i++) {
    this->particles[i] = copyNext->particles[i];
  }
  return dT;
}

int main (int argc, char **argv) {
  // Read in Scene
  //Scene sampleScene = ReadScene(argv[0]);
    
  Scene *sampleScene = new Scene(); // don't change this
  sampleScene->particles.push_back(new Particle);
  sampleScene->particles[0]->pos = Vector3d(1, 2, 3.73);
  sampleScene->particles[0]->vel = Vector3d(4, 1, 7);
  sampleScene->particles[0]->drag = 1;
  sampleScene->particles[0]->mass = 5;
  sampleScene->particles[0]->id = 0;

  sampleScene->gravity = true;
  
  // Define constants
  double fEulerMin = 1e10;
  double bEulerMin = 1e10;
  double midpointMin = 1e10;
  double rk4Min = 1e10;
  double fEulerAverage = 0;
  double bEulerAverage = 0;
  double midpointAverage = 0;
  double rk4Average = 0;
  
  Scene sceneCopy(*sampleScene); // sceneCopy is our workspace
  // Run iterations of adaptive Forward Euler's method
  for (int i = 0; i < ITERATIONS; i++)
  {
    double fEulerDT = sceneCopy.adaptiveSolver(DT, &Scene::forwardEulersUpdate);
    
    fEulerAverage += fEulerDT;
    if (fEulerDT < fEulerMin) {
      fEulerMin = fEulerDT;
    }
        
    // Print minimum step size of the given iteration
    printf("Forward Euler DT: %f\n", fEulerDT);
  }

  // Compute average min step size for adaptive Euler's method
  fEulerAverage = fEulerAverage / ITERATIONS;
  printf("Forward Euler: \n");
  printf("Min step size: %f\n", fEulerMin);	 // min across all iterations
  printf("Average step size: %f\n\n", fEulerAverage); // average across all iterations

  // return sceneCopy to the state of sample scene
  for (int i = 0; i < sampleScene->particles.size() ;i++) {
    sceneCopy.particles[i] = sampleScene->particles[i];
  }
  
  // Run iterations of adaptive Backward Euler's method
  printf("Backward Euler: \n");
  for (int i = 0; i < ITERATIONS; i++) {
    double bEulerDT = sceneCopy.adaptiveSolver(DT, &Scene::backwardEulersUpdate);
    
    bEulerAverage += bEulerDT;
    if (bEulerDT < bEulerMin)  {
      bEulerMin = bEulerDT;
    }
    
    // Print minimum step size of the given iteration
    printf("Backward Euler DT: %f\n", bEulerDT);
  }
  // Compute average min step size for adaptive Euler's method
  bEulerAverage = bEulerAverage / ITERATIONS;
  printf("Min step size: %f\n", bEulerMin);
  printf("Average step size: %f\n", bEulerAverage);


  // return sceneCopy to the state of sample scene
  for (int i = 0; i < sampleScene->particles.size() ;i++) {
    sceneCopy.particles[i] = sampleScene->particles[i];
  }
  
  printf("Midpoint: \n");
  for (int i = 0; i < ITERATIONS; i++) {
    double midpointDT = sceneCopy.adaptiveSolver(DT,
						  &Scene::midPointUpdate);
    
    midpointAverage += midpointDT;
    if (midpointDT < midpointMin) {
      midpointMin = midpointDT;
    }
    
    // Print minimum step size of the given iteration
    printf("Midpoint DT: %f\n", midpointDT);
  }
  // Compute average min step size for adaptive Euler's method
  midpointAverage = midpointAverage / ITERATIONS;
  printf("Min step size: %f\n", midpointMin);
  printf("Average step size: %f\n\n", midpointAverage);
  
  // return sceneCopy to the state of sample scene
  for (int i = 0; i < sampleScene->particles.size() ;i++) {
    sceneCopy.particles[i] = sampleScene->particles[i];
  }
  
  for (int i = 0; i < ITERATIONS; i++) {
    double rk4DT = sceneCopy.adaptiveSolver(DT, &Scene::rK4Update);
    
    rk4Average += rk4DT;
    if (rk4DT < rk4Min) {
      rk4Min = rk4DT;
    }
    
    // Print minimum step size of the given iteration
    // printf("%f\n", rk4DT);
  }

  // Compute average min step size for adaptive Euler's method
  rk4Average = rk4Average / ITERATIONS;
  printf("Min step size: %f\n", rk4Min);
  printf("Average step size: %f\n", rk4Average);

  return 0;
}

