#ifndef SCENE
#define SCENE

# include <vector> 
# include "Vector3d.h" 
# include "Particle.h" 

class Scene {
 public:
  vector<Particle*> particles;
  bool gravity;
  vector<Spring*> springs;
  vector<Box*> boxes;
  Scene(const Scene& scene);
  Scene();
  
  Vector3d computeSimpleForces(Particle *particle);
  std::vector<Vector3d> computeForces();
  void forwardEulersUpdate (double dT);
    void backwardEulersUpdate (double dT);
    void midPointUpdate (double dT);
    void rK4Update (double dT);
    double adaptiveSolver (double dT, void (Scene::*solver) (double));
  
};

Scene::Scene () {
}

Scene::Scene(const Scene& scene) {
  for (int i = 0; i < scene.particles.size(); i++) 
    particles.push_back(new Particle(*scene.particles[i]));
  
  gravity = scene.gravity;
  springs = scene.springs;
  boxes = scene.boxes;
}




#endif
