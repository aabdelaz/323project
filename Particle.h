//
//  Particle.h
//  COS323FinalProject
//
//  Created by Daniel Arias on 1/7/14.
//  Copyright (c) 2014 __MyCompanyName__. All rights reserved.
//

#ifndef COS323FinalProject_Particle_h
#define COS323FinalProject_Particle_h

# include "Vector3d.h"
# include <vector>

using namespace std;

struct Spring;
struct Particle;
struct Box;

struct Particle {
    int id;
    double mass;
    Vector3d pos;
    Vector3d vel;
    double drag;
    std::vector<Spring*> springs;
    Particle(const Particle& p): mass(p.mass), pos(p.pos), vel(p.vel), drag(p.drag), springs(p.springs) {}
    Particle() {}
};

struct Spring {
    std::vector<Particle*> friends;
    double restLength;
    double kSpring;
    double kDamp;
};

struct Box {
    double sideLength;
    Vector3d center;
};



#endif
