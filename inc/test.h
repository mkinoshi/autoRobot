//
// Created by mjmartin on 4/14/17.
//

#ifndef ROBOT_TEST_H
#define ROBOT_TEST_H

#include "map.h"
#include "particle.h"

Particle* testProbabilities(float*,float*,int,Map*,Pixel*);

void testPosition(Particle* p, float* angles, float *sensor,int N, Map *map,Pixel*);

#endif //ROBOT_TEST_H
