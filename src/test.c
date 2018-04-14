//
// Created by mjmartin on 4/13/17.
//

#include <stdlib.h>
#include <math.h>
#include <utils.h>
#include <map.h>
#include <particle.h>
#include "test.h"

Particle* testProbabilities(float* angles,float* sensor,int N, Map *map,Pixel *img) {
    Particle *p = (Particle*)malloc(sizeof(Particle));
    p->x = 0.5;
    p->y = 2.5;
    p->t = 1.8;
    p->pi = 0.1;
    testPosition(p,angles,sensor,N,map,img);
    return (p);
}

void testPosition(Particle* p, float* angles, float *sensor,int N, Map *map,Pixel*img) {
    float z;
    double pr;
    int x0,y0;
    double probProd = 1.0;
    // calculate the start location
    map_xy2pix( map, p->x, p->y, &x0, &y0 );

    if(map_getraw( map, x0, y0 ) < 250 ) { // invalid location for particle
        probProd = 0.0000001;

    } else {
        for (int j = 0; j < N; ++j) {
            z = pf_calcExpected(p, angles[j], map, img);
            if (z == 0.0 && sensor[j] == 0) { // max range
                pr = .99;
            } else if (z == 0.0) {
                pr = 0.01;
            } else {
                pr = pf_sensorModel(sensor[j] / 1000.0, z, LASER_SIGMA);
            }
            printf("prob:%f,z:%f,sens:%f,angle:%f\n",pr,z,sensor[j]/1000.0,angles[j]);
            probProd *= pr < 0.0001 ? 0.0001 : pr;
        }
    }
    printf("%e\n",probProd);
}