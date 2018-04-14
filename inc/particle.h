/**
 * Created by Bruce Maxwell
 * Edited by Matt Martin, Makoto Kinoshita
 * Apr 3, 2017
 * CS363 Robotics
 * Particle filter header file
 */

#ifndef PARTICLE_H
#define PARTICLE_H

#include "robot.h"
#include "map.h"
#include "ppmIO.h"

#define N_PARTICLES 5000

typedef struct PStruct {
    float x;
    float y;
    float t;
    double pi;
} Particle;

typedef struct PFStruct {
    Particle *p;     // particle array
    int N;           // number of active particles
    int MaxN;        // size of the particle array
    float pRandom;   // % of new random particles to create each iteration

    // robot noise
    float sigma_v;   // trans velocity noise (m/s)
    float sigma_w;   // angle velocity noise (rad/s)

    // sensor noise and data
    float dxlaser,dylaser; // x and y offsets for the laser relative to the robot
    float sigma_s;   // sensor noise (m)
    int nReadings; // number of sensor readings to use when assigning probs

} ParticleFilter;

void pf_create(ParticleFilter *,int nSensors, float pRandom);
void pf_free(ParticleFilter *pf);
void pf_init(ParticleFilter *pf, Map *map);
Particle pf_createRandomParticle(Map*map);
void pf_motion( Particle *p, float v, float w, double dt, float sigma_v, float sigma_w );

double pf_sensorModel( float actual_sensor, float expected_sensor, float sigma );
float pf_calcExpected( Particle *p, float angle, Map *map, Pixel *test );
void pf_sensor( ParticleFilter *pf, Particle *p, float* angles,float *sensor, Map *map );
void pf_resample( ParticleFilter *pf, int N ,Map*map);
int pf_iterate( ParticleFilter *pf, float v, float w, double dt, float* angles,float *sensor, Map *map,Pixel *img, int iter );

Particle pf_getBestParticle(ParticleFilter *pf);
void pf_normPis(Particle *particles,int N);


#endif
