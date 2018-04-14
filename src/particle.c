/**
 * Created by Bruce Maxwell
 * Edited by Matt Martin, Makoto Kinoshita
 * Apr 3, 2017
 * CS363 Robotics
 * Particle Filter file
 */

#include <stdlib.h>
#include <math.h>
#include <utils.h>
#include <map.h>
#include "map.h"


// initialize some fields of a ParticleFilter and return a pointer to it
void pf_create(ParticleFilter *pf,int nSensors,float pRandom) {
    printf("creating PF\n");
    pf->MaxN = N_PARTICLES;
    pf->N = N_PARTICLES;
    pf->sigma_w = ROT_SIGMA;
    pf->sigma_v = TRANS_SIGMA;
    pf->sigma_s = LASER_SIGMA;

    pf->pRandom = pRandom;
    pf->nReadings = nSensors;

    pf->p = (Particle*)malloc(sizeof(Particle) * pf->MaxN);
    pf->dxlaser = 22.0;
    pf->dylaser = 0.0;
}

// frees particleFilter if it has been allocated
void pf_free(ParticleFilter *particleFilter) {
    if (particleFilter) {
        if (particleFilter->p) {
            free(particleFilter->p);
        }
        free(particleFilter);
    }
}

// creates random particles throughout map for pf
void pf_init(ParticleFilter *pf, Map *map) {
    printf("initing PF\n");
    for (int i = 0; i < N_PARTICLES; i++) {
        pf->p[i] = pf_createRandomParticle(map);
    }
    printf("done init\n");
}

// create and return a Particle at random position in the map
Particle pf_createRandomParticle(Map *map) {
    Particle p;
    float x,y;
    x = ((double)rand()/(double)RAND_MAX)*map->size[0] + map->origin[0];
    y = ((double)rand()/(double)RAND_MAX)*map->size[1] + map->origin[1];
    while (map_get(map,x,y) < 250) { // random point isn't in a valid location
        x = ((double)rand()/(double)RAND_MAX)*map->size[0] + map->origin[0];
        y = ((double)rand()/(double)RAND_MAX)*map->size[1] + map->origin[1];
    }
    p.x = x;
    p.y = y;
    p.t = (float)(rand() % (int)(2000.0*PI))/(float)1000.0;
    p.pi = 1.0 / N_PARTICLES;
//    printf("creating: %f,%f,%f\n",p.x,p.y,p.t);
    return p;
}

int pf_iterate( ParticleFilter *pf, float v, float w, double dt, float* angles, float *sensor, Map *map, Pixel *img, int iter ) {
    printf("in pf iterate\n");
    for (int i = 0; i < pf->N; ++i) {
        // update particle i with motion model
        pf_motion(&(pf->p[i]),v,w,dt,pf->sigma_v,pf->sigma_w);

        // correct based on laser readings
        pf_sensor(pf,&(pf->p[i]),angles,sensor,map);

//        printf("%f,%f,%f\n",pf->p[i].x,pf->p[i].y,pf->p[i].pi);
    }
//    Particle est = pf_getBestParticle(pf);
//    writePFToImage(pf,est,map,img,iter*2+1);
    pf_normPis(pf->p,pf->N);
    // resample states based on new probabilities
    pf_resample(pf,pf->N,map);
    //pf_normPis(pf->p,pf->N);

    return 1;
}

void pf_normPis(Particle *particles, int N) {
    double totalpi = 0.0;
    for (int i = 0; i < N; i++) {
        totalpi += particles[i].pi;
    }
    printf("total prob: %f\n",totalpi);
    for (int i = 0; i < N; i++) {
        particles[i].pi /= totalpi;
    }
}


void pf_resample( ParticleFilter *pf, int N , Map * map) {
    printf("in pf resample\n");
    double offset = (double) rand() / (double)RAND_MAX / (double)N;
    double psum = pf->p[0].pi;
    int i,count;
    i = count = 0;
    Particle *tempParticles = (Particle*)malloc(sizeof(Particle) * N);
    while ( count < N ) {
        // loop by new offset
        while (offset > psum) {
            i++;
            psum += pf->p[i].pi;
        }
        // replace
        if ((double)rand()/(double)RAND_MAX < pf->pRandom) {
            tempParticles[count] = pf_createRandomParticle(map);
        } else {
            tempParticles[count] = pf->p[i];
        }
        count++;
        offset += 1.0/N;
//        printf("%f,%f\n%d,%d\n",offset,psum,count,i);
    }
    memcpy(pf->p,tempParticles,sizeof(Particle) * N);
    free(tempParticles);
}


void pf_sensor( ParticleFilter *pf, Particle *p, float *angles, float *sensor, Map *map ) {
    float z;
    double pr = 0.0;
    int x0,y0;
    double probSum = 0.0;
    double probProd = 1.0;
    // calculate the start location
    map_xy2pix( map, p->x, p->y, &x0, &y0 );

    if(map_getraw( map, x0, y0 ) < 250 ) { // invalid location for particle
        probProd = 0.0;

    } else {
        for (int j = 0; j < pf->nReadings; ++j) {
            z = pf_calcExpected(p, angles[j], map, NULL);
            if (z == 0.0 && sensor[j] == 0) { // max range
                pr = pf_sensorModel(0.0,0.0, pf->sigma_s);
            } else if (z == 0.0) {
                pr = pf_sensorModel(sensor[j] / 1000.0, 4.0, pf->sigma_s);
            } else {
                pr = pf_sensorModel(sensor[j] / 1000.0, z, pf->sigma_s);
            }
//            printf("prob:%f,z:%f,sens:%f,angle:%f\n",pr,z,sensor[j]/1000.0,angles[j]);
            probProd *= pr;
        }
    }
    p->pi = probProd;
//    printf("pi: %e\n",p->pi);
}


// sensor model
// returns likelihod that the actual sensor reading matches the expected sensor reading
// uses a Gaussian centered on the expected sensor reading with stdev sigma
double pf_sensorModel( float actual_sensor, float expected_sensor, float sigma ) {
    const double divisor = sqrt( 2.0 * M_PI );
    const double diff = actual_sensor - expected_sensor;

    return( (1.0 / (sigma * divisor)) * exp( -0.5 * (diff*diff) / (sigma * sigma) ) );
}


void pf_motion(Particle *p, float v, float w, double dt, float sigma_v, float sigma_w ) {
    // update x,y,t of p based on parameters
    //printf("in pf motion\n");
    double randv = v/1000.0 + gaussDist() * sigma_v;
    double randw = w/1000.0 + gaussDist() * sigma_w;
    p->x += randv * dt * cos(p->t);
    p->y += randv * dt * sin(p->t);
    p->t += randw * dt;
}

// Calculate the expected value of the laser at the given angle (rad)
// at position p given a map the Pixel array test can be NULL.  If it
// is not NULL, then the laser line is drawn into the Pixel array,
// which can be saved as a PPM image.
//
// The Particle is expected to have x, y, and t (theta) fields
//
// The ParticleFilter is expected to have a dlaser field that
// specifies the position of the laser relative to the center of the
// robot.  On the Magellan's, this should be something like (x, y) = (22, 0)
// calc the expected value of the laser at the given angle (rad) at position p
// this needs fixing
float pf_calcExpected( Particle *p, float angle, Map *map, Pixel *test ) {
    int x0;
    int y0;
    float a;
    float dx;
    float dy;
    float tx, ty;
    int x, y;
    int i;
    const int maxDistance = (int)(4.0/0.05);

    // calculate the start location
    tx = p->x + 0.22 * cos(p->t) - 0.0 * sin(p->t);
    ty = p->y + 0.22 * sin(p->t) + 0.0 * cos(p->t);

//    printf("dlaser: %f,%f\n",pf->dxlaser,pf->dylaser);

    map_xy2pix( map, tx, ty, &x0, &y0 );

    // calculate the angle
    a = p->t + angle;

    // calculate the gradients, these are in pixel space on the map
    // use a negative angle to adjust from LHS to RHS on the map
    dx = cos(-a);
    dy = sin(-a);

    x = 0;
    y = 0;
    if( fabs(dx) > fabs(dy) ) { // step in x
        for(i=0;i <maxDistance;i++) {
            x = dx >= 0.0 ? i : -i;
            y = (int)((fabs(dy)/fabs(dx)) * i +  0.5);
            y = dy >= 0.0 ? y : -y;

            if( x0 +x < 0 || x0+x >= map->cols || y0+y < 0 || y0+y >= map->rows ) {
                return(0.0);
            }

            if(test != NULL) {
                test[(y0+y)*map->cols + (x0+x)].r = 255;
                test[(y0+y)*map->cols + (x0+x)].g = 10;
                test[(y0+y)*map->cols + (x0+x)].b = 10;
            }

            if( map_getraw( map, x0+x, y0+y ) < 20 ) { // obstacle
                return( sqrt( x*x + y*y ) * map->gridSize );
            }
        }
    }
    else { // step in y
        for(i=0;i<maxDistance;i++) {
            y = dy >= 0.0 ? i : -i;
            x = (int)((fabs(dx) / fabs(dy)) * i + 0.5);
            x = dx >= 0.0 ? x : -x;

            if( x0 +x < 0 || x0+x >= map->cols || y0+y < 0 || y0+y >= map->rows ) {
                return(0.0);
            }

            if(test != NULL) {
                test[(y0+y)*map->cols + (x0+x)].r = 255;
                test[(y0+y)*map->cols + (x0+x)].g = 10;
                test[(y0+y)*map->cols + (x0+x)].b = 10;
            }

            if( map_getraw( map, x0+x, y0+y ) < 20) { // obstacle
                return( sqrt( x*x + y*y ) * map->gridSize );
            }
        }
    }
    // no obstacle along the line so return 0
    return(0.0);
}


Particle pf_getBestParticle(ParticleFilter *pf) {
    double maxProb = 0.0;
    Particle best = pf->p[0];
    for (int i = 0; i < pf->N; ++i) {
        if (pf->p[i].pi > maxProb) {
            maxProb = pf->p[i].pi;
            best = pf->p[i];
        }
    }
    return best;
}
