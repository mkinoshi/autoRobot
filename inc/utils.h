/**
 * Matt Martin, Makoto Kinoshita
 * Feb 26, 2017
 * CS363 Robotics
 * Utility header file
 */

#ifndef UTILS_H
#define UTILS_H

#include "robot.h"


#define PI 3.14159265


Robot * robotCreate();

void robotInit(Robot* robot);


int handleCommandLine(int argc, char const* argv[], Robot *robot);

void sighandler(int signal);

void getAllIRReadings(long* State, int* readings);

void getAllSonarReadings(long* State, int* readings);

void getVirtualSensorReadings(Robot *robot);

void getLaserSensorReadings(Robot *robot);

void findClosestFlatSurface(Robot *robot);

void updatePFSensors(Robot*);

void writeHoughAccumulator(int** accum, int width, int height);

void houghLine(Robot * robot, int lines);

void houghCircle(Robot *robot);

void convertLaserToXY(Robot *robot);

double gaussDist(void);

void buildOccupancyGrid(Robot *robot);

void findLineSegCircleIntersection(Point center, double radius, Point p0, Point p1, Point* intersection);

void convertOrientation(Robot *robot);

void limitVMCommands(Robot *robot,int *vt, int *wt);

void stopForObstacles(Robot *robot,int*vt,int*wt);

float getPValue(float zscore);

void writePFToImage(ParticleFilter *particleFilter,Particle,Map*map,Pixel*pfImage,int N);

void freeRobot(Robot *robot);

#endif
