/**
 * Matt Martin, Makoto Kinoshita
 * Feb 26, 2017
 * CS363 Robotics
 * Robot header file and declaration of struct
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <SVM_VisionModule.h>
#include "particle.h"
#include <Mage.h>

typedef enum {
    TEST,
    IDLE,
    POINT_FOLLOW,
    WALL_FIND,
    WALL_FOLLOW,
    WALL_ECORNER,
    WALL_ICORNER,
    MAZE,
    ROTATE,
    WANDER,
    GOTO,
    PINK_FIND,
    PINK_TRACK,
    HOUGH_WALL_FOLLOW,
    FACE_FIND,
    FACE_TRACK,
    TAKE_PICTURE,
    LOCALIZE,
    WAIT_FOR_FACE,
    FOLLOW_PERSON,
    QUIT
} ROBOT_TASK;


#define NUM_VSENSORS 8
#define NUM_LASERS 341
#define DEGREES_PER_LASER 240/341
#define LASERS_PER_DEGREE 341/240
#define OG_RES 50
#define OG_SIZE 4000

#define ROBOT_RADIUS_MM 150

#define MAX_TRANS_SPEED 400
#define MAX_ROT_SPEED 500
#define MAX_TRANS_ACCEL 50
#define MAX_ROT_ACCEL 80
#define TRANS_SIGMA 0.1
#define ROT_SIGMA 0.5
#define LASER_SIGMA 0.1



typedef struct PointStruct {
    int x;
    int y;
} Point;


typedef struct LineStruct {
    int votes;
    int theta;
    int rho;
} Line;

typedef struct RobotStruct {
    //current v and w commands
    int vt;
    int wt;
    int iterations;

    long State[NUM_STATE];
    ROBOT_TASK task;
    long *vSensors;
    int *laser;
    Point *laserXY;

    int goalOrientation;
    int goalDistance;
    int goalPosition[2];
    // fields for POINT_FOLLOW state tracking
    int *pfPoints;
    int pfIndex;
    int nPfPoints;

    int orientationPiToPi;

    int laserIdxClosestFlat;
    // making a robot to see only one side
    int laserSide;

    // hough circle fields
    float houghRadius;
    int houghBuffer;
    Point houghCircleCenter;

    // hough line fields
//    int *houghLineDirection;
//    int *houghLineDistance;
    Line *houghBestLines;

    // svm command field
    int svm_operator;
    SVM_Operator_Data *SVMData;

    // field necessary to track face
    int face_track_counter;
    int face_track_x_location;
    int face_track_y_location;

    int connectToIPC;
    int IPCLostCount;

    Point *occupancyGrid;
    int occupancyGridSize;

    ParticleFilter *particleFilter;
    float *particleLasers;
    float * particleLaserAngles;
    Map *map;
    Pixel *pfImage;
    struct timeval time;
    int particle_iter;

    int timeSinceLastFace;

    //change of svm command
    int svm_command_change;
    int svm_previous_operator;
    int histogramsMade;
} Robot;

#endif
