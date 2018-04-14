/**
 * Matt Martin, Makoto Kinoshita
 * Mar 26, 2017
 * CS363 Robotics
 * utility function file
 */
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <SVM_VisionModule.h>
#include <map.h>
#include "Mage.h"
#include "socketUtil.h"
#include "utils.h"
#include "robot.h"
#include "particle.h"
#include "../inc/robot.h"

/**
 * Handles the command line arguments for all the different tasks that the robot can do
 * @param argc  number of arguments
 * @param argv  array of argguments
 * @param robot pointer for field correction
 * @return -1 if invalid command, 1 otherwise
 */
int handleCommandLine(int argc, char const* argv[], Robot *robot) {
    if (argc < 2) {
        printf("Usage: %s <type of movement> <other args...>\n"
                "  type:\n"
                "    -1 ->TEST\n"
                "    0 -> IDLE\n"
                "    1 -> POINT_FOLLOW\n"
                "    2 -> WALL_FOLLOW\n"
                "    3 -> MAZE\n"
                "    4 -> ROTATE\n"
                "    5 -> WANDER\n"
                "    6 -> GOTO\n"
                "    7 -> PINK_TRACK\n"
                "    8 -> HOUGH_WALL_FOLLOW\n"
                "    9 -> FACE_FIND\n"
                "   10 -> LOCALIZE\n"
                "   11 -> GET_TO_SECOND_FLOOR\n",
               argv[0] );

        freeRobot(robot);
        return(-1);
    }
    switch (atoi(argv[1])) {
        case -1:
            if (argc < 2) {
                freeRobot(robot);
                return -1;
            }
            robot->task = TEST;
            break;
        case 0:
            robot->task = IDLE;
            break;
        case 1:
            if (argc < 4) {
                printf("Usage: %s 1 <nPoints> <x> <y> <x> <y> <x> <y> ...\n"
                               "  nPoints:\n"
                               "    number of points to follow\n"
                               "  x y x y x,y,...:\n"
                               "    series of integer points, separated by"
                               "a space. A single x y  pair is one point.\n", argv[0] );
                freeRobot(robot);
                return(-1);
            }
            robot->task = POINT_FOLLOW;
            robot->pfIndex = 0;
            robot->nPfPoints = atoi(argv[2]);
            robot->pfPoints = (int *)malloc(sizeof(int) * robot->nPfPoints * 2);
            for (int i = 0; i < robot->nPfPoints * 2; ++i) {
                robot->pfPoints[i] = atoi(argv[3+i]);
            }
            robot->goalPosition[0] = robot->pfPoints[0];
            robot->goalPosition[1] = robot->pfPoints[1];
            break;
        case 2:
            if (argc < 3) {
                printf("Usage: %s 2 <buffer> <side> \n"
                               "  buffer:\n"
                               "    distance in mm to stay away from wall\n"
                               "  side:\n"
                               "    -1 = wall on left side of robot\n"
                               "    1  = wall on right side of robot\n" , argv[0] );
                freeRobot(robot);
                return(-1);
            }
            robot->task = WALL_FIND;
            robot->goalDistance = atoi(argv[2]);
            robot->laserSide = -1;
            if (atoi(argv[3]) == -1)
                robot->goalOrientation = 280;
            else if (atoi(argv[3]) == 1)
                robot->goalOrientation = 60;
            else {
                printf("Invalid side argument %s\n", argv[3]);
                return(-1);
            }
            break;
        case 3:
            if (argc < 5) {

                printf("Usage: %s 3 <buffer> \n"
                               "  buffer:\n"
                               "    distance in mm to stay away from wall\n" , argv[0] );
                freeRobot(robot);
                return(-1);
            }
            robot->task = MAZE;
            robot->goalDistance = atoi(argv[2]);
            robot->houghRadius = 57.15;
            robot->houghBuffer = 100;
            robot->laserSide = -1;
            break;

        case 4:
            if (argc < 3) {
                printf("Usage: %s 4 <angle>\n"
                               "  angle:\n"
                               "    angle in degrees to rotate from [-180,180]\n", argv[0] );
                freeRobot(robot);
                return(-1);
            }
            robot->task = ROTATE;
            robot->goalOrientation = (int)((float)atoi(argv[2]) * 1000.0 *  PI / 180.0);
            break;
        case 5:
            robot->task = WANDER;
            break;
        case 6:
            if (argc < 3) {
                printf("Usage: %s 6 <x> <y>\n"
                               "  x:\n"
                               "    x (in mm) positoin\n"
                               "  y:\n"
                               "    y (in mm) position\n", argv[0]);
                freeRobot(robot);
                return -1;
            }
            robot->task = GOTO;
            robot->goalPosition[0] = atoi(argv[2]);
            robot->goalPosition[1] = atoi(argv[3]);
            break;
        case 7:
            if (argc < 3) {
                printf("Usage: %s 7 <distance>\n"
                                  " distance:\n"
                                  "  distance in mm to stay away from pink object\n", argv[0]);
                freeRobot(robot);
                return -1;
            }
            robot->task = PINK_FIND;
            robot->svm_operator = SVM_OP_Pink_Blob;
            robot->svm_previous_operator = 0;
            robot->svm_command_change = 0;
            robot->goalDistance = atoi(argv[2]);
            robot->connectToIPC = 1;
            break;
        case 8:
            if (argc < 3) {
                printf("Usage: %s 8 <buffer> <side> \n"
                               "  buffer:\n"
                               "    distance in mm to stay away from wall (must be < 700)\n"
                               "  side:\n"
                               "    -1 = wall on left side of robot\n"
                               "    1  = wall on right side of robot\n" , argv[0] );
                freeRobot(robot);
                return(-1);
            }
            robot->task = HOUGH_WALL_FOLLOW;
            robot->goalDistance = atoi(argv[2]);
            robot->laserSide = -1;
            if (atoi(argv[3]) == -1)
                robot->goalOrientation = 1570;
            else if (atoi(argv[3]) == 1)
                robot->goalOrientation = -1570;
            else {
                printf("Invalid side argument %s\n", argv[3]);
                return(-1);
            }
            break;
        case 9:
            if (argc < 3) {
                printf("Usage: %s 9 <distance>\n"
                                  " distance:\n"
                                  "  distance in mm to stay away from face\n", argv[0]);
                freeRobot(robot);
                return -1;
            }
            robot->task = FACE_FIND;
            robot->svm_operator = 24;
            robot->goalDistance = atoi(argv[2]);
            robot->face_track_counter = 1;
            robot->face_track_x_location = 0;
            robot->face_track_y_location = 0;
            robot->svm_command_change = 0;
            robot->connectToIPC = 1;
            break;
        case 10:
            if (argc < 4) {
                printf("Usage: %s 10 <nLasers> <mapFile>\n"
                               " nLasers:\n"
                               "   number of laser sensors to sample from in particle filter\n"
                               "mapFile:\n"
                               "   pgm image file of the map to localize in\n", argv[0]);
                freeRobot(robot);
                return -1;
            }
            robot->task = LOCALIZE;
            robot->map = map_read((char*)argv[3]);
            robot->pfImage = (Pixel *)malloc(sizeof(Pixel) * robot->map->rows * robot->map->cols);
            robot->particle_iter = 0;
            pf_create(robot->particleFilter,atoi(argv[2]),0.2);
            pf_init(robot->particleFilter,robot->map);
            robot->particleLasers = (float*)malloc(sizeof(float) * robot->particleFilter->nReadings);
            robot->particleLaserAngles = (float*)malloc(sizeof(float) * robot->particleFilter->nReadings);
            break;
        case 11:
            if (argc < 3) {
                printf("Usage: %s 11 <distance>\n"
                               " distance:\n"
                               "   distance in mm to stay away from person when following\n", argv[0]);
                freeRobot(robot);
                return -1;
            }
            robot->task = WAIT_FOR_FACE;
            robot->svm_operator = SVM_OP_HaarFaceDetector;
            robot->svm_previous_operator = 0;
            robot->goalDistance = atoi(argv[2]);
            robot->connectToIPC = 1;
            robot->IPCLostCount = 0;
            robot->svm_command_change = 0;
            robot->histogramsMade = 0;
            break;
        default:
            printf("Invalid command: ");
            for (int j = 0; j < argc; ++j) {
                printf("%s ",argv[j]);
            }
            printf("\n");
            return(-1);
    }
    return 1;
}

/**
 * creates a pointer to a Robot struct and mallocs for fields that can be malloced now
 * call this whenever you want to get a new Robot
 * @return pointer to Robot
 */
Robot * robotCreate() {
    Robot *robot = (Robot*)malloc(sizeof(Robot));
    // create robot object and create important fields
    robot->vSensors = (long *) malloc(sizeof(long) * NUM_VSENSORS);
    robot->laser = (int *) malloc(sizeof(int) * NUM_LASERS);
    robot->laserXY = (Point *) malloc(sizeof(Point) * NUM_LASERS);
    robot->SVMData = (SVM_Operator_Data *) malloc(sizeof(SVM_Operator_Data));
    robot->occupancyGrid = (Point*)malloc(sizeof(Point) * OG_SIZE * OG_SIZE);
    robot->particleFilter = (ParticleFilter*)malloc(sizeof(ParticleFilter));

    return robot;
}

/**
 * initializes fields of robot so default values
 * call this after calling robotCreate()
 * @param robot pointer to robot
 */
void robotInit(Robot* robot) {
    // give all fields appropriate initial values
    robot->vt = 0;
    robot->wt = 0;
    robot->iterations = 0;

    robot->task = IDLE;
    robot->goalDistance = 100;
    robot->goalOrientation = 0;
    robot->goalPosition[0] = 0;
    robot->goalPosition[1] = 0;

    robot->pfIndex = 0;
    robot->nPfPoints = 0;

    robot->orientationPiToPi = 0;
    robot->laserIdxClosestFlat = 170;
    robot->laserSide = 0;

    robot->houghRadius = 0;
    robot->houghBuffer = 100;
    robot->houghCircleCenter.x = 0;
    robot->houghCircleCenter.y = 0;

    robot->svm_operator = NULL;
    robot->face_track_counter = 0;
    robot->face_track_x_location = 0;
    robot->face_track_y_location = 0;

    robot->connectToIPC = 0;
    robot->IPCLostCount = 0;

    robot->occupancyGridSize = 0;
    gettimeofday(&(robot->time),NULL);
    robot->particle_iter = 0;

    robot->timeSinceLastFace = 0;

    for (int i = 0; i < NUM_LASERS; ++i) {
        robot->laserXY[i].x = 0;
        robot->laserXY[i].y = 0;
    }
}

/**
 * handles control-c for quitting the program
 * @param signal
 */
void sighandler(int signal) {

    // turn everything off and disconnect
    irOff();
    sonarOff();
    usleep(1000000);
    disconnectRobot();

    fprintf(stderr, "Exiting on signal %d after 1 second\n", signal);

    usleep(1000000);

    exit(signal);

}

/**
 * puts the IR readings into the readings pointer for easier access
 * @param State     pointer to Mage State array
 * @param readings  pointer to destination array
 */
void getAllIRReadings(long* State, int* readings) {
    for (int i = 0; i < INFRAREDS; ++i)
    {
        readings[i] = State[i];
    }
}

/**
 * puts the sonar readings into the readings pointer for easier access
 * @param State     pointer to Mage State array
 * @param readings  pointer to destination array
 */void getAllSonarReadings(long* State, int* readings) {
    for (int i = 0; i < SONARS; ++i)
    {
        readings[i] = State[i+17];
    }
}

/**
 * reads both IR and sonars and assign the minimum values to create a virtual sensor
 * @param robot     pointer
 */
void getVirtualSensorReadings(Robot *robot) {
    int *ireads = (int *)malloc(sizeof(int) * INFRAREDS);
    int *sreads = (int *)malloc(sizeof(int) * SONARS);
    getAllIRReadings(robot->State,ireads);
    getAllSonarReadings(robot->State,sreads);
    int minReading0,minReading1;
    for (int i = 0; i < SONARS/2; i++) {
        minReading0 = ireads[2*i] < sreads[2*i] ? ireads[2*i] : sreads[2*i];
        minReading1 = ireads[2*i+1] < sreads[2*i+1] ? ireads[2*i+1] : sreads[2*i+1];
        robot->vSensors[i] = minReading1 < minReading0 ? minReading1 : minReading0;
    }

    free(ireads);
    free(sreads);
}

/**
 * grabs the laser readings from the URGserver and stores them in robot pointer
 * @param robot pointer to store the readings
 */
void getLaserSensorReadings(Robot *robot) {
    int sock;
    char serverName[128];
    char clientName[128];
    char message[512];
    int N;
    int *data;

    strcpy( serverName, "/tmp/socket-urglaser-server" );
    strcpy( clientName, "/tmp/socket-urglaser-client" );

    sock = makeFilenameSocket( clientName );

    // send request message
    strcpy( message, "I" );
    N = sendFilenameSocket( sock, serverName, message, strlen(message)+1 );

    // read the image
    data = readFilenameData( sock, &N);

    for (int i = 0; i < NUM_LASERS; ++i) {
        robot->laser[i] = data[i] < 10 || data[i] > 7000 ? 0 : data[i];
    }

    closeFilenameSocket( sock, clientName );
    free(data);

}

/**
 * convert laser readings (which are in polar coords) to euclidean X-Y coords
 * @param robot     struct to with pointer to laserXY field store the new points in
 */
void convertLaserToXY(Robot *robot) {
    int li;
    for (int i = 0; i < NUM_LASERS; ++i) {
        li = i - NUM_LASERS/2;
        robot->laserXY[i].x = (robot->laser[i] * cos((double)(li * DEGREES_PER_LASER * PI / 180.0))) + ROBOT_RADIUS_MM;
        robot->laserXY[i].y = robot->laser[i] * sin((double)(li * DEGREES_PER_LASER * PI / 180.0));
    }
}

/**
 * stores the correct laser readings into the particle filter laser array and sets the angles correctly
 * @param robot
 */
void updatePFSensors(Robot * robot) {
    int idx;
    printf("n: %d\n",robot->particleFilter->nReadings);
    for (int i = 0; i < robot->particleFilter->nReadings; ++i) {
        idx = (int)((float)i*(float)NUM_LASERS/(float)robot->particleFilter->nReadings);
        robot->particleLaserAngles[i] = (idx - NUM_LASERS/2) * DEGREES_PER_LASER * PI / 180.0;
        robot->particleLaserAngles[i] = robot->particleLaserAngles[i] < 0 ? robot->particleLaserAngles[i] + 2.0 * PI : robot->particleLaserAngles[i];
        robot->particleLasers[i] = robot->laser[idx];
    }
}

/**
 * builds an occupancy grid based on the laseyXY field, where a 1 means that grid cell
 * is occupied and a 0 means not occupied. the grid is square with side size OG_SIZE/OG_RES,
 * which are both constants defined in "robot.h"
 * @param robot
 */
void buildOccupancyGrid(Robot * robot) {
    int size = OG_SIZE / OG_RES;
    int **grid = (int**)malloc(sizeof(int*) * size);
    for (int i = 0; i < size; i++) {
        grid[i] = (int*)malloc(sizeof(int) * size);
        for (int j = 0; j < size; j++) {
            if (i == 0 || i == size - 1 ||
                    j == 0 || j == size -1) {
                grid[i][j] = 1;
            } else {
                grid[i][j] = 0;
            }
        }
    }

    robot->occupancyGridSize = 0;
    for (int i = 0; i < NUM_LASERS; i++) {
        double x = (double)robot->laserXY[i].x/(double)OG_RES;
        double y = (double)robot->laserXY[i].y/(double)OG_RES;
        long row = lround(x);
        long col = lround(y);
        if (row < size/2 && row > -size/2 &&
                col < size/2 && col > -size/2) {
            // if within OG_SIZE range
            grid[row + size/2][col + size/2] = 1;
            Point p;
            p.x = (int)col + size/2;
            p.y = (int)row + size/2;
            robot->occupancyGrid[robot->occupancyGridSize] = p;
            robot->occupancyGridSize++;
        }
    }

    for (int i = 0; i < OG_SIZE / OG_RES; ++i) {
        for (int j = 0; j < OG_SIZE / OG_RES; ++j) {
            if (grid[i][j] == 0) {
                printf(" ");
            } else {
                printf("0");
            }
        }
        printf("\n");
    }

    // free grid
    for (int i = 0; i < size; i++) {
        free(grid[i]);
    }
    free(grid);

}


/**
 * Finds if a line segment defined by p0 and p1 intersects a circle defined by center and radius
 * @param center    circle definition
 * @param radius    center definition
 * @param p0        line start
 * @param p1        line end
 * @param intersection  Point pointer of the intersection point, NULL if no intersection
 */
void findLineSegCircleIntersection(Point center, double radius, Point p0, Point p1, Point* intersection) {
    float dx, dy, A, B, C, det, t;

    dx = p1.x - p0.x;
    dy = p1.y - p0.y;

    A = dx * dx + dy * dy;
    B = 2 * (dx * (p0.x - center.x) + dy * (p0.y - center.y));
    C = (p0.x - center.x) * (p0.x - center.x) + (p0.y - center.y) * (p0.y - center.y) - radius * radius;
    det = B * B - 4 * A * C;
    if ((A <= 0.000001) || (det < 0)) {
        // no real solutions
        intersection->x = NULL;
        intersection->y = NULL;
    } else if (det == 0) {
        // one solution
        t = -B / (2 * A);
        intersection->x = p0.x + t * dx;
        intersection->y = p0.y + t * dy;
    } else {
        // two solutions
        if (p0.x + (-B + sqrt(det)) / (2 * A) * dx > p0.x + (-B - sqrt(det)) / (2 * A) * dx) {
            intersection->x = p0.x + (-B + sqrt(det)) / (2 * A) * dx;
            intersection->y = p0.y + (-B + sqrt(det)) / (2 * A) * dy;
        } else {
            intersection->x = p0.x + (-B - sqrt(det)) / (2 * A) * dx;
            intersection->y = p0.y + (-B - sqrt(det)) / (2 * A) * dy;
        }
    }


}

/**
 * this funciton converts the orientation of robot to -pi to pi range
 * @param robot
 */
void convertOrientation(Robot *robot) {
    robot->orientationPiToPi = robot->State[STATE_T] > PI * 1000.0 ? robot->State[STATE_T] - (int)(2.0 * PI * 1000.0) : robot->State[STATE_T];
}

/*
  Generates numbers from a Gaussian distribution with zero mean and
  unit variance.

  Concept and code from NRC

  Modified to use drand48() and return doubles
*/
double gaussDist() {
    static int iset=0;
    static double gset;
    float fac,rsq,v1,v2;

    // generate a new pair of numbers and return the first
    if(iset == 0) {
        do {
            v1 = 2.0*drand48()-1.0;
            v2 = 2.0*drand48()-1.0;
            rsq = v1*v1+v2*v2;
        } while (rsq >= 1.0 || rsq == 0.0);

        fac = sqrt(-2.0*log(rsq)/rsq);
        gset = v1*fac;
        iset = 1;

        return(v2*fac);
    }

    // return the last number we generated
    iset = 0;

    return(gset);
}

/**
 * finds the closest flat surface around the robot by taking the minimum reading and
 * scanning the adjacent readings to look for values that stay pretty similar,
 * then returns the center of that segment
 * @param robot
 */
void findClosestFlatSurface(Robot *robot) {
    int i, dir, min, mind;

    min = 8000;
    mind = 0;
    for (i=20; i < NUM_LASERS - 20; i++) {
        if (robot->laser[i] < min )  {
            if (abs(robot->laser[i+1] - robot->laser[i]) < 20 && abs(robot->laser[i-1] - robot->laser[i]) < 20
                && abs(robot->laser[i+2] - robot->laser[i]) < 20 && abs(robot->laser[i-2] - robot->laser[i]) < 20) {
                min = robot->laser[i];
                mind = i;
            }
        }
    }
    int distFromMin = 1;
    int donel = 0;
    int doner = 0;
    int indl = mind;
    int indr = mind;
    printf("min: %d, mind: %d\n", min, mind);
    while (donel != 1 && doner != 1) {
        distFromMin = abs(mind - indl);
        if ((abs(robot->laser[indl] - min) > distFromMin*10) || indl == 0)
            donel = 1;
        distFromMin = abs(mind - indr);
        if ((abs(robot->laser[indr] - min) > distFromMin*10) || indr == NUM_LASERS -1)
            doner = 1;
        indr++;
        indl--;
    }

    dir = (indl + indr)/2;
    robot->laserIdxClosestFlat = dir;
}

/**
 * writes a hough accumulator to a text file for debugging
 * @param accum     double pointer to the accumulator
 * @param nump      first dimension size
 * @param numtheta  second dimension size
 */
void writeHoughAccumulator(int** accum, int nump, int numtheta) {
    int max = 0;
    for (int i = 0; i < nump; ++i) {
        for (int j = 0; j < numtheta; j++) {
            max = accum[i][j] > max ? accum[i][j] : max;
        }
    }
    printf("done max\n");
    for (int i = 0; i < nump; ++i) {
        for (int j = 0; j < numtheta; j++) {
            accum[i][j] = (int)(((float)accum[i][j] / (float)max) * 255.0);
        }
    }
    FILE *f = fopen("accum.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    printf("opened file\n");
    for (int i = 0; i < nump; ++i) {
        for (int j = 0; j < numtheta; j++) {
            if (j == numtheta -1 ) {
                fprintf(f, "%d", accum[i][j] );
            } else {
                fprintf(f, "%d,", accum[i][j] );
            }

        }
        fprintf(f,"\n");
    }
    printf("done writing\n");
    fclose(f);
}

/**
 * performs a hough transform using the laser readings to look for a flat surface
 *
 * tweak pmax, thetamaz, deltap, deltatheta for different use cases
 * stores direction and distance of best straight line in fields of robot
 * @param robot for accessing lasers
 */
void houghLine(Robot *robot, int numLines) {
    printf("in hough\n");
    int pmax, deltarho, deltatheta, thetamax, numrho, numtheta, currho, curtheta, x, y, smoothed,rhoidx;
    pmax = 2000; // max 0.75m
    thetamax = (int)(2.0 * PI * 1000.0); // 2 pi millirads
    deltarho = 50;
    deltatheta = (int)(2.0 * PI * 1000.0 / 90.0);
    numrho = pmax/deltarho;
    numtheta = thetamax/deltatheta;


    // initialize accumulator to zeros
    Line *lines = (Line*)malloc(sizeof(Line) * numrho * numtheta);
    for (int i = 0; i < numrho; i++) {
        for (int j = 0; j < numtheta; j++) {
            Line l;
            l.votes = 0;
            l.rho = i * deltarho;
            l.theta = (j*deltatheta > PI * 1000.0 ? j*deltatheta - (int)(2.0 * PI * 1000.0) : j*deltatheta);
            lines[i * numtheta + j] = l;
        }
    }


    printf("made accumulator\n");

    for (int i = 0; i < NUM_LASERS; ++i) {
        for (int j = 0; j < numtheta; j++) {
            curtheta = j * deltatheta;
            if (robot->laser[i] == 0) {
                break;
            }
            x = robot->laserXY[i].x;
            y = robot->laserXY[i].y;

            currho = (int)(x * cos(((float)curtheta/1000.0)) + y * sin(((float)curtheta/1000.0)));

            rhoidx = currho/deltarho;
            if (currho < pmax && currho >= 0) {
                lines[rhoidx * numtheta + j].votes++;
            }

        }
    }
    printf("did counts\n");

    Line tmp;
    // sort line list to get best ones
    for (int i = 0; i < numrho * numtheta - 1; i++) {
        for (int j = i + 1; j < numrho * numtheta; j++) {
            if (lines[j].votes > lines[i].votes) {
                // swap i and j
                tmp = lines[i];
                lines[i] = lines[j];
                lines[j] = tmp;
            }
        }
    }
    static Line *bestLines;
    bestLines = NULL;
    if (bestLines == NULL) {
        bestLines = (Line *) malloc(sizeof(Line) * numLines);
    }

    for (int k = 0; k < numLines; ++k) {
        bestLines[k].rho = -99999;
        bestLines[k].theta = -99999;
        bestLines[k].votes = -99999;
    }

    // get the best different lines
    int numAdded = 0;
    int toAdd;
    for (int i = 0; i < numrho * numtheta; i++) {
        toAdd = 1;
        for (int j = 0; j < numLines; j++) {
            int thetaDiff = abs(lines[i].theta - bestLines[j].theta);
            //int rhoDiff = abs(lines[i].rho - bestLines[j].rho);
            if (thetaDiff < 250) {
                toAdd = 0;
            }
        }
        if (toAdd){
            bestLines[numAdded++] = lines[i];
        }
        if (numAdded == numLines) {
            break;
        }
    }

    robot->houghBestLines = bestLines;
    for (int i = 0; i < numLines; ++i) {
        printf("max accum: %d, distance to line (mm): %d, angle from origin (millirad): %d\n",
               bestLines[i].votes,bestLines[i].rho,bestLines[i].theta);
    }

    if (bestLines[numLines-1].votes < 40) {
        for (int i = 0; i < numLines; ++i) {
            printf("bestCounts is less than 40, there are no lines visible\n");
            bestLines[i].rho = -999;
            bestLines[i].theta = -999;
        }
    }

    // writeHoughAccumulator(accumulator,numrho,numtheta);

    free(lines);
}

/**
 * performs a hough transform looking for a circle instead of a line
 * @param robot
 */
void houghCircle(Robot *robot) {
    printf("in hough\n");
    int xmax,ymax,numx,numy, x, y, smoothed, maxaccum, bestx, besty, numpoints, thisx, thisy;
    double deltatheta,curtheta;
    xmax = 500;
    ymax = 500; // max .5m
    numx = xmax*2;
    numy = ymax*2;
    numpoints = 720;
    deltatheta = 2 * PI / 180.0;
    // initialize accumulator to zeros
    int **accumulator = (int**)malloc(sizeof(int*) * numx);
    int **smoothAccum = (int**)malloc(sizeof(int*) * numx);
    for (int i = 0; i < numx; ++i) {
        accumulator[i] = (int*)malloc(sizeof(int) * numy);
        smoothAccum[i] = (int*)malloc(sizeof(int) * numy);
    }

    for (int i = 0; i < numx; ++i) {
        for (int j = 0; j < numy; ++j) {
            accumulator[i][j] = 0;
        }
    }

    printf("made accumulator\n");

    for (int i = 0; i < NUM_LASERS; ++i) {
        for (int j = 0; j < numpoints; j++) {
            curtheta = j * deltatheta;
            x = robot->laserXY[i].x;
            y = robot->laserXY[i].y;
            thisx = (int)(x + robot->houghRadius * cos(curtheta));
            thisy = (int)(y + robot->houghRadius * sin(curtheta));
            if (thisx < xmax && thisx > -xmax && thisy < ymax && thisy > -ymax) {
                accumulator[thisx+xmax][thisy+ymax]++;
            }
        }
    }
    printf("did counts\n");
    // smooth accumulator
    maxaccum = 0;
    for (int i = 1; i < numx-1; ++i) {
        for (int j = 1; j < numy-1; ++j) {
            smoothed = accumulator[i][j]*0.3 + accumulator[i-1][j]*0.125 + accumulator[i+1][j]*0.125 + accumulator[i][j-1]*0.125 +
                       accumulator[i][j+1]*0.125 + accumulator[i-1][j-1]*0.05 + accumulator[i+1][j-1]*0.05 + accumulator[i-1][j+1]*0.05 + accumulator[i-1][j-1]*0.05;
            smoothAccum[i][j] = smoothed;
            if (maxaccum < smoothed) {
                maxaccum = smoothed;
                bestx = i-xmax;
                besty = j-ymax;
            }
        }
    }
    if (maxaccum < 5) {
        printf("maxaccum is less than 3, there are no circles visible\n");
        robot->houghCircleCenter.x = 8000;
        robot->houghCircleCenter.y = 8000;
    } else {
        printf("max accum: %d, x: %d, y: %d\n",maxaccum,bestx,besty);
        robot->houghCircleCenter.x = bestx;
        robot->houghCircleCenter.y = besty;
    }
    // writeHoughAccumulator(accumulator,nump,numtheta);

    for (int i = 0; i < numx; ++i) {
        free(accumulator[i]);
        free(smoothAccum[i]);
    }
    free(accumulator);
    free(smoothAccum);
}

/**
 * limits the translation and rotation velocity commands to a hard max and an acceleration max
 * @param robot
 * @param vt    proposed translation command
 * @param wt    proposed rotation command
 */
void limitVMCommands(Robot* robot,int *vt, int *wt) {
    int vaccel,waccel;
    while (*vt > MAX_TRANS_SPEED ||
           *wt > MAX_ROT_SPEED ||
           *vt < -MAX_TRANS_SPEED ||
           *wt < -MAX_ROT_SPEED ) {
        //printf("the first while loop %d,%d,%d,%d\n", *vt,*wt,lastv,lastw);
        *vt = (*vt) * 9 / 10;
        *wt = (*wt) * 9 / 10;
    }
    vaccel = (*vt) - robot->vt;
    waccel = (*wt) - robot->wt;
    *vt = vaccel > MAX_TRANS_ACCEL ? robot->vt + MAX_TRANS_ACCEL : *vt;
    *vt = vaccel < -MAX_TRANS_ACCEL ? robot->vt - MAX_TRANS_ACCEL : *vt;
    *wt = waccel > MAX_ROT_ACCEL ? robot->wt + MAX_ROT_ACCEL : *wt;
    *wt = waccel < -MAX_ROT_ACCEL ? robot->wt - MAX_ROT_ACCEL : *wt;
}

/**
 * makes sure the robot isn't gonna run into something, and stops it if it is
 * called right before executing the vm() command in the main loop
 * @param robot
 * @param vt    proposed translation velocity
 * @param wt    proposed rotation velocity
 */
void stopForObstacles(Robot *robot,int*vt,int*wt) {
    int minObj = 80000;
    int dir;
    // if moving forward
    if ((*vt) > 0) {
        // loop through front IR/sonar
        for (int i = 0; i < NUM_VSENSORS; i++) {
            if (robot->vSensors[i] < minObj && robot->vSensors[i] > 3) {
                i = i == -1 ? 6 : i;
                minObj = robot->vSensors[i];
                dir = i;

            }
        }
        // loop through lasers
        for (int i = 0; i < NUM_LASERS; i++) {
            int dist = (int)sqrt(robot->laserXY->x*robot->laserXY->x - robot->laserXY->y*robot->laserXY->y);
            minObj = dist - ROBOT_RADIUS_MM < minObj ? dist - ROBOT_RADIUS_MM : minObj;
        }
    // else moving backward
    } else {
        for (int i = 2; i < 6; ++i) {
            if (robot->vSensors[i] < minObj  && robot->vSensors[i] > 3) {
                minObj = robot->vSensors[i];
                dir = i;
            }
        }

    }
    if (minObj < 75 && minObj != 0) {
        printf("emergency stop! Object in %d mm\n",minObj);
        *vt = (*vt) > 0 ? -100 : 100;
        *wt = 0;
    }
}

/**
 * returns the p-value association with a given zscore based on a table
 * @param zscore
 * @return
 */
float getPValue(float zscore) {
    float pVals[] = {
            .0003 ,.0003 ,.0003 ,.0003 ,.0003 ,.0003 ,.0003 ,.0003 ,.0003 ,.0002
            ,.0005 ,.0005 ,.0005 ,.0004 ,.0004 ,.0004 ,.0004 ,.0004 ,.0004 ,.0003
            ,.0007 ,.0007 ,.0006 ,.0006 ,.0006 ,.0006 ,.0006 ,.0005 ,.0005 ,.0005
            ,.0010 ,.0009 ,.0009 ,.0009 ,.0008 ,.0008 ,.0008 ,.0008 ,.0007 ,.0007
            ,.0013 ,.0013 ,.0013 ,.0012 ,.0012 ,.0011 ,.0011 ,.0011 ,.0010 ,.0010
            ,.0019 ,.0018 ,.0018 ,.0017 ,.0016 ,.0016 ,.0015 ,.0015 ,.0014 ,.0014
            ,.0026 ,.0025 ,.0024 ,.0023 ,.0023 ,.0022 ,.0021 ,.0021 ,.0020 ,.0019
            ,.0035 ,.0034 ,.0033 ,.0032 ,.0031 ,.0030 ,.0029 ,.0028 ,.0027 ,.0026
            ,.0047 ,.0045 ,.0044 ,.0043 ,.0041 ,.0040 ,.0039 ,.0038 ,.0037 ,.0036
            ,.0062 ,.0060 ,.0059 ,.0057 ,.0055 ,.0054 ,.0052 ,.0051 ,.0049 ,.0048
            ,.0082 ,.0080 ,.0078 ,.0075 ,.0073 ,.0071 ,.0069 ,.0068 ,.0066 ,.0064
            ,.0107 ,.0104 ,.0102 ,.0099 ,.0096 ,.0094 ,.0091 ,.0089 ,.0087 ,.0084
            ,.0139 ,.0136 ,.0132 ,.0129 ,.0125 ,.0122 ,.0119 ,.0116 ,.0113 ,.0110
            ,.0179 ,.0174 ,.0170 ,.0166 ,.0162 ,.0158 ,.0154 ,.0150 ,.0146 ,.0143
            ,.0228 ,.0222 ,.0217 ,.0212 ,.0207 ,.0202 ,.0197 ,.0192 ,.0188 ,.0183
            ,.0287 ,.0281 ,.0274 ,.0268 ,.0262 ,.0256 ,.0250 ,.0244 ,.0239 ,.0233
            ,.0359 ,.0351 ,.0344 ,.0336 ,.0329 ,.0322 ,.0314 ,.0307 ,.0301 ,.0294
            ,.0446 ,.0436 ,.0427 ,.0418 ,.0409 ,.0401 ,.0392 ,.0384 ,.0375 ,.0367
            ,.0548 ,.0537 ,.0526 ,.0516 ,.0505 ,.0495 ,.0485 ,.0475 ,.0465 ,.0455
            ,.0668 ,.0655 ,.0643 ,.0630 ,.0618 ,.0606 ,.0594 ,.0582 ,.0571 ,.0559
            ,.0808 ,.0793 ,.0778 ,.0764 ,.0749 ,.0735 ,.0721 ,.0708 ,.0694 ,.0681
            ,.0968 ,.0951 ,.0934 ,.0918 ,.0901 ,.0885 ,.0869 ,.0853 ,.0838 ,.0823
            ,.1151 ,.1131 ,.1112 ,.1093 ,.1075 ,.1056 ,.1038 ,.1020 ,.1003 ,.0985
            ,.1357 ,.1335 ,.1314 ,.1292 ,.1271 ,.1251 ,.1230 ,.1210 ,.1190 ,.1170
            ,.1587 ,.1562 ,.1539 ,.1515 ,.1492 ,.1469 ,.1446 ,.1423 ,.1401 ,.1379
            ,.1841 ,.1814 ,.1788 ,.1762 ,.1736 ,.1711 ,.1685 ,.1660 ,.1635 ,.1611
            ,.2119 ,.2090 ,.2061 ,.2033 ,.2005 ,.1977 ,.1949 ,.1922 ,.1894 ,.1867
            ,.2420 ,.2389 ,.2358 ,.2327 ,.2296 ,.2266 ,.2236 ,.2206 ,.2177 ,.2148
            ,.2743 ,.2709 ,.2676 ,.2643 ,.2611 ,.2578 ,.2546 ,.2514 ,.2483 ,.2451
            ,.3085 ,.3050 ,.3015 ,.2981 ,.2946 ,.2912 ,.2877 ,.2843 ,.2810 ,.2776
            ,.3446 ,.3409 ,.3372 ,.3336 ,.3300 ,.3264 ,.3228 ,.3192 ,.3156 ,.3121
            ,.3821 ,.3783 ,.3745 ,.3707 ,.3669 ,.3632 ,.3594 ,.3557 ,.3520 ,.3483
            ,.4207 ,.4168 ,.4129 ,.4090 ,.4052 ,.4013 ,.3974 ,.3936 ,.3897 ,.3859
            ,.4602 ,.4562 ,.4522 ,.4483 ,.4443 ,.4404 ,.4364 ,.4325 ,.4286 ,.4247
            ,.5000 ,.4960 ,.4920 ,.4880 ,.4840 ,.4801 ,.4761 ,.4721 ,.4681 ,.4641
    };
    int z,total,rem,tens,idx;
    z = zscore < 0 ? (int)(-100.0*zscore) : (int)(100.0*zscore);
    if (z > 350) {
        return 0.0001;
    }
    total = 350;
    rem = z%10;
    tens = (int)ceil((double)z/10.0) * 10;
    idx = total - (tens - rem) - 1;
//    printf("idx:%d,tens:%d,rem%d\n",idx,tens,rem);
    return pVals[idx];
}

/**
 * writes the particle filter to an image with all particles being red dots
 * @param particleFilter
 * @param est best particle for special drawing
 * @param map to draw on
 * @param pfImage Pixel array
 * @param N index of image
 */
void writePFToImage(ParticleFilter *particleFilter,Particle est,Map*map,Pixel*pfImage,int N) {
    for (int i = 0; i < particleFilter->N; i++) {
        map_set(map, particleFilter->p[i].x, particleFilter->p[i].y, 50);
    }
    map_set(map, est.x, est.y, 49);
    printf("done drawing particles\n");
    map_setPix(map, pfImage);
    // reset particles to white
    for (int i = 0; i < particleFilter->N; i++) {
        map_set(map, particleFilter->p[i].x, particleFilter->p[i].y, 255);
    }
    char f[50];
    sprintf(f, "../out/p%03d.ppm", N);
    writePPM(pfImage, map->rows, map->cols, 255, f);
    printf("wrote ppm\n");
}

/**
 * frees a robot and all inner fields. needs to be called before program exits
 * @param robot to free
 */
void freeRobot(Robot *robot) {
    free(robot->vSensors);
    free(robot->laser);
    free(robot->laserXY);
    free(robot->pfPoints);
    IPC_freeData(IPC_msgFormatter(SVM_DATA_RESPONSE), robot->SVMData);
    free(robot->occupancyGrid);
    pf_free(robot->particleFilter);
    map_free(robot->map);
    free(robot->particleLasers);
    free(robot->particleLaserAngles);
    free(robot->pfImage);
    free(robot);
}
