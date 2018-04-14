/**
 * Matt Martin, Makoto Kinoshita
 * Mar 16, 2017
 * CS363 Robotics
 * movements.c
 * this file is consist of functions called in nav.c file.
 * It deals with which funciton should be callsed in a certain State
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <SVM_VisionModule.h>
#include <time.h>
#include <map.h>
#include "Mage.h"
#include "robot.h"
#include "utils.h"
#include "test.h"
#include "histogram.h"


// This function is to test houghCircle or houghLine
// You can call houghLine by changing houghCircle to houghLine
void handle_test(Robot *robot_s, int*v, int*w) {
    houghLine(robot_s,3);
    *v = 0;
    *w = 0;
}

//This stops a robot
void handle_idle(Robot* robot_s, int* v, int *w) {
    *v = 0;
    *w = 0;
}

//This function makes a robot follow a certain point
void handle_point_follow(Robot* robot_s, int* v, int* w) {
    // check if we've achieved the current goal point
    if (abs(robot_s->State[STATE_X] - robot_s->goalPosition[0]) < 15 &&
        abs(robot_s->State[STATE_Y] - robot_s->goalPosition[1]) < 15) {
        robot_s->pfIndex++;
    }

    // check if we've achieved all points, else set goalPosition
    if (robot_s->pfIndex == robot_s->nPfPoints) {
        robot_s->task = QUIT;
        return;
    } else {
        robot_s->goalPosition[0] = robot_s->pfPoints[2*robot_s->pfIndex];
        robot_s->goalPosition[1] = robot_s->pfPoints[2*robot_s->pfIndex+1];
    }

    int goal_x = robot_s->goalPosition[0];
    int goal_y = robot_s->goalPosition[1];
    int x = robot_s->State[STATE_X];
    int y = robot_s->State[STATE_Y];
    int ori = robot_s->State[STATE_T];
    ori = ori > PI * 1000.0 ? ori - (int)(2.0 * PI * 1000.0) : ori;


    //calculate the distance diffrence and orientation difference
    int diff_x = goal_x - x;
    int diff_y = goal_y - y;
    int diff_dis = (int)sqrt(diff_x * diff_x + diff_y * diff_y);
    int goal_ori = (int)(atan2(diff_y, diff_x)*1000.0);
    int diff_ori = goal_ori - ori;

    printf("goalPosition: %d, %d\ncurrentPos: %d, %d\n",goal_x, goal_y, x, y);

    printf("diff_dis: %d\n", diff_dis);

    printf("robot orientation: %d, goal_ori: %d, diff_ori: %d\n", ori,goal_ori,diff_ori);

    int minT = (int)(-PI / 3.0)*1000;
    int maxT = (int)(PI / 3.0)*1000;

    if ((diff_ori > minT) && (diff_ori < maxT)) {
        printf("driving and turning\n");
        *v = diff_dis/(abs(diff_ori)/20+1);
        *w = diff_ori/5;
    } else {
        printf("only turning\n");
        *v = 0;
        *w = diff_ori/2;
    }

}

// This function tries to find a flat wall around a robot
void handle_wall_find(Robot* robot_s, int* v, int *w) {

    int left, lcount, center, ccount, right, rcount, direction, fwd, vel;

    vel = 100;

    //calculate the scores for each group
    left = 0;
    lcount = 0;
    for (int i = 200; i < 285;i++) {
      if (robot_s->laser[i] > 0) {
        left += robot_s->laser[i];
        lcount += 1;
      }
    }
    center = 0;
    ccount = 0;
    for (int i = 128; i < 213; i++) {
      if (robot_s->laser[i] > 0) {
        center += robot_s->laser[i];
        ccount += 1;
      }
    }
    right = 0;
    rcount = 0;
    for (int i = 55; i < 140; i++) {
      if (robot_s->laser[i] > 0) {
        right += robot_s->laser[i];
        rcount += 1;
      }
    }
    left = (int)left/lcount;
    center = (int)center/ccount;
    right = (int)right/rcount;

    if (left < robot_s->goalDistance *1.5 || center < robot_s->goalDistance *1.5 || right < robot_s->goalDistance*1.5) {
        robot_s->task = WALL_FOLLOW;
        *v = 0;
        *w = 0;
        return;
    }

    printf("l: %d, c: %d, r: %d\n",left,center,right);
    // determine the direction to go
    if ((left < center && left < right) || robot_s->vSensors[1] < 150) {
      direction = 5;
      fwd = 1;
    } else if ((right < left && right < center) || robot_s->vSensors[6] < 150) {
      direction = -5;
      fwd = 1;
    } else {
      direction = 0;
      fwd = 2;
    }
    *v = vel*fwd;
    *w = vel*direction;
}

// Once a robot finds a wall, this function is called which makes a robot follow a wall
void handle_wall_follow(Robot* robot_s, int* v, int *w) {
    int diffDir, diffDist, turn, speed;
    float alpha,beta;

    alpha = 1.2;
    beta = 1.0;

    findClosestFlatSurface(robot_s);

    diffDir = robot_s->laserIdxClosestFlat - robot_s->goalOrientation;
    diffDist = robot_s->goalDistance - robot_s->laser[robot_s->laserIdxClosestFlat];

    printf("dist: %d\n", robot_s->laser[robot_s->laserIdxClosestFlat]);
    printf("diffDist: %d, diffDir: %d\n", diffDist, diffDir);

    if (robot_s->goalOrientation > 170) {
        printf("inside the first loop\n");
        turn = (int)((1000.0 / (float)robot_s->goalDistance) * diffDir + alpha * -1.0 * diffDist);
    } else {
        printf("inside the second loop\n");
        turn = (int)((1000.0 / (float)robot_s->goalDistance) * diffDir + alpha * diffDist);
    }
    speed = beta * 100;

    *v = speed;
    *w = turn;
}

void handle_hough_wall_follow(Robot *robot_s, int*v, int*w) {
    int vel = 100;
    float alpha,beta;
    int diffDir,diffDist;
    alpha = 1.0;
    beta = 0.5;

    houghLine(robot_s,1);
    diffDir = robot_s->houghBestLines[0].theta - robot_s->goalOrientation;
    if (robot_s->goalOrientation < 3000) {
        // wall on right side
        diffDist = robot_s->houghBestLines[0].rho - robot_s->goalDistance;
    } else {
        // wall on left side
        diffDist = robot_s->goalDistance - robot_s->houghBestLines[0].rho;
    }
    printf("diff dir: %d, diff dist: %d\n",diffDir,diffDist);
    *v = vel;
    *w = (int)(diffDist * alpha + diffDir * beta);
}

// This function makes it posibble for a robot to traverse a maze
void handle_maze(Robot* robot_s, int* v, int*w) {
    int diffDir, diffDist, turn, speed;
    float alpha,beta;

    alpha = 1.2;
    beta = 0.7;

    /*
     * circle detection not working and too slow
     *
    houghCircle(robot_s);

    if (robot_s->houghBuffer * robot_s->houghBuffer >
        (robot_s->houghCircleCenter.x * robot_s->houghCircleCenter.x +
        robot_s->houghCircleCenter.y  * robot_s->houghCircleCenter.y)) {
        printf("reached end of maze see circle at %d, %d\n",robot_s->houghCircleCenter.x, robot_s->houghCircleCenter.y);
        robot_s->task = QUIT;
    }
    */


    findClosestFlatSurface(robot_s);

    if (robot_s->laserSide == 1 && robot_s->laser[robot_s->laserIdxClosestFlat] > robot_s->goalDistance && robot_s->laserIdxClosestFlat < 180) {
        robot_s->laserSide = 0;
        robot_s->goalOrientation = 60;
    } else if (robot_s->laserSide == 0 && robot_s->laser[robot_s->laserIdxClosestFlat] > robot_s->goalDistance && robot_s->laserIdxClosestFlat > 160) {
        robot_s->laserSide = 1;
        robot_s->goalOrientation = 280;
    } else if (robot_s->laserSide == -1) {
        robot_s->laserSide = robot_s->laserIdxClosestFlat > 170 ? 1 : 0;
        robot_s->goalOrientation = robot_s->laserIdxClosestFlat > 170 ? 280 : 60;
    }
    diffDir = robot_s->laserIdxClosestFlat - robot_s->goalOrientation;
    diffDist = robot_s->goalDistance - robot_s->laser[robot_s->laserIdxClosestFlat];
    printf("diffDist: %d, diffDir: %d\n", diffDist, diffDir);


    if (robot_s->goalOrientation > 170) {
        turn = (int)((1000.0 / (float)robot_s->goalDistance) * diffDir - alpha  * diffDist);
    } else {
        turn = (int)((1000.0 / (float)robot_s->goalDistance) * diffDir + alpha * diffDist);
    }


    speed = beta * 100;

    *v = speed;
    *w = turn;

}

void handle_go_to(Robot* robot_s, int*v, int*w) {
    // velocity space navigation
    float alpha,beta,gamma;
    int nextGoalOrientation,i,j,l,D,H,V,max,maxv,maxw;
    alpha = 2.0;
    beta = 0.2;
    gamma = 0.2;
    // limit v and w based on max acceleration
    int vmin = robot_s->vt - MAX_TRANS_ACCEL;
    int vmax = robot_s->vt + MAX_TRANS_ACCEL;
    int wmin = robot_s->wt - MAX_ROT_ACCEL;
    int wmax = robot_s->wt + MAX_ROT_ACCEL;

    // circle intersection point
    Point * collide = (Point *)malloc(sizeof(Point));

    // create arrays for storing objective function values
    int **G = (int**)malloc(sizeof(int*)*(vmax-vmin));
    int **smoothG = (int**)malloc(sizeof(int*)*(vmax-vmin));
    for (i = 0; i < vmax-vmin; i++) {
        G[i] = (int*)malloc(sizeof(int)*(wmax-wmin));
        smoothG[i] = (int*)malloc(sizeof(int)*(wmax-wmin));
    }
    // loop thru possible v and w
    for (i = vmin; i < vmax; i++) {
        for (j = wmin; j < wmax; j++) {
            // calculate distance to nearest object along path v,w
            D = 500*MAX_TRANS_SPEED;
            // radius in mm of this circle
            double radius = (double)i/((double)j/1000.0);
            Point center;
            center.x = robot_s->State[STATE_X];
            center.y = robot_s->State[STATE_Y] - radius;
            for (l = 0; l < robot_s->occupancyGridSize; l++) {
                // check potential intersections with every line segment
                findLineSegCircleIntersection(center,radius,robot_s->occupancyGrid[l],
                                              robot_s->occupancyGrid[(l+1)%robot_s->occupancyGridSize],collide);
                if (collide->x != NULL) {
                    // calculate D
                    double theta = atan2(collide->y - center.y, collide->x - center.x);
                    D = theta * radius < D ? (int)(theta * radius) : D;
                }
            }
            // calculate goal heading
            nextGoalOrientation = (int)(atan2(robot_s->State[STATE_X] - robot_s->goalPosition[0],
                                          robot_s->State[STATE_Y] - robot_s->goalPosition[1]));
            H = nextGoalOrientation - robot_s->orientationPiToPi;

            V = D / (500*MAX_TRANS_SPEED);

            //printf("D: %d, H: %d, V: %d\n",D,H,V);

            G[i + vmax - vmin][j + wmax - wmin] = (int)(alpha*H + beta*D + gamma*V);
//            printf("%d\n",j);
        }
//        printf("%d\n",i);
    }
    printf("made G\n");
    // smooth results
    max = 0;
    for (i = 1; i < vmax-vmin-1; ++i) {
        for (j = 1; j < wmax-wmin-1; ++j) {
            smoothG[i][j] = (int)(G[i][j]*0.3 + G[i-1][j]*0.125 + G[i+1][j]*0.125 + G[i][j-1]*0.125 +
                       G[i][j+1]*0.125 + G[i-1][j-1]*0.05 + G[i+1][j-1]*0.05 + G[i-1][j+1]*0.05 + G[i-1][j-1]*0.05);
            if (max < smoothG[i][j]) {
                max = smoothG[i][j];
                maxv = i + (vmax-vmin);
                maxw = j + (wmax-wmin);
            }
        }
    }

    *v = maxv;
    *w = maxw;

    for (i = 0; i < vmax-vmin; i++) {
        free(G[i]);
        free(smoothG[i]);
    }
    free(G);
    free(smoothG);
    free(collide);
}


// This function is called when we want a robot wander
void handle_wander(Robot *robot_s, int* v, int* w) {
    int left, lcount, center, ccount, right, rcount, direction, fwd, vel;

    vel = 100;

    //calculate the scores for each group
    left = 0;
    lcount = 0;
    for (int i = 200; i < 285;i++) {
      if (robot_s->laser[i] > 0) {
        left += robot_s->laser[i];
        lcount += 1;
      }
    }
    center = 0;
    ccount = 0;
    for (int i = 128; i < 213; i++) {
      if (robot_s->laser[i] > 0) {
        center += robot_s->laser[i];
        ccount += 1;
      }
    }
    right = 0;
    rcount = 0;
    for (int i = 55; i < 140; i++) {
      if (robot_s->laser[i] > 0) {
        right += robot_s->laser[i];
        rcount += 1;
      }
    }
    left = (int)left/lcount;
    center = (int)center/ccount;
    right = (int)right/rcount;

    printf("l: %d, c: %d, r: %d\n",left,center,right);
    // determine the direction to go
    if ((left > center && left > right) || robot_s->vSensors[6] < 150) {
      direction = 5;
      fwd = 1;
    } else if ((right > left && right > center) || robot_s->vSensors[1] < 150) {
      direction = -5;
      fwd = 1;
    } else {
      direction = 0;
      fwd = 2;
    }
    *v = vel*fwd;
    *w = vel*direction;
}

// This fucntion is called when we wanna make a robot rotate
void handle_rotate(Robot *robot_s, int* v, int* w) {
    float kp = 0.5;
    int diffOri = robot_s->goalOrientation - robot_s->orientationPiToPi;
    if (robot_s->goalOrientation > 0 && diffOri > PI * 1000.0 ) {
        diffOri = diffOri - 2.0 * PI * 1000.0;
    } else if (robot_s->goalOrientation < 0 && diffOri < PI * -1000.0) {
        diffOri = diffOri + 2.0 * PI * 1000.0;
    }
    printf("goal: %d, cur: %d, diff: %d \n", robot_s->goalOrientation, robot_s->orientationPiToPi, diffOri);

    if (abs(diffOri) < 5) {
        robot_s->task = QUIT;
    }
    *v = 0;
    *w = kp*diffOri;
}

// This function is called when a robot is in PINK_FIND state and goes to PINK_TRACK state
// once it detects a face
void handle_pink_find(Robot *robot_s, int* v, int* w) {
    if (robot_s->SVMData->numLocations > 0) {
        robot_s->task = PINK_TRACK;
        return;
    }
    handle_idle(robot_s,v,w);
}

// This function is called when a robot is in PINK_TRACK and it keeps tracking the pink badge
void handle_pink_track(Robot *robot_s, int* v, int* w) {
    int window = 50;
    if (robot_s->SVMData->numLocations == 0) {
        robot_s->task = PINK_FIND;
        return;
    }
    float kpd = 0.5;
    float kpo = 1.5;
    // rotate to face pink blob
    int diffOri = robot_s->goalOrientation - robot_s->SVMData->location[0][0];
    // maintain constant distance to object under pink blob
    int minDist = 8000;
    int lookNearIndex = NUM_LASERS/2;
    for (int i = lookNearIndex - window; i < lookNearIndex + window; ++i) {
        minDist = robot_s->laser[i] < minDist ? robot_s->laser[i] : minDist;
    }
    printf("found min dist: %d near index %d\n",minDist,lookNearIndex);
    int diffDist = 0;
    if (minDist != 8000) {
        diffDist =  minDist - robot_s->goalDistance;
    }
    *v = (int)(diffDist*kpd);
    *w = (int)(diffOri*kpo);
}

// This function is called when a robot is in FACE_FIND state and goes to FACE_TRACK state
// once it detects a face. It wonders to find a face.
void handle_face_find(Robot *robot_s, int* v, int* w) {
    printf("handle_face_find phase %d\n", robot_s->SVMData->numLocations);
    if (robot_s->SVMData->numLocations > 0) {
        printf("found face at (%d, %d)\n", robot_s->SVMData->location[0][0], robot_s->SVMData->location[0][1]);
        robot_s->task = FACE_TRACK;
        robot_s->face_track_x_location = robot_s->SVMData->location[0][0];
        robot_s->face_track_y_location = robot_s->SVMData->location[0][1];
        return;
    }
    int left, lcount, center, ccount, right, rcount, direction, fwd, vel;

    vel = 100;

    //calculate the scores for each group
    left = 0;
    lcount = 1;
    for (int i = 200; i < 285;i++) {
      if (robot_s->laser[i] < 8000) {
        left += robot_s->laser[i];
        lcount += 1;
      }
    }
    center = 0;
    ccount = 1;
    for (int i = 128; i < 213; i++) {
      if (robot_s->laser[i] < 8000) {
        center += robot_s->laser[i];
        ccount += 1;
      }
    }
    right = 0;
    rcount = 1;
    for (int i = 55; i < 140; i++) {
      if (robot_s->laser[i] < 8000) {
        right += robot_s->laser[i];
        rcount += 1;
      }
    }
    left = (int)left/lcount;
    center = (int)center/ccount;
    right = (int)right/rcount;

    printf("l: %d, c: %d, r: %d\n",left,center,right);
    // determine the direction to go
    if ((left > center && left > right) || robot_s->vSensors[6] < 150) {
      direction = 5;
      fwd = 1;
    } else if ((right > left && right > center) || robot_s->vSensors[1] < 150) {
      direction = -5;
      fwd = 1;
    } else {
      direction = 0;
      fwd = 2;
    }
    *v = vel*fwd;
    *w = vel*direction;
}

// This function is called when a robot is in FACE_TRACK and it keeps tracking face
void handle_face_track(Robot *robot_s, int* v, int* w) {
    printf("handle_face_track phase\n");
    int window = 50;
    robot_s->face_track_counter++;
    if (robot_s->face_track_counter % 10 == 0) {
        if (robot_s->SVMData->location[0][0] == robot_s->face_track_x_location && robot_s->SVMData->location[0][1] == robot_s->face_track_y_location) {
            robot_s->task = FACE_FIND;
            robot_s->face_track_counter = 1;
            return;
        }
        robot_s->face_track_x_location = robot_s->SVMData->location[0][0];
        robot_s->face_track_y_location = robot_s->SVMData->location[0][1];
    }
    float kpd = 0.5;
    float kpo = 1.5;
    printf("found face at (%d, %d)\n", robot_s->SVMData->location[0][0], robot_s->SVMData->location[0][1]);
    // rotate to face pink blob
    int diffOri = robot_s->goalOrientation - robot_s->SVMData->location[0][0];
    // maintain constant distance to object under pink blob
    int minDist = 8000;
    int lookNearIndex = NUM_LASERS/2;
    for (int i = lookNearIndex - window; i < lookNearIndex + window; ++i) {
        minDist = robot_s->laser[i] < minDist ? robot_s->laser[i] : minDist;
    }
    printf("found min dist: %d near index %d\n",minDist,lookNearIndex);
    int diffDist = 0;
    if (minDist != 8000) {
        diffDist =  minDist - robot_s->goalDistance;
    }

    if (diffDist < 150) {
        robot_s->svm_operator = SVM_OP_Display;
        robot_s->task = TAKE_PICTURE;
        robot_s->face_track_counter = 1;
        return;
    }
    *v = (int)(diffDist*kpd);
    *w = (int)(diffOri*kpo);
}

//this function is to tak a picture in TAKE_PICTURE state
void handle_take_picture(Robot *robot_s, int* v, int* w) {
    robot_s->face_track_counter++;
    printf("taking a picture now\n");
    printf("operator id is %d\n", robot_s->svm_operator);
    if (robot_s->face_track_counter % 10 == 0) {
        printf("going back to wandering\n");
        *v = -300;
        *w = (int)-1 * PI * 1000.0;
        robot_s->task = FACE_FIND;
        robot_s->face_track_counter = 1;
        return;
    }
    *v = 0;
    *w = 0;
}

void handle_localize(Robot *robot_s, int*v, int*w) {
//    if (robot_s->iterations % 5 == 0) {
//        writePFToImage(robot_s->particleFilter,robot_s->map,robot_s->pfImage,robot_s->iterations/5*2);

    if (robot_s->particle_iter % 5 == 0) {
        struct timeval t1;
        gettimeofday(&(t1),NULL);
        double dt = (t1.tv_sec + t1.tv_usec/ 1000000.0) - (robot_s->time.tv_sec + robot_s->time.tv_usec/ 1000000.0);
        gettimeofday(&(robot_s->time),NULL);
        printf("dt value is %f\n",dt);

        pf_iterate(robot_s->particleFilter, *v, *w, dt,
                   robot_s->particleLaserAngles, robot_s->particleLasers,
                   robot_s->map,robot_s->pfImage,robot_s->iterations);
        printf("done pf\n");
        // get best particle and jump to its position
        Particle newPos = pf_getBestParticle(robot_s->particleFilter);

        writePFToImage(robot_s->particleFilter,newPos,robot_s->map,robot_s->pfImage,robot_s->iterations);

        printf("jumping to position: %f,%f,%f,%f\n", newPos.x, newPos.y, newPos.t, newPos.pi);
        jump((int) newPos.x / robot_s->map->gridSize, (int) newPos.y / robot_s->map->gridSize,
             (int) (newPos.t * 1000.0));
        robot_s->particle_iter = 0;
    }
    robot_s->particle_iter++;
    handle_wander(robot_s,v,w);
//    handle_idle(robot_s,v,w);
}

void handle_wait_for_face(Robot *robot_s, int*v, int*w) {
    // we see a face
    printf("lost count: %d\n",robot_s->IPCLostCount);
    if (robot_s->IPCLostCount == 0 && robot_s->timeSinceLastFace > 10) {
        char command[50];
        strcpy(command,"say hello can you help me");
        system(command);
        robot_s->timeSinceLastFace = 0;
    } else {
        robot_s->timeSinceLastFace++;
    }
    // someone comes within some range
    int minLaser = 8000;
    for (int i = 155; i < 185; ++i) {
        minLaser = robot_s->laser[i] < minLaser && robot_s->laser[i] != 0 ? robot_s->laser[i] : minLaser;
    }
    printf("minlaser: %d\n",minLaser);

    // write histogram for shirt color
    char inImage[50], outHist[50];
    strcpy(inImage,"../inc/shirt.ppm");
    sprintf(outHist,"/home/usr/svm/modules/svm_color%d.pgm",robot_s->histogramsMade);
    histogram(inImage,outHist);
    printf("made histogram number: %d\n",robot_s->histogramsMade);
    robot_s->histogramsMade = robot_s->histogramsMade >= 3 ? 0 : robot_s->histogramsMade+1;

    if (minLaser < robot_s->goalDistance) {
        char command[50];
        strcpy(command,"say can you take me to the elevator please");
        system(command);
        printf("Type y/n:\n");
        char c;
        scanf("%c",&c);
        if (c == 'y' || c == 'Y' || c == 'yes') {
            robot_s->svm_command_change = 1;
            robot_s->svm_previous_operator = robot_s->svm_operator;

            robot_s->svm_operator = SVM_OP_Color_Blobs;
            robot_s->task = FOLLOW_PERSON;
        }
    }
    *v = 0;
    *w = 0;

}

void handle_follow_person(Robot * robot_s, int *v, int *w) {

    // first, check if we are in the elevator
    int nWalls = 3;
    houghLine(robot_s,nWalls);

    int maxTotalDistance = 4000;
    int minTotalDistance = 2500;
    int minTotalVotes = 200;
    int rightAngles = 0;
    int angleBuffer = 400;
    int sumDistance = 0;
    int sumAngles = 0;
    int sumVotes = 0;
    int longestIsStraightAhead = 0;

    // find angle closest to 0
    Line straightAhead;
    straightAhead.theta = (int)(2 * PI * 1000.0);
    for (int l = 0; l < nWalls; ++l) {
        straightAhead = abs(robot_s->houghBestLines[l].theta) < abs(straightAhead.theta) ?
                             robot_s->houghBestLines[l] : straightAhead;
    }

    // sum the distances of the lines and norm angles
    for (int k = 0; k < nWalls; ++k) {
        sumDistance += robot_s->houghBestLines[k].rho;
        robot_s->houghBestLines[k].theta -= straightAhead.theta;
        sumVotes += robot_s->houghBestLines[k].votes;
    }

    for (int k = 0; k < nWalls; ++k) {
        sumAngles += robot_s->houghBestLines[k].theta;
    }

    // check the angles of the lines are square
    if (sumAngles > -angleBuffer && sumAngles < angleBuffer) {
        rightAngles = 1;
    }

    printf("sum dist: %d, nrmAngle: %d sumAngles: %d\n",sumDistance,straightAhead.theta,sumAngles);
    // change state if we are in the elevator
    if (robot_s->houghBestLines[0].rho != -999 &&
//            sumDistance < maxTotalDistance &&
//            sumDistance > minTotalDistance &&
            sumVotes > minTotalVotes &&
            straightAhead.votes > 100 &&
            rightAngles) {
        // TODO change state to in elevator
        printf("in elevator\n");
        robot_s->task = IDLE;
        *v = 0;
        *w = 0;
        return;
    }

    // then, compute the motor commands for following the person
    int medianX;
    int numHist = 8;
    int temp;
    int i, j;

    // sort the location points in ascending order by x
    for(i = 0; i < numHist-1; i++) {

        for(j = i + 1; j < numHist; j++) {
            if(robot_s->SVMData->location[j][0] < robot_s->SVMData->location[i][0]) {
                // swap elements
                temp = robot_s->SVMData->location[i][0];
                robot_s->SVMData->location[i][0] = robot_s->SVMData->location[j][0];
                robot_s->SVMData->location[j][0] = temp;
            }
        }
    }

    medianX = robot_s->SVMData->location[numHist/2][0];

    int window = 50;
    if (robot_s->SVMData->numLocations == 0) {
//        robot_s->task = QUIT;
        *v = 0;
        *w = 0;
        return;
    }
    float kpd = 0.5;
    float kpo = 1.0;
    // rotate to face person
    int diffOri = robot_s->goalOrientation - medianX;
    // maintain constant distance to object under person
    int minDist = 8000;
    int lookNearIndex = NUM_LASERS/2;
    for (i = lookNearIndex - window; i < lookNearIndex + window; ++i) {
        minDist = robot_s->laser[i] < minDist && robot_s->laser[i] != 0 ? robot_s->laser[i] : minDist;
    }
    printf("found min dist: %d near index %d\n",minDist,lookNearIndex);
    int diffDist = 0;
    if (minDist != 8000) {
        diffDist =  minDist - robot_s->goalDistance;
    }
    printf("IPCLostCount %d\n", robot_s->IPCLostCount);
    // if we haven't seen it in a while
    if (robot_s->IPCLostCount >= 10) {
        *v = 0;
        kpo *= 2.0;
    } else {
        *v = (int) (diffDist * kpd);
//         *v = 0;
    }
    *w = (int)(diffOri*kpo);
//     *w = 0;
}

// This stops a robot
void handle_quit(Robot* robot_s, int* v, int *w) {
    *v = 0;
    *w = 0;
}
