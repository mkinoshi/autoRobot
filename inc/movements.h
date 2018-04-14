/**
 * Matt Martin, Makoto Kinoshita
 * Feb 26, 2017
 * CS363 Robotics
 * Movementes header file
 */

#include "robot.h"

#ifndef MOVEMENTS_H
#define MOVEMENTS_H

void handle_test(Robot* robot_s, int* v, int* w);

void handle_point_follow(Robot* robot_s, int* v, int* w);

void handle_idle(Robot* robot_s, int* v, int *w);

void handle_wall_find(Robot* robot_s, int* v, int *w);

void handle_wall_follow(Robot* robot_s, int* v, int *w);

void handle_maze(Robot* robot_s, int* v, int*w);

void handle_rotate(Robot* robot_s, int *v, int *w);

void handle_wander(Robot* robot_s, int*v, int*w);

void handle_go_to(Robot* robot_s, int*v, int*w);

void handle_quit(Robot* robot_s, int* v, int *w);

void handle_pink_find(Robot *robot_s, int* v, int* w);

void handle_pink_track(Robot *robot_s, int* v, int* w);

void handle_hough_wall_follow(Robot *robot_s, int* v, int* w);

void handle_face_find(Robot *robot_s, int* v, int* w);

void handle_face_track(Robot *robot_s, int* v, int* w);

void handle_take_picture(Robot *robot_s, int* v, int* w);

void handle_localize(Robot *robot_s, int*v, int*w);

void handle_wait_for_face(Robot *robot_s, int*v, int*w);

void handle_follow_person(Robot*, int*, int*);

#endif
