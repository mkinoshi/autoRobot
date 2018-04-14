/**
 * Created by Bruce Maxwell
 * Edited by Matt Martin, Makoto Kinoshita
 * Feb 26, 2017
 * CS363 Robotics
 * Map header file
 */

#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H


#include "ppmIO.h"

typedef struct {
    int rows;
    int cols;
    int size[2];
    int intensities;
    float gridSize;
    float origin[2];
    unsigned char *grid;
} Map;

Map *map_read(char *filename);
void map_free( Map *map );
int map_get( Map *map, float x, float y );
int map_getraw( Map *map, int c, int r );
void map_set( Map *map, float x, float y, unsigned char val );
void map_line( Map *map, float xf0, float yf0, float xf1, float yf1, Pixel *src, Pixel value);
void map_setPix( Map *map, Pixel *pix );
void map_xy2pix( Map *map, float x, float y, int *col, int *row );


#endif
