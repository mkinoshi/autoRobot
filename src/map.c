/**
 * Created by Bruce Maxwell
 * Edited by Matt Martin, Makoto Kinoshita
 * April, 2017
 * CS363 Robotics
 * Map file
 */

#include <stdio.h>
#include <stdlib.h>
#include <map.h>
#include "map.h"
#include "ppmIO.h"


// read a map from a PGM file
Map *map_read(char *filename) {
    Map *map;

    map = (Map *)malloc(sizeof(Map));
    if(!map) {
        printf("map_read: unable to malloc map\n");
        return(NULL);
    }

    map->grid = readPGM( &(map->rows), &(map->cols), &(map->intensities), filename );
    if( !map->grid ) {
        printf("map_read: unable to read map %s\n", filename);
        free(map);
        return(NULL);
    }

    // need to read these from the yaml file
    map->gridSize = 0.05;
    map->origin[0] = -100.0 + 1750.0*map->gridSize;
    map->origin[1] = -100.0 + 1750.0*map->gridSize; // convert the map to Cartesian coords so (x, y) in world space is x right, y up
    map->size[0] = map->cols * map->gridSize;
    map->size[1] = map->rows * map->gridSize;

    return( map );
}

// free all malloc'd memory in the map structure
void map_free( Map *map ) {
    if( map ) {
        if( map->grid )
            free(map->grid);
        free(map);
    }
}

/*
	Takes in an (x, y) in world coords and returns the corresponding pixel value
 */
int map_get( Map *map, float x, float y ) {
    int r, c;
    map_xy2pix( map, x, y, &c, &r );
    return( map->grid[ r * map->cols + c ] );
}

/*
	Takes in the raw row, column values
 */
int map_getraw( Map *map, int c, int r ) {
    return( map->grid[ r * map->cols + c ] );
}

/*
	Takes in an (x, y) in world coords and sets the corresponding pixel value
 */
void map_set( Map *map, float x, float y, unsigned char val ) {
    int r, c;
    map_xy2pix( map, x, y, &c, &r );
    map->grid[ r * map->cols + c ] = val;
}

/*
	Takes in an (x, y) in world coords and returns the corresponding row/col of the map
 */
void map_xy2pix( Map *map, float x, float y, int *col, int *row ) {
    *col = (int)((x - map->origin[0]) / map->gridSize  + 0.5);
    *row = (int)(-(map->origin[1] + y) / map->gridSize  + 0.5);
}

// copy the map data over to the pixels
void map_setPix( Map *map, Pixel *pix ) {
    int i;

    for(i=0;i<map->rows*map->cols;i++) {
        if (map->grid[i] == 50) {
            pix[i].r = 0; pix[i].g = 0; pix[i].b = 200;
        } else if (map->grid[i] == 49) {
            pix[i].r = 0; pix[i].g = 250; pix[i].b = 0;
        } else {
            pix[i].r = pix[i].g = pix[i].b = map->grid[i];
        }
    }
}

// takes in two (x, y) locations in world coords and draws a line in the src image
// draws a line that includes the two endpoint pixels
// draws into src using the color value
void map_line( Map *map, float xf0, float yf0, float xf1, float yf1, Pixel *src, Pixel value) {
    int x0, y0, x1, y1;
    int x, y;
    int p;
    int dx, dy;
    int twoDx, twoDy;
    int xstep, ystep;
    int i;

    map_xy2pix( map, xf0, yf0, &x0, &y0 );
    map_xy2pix( map, xf1, yf1, &x1, &y1 );

    if(x0 < 0 || x0 >= map->cols || x1 < 0 || x1 >= map->cols || y0 < 0 || y0 >= map->rows || y1 < 0 || y1 >= map->rows)
        return;

    dx = x1 - x0;
    dy = y1 - y0;
    x = x0;
    y = y0;
    xstep = dx < 0 ? -1 : 1;
    ystep = dy < 0 ? -1 : 1;


    // horizontal and vertical lines
    if(dx == 0) {
        if(dy == 0) {
            src[y*map->cols + x] = value;
            return;
        }
        for(; y != y1;y+=ystep) {
            src[y*map->cols + x] = value;
        }
        return;
    }
    if(dy == 0) {
        for(;x!=x1;x+=xstep) {
            src[y*map->cols + x] = value;
        }
        return;
    }

    twoDx = abs(dx*2);
    twoDy = abs(dy*2);

    if( twoDx > twoDy ) {
        p = twoDy - abs(dx);
        for(;x!=x1;x+=xstep) {
            src[y*map->cols + x] = value;
            if(p > 0) {
                y += ystep;
                p -= twoDx;
            }
            p += twoDy;
        }
    }
    else {
        p = twoDx - abs(dy);
        for(;y!=y1;y+=ystep) {
            src[y*map->cols + x] = value;
            if(p > 0) {
                x += xstep;
                p -= twoDy;
            }
            p += twoDx;
        }
    }

    return;
}
