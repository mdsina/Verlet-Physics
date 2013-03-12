#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glut.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

#include "Vector.h"
#include "Physics.h"

#define MIN( A, B ) ( (A) < (B) ? (A) : (B) ) 
#define MAX( A, B ) ( (A) > (B) ? (A) : (B) )
#define SGN( A ) ( (A) < 0 ? (-1) : (1) )

extern int GWidth; 
extern int GHeight;

extern Physics World;

#endif
