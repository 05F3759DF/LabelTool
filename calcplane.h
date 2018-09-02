#ifndef CALCPLANE_H
#define CALCPLANE_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <memory.h>
#include <math.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

void FindScatterMatrix(double Centroid[3], int Points_Total,
                       double *X_Coord, double *Y_Coord, double *Z_Coord,
                       double ScatterMatrix[3][3], int Order[3]);
void tred2(double a[3][3], double d[3], double e[3]);
void tqli(double d[3], double e[3], double z[3][3]);

void CalculatePlane(int Points_Total, double *X_Coord, double *Y_Coord, double *Z_Coord,
                    int Origin_Flag, double Plane_Eq[4]);

void CalculateResiduals(double *X, double *Y, double *Z, double Equation[4],
                        double *Error, int PointsTotal);
#endif // CALCPLANE_H
