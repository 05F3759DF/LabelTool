#include "coordinatetransform.h"
#include <cmath>
void X::rMatrixInit(Matrix &rt) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (i == j) {
                rt[i][j] = 1;
            } else {
                rt[i][j] = 0;
            }
}

void X::rMatrixmulti(Matrix &r, Matrix &rt) {
    double rin[3][3];
    int i, j;

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            rin[i][j] = r[i][j];

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            r[i][j] = rin[i][0] * rt[0][j] +
                      rin[i][1] * rt[1][j] +
                      rin[i][2] * rt[2][j];
        }
}

void X::createRotMatrix_ZYX(Matrix &rt, double rotateX, double rotateY, double rotateZ) {
    double sinx, siny, sinz, cosx, cosy, cosz;
    double rr[3][3];
    int i, j;

    sinx = sin(rotateX);
    siny = sin(rotateY);
    sinz = sin(rotateZ);
    cosx = cos(rotateX);
    cosy = cos(rotateY);
    cosz = cos(rotateZ);

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            if (i == j) {
                rt[i][j] = 1;
            } else {
                rt[i][j] = 0;
            }

    if (rotateZ != 0.0) {
        /*	R3 :   cosz  -sinz   0.0
                   sinz  cosz   0.0
                   0.0   0.0   1.0
         */
        rr[0][0] = cosz;
        rr[0][1] = -sinz;
        rr[0][2] = 0.0;
        rr[1][0] = sinz;
        rr[1][1] = cosz;
        rr[1][2] = 0.0;
        rr[2][0] = 0.0;
        rr[2][1] = 0.0;
        rr[2][2] = 1.0;
        rMatrixmulti(rt, rr);
    }

    if (rotateY != 0.0) {
        /*	R2 :   cosy   0.0  siny
                    0.0   1.0   0.0
                  -siny   0.0  cosy
         */
        rr[0][0] = cosy;
        rr[0][1] = 0.0;
        rr[0][2] = siny;
        rr[1][0] = 0.0;
        rr[1][1] = 1.0;
        rr[1][2] = 0.0;
        rr[2][0] = -siny;
        rr[2][1] = 0.0;
        rr[2][2] = cosy;
        rMatrixmulti(rt, rr);
    }

    if (rotateX != 0.0) {
        /*	R1 :	1.0   0.0   0.0
                    0.0  cosx  -sinx
                    0.0  sinx  cosx
         */
        rr[0][0] = 1.0;
        rr[0][1] = 0.0;
        rr[0][2] = 0.0;
        rr[1][0] = 0.0;
        rr[1][1] = cosx;
        rr[1][2] = -sinx;
        rr[2][0] = 0.0;
        rr[2][1] = sinx;
        rr[2][2] = cosx;
        rMatrixmulti(rt, rr);
    }
}

void X::createRotMatrix_XYZ(Matrix &rt, double rotateX, double rotateY, double rotateZ) {
    double sinx, siny, sinz, cosx, cosy, cosz;
    double rr[3][3];
    int i, j;

    sinx = sin(rotateX);
    siny = sin(rotateY);
    sinz = sin(rotateZ);
    cosx = cos(rotateX);
    cosy = cos(rotateY);
    cosz = cos(rotateZ);

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            if (i == j) {
                rt[i][j] = 1;
            } else {
                rt[i][j] = 0;
            }

    if (rotateX != 0.0) {
        /*	R1 :	1.0   0.0   0.0
                    0.0  cosx  -sinx
                    0.0  sinx  cosx
         */
        rr[0][0] = 1.0;
        rr[0][1] = 0.0;
        rr[0][2] = 0.0;
        rr[1][0] = 0.0;
        rr[1][1] = cosx;
        rr[1][2] = -sinx;
        rr[2][0] = 0.0;
        rr[2][1] = sinx;
        rr[2][2] = cosx;
        rMatrixmulti(rt, rr);
    }

    if (rotateY != 0.0) {
        /*	R2 :   cosy   0.0 siny
                    0.0   1.0   0.0
                   -siny   0.0  cosy
         */
        rr[0][0] = cosy;
        rr[0][1] = 0.0;
        rr[0][2] = siny;
        rr[1][0] = 0.0;
        rr[1][1] = 1.0;
        rr[1][2] = 0.0;
        rr[2][0] = -siny;
        rr[2][1] = 0.0;
        rr[2][2] = cosy;
        rMatrixmulti(rt, rr);
    }

    if (rotateZ != 0.0) {
        /*	R3 :   cosz  -sinz   0.0
                   sinz cosz   0.0
                    0.0   0.0   1.0
         */
        rr[0][0] = cosz;
        rr[0][1] = -sinz;
        rr[0][2] = 0.0;
        rr[1][0] = sinz;
        rr[1][1] = cosz;
        rr[1][2] = 0.0;
        rr[2][0] = 0.0;
        rr[2][1] = 0.0;
        rr[2][2] = 1.0;
        rMatrixmulti(rt, rr);
    }
}

// Mei Jilin 20160716
void X::createRotMatrix_XYZ_Inv(Matrix &rt, double rotateX, double rotateY, double rotateZ) {
    double sinx, siny, sinz, cosx, cosy, cosz;
    double rr[3][3];
    int i, j;

    sinx = sin(rotateX);
    siny = sin(rotateY);
    sinz = sin(rotateZ);
    cosx = cos(rotateX);
    cosy = cos(rotateY);
    cosz = cos(rotateZ);

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            if (i == j) {
                rt[i][j] = 1;
            } else {
                rt[i][j] = 0;
            }

    if (rotateX != 0.0) {
        /*	R1 :	1.0   0.0   0.0
                    0.0  cosx  sinx
                    0.0  -sinx  cosx
         */
        rr[0][0] = 1.0;
        rr[0][1] = 0.0;
        rr[0][2] = 0.0;
        rr[1][0] = 0.0;
        rr[1][1] = cosx;
        rr[1][2] = sinx;
        rr[2][0] = 0.0;
        rr[2][1] = -sinx;
        rr[2][2] = cosx;
        rMatrixmulti(rt, rr);
    }

    if (rotateY != 0.0) {
        /*	R2 :   cosy   0.0  -siny
                    0.0   1.0   0.0
                   siny   0.0  cosy
         */
        rr[0][0] = cosy;
        rr[0][1] = 0.0;
        rr[0][2] = -siny;
        rr[1][0] = 0.0;
        rr[1][1] = 1.0;
        rr[1][2] = 0.0;
        rr[2][0] = siny;
        rr[2][1] = 0.0;
        rr[2][2] = cosy;
        rMatrixmulti(rt, rr);
    }

    if (rotateZ != 0.0) {
        /*	R3 :   cosz  sinz   0.0
                   -sinz cosz   0.0
                    0.0   0.0   1.0
         */
        rr[0][0] = cosz;
        rr[0][1] = sinz;
        rr[0][2] = 0.0;
        rr[1][0] = -sinz;
        rr[1][1] = cosz;
        rr[1][2] = 0.0;
        rr[2][0] = 0.0;
        rr[2][1] = 0.0;
        rr[2][2] = 1.0;
        rMatrixmulti(rt, rr);
    }
}

void X::shiftPoint3d(Point3d &pt, Point3d &sh) {
    Point3d p;

    p.x = pt.x + sh.x;
    p.y = pt.y + sh.y;
    p.z = pt.z + sh.z;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
}

void X::rotatePoint3d(Point3d &pt, Matrix &a) {
    Point3d p;

    p.x = a[0][0] * pt.x + a[0][1] * pt.y + a[0][2] * pt.z;
    p.y = a[1][0] * pt.x + a[1][1] * pt.y + a[1][2] * pt.z;
    p.z = a[2][0] * pt.x + a[2][1] * pt.y + a[2][2] * pt.z;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
}

void X::normalPoint3d(Point3d &pt) {
    double dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    pt.x /= dist;
    pt.y /= dist;
    pt.z /= dist;
}
