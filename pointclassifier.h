#ifndef POINTCLASSIFIER_H
#define POINTCLASSIFIER_H

#include <vector>

#include <opencv2/opencv.hpp>

#include "coordinatetransform.h"
#include "calcplane.h"

#define PNTS_PER_LINE           32
#define LINES_PER_BLK           12
#define PTNUM_PER_BLK           (32 * 12)
#define BKNUM_PER_FRM           180

#define HORIERRFACTOR 0.02
#define VERTERRFACTOR 0.05
#define BASEERROR    0.1
#define MAXSMOOTHERR 1.0
#define MAXDISTHRE   2.0

#define UNKNOWN                 0
#define NONVALID                -9999
#define EDGEPT                  -9

class SegBuf {
public:
    unsigned short lab;
    cv::Point2i dmin;
    cv::Point2i dmax;
    X::Point3d maxxp, maxyp, maxzp;
    X::Point3d minxp, minyp, minzp;
    X::Point3d cp;
    int ptnum;
    X::Point3d norm;
    double var;
};

class RMap {
public:
    int width;
    int length;
    X::Point3d *pts;
    int *regionID;
    int regnum;
    SegBuf *segbuf;
    cv::Mat rMap;
    cv::Mat lMap;
};

class PointClassifier {
public:
    PointClassifier();
    std::vector<std::vector<cv::Point3d>> pointcloud;
    RMap rmap;
    void setPointCloud(std::vector<std::vector<cv::Point3d>> points);
    void initRMap();
    void smoothingData();
    void contourExtraction();
    unsigned int regionGrow();
    void growOne(cv::Point2i seed, unsigned int _regid);
    bool addPoints(cv::Point2i seed, std::vector<cv::Point2i> &vec, int _regid);
    void region2Seg();
};

#endif // POINTCLASSIFIER_H
