#include "pointclassifier.h"

PointClassifier::PointClassifier() {}

void PointClassifier::setPointCloud(std::vector<std::vector<X::Point3d>> points) {
    pointcloud = points;
    initRMap();

    for (int i = 0; i < rmap.width * rmap.length; i++) {
        rmap.pts[i].x = rmap.pts[i].y = rmap.pts[i].z = 0;
        rmap.regionID[i] = UNKNOWN;
    }
    rmap.regnum = 0;

    // TODO: regist point to rmap
    for (int k = 0; k < 32; k++) {
        for (int i = 0; i < pointcloud[k].size(); i++) {
            rmap.pts[k * rmap.width + i] = X::Point3d(pointcloud[k][i].x, pointcloud[k][i].y, pointcloud[k][i].z);
        }
    }

    smoothingData();

    contourExtraction();
    rmap.regnum = regionGrow() + 1;

    if (rmap.regnum) {
        rmap.segbuf = new SegBuf[rmap.regnum];
        region2Seg();
    }

    rmap.rMap.setTo(0);            // 距离图像
    rmap.lMap.setTo(0);            // 分割图像
    //按分割分类结果赋予可视化位图颜色
    for (int y = 0; y < rmap.length; y++) {
        for (int x = 0; x < rmap.width; x++) {
            if (!rmap.pts[y * rmap.width + x].i) {
                rmap.rMap.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                continue;
            }
            if (rmap.pts[y * rmap.width + x].z < 0.0) {
                rmap.rMap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, BOUND(NINT(-rmap.pts[y * rmap.width + x].z * 100.0), 0, 255));
            } else {
                rmap.rMap.at<cv::Vec3b>(y, x) = cv::Vec3b(BOUND(NINT(-rmap.pts[y * rmap.width + x].z * 100.0), 0, 255), 0, 0);
            }
            if (rmap.regionID[y * rmap.width + x] == EDGEPT) {
                rmap.lMap.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 128, 128);
            } else if (rmap.regionID[y * rmap.width + x] == NONVALID) {
                rmap.lMap.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            } else if (rmap.regionID[y * rmap.width + x] == UNKNOWN) {
                rmap.lMap.at<cv::Vec3b>(y, x) = cv::Vec3b(64, 64, 64);
            } else {
                SegBuf *segbuf = &rmap.segbuf[rmap.regionID[y * rmap.width + x]];
                if (segbuf->ptnum) {
                    rmap.lMap.at<cv::Vec3b>(y, x) =
                        cv::Vec3b(NINT(fabs(segbuf->norm.x) * 255.0), NINT(fabs(segbuf->norm.y) * 255.0), NINT(segbuf->norm.z * 255.0));
                }
            }
        }
    }
    cv::resize(rmap.rMap, rmap.rMap, cv::Size(1024, 96));
    cv::resize(rmap.lMap, rmap.lMap, cv::Size(1024, 96));
    cv::imwrite("rmap.png", rmap.rMap);
    cv::imwrite("lmap.png", rmap.lMap);
}

void PointClassifier::initRMap() {
    rmap.width = pointcloud[0].size();
    rmap.length = 32;
    rmap.pts = new X::Point3d[rmap.width * rmap.length];
    rmap.regionID = new int[rmap.width * rmap.length];
    rmap.segbuf = NULL;
    rmap.rMap = cv::Mat(rmap.length, rmap.width, CV_8UC3);
    rmap.lMap = cv::Mat(rmap.length, rmap.width, CV_8UC3);
}

void PointClassifier::smoothingData() {
    int maxcnt = 3;
    for (int y = 0; y < rmap.length; y++) {
        for (int x = 1; x < (rmap.width - 1); x++) {
            if (rmap.pts[y * rmap.width + (x - 1)].i && !rmap.pts[y * rmap.width + x].i) {

                int xx;
                for (xx = x + 1; xx < rmap.width; xx++) {
                    if (rmap.pts[y * rmap.width + xx].i) {
                        break;
                    }
                }
                if (xx >= rmap.width) {
                    continue;
                }
                int cnt = xx - x + 1;
                if (cnt > maxcnt) {
                    x = xx;
                    continue;
                }
                X::Point3d p1 = rmap.pts[y * rmap.width + (x - 1)];
                X::Point3d p2 = rmap.pts[y * rmap.width + xx];
                double dis = X::distance(p1, p2);
                double rng = std::max(X::distance(p1), X::distance(p2));
                double maxdis = std::min(MAXSMOOTHERR, std::max(BASEERROR, HORIERRFACTOR * cnt * rng));
                if (dis < maxdis) {
                    for (int xxx = x; xxx < xx; xxx++) {
                        X::Point3d &p = rmap.pts[y * rmap.width + xxx];
                        p.x = (p2.x - p1.x) / cnt * (xxx - x + 1) + p1.x;
                        p.y = (p2.y - p1.y) / cnt * (xxx - x + 1) + p1.y;
                        p.z = (p2.z - p1.z) / cnt * (xxx - x + 1) + p1.z;
                        p.i = 1;
                    }
                }
                x = xx;
            }
        }
    }
}

void PointClassifier::contourExtraction() {
    X::Point3d cp, pt;
    int x, y, xx, yy;

    for (x = 0; x < rmap.length; x++) {
        for (y = 0; y < rmap.width; y++) {
            cp = rmap.pts[rmap.width * x + y];
            if (!cp.i) {
                rmap.regionID[rmap.width * x + y] = NONVALID;
                continue;
            }

            for (yy = y - 1; yy <= y + 1; yy++) {
                if (yy < 0 || yy >= rmap.width) {
                    continue;
                }
                for (xx = x - 1; xx <= x + 1; xx++) {
                    if (xx < 0 || xx >= rmap.length) {
                        continue;
                    }
                    if (xx != x && yy != y) {
                        continue;
                    }

                    pt = rmap.pts[rmap.width * xx + yy];
                    if (!pt.i) {
                        continue;
                    }

                    bool ishori = yy != y;
                    if (!X::isNeighbor(pt, cp, ishori)) {
                        rmap.regionID[rmap.width * x + y] = EDGEPT;
                        break;
                    }
                }
                if (rmap.regionID[rmap.width * x + y] == EDGEPT) {
                    break;
                }
            }
        }
    }
}

unsigned int PointClassifier::regionGrow() {
    cv::Point2i seed;
    unsigned int _regid = 1;

    int x, y, yf;
    for (x = 0; x < rmap.length; x++) {
        for (yf = 0; yf < rmap.width; yf++) {
            if (rmap.pts[rmap.width * x + yf].i) {
                break;
            }
        }
        if (yf >= rmap.width) {
            continue;
        }
        for (y = yf; y < rmap.width; y++) {
            if (rmap.regionID[rmap.width * x + y] == UNKNOWN && rmap.pts[rmap.width * x + y].z > -3 && rmap.pts[rmap.width * x + y].z < 3) {
                seed.x = x;
                seed.y = y;
                growOne(seed, _regid);
                _regid++;
            }
        }
    }

    return _regid - 1;
}

void PointClassifier::growOne(cv::Point2i seed, unsigned int _regid) {
    std::vector<cv::Point2i> vec1;
    std::vector<cv::Point2i> vec2;
    bool isVec1 = true;
    int j;
    vec1.clear();
    vec2.clear();
    vec1.push_back(seed);
    while (1) {
        if (isVec1) {
            vec2.clear();
            for (j = 0; j < vec1.size(); j++) {
                addPoints(vec1[j], vec2, _regid);
            }
            isVec1 = !isVec1;
            if (vec2.size() < 1) {
                return;
            }
        } else {
            vec1.clear();
            for (j = 0; j < vec2.size(); j++) {
                addPoints(vec2[j], vec1, _regid);
            }
            isVec1 = !isVec1;
            if (vec1.size() < 1) {
                return;
            }
        }
    }
}

bool PointClassifier::addPoints(cv::Point2i seed, std::vector<cv::Point2i> &vec, int _regid) {
    if (rmap.regionID[rmap.width * seed.x + seed.y] == UNKNOWN) {
        rmap.regionID[rmap.width * seed.x + seed.y] = _regid;
    } else if (rmap.regionID[rmap.width * seed.x + seed.y] != _regid) {
        return false;
    }

    cv::Point2i neighbor;

    for (int k = 0; k < 4; k++) {
        double errfactor;
        switch (k) {
        case 0: neighbor.x = seed.x - 1; neighbor.y = seed.y; errfactor = VERTERRFACTOR; break;
        case 1: neighbor.x = seed.x + 1; neighbor.y = seed.y; errfactor = VERTERRFACTOR; break;
        case 2: neighbor.x = seed.x; neighbor.y = seed.y - 1; errfactor = HORIERRFACTOR; break;
        case 3: neighbor.x = seed.x; neighbor.y = seed.y + 1; errfactor = HORIERRFACTOR; break;
//		case 4:	Neighbor.x = seed.x - 1; Neighbor.y = seed.y - 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
//		case 5: Neighbor.x = seed.x + 1; Neighbor.y = seed.y + 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
//		case 6: Neighbor.x = seed.x + 1; Neighbor.y = seed.y - 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
//		case 7: Neighbor.x = seed.x - 1; Neighbor.y = seed.y + 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
        }

        if (neighbor.x >= 0 && neighbor.x < rmap.length &&
            neighbor.y >= 0 && neighbor.y < rmap.width &&
            rmap.pts[rmap.width * neighbor.x + neighbor.y].i &&
            rmap.regionID[rmap.width * neighbor.x + neighbor.y] == UNKNOWN) {
            vec.push_back(neighbor);
            rmap.regionID[rmap.width * neighbor.x + neighbor.y] = _regid;
        }
    }
    return true;
}

void PointClassifier::region2Seg() {
    SegBuf *segbuf;
    int regionid;

    for (regionid = 0; regionid < rmap.regnum; regionid++) {
        segbuf = &rmap.segbuf[regionid];
        segbuf->minxp.x = segbuf->minxp.y = segbuf->minxp.z = 9999.0;
        segbuf->minyp.x = segbuf->minyp.y = segbuf->minyp.z = 9999.0;
        segbuf->minzp.x = segbuf->minzp.y = segbuf->minzp.z = 9999.0;
        segbuf->maxxp.x = segbuf->maxxp.y = segbuf->maxxp.z = -9999.0;
        segbuf->maxyp.x = segbuf->maxyp.y = segbuf->maxyp.z = -9999.0;
        segbuf->maxzp.x = segbuf->maxzp.y = segbuf->maxzp.z = -9999.0;
        segbuf->cp.x = segbuf->cp.y = segbuf->cp.z = 0;
        segbuf->dmin.x = rmap.width;
        segbuf->dmin.y = rmap.length;
        segbuf->ptnum = 0;
    }

    for (int y = 0; y < rmap.length; y++) {
        for (int x = 0; x < rmap.width; x++) {
            if (!rmap.pts[y * rmap.width + x].i) {
                continue;
            }
            regionid = rmap.regionID[y * rmap.width + x];
            if (regionid <= 0 || regionid >= rmap.regnum) {
                continue;
            }

            segbuf = &rmap.segbuf[regionid];

            segbuf->cp.x += rmap.pts[y * rmap.width + x].x;
            segbuf->cp.y += rmap.pts[y * rmap.width + x].y;
            segbuf->cp.z += rmap.pts[y * rmap.width + x].z;

            segbuf->dmin.x = std::min(segbuf->dmin.x, x);
            segbuf->dmin.y = std::min(segbuf->dmin.y, y);
            segbuf->dmax.x = std::max(segbuf->dmax.x, x);
            segbuf->dmax.y = std::max(segbuf->dmax.y, y);

            if (segbuf->minxp.x > rmap.pts[y * rmap.width + x].x) {
                segbuf->minxp = rmap.pts[y * rmap.width + x];
            }
            if (segbuf->minyp.y > rmap.pts[y * rmap.width + x].y) {
                segbuf->minyp = rmap.pts[y * rmap.width + x];
            }
            if (segbuf->minzp.z > rmap.pts[y * rmap.width + x].z) {
                segbuf->minzp = rmap.pts[y * rmap.width + x];
            }
            if (segbuf->maxxp.x < rmap.pts[y * rmap.width + x].x) {
                segbuf->maxxp = rmap.pts[y * rmap.width + x];
            }
            if (segbuf->maxyp.y < rmap.pts[y * rmap.width + x].y) {
                segbuf->maxyp = rmap.pts[y * rmap.width + x];
            }
            if (segbuf->maxzp.z < rmap.pts[y * rmap.width + x].z) {
                segbuf->maxzp = rmap.pts[y * rmap.width + x];
            }
            segbuf->ptnum++;
        }
    }

    double Equation[4];
    double WX[6], WY[6], WZ[6];
    int num;
    for (regionid = 0; regionid < rmap.regnum; regionid++) {
        segbuf = &rmap.segbuf[regionid];
        if (segbuf->ptnum < 100) {
            segbuf->ptnum = 0;
            continue;
        }
        segbuf->cp.x /= (double)segbuf->ptnum;
        segbuf->cp.y /= (double)segbuf->ptnum;
        segbuf->cp.z /= (double)segbuf->ptnum;

        WX[0] = segbuf->minxp.x; WX[1] = segbuf->minyp.x; WX[2] = segbuf->minzp.x; WX[3] = segbuf->maxxp.x; WX[4] = segbuf->maxyp.x; WX[5] = segbuf->maxzp.x;
        WY[0] = segbuf->minxp.y; WY[1] = segbuf->minyp.y; WY[2] = segbuf->minzp.y; WY[3] = segbuf->maxxp.y; WY[4] = segbuf->maxyp.y; WY[5] = segbuf->maxzp.y;
        WZ[0] = segbuf->minxp.z; WZ[1] = segbuf->minyp.z; WZ[2] = segbuf->minzp.z; WZ[3] = segbuf->maxxp.z; WZ[4] = segbuf->maxyp.z; WZ[5] = segbuf->maxzp.z;
        num = 6;
        CalculatePlane(num, WX, WY, WZ, 0, Equation);
        CalculateResiduals(&segbuf->cp.x, &segbuf->cp.y, &segbuf->cp.z, Equation, &segbuf->var, 1);

        if (Equation[2] < 0) {
            segbuf->norm.x = -Equation[0];
            segbuf->norm.y = -Equation[1];
            segbuf->norm.z = -Equation[2];
        } else {
            segbuf->norm.x = Equation[0];
            segbuf->norm.y = Equation[1];
            segbuf->norm.z = Equation[2];
        }
        if (segbuf->norm.z < 0.9) {
            segbuf->ptnum = 0;
        } else {
            segbuf->ptnum = segbuf->ptnum;
        }
    }
}
