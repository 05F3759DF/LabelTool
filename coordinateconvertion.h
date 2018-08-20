// CoordinateConvertion.h: interface for the CCoordinateConvertion class.
//
//////////////////////////////////////////////////////////////////////
#ifndef COORDINATE_CONVERTION_H
#define COORDINATE_CONVERTION_H

#ifndef DOXYGEN_SHOULD_SKIP_THIS

class CoordinateConvertion {
public:
    CoordinateConvertion();
    ~CoordinateConvertion();
    typedef enum {
        _BEJING54 = 0, _BEIJINGLOCAL, _SHANGHAI, _TOKYO9K, _NAGOYA7K, _SELFDEF
    } COORDINATESYSTEMS;
    double DEF_A1 = 6378137.0;
    double DEF_B1 = 6356752.3;

    double DEF_A2 = 6377397.155;
    double DEF_B2 = 6356078.963;

    double DEF_B0 = 40.0;
    double DEF_L0 = 116.0;

    double GPSHEIGHT = 0;

public:
    int Local_WGS84(double *Local);
    int WGS84_Local(double *WGS84);
    int XYZ_LBH(double *xyz, double *lbh, double a, double b);
    int LBH_XYZ(double *lbh, double *xyz, double a, double b);
    int LocalXYZ_LBH(double *xyz, double *lbh);
    int LocalLBH_XYZ(double *lbh, double *xyz); // In fact, l=latitude, b=longtitude, h=altitude, x=coor_y, y=coor_x, z=coor_z
    int CenterXYZ_TangentPlaneXYZ(double *lbh0, double *xyz0, double *xyz);
    int TangentPlaneXYZ_CenterXYZ(double *lbh0, double *xyz0, double *xyz);
    void GPS84LBH_LocalXYZ(double *lbh, double *XYZ);
    void LocalXYZ_GPS84LBH(double *XYZ, double *lbh);
    void SetCoordinateParameters(COORDINATESYSTEMS coordsys, double lat = 0, double lon = 0);
};
#endif
#endif // !defined(AFX_COORDINATECONVERTION_H__E4FD82EB_782B_474C_A802_5BE18D7E973A__INCLUDED_)
