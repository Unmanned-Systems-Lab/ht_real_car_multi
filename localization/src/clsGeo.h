#ifndef _CLSGEO_H_
#define _CLSGEO_H_
#include <math.h>
#include <stdio.h>

#ifndef PI
#define PI (3.1415926f)
#endif

#define  TORAD(x)   ( (x) * PI / 180.0 )   // 角度转弧度
#define  TOANG(x)   ( (x) * 180.0/PI   )   // 弧度转角度

#define bool int
#define true 1
#define false 0



/*
使用说明：
（1）ENUM_ZONEDEGRRE "zd">6度分带或3度分带</param>，墨卡托（UTM）投影下只能选择6分带
（2）"isYhasZid">Y坐标是否包含分带号</param> 只能选择否 0
（3）求出来的绝对坐标X，Y是反的，即X坐标指向正北；Y坐标指向正南

void GetDistAngleFromGpsByUTM( );函数说明：
（1）精度高，但是两点之间的距离不能太大，不能跨两个以上的分带（测试）;
(2) 不能跨坐标域，即两点位置跨过赤道或者零度子午线。
*/

//椭圆参数
typedef struct ellipsoid
{
    double semimajorAxis;//长半轴
    double semiminorAxis;//短半轴
    double inverseFlattening;//扁率
    double eccentricity2_1st;//第一偏心率平方
    double eccentricity2_2nd;//第二偏心率平方

}STR_ELLIPSOIP;

/*
semimajorAxis;//长半轴                 6378137
semiminorAxis;//短半轴                 6356752.3142
inverseFlattening;//扁率               0.0033528106643316
eccentricity2_1st;//第一偏心率平方      0.00669437999013
eccentricity2_2nd;//第二偏心率平方      0.006739496742227
*/


typedef enum zoneDegree
{
    threeDegree = 3,
    sixDegree = 6,
}ENUM_ZONEDEGRRE;


typedef enum Latitude
{
    northen,
    southern,
}ENUM_LATITUDE;


typedef enum Longitude
{
    eastern,
    western,
}ENUM_LONGITUDE;


//根据经度得到分带号索引
int GetZoneIndexFromLon(ENUM_ZONEDEGRRE zd,double longitude);
//初始化
void Initialize(STR_ELLIPSOIP str_ellipsoip);
//高斯-克吕格（Gauss-Kruger）投影
//经纬度-XY
bool ToGaussKruger(ENUM_ZONEDEGRRE zd, int zid, double B, ENUM_LATITUDE bns, double L, ENUM_LONGITUDE lew, bool isYhasZid, double *X,double *Y);
//UTM投影
//经纬度-XYGetDistAngleFromGpsByUTM
bool ToUTM(ENUM_ZONEDEGRRE zd, int zid, double B, ENUM_LATITUDE bns, double L, ENUM_LONGITUDE lew, bool isYhasZid, double *X,double *Y);
//高斯-克吕格（Gauss-Kruger）投影
//XY-经纬度
bool FromGaussKruger(ENUM_ZONEDEGRRE zd, int zid, double X, double Y, double *B, ENUM_LATITUDE *bns, double *L, ENUM_LONGITUDE *lew);
//UTM投影
//XY-经纬度
bool FromUTM(ENUM_ZONEDEGRRE zd, int zid, double X, double Y, double *B, ENUM_LATITUDE *bns, double *L, ENUM_LONGITUDE *lew);

void GetUtmFromGps(double dLon, double dLat, double *X,double *Y);

void GetGpsPositonFromDistAngleByUTM( double dStartPosLon, double dStartPosLat, float fAngle, float fDistance, double* pdDestPosLon, double* pdDestPosLat );

void GetAngleDistByAbsPos( float fRefX, float fRefY, float fDestX, float fDestY, float* pfAngle, float* pfDist );

#endif // CLSGEO_H
