#include "clsGeo.h"

static double a; // 长半径
static double b; // 短半径
static double f; // 扁率
static double e2_1st;  // 第一偏心率的平方
static double e2_2nd;  // 第二偏心率的平方

static float TurnAngleTo360( float fAngle );

/// <summary>
///  待参数的构造函数
/// </summary>
/// <param name="spheroid">地球椭球体</param>
void Initialize(STR_ELLIPSOIP str_ellipsoip)
{
    a = str_ellipsoip.semimajorAxis;
    b = str_ellipsoip.semiminorAxis;
    f = str_ellipsoip.inverseFlattening;
    e2_1st = str_ellipsoip.eccentricity2_1st;
    e2_2nd = str_ellipsoip.eccentricity2_2nd;
}

//根据经度得到分带号索引
///
/// \brief GetZoneIndexFromLon
/// \param zd
/// \param longitude
/// \return
///
///
int GetZoneIndexFromLon(ENUM_ZONEDEGRRE zd,double longitude)
{
    int zid = 0;//默认为zone1
    int i, j;

    if(zd == sixDegree)//1:2.5～1:50万地形图
    {
        for(i=0;i<30;i++)
        {
            if(longitude>=(0+6*i)&&longitude<(6+6*i))
            {
                zid = 1+i;
                break;
            }
        }
        for(j=0;j<30;j++)
        {
            if(longitude>=(-180+6*j)&&longitude<(-174+6*j))
            {
                zid = 31+j;
                break;
            }
        }
    }
    else if(zd == threeDegree)//"1:1万及以上地形图",只有东经的,也不知道为啥
    {
        for(i=0;i<60;i++)
        {
            if(longitude>=(1.5+3*i)&&longitude<(4.5+3*i))
            {
                zid = 1+i;
                break;
            }
        }
    }
    return zid;
}

/// <summary>
/// 转换到高斯-克吕格（Gauss-Kruger）投影
/// </summary>
/// <remarks>
/// 原点纬度为0，若X坐标小于0，则代表在南半球
/// </remarks>
/// <param name="zd">6度分带或3度分带</param>
/// <param name="zid">分带号</param>
/// <param name="B">纬度，单位角度</param>
/// <param name="bns">北纬南纬</param>
/// <param name="L">经度，单位角度</param>
/// <param name="lew">东经西经</param>
/// <param name="isYhasZid">Y坐标是否包含分带号</param>
/// <param name="X">X坐标</param>
/// <param name="Y">Y坐标</param>
/// <returns></returns>
///
bool ToGaussKruger(ENUM_ZONEDEGRRE zd, int zid, double B, ENUM_LATITUDE bns, double L, ENUM_LONGITUDE lew, bool isYhasZid, double *X,double *Y)
{
    *X = *Y = 0;

    double L0 = 0; // 中央子午线经度
    if (zd == sixDegree)
    {
        if (zid < 1 || zid > 60) return false;
        L0 = 6 * zid - 3;
    }
    else if (zd == threeDegree)
    {
        if (zid < 1 || zid > 120)
            return false;
        L0 = 3 * zid;
    }
    else
    {
        return false;
    }

    double B_angle = B; // 纬度的角度值
    double L_angle = L; // 经度的角度值
    double L0_angle = L0; // 中央经度的角度值

    B = B_angle * PI / 180.0; // 纬度的弧度值
    L = L_angle * PI / 180.0; // 经度的弧度值
    L0 = L0_angle * PI / 180.0; // 中央经度的弧度值

    double k0 = 1.0; // 高斯-克吕格比例因子
    double FE;  // 东经偏移
    if (isYhasZid) FE = 500000 + zid * 1000000; // 加带号
    else FE = 500000; // 不加带号

    double T = tan(B) * tan(B);
    double C = e2_2nd * cos(B) * cos(B);
    double A = (L - L0) * cos(B);
    double M = a * ((1 - e2_1st / 4.0 - 3 * pow(e2_1st, 2) / 64.0 - 5 * pow(e2_1st, 3) / 256.0) * B
                    - (3 * e2_1st / 8.0 + 3 * pow(e2_1st, 2) / 32.0 + 45 * pow(e2_1st, 3) / 1024.0) * sin(2 * B)
                    + (15 * pow(e2_1st, 2) / 256.0 + 45 * pow(e2_1st, 3) / 1024.0) * sin(4 * B)
                    - (35 * pow(e2_1st, 3) / 3072.0) * sin(6 * B));
    double N = a / sqrt(1 - e2_1st * sin(B) * sin(B));

    *X = k0 * (M + N * tan(B) * (A * A / 2.0 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24.0)
               + (61 - 58 * T + T * T + 270 * C - 330 * T * C) * pow(A, 6) / 720.0);
    *Y = FE + k0 * N * (A + (1 - T + C) * pow(A, 3) / 6.0
                        + (5 - 18 * T + T * T + 14 * C - 58 * T * C) * pow(A, 5) / 120);

    if (bns == southern) *X = -*X;
    return true;
}

/// <summary>
///  将高斯-克吕格（Gauss-Kruger）投影转化为经纬网
/// </summary>
/// <remarks>
/// 原点纬度为0，若X坐标小于0，则代表在南半球
/// </remarks>
/// <param name="zd">6度分带或3度分带</param>
/// <param name="zid">分带号</param>
/// <param name="X">X坐标</param>
/// <param name="Y">Y坐标</param>
/// <param name="B">纬度，单位角度</param>
/// <param name="bns">北纬南纬</param>
/// <param name="L">经度，单位角度</param>
/// <param name="lew">东经西经</param>
/// <returns></returns>
bool FromGaussKruger(ENUM_ZONEDEGRRE zd, int zid, double X, double Y, double *B, ENUM_LATITUDE *bns, double *L, ENUM_LONGITUDE *lew)
{
    *B = *L = 0;
    *bns=northen;
    *lew=eastern;

    double L0 = 0; // 中央子午线经度
    if (zd == sixDegree)
    {
        if (zid < 1 || zid > 60) return false;
        L0 = 6 * zid - 3;
    }
    else if (zd == threeDegree)
    {
        if (zid < 1 || zid > 120) return false;
        L0 = 3 * zid;
    }
    else
    {
        return false;
    }
    double L0_angle = L0; // 中央经度的角度值
    L0 = L0_angle * PI / 180.0; // 中央经度的弧度值

    double k0 = 1.0; // 高斯-克吕格比例因子
    double FE = 500000; // 东经偏移不加带号

    double Mf = X / k0;
    double _u = Mf / (a * (1 - e2_1st / 4.0 - 3 * e2_1st * e2_1st / 64.0 - 5 * e2_1st * e2_1st * e2_1st / 256.0)); // φ
    double e1 = (1 - b / a) / (1 + b / a);
    double Bf = _u + (3 * e1 / 2.0 - 27 * e1 * e1 * e1 / 32.0) * sin(2 * _u)
            + (21 * e1 * e1 / 16.0 - 55 * e1 * e1 * e1 * e1 / 32.0) * sin(4 * _u)
            + (151 * e1 * e1 * e1 / 96.0) * sin(6 * _u);
    double Nf = a / sqrt(1 - e2_1st * sin(Bf) * sin(Bf));
    double Rf = a * (1 - e2_1st) / (pow(1 - e2_1st * sin(Bf) * sin(Bf), 3.0 / 2.0));
    double Tf = tan(Bf) * tan(Bf);
    double Cf = e2_2nd * cos(Bf) * cos(Bf);
    double D = (Y - FE) / (k0 * Nf);

    *B = Bf - (Nf * tan(Bf) / Rf) * (D * D / 2.0 - (5 + 3 * Tf + Cf - 9 * Tf * Cf) * D * D * D * D / 24.0
                                     + (61 + 90 * Tf + 45 * Tf * Tf) * pow(D, 6) / 720.0);
    *L = L0 + (1 / cos(Bf)) * (D - (1 + 2 * Tf + Cf) * pow(D, 3) / 6.0
                               + (5 + 28 * Tf + 6 * Cf + 8 * Tf * Cf + 24 * Tf * Tf) * pow(D, 5) / 120.0);

    *B = *B * 180 / PI;
    *L = *L * 180 / PI;
    if (*B < 0)
    {
        *B=-*B;
        *bns=southern;
    }
    return true;
}

/// <summary>
/// 转换到通用横轴墨卡托（UTM）投影
/// </summary>
/// <remarks>
/// 北半球原点纬度为0，南半球原点南移10000000M
/// </remarks>
/// <param name="zd">6度分带或3度分带</param>
/// <param name="zid">分带号</param>
/// <param name="B">纬度，单位角度</param>
/// <param name="bns">北纬南纬</param>
/// <param name="L">经度，单位角度</param>
/// <param name="lew">东经西经</param>
/// <param name="isYhasZid">Y坐标是否包含分带号</param>
/// <param name="X">X坐标</param> 对应纬度
/// <param name="Y">Y坐标</param> 对应经度
/// <returns></returns>
bool ToUTM(ENUM_ZONEDEGRRE zd, int zid, double B, ENUM_LATITUDE bns, double L, ENUM_LONGITUDE lew, bool isYhasZid, double *X,double *Y)
{
    *X = *Y = 0;

    double L0 = 0; // 中央子午线经度
    if (zd == sixDegree)
    {
        if (zid < 1 || zid > 60) return false;
        L0 = 6 * zid - 3;
    }
    else if (zd == threeDegree)
    {
        if (zid < 1 || zid > 120) return false;
        L0 = 3 * zid;
    }
    else
    {
        return false;
    }

    double B_angle = B; // 纬度的角度值
    double L_angle = L; // 经度的角度值
    double L0_angle = L0; // 中央经度的角度值

    B = B_angle *PI / 180.0; // 纬度的弧度值
    L = L_angle *PI / 180.0; // 经度的弧度值
    L0 = L0_angle *PI / 180.0; // 中央经度的弧度值

    double k0 = 0.9996; // UTM比例因子
    double FE;  // 东经偏移
    if(isYhasZid) FE = 500000 + zid * 1000000; // 加带号
    else FE = 500000; // 不加带号

    double T =tan(B) *tan(B);
    double C = e2_2nd *cos(B) *cos(B);
    double A = (L - L0) *cos(B);
    double M = a * ((1 - e2_1st / 4.0 - 3 *pow(e2_1st, 2) / 64.0 - 5 *pow(e2_1st, 3) / 256.0) * B
                    - (3 * e2_1st / 8.0 + 3 *pow(e2_1st, 2) / 32.0 + 45 *pow(e2_1st, 3) / 1024.0) *sin(2 * B)
                    + (15 *pow(e2_1st, 2) / 256.0 + 45 *pow(e2_1st, 3) / 1024.0) *sin(4 * B)
                    - (35 *pow(e2_1st, 3) / 3072.0) *sin(6 * B));
    double N = a /sqrt(1 - e2_1st *sin(B) *sin(B));

    *X = k0 * (M + N *tan(B) * (A * A / 2.0 + (5 - T + 9 * C + 4 * C * C) *pow(A, 4) / 24.0)
               + (61 - 58 * T + T * T + 270 * C - 330 * T * C) *pow(A, 6) / 720.0);
    *Y = FE + k0 * N * (A + (1 - T + C) *pow(A, 3) / 6.0
                        + (5 - 18 * T + T * T + 14 * C - 58 * T * C) *pow(A, 5) / 120);

    if (bns == southern) *X += 10000000; // 南半球北纬偏移
    return true;
}

/// <summary>
///  将通用横轴墨卡托（UTM）投影转化为经纬网
/// </summary>
/// <remarks>
/// 北半球原点纬度为0，南半球原点南移10000000M
/// </remarks>
/// <param name="zd">6度分带或3度分带</param>
/// <param name="zid">分带号</param>
/// <param name="X">X坐标</param> 对应纬度
/// <param name="Y">Y坐标</param> 对应经度
/// <param name="B">纬度，单位角度</param>
/// <param name="bns">北纬南纬</param>
/// <param name="L">经度，单位角度</param>
/// <param name="lew">东经西经</param>
/// <returns></returns>
bool FromUTM(ENUM_ZONEDEGRRE zd, int zid, double X, double Y, double *B, ENUM_LATITUDE *bns, double *L, ENUM_LONGITUDE *lew)
{
    *B = *L = 0;
    *bns = northen;
    *lew = eastern;

    if (X >= 10000000)
    {
        X -= 10000000;  // 南半球北纬偏移
        *bns = southern;
    }

    double L0 = 0; // 中央子午线经度
    if (zd == sixDegree)
    {
        if (zid < 1 || zid > 60) return false;
        L0 = 6 * zid - 3;
    }
    else if (zd == threeDegree)
    {
        if (zid < 1 || zid > 120) return false;
        L0 = 3 * zid;
    }
    else
    {
        return false;
    }


    double L0_angle = L0; // 中央经度的角度值
    L0 = L0_angle * PI / 180.0; // 中央经度的弧度值

    double k0 = 0.9996; // UTM比例因子
    double FE = 500000; // 东经偏移不加带号

    double Mf = X / k0;
    double _u = Mf / (a * (1 - e2_1st / 4.0 - 3 * e2_1st * e2_1st / 64.0 - 5 * e2_1st * e2_1st * e2_1st / 256.0)); // φ
    double e1 = (1 - b / a) / (1 + b / a);
    double Bf = _u + (3 * e1 / 2.0 - 27 * e1 * e1 * e1 / 32.0) * sin(2 * _u)
            + (21 * e1 * e1 / 16.0 - 55 * e1 * e1 * e1 * e1 / 32.0) * sin(4 * _u)
            + (151 * e1 * e1 * e1 / 96.0) * sin(6 * _u);
    double Nf = a / sqrt(1 - e2_1st * sin(Bf) * sin(Bf));
    double Rf = a * (1 - e2_1st) / (pow(1 - e2_1st * sin(Bf) * sin(Bf), 3.0 / 2.0));
    double Tf = tan(Bf) * tan(Bf);
    double Cf = e2_2nd * cos(Bf) * cos(Bf);
    double D = (Y - FE) / (k0 * Nf);

    *B = Bf - (Nf * tan(Bf) / Rf) * (D * D / 2.0 - (5 + 3 * Tf + 10 * Cf - 4 * Cf * Cf - 9 * e2_2nd) * D * D * D * D / 24.0
                                     + (61 + 90 * Tf + 298 * Cf + 45 * Tf * Tf - 252 * e2_2nd - 3 * Cf * Cf) * pow(D, 6) / 720.0);
    *L = L0 + (1 / cos(Bf)) * (D - (1 + 2 * Tf + Cf) * pow(D, 3) / 6.0
                               + (5 + 28 * Tf + 6 * Cf + 8 * Tf * Cf + 24 * Tf * Tf) * pow(D, 5) / 120.0);

    *B = *B * 180 / PI;
    *L = *L * 180 / PI;

    return true;
}


void GetUtmFromGps( double dLon, double dLat, double *X,double *Y)
{
    STR_ELLIPSOIP stEarthInfo;
    stEarthInfo.semimajorAxis = 6378137;
    stEarthInfo.semiminorAxis = 6356752.3142;
    stEarthInfo.inverseFlattening = 0.0033528106643316;
    stEarthInfo.eccentricity2_1st = 0.00669437999013;
    stEarthInfo.eccentricity2_2nd = 0.006739496742227;

    /// 初始化公用参数
    Initialize( stEarthInfo );
    ENUM_ZONEDEGRRE ZD = sixDegree;

    /// 计算起点墨卡托坐标
    int izidStart = GetZoneIndexFromLon( ZD, dLon );
//    printf("izidStart = %d\n", izidStart );
    ENUM_LATITUDE BNS_Start;
    bool bIsYhasZid = 0;
    if( dLat >= 0 )
    {
        BNS_Start = northen;
    }
    else
    {
        BNS_Start = southern;
    }


    ENUM_LONGITUDE LEW_Start;
    if( dLon >= 0 )
    {
        LEW_Start = eastern;
    }
    else
    {
        LEW_Start = western;
    }

    double dXStart, dYStart;
//    bool iValid = ToUTM( ZD, izidStart, dStartPosLat, BNS_Start, dStartPosLon, LEW_Start, bIsYhasZid, &dXStart, &dYStart );
    ToUTM( ZD, izidStart, dLat, BNS_Start, dLon, LEW_Start, bIsYhasZid, X, Y );

}


void GetGpsPositonFromDistAngleByUTM( double dStartPosLon, double dStartPosLat, float fAngle, float fDistance, double* pdDestPosLon, double* pdDestPosLat )
{
    STR_ELLIPSOIP stEarthInfo;
    stEarthInfo.semimajorAxis = 6378137;
    stEarthInfo.semiminorAxis = 6356752.3142;
    stEarthInfo.inverseFlattening = 0.0033528106643316;
    stEarthInfo.eccentricity2_1st = 0.00669437999013;
    stEarthInfo.eccentricity2_2nd = 0.006739496742227;

    /// 初始化公用参数
    Initialize( stEarthInfo );
    ENUM_ZONEDEGRRE ZD = sixDegree;

    /// 计算起点墨卡托坐标
    int izidStart = GetZoneIndexFromLon( ZD, dStartPosLon );
//    printf("izidStart = %d\n", izidStart );
    ENUM_LATITUDE BNS_Start;
    bool bIsYhasZid = 0;
    if( dStartPosLat >= 0 )
    {
        BNS_Start = northen;
    }
    else
    {
        BNS_Start = southern;
    }


    ENUM_LONGITUDE LEW_Start;
    if( dStartPosLon >= 0 )
    {
        LEW_Start = eastern;
    }
    else
    {
        LEW_Start = western;
    }

    double dXStart, dYStart;
//    bool iValid = ToUTM( ZD, izidStart, dStartPosLat, BNS_Start, dStartPosLon, LEW_Start, bIsYhasZid, &dXStart, &dYStart );
    ToUTM( ZD, izidStart, dStartPosLat, BNS_Start, dStartPosLon, LEW_Start, bIsYhasZid, &dXStart, &dYStart );
//    printf("iValid = %d\n", iValid );
//    printf("dXStart = %f\n", dXStart );
//    printf("dYStart = %f\n", dYStart );

    double dConvertX, dConvertY;
    dConvertX = dXStart + fDistance * cos( TORAD( fAngle ) );
    dConvertY = dYStart + fDistance * sin( TORAD( fAngle ) );

//    printf(" dConvertX = %f\n", fDistance * cos( TORAD( fAngle ) ) );
//    printf(" dConvertY = %f\n", fDistance * sin( TORAD( fAngle ) ) );
//
//    printf(" dConvertX = %f\n", dConvertX );
//    printf(" dConvertY = %f\n", dConvertY );

    double dlat, dlon;

//    iValid = FromUTM( ZD, izidStart, dConvertX, dConvertY, &dlat, &BNS_Start, &dlon, &LEW_Start);
    FromUTM( ZD, izidStart, dConvertX, dConvertY, &dlat, &BNS_Start, &dlon, &LEW_Start);
//    printf(" kkk = %d\n ", kkk );

    // bool FromUTM(ENUM_ZONEDEGRRE zd, int zid, double X, double Y, double *B, ENUM_LATITUDE *bns, double *L, ENUM_LONGITUDE *lew)

//    printf(" dLon = %.8f\n", dlon );
//    printf(" dLat = %.8f\n", dlat );

    *pdDestPosLon = dlon;
    *pdDestPosLat = dlat;
}


/******************************************************
函数功能：规整输入的遍历角度
输入：任意角度
输出：0-360度以内的角度
******************************************************/
static float TurnAngleTo360( float fAngle )
{
    while( fAngle < 0 || fAngle > 360 )
    {
        if( fAngle < 0 )
        {
            fAngle = fAngle + 360;
        }
        if( fAngle > 360 )
        {
            fAngle = fAngle - 360;
        }
    }
    //printf("%.2f\n",fAngle);
    return fAngle;
}


void GetAngleDistByAbsPos( float fRefX, float fRefY, float fDestX, float fDestY, float* pfAngle, float* pfDist )
{
    double  dDeltaX = fDestX - fRefX;
    double  dDeltaY = fDestY - fRefY;

    *pfDist = sqrt( pow(dDeltaY,2)+pow(dDeltaX,2) );

    if( dDeltaY==0 && dDeltaX==0 )
    {
        *pfAngle = 0;
        return;
    }

    if( dDeltaY==0 && dDeltaX>0 )
    {
        *pfAngle = 90;
        return;
    }

    if( dDeltaY==0 && dDeltaX<0 )
    {
        *pfAngle = 270;
        return;
    }

    if( dDeltaY>0 && dDeltaX==0 )
    {
        *pfAngle = 0;
        return;
    }

    if( dDeltaY<0 && dDeltaX==0 )
    {
        *pfAngle = 180;
        return;
    }

    *pfAngle = TOANG( atan( dDeltaX /dDeltaY  ) );
//    printf("********************fAngle = %f\n", *pfAngle );
//    printf(" dDeltaY = %f\n ", dDeltaY );
//    printf(" dDeltaX = %f\n ", dDeltaX );
    if( dDeltaY>0 && dDeltaX>0 )
    {
        *pfAngle = *pfAngle;
    }
    if( dDeltaY>0 && dDeltaX<0 )
    {
        *pfAngle = *pfAngle + 360;
    }
    if( dDeltaY<0 && dDeltaX<0 )
    {
        *pfAngle = *pfAngle + 180;
    }
    if( dDeltaY<0 && dDeltaX>0 )
    {
        *pfAngle = 180 + *pfAngle;
    }

    *pfAngle = TurnAngleTo360( *pfAngle );

}

