#pragma once
#define PCL_NO_PRECOMPILE
// #include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>

enum Box2DRelativeZone{
    INSIDE,
    ABOVE_LEFT,
    ABOVE,
    ABOVE_RIGHT,
    LEFT,
    RIGHT,
    BELOW_LEFT,
    BELOW,
    BELOW_RIGHT,
};

struct cameraParams{
	double Fx;
	double Fy;
	double Cx;
	double Cy;
    std::string frameId;
	float depthMaxDist;
};

struct point3D{
    float x;
    float y;
    float z;
};

struct point2D{
    float x;
    float y;
};

struct bbox{
    point3D centroid;
    
    union{
       struct{
        float xMin;
        float yMin;
        float zMin;
       }; 
       point3D min;
    };

    union{
       struct{
        float xMax;
        float yMax;
        float zMax;
       }; 
       point3D max;
    };
};

struct SemBbox{
    point3D centroid;
    int id;
    union{
       struct{
        float xMin;
        float yMin;
        float zMin;
       }; 
       point3D min;
    };

    union{
       struct{
        float xMax;
        float yMax;
        float zMax;
       }; 
       point3D max;
    };
};

struct box2D{
    point2D tl;
    point2D br;
};

struct box3D{
    point3D min;
    point3D max;
};

struct localizationObject{
    uint8_t semId;
    float confidence;
    point3D closestPoint;
    point3D centroid;
    float distanceFromBaseLink;
    bbox box;
};

float box3DIoU(const bbox& referenceBox, const std::vector<bbox>& boxList);

// float box2DIoU(const bbox& referenceBox, const std::vector<bbox>& boxList);

void getBboxVertices(const bbox &bbox, std::vector<point3D> &vertices);

struct sphere{

    // center
    float x;
    float y;
    float z;

    double radius;
    // id of the object in database  they belong 
    int id;
};

struct disk{
    
    // center
    float x;
    float y;
    float z;

    double innerRadius;
    double outerRadius;

    // id of the object in database  they belong 
    int id;
    
};

void getCameraParams(const std::array<double,9UL>& K, cameraParams &params);
void setCameraParams(std::array<double,9UL>& K, const cameraParams &params);

struct EIGEN_ALIGN16 PointSemXYZRGB{  // enforce SSE padding for correct memory alignment
    
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    float conf;
    uint8_t id;

    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};
POINT_CLOUD_REGISTER_POINT_STRUCT (PointSemXYZRGB,           
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, conf, conf)
                                   (uint8_t, id, id)
                                   
)

struct EIGEN_ALIGN16 PointSemXYZ{  // enforce SSE padding for correct memory alignment
    
    PCL_ADD_POINT4D;
    float conf;
    uint8_t id;

    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};
POINT_CLOUD_REGISTER_POINT_STRUCT (PointSemXYZ,           
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, conf, conf)
                                   (uint8_t, id, id)
                                   
)

void rainbow_rgb(const double &ratio, uint8_t &r, uint8_t &g, uint8_t &b);

bool getClasses(const std::string& labelPath, std::vector<std::string>& classNames, int& classNumber);

int getSign(float num);

/**
 * @brief 
 * 
 * @param rgbRow rgb row [input]
 * @param rgbCol rgb col [input]
 * @param deptRow depth row [output]
 * @param depthCol depth col [output]
 * @param depthCameraInfo 
 * @param rgbCameraInfo
 */
void RGB2Depth(const int &rgbRow , const  int &rgbCol, int &depthRow, int &depthCol,cameraParams &depthCameraInfo, cameraParams &rgbCameraInfo);

/**
 * @brief 
 * @param deptRow depth row [input]
 * @param depthCol depth col [input]
 * @param rgbRow rgb row [output]
 * @param rgbCol rgb col [output]
 * @param depthCameraInfo 
 * @param rgbCameraInfo
 */
void Depth2RGB(const int &depthRow, const  int &depthCol, int &rgbRow, int &rgbCol, cameraParams &depthCameraInfo, cameraParams &rgbCameraInfo);


void fillSeenObjectMatrices(const std::vector<localizationObject> seenObjects,
                            std::unordered_map<uint8_t, float> maxSizeMap,
                            std::vector<std::vector<int>> &seenObjDxSxMatrix,
                            std::vector<std::vector<float>> &seenObjMaxDistanceMatrix,
                            std::vector<std::vector<float>> &seenObjMinDistanceMatrix);

void addNewObjectSeen(uint8_t semId,float &conf, point3D &closestPoint, point3D &centroid, bbox &objBox, localizationObject &seenObject, Eigen::Matrix4d &tBaselinkCamera);
void addNewObjectSeen(uint8_t semId,float &conf, point3D &closestPoint, point3D &centroid, localizationObject &seenObject, Eigen::Matrix4d &tBaselinkCamera);

bool findClosestPoint(const point2D &point, const box2D &box, point2D &closestPoint);

Box2DRelativeZone getPointZone(const box2D& box, const point2D& point);

double wrapToPi(const double &angleInRadians);

double wrapTo2Pi(const double &angleInRadians);

double circularMean(double angle1, double angle2);

double toRadians(const double degrees);

//return y value of a 0-centered normal distribution
double normPdf(const double &x, const double &cov);

void floatToUint8Mat(const cv::Mat &input, cv::Mat &output);

bool hasHugeOrNaNValues(const cv::Mat& data);

bool isPointInsideFOV(const point3D &point, const point3D &sourcePoint, const float &cameraPoseAngle, float &fov);
bool isVoxelInsideFOV(const point3D &voxCenter, const float &voxSize, const point3D &sourcePoint, const float &cameraPoseAngle, float &fov);

double logOddToProb(const double &logOdd);
double probToLogOdd(const double &prob);