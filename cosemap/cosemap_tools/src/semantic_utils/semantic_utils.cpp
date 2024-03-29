#include "cosemap_tools/semantic_utils/semantic_utils.h"

void rainbow_rgb(const double &ratio, uint8_t &r, uint8_t &g, uint8_t &b)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 6);

    //find the region for this position
    int region = normalized / 256;

    //find the distance to the start of the closest region
    int x = normalized % 256;

    // uint8_t r = 0, g = 0, b = 0;
    switch (region)
    {
    case 0: r = 255; g = 0;   b = 0;   g += x; break;
    case 1: r = 255; g = 255; b = 0;   r -= x; break;
    case 2: r = 0;   g = 255; b = 0;   b += x; break;
    case 3: r = 0;   g = 255; b = 255; g -= x; break;
    case 4: r = 0;   g = 0;   b = 255; r += x; break;
    case 5: r = 255; g = 0;   b = 255; b -= x; break;
    }
}

bool getClasses(const std::string& labelPath, std::vector<std::string>& classNames, int& classNumber){

	std::ifstream in;
	in.open(labelPath.c_str());
    if(!in){
		return false;
	}

	std::string line;
    classNumber = 0;
	while (std::getline(in,line)){
        classNames.push_back(line);
		classNumber++;

	}
	return true;
}

int getSign(float num){
    if(num < 0){
        return -1;
    }else if(num > 0){
        return +1;
    }else{
        return 0;
    }
}

void getCameraParams(const std::array<double,9UL>& K, cameraParams &params){
    
    //reading camera params from msg
    params.Fx = K.at(0);
    params.Fy = K.at(4);
    params.Cx = K.at(2);
    params.Cy = K.at(5);

}

void setCameraParams(std::array<double,9UL>& K, const cameraParams &params){
    
    K.at(0) = params.Fx;
    K.at(4) = params.Fy;
    K.at(2) = params.Cx;
    K.at(5) = params.Cy;
}

float getBboxMaxSize(const bbox &bbox){
    float maxSize = 0;
    

}

void getBboxVertices(const bbox &bbox, std::vector<point3D> &vertices){
    // Iterazione su tutti gli otto vertici
    point3D vertex;
    int l = 0;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                vertex.x = (i == 0) ? bbox.min.x : bbox.max.x;
                vertex.y = (j == 0) ? bbox.min.y : bbox.max.y;
                vertex.z = (k == 0) ? bbox.min.z : bbox.max.z;

                vertices.at(l) = vertex;
                l++;
            }
        }
    }
}

void RGB2Depth(const int &rgbRow , const  int &rgbCol, int &depthRow, int &depthCol, cameraParams &depthCameraInfo, cameraParams &rgbCameraInfo){
                
    depthCol = depthCameraInfo.Cx + ( (depthCameraInfo.Fx / rgbCameraInfo.Fx) * (rgbCol - rgbCameraInfo.Cx)) ;

    depthRow = depthCameraInfo.Cy + ( (depthCameraInfo.Fy / rgbCameraInfo.Fy) * (rgbRow - rgbCameraInfo.Cy)) ;
}

void Depth2RGB(const int &depthRow, const  int &depthCol, int &rgbRow, int &rgbCol, cameraParams &depthCameraInfo, cameraParams &rgbCameraInfo){

    rgbCol = rgbCameraInfo.Cx + (rgbCameraInfo.Fx * ( (depthCol - depthCameraInfo.Cx) / (depthCameraInfo.Fx) ) );

    rgbRow = rgbCameraInfo.Cy + (rgbCameraInfo.Fy * ( (depthRow - depthCameraInfo.Cy) / (depthCameraInfo.Fy) ) );
}

void fillSeenObjectMatrices(const std::vector<localizationObject> seenObjects,
                            std::unordered_map<uint8_t, float> maxSizeMap,
                            std::vector<std::vector<int>> &seenObjDxSxMatrix,
                            std::vector<std::vector<float>> &seenObjMaxDistanceMatrix,
                            std::vector<std::vector<float>> &seenObjMinDistanceMatrix){
    /* seen objects max distance matrix*/
    float distance = 0;

    // iteration among seen objects to find possible candidate pairs in database and to create correct lambda function in order to reduce the localization search zone
    for(size_t i = 0; i < seenObjects.size(); i++){
        for(size_t j = 0; j < seenObjects.size(); j++ ){
            if(i < j){

                distance = sqrt(pow(seenObjects.at(i).closestPoint.x-seenObjects.at(j).closestPoint.x, 2) + pow(seenObjects.at(i).closestPoint.y-seenObjects.at(j).closestPoint.y, 2));

                seenObjMaxDistanceMatrix[i][j] = (distance + maxSizeMap[seenObjects[i].semId] + maxSizeMap[seenObjects[j].semId]);
                seenObjMinDistanceMatrix[i][j] = distance/2;
                
                //comparing positions in base link to find which is left/right to the other
                if (seenObjects.at(i).centroid.y < seenObjects.at(j).centroid.y){
                    // negative: i=dx,j=sx
                    seenObjDxSxMatrix[i][j] = -1;
                }else if(seenObjects.at(i).centroid.y > seenObjects.at(j).centroid.y){
                    //  positive: i=sx,j=dx 
                    seenObjDxSxMatrix[i][j] = +1;
                }else{
                    //  0: same position 
                    seenObjDxSxMatrix[i][j] = 0;
                }
            }
        }
    }
}



void addNewObjectSeen(uint8_t semId,float &conf, point3D &closestPoint, point3D &centroid, bbox &objBox, localizationObject &seenObject, Eigen::Matrix4d &tBaselinkCamera){
    
    // transforming object parameters from camera frame to base link frame
    Eigen::Vector4d centroidCamera( centroid.x, centroid.y, centroid.z, 1);
    Eigen::Vector4d closestCamera ( closestPoint.x, closestPoint.y, closestPoint.z, 1);
    Eigen::Vector4d cameraFrameMin( objBox.min.x, objBox.min.y, objBox.min.z, 1);
    Eigen::Vector4d cameraFrameMax( objBox.max.x, objBox.max.y, objBox.max.z, 1);

    Eigen::Vector4d baselinkCentroid = tBaselinkCamera * centroidCamera;
    Eigen::Vector4d baseLinkClosest =  tBaselinkCamera * closestCamera;

    Eigen::Vector4d baselinkMin = tBaselinkCamera * cameraFrameMin;
    Eigen::Vector4d baselinkMax = tBaselinkCamera * cameraFrameMax;

    seenObject.semId = semId;
    seenObject.confidence = conf ;

    seenObject.centroid.x = baselinkCentroid.x();
    seenObject.centroid.y = baselinkCentroid.y();
    seenObject.centroid.z = baselinkCentroid.z();

    seenObject.closestPoint.x = baseLinkClosest.x();
    seenObject.closestPoint.y = baseLinkClosest.y();
    seenObject.closestPoint.z = baseLinkClosest.z();

    //taking new maxima and minima
    seenObject.box.xMin = std::min(baselinkMin.x(), baselinkMax.x());
    seenObject.box.yMin = std::min(baselinkMin.y(), baselinkMax.y());
    seenObject.box.zMin = std::min(baselinkMin.z(), baselinkMax.z());
    
    
    seenObject.box.xMax = std::max(baselinkMin.x(), baselinkMax.x());
    seenObject.box.yMax = std::max(baselinkMin.y(), baselinkMax.y());
    seenObject.box.zMax = std::max(baselinkMin.z(), baselinkMax.z());
    
    //calculating distance from base link frame to objects
    seenObject.distanceFromBaseLink = sqrt( pow(seenObject.closestPoint.x, 2) +
                                            pow(seenObject.closestPoint.y, 2));// +   just 2-D distance
                                            // pow(seenObject.z,2));
}

void addNewObjectSeen(uint8_t semId,float &conf, point3D &closestPoint, point3D &centroid, localizationObject &seenObject, Eigen::Matrix4d &tBaselinkCamera){
    
    // std::cout<<std::endl;
    // std::cout<<"Semantic_utils addNewObjectSeen"<<std::endl;

    // transforming object parameters from camera frame to base link frame
    Eigen::Vector4d centroidCamera( centroid.x, centroid.y, centroid.z, 1);
    Eigen::Vector4d closestCamera ( closestPoint.x, closestPoint.y, closestPoint.z, 1);

    // std::cout<<"closest[camera]: "<<closestCamera.x()<<",  "<<closestCamera.y()<<",  "<<closestCamera.z()<<std::endl;

    Eigen::Vector4d baselinkCentroid = tBaselinkCamera * centroidCamera;
    Eigen::Vector4d baseLinkClosest =  tBaselinkCamera * closestCamera;
    
    seenObject.semId = semId;
    seenObject.confidence = conf ;

    seenObject.centroid.x = baselinkCentroid.x();
    seenObject.centroid.y = baselinkCentroid.y();
    seenObject.centroid.z = baselinkCentroid.z();

    seenObject.closestPoint.x = baseLinkClosest.x();
    seenObject.closestPoint.y = baseLinkClosest.y();
    seenObject.closestPoint.z = baseLinkClosest.z();
    
    //calculating distance from base link frame to objects
    seenObject.distanceFromBaseLink = sqrt( pow(seenObject.closestPoint.x, 2) +
                                            pow(seenObject.closestPoint.y, 2));// +   just 2-D distance
                                            // pow(seenObject.z,2));


    // std::cout<<"seenObject.semId: "<<(int)seenObject.semId<<std::endl;
    // std::cout<<"closest[base link]: "<<seenObject.closestPoint.x<<",  "<<seenObject.closestPoint.y<<",  "<<seenObject.closestPoint.z<<std::endl;
}

bool findClosestPoint(const point2D &point, const box2D &box, point2D &closestPoint){

    bool isOutside;
    Box2DRelativeZone zone = getPointZone(box, point);
    switch (zone){
    case INSIDE:{
        isOutside = false;
        break;
    }
    case ABOVE:{
        closestPoint.x = point.x;
        closestPoint.y = box.tl.y;
        isOutside = true;
        break;
    }
    case BELOW:{
        closestPoint.x = point.x;
        closestPoint.y = box.br.y;
        isOutside = true;
        break;
    }
    case LEFT:{
        closestPoint.x = box.tl.x;
        closestPoint.y = point.y;
        isOutside = true;
        break;
    }
    case RIGHT:{
        closestPoint.x = box.br.x;
        closestPoint.y = point.y;
        isOutside = true;
        break;
    }
    case ABOVE_LEFT:{
        closestPoint.x = box.tl.x;
        closestPoint.y = box.tl.y;
        isOutside = true;
        break;
    }
    case ABOVE_RIGHT:{
        closestPoint.x = box.br.x;
        closestPoint.y = box.tl.y;
        isOutside = true;
        break;
    }
    case BELOW_LEFT:{
        closestPoint.x = box.tl.x;
        closestPoint.y = box.br.y;
        isOutside = true;
        break;
    }
    case BELOW_RIGHT:{
        closestPoint.x = box.br.x;
        closestPoint.y = box.br.y;
        isOutside = true;
        break;
    }
    default:
        break;
    }

    return isOutside;

}

Box2DRelativeZone getPointZone(const box2D& box, const point2D& point) {
    if ( (point.x >= box.tl.x) && (point.x <= box.br.x) &&
         (point.y >= box.br.y) && (point.y <= box.tl.y)) {
        return INSIDE;
    }

    if (point.y > box.tl.y && point.x >= box.tl.x && point.x <= box.br.x) {
        return ABOVE;
    } else if (point.y < box.br.y && point.x >= box.tl.x && point.x <= box.br.x) {
        return BELOW;
    } else if (point.x < box.tl.x && point.y >= box.br.y && point.y <= box.tl.y) {
        return LEFT;
    } else if (point.x > box.br.x && point.y >= box.br.y && point.y <= box.tl.y) {
        return RIGHT;
    } else if (point.y > box.tl.y && point.x > box.br.x) {
        return ABOVE_RIGHT;
    } else if (point.y > box.tl.y && point.x < box.tl.x) {
        return ABOVE_LEFT;
    } else if (point.y < box.br.y && point.x > box.br.x) {
        return BELOW_RIGHT;
    } else if (point.y < box.br.y && point.x < box.tl.x) {
        return BELOW_LEFT;
    }
}

double wrapToPi( const double &angleInRadians) {
    // Normalize the angle to the range (-2*pi; +2*pi]
    double normalizedAngle = fmod(angleInRadians, 2.0 * M_PI);

    // Adjust the angle to be in the range (-pi; +pi]
    if (normalizedAngle <= -M_PI) {
        return normalizedAngle + 2.0 * M_PI;
    } else if (normalizedAngle > M_PI) {
        return normalizedAngle - 2.0 * M_PI;
    } else {
        return normalizedAngle;
    }
}

double wrapTo2Pi(const double &angleInRadians) {
    // Normalize the angle to the range [0; 2*pi)
    double normalizedAngle = fmod(angleInRadians, 2.0 * M_PI);

    // Adjust the angle to be in the range [0; 2*pi)
    if (normalizedAngle < 0.0) {
        return normalizedAngle + 2.0 * M_PI;
    } else {
        return normalizedAngle;
    }
}

double circularMean(double angle1, double angle2){
    double x1 = cos(angle1);
    double y1 = sin(angle1);
    double x2 = cos(angle2);
    double y2 = sin(angle2);
    
    double mean_x = (x1 + x2) / 2.0;
    double mean_y = (y1 + y2) / 2.0;
    
    return atan2(mean_y, mean_x);
}

double toRadians(const double degrees){
    return degrees * (M_PI / 180);
}

double normPdf(const double &x, const double &cov){
    double a = 1/(sqrt(2* M_PI * pow(cov,2)));

    double b = -(pow(x,2)/(2 * pow(cov,2)));

    return a*exp(b);
}

double logOddToProb(const double &logOdd){
    return ((exp(logOdd))/(1+exp(logOdd)));
}

double probToLogOdd(const double &prob){
    return (log(prob/(1-prob)));
}

// void floatToUint8Mat(const cv::Mat &input, cv::Mat &output){
//     if( (input.rows != output.rows) || (input.cols != output.cols)){
//         std::cout<<"Mat must have same size"<<std::endl;
//     }

//     if( (input.type() != CV_16F) || (input.type() != CV_32F) || (input.type() != CV_64F)){
//         std::cout<<"input must be float"<<std::endl;
//     }

//     if( (input.type() != CV_8U)){
//         std::cout<<"input must be float"<<std::endl;
//     }

//     double minVal, maxVal;
//     cv::Point minLoc, maxLoc;
//     cv::minMaxLoc(input, &minVal, &maxVal, &minLoc, &maxLoc);
//     for(size_t j = 0; j < input.rows; j++){
//         for(size_t k = 0; k < input.cols; k++){
//             if (isnan(input.at<float>(j,k))){
//                 output.at<uint8_t>(j,k) = 0;
//             }else{
//                 output.at<uint8_t>(j,k) = ( 255 * input.at<float>(j,k) ) /maxVal; 
//             }
//         }
//     }
// }


bool hasHugeOrNaNValues(const cv::Mat& data) {
    for (int row = 0; row < data.rows; ++row) {
        for (int col = 0; col < data.cols; ++col) {
            float value = data.at<float>(row, col); // Assumi che i dati siano di tipo float, adatta se necessario
            if (std::isnan(value) || std::isinf(value)) {
                return true; // Trovato un valore NaN o infinito
            }
        }
    }
    return false; // Nessun valore NaN o infinito trovato
}

bool isPointInsideFOV(const point3D &point, const point3D &sourcePoint,const float &cameraPoseAngle, float &fov) {
    float deltaX = point.x - sourcePoint.x;
    float deltaY = point.y - sourcePoint.y;
    double alpha = std::atan2 (deltaY, deltaX) - cameraPoseAngle;

    alpha = wrapToPi(alpha);
    
    if (  ((-fov / 2) <= alpha) && (alpha <= (fov/2))  ){
        return true;
    }else{
        return false;
    }
}

bool isVoxelInsideFOV(const point3D &voxCenter, const float &voxSize, const point3D &sourcePoint, const float &cameraPoseAngle, float &fov) {
    point3D voxTL, voxTR, voxBL, voxBR;
    float voxHalfSize = voxSize/2;
    voxTL.x = voxCenter.x - voxHalfSize;
    voxTL.y = voxCenter.y + voxHalfSize;

    voxTR.x = voxCenter.x + voxHalfSize;
    voxTR.y = voxCenter.y + voxHalfSize;

    voxBL.x = voxCenter.x - voxHalfSize;
    voxBL.y = voxCenter.y - voxHalfSize;
    
    voxBR.x = voxCenter.x + voxHalfSize;
    voxBR.y = voxCenter.y - voxHalfSize;

    if(isPointInsideFOV(voxTL, sourcePoint, cameraPoseAngle, fov)){
        return true;
    }
    if(isPointInsideFOV(voxTR, sourcePoint, cameraPoseAngle, fov)){
        return true;
    }
    if(isPointInsideFOV(voxBL, sourcePoint, cameraPoseAngle, fov)){
        return true;
    }
    if(isPointInsideFOV(voxBR, sourcePoint, cameraPoseAngle, fov)){
        return true;
    }

    return false;

}


float box3DIoU(const bbox& referenceBox, const std::vector<bbox>& boxList) {
    // Calcola i volumi delle box di riferimento e delle box della lista
    float referenceVolume =
        (referenceBox.max.x - referenceBox.min.x) *
        (referenceBox.max.y - referenceBox.min.y) *
        (referenceBox.max.z - referenceBox.min.z);

    float intersectionVolume = 0.0f;

    for (const bbox& box : boxList) {
        // Calcola l'intersezione tra la box di riferimento e la box corrente nella lista
        float xOverlap = std::max(0.0f, std::min(referenceBox.max.x, box.max.x) - std::max(referenceBox.min.x, box.min.x));
        float yOverlap = std::max(0.0f, std::min(referenceBox.max.y, box.max.y) - std::max(referenceBox.min.y, box.min.y));
        float zOverlap = std::max(0.0f, std::min(referenceBox.max.z, box.max.z) - std::max(referenceBox.min.z, box.min.z));

        float currentIntersectionVolume = xOverlap * yOverlap * zOverlap;

        // Aggiorna il volume totale dell'intersezione
        intersectionVolume += currentIntersectionVolume;
    }

    // Intersection over Union (IoU) for the reference box
    float iou = intersectionVolume / referenceVolume;

    return iou;
}

// // Funzione per calcolare l'IoU tra due box2D
// float calculateIoU(const cv::Rect& box1, const cv::Rect& box2) {
//     cv::Rect intersection = box1 & box2;
//     cv::Rect unionRect = box1 | box2;

//     float iou = static_cast<float>(intersection.area()) / unionRect.area();
//     return iou;
// }

// float box2DIoU(const box3D& b0ox, const std::vector<box3D>& boxList) {
//     // Proietta la box3D nel piano x, y
//     cv::Rect projectedBox(cv::Point2f(box.min.x, box.min.y), cv::Point2f(box.max.x, box.max.y));

//     // Inizializza una copia della box proiettata
//     cv::Rect resultBox = projectedBox.clone();

//     // Sottrai via via le parti in comune con le altre box
//     for (const auto& otherBox : boxList) {
//         cv::Rect otherProjectedBox(cv::Point2f(otherBox.min.x, otherBox.min.y), cv::Point2f(otherBox.max.x, otherBox.max.y));

//         // Calcola l'intersezione tra le due box
//         cv::Rect intersection = resultBox & otherProjectedBox;

//         // Sottrai l'intersezione dalla box risultante
//         resultBox -= intersection;
//     }

//     // Calcola l'IoU come area_originale / area_senza_intersezioni
//     float iou = calculateIoU(projectedBox, resultBox);
//     return iou;

// }