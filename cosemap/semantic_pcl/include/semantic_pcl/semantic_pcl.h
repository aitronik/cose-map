#pragma once

#include <vector>
#include <fstream>
#include <unordered_map>
#include <chrono>

#define PCL_NO_PRECOMPILE
// #include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ml/kmeans.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/time.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/functional/hash.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include "pcl_ros/transforms.hpp"

#include "cosemap_tools/inference_utils/inference_utils.h"
#include "cosemap_tools/semantic_utils/semantic_utils.h"

#include "cosemap_msgs/msg/semantic_pcl.hpp"

using namespace std::chrono;

// forward declaration
bool isSemanticEqual (const PointSemXYZRGB& point_a, const PointSemXYZRGB& point_b, float /*squared_distance*/);

class SemanticPcl{

public:
    /**
     * @brief Construct a new Semantic Pcl object
     * 
     */
    SemanticPcl();

    /**
     * @brief Destroy the Semantic Pcl object
     * 
     */
    ~SemanticPcl();
    /**
     * @brief  Loading dnn model and counting classes
     * 
     * @param labelPath 
     * @param modelPath 
     * @param depthMaxDistance
     * @param realColor
     * @param confidenceThreshold
     * @return true 
     * @return false 
     */
    bool init(  std::string labelPath, 
                std::string modelPath, 
                float depthMaxDistance, 
                bool realColor, 
                float confidenceThreshold, 
                cameraParams &rgbCameraInfo, 
                cameraParams &depthCameraInfo, 
                float clusterTolerance,
                float clusterMinSizeRatio,
                bool clusterObjectsPcl,
                bool outliersRemoval,
                int outliersRemovalMeanK,
                float outliersRemovalStdDevThresh,
                bool includeFilteredPoints,
                int depthPixelSkip);

    /**
     * @brief Creating semantic segmentation masks for a source rgb image
     * 
     * @param[in] rgbImg source rgb image
     * @param[in] idMask output segmentation id mask of source RGB image
     * @param[in] confMask output segmentation confidence mask of source RGB image
     */
    void semanticMasks(const cv::Mat& rgbImg, cv::Mat& idMask, cv::Mat& confMask, cv::Mat& resultIndexMask, std::vector<OutputSeg>& result);
    
    /**
     * @brief creating RGB-semantic pcl from RGB and depth images
     * 
     * @param cvPtrRGB RGB image
     * @param cvPtrDepth depth image
     * @param msgPointCloud empty pcl message to be filled
     * @param infImg output image with inference visualization
     */
    void imageCallback( const cv_bridge::CvImageConstPtr cvPtrRGB,
                              cv_bridge::CvImageConstPtr cvPtrDepth,
                        cosemap_msgs::msg::SemanticPcl& msgPointCloud,
                        sensor_msgs::msg::PointCloud2& msgPointCloudViz,
                        cv::Mat& infImg);


private:
    
    /* number of classes labels*/
    int m_classesNumber;
    /* dnn .onnx net*/
    cv::dnn::Net m_onnnxNet;
    /*use cuda during inference*/
    bool m_useCuda;
    /* camera params */
    cameraParams m_depthCameraInfo;
    cameraParams m_rgbCameraInfo;
    /* net parameters */
    netParams m_netParams;
    /* camera Params check*/
    bool m_isCameraSet = false;    
    /* true for real color in pointcloud rgb, false for semantic colors*/
    bool m_realColor;
    /* vector of class labels */
    std::vector<std::string> m_classNames;
    // number of clusters for background filtering
    double m_seconds;

    float m_clusterTolerance;
    float m_clusterMinSizeRatio;
    bool m_clusterObjectsPcl;
    bool m_outliersRemoval;
    int m_outliersRemovalMeanK;
    float m_outliersRemovalStdDevThresh;
    bool m_includeBackgroundPoints;
    int m_depthPixelSkip;

    int m_pclSizeCounter;
    double m_pclSizeMean;

    int m_meanTimeCounter;
    int64_t m_meanCallbackTime;
    
    /**
     * @brief load onnx model from file to m_onnxNet
     * 
     * @return true : model loaded correctly
     * @return false : model not loaded
     */
    bool readONNXModel(std::string modelPath);

    /**
     * @brief copy points from point to iterator
     * 
     * @param fromPoint source point
     * @param toIterator pcl iterator destination
     */
    void copyPoint(PointSemXYZRGB &fromPoint, pcl::PointCloud<PointSemXYZRGB>::iterator toIterator);

    /**
     * @brief fills sematic and colored pcl from semantics and depth images
     * 
     * @param pointCloud pcl to fill
     * @param depthImg depth image
     * @param rgbImg rgb image
     * @param semanticIDMask semantic id mask
     * @param confidenceMask semantic confidence mask
     */
    void fillPcl(   pcl::PointCloud<PointSemXYZRGB>::Ptr backgroundPcl,
                    std::vector<pcl::PointCloud<PointSemXYZRGB>::Ptr> objPclPtrs,
                    std::vector<pcl::CentroidPoint<PointSemXYZRGB>> &objPclCentroids,
                    const cv::Mat &depthImg, 
                    const cv::Mat &rgbImg, 
                    const cv::Mat &semanticIDMask,
                    const cv::Mat &confidenceMask,
                    const cv::Mat &resultIndexMask,
                    const std::vector<OutputSeg> &result);

    /**
     * @brief perform back ground filtering based on depth statistics of segmented box 
     * 
     * @param rgbImg RGB source img
     * @param semanticIdMask semantic segmented mask
     * @param depthImg depth source img
     * @param infImg rgb image where the filtered inference needs to be visualized
     * @param result semantic inference result. boxMasks will be changed to the filtered ones
     */

    /**
     * @brief perform background filtering with kmeans clustering for all objects using depth 
     * 
     * @param semanticIdMask [out] semantic ID mask
     * @param depthImg depth image
     * @param result semantic inference results
     * @param clustersCenters [out] vector of objects centroids after clusterization
     */
    void filterBackground(cv::Mat &semanticIdMask, const cv::Mat &depthImg, std::vector<OutputSeg> &result,
                                        std::vector<float> &clustersCenters, cv::Mat &rgbImg);    

    void realsenseKmeans(cv::Mat &semanticIdMask,const cv::Mat &depthImg, std::vector<OutputSeg> &result,
                                std::vector<float> &clustersCenters, cv::Mat &rgbImg);

    void pclClustering( pcl::PointCloud<PointSemXYZRGB>::Ptr pcl, 
                        PointSemXYZRGB &centroid,
                        pcl::PointCloud<PointSemXYZRGB>::Ptr discardedPoints, 
                        size_t &discardedCounter);

    void pclOutliersRemoval(pcl::PointCloud<PointSemXYZRGB>::Ptr pcl,
                            pcl::PointCloud<PointSemXYZRGB>::Ptr discardedPoints, 
                            size_t &discardedCounter);
};

