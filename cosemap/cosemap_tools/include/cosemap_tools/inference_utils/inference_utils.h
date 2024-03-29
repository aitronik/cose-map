#pragma once
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "cosemap_tools/semantic_utils/semantic_utils.h"

#define YOLO_P6 false //�Ƿ�ʹ��P6ģ��
#define ORT_OLD_VISON 12  //ort1.12.0 ֮ǰ�İ汾Ϊ�ɰ汾API

struct OutputSeg {
    int id;             
    float confidence;   
    cv::Rect box;      
    cv::Mat boxMask;    
};   

struct MaskParams {
    int segChannels = 32;
    int segWidth = 160;
    int segHeight = 160;
    int netWidth = 640;
    int netHeight = 640;
    float maskThreshold = 0.5;
    cv::Size srcImgShape;
    cv::Vec4d params;

};

struct netParams{
    int netWidth;       //ONNX
    int netHeight;      //ONNX
    int segWidth;            //_segWidth=_netWidth/mask_ratio
    int segHeight;
    int segChannels;
    /* class detection threshold*/
    float classThreshold;
    /* Non maximum suppression threshold*/
    float nmsThreshold;
};

bool CheckParams(int netHeight, int netWidth, const int* netStride, int strideSize); 
void DrawPred(const cv::Mat& img, const cv::Mat &IdMask, cv::Mat& infImg, std::vector<OutputSeg>& result,const std::vector<std::string>& classNames,const int& classNumber);

void LetterBox(const cv::Mat& image, cv::Mat& outImage,
    cv::Vec4d& params, //[ratio_x,ratio_y,dw,dh]
    const cv::Size& newShape = cv::Size(640, 640),
    bool autoShape = false,
    bool scaleFill = false,
    bool scaleUp = true,
    int stride = 32,
    const cv::Scalar& color = cv::Scalar(114, 114, 114));

void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, std::vector<OutputSeg>& output, const MaskParams& maskParams);
void GetMask2(const cv::Mat& maskProposals, const cv::Mat& maskProtos, OutputSeg& output, const MaskParams& maskParams);

bool inference(const cv::Mat& srcImg, std::vector<OutputSeg>& output, cv::dnn::Net onnnxNet, int classesNumber, netParams netParams);


void semanticMasks(const cv::Mat& rgbImg, cv::Mat& outIdMask, cv::Mat& outConfMask, std::vector<OutputSeg>& result,
                   const cv::dnn::Net &net,const  int classNumber,const  netParams &netParams, const std::vector<std::string> classNames, double seconds);