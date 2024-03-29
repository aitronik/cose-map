#pragma once
#include "cosemap_tools/inference_utils/inference_utils.h"
using namespace cv;
using namespace std;

bool CheckParams(int netHeight, int netWidth, const int* netStride, int strideSize) {
    if (netHeight % netStride[strideSize - 1] != 0 || netWidth % netStride[strideSize - 1] != 0)
    {
        cout << "Error:_netHeight and _netWidth must be multiple of max stride " << netStride[strideSize - 1] << "!" << endl;
        return false;
    }
    return true;
}

void LetterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params, const cv::Size& newShape,
    bool autoShape, bool scaleFill, bool scaleUp, int stride, const cv::Scalar& color)
{
    if (false) {
        int maxLen = MAX(image.rows, image.cols);
        outImage = Mat::zeros(Size(maxLen, maxLen), CV_8UC3);
        image.copyTo(outImage(Rect(0, 0, image.cols, image.rows)));
        params[0] = 1;
        params[1] = 1;
        params[3] = 0;
        params[2] = 0;
    }

    cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
        (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);

    float ratio[2]{ r, r };
    int new_un_pad[2] = { (int)std::round((float)shape.width * r),(int)std::round((float)shape.height * r) };

    auto dw = (float)(newShape.width - new_un_pad[0]);
    auto dh = (float)(newShape.height - new_un_pad[1]);

    if (autoShape)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        new_un_pad[0] = newShape.width;
        new_un_pad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f;
    dh /= 2.0f;

    if (shape.width != new_un_pad[0] && shape.height != new_un_pad[1])
    {
        cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
    }
    else {
        outImage = image.clone();
    }

    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    params[0] = ratio[0];
    params[1] = ratio[1];
    params[2] = left;
    params[3] = top;
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, std::vector<OutputSeg>& output, const MaskParams& maskParams) {
    //cout << maskProtos.size << endl;

    int seg_channels = maskParams.segChannels;
    int net_width = maskParams.netWidth;
    int seg_width = maskParams.segWidth;
    int net_height = maskParams.netHeight;
    int seg_height = maskParams.segHeight;
    float mask_threshold = maskParams.maskThreshold;
    Vec4f params = maskParams.params;
    Size src_img_shape = maskParams.srcImgShape;

    Mat protos = maskProtos.reshape(0, { seg_channels,seg_width * seg_height });

    Mat matmul_res = (maskProposals * protos).t();
    Mat masks = matmul_res.reshape(output.size(), { seg_width,seg_height });
    vector<Mat> maskChannels;
    split(masks, maskChannels);
    for (int i = 0; i < output.size(); ++i) {
        Mat dest, mask;
        //sigmoid
        cv::exp(-maskChannels[i], dest);
        dest = 1.0 / (1.0 + dest);

        Rect roi(int(params[2] / net_width * seg_width), int(params[3] / net_height * seg_height), int(seg_width - params[2] / 2), int(seg_height - params[3] / 2));
        dest = dest(roi);
        resize(dest, mask, src_img_shape, INTER_NEAREST);

        //crop
        Rect temp_rect = output[i].box;
        mask = mask(temp_rect) > mask_threshold;
        output[i].boxMask = mask;
    }
}

void GetMask2(const Mat& maskProposals, const Mat& mask_protos, OutputSeg& output, const MaskParams& maskParams) {
    int seg_channels = maskParams.segChannels;
    int net_width = maskParams.netWidth;
    int seg_width = maskParams.segWidth;
    int net_height = maskParams.netHeight;
    int seg_height = maskParams.segHeight;
    float mask_threshold = maskParams.maskThreshold;
    Vec4f params = maskParams.params;
    Size src_img_shape = maskParams.srcImgShape;

    Rect temp_rect = output.box;
    //crop from mask_protos
    int rang_x = floor((temp_rect.x * params[0] + params[2]) / net_width * seg_width);
    int rang_y = floor((temp_rect.y * params[1] + params[3]) / net_height * seg_height);
    int rang_w = ceil(((temp_rect.x + temp_rect.width) * params[0] + params[2]) / net_width * seg_width) - rang_x;
    int rang_h = ceil(((temp_rect.y + temp_rect.height) * params[1] + params[3]) / net_height * seg_height) - rang_y;

    //�������� mask_protos(roi_rangs).clone()λ�ñ�����˵�����output.box���ݲ��ԣ����߾��ο��1�����صģ����������ע�Ͳ��ַ�ֹ������
    rang_w = MAX(rang_w, 1);
    rang_h = MAX(rang_h, 1);
    if (rang_x + rang_w > seg_width) {
        if (seg_width - rang_x > 0)
            rang_w = seg_width - rang_x;
        else
            rang_x -= 1;
    }
    if (rang_y + rang_h > seg_height) {
        if (seg_height - rang_y > 0)
            rang_h = seg_height - rang_y;
        else
            rang_y -= 1;
    }

    vector<Range> roi_rangs;
    roi_rangs.push_back(Range(0, 1));
    roi_rangs.push_back(Range::all());
    roi_rangs.push_back(Range(rang_y, rang_h + rang_y));
    roi_rangs.push_back(Range(rang_x, rang_w + rang_x));

    //crop
    Mat temp_mask_protos = mask_protos(roi_rangs).clone();
    Mat protos = temp_mask_protos.reshape(0, { seg_channels,rang_w * rang_h });
    Mat matmul_res = (maskProposals * protos).t();
    Mat masks_feature = matmul_res.reshape(1, { rang_h,rang_w });
    Mat dest, mask;

    //sigmoid
    cv::exp(-masks_feature, dest);
    dest = 1.0 / (1.0 + dest);

    int left = floor((net_width / seg_width * rang_x - params[2]) / params[0]);
    int top = floor((net_height / seg_height * rang_y - params[3]) / params[1]);
    int width = ceil(net_width / seg_width * rang_w / params[0]);
    int height = ceil(net_height / seg_height * rang_h / params[1]);

    resize(dest, mask, Size(width, height), INTER_NEAREST);
    mask = mask(temp_rect - Point(left, top)) > mask_threshold;
    output.boxMask = mask;

}

void DrawPred(const Mat& img, const Mat &IdMask, Mat& infImg, vector<OutputSeg>& result,const std::vector<std::string>& classNames,const  int& classNumber) {
    Mat segMask = img.clone();    //creazione maschera copiando l'immagine originale

    uint8_t r, g, b;
    for(size_t i = 0; i < IdMask.rows; i++){
        for(size_t j = 0; j < IdMask.cols; j++){
            if(IdMask.at<uint8_t>(i,j) != 255){
                rainbow_rgb(IdMask.at<uint8_t>(i,j)/(double)classNumber, b, g, r);
                segMask.at<cv::Vec3b>(i, j) = cv::Vec3b(r, g, b);
            }
        }
    }

    addWeighted(infImg, 0.5, segMask, 0.5, 0, infImg); //aggiunge la maschera semitrasparente all'immagine originale


    for (int i = 0; i < result.size(); i++) {
        
        int left, top;
        left = result[i].box.x;
        top = result[i].box.y;

        uint8_t r, g, b;
        rainbow_rgb(result[i].id/(double)classNumber, b, g, r);

        cv::Scalar color(r, g, b, 255);

        int color_num = i;
        rectangle(infImg, result[i].box, color, 2, 8);

        std::string confidenceStr = std::to_string(result[i].confidence);
        size_t decimalPos = confidenceStr.find('.');
        if (decimalPos != std::string::npos) {
            // Limita a due cifre decimali
            confidenceStr = confidenceStr.substr(0, decimalPos + 3); // +3 per includere il punto e due cifre decimali
        }
        string label = classNames[result[i].id] + ":" + confidenceStr;


        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 1.0, 1, &baseLine);
        top = max(top, labelSize.height);
        rectangle(infImg, Point(left, top /*- int(1.5 * labelSize.height)*/), Point(left + int(labelSize.width), top +  int(1.5 * labelSize.height)), Scalar(r, g, b, 255), FILLED);
        
        cv::putText(infImg, label, Point(left, top+(labelSize.height) ), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255, 255), 3);

    }
    

}

bool inference(const cv::Mat& srcImg, std::vector<OutputSeg>& output, cv::dnn::Net onnnxNet, int classesNumber, netParams netParams){


	output.clear();

	// initializing inference output
	std::vector<int> class_ids;
	std::vector<float> confidences;
	std::vector<cv::Rect> boxes;
	std::vector<std::vector<float>> picked_proposals;  //output0[:,:, 4 + _className.size():net_width]===> for mask

	
	//creating a blob from the source image 
	cv::Mat blob;
	cv::Mat netInputImg;
	cv::Vec4d params;
	LetterBox(srcImg, netInputImg, params, cv::Size(netParams.netWidth, netParams.netHeight));
	cv::dnn::blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netParams.netWidth, netParams.netHeight), cv::Scalar(0, 0, 0), true, false);

	// setting the blob as input for the net and spin
	onnnxNet.setInput(blob);
	std::vector<cv::Mat> net_output_img;
	std::vector<cv::String> output_layer_names{ "output0","output1" };
	onnnxNet.forward(net_output_img, output_layer_names); 

	// processing the output data
	int net_width = classesNumber + 4 + netParams.segChannels;
	cv::Mat output0 =cv::Mat( cv::Size(net_output_img[0].size[2], net_output_img[0].size[1]), CV_32F, (float*)net_output_img[0].data).t();  //[bs,116,8400]=>[bs,8400,116]
	int rows = output0.rows;
	float* pdata = (float*)output0.data;

	for (int r = 0; r < rows; ++r) {
			cv::Mat scores(1, classesNumber, CV_32FC1, pdata + 4);
			cv::Point classIdPoint;
			double max_class_socre;
			minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
			max_class_socre = (float)max_class_socre;

			if (max_class_socre >= netParams.classThreshold) {
				std::vector<float> temp_proto(pdata + 4 + classesNumber, pdata + net_width);
				picked_proposals.push_back(temp_proto);
				
				//rect [x,y,w,h]
				float x = (pdata[0] - params[2]) / params[0];  
				float y = (pdata[1] - params[3]) / params[1];  
				float w = pdata[2] / params[0];  
				float h = pdata[3] / params[1];  
				int left = MAX(int(x - 0.5 * w + 0.5), 0);
				int top = MAX(int(y - 0.5 * h + 0.5), 0);
				class_ids.push_back(classIdPoint.x);
				confidences.push_back(max_class_socre);
				boxes.push_back(cv::Rect(left, top, int(w + 0.5), int(h + 0.5)));
			}
			pdata += net_width;//next line
	}

	//Non Maximum Suppression
	std::vector<int> nms_result;
	cv::dnn::NMSBoxes(boxes, confidences, netParams.classThreshold, netParams.nmsThreshold, nms_result);
	std::vector<std::vector<float>> temp_mask_proposals;
	cv::Rect holeImgRect(0, 0, srcImg.cols, srcImg.rows);

	//filling the segmentation output structure
	for (size_t i = 0; i < nms_result.size(); ++i) {
		int idx = nms_result[i];
		OutputSeg result;
		result.id = class_ids[idx];
		result.confidence = confidences[idx];
		result.box = boxes[idx] & holeImgRect;
		temp_mask_proposals.push_back(picked_proposals[idx]);
		output.push_back(result);
	}

	MaskParams mask_params;
	mask_params.params = params;
	mask_params.srcImgShape = srcImg.size();
	for (size_t i = 0; i < temp_mask_proposals.size(); ++i) {
		GetMask2(cv::Mat(temp_mask_proposals[i]).t(), net_output_img[1], output[i], mask_params);
	}

	if (output.size())
		return true;
	else
		return false;

}

void semanticMasks(const cv::Mat& rgbImg, cv::Mat& outIdMask, cv::Mat& outConfMask,std::vector<OutputSeg>& result,
                   const cv::dnn::Net &net,const  int classNumber,const  netParams &netParams, const std::vector<std::string> classNames,
                   double seconds){

    // making inference 
	inference(rgbImg, result, net, classNumber, netParams);
    
    // TRICK ALWAYS: person remove
    for(size_t i = 0; i < result.size(); i++){
        if (result[i].id == 0 /* person*/){
            result.erase(result.begin() + i);
        }
    }

    
	//filling masks with semantic data
	for (size_t i = 0; i < result.size(); i++){
		if (result[i].boxMask.rows && result[i].boxMask.cols > 0){
			outIdMask(result.at(i).box).setTo(result.at(i).id, result.at(i).boxMask);
			outConfMask(result.at(i).box).setTo(result.at(i).confidence, result.at(i).boxMask);
		}
	}
}