#include "semantic_pcl/semantic_pcl.h"

SemanticPcl::SemanticPcl(){
        
    m_useCuda = true;
    m_classesNumber = 0;

    /* Net parameters*/
    m_netParams.netWidth = 640;         
    m_netParams.netHeight = 640;
    m_netParams.segWidth = 160;     
    m_netParams.segHeight = 160;
    m_netParams.segChannels = 32;
    m_netParams.classThreshold = 0.25;
    m_netParams.nmsThreshold = 0.45;
}

SemanticPcl::~SemanticPcl(){

}

bool SemanticPcl::init( std::string labelPath, 
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
                        int depthPixelSkip){
        
    if (readONNXModel(modelPath)){
        std::cout <<"[SemanticPcl::init]: Model loaded!" <<std::endl;
    }
    else{
        std::cout <<"[SemanticPcl::init]: Couldn't load model!" <<std::endl;
        std::cout <<"[SemanticPcl::init]: Path \""<< modelPath << "\" not found" <<std::endl;
        return false;
    }
        
    if(!getClasses(labelPath, m_classNames, m_classesNumber)){
        std::cout <<"[SemanticPcl::init]: Couldn't load classes!" <<std::endl;
        std::cout <<"[SemanticPcl::init]: Path \""<< labelPath << "\" not found" <<std::endl;
        return false;
    }else{
        std::cout << "[SemanticPcl::init]: Classes loaded [" <<m_classesNumber<<"] " <<std::endl;
    }

    m_realColor = realColor;
    m_netParams.classThreshold = confidenceThreshold;

    m_depthCameraInfo = depthCameraInfo;
    m_rgbCameraInfo = rgbCameraInfo;
    m_depthCameraInfo.Cx = m_depthCameraInfo.Cx;
    m_depthCameraInfo.Cy = m_depthCameraInfo.Cy;
    m_depthCameraInfo.Fx = m_depthCameraInfo.Fx;
    m_depthCameraInfo.Fy = m_depthCameraInfo.Fy;
    m_rgbCameraInfo.Cx = m_rgbCameraInfo.Cx;        
    m_rgbCameraInfo.Cy = m_rgbCameraInfo.Cy;    
    m_rgbCameraInfo.Fx = m_rgbCameraInfo.Fx;    
    m_rgbCameraInfo.Fy = m_rgbCameraInfo.Fy;
    m_depthCameraInfo.depthMaxDist = depthMaxDistance;
    m_isCameraSet = true;
    
    m_clusterObjectsPcl = clusterObjectsPcl;
    m_clusterTolerance = clusterTolerance;
    m_clusterMinSizeRatio = clusterMinSizeRatio;
    
    m_outliersRemoval = outliersRemoval;
    m_outliersRemovalMeanK = outliersRemovalMeanK;
    m_outliersRemovalStdDevThresh = outliersRemovalStdDevThresh;

    m_includeBackgroundPoints = includeFilteredPoints;
    
    m_depthPixelSkip = depthPixelSkip;
    m_pclSizeCounter = 0;
    m_pclSizeMean = 0.0;
    
    m_meanCallbackTime = 0;
    m_meanTimeCounter = 0;
    std::cout <<"[SemanticPcl::init]: SemanticPcl initialized!"<<std::endl;
    return true;
}

void SemanticPcl::imageCallback(const   cv_bridge::CvImageConstPtr cvPtrRGB,
                                        cv_bridge::CvImageConstPtr cvPtrDepth,
                                        cosemap_msgs::msg::SemanticPcl& msgPointCloud,
                                        sensor_msgs::msg::PointCloud2& msgPointCloudViz,
                                        cv::Mat& infImg){

    if(m_isCameraSet == true){        
        
        auto start_time = std::chrono::high_resolution_clock::now();

        cv::Mat rgbImg = cvPtrRGB->image;
        cv::Mat depthImg = cvPtrDepth->image;
        int depthCol = depthImg.cols;
        int depthRow = depthImg.rows;
        int rgbCol = rgbImg.cols;
        int rgbRow = rgbImg.rows;        
        cv::resize(depthImg, depthImg, cv::Size(depthCol, depthRow), cv::INTER_CUBIC);
        cv::resize(rgbImg,   rgbImg,   cv::Size(rgbCol, rgbRow), cv::INTER_CUBIC);
        infImg = rgbImg.clone();               
        
        // Creating semantic Masks of the image
        cv::Mat semanticIDMask = cv::Mat::ones(rgbImg.rows, rgbImg.cols, CV_8U) * 255;
        cv::Mat semanticConfidenceMask = cv::Mat::ones(rgbImg.rows, rgbImg.cols, CV_32F) * 0.0;
        
        /* segmentation results*/
        std::vector<OutputSeg> result;
        
        // filling ID, conf and binary masks with semantic info
        cv::Mat resultIndexMask(rgbImg.rows, rgbImg.cols, CV_8U, cv::Scalar(255));
        
        semanticMasks( rgbImg, semanticIDMask, semanticConfidenceMask, resultIndexMask, result);

        // drawing filtered prediction on output inference image
        DrawPred(rgbImg, semanticIDMask, infImg, result, m_classNames, m_classesNumber);

        // backgroundPcl
        pcl::PointCloud<PointSemXYZRGB>::Ptr backgroundPcl(new pcl::PointCloud<PointSemXYZRGB>);
        std::vector<pcl::PointCloud<PointSemXYZRGB>::Ptr> objPclPtrs(result.size());
        std::vector<pcl::CentroidPoint<PointSemXYZRGB>> objPclCentroids(result.size());
        for (size_t i = 0; i < objPclPtrs.size(); i++){
            objPclPtrs[i] = pcl::PointCloud<PointSemXYZRGB>::Ptr(new pcl::PointCloud<PointSemXYZRGB>);
        }
        
        fillPcl(backgroundPcl, objPclPtrs, objPclCentroids, depthImg, rgbImg, semanticIDMask, semanticConfidenceMask, resultIndexMask, result);

        pcl::PointCloud<PointSemXYZRGB>::Ptr discardedPoints(new pcl::PointCloud<PointSemXYZRGB>);
        discardedPoints->resize(depthCol*depthRow);
        size_t discardedCounter = 0;
        for (size_t i = 0; i < objPclPtrs.size(); i++){
            if(m_outliersRemoval){
            
                pclOutliersRemoval(objPclPtrs[i], discardedPoints, discardedCounter);
                
            }
            if(m_clusterObjectsPcl){
                
                PointSemXYZRGB centroid;
                objPclCentroids[i].get(centroid);

                auto pclClustering_start_time = std::chrono::high_resolution_clock::now();

                pclClustering(objPclPtrs[i], centroid, discardedPoints, discardedCounter);
            
                auto pclClustering_end_time = std::chrono::high_resolution_clock::now();
                auto pclClustering_duration = std::chrono::duration_cast<std::chrono::milliseconds>(pclClustering_end_time - pclClustering_start_time);

            }

        }

        // filling background part of msg
        pcl::toROSMsg(*backgroundPcl, msgPointCloud.background);
        msgPointCloud.background.is_bigendian       = false; 
        msgPointCloud.background.is_dense           = true;
        

        // copying to message only not empty Point Clouds
        int count = 0;
        for(size_t i = 0; i < objPclPtrs.size(); i++){
            if (objPclPtrs.at(i)->size() != 0){
                count ++;
            }
        }

        msgPointCloud.objs_pcl.resize(count);
        int targetIndex = 0; // Variabile di controllo per gli indici di msgPointCloud

        for (size_t i = 0; i < objPclPtrs.size(); i++) {
            if (objPclPtrs[i]->size() != 0) {
                pcl::toROSMsg(*objPclPtrs[i], msgPointCloud.objs_pcl[targetIndex]);

                msgPointCloud.objs_pcl[targetIndex].is_bigendian = false;
                msgPointCloud.objs_pcl[targetIndex].is_dense = true;
                targetIndex++;
            }
        }

        // std::cout<<"m_includeBackgroundPoints = "<<m_includeBackgroundPoints<<std::endl;
        pcl::PointCloud<PointSemXYZRGB>::Ptr finalPcl(new pcl::PointCloud<PointSemXYZRGB>);
        if(m_includeBackgroundPoints){ *finalPcl += *backgroundPcl;}
        bool test = true;

        for (const auto& cloudPtr : objPclPtrs){
            *finalPcl += *cloudPtr;
        }

        m_pclSizeMean = ((m_pclSizeMean*m_pclSizeCounter) + finalPcl->size()) / (m_pclSizeCounter + 1);
        m_pclSizeCounter++;

        // // converting pcl to ROS message PCL and setting params
        pcl::toROSMsg(*finalPcl, msgPointCloudViz);

        msgPointCloudViz.is_bigendian       = false; 
        msgPointCloudViz.is_dense           = true;

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        m_meanCallbackTime = ( (m_meanCallbackTime * m_meanTimeCounter) + duration.count()) / (m_meanTimeCounter + 1);
        m_meanTimeCounter ++;
    }

}

void SemanticPcl::fillPcl(  pcl::PointCloud<PointSemXYZRGB>::Ptr backgroundPcl,
                            std::vector<pcl::PointCloud<PointSemXYZRGB>::Ptr> objPclPtrs,
                            std::vector<pcl::CentroidPoint<PointSemXYZRGB>> &objPclCentroids,
                            const cv::Mat &depthImg, 
                            const cv::Mat &rgbImg, 
                            const cv::Mat &semanticIDMask,
                            const cv::Mat &confidenceMask,
                            const cv::Mat &resultIndexMask,
                            const std::vector<OutputSeg> &result){
    // resize to the maximum point possible
    backgroundPcl->resize(depthImg.rows*depthImg.cols);
    for(size_t i = 0; i < objPclPtrs.size(); i++){
        objPclPtrs[i]->resize(result[i].box.height * result[i].box.width);
    }
    
    // init counters
    size_t backgroundPclCounter = 0;
    std::vector<size_t> objPclCounters(objPclPtrs.size(), 0);

    //init iterators
    pcl::PointCloud<PointSemXYZRGB>::iterator backgroundIter = backgroundPcl->begin();
    std::vector<pcl::PointCloud<PointSemXYZRGB>::iterator> objPclIters(objPclPtrs.size());
    for(size_t i = 0; i < objPclIters.size(); i++){
        objPclIters[i] = objPclPtrs[i]->begin();
    }


    PointSemXYZRGB point;
    float depth_m;
    int u, v;
    // creating semantic pointcloud from semantic masks and depth images
    for (int i = 0; i < (depthImg.rows); i+=m_depthPixelSkip){              //+){
        for (int j = 0; j < (depthImg.cols); j+=m_depthPixelSkip){          //+){
            
            //depth measure in meters

            if(depthImg.type() == CV_16U){                
                depth_m = (float)depthImg.at<uint16_t>(i, j) / 1000.0; //realsense depth is uint16 in millimeters
            }else if (depthImg.type() == CV_32F){
                depth_m = depthImg.at<float>(i, j);
            }
            
            if( (depth_m > 0.0) && (depth_m < m_depthCameraInfo.depthMaxDist) ){
                Depth2RGB(i, j, u, v, m_depthCameraInfo, m_rgbCameraInfo);

                if ((u >= 0) && (u < (rgbImg.rows)) &&
                    (v >= 0) && (v < (rgbImg.cols))) {

                    int32_t dxPixel = j - m_depthCameraInfo.Cx;
                    int32_t dyPixel = i - m_depthCameraInfo.Cy;
                    // XYZ points
                    point.x = depth_m * (dxPixel/m_depthCameraInfo.Fx);
                    point.y = depth_m * (dyPixel/m_depthCameraInfo.Fy);
                    point.z = depth_m;

                    // Semantic pixel info
                    size_t index = (v * rgbImg.cols ) + (u);
                    // point.id = semanticIDMask.data[index];
                    point.id = semanticIDMask.at<uint8_t>(u,v);
                    point.conf = confidenceMask.at<float>(u,v);

                    if(m_realColor == true){
                        point.b = rgbImg.at<cv::Vec3b>(u,v)[0];
                        point.g = rgbImg.at<cv::Vec3b>(u,v)[1];
                        point.r = rgbImg.at<cv::Vec3b>(u,v)[2];
                    }else{
                        if(point.id != 255){
                            
                            // setting colors from semantics
                            double ratio = point.id/(double)m_classesNumber;
                            rainbow_rgb(ratio, point.r, point.g, point.b);
                        }else{
                            point.r = 255; point.g = 255; point.b = 255;
                        }
                    }


                    // if( !(point.id == 62 && point.z < 1.5)){
                        if(resultIndexMask.at<uint8_t>(u,v) == 255){
                            // background point
                            copyPoint(point, backgroundIter);
                            backgroundIter++;
                            backgroundPclCounter++;
                        }else{
                            //point belonging to an object
                            int resultIndex = resultIndexMask.at<uint8_t>(u,v);
                            copyPoint(point, objPclIters[resultIndex]);
                            objPclCentroids[resultIndex].add(point);
                            
                            objPclIters[resultIndex]++;
                            objPclCounters[resultIndex]++;
                        }
                    // } 


                }
            }
        }
    }

    // final resize
    backgroundPcl->resize(backgroundPclCounter);
    for(size_t i = 0; i < objPclPtrs.size(); i++){
        objPclPtrs[i]->resize(objPclCounters[i]);
    }

}

void SemanticPcl::filterBackground(cv::Mat &semanticIdMask,const cv::Mat &depthImg, std::vector<OutputSeg> &result,
                                    std::vector<float> &clustersCenters, cv::Mat &rgbImg){                            
    // switching between depth image sources

}


void SemanticPcl::copyPoint(PointSemXYZRGB &fromPoint, pcl::PointCloud<PointSemXYZRGB>::iterator toIterator){
    toIterator->x = fromPoint.x;
    toIterator->y = fromPoint.y;
    toIterator->z = fromPoint.z;

    toIterator->r = fromPoint.r;
    toIterator->g = fromPoint.g;
    toIterator->b = fromPoint.b;

    toIterator->id = fromPoint.id;
    toIterator->conf = fromPoint.conf;
}

bool SemanticPcl::readONNXModel(std::string modelPath){
    try {
        m_onnnxNet = cv::dnn::readNet(modelPath);
    }catch (const std::exception&){
        return false;
            }
        
    if (m_useCuda) {
        //cuda
        m_onnnxNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        m_onnnxNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA); //or DNN_TARGET_CUDA_FP16
        std::cout<<"[SemanticPcl::readONNXModel]: cuda set as dnn backend"<<std::endl;
    }
    else {
        //cpu
        std::cout << "SemanticPcl::readONNXModel]: CPU set as dnn backend" << std::endl;
        m_onnnxNet.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    	m_onnnxNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	}
	return true;
}

void SemanticPcl::semanticMasks(const cv::Mat& rgbImg, cv::Mat& idMask, cv::Mat& confMask, cv::Mat& resultIndexMask, std::vector<OutputSeg>& result){

	inference(rgbImg, result, m_onnnxNet, m_classesNumber, m_netParams);
    
	//filling masks with semantic data
	for (size_t i = 0; i < result.size(); i++){
		if (result[i].boxMask.rows && result[i].boxMask.cols > 0){
			idMask(result.at(i).box).setTo(result.at(i).id, result.at(i).boxMask);
			confMask(result.at(i).box).setTo(result.at(i).confidence, result.at(i).boxMask);
            resultIndexMask(result.at(i).box).setTo(cv::Scalar(i), result.at(i).boxMask);
		}
	}
}


void SemanticPcl::pclClustering(pcl::PointCloud<PointSemXYZRGB>::Ptr pcl, 
                                PointSemXYZRGB &pclCentroid,
                                pcl::PointCloud<PointSemXYZRGB>::Ptr discardedPoints, 
                                size_t &discardedCounter){
    if(pcl->empty()){
        return;
    }

    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<PointSemXYZRGB>::Ptr search_tree (new pcl::search::KdTree<PointSemXYZRGB>);
    pcl::console::TicToc tt;

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointSemXYZRGB> cec (true);
    cec.setInputCloud(pcl);
    cec.setSearchMethod(search_tree);
    cec.setConditionFunction(&isSemanticEqual);
    cec.setClusterTolerance(m_clusterTolerance);
    
    cec.setMinClusterSize(pcl->size() * m_clusterMinSizeRatio); // pcl->size()/6
    cec.setMaxClusterSize(pcl->size());
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);
    
    // std::cout<<"CLUSTERS = "<<clusters->size()<<", SMALL CLUSTERS = "<<small_clusters->size()<<", LARGE CLUSTERS"<<large_clusters->size() <<std::endl;

    if(clusters->size() == 0){
        std::cout<<"PclClustering: Ripescaggio"<<std::endl;
        // ripescaggio dei cluster scartati
        size_t maxClusterSize = 0;
        for (size_t i = 0; i< small_clusters->size(); i++){
            if(small_clusters->at(i).indices.size() > maxClusterSize ){
                maxClusterSize = small_clusters->at(i).indices.size();
            }
        }

        // discarding all clusters minor to 70% of maxSize
        pcl::IndicesPtr toRemove(new std::vector<int>);
        for (size_t i = 0; i< small_clusters->size(); i++){
            if(small_clusters->at(i).indices.size() < (0.1 * maxClusterSize)){
                toRemove->insert(toRemove->end(), small_clusters->at(i).indices.begin(),small_clusters->at(i).indices.end());
            }
        }
        pcl::ExtractIndices<PointSemXYZRGB> extract;
        extract.setIndices(toRemove);
        extract.setInputCloud(pcl);
        extract.setNegative(true);
        extract.filter(*pcl);
    }else{
        
        // closest cluster
        size_t closestClusterIndex = 0;
        float minDistance = std::numeric_limits<float>::max();
        size_t clusterIndex = 0;
        for(const auto &cluster : (*clusters)){
            pcl::CentroidPoint<PointSemXYZRGB> centroid;
            for (size_t i = 0; i < cluster.indices.size(); i++){
                centroid.add( pcl->at(cluster.indices[i]) );
            }
            PointSemXYZRGB centroidPoint;
            centroid.get(centroidPoint);
            float distance = sqrt(pow(centroidPoint.x,2) + pow(centroidPoint.y,2) + pow(centroidPoint.z,2)); 
            if(  distance < minDistance){
                minDistance = distance;
                closestClusterIndex = clusterIndex;
            }

            clusterIndex++;
        }

        // Discarding all the remaaining clusters (small, normal)  
        // TODO: this point are completely discarded. they need to be added  

        pcl::IndicesPtr toRemoveIndices(new std::vector<int>);

        // rimozione dei punti all'interno di small_clusters
        for (const auto &cluster : (*small_clusters)) {
            toRemoveIndices->insert(toRemoveIndices->end(), cluster.indices.begin(), cluster.indices.end());
        }

        // rimozione dei punti all'interno dei cluster non closest
        for(size_t i = 0; i < clusters->size(); i++){
            if(i != closestClusterIndex){
                toRemoveIndices->insert(toRemoveIndices->end(), clusters->at(i).indices.begin(), clusters->at(i).indices.end());
            }
        }

        pcl::ExtractIndices<PointSemXYZRGB> extract;
        extract.setIndices(toRemoveIndices);
        extract.setInputCloud(pcl);
        extract.setNegative(true);
        extract.filter(*pcl);
    }

    // // iteration among small clusters (for every cluster)
    // for (size_t i = 0; i< small_clusters->size(); i++){
    //     for (size_t j = 0; j< small_clusters->at(i).indices.size(); j++){
    //         size_t index = small_clusters->at(i).indices.at(j);
    //         //discardedPoints->at(discardedCounter) = pcl->at(index);
    //         //discardedPoints->at(discardedCounter).id = 255;
    //         //discardedPoints->at(discardedCounter).conf = 0.0;
    //         //discardedPoints->at(discardedCounter).r = 102;
    //         //discardedPoints->at(discardedCounter).g = 255;
    //         //discardedPoints->at(discardedCounter).b = 51;
    //         ++discardedCounter;
    //     }
    // }



}

void SemanticPcl::pclOutliersRemoval(   pcl::PointCloud<PointSemXYZRGB>::Ptr pcl,
                                        pcl::PointCloud<PointSemXYZRGB>::Ptr discardedPoints, 
                                        size_t &discardedCounter){
    
    if(pcl->empty()){
        return;
    }
    // std::cout<<"OK 1.1 , pcl size = "<<pcl->size()<<std::endl;
    // STATISTICAL OUTLIERS REMOVAL
    pcl::StatisticalOutlierRemoval<PointSemXYZRGB> sor;
    pcl::IndicesPtr indices(new std::vector<int>);
    sor.setInputCloud(pcl);
    sor.setMeanK (m_outliersRemovalMeanK);
    sor.setStddevMulThresh (m_outliersRemovalStdDevThresh);
    sor.setNegative(false);
    // std::cout<<"OK 1.2"<<std::endl;
    sor.filter(*indices);
    // sor.filter(*pcl);
    // std::cout<<"OK 1.2"<<std::endl;
    
    // // copying outliers to discarded
    // for (size_t j = 0; j < indices->size(); j++){
    //     discardedPoints->at(discardedCounter) = pcl->at(indices->at(j));
    //     discardedPoints->at(discardedCounter).id = 255;
    //     discardedPoints->at(discardedCounter).conf = 0.0;
    //     discardedPoints->at(discardedCounter).r = 255;
    //     discardedPoints->at(discardedCounter).g = 153;
    //     discardedPoints->at(discardedCounter).b = 0;
    //     ++discardedCounter;
    // }

    pcl::ExtractIndices<PointSemXYZRGB> extract;
    extract.setIndices(indices);
    extract.setInputCloud(pcl);
    extract.setNegative(false);
    extract.filter(*pcl);

}


bool isSemanticEqual (const PointSemXYZRGB& point_a, const PointSemXYZRGB& point_b, float /*squared_distance*/){
    
    // points with same semantic id are considered with equal semantics
    if (point_a.id == point_b.id){
        return (true);
    }
    else{
      return (false);
    }
}