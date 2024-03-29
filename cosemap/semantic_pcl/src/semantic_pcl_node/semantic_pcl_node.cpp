#include "semantic_pcl/semantic_pcl_node.h"

SemanticPclNode::SemanticPclNode() : BaseNode("semantic_pcl_node"), SemanticPcl(){
    /* parameters */
    m_cameraFrameId             = "camera_rgb_frame";

}

SemanticPclNode::~SemanticPclNode(){
    delete m_sync;
}

void SemanticPclNode::init(){
    
    /*parameters. */
    std::string labelPathFile = "";
    std::string modelPathFile = "";
    std::string rgbTopic = "";
    std::string depthTopic = "";
    std::string pclTopic = "";
    std::string inferenceImageTopic = "";
    float depthMaxDistance;
    float confidenceThreshold;
    bool realColor;
    int k;
    cameraParams depthCameraInfo;
    cameraParams rgbCameraInfo;
    bool clusterObjectsPcl;
    bool outliersRemoval;
    float clusterTolerance;
    float clusterMinSizeRatio;
    int outliersRemovalMeanK;
    float outliersRemovalStdDevThresh;
    bool includeBackgroundPoints;
    int depthPixelSkip;

    /* Get parameter. */
    this->declare_parameter<std::string>("labels_file_path",labelPathFile);
    this->declare_parameter<std::string>("model_file_path",modelPathFile);
    this->declare_parameter<std::string>("camera_frame_id", m_cameraFrameId);
    this->declare_parameter<std::string>("rgb_topic", rgbTopic);
    this->declare_parameter<std::string>("depth_topic", depthTopic);
    this->declare_parameter<std::string>("point_cloud_topic", pclTopic);
    this->declare_parameter<std::string>("inference_image_topic", inferenceImageTopic);    
    this->declare_parameter<float>("depth_max_distance", depthMaxDistance);
    this->declare_parameter<bool>("real-sem_colors", realColor);
    this->declare_parameter<float>("inference_confidence_threshold",confidenceThreshold);
    this->declare_parameter<float>("depth_fx", depthCameraInfo.Fx);
    this->declare_parameter<float>("depth_fy", depthCameraInfo.Fy);
    this->declare_parameter<float>("depth_cx", depthCameraInfo.Cx);
    this->declare_parameter<float>("depth_cy", depthCameraInfo.Cx);
    this->declare_parameter<float>("rgb_fx", rgbCameraInfo.Fx);
    this->declare_parameter<float>("rgb_fy", rgbCameraInfo.Fy);
    this->declare_parameter<float>("rgb_cx", rgbCameraInfo.Cx);
    this->declare_parameter<float>("rgb_cy", rgbCameraInfo.Cx);
    this->declare_parameter<float>("cluster_tolerance", clusterTolerance);
    this->declare_parameter<float>("cluster_min_size_ratio", clusterMinSizeRatio);
    this->declare_parameter<bool>("cluster_objects_pcl", clusterObjectsPcl);
    this->declare_parameter<bool>("outliers_removal", outliersRemoval);
    this->declare_parameter<int>("outliers_removal_mean_k", outliersRemovalMeanK);
    this->declare_parameter<float>("outliers_removal_std_dev_thresh", outliersRemovalStdDevThresh);
    this->declare_parameter<bool>("include_background_points", includeBackgroundPoints);
    this->declare_parameter<int>("depth_pixel_skip", depthPixelSkip);


    this->get_parameter("labels_file_path", labelPathFile);
    this->get_parameter("model_file_path", modelPathFile);
    this->get_parameter("rgb_topic", rgbTopic);
    this->get_parameter("depth_topic", depthTopic);
    this->get_parameter("point_cloud_topic", pclTopic);
    this->get_parameter("inference_image_topic", inferenceImageTopic);
    this->get_parameter("depth_max_distance", depthMaxDistance);
    this->get_parameter("real-sem_colors", realColor);
    this->get_parameter("inference_confidence_threshold",confidenceThreshold);
    this->get_parameter("depth_fx", depthCameraInfo.Fx);
    this->get_parameter("depth_fy", depthCameraInfo.Fy);
    this->get_parameter("depth_cx", depthCameraInfo.Cx);
    this->get_parameter("depth_cy", depthCameraInfo.Cy);
    this->get_parameter("rgb_fx", rgbCameraInfo.Fx);
    this->get_parameter("rgb_fy", rgbCameraInfo.Fy);
    this->get_parameter("rgb_cx", rgbCameraInfo.Cx);
    this->get_parameter("rgb_cy", rgbCameraInfo.Cy);
    this->get_parameter("camera_frame_id", m_cameraFrameId);
    this->get_parameter("cluster_tolerance", clusterTolerance);
    this->get_parameter("cluster_min_size_ratio", clusterMinSizeRatio);
    this->get_parameter("cluster_objects_pcl", clusterObjectsPcl);
    this->get_parameter("outliers_removal", outliersRemoval);
    this->get_parameter("outliers_removal_mean_k", outliersRemovalMeanK);
    this->get_parameter("outliers_removal_std_dev_thresh", outliersRemovalStdDevThresh);
    this->get_parameter("include_background_points", includeBackgroundPoints);
    this->get_parameter("depth_pixel_skip", depthPixelSkip);


    if (labelPathFile == ""){
        std::cout << "ERROR - labels_file_path not set"<< std::endl;
        exit(-1);
    }
    if (modelPathFile == ""){
        std::cout << "ERROR - model_file_path not set"<< std::endl;
        exit(-1);
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(
                    rclcpp::QoSInitialization(
                        qos_profile.history,
                        qos_profile.depth),
                    qos_profile);

    /* Subscribers*/
    /* Sync RGB and Depth subscribers*/
    m_subRGB = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), rgbTopic, qos_profile);
    m_subDepth = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), depthTopic, qos_profile);
    
    // Creating and binding the two subscription to the same callback
    m_sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *m_subRGB, *m_subDepth);
    m_sync->registerCallback(std::bind(&SemanticPclNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

    /* Publishers*/
    /* Point Cloud publisher*/
    // m_pubSemPointCloud      =   this->create_publisher<sensor_msgs::msg::PointCloud2>(pclTopic, qos);

    m_pubSemPointCloudViz      =   this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_viz", qos);
    m_pubSemPointCloud      =   this->create_publisher<cosemap_msgs::msg::SemanticPcl>(pclTopic, qos);
    
    m_pubInferenceImage     =   this->create_publisher<sensor_msgs::msg::Image>(inferenceImageTopic, qos);

    bool initialized = SemanticPcl::init(   labelPathFile,
                                            modelPathFile,
                                            depthMaxDistance,
                                            realColor,
                                            confidenceThreshold,
                                            rgbCameraInfo,
                                            depthCameraInfo,
                                            clusterTolerance,
                                            clusterMinSizeRatio,
                                            clusterObjectsPcl,
                                            outliersRemoval,
                                            outliersRemovalMeanK,
                                            outliersRemovalStdDevThresh,
                                            includeBackgroundPoints,
                                            depthPixelSkip);
    
    
    if (!initialized){
        std::cout << "ERROR - Could not initialize the module"<< std::endl;
        exit(-1);
    }
    std::cout <<"SemanticPclNode initialized!"<<std::endl;
}

void SemanticPclNode::step(){

}


void SemanticPclNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgbMsg, const sensor_msgs::msg::Image::ConstSharedPtr& depthMsg){

    //converting image_msg into cv::Mat
    cv_bridge::CvImageConstPtr cvPtrRGB;
    try {
        cvPtrRGB = cv_bridge::toCvShare(rgbMsg);
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cvPtrDepth;
    try {
        cvPtrDepth = cv_bridge::toCvShare(depthMsg);
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cosemap_msgs::msg::SemanticPcl msgPointCloud;
    sensor_msgs::msg::PointCloud2 msgPointCloudViz;
    sensor_msgs::msg::Image msgInfImg;
    
    if((rgbMsg->encoding) == std::string("rgb8")){
        cv::cvtColor(cvPtrRGB->image, cvPtrRGB->image, cv::COLOR_RGB2BGR);    
    }

    // Copying src image to create inference image
    cv::Mat infImg = cvPtrRGB->image.clone();
    
    SemanticPcl::imageCallback(cvPtrRGB, cvPtrDepth, msgPointCloud, msgPointCloudViz, infImg);

    // setting time and frame from source image
    msgPointCloud.header.stamp = depthMsg->header.stamp;
    msgPointCloud.header.frame_id = m_cameraFrameId;

    msgPointCloud.background.header.stamp = depthMsg->header.stamp;
    msgPointCloud.background.header.frame_id = m_cameraFrameId;

    for(int i = 0; i < msgPointCloud.objs_pcl.size(); i++){
        msgPointCloud.objs_pcl[i].header.stamp = depthMsg->header.stamp;
        msgPointCloud.objs_pcl[i].header.frame_id = m_cameraFrameId;
    }

    msgPointCloudViz.header.stamp = depthMsg->header.stamp;
    msgPointCloudViz.header.frame_id = m_cameraFrameId;

    // setting time and frame from source image
    std_msgs::msg::Header header;
    header.stamp = rgbMsg->header.stamp;
    header.frame_id = m_cameraFrameId;

    // converting from cv::Mat to sensor_msg/Image 
    cv_bridge::CvImage img_bridge;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, infImg);

    img_bridge.toImageMsg(msgInfImg);

    msgInfImg.encoding = "bgr8";
    msgInfImg.header.stamp = rclcpp::Node::now();

    
    m_pubInferenceImage->publish(msgInfImg);
    m_pubSemPointCloud->publish(msgPointCloud);
    m_pubSemPointCloudViz->publish(msgPointCloudViz);
}
