#include <iostream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cosemap_msgs/msg/semantic_pcl.hpp>

#include <message_filters/subscriber.h>
#include <tf2/utils.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "cosemap_tools/base_node/base_node.h"

#include "semantic_pcl/semantic_pcl.h"

/* Node that manage the semantic pcl's callback events. */
class SemanticPclNode : public BaseNode, public SemanticPcl {

public:
    /**
     * @brief Construct a new Semantic Pcl Node object
     * 
     */
    SemanticPclNode();

    /**
     * @brief Destroy the Semantic Pcl Node object
     * 
     */
    ~SemanticPclNode();

    /* Subscribers */
    /* Subscriber to rgb msg. */
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>     m_subRGB;
    /* Subscriber to depth msg.*/
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>     m_subDepth;

    /* Publishers*/
    /* Publisher of semantic point cloud message*/
    
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr             m_pubSemPointCloud;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr             m_pubSemPointCloudViz;
    rclcpp::Publisher<cosemap_msgs::msg::SemanticPcl>::SharedPtr           m_pubSemPointCloud;

    /* Publisher of RGB image with semantic inference*/
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                   m_pubInferenceImage;

private:
    /**
     * @brief Initialize node and define of synced callback for rgb and depth messages
     * 
     */
    void init() override;

    /**
     * @brief Step function
     * 
     */
    void step() override;


    /* camera frame id*/
    std::string m_cameraFrameId;
    bool m_isRgb;

    /* Defining sync policy for syncronizer*/ 
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    /* Syncronizer */
    message_filters::Synchronizer<sync_pol> *m_sync;

    /**
     * @brief Callback to make inference on rgb image and publish semantic point cloud
     * 
     * @param[in] rgbMsg 
     *         message containing RGB image
     * @param[in] deptMsg
     *         message containig Depth image 
     */        
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgbMsg, const sensor_msgs::msg::Image::ConstSharedPtr& depthMsg);
    
};