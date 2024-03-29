#include "semantic_database/map_visualizer_node.h"


MapVisualizerNode::MapVisualizerNode() : BaseNode("map_visualizer_node"){
}

MapVisualizerNode::~MapVisualizerNode(){
}

void MapVisualizerNode::init(){

    // getting parameters
    std::string treefilePath;
    std::string mapTopic;
    std::string dbUrl;
    std::string dbUser;
    std::string dbPassword;
    float textSize;
    bool showId;

    this->declare_parameter<std::string>("semantic_octree_path");
    this->declare_parameter<std::string>("map_topic");
    this->declare_parameter<std::string>("map_frame_id");
    this->declare_parameter<std::string>("url");
    this->declare_parameter<std::string>("user");
    this->declare_parameter<std::string>("password");
    this->declare_parameter<float>("text_size");
    this->declare_parameter<bool>("show_id");


    this->get_parameter("semantic_octree_path", treefilePath);
    this->get_parameter("map_topic", mapTopic);
    this->get_parameter("map_frame_id", m_mapFrameId);
    this->get_parameter("url", dbUrl);
    this->get_parameter("user", dbUser);
    this->get_parameter("password", dbPassword);
    this->get_parameter("text_size", textSize);
    this->get_parameter("show_id", showId);

    
    // initializating publishers
    m_mapPub = create_publisher<octomap_msgs::msg::Octomap>(mapTopic, rclcpp::QoS{1});
    m_boxesPub = create_publisher<visualization_msgs::msg::MarkerArray>("bboxes", rclcpp::QoS{1});
    m_centroidsPub = create_publisher<visualization_msgs::msg::MarkerArray>("centroids", rclcpp::QoS{1});
    m_labelsPub = create_publisher<visualization_msgs::msg::MarkerArray>("labels", rclcpp::QoS{1});
    
    this->setLoopRate(1);

    m_sqlConn.init(dbUrl, dbUser, dbPassword);
    m_mapLoader.loadMap(treefilePath);
    std::cout<<"Map visualizer initialized !"<<std::endl;


    // init objects boxes and labels
    sql::ResultSet *query;
    visualization_msgs::msg::MarkerArray objectsBoxes;
    visualization_msgs::msg::MarkerArray objectsCentroids;
    visualization_msgs::msg::MarkerArray objectsLabels;
    m_sqlConn.getAllObjects(&query);
    objectsBoxes.markers.resize(query->rowsCount());
    objectsLabels.markers.resize(query->rowsCount());
    objectsCentroids.markers.resize(query->rowsCount());

    size_t i = 0;
    float minX, minY, minZ, maxX, maxY, maxZ;
    while(query->next()){

        minX = query->getFloat("bbox_minX");
        minY = query->getFloat("bbox_minY");
        minZ = query->getFloat("bbox_minZ");
        maxX = query->getFloat("bbox_maxX");
        maxY = query->getFloat("bbox_maxY");
        maxZ = query->getFloat("bbox_maxZ");
        
        uint8_t r, g, b;
        rainbow_rgb(query->getInt("classId")/80.0, r, g, b);

        objectsBoxes.markers[i].header.frame_id = "map";
        objectsBoxes.markers[i].header.stamp = this->now();
        objectsBoxes.markers[i].action = visualization_msgs::msg::Marker::ADD;
        objectsBoxes.markers[i].type = visualization_msgs::msg::Marker::CUBE;
        objectsBoxes.markers[i].lifetime = rclcpp::Duration(0);
        objectsBoxes.markers[i].frame_locked = true;
        objectsBoxes.markers[i].ns = "boxes";
        objectsBoxes.markers[i].id = query->getInt("id");
        objectsBoxes.markers[i].pose.position.x = minX + (maxX - minX)/2;
        objectsBoxes.markers[i].pose.position.y = minY + (maxY - minY)/2;
        objectsBoxes.markers[i].pose.position.z = minZ + (maxZ - minZ)/2;
        objectsBoxes.markers[i].scale.x = (minX - maxX);
        objectsBoxes.markers[i].scale.y = (minY - maxY); 
        objectsBoxes.markers[i].scale.z = (minZ - maxZ);
        objectsBoxes.markers[i].color.r = r/255.0;
        objectsBoxes.markers[i].color.g = g/255.0;
        objectsBoxes.markers[i].color.b = b/255.0;
        objectsBoxes.markers[i].color.a = 0.8;

        objectsCentroids.markers[i].header.frame_id = "map";
        objectsCentroids.markers[i].header.stamp = this->now();
        objectsCentroids.markers[i].action = visualization_msgs::msg::Marker::ADD;
        objectsCentroids.markers[i].type = visualization_msgs::msg::Marker::SPHERE;
        objectsCentroids.markers[i].lifetime = rclcpp::Duration(0);
        objectsCentroids.markers[i].frame_locked = true;
        objectsCentroids.markers[i].ns = "centroids";
        objectsCentroids.markers[i].id = query->getInt("id");
        objectsCentroids.markers[i].pose.position.x = query->getFloat("centroidX");
        objectsCentroids.markers[i].pose.position.y = query->getFloat("centroidY");
        objectsCentroids.markers[i].pose.position.z = query->getFloat("centroidZ");
        objectsCentroids.markers[i].scale.x = 0.1;
        objectsCentroids.markers[i].scale.y = 0.1; 
        objectsCentroids.markers[i].scale.z = 0.1;
        objectsCentroids.markers[i].color.r = 1.0;
        objectsCentroids.markers[i].color.g = 0.0;
        objectsCentroids.markers[i].color.b = 0.0;
        objectsCentroids.markers[i].color.a = 1.0;    

        objectsLabels.markers[i].header.frame_id = "map";
        objectsLabels.markers[i].header.stamp = this->now();
        objectsLabels.markers[i].action = visualization_msgs::msg::Marker::ADD;
        objectsLabels.markers[i].type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        objectsLabels.markers[i].lifetime = rclcpp::Duration(0);
        objectsLabels.markers[i].frame_locked = true;
        objectsLabels.markers[i].ns = "labels";
        objectsLabels.markers[i].id = query->getInt("id");
        objectsLabels.markers[i].pose.position.x = query->getFloat("centroidX");
        objectsLabels.markers[i].pose.position.y = query->getFloat("centroidY");

        if(query->getInt("classId") == 64){
            objectsLabels.markers[i].pose.position.z = maxZ - 0.2;
        }else{
            objectsLabels.markers[i].pose.position.z = maxZ + 0.1;
        }
        
        objectsLabels.markers[i].scale.z = textSize;
        objectsLabels.markers[i].color.r = 0.0;
        objectsLabels.markers[i].color.g = 0.0;
        objectsLabels.markers[i].color.b = 0.0;
        objectsLabels.markers[i].color.a = 1.0;
        int classId = query->getInt("classId");
        objectsLabels.markers[i].text =    m_sqlConn.getLabel(classId);

        if(showId){
            objectsLabels.markers[i].text += "[" + std::to_string(query->getInt("id")) + "]";
        }

        i++;
    }

    m_boxesPub->publish(objectsBoxes);
    m_labelsPub->publish(objectsLabels);
    m_centroidsPub->publish(objectsCentroids);
}


void MapVisualizerNode::step(){
    octomap_msgs::msg::Octomap mapMsg;
    mapMsg.header.frame_id = m_mapFrameId;
    mapMsg.header.stamp = rclcpp::Node::now();
    if (octomap_msgs::fullMapToMsg(*m_mapLoader.m_octree, mapMsg)) {
        m_mapPub->publish(mapMsg);
    } else {
        RCLCPP_ERROR(get_logger(), "Error serializing OctoMap");
    }

        
}

