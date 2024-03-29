#include "cosemap_tools/visualizer/database_visualizer.h"

DatabaseVisualizer::DatabaseVisualizer(){

}

DatabaseVisualizer::~DatabaseVisualizer(){

}

void DatabaseVisualizer::delObjViz(const int dbId){

    if( m_idMap.find(dbId) != m_idMap.end() ){
        
        int markerIndex = m_idMap[dbId];
        m_boxesMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::DELETE;
        m_centroidsMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::DELETE;
        m_labelsMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::DELETE;
    }else{
        std::cout<<"[DatabaseVisualizer] No object with id = "<<dbId<<" has been found"<<std::endl;
    }
}

void DatabaseVisualizer::addObjViz(const int dbId){
    
    if(m_idMap.find(dbId) != m_idMap.end() ){
        int markerIndex = m_idMap[dbId];
        m_boxesMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::ADD;
        m_centroidsMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::ADD;
        m_labelsMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::ADD;
    }else{
        std::cout<<"[DatabaseVisualizer] No object with id = "<<dbId<<" has been found"<<std::endl;
    }
}

void DatabaseVisualizer::addBox(const clusterData &obj,int dbId, const int numClasses){
    
    int markerIndex = m_idMap[dbId];

    m_boxesMarkerArray.markers.at(markerIndex).frame_locked = true;
    m_boxesMarkerArray.markers.at(markerIndex).ns = "boxes";
    m_boxesMarkerArray.markers.at(markerIndex).id = dbId;
    m_boxesMarkerArray.markers.at(markerIndex).type = visualization_msgs::msg::Marker::CUBE;
    m_boxesMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::ADD;
    m_boxesMarkerArray.markers.at(markerIndex).pose.position.x =obj.minX + (obj.maxX - obj.minX)/2;
    m_boxesMarkerArray.markers.at(markerIndex).pose.position.y =obj.minY + (obj.maxY - obj.minY)/2;
    m_boxesMarkerArray.markers.at(markerIndex).pose.position.z =obj.minZ + (obj.maxZ - obj.minZ)/2;
    m_boxesMarkerArray.markers.at(markerIndex).scale.x = (obj.minX - obj.maxX);
    m_boxesMarkerArray.markers.at(markerIndex).scale.y = (obj.minY - obj.maxY); 
    m_boxesMarkerArray.markers.at(markerIndex).scale.z = (obj.minZ - obj.maxZ);

    //choosing semantic color
    uint8_t r, g, b;
    rainbow_rgb(obj.semId/(double)numClasses, r, g, b);
    m_boxesMarkerArray.markers.at(markerIndex).color.r = r/255.0;
    m_boxesMarkerArray.markers.at(markerIndex).color.g = g/255.0;
    m_boxesMarkerArray.markers.at(markerIndex).color.b = b/255.0;
    m_boxesMarkerArray.markers.at(markerIndex).color.a = 0.3;
}

void DatabaseVisualizer::addCentroid(const clusterData &obj, int dbId){
    int markerIndex = m_idMap[dbId];

    m_centroidsMarkerArray.markers.at(markerIndex).frame_locked = true;
    m_centroidsMarkerArray.markers.at(markerIndex).ns = "centroids";
    m_centroidsMarkerArray.markers.at(markerIndex).id = dbId;
    m_centroidsMarkerArray.markers.at(markerIndex).type = visualization_msgs::msg::Marker::SPHERE;
    m_centroidsMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::ADD;
    m_centroidsMarkerArray.markers.at(markerIndex).pose.position.x =obj.centerX;
    m_centroidsMarkerArray.markers.at(markerIndex).pose.position.y =obj.centerY;
    m_centroidsMarkerArray.markers.at(markerIndex).pose.position.z =obj.centerZ;
    m_centroidsMarkerArray.markers.at(markerIndex).scale.x = 0.1;
    m_centroidsMarkerArray.markers.at(markerIndex).scale.y = 0.1; 
    m_centroidsMarkerArray.markers.at(markerIndex).scale.z = 0.1;
    m_centroidsMarkerArray.markers.at(markerIndex).color.r = 1.0;
    m_centroidsMarkerArray.markers.at(markerIndex).color.g = 0;
    m_centroidsMarkerArray.markers.at(markerIndex).color.b = 0;
    m_centroidsMarkerArray.markers.at(markerIndex).color.a = 1;
}

void DatabaseVisualizer::addLabel(const clusterData &obj, int dbId, const std::string &text, const int numClasses){
    int markerIndex = m_idMap[dbId];

    m_labelsMarkerArray.markers.at(markerIndex).frame_locked = true;
    m_labelsMarkerArray.markers.at(markerIndex).ns = "labels";
    m_labelsMarkerArray.markers.at(markerIndex).id = dbId;
    m_labelsMarkerArray.markers.at(markerIndex).type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m_labelsMarkerArray.markers.at(markerIndex).action = visualization_msgs::msg::Marker::ADD;
    m_labelsMarkerArray.markers.at(markerIndex).pose.position.x = obj.centerX;
    m_labelsMarkerArray.markers.at(markerIndex).pose.position.y = obj.centerY;
    m_labelsMarkerArray.markers.at(markerIndex).pose.position.z = obj.maxZ+0.1;
    m_labelsMarkerArray.markers.at(markerIndex).scale.x = 0.2;
    m_labelsMarkerArray.markers.at(markerIndex).scale.y = 0.2; 
    m_labelsMarkerArray.markers.at(markerIndex).scale.z = m_textSize;

    //choosing semantic color
    uint8_t r, g, b;
    rainbow_rgb(obj.semId/(double)numClasses, r, g, b);
    m_labelsMarkerArray.markers.at(markerIndex).color.r = r/255.0;
    m_labelsMarkerArray.markers.at(markerIndex).color.g = g/255.0;
    m_labelsMarkerArray.markers.at(markerIndex).color.b = b/255.0;
    m_labelsMarkerArray.markers.at(markerIndex).color.a = 1;
    m_labelsMarkerArray.markers.at(markerIndex).text = text;
}

void DatabaseVisualizer::resizeMarkers(const int count){
    m_boxesMarkerArray.markers.resize(count);
    m_centroidsMarkerArray.markers.resize(count);
    m_labelsMarkerArray.markers.resize(count);
}

