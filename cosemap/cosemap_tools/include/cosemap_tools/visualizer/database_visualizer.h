#include <unordered_map>

#include "cosemap_tools/semantic_utils/semantic_utils.h"
#include "cosemap_tools/sql_helper/sql_helper_utils.h"

#include "visualization_msgs/msg/marker_array.hpp"


class DatabaseVisualizer{

private:

    /* clusters bounding boxes marker array*/
    visualization_msgs::msg::MarkerArray m_boxesMarkerArray;
    /* clusters centroids marker array*/
    visualization_msgs::msg::MarkerArray m_centroidsMarkerArray;
    /* cluster labels marker array*/
    visualization_msgs::msg::MarkerArray m_labelsMarkerArray;

    /* map linking database objects ids (key) into markersArray index (value)*/
    std::unordered_map<int, int> m_idMap; 

    float m_textSize;
    
public:
    DatabaseVisualizer();


    ~DatabaseVisualizer();

    friend class SemanticDatabase;

    /**
    * @brief delete object from visualization
     * 
     * @param id of the object to be removed from visualization
     */
    void delObjViz(const int id);

    /**
     * @brief add object to visualization
     * 
     * @param id of the object to be added to visualization
     */
    void addObjViz(const int id);

    /**
     * @brief add object box to boxes marker array
     * 
     * @param obj
     * @param idx id (PRIMARY KEY) of the database object to be added
     */
    void addBox(const clusterData &obj, int idx, const int numClasses);

    /**
     * @brief add object centroid to boxes marker array
     * 
     * @param obj
     * @param idx id (PRIMARY KEY) of the database object to be added
     */
    void addCentroid(const clusterData &obj, int idx);

    /**
     * @brief add object label to boxes marker array
     * 
     * @param obj
     * @param idx id (PRIMARY KEY) of the database object to be added
     * @param text label to be added
     */
    void addLabel(const clusterData &obj, int idx, const std::string &text, const int numClasses);


    void resizeMarkers(const int count);

    void getBoxes(visualization_msgs::msg::MarkerArray &boxes){
        boxes = m_boxesMarkerArray;
    }

    void getCentroids(visualization_msgs::msg::MarkerArray &centroids){
        centroids = m_centroidsMarkerArray;
    }

    void getLabels(visualization_msgs::msg::MarkerArray &labels){
        labels = m_labelsMarkerArray;
    }

};
