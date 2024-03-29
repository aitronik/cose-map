#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <unordered_map>
#include <chrono>

#include "cosemap_tools/base_node/base_node.h"
#include "cosemap_tools/sql_helper/sql_helper.h"
#include "cosemap_tools/sql_helper/sql_helper_utils.h"
#include "cosemap_msgs/msg/semantic_octomap.hpp"
#include "cosemap_msgs/msg/semantic_pcl.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "octomap_msgs/conversions.h"
#include "visualization_msgs/msg/marker_array.hpp"



/* Node that manage the database from the Semantic  */
class SemanticDatabaseNode : public BaseNode{

public:
    /**
     * @brief Construct a new Semantic Database Node object
     * 
     */
    SemanticDatabaseNode();

    /**
     * @brief Destroy the Ground Truth Node object
     * 
     */
    ~SemanticDatabaseNode();

private:

    /**
     * @brief Initialize node 
     */
    void init() override;

    /**
     * @brief Step function
     * 
     */
    void visualizeObjects();

    void onExit() override;

    sqlHelper m_conn;
    int m_numClasses;
    int m_meanTimeCounter;
    int64_t m_meanCallbackTime;
    bool m_truncatetablesOnExit;
    float m_loopRate;
    std::unordered_set<int> m_lastInsertedIds;
    std::unordered_set<int> m_insertedIds;
    float m_IoUThresh;
    
    rclcpp::Subscription<cosemap_msgs::msg::SemanticOctomap>::ConstSharedPtr m_octomapSub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_boxesPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_labelsPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_centroidsPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_objectsByOthersPclPub;

    bool uploadClasses(const std::string &pathToClassesFile);

    void octomapCallback(cosemap_msgs::msg::SemanticOctomap::ConstSharedPtr octomapMsg);

    bool addNewObject(const octomap::SemanticOcTree *tree, const bbox &objBox, const int &semID);

    void updateObject(const octomap::SemanticOcTree *tree, const bbox &objBox, const int &semID, sql::ResultSet *res);
    void updateObject(  const octomap::SemanticOcTree *tree,
                        const bbox &objBox,
                        const int &semID, 
                        const int dbObj);

    void checkScene(const octomap::SemanticOcTree *tree, sql::ResultSet *res);
    
    // return the id of the choosen object 
    int resolveConfict(const bbox &objBox, sql::ResultSet *res);

};