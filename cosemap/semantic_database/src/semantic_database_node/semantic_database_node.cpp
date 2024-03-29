#include "semantic_database/semantic_database_node.h"


SemanticDatabaseNode::SemanticDatabaseNode() : BaseNode("semantic_database_node"){
}

SemanticDatabaseNode::~SemanticDatabaseNode(){
}

void SemanticDatabaseNode::init(){
    std::string url = "jdbc:mariadb://localhost:3306/semanticDatabase";
    std::string user = "aitronik";
    std::string password = "aitronik";
    std::string labelPath = "/home/semantic/ws/src/semantic/semantic_pkgs/semantic_pcl/labels/labels.txt";
    bool needTouploadClasses = false;

    this->declare_parameter<std::string>("url", url);
    this->declare_parameter<std::string>("user", user);
    this->declare_parameter<std::string>("password", password);
    this->declare_parameter<std::string>("labels_file_path", labelPath);
    this->declare_parameter<bool>("truncate_tables_on_exit", m_truncatetablesOnExit);
    this->declare_parameter<float>("loop_rate", m_loopRate);
    this->declare_parameter<bool>("upload_classes", needTouploadClasses);
    this->declare_parameter<float>("iou_thresh", m_IoUThresh);



    this->get_parameter("url", url);
    this->get_parameter("user", user);
    this->get_parameter("password", password);
    this->get_parameter("labels_file_path", labelPath);
    this->get_parameter("truncate_tables_on_exit", m_truncatetablesOnExit);
    this->get_parameter("loop_rate", m_loopRate);
    this->get_parameter("upload_classes", needTouploadClasses);
    this->get_parameter("iou_thresh", m_IoUThresh);

    
    m_conn.init(url, user, password);
    // m_conn.truncTables();
    if(needTouploadClasses){
        uploadClasses(labelPath);
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default; //rmw_qos_profile_sensor_data;
    rmw_qos_profile_t qos_profile_sensor = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS( rclcpp::QoSInitialization( qos_profile.history, qos_profile.depth), qos_profile);
    auto qos_sensor = rclcpp::QoS( rclcpp::QoSInitialization( qos_profile_sensor.history, qos_profile_sensor.depth), qos_profile_sensor);

    m_octomapSub = this->create_subscription<cosemap_msgs::msg::SemanticOctomap>("semantic_octomap", qos, std::bind(&SemanticDatabaseNode::octomapCallback, this, std::placeholders::_1));
    m_boxesPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("objects_boxes", qos);
    m_centroidsPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("objects_centroids", qos);
    m_labelsPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("objects_labels", qos);
    m_objectsByOthersPclPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_no_raycast", qos_sensor);

    m_meanCallbackTime = 0.0;
    m_meanTimeCounter = 0;

    this->setLoopRate(m_loopRate);

    std::cout<<"[SemanticDatabaseNode]: INITIALIZED! Looprate:  " << m_loopRate <<std::endl;
}


void SemanticDatabaseNode::visualizeObjects(){
    sql::ResultSet *query;

    // filling current ids set
    std::unordered_set<int> currentIds;
    m_conn.getAllObjectIds(&query);
    while(query->next()){
        int id = query->getInt("id");
        currentIds.insert(id);        
    }
    
    visualization_msgs::msg::MarkerArray boxesDeleteMsg;
    visualization_msgs::msg::MarkerArray labelsDeleteMsg;
    visualization_msgs::msg::MarkerArray centroidsDeleteMsg;
    boxesDeleteMsg.markers.reserve(m_lastInsertedIds.size());
    labelsDeleteMsg.markers.reserve(m_lastInsertedIds.size());
    centroidsDeleteMsg.markers.reserve(m_lastInsertedIds.size());
    // deleting prev ids that are no more in current ids set
    for (auto it = m_lastInsertedIds.begin(); it != m_lastInsertedIds.end(); ++it) {
        if (currentIds.find(*it) == currentIds.end()){
            std::cout<<"deleting : "<<*it<<std::endl;
            visualization_msgs::msg::Marker marker;
            marker.id = *it;
            marker.action = visualization_msgs::msg::Marker::DELETE;
            marker.header.stamp = this->now();
            marker.color.a = 0.0;

            marker.ns = "boxes";
            boxesDeleteMsg.markers.push_back(marker);        

            marker.ns = "labels";
            labelsDeleteMsg.markers.push_back(marker);        

            marker.ns = "centroids";
            centroidsDeleteMsg.markers.push_back(marker);        
        }
    }
    m_boxesPub->publish(boxesDeleteMsg);
    m_labelsPub->publish(labelsDeleteMsg);
    m_centroidsPub->publish(centroidsDeleteMsg);

    m_lastInsertedIds = currentIds;


    m_conn.getAllObjects(&query);
    visualization_msgs::msg::MarkerArray vizBoxesMsg;
    visualization_msgs::msg::MarkerArray vizLabelsMsg;
    visualization_msgs::msg::MarkerArray vizCentroidsMsg;

    std::unordered_map<int, int> classCountMap;

    while(query->next()){
        visualization_msgs::msg::Marker objBoxViz;
        visualization_msgs::msg::Marker objCentroidViz;
        visualization_msgs::msg::Marker objLabelViz;

        point3D max;
        point3D min;
        point3D centroid;
        int id = query->getInt("id");
        int classId = query->getInt("classId");
        max.x = query->getFloat("bbox_maxX");        
        max.y = query->getFloat("bbox_maxY");        
        max.z = query->getFloat("bbox_maxZ");     
        min.x = query->getFloat("bbox_minX");        
        min.y = query->getFloat("bbox_minY");        
        min.z = query->getFloat("bbox_minZ");
        centroid.x = query->getFloat("centroidX");    
        centroid.y = query->getFloat("centroidY");  
        centroid.z = query->getFloat("centroidZ");

        classCountMap[classId]++;

        std::string label = m_conn.getLabel(classId);  
        uint8_t r, g, b;
        rainbow_rgb(classId/80.0, r, g, b);

        objBoxViz.header.frame_id = "map";
        objBoxViz.header.stamp = this->now();
        objBoxViz.action = visualization_msgs::msg::Marker::ADD;
        objBoxViz.type = visualization_msgs::msg::Marker::CUBE;
        objBoxViz.lifetime = rclcpp::Duration(0);
        objBoxViz.frame_locked = true;
        objBoxViz.ns = "boxes";
        objBoxViz.id = id;
        objBoxViz.pose.position.x = min.x + (max.x - min.x)/2;
        objBoxViz.pose.position.y = min.y + (max.y - min.y)/2;
        objBoxViz.pose.position.z = min.z + (max.z - min.z)/2;
        objBoxViz.scale.x = (min.x - max.x);
        objBoxViz.scale.y = (min.y - max.y); 
        objBoxViz.scale.z = (min.z - max.z);
        objBoxViz.color.r = r/255.0;
        objBoxViz.color.g = g/255.0;
        objBoxViz.color.b = b/255.0;
        if (classId == 58){
            objBoxViz.color.r = 1.0;
            objBoxViz.color.g = 0.0;
            objBoxViz.color.b = 0.0;
        }
        
        objBoxViz.color.a = 0.6;  
        vizBoxesMsg.markers.push_back(objBoxViz);

        objCentroidViz.header.frame_id = "map";
        objCentroidViz.header.stamp = this->now();
        objCentroidViz.action = visualization_msgs::msg::Marker::ADD;
        objCentroidViz.type = visualization_msgs::msg::Marker::SPHERE;
        objCentroidViz.lifetime = rclcpp::Duration(0);
        objCentroidViz.frame_locked = true;
        objCentroidViz.ns = "centroids";
        objCentroidViz.id = id;
        objCentroidViz.pose.position.x = centroid.x;
        objCentroidViz.pose.position.y = centroid.y;
        objCentroidViz.pose.position.z = centroid.z;
        objCentroidViz.scale.x = 0.35;
        objCentroidViz.scale.y = 0.35; 
        objCentroidViz.scale.z = 0.35;
        objCentroidViz.color.r = 1.0;
        objCentroidViz.color.g = 0.0;
        objCentroidViz.color.b = 0.0;
        objCentroidViz.color.a = 1.0;  
        vizCentroidsMsg.markers.push_back(objCentroidViz);


        objLabelViz.header.frame_id = "map";
        objLabelViz.header.stamp = this->now();
        objLabelViz.frame_locked = true;
        objLabelViz.ns = "labels";
        objLabelViz.id = id;
        objLabelViz.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        objLabelViz.action = visualization_msgs::msg::Marker::ADD;
        objLabelViz.pose.position.x = min.x + (max.x - min.x)/2 + 0.125;
        objLabelViz.pose.position.y = min.y + (max.y - min.y)/2;
        objLabelViz.pose.position.z = max.z + 0.1;
        objLabelViz.scale.x = 0.2;
        objLabelViz.scale.y = 0.2; 
        objLabelViz.scale.z = 0.3;
        objLabelViz.color.r = 0.0;//r/255.0;
        objLabelViz.color.g = 0.0;//g/255.0;
        objLabelViz.color.b = 0.0;//b/255.0;
        objLabelViz.color.a = 1;
        objLabelViz.text = label + "[" + std::to_string(id) + "]";

        vizLabelsMsg.markers.push_back(objLabelViz);
    }

    m_boxesPub->publish(vizBoxesMsg);
    m_centroidsPub->publish(vizCentroidsMsg);
    m_labelsPub->publish(vizLabelsMsg);


    // Print unordered map
    // if(!classCountMap.empty()){
    //     for (const auto& pair : classCountMap) {
    //         std::cout << "Class: " << m_conn.getLabel(pair.first) << ", Count: " << pair.second << std::endl;
    //     }
    //     std::cout<<std::endl;
    // }
}


void SemanticDatabaseNode::onExit(){
    
    if (m_truncatetablesOnExit){
        m_conn.truncTables();
    }
}

bool SemanticDatabaseNode::uploadClasses(const std::string &pathToClassesFile){
    
    //opening file
    std::ifstream ifs;
    std::string line;
    ifs.open(pathToClassesFile.c_str());
    if(!ifs){
        return false;
    }

    //reading each line
    m_numClasses = 0;
    while (std::getline(ifs,line) ){
        std::istringstream iss(line);
        std::string token;

        //reading tokens in line and assign values separated by ',' to token vector
        while (std::getline(iss, token, ',')){  
            m_conn.uploadClass(token, m_numClasses);
            m_numClasses++;
        }
    }

    // upload no Semantic class
    m_conn.uploadClass("none", 255);
    return true;
}

void SemanticDatabaseNode::octomapCallback(cosemap_msgs::msg::SemanticOctomap::ConstSharedPtr octomapMsg){

    auto start_time = std::chrono::high_resolution_clock::now();

    point3D maxScene;
    maxScene.x = octomapMsg->scene_max.x;
    maxScene.y = octomapMsg->scene_max.y;
    maxScene.z = octomapMsg->scene_max.z;
    point3D minScene;
    minScene.x = octomapMsg->scene_min.x;
    minScene.y = octomapMsg->scene_min.y;
    minScene.z = octomapMsg->scene_min.z;

    bbox scene;
    scene.max = maxScene;
    scene.min = minScene;

    octomap::SemanticOcTree * tree = NULL;
    octomap::AbstractOcTree * octomap = octomap_msgs::msgToMap(octomapMsg->octomap);
    if (octomap) {
        tree = dynamic_cast<octomap::SemanticOcTree *>(octomap);
    }

    sql::ResultSet *res;
    m_conn.getObjectInBox(scene, &res);

    // controllare se gli oggetti esistono ancora nell'octree. Se non esistono -> cancellarli dal database
    checkScene(tree, res);

    //map containing <id, count> to keep track of same semantic objects in the same scene
    std::unordered_map<int, std::vector<int>> seenObjCountMap;
    // filling map
    for(int i = 0; i < octomapMsg->object_boxes.size(); i++){
        int semId = octomapMsg->object_boxes[i].semantic_id;     
        if(seenObjCountMap.find(semId) != seenObjCountMap.end() ){
            seenObjCountMap[semId].push_back(i);
        }else{
            std::vector<int> tmp(1,i);
            seenObjCountMap.insert(std::pair<int, std::vector<int>>(semId, tmp));   
        }
    }

    // scorro la mappa sapendo gia se ci sono classi con più di una istanza
    for (const auto& pair : seenObjCountMap){
        const int& semId = pair.first;
        const std::vector<int> &indexVect = pair.second;
        std::vector<std::pair<int,point3D> > centroidsVect(indexVect.size());

        // if there are more objects we get centroids
        if(indexVect.size() > 1){
            int j = 0;
            for (const auto& i : indexVect){
                centroidsVect[j].first = i;
                centroidsVect[j].second.x = octomapMsg->object_boxes.at(i).centroid.x;
                centroidsVect[j].second.y = octomapMsg->object_boxes.at(i).centroid.y;
                centroidsVect[j].second.z = octomapMsg->object_boxes.at(i).centroid.z;

                j++;
            }
        }
        

        for (const auto& i : indexVect){
            bbox objBox;
            objBox.max.x = octomapMsg->object_boxes[i].max.x;
            objBox.max.y = octomapMsg->object_boxes[i].max.y;
            objBox.max.z = octomapMsg->object_boxes[i].max.z;
            objBox.min.x = octomapMsg->object_boxes[i].min.x;
            objBox.min.y = octomapMsg->object_boxes[i].min.y;
            objBox.min.z = octomapMsg->object_boxes[i].min.z;
            objBox.centroid.x = octomapMsg->object_boxes[i].centroid.x;
            objBox.centroid.y = octomapMsg->object_boxes[i].centroid.y;
            objBox.centroid.z = octomapMsg->object_boxes[i].centroid.z;
        
            int resultCount = m_conn.countObjectIntersection(objBox, semId);

            switch (resultCount){
                case 0:{    // NO OBJECT FOUND, ADD NEW OBJECT
                    if (addNewObject(tree, objBox, octomapMsg->object_boxes[i].semantic_id)){
                        // std::cout<<"NEW OBJECT UPLOADED!"<<std::endl;
                    }
                    break;
                }
                case 1: { //ONE OBJECT FOUND
                    
                    m_conn.getObjectIntersection(objBox, octomapMsg->object_boxes[i].semantic_id ,  &res);
                    res->first();
                    if(indexVect.size() > 1){
                        //search among centroids the closest one to the given object
                        float minDistance = std::numeric_limits<float>::max();
                        int rightIndex = 0;
                        for (size_t j = 0; j < centroidsVect.size(); j++){
                            float distance = sqrt(  pow(centroidsVect[j].second.x - res->getFloat("centroidX"), 2) +
                                                    pow(centroidsVect[j].second.y - res->getFloat("centroidY"), 2) +
                                                    pow(centroidsVect[j].second.z - res->getFloat("centroidZ"), 2) );
                            if(distance < minDistance){
                                minDistance = distance;
                                rightIndex = centroidsVect[j].first;
                            }
                        }

                        // se l'ogetto corrente è effettivamente il più vicino allora aggiorna quell'ogetto
                        if(i == rightIndex){
                            updateObject(tree, objBox, octomapMsg->object_boxes[i].semantic_id, res);
                        }else{
                            // aggiungi un nuovo oggetto
                            addNewObject(tree, objBox, octomapMsg->object_boxes[i].semantic_id);
                        }
                    }else{
                        updateObject(tree, objBox, octomapMsg->object_boxes[i].semantic_id, res);
                    }
                    break;
                
                
                
                    }
                default:{ // MULTIPLE INSTANCES FOUND, RESOLVE CONFICT AND UPDATE
                    // if (indexVect.size() > resultCount){
                    //     m_conn.getObjectIntersection(objBox, octomapMsg->object_boxes[i].semantic_id ,  &res);
                    
                        

                    // }else{
                        m_conn.getObjectIntersection(objBox, octomapMsg->object_boxes[i].semantic_id ,  &res);

                        int chosen = resolveConfict(objBox, res);
                        updateObject(tree, objBox, octomapMsg->object_boxes[i].semantic_id, chosen);
                    // }

                    break;
                }

            }
        }
    }

    sql::ResultSet* query;
    m_conn.getAllObjects(&query);
    while(query->next()){
        int objId = query->getInt("id");
        int semId = query->getInt("classId");
        bbox objBox;
        objBox.min.x = query->getFloat("bbox_minX");
        objBox.min.y = query->getFloat("bbox_minY");
        objBox.min.z = query->getFloat("bbox_minZ");
        objBox.max.x = query->getFloat("bbox_maxX");
        objBox.max.y = query->getFloat("bbox_maxY");
        objBox.max.z = query->getFloat("bbox_maxZ");


        sql::ResultSet* subQuery;
        // removing object if its IoU with other of same class is over thresh
        m_conn.getClassObjects(m_conn.getLabel(semId), &subQuery);
        std::vector<bbox> boxList;
        while (subQuery->next()){
            if(subQuery->getInt("id") != objId){
                bbox box;
                box.min.x = subQuery->getFloat("bbox_minX");
                box.min.y = subQuery->getFloat("bbox_minY");
                box.min.z = subQuery->getFloat("bbox_minZ");
                box.max.x = subQuery->getFloat("bbox_maxX");
                box.max.y = subQuery->getFloat("bbox_maxY");
                box.max.z = subQuery->getFloat("bbox_maxZ");
                boxList.push_back(box);
            }
        }

        // std::cout<<"Computing IoU"<<std::endl;
        float IoUScore = box3DIoU(objBox, boxList);
        // std::cout<<"id : "<<objId<<", class : "<<m_conn.getLabel(semId)<<", IoU score = "<<IoUScore<<std::endl;
        if( IoUScore > m_IoUThresh){
            m_conn.deleteObj(objId);
        }
    }

    

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    m_meanCallbackTime = ((m_meanCallbackTime*m_meanTimeCounter) + duration.count()) / (m_meanTimeCounter + 1);
    m_meanTimeCounter++;

    // std::cout << std::fixed << std::setprecision(10);
    // std::cout << "databse_callback_mean_duration: " << m_meanCallbackTime << " milliseconds" << std::endl;
    // std::cout << std::endl;


    // deleting inserted id's not existing anymore
    std::unordered_set<int> currentIds;
    m_conn.getAllObjectIds(&query);
    while(query->next()){
        int id = query->getInt("id");
        currentIds.insert(id);        
    }
    auto it = m_insertedIds.begin();
    while (it != m_insertedIds.end()) {
        if (currentIds.find(*it) == currentIds.end()){
            it = m_insertedIds.erase(it);
        } else {
            it++;
        }
    }

    // objects uploaded by others are in currentIds but not in m_insertedIds
    std::unordered_set<int> insertedByOtherIds;
    for (const auto& objId : currentIds) {
        if (m_insertedIds.find(objId) == m_insertedIds.end()) {
            insertedByOtherIds.insert(objId);
        }
    }

    // filling pcl with voxel of objects added by others
    int num_voxels = 0;
    for (const auto& objId : insertedByOtherIds) {
        num_voxels += m_conn.countObjVoxels(objId);
    }    

    pcl::PointCloud<PointSemXYZRGB> pcl;
    pcl.reserve(num_voxels);
    for (const auto& objId : insertedByOtherIds) {
        m_conn.getObjVoxelsData(objId, &query);
        while (query->next()){
            float x = query->getFloat("positionX");
            float y = query->getFloat("positionY");
            float z = query->getFloat("positionZ");
            int r = query->getInt("colorR");
            int g = query->getInt("colorG");
            int b = query->getInt("colorB");
            int semClass = query->getInt("classId");
            float conf = query->getFloat("confidence");
            PointSemXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.r = r;
            point.g = g;
            point.b = b;
            point.id = semClass;
            point.conf = conf;

            pcl.push_back(point);
        }
    }


    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pcl, msg);
    msg.header.stamp = rclcpp::Node::now();
    msg.header.frame_id = "map";
    m_objectsByOthersPclPub->publish(msg);
    
    visualizeObjects();
}


bool SemanticDatabaseNode::addNewObject(const octomap::SemanticOcTree *tree, const bbox &objBox, const int &semID){
    clusterData dbObj;
    octomap::point3d boxMin(objBox.xMin, objBox.yMin, objBox.zMin);
    octomap::point3d boxMax(objBox.xMax, objBox.yMax, objBox.zMax);
    octomap::point3d centroid(0,0,0);
    float confidence = 0.0;
    uint8_t r = 0, g = 0, b = 0;
    int counter = 0;
    std::vector<voxelData> objectVoxels;    
    for(octomap::SemanticOcTree::leaf_bbx_iterator boxIt = tree->begin_leafs_bbx(boxMin, boxMax), end=tree->end_leafs_bbx(); boxIt!= end; ++boxIt){
        try
        {
            boxIt->getSemOutput().first;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() <<std::endl;
        }
        
        
        if( (tree->isNodeOccupied(*boxIt)) && (boxIt->getSemOutput().first == semID)){

            counter++;
            
            centroid.x() = ((centroid.x() * (counter - 1)) + boxIt.getCoordinate().x()) / counter;
            centroid.y() = ((centroid.y() * (counter - 1)) + boxIt.getCoordinate().y()) / counter;
            centroid.z() = ((centroid.z() * (counter - 1)) + boxIt.getCoordinate().z()) / counter;

            r = ((r * (counter - 1)) + boxIt->getColor().r ) / counter;
            g = ((g * (counter - 1)) + boxIt->getColor().g ) / counter;
            b = ((b * (counter - 1)) + boxIt->getColor().b ) / counter;
            
            confidence = ((confidence * (counter - 1)) + boxIt->getSemOutput().second ) / counter;

            // setting voxels without object ID
            voxelData voxel;
            voxel.semId = boxIt->getSemOutput().first;
            voxel.conf = boxIt->getSemOutput().second;
            voxel.x = boxIt.getX();
            voxel.y = boxIt.getY();
            voxel.z = boxIt.getZ();
            voxel.r = boxIt->getColor().r;
            voxel.g = boxIt->getColor().g;
            voxel.b = boxIt->getColor().b;
            voxel.volume = pow(boxIt.getSize(), 3);
            objectVoxels.push_back(voxel);
        }
    }

    if(counter == 0){
        return false;
    }else{
        dbObj.centerX = centroid.x();
        dbObj.centerY = centroid.y();
        dbObj.centerZ = centroid.z();

        dbObj.r = r;
        dbObj.g = g;
        dbObj.b = b;

        dbObj.conf = logOddToProb(confidence);

        dbObj.semId = semID;
        dbObj.maxX = objBox.xMax; //  + (tree->getResolution()/2);
        dbObj.maxY = objBox.yMax; //  + (tree->getResolution()/2);
        dbObj.maxZ = objBox.zMax; //  + (tree->getResolution()/2);
        dbObj.minX = objBox.xMin; //  - (tree->getResolution()/2);
        dbObj.minY = objBox.yMin; //  - (tree->getResolution()/2);
        dbObj.minZ = objBox.zMin; //  - (tree->getResolution()/2);

        m_conn.uploadNewObject(dbObj);

        int objectId =  m_conn.getLastInsertedId();
        if (objectId != -1){
            
            m_insertedIds.insert(objectId);

            for(size_t i = 0; i < objectVoxels.size(); i++){
                objectVoxels.at(i).objId = objectId;
                m_conn.uploadNewVoxel(objectVoxels.at(i));
            }
        }

    }


    return true;
}

void SemanticDatabaseNode::updateObject(  const octomap::SemanticOcTree *tree,
                                                const bbox &objBox,
                                                const int &semID, 
                                                sql::ResultSet *res){

    clusterData oldDbObj;
    int oldDbObjID = res->getInt("id");
    oldDbObj.minX = res->getFloat("bbox_minX");
    oldDbObj.minY = res->getFloat("bbox_minY");
    oldDbObj.minZ = res->getFloat("bbox_minZ");
    oldDbObj.maxX = res->getFloat("bbox_maxX");
    oldDbObj.maxY = res->getFloat("bbox_maxY");
    oldDbObj.maxZ = res->getFloat("bbox_maxZ");
    

    bbox newBox;
    // Calcolo i nuovi vertici minimi
    newBox.min.x = std::min(oldDbObj.minX, objBox.min.x);
    newBox.min.y = std::min(oldDbObj.minY, objBox.min.y);
    newBox.min.z = std::min(oldDbObj.minZ, objBox.min.z);

    // Calcolo i nuovi vertici massimi
    newBox.max.x = std::max(oldDbObj.maxX, objBox.max.x);
    newBox.max.y = std::max(oldDbObj.maxY, objBox.max.y);
    newBox.max.z = std::max(oldDbObj.maxZ, objBox.max.z);

    octomap::point3d boxMin(newBox.xMin, newBox.yMin, newBox.zMin);
    octomap::point3d boxMax(newBox.xMax, newBox.yMax, newBox.zMax);
    octomap::point3d centroid(0,0,0);
    float confidence = 0.0;
    uint8_t r = 0, g = 0, b = 0;
    int counter = 0;

    m_conn.deleteObjectVoxels(oldDbObjID);

    for(octomap::SemanticOcTree::leaf_bbx_iterator boxIt = tree->begin_leafs_bbx(boxMin, boxMax), end=tree->end_leafs_bbx(); boxIt!= end; ++boxIt){
        if( (tree->isNodeOccupied(*boxIt)) && (boxIt->getSemOutput().first == semID)){
            
            counter++;
            
            centroid.x() = ((centroid.x() * (counter - 1)) + boxIt.getCoordinate().x()) / counter;
            centroid.y() = ((centroid.y() * (counter - 1)) + boxIt.getCoordinate().y()) / counter;
            centroid.z() = ((centroid.z() * (counter - 1)) + boxIt.getCoordinate().z()) / counter;

            r = ((r * (counter - 1)) + boxIt->getColor().r ) / counter;
            g = ((g * (counter - 1)) + boxIt->getColor().g ) / counter;
            b = ((b * (counter - 1)) + boxIt->getColor().b ) / counter;
            
            confidence = ((confidence * (counter - 1)) + boxIt->getSemOutput().second) / counter;

            // TO DO: update voxels
            // setting voxels without object ID
            voxelData voxel;
            voxel.semId = boxIt->getSemOutput().first;
            voxel.conf = boxIt->getSemOutput().second;
            voxel.x = boxIt.getX();
            voxel.y = boxIt.getY();
            voxel.z = boxIt.getZ();
            voxel.r = boxIt->getColor().r;
            voxel.g = boxIt->getColor().g;
            voxel.b = boxIt->getColor().b;
            voxel.volume = pow(boxIt.getSize(), 3);
            voxel.objId = oldDbObjID;

            m_conn.uploadNewVoxel(voxel);
        }
    }

    clusterData newDbObj;
    newDbObj.centerX = centroid.x();
    newDbObj.centerY = centroid.y();
    newDbObj.centerZ = centroid.z();

    newDbObj.r = r;
    newDbObj.g = g;
    newDbObj.b = b;

    newDbObj.conf = logOddToProb(confidence);

    newDbObj.semId = semID;
    newDbObj.maxX = newBox.xMax; //objBox.xMax + (tree->getResolution()/2);
    newDbObj.maxY = newBox.yMax; //objBox.yMax + (tree->getResolution()/2);
    newDbObj.maxZ = newBox.zMax; //objBox.zMax + (tree->getResolution()/2);
    newDbObj.minX = newBox.xMin; //objBox.xMin - (tree->getResolution()/2);
    newDbObj.minY = newBox.yMin; //objBox.yMin - (tree->getResolution()/2);
    newDbObj.minZ = newBox.zMin; //objBox.zMin - (tree->getResolution()/2);


    m_conn.updateObject(newDbObj, oldDbObjID);

}

void SemanticDatabaseNode::updateObject(  const octomap::SemanticOcTree *tree,
                                                const bbox &objBox,
                                                const int &semID, 
                                                const int dbObjID){
    sql::ResultSet *res;
    m_conn.getObjData(dbObjID, &res);
    res->first();
    updateObject(tree, objBox, semID, res);
}

void SemanticDatabaseNode::checkScene(const octomap::SemanticOcTree *tree, sql::ResultSet *res){
    
    while (res->next()){
        clusterData dbObj;
        int dbObjID = res->getInt("id");
        dbObj.minX = res->getFloat("bbox_minX");
        dbObj.minY = res->getFloat("bbox_minY");
        dbObj.minZ = res->getFloat("bbox_minZ");
        dbObj.maxX = res->getFloat("bbox_maxX");
        dbObj.maxY = res->getFloat("bbox_maxY");
        dbObj.maxZ = res->getFloat("bbox_maxZ");

        dbObj.semId = res->getInt("classId");

        octomap::point3d boxMin(dbObj.minX, dbObj.minY, dbObj.minZ);
        octomap::point3d boxMax(dbObj.maxX, dbObj.maxY, dbObj.maxZ);

        int occupiedCounter = 0;
        int semanticCounter = 0;
        for(octomap::SemanticOcTree::leaf_bbx_iterator boxIt = tree->begin_leafs_bbx(boxMin, boxMax), end=tree->end_leafs_bbx(); boxIt!= end; ++boxIt){
            if( tree->isNodeOccupied(*boxIt)) {
                occupiedCounter++;
                if(boxIt->getSemOutput().first == dbObj.semId){
                    semanticCounter++;
                }
            }
        }

        if(semanticCounter < (0.2 *occupiedCounter)){
            // std::cout<<"Deleting object "<<dbObjID<<" of class "<<m_conn.getLabel(dbObj.semId)<<std::endl;
            // std::cout<<"occupiedCounter = "<<occupiedCounter<<", semanticCounter  "<<semanticCounter<<std::endl;
            // std::cout<<std::endl;
            m_conn.deleteObj(dbObjID);
        }

    }
}
                
int SemanticDatabaseNode::resolveConfict(const bbox &objBox, sql::ResultSet *res){
    u_int32_t chosen = std::numeric_limits<u_int32_t>::max();
    float minimumDistance = std::numeric_limits<float>::max();
    point3D dbObjCentroid;
    while(res->next()){
        dbObjCentroid.x = res->getFloat("centroidX");
        dbObjCentroid.y = res->getFloat("centroidY");
        dbObjCentroid.z = res->getFloat("centroidZ");

        int dbObjID = res->getInt("id");

        float distance = sqrt(  pow(objBox.centroid.x - dbObjCentroid.x ,2) +
                                pow(objBox.centroid.y - dbObjCentroid.y ,2) +
                                pow(objBox.centroid.z - dbObjCentroid.z ,2) );

        if(distance < minimumDistance){
            minimumDistance = distance;
            chosen = dbObjID;
        }
    }
    // std::cout<<" distance = " <<minimumDistance <<", chosen = "<<chosen<<std::endl;
    
    if( minimumDistance == std::numeric_limits<float>::max() || 
        chosen == std::numeric_limits<u_int32_t>::max()){
        return 0;
    }else{
        return chosen;
    }
}
   