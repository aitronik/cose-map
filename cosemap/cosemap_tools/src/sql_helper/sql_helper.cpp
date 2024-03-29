#include "cosemap_tools/sql_helper/sql_helper.h"


sqlHelper::sqlHelper(){

}

sqlHelper::~sqlHelper(){

}

bool sqlHelper::init(const std::string &url, const std::string &user, const std::string &password){
    // Connect to database
    sql::Driver* driver = sql::mariadb::get_driver_instance();
    sql::SQLString sqlUrl(url);
    sql::Properties properties({{"user", user}, {"password", password}});
    try{
        this->m_conn.reset(driver->connect(sqlUrl, properties));
    }
    catch(const sql::SQLSyntaxErrorException& e){
        std::cout<<"[sqlHelper] Unable to connect to database"<<std::endl;
        std::cout << e.what() << std::endl;
        return false;
    }
    m_closestVoxelMeanTime = 0;
    m_closestVoxelCounter = 0;
    m_objBoxMeanTime = 0;
    m_objBoxCounter = 0;

    std::cout<<"[sqlHelper] Connected to database"<<std::endl;
    return true;
}

void sqlHelper::truncTables(){
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement("SET FOREIGN_KEY_CHECKS=0;"));
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    stmnt.reset(m_conn->prepareStatement("TRUNCATE TABLE classes"));
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    stmnt.reset(m_conn->prepareStatement("TRUNCATE TABLE voxels"));
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    stmnt.reset(m_conn->prepareStatement("TRUNCATE TABLE objects"));
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    stmnt.reset(m_conn->prepareStatement("SET FOREIGN_KEY_CHECKS=1;"));
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    std::cout<<"[sqlHelper] Tables truncated"<<std::endl;
}

void sqlHelper::uploadObject(const clusterData &obj){
    
    // prepare statement to be execute in queries
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement( "INSERT INTO objects (classId, confidence, colorR, colorG, colorB, "
                                                                            "centroidX, centroidY, centroidZ, "
                                                                            "bbox_minX, bbox_minY, bbox_minZ, bbox_maxX, bbox_maxY, bbox_maxZ) "
                                                                            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)"));
    stmnt->setInt(1, obj.semId);
    stmnt->setFloat(2, obj.conf);
    stmnt->setUInt(3, obj.r);
    stmnt->setUInt(4, obj.g);
    stmnt->setUInt(5, obj.b);
    stmnt->setFloat(6, obj.centerX);
    stmnt->setFloat(7, obj.centerY);
    stmnt->setFloat(8, obj.centerZ);
    stmnt->setFloat(9, obj.minX);
    stmnt->setFloat(10, obj.minY);
    stmnt->setFloat(11, obj.minZ);
    stmnt->setFloat(12, obj.maxX);
    stmnt->setFloat(13, obj.maxY);
    stmnt->setFloat(14, obj.maxZ);
                
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }

    // prendo l'ultima primary key inserita
    stmnt.reset(m_conn->prepareStatement("SELECT LAST_INSERT_ID();"));
    sql::ResultSet* res;
    try{
        res = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
    int key;
    while(res->next()){
        key = res->getInt(1);
    }
    // assegnamento di tutti i voxels all'interno della bounding box all'oggetto corrispondente
    // NB: voxels should be already into the database. 
    stmnt.reset(m_conn->prepareStatement(   "UPDATE voxels SET objectId = ? "
                                            "WHERE  positionX >= ? AND positionX <= ? "
                                            "AND    positionY >= ? AND positionY <= ? "
                                            "AND    positionZ >= ? AND positionZ <= ? "
                                            "AND    classId = ?;"));

    stmnt->setInt(1, key);
    stmnt->setFloat(2, obj.minX);
    stmnt->setFloat(3, obj.maxX);
    stmnt->setFloat(4, obj.minY);
    stmnt->setFloat(5, obj.maxY);
    stmnt->setFloat(6, obj.minZ);
    stmnt->setFloat(7, obj.maxZ);
    stmnt->setInt(8, obj.semId);

    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

}

void sqlHelper::uploadClass(const std::string &className, const uint8_t &classId){
    // preparing the statement to be executed in the query
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement("INSERT INTO classes (classId, name) VALUES (?, ?)"));
    stmnt->setInt(1, classId);
    stmnt->setString(2, className);
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void sqlHelper::uploadVoxel(const voxelData voxel){
    std::string query = "";
    query += "INSERT INTO voxels (classId, confidence, "
                "colorR, colorG, colorB, positionX, positionY, positionZ, volume) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";

    // prepare statement to be execute in queries
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, voxel.semId);
    stmnt->setFloat(2, voxel.conf);
    stmnt->setUInt(3, voxel.r);
    stmnt->setUInt(4, voxel.g);
    stmnt->setUInt(5, voxel.b);
    stmnt->setDouble(6, voxel.x);
    stmnt->setDouble(7, voxel.y);
    stmnt->setDouble(8, voxel.z);
    stmnt->setFloat(9, voxel.volume);
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
                
}

void sqlHelper::getAllClasses(sql::ResultSet **res){

    std::string query = "";
    query =  query +    "SELECT voxels.classId, classes.name, COUNT(voxels.classId) as count " + 
                            "FROM  voxels LEFT JOIN classes " + "ON voxels.classId = classes.classId " +
                            "GROUP BY classes.classId , classes.name " +
                            "ORDER BY voxels.classId ";
    
    //executing query
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    

    try{
        *res = stmnt->executeQuery(); 
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
}

void sqlHelper::getAllObjectIds(sql::ResultSet **res){
    std::string query = "";
    query =  query +    "SELECT objects.id " + 
                            "FROM  objects";

    //executing query
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
}


void sqlHelper::getAllObjects(sql::ResultSet **res){
    std::string query = "";
    query =  query +    "SELECT objects.*, classes.name " + 
                            "FROM  objects LEFT JOIN classes ON objects.classId = classes.classId " +
                            "ORDER BY objects.id ";

    //executing query
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
}

void sqlHelper::getClassObjects(const std::string &className, sql::ResultSet **res) const{

    std::string query = "";
    query =  query +    "SELECT objects.* " + 
                            "FROM objects INNER JOIN classes " +
                            "ON objects.classId  = classes.classId " +
                            "WHERE classes.name = ? "
                            "ORDER BY id";

    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    //filling query query
    stmnt->setString(1, className);

    //executing query
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
}

uint8_t sqlHelper::getObjClass(const uint8_t& objId){
    sql::ResultSet *res = nullptr;
    std::string query = "";
    query =  query +    "SELECT objects.classId " + 
                            "FROM objects " +
                            "WHERE objects.id = ? ";


    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    //filling query query
    stmnt->setInt(1, objId);

    //executing query
    try{
        res = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 255;
    }

    //res = stmnt->executeQuery();
    res->next();

    return res->getInt(1);  

}

std::string sqlHelper::getLabel(const uint8_t id) const {
    std::string query = "";
    sql::ResultSet *res = nullptr;

    query =  query + "SELECT classes.name FROM  classes WHERE classes.classId = ?";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, id);
    
   
    try{
        res = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return "";
    }
    res->next();

    return std::string(res->getString(1));  
}

void sqlHelper::getAllLabels(sql::ResultSet **res){
    std::string query = "";
    query = query + "SELECT classes.name "
                        "FROM  classes "
                        "WHERE classId <> 255 ";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));

    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }   
}

uint8_t sqlHelper::countClasses(bool includeNoSem){

    std::string query = "SELECT COUNT(classes.classId) as count FROM classes";
    if(includeNoSem){
        query.append(" WHERE classId <> 255");
    }

    // preparing the statement to be executed in the query
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));

    sql::ResultSet* res;

    try{
        res = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 0;
    }   

   

    int nClasses;
    while(res->next()){
        nClasses = res->getInt(1);
    }

    return nClasses;
}

uint8_t sqlHelper::countObjects(){

    std::string command = "SELECT COUNT(objects.id) AS count FROM objects";

    // preparing the statement to be executed in the query
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(command));

    sql::ResultSet* res;
    res = stmnt->executeQuery();

    int nClasses = 0;
    while(res->next()){
        nClasses = res->getInt("count");
    }

    return nClasses;
}

int sqlHelper::countObjVoxels(const int objId){

    std::string command = "SELECT COUNT(*) AS voxelCount FROM voxels WHERE objectId = ?;";

    // preparing the statement to be executed in the query
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(command));
    stmnt->setInt(1, objId);

    sql::ResultSet* res;
    res = stmnt->executeQuery();

    int count = 0;
    while(res->next()){
        count = res->getInt("voxelCount");
    }

    return count;
}

void sqlHelper::getObjVoxels(const int id, sql::ResultSet **res){
    std::string query = "";
    query = query + "SELECT voxels.positionX , voxels.positionY, voxels.positionZ  "
                        "FROM voxels "
                        "WHERE  voxels.objectId = ?";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, id);

    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }   

    stmnt->close();
    m_conn->commit();
}

void sqlHelper::getObjVoxelsData(const int id, sql::ResultSet **res){
    std::string query = "";
    query = query + "SELECT voxels.*  "
                        "FROM voxels "
                        "WHERE  voxels.objectId = ?";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, id);
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
    stmnt->close();
    m_conn->commit();
}

void sqlHelper::getNoObjVoxels( sql::ResultSet **res){
    std::string query = "";
    query = query + "SELECT voxels.positionX , voxels.positionY, voxels.positionZ  "
                        "FROM voxels "
                        "WHERE  voxels.objectId IS NULL";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
}

void sqlHelper::getClassVoxels(const std::string &className, sql::ResultSet **res){
    
    std::string query = "";
    query = query + "SELECT voxels.positionX , voxels.positionY, voxels.positionZ " + 
                        "FROM voxels INNER JOIN classes " +
                        "ON voxels.classId  = classes.classId " +
                        "WHERE classes.name = ?";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setString(1, className);
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
}

void sqlHelper::deleteNoObjVox(){
    std::string query = "";
    query = query + "DELETE FROM voxels WHERE voxels.objectId IS NULL ";
    
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    std::cout <<"All voxels with no semantics removed from map and database"<<std::endl;
}

void sqlHelper::deleteObj(const int id){

    std::string query = "";
    query =  query + "DELETE FROM objects WHERE objects.id = ?";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));

    stmnt.reset(m_conn->prepareStatement(query));
    //filling query 
    stmnt->setInt(1, id);
    //executing query
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    stmnt->close();
}

void sqlHelper::getAllVoxels(sql::ResultSet** res){
    std::string query = "";
    query =  query + "SELECT voxels.positionX , voxels.positionY, voxels.positionZ " + 
                             "FROM voxels ";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
}


void sqlHelper::customQuery(const std::string &query, sql::ResultSet** res){
    
    (*res) = nullptr;
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    try{
        (*res) = stmnt->executeQuery();
    }
    catch(const sql::SQLException e){
        std::cerr << e.what() << '\n';
    }
    
}

void sqlHelper::getMatchingObjects( const std::vector<std::vector<float>> &objMaxDistMatrix,
                                    const std::vector<std::vector<float>> &objMinDistMatrix, 
                                    const std::vector<uint8_t> &classIds, sql::ResultSet** res) const{

    std::string query = "SELECT ";
    for (size_t i = 0; i < objMaxDistMatrix.size(); i++){
        query += "subquery.id_" + std::to_string(i) ;
        if(i < (objMaxDistMatrix.size()-1)){
            query += ", "; 
        }
    }
    query += " \nFROM (\n\tSELECT ";
    for (size_t i = 0; i < objMaxDistMatrix.size(); i++){
        query += "objects_" + std::to_string(i) + ".id AS id_" + std::to_string(i) + ", ";
    }
    for (size_t i = 0; i < objMaxDistMatrix.size(); i++){
        for (size_t j = 0; j < objMaxDistMatrix.at(i).size(); j++){
            if (i < j){            
                query += "\n\t\tSQRT(POW(objects_" + std::to_string(i) + ".centroidX - objects_" + std::to_string(j) + ".centroidX, 2) " + 
                         " + POW(objects_" + std::to_string(i) + ".centroidY - objects_"+ std::to_string(j) + ".centroidY, 2) " +
                         " + POW(objects_" + std::to_string(i) + ".centroidZ - objects_"+ std::to_string(j) + ".centroidZ, 2)) " +
                         "AS calculated_distance_" + std::to_string(i) + std::to_string(j) +"," ;
            }
        }        
    }
    query.pop_back();
    query += "\n\t FROM objects objects_0 ";
    for (size_t i = 1; i < objMaxDistMatrix.size(); i++){
        query += "\n\t JOIN objects objects_" + std::to_string(i) + " ON objects_" + std::to_string(0) + ".classId = " + std::to_string(classIds[0]) +
                " AND objects_" +std::to_string(i) + ".classId = " + std::to_string(classIds.at(i)) + " ";
        for (size_t j = 0; j < i; j++){
            if (j != i){
                query += " AND objects_" + std::to_string(j) + ".id != objects_" + std::to_string(i) + ".id";
            }
        }
    }

    query += "\n) subquery \n WHERE ";
    for (size_t i = 0; i < objMaxDistMatrix.size(); i++){
        for (size_t j = 0; j < objMaxDistMatrix.size(); j++){
            if (i < j){
                query +=  "\n (subquery.calculated_distance_" + std::to_string(i) + std::to_string(j) + " < " + std::to_string(abs(objMaxDistMatrix[i][j])) + " ) " +
                        " AND (subquery.calculated_distance_" + std::to_string(i) + std::to_string(j) + " > " + std::to_string(abs(objMinDistMatrix[i][j])) + " ) AND ";
            }
        }
    }
    query.erase(query.end()-4, query.end());
    
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
}

void sqlHelper::getObjPosition(const uint8_t &objId, sql::ResultSet** res) const{
    std::string query = "";
    query = query +     "SELECT objects.centroidX, objects.centroidY, objects.centroidZ "
                            "FROM objects "
                            "WHERE objects.id = ? ";

    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, objId);

   try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
}

void sqlHelper::getObjData(const uint8_t &objId, sql::ResultSet** res) const{
    std::string query = "";
    query = query +     "SELECT * "
                            "FROM objects "
                            "WHERE objects.id = ? ";

    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, objId);

    try{
        (*res) = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }    
}

void sqlHelper::updateMaxSizeMap(const uint8_t semId, std::unordered_map<uint8_t, float> &maxSizeMap){

    sql::ResultSet* res = nullptr;
    std::string className = getLabel(semId);
    // getting every object of a specific class
    getClassObjects(className, &res);
    
    bbox dbObjBbox;
    float maxWidth = maxSizeMap[semId]; // 0 if not exists in maxSizeMap
    float distance;
    std::vector<point3D> vertices;
    vertices.resize(8);

    // for every object of that class found 
    while(res->next()){
        dbObjBbox.centroid.x = res->getFloat("centroidX");
        dbObjBbox.centroid.y = res->getFloat("centroidY");
        dbObjBbox.centroid.z = res->getFloat("centroidZ");
        dbObjBbox.xMax = res->getFloat("bbox_maxX");
        dbObjBbox.yMax = res->getFloat("bbox_maxY");
        dbObjBbox.zMax = res->getFloat("bbox_maxZ");
        dbObjBbox.xMin = res->getFloat("bbox_minX");
        dbObjBbox.yMin = res->getFloat("bbox_minY");
        dbObjBbox.zMin = res->getFloat("bbox_minZ");
        
        // getting all the vertex in the box
        getBboxVertices(dbObjBbox, vertices);

        // getting max width as the distance from centroid to the furthest vertex
        for (size_t i = 0; i < vertices.size(); i++){
            distance = sqrt(pow((vertices.at(i).x - dbObjBbox.centroid.x),2) + pow((vertices.at(i).y - dbObjBbox.centroid.y),2));  // uncomment for 3d implementation : + pow((vertices.at(i).z - dbObjBbox.centroid.z),2));
            if (distance > maxWidth){
                maxWidth = distance;
            }             
        }
    }
    maxSizeMap[semId] = maxWidth;
}

box2D sqlHelper::getObj2DBox(int &id){
    std::string query = "";
    sql::ResultSet *res = nullptr;

    query =  query +    "SELECT objects.bbox_minX , objects.bbox_maxX, objects.bbox_minY , objects.bbox_maxY, objects.bbox_minZ , objects.bbox_maxZ " +
                        "FROM objects " +
                        "WHERE objects.id = ?";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, id);
    
    auto query_start_time = std::chrono::high_resolution_clock::now();

    
    try{
        res = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        box2D box;
        return box;
    }  
            
    auto query_end_time = std::chrono::high_resolution_clock::now();
    auto query_duration = std::chrono::duration_cast<std::chrono::milliseconds>(query_end_time - query_start_time);

    m_objBoxMeanTime = ((m_objBoxMeanTime*m_objBoxCounter) + query_duration.count()) / (m_objBoxCounter + 1);
    m_objBoxCounter++;

    std::cout << std::fixed << std::setprecision(10);
    std::cout << "m_objBoxMeanTime query: " << query_duration.count() << " milliseconds" << std::endl;

    res->first();

    box2D box;
    box.tl.x = res->getFloat("bbox_minX");
    box.tl.y = res->getFloat("bbox_maxY");
    box.br.x = res->getFloat("bbox_maxX");
    box.br.y = res->getFloat("bbox_minY");

    return box; 
}

bool sqlHelper::findClosestVoxel(const int &id, const point3D &sourcePoint, point3D &closestVoxel){
    
    sql::ResultSet *res = nullptr;
    std::string query = "";
    
    query = query + "SELECT voxels.positionX, voxels.positionY , voxels.positionZ "
                        "FROM voxels "
    	                "WHERE  voxels.objectId = ? "
                        "ORDER BY SQRT( POW(voxels.positionX - ?, 2) + POW(voxels.positionY - ?, 2) + POW(voxels.positionZ - ?, 2) ) " + 
                    "LIMIT 1;";
   
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, id);
    stmnt->setFloat(2, sourcePoint.x);
    stmnt->setFloat(3, sourcePoint.y);
    stmnt->setFloat(4, sourcePoint.z);
    try{
        res = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return false;
    }  
    
    if(res->first()){
        closestVoxel.x = res->getFloat("positionX");
        closestVoxel.y = res->getFloat("positionY");
        closestVoxel.z = res->getFloat("positionZ");
        
        return true;
    }
    return false;

    // point3D voxelCenter;
    // float voxelSize;
    // float closestDistance = std::numeric_limits<float>::max();
    // float distance;
    // bool found = false;
    // while(res->next()){
    //     voxelCenter.x = res->getFloat("positionX");
    //     voxelCenter.y = res->getFloat("positionY");
    //     voxelCenter.z = res->getFloat("positionZ");
        // bool isInsideFov = false;

        // if (isPointInsideFOV(voxelCenter, sourcePoint, angle, fov) ){ //voxel center is inside the camera fov
        //     isInsideFov = true;
        // }
        // else{ //voxel center is outside fov, but maybe a part of it is inside
        //     voxelSize = std::cbrt(res->getFloat("volume"));
        //     if(isVoxelInsideFOV(voxelCenter, voxelSize, sourcePoint, angle, fov)){
        //         isInsideFov = true;

        //     }
        // }

        // if(isInsideFov){
            // distance = sqrt( pow(voxelCenter.x - sourcePoint.x, 2) + pow(voxelCenter.y- sourcePoint.y,2) + pow(voxelCenter.z - sourcePoint.z,2) );
            // if(distance < closestDistance){
            //     found = true;
            //     closestDistance = distance;
            //     closestVoxel.x = voxelCenter.x;
            //     closestVoxel.y = voxelCenter.y;
            //     closestVoxel.z = voxelCenter.z;
            // }
        // }    
    // }
}
bool sqlHelper::findClosestVoxelInFov(const int &id, const point3D &sourcePoint, const float &angle, float& fov, point3D &closestVoxel){
    // auto query_start_time = std::chrono::high_resolution_clock::now();

    sql::ResultSet *res = nullptr;
    std::string query = "";
    

    // query che ritorna i voxel dal più vicino al più lontano
    query = query + "SELECT voxels.positionX, voxels.positionY , voxels.positionZ, voxels.volume "
                        "FROM voxels "
    	                "WHERE  voxels.objectId = ? "
                        // "ORDER BY SQRT( POW(voxels.positionX - ?, 2) + POW(voxels.positionY - ?, 2) + POW(voxels.positionZ - ?, 2) );";
                        "ORDER BY ( POW(voxels.positionX - ?, 2) + POW(voxels.positionY - ?, 2) + POW(voxels.positionZ - ?, 2) );";
   
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, id);
    stmnt->setFloat(2, sourcePoint.x);
    stmnt->setFloat(3, sourcePoint.y);
    stmnt->setFloat(4, sourcePoint.z);

    try{
        res = stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return false;
    }  
            
    // auto query_end_time = std::chrono::high_resolution_clock::now();
    // auto query_duration = std::chrono::duration_cast<std::chrono::milliseconds>(query_end_time - query_start_time);

    // m_closestVoxelMeanTime = ((m_closestVoxelMeanTime*m_closestVoxelCounter) + query_duration.count()) / (m_closestVoxelCounter + 1);
    // m_closestVoxelCounter++;

    // std::cout << std::fixed << std::setprecision(10);
    // std::cout << "m_closestVoxelMeanTime query: "  << query_duration.count() << " milliseconds" << std::endl;

    float voxelSize;
    while(res->next()){
        closestVoxel.x = res->getFloat("positionX");
        closestVoxel.y = res->getFloat("positionY");
        closestVoxel.z = res->getFloat("positionZ");

        if (isPointInsideFOV(closestVoxel, sourcePoint, angle, fov) ){ //voxel center is inside the camera fov
            // std::cout<<"voxel CENTER is inside fov"<<std::endl;

            // auto end_time3 = std::chrono::high_resolution_clock::now();
            // auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time3 - query_start_time);
            // std::cout << std::fixed << std::setprecision(10);
            // std::cout << "voxel_found query: "  << duration3.count() << " milliseconds" << std::endl;


            return true;
        }else{ //voxel center is outside fov, but maybe a part of it is inside

            voxelSize = std::cbrt(res->getFloat("volume"));
            if(isVoxelInsideFOV(closestVoxel, voxelSize, sourcePoint, angle, fov)){
                // std::cout<<"voxel VERTEX is inside fov"<<std::endl;

                // auto end_time1 = std::chrono::high_resolution_clock::now();
                // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - query_start_time);
                // std::cout << std::fixed << std::setprecision(10);
                // std::cout << "voxel_found query: "  << duration1.count() << " milliseconds" << std::endl;

                return true;
            }
        }
    }
    
    closestVoxel.x = 0;
    closestVoxel.y = 0;
    closestVoxel.z = 0;
    
    // auto end_time2 = std::chrono::high_resolution_clock::now();
    // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time2 - query_start_time);
    // std::cout << std::fixed << std::setprecision(10);
    // std::cout << "voxel NOT found query: "  << duration2.count() << " milliseconds" << std::endl;

    return false;
}

void sqlHelper::reOrderObject(){
    std::string query = "";
    // sql::ResultSet *res = nullptr;

    query =  query +    "SET @new_id = 0; " +
                        "UPDATE objects  SET objects.id = (@new_id := @new_id + 1) ORDER BY id;";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt.reset(m_conn->prepareStatement(query));
    
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
    stmnt->close();
    stmnt->isClosed();
}

void sqlHelper::getObjectInBox(const bbox &box, sql::ResultSet** res){
    std::string query = "";
    query = query +     "SELECT o.* " +
                            "FROM objects o "
                            "WHERE " 
                                "o.bbox_minX <= ? AND " +
                                "o.bbox_maxX >= ? AND " +
                                "o.bbox_minY <= ? AND " +
                                "o.bbox_maxY >= ? AND " +
                                "o.bbox_minZ <= ? AND " +
                                "o.bbox_maxZ >= ?;";

    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setFloat(1, box.max.x);
    stmnt->setFloat(2, box.min.x);

    stmnt->setFloat(3, box.max.y);
    stmnt->setFloat(4, box.min.y);
    
    stmnt->setFloat(5, box.max.z);
    stmnt->setFloat(6, box.min.z);

    try{
        (*res) = stmnt->executeQuery();  
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }  
}
int sqlHelper::countObjectIntersection(const bbox &box, const int semID){
    std::string query = "";
    query = query +     "SELECT COUNT(*) AS count_results " +
                            "FROM objects o "
                            "WHERE " 
                                "o.bbox_minX <= ? AND " +
                                "o.bbox_maxX >= ? AND " +
                                "o.bbox_minY <= ? AND " +
                                "o.bbox_maxY >= ? AND " +
                                "o.bbox_minZ <= ? AND " +
                                "o.bbox_maxZ >= ? AND " +
                                "o.classId = ?;";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setFloat(1, box.max.x);
    stmnt->setFloat(2, box.min.x);

    stmnt->setFloat(3, box.max.y);
    stmnt->setFloat(4, box.min.y);
    
    stmnt->setFloat(5, box.max.z);
    stmnt->setFloat(6, box.min.z);

    stmnt->setInt(7, semID);

    sql::ResultSet *res;
    
    try{
        res = stmnt->executeQuery();  
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 0;
    }  

    res->first();
    int count = res->getInt("count_results");

    return count;

}
void sqlHelper::getObjectIntersection(const bbox &box, const int semID, sql::ResultSet** res){
    std::string query = "";
    query = query +     "SELECT o.* " +
                            "FROM objects o "
                            "WHERE " 
                                "o.bbox_minX <= ? AND " +
                                "o.bbox_maxX >= ? AND " +
                                "o.bbox_minY <= ? AND " +
                                "o.bbox_maxY >= ? AND " +
                                "o.bbox_minZ <= ? AND " +
                                "o.bbox_maxZ >= ? AND " +
                                "o.classId = ?;";

    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    

    stmnt->setFloat(1, box.max.x);
    stmnt->setFloat(2, box.min.x);

    stmnt->setFloat(3, box.max.y);
    stmnt->setFloat(4, box.min.y);
    
    stmnt->setFloat(5, box.max.z);
    stmnt->setFloat(6, box.min.z);

    stmnt->setInt(7, semID);

    try{
        (*res) = stmnt->executeQuery();  
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    } 
}

void sqlHelper::uploadNewObject(const clusterData &obj){
    // prepare statement to be execute in queries
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement( "INSERT INTO objects (classId, confidence, colorR, colorG, colorB, "
                                                                            "centroidX, centroidY, centroidZ, "
                                                                            "bbox_minX, bbox_minY, bbox_minZ, bbox_maxX, bbox_maxY, bbox_maxZ) "
                                                                            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)"));
    stmnt->setInt(1, obj.semId);
    stmnt->setFloat(2, obj.conf);
    stmnt->setUInt(3, obj.r);
    stmnt->setUInt(4, obj.g);
    stmnt->setUInt(5, obj.b);
    stmnt->setFloat(6, obj.centerX);
    stmnt->setFloat(7, obj.centerY);
    stmnt->setFloat(8, obj.centerZ);
    stmnt->setFloat(9, obj.minX);
    stmnt->setFloat(10, obj.minY);
    stmnt->setFloat(11, obj.minZ);
    stmnt->setFloat(12, obj.maxX);
    stmnt->setFloat(13, obj.maxY);
    stmnt->setFloat(14, obj.maxZ);
                
    try{
        stmnt->executeQuery();  
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    } 
}

void sqlHelper::updateObject(const clusterData &obj, const int &id){
    // prepare statement to be execute in queries
    std::string query = "";
    query = query + "UPDATE objects " +
                    " SET " +
                    "   classId = ?, " +
                    "   confidence = ?, " +
                    "   colorR = ?, " +
                    "   colorG = ?, " +
                    "   colorB = ?, " +
                    "   centroidX = ?, " +
                    "   centroidY = ?, " +
                    "   centroidZ = ?, " +
                    "   bbox_minX = ?, " +
                    "   bbox_minY = ?, " +
                    "   bbox_minZ = ?, " +
                    "   bbox_maxX = ?, " +
                    "   bbox_maxY = ?, " +
                    "   bbox_maxZ = ? " +
                    " WHERE id = ?;";

    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    
    stmnt->setInt(1,    obj.semId);
    stmnt->setFloat(2,  obj.conf);
    stmnt->setUInt(3,   obj.r);
    stmnt->setUInt(4,   obj.g);
    stmnt->setUInt(5,   obj.b);
    stmnt->setFloat(6,  obj.centerX);
    stmnt->setFloat(7,  obj.centerY);
    stmnt->setFloat(8,  obj.centerZ);
    stmnt->setFloat(9,  obj.minX);
    stmnt->setFloat(10, obj.minY);
    stmnt->setFloat(11, obj.minZ);
    stmnt->setFloat(12, obj.maxX);
    stmnt->setFloat(13, obj.maxY);
    stmnt->setFloat(14, obj.maxZ);   
    stmnt->setInt(15, id);

    try{
        stmnt->executeQuery();  
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    } 
}


int sqlHelper::getLastInsertedId(){
    // prendo l'ultima primary key inserita
    std::unique_ptr<sql::PreparedStatement> stmnt;
    stmnt.reset(m_conn->prepareStatement("SELECT LAST_INSERT_ID();"));
    sql::ResultSet* res;
    
    try{
        res = stmnt->executeQuery(); 
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    } 
    res->first();

    return res->getInt(1);    
  
}


void sqlHelper::uploadNewVoxel(const voxelData &voxel){
    std::string query = "";
    query += "INSERT INTO voxels (classId, confidence, "
                "colorR, colorG, colorB, positionX, positionY, positionZ, volume, objectId) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)";

    // prepare statement to be execute in queries
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setInt(1, voxel.semId);
    stmnt->setFloat(2, voxel.conf);
    stmnt->setUInt(3, voxel.r);
    stmnt->setUInt(4, voxel.g);
    stmnt->setUInt(5, voxel.b);
    stmnt->setDouble(6, voxel.x);
    stmnt->setDouble(7, voxel.y);
    stmnt->setDouble(8, voxel.z);
    stmnt->setFloat(9, voxel.volume);
    stmnt->setInt(10, voxel.objId);
  
    try{
        stmnt->executeQuery(); 
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    } 
                
}

void sqlHelper::updateVoxel(const voxelData &voxel){

    sql::ResultSet *res;
    std::string query = "";
    query = query +     " SELECT * " +
                        " FROM voxels " +
                        " WHERE positionX = ?" +
                        " AND positionY = ?" +
                        " AND positionZ = ?;";

    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));
    stmnt->setFloat(1,voxel.x );
    stmnt->setFloat(2,voxel.y );
    stmnt->setFloat(3,voxel.z );

    try{
        res = stmnt->executeQuery(); 
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    } 

    int counter = 0;
    while (res->next()){   
        counter ++;
    }

    if( counter > 0 ){
        
        std::cout<< counter<<" voxel found"<<std::endl;
    }
    
}

void sqlHelper::deleteObjectVoxels(const int objId){

    std::string query = "";
    query =  query + "DELETE FROM voxels WHERE voxels.objectId = ?";
    std::unique_ptr<sql::PreparedStatement> stmnt(m_conn->prepareStatement(query));

    stmnt.reset(m_conn->prepareStatement(query));
    //filling query 
    stmnt->setInt(1, objId);
    //executing query
    try{
        stmnt->executeQuery();
    }
    catch (std::exception &e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    stmnt->close();
}
