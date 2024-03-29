#pragma once
#include <vector>
#include <chrono>
#include <string.h>
#include <iomanip>
#include <iostream>
#include <unordered_map>

#include <mariadb/conncpp.hpp>

#include "sql_helper_utils.h"
#include "cosemap_tools/semantic_utils/semantic_utils.h"


class sqlHelper{

    protected:
        /* Connection to Database pointer*/
        std::unique_ptr<sql::Connection> m_conn;
    private: 
        int64_t m_closestVoxelMeanTime;
        int m_closestVoxelCounter;
        int64_t m_objBoxMeanTime;
        int m_objBoxCounter;

    public:
        sqlHelper();
        ~sqlHelper();
        
        /**
         * @brief initialize class, connecting to database
         * 
         * @param url 
         * @param user 
         * @param password 
         */
        bool init(const std::string &url, const std::string &user, const std::string &password);

        /**
         * @brief truncate all tables in database
         * 
         */
        void truncTables();

        /**
         * @brief get querie result about all semantic classes that detected in map. Columns returned are (classId, name, count).
         *        Count column is the count of voxels of that class
         * 
         * @param res double pointer to query result
         */
        void getAllClasses(sql::ResultSet **res);

        /**
         * @brief get query result about all objects detected in map. Columns are (objects.*, classes.name)
         * 
         * @param res double pointer to query result
         */
        void getAllObjects(sql::ResultSet **res);
        void getAllObjectIds(sql::ResultSet **res);

        /**
         * @brief get query result about all objects in database belonging to class className.Columns are (objects.*)
         * 
         * @param className name of the requested class
         * @param res double pointer to query result
         */
        void getClassObjects(const std::string &className, sql::ResultSet **res) const;

        /**
         * @brief get classId of the object identified by objId
         * 
         * @param objId id of the object to be queried
         * @return uint8_t 
         */
        uint8_t getObjClass(const uint8_t& objId); 

        /**
         * @brief get query result about voxels belonging to the object identified by id. Columns returned are (voxels.positionX , voxels.positionY, voxels.positionZ)
         * 
         * @param id object id
         * @param res double pointer to query result
         */
        void getObjVoxels(const int id, sql::ResultSet **res);

        /**
         * @brief get query result about voxels belonging to the class identified by className. Columns returned are (voxels.positionX , voxels.positionY, voxels.positionZ)
         * 
         * @param className name of the class
         * @param res double pointer to query result
         */
        void getClassVoxels(const std::string &className, sql::ResultSet **res);

        /**
         * @brief get query result about voxels not belonging to any object identified by id. Columns returned are (voxels.positionX , voxels.positionY, voxels.positionZ)
         * 
         * @param res double pointer to query result
         */
        void getNoObjVoxels(sql::ResultSet **res);

        /**
         * @brief get query result about voxels stored in database. Columns returned are (voxels.positionX , voxels.positionY, voxels.positionZ)
         * 
         * @param res double pointer to query result
         */
        void getAllVoxels(sql::ResultSet **res);

        /**
         * @brief delete object identified by id from database
         * 
         * @param id id of the object
         */
        void deleteObj(const int id);

        /**
         * @brief delete all voxels with no object associated
         * 
         */
        void deleteNoObjVox();
        
        /**
         * @brief count all classes stored in table "classes"
         * 
         * @param includeNoSem : include class "none"
         * @return uint8_t : classes count
         */
        uint8_t countClasses(bool includeNoSem);

        /**
         * @brief count all objects stored in table "objects"
         * 
         * @return uint8_t : classes count
         */
        uint8_t countObjects();

        /**
         * @brief get label of classes identified by id
         * 
         * @param id id of the class
         * @return std::string class name
         */
        std::string getLabel(const uint8_t id) const;

        /**
         * @brief get query result about all class labels. Columns returned are (classes.name)
         * 
         * @param res double pointer to query result
         */
        void getAllLabels(sql::ResultSet **res);

        /**
         * @brief  uploads object to database table objects
         * 
         * @param obj object to be uploaded
         */
        void uploadObject(const clusterData &obj);

        /**
         * @brief uploads voxel to database table voxels
         * 
         * @param voxel voxel to be uploaded
         */
        void uploadVoxel(const voxelData voxel);

        /**
         * @brief uploads class to database table classes
         * 
         * @param className name of the class
         * @param classId id of the class
         */
        void uploadClass(const std::string &className, const uint8_t &classId);

        void getMatchingObjects(const std::vector<std::vector<float>> &objMaxDistMatrix,
                                const std::vector<std::vector<float>> &objMinDistMatrix, 
                                const std::vector<uint8_t> &classIds, sql::ResultSet** res) const;
 
 
        void getObjPosition(const uint8_t &objId, sql::ResultSet** res) const;

        void getObjData(const uint8_t &objId, sql::ResultSet** res) const;

        /**
         * @brief executes custom query defined by command
         * 
         * @param command Query 
         * @param res double pointer to query result
         */
        void customQuery(const std::string &command, sql::ResultSet **res);

        void updateMaxSizeMap(const uint8_t semId, std::unordered_map<uint8_t, float> &maxSizeMap);

        box2D getObj2DBox(int &id);
        
        bool findClosestVoxel(const int &id, const point3D &sourcePoint, point3D &closestVoxel);

        bool findClosestVoxelInFov(const int &id, const point3D &sourcePoint, const float &angle, float& fov, point3D &closestVoxel);

        void getObjVoxelsData(const int id, sql::ResultSet **res);
        
        void reOrderObject();

        void getObjectInBox(const bbox &box, sql::ResultSet** res);

        // prende tutti gli ogetti del tipo semId le cui box intersecano con quella in ingresso
        void getObjectIntersection(const bbox &box, const int semID, sql::ResultSet** res);
        // conta tutti gli ogetti del tipo semId le cui box intersecano con quella in ingresso
        int  countObjectIntersection(const bbox &box, const int semID);

        void uploadNewObject(const clusterData &obj);

        void updateObject(const clusterData &obj, const int &id);

        int getLastInsertedId();

        void uploadNewVoxel(const voxelData &voxel);

        void updateVoxel(const voxelData &voxel);

        void deleteObjectVoxels(const int id);

        int countObjVoxels(const int objId);

};      

