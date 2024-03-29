#include <string>
#include <iostream>
#include <memory>
#include "octomap/SemanticOcTree.h"

class MapLoader{

private:
    
public:
    MapLoader(/* args */);
    ~MapLoader();

    /**
     * @brief loads semantic octree from file
     * 
     * @param pathToTree : path to the semantic octree
     * @param octree: ponter to store the loaded octree
     * @return true: tree loaded correctly
     * @return false : tree not loaded
     */
    bool loadMap(const std::string &pathToOt);

    std::unique_ptr<octomap::SemanticOcTree> m_octree;

private:
    
};

