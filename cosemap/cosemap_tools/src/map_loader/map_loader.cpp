#include "cosemap_tools/map_loader/map_loader.h"

MapLoader::MapLoader(){
}

MapLoader::~MapLoader(){
}

bool MapLoader::loadMap(const std::string &pathToOt){

    std::string suffix = pathToOt.substr(pathToOt.length() - 3, 3);
    if (suffix == ".ot") { //full octomap file extension
        //reading the tree as an abstract octree
        std::unique_ptr<octomap::AbstractOcTree> tree{octomap::AbstractOcTree::read(pathToOt)};
        if (!tree) {
            std::cout << "[MapLoader::loadMap]: Unable to load the tree. Exiting" << std::endl;
            return false;
        }
        // casting the abstract tree into a semantic
        m_octree = std::unique_ptr<octomap::SemanticOcTree>(dynamic_cast<octomap::SemanticOcTree *>(tree.release()));        
    }
    else {
        std::cout << "[MapLoader::loadMap]: Incorrect map file extension. Exiting" << std::endl;
        return false;
    }
    std::cout << "[MapLoader::loadMap]: Tree correctly loaded. Nodes: "<<m_octree->size() << std::endl;
    return true;

}