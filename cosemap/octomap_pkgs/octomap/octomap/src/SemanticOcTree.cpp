#include <octomap/SemanticOcTree.h>

namespace octomap {

    //node implementation ---------------------------------------------
    std::ostream& SemanticOcTreeNode::writeData(std::ostream &s) const{
        s.write((const char*) &value, sizeof(value));	// occupancy
        s.write((const char*) &m_semanticData, sizeof(SemanticData)); 	// semantics
        s.write((const char*) &color, sizeof(Color)); 
        s.write((const char*) &viz_select, sizeof(bool));
        return s;
    }

    std::istream& SemanticOcTreeNode::readData(std::istream &s) {
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &m_semanticData, sizeof(SemanticData)); // semantics
        s.read((char*) &color, sizeof(Color)); // semantics
        s.read((char*) &viz_select, sizeof(bool)); // semantics
        return s;
    }


    SemanticOcTreeNode::SemanticData SemanticOcTreeNode::getAverageChildSemantics() const {
        
        // creating a map linking semantic id to (count, logOdds)
        std::unordered_map<uint8_t, std::pair<int, float> > childrenMap;
        int c = 0;
        float semValueAvg = 0;
        if (children != NULL){
            
            int cSemVaulue = 0;
            for (int i=0; i < 8; i++) {
                SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);
                if(child != NULL){
                    // first, get average semantic value 
                    semValueAvg += child->m_semanticData.m_semValue;
                    cSemVaulue++;
            
                    // filling the map with the found classIds and average log odds among childs
                    for(int j = 0; j < m_semanticData.m_n; j++){
                        if(child->m_semanticData.m_semFlags[j] == true){
                            int childID = child->m_semanticData.m_semClasses[j];
                            float childLogOdd = child->m_semanticData.m_semLogOdds[j];

                            int& mapSemCounter = childrenMap[childID].first;
                            float& mapSemAvg = childrenMap[childID].second;

                            mapSemCounter++;
                            mapSemAvg = ( (mapSemAvg * (mapSemCounter - 1)) + childLogOdd ) / mapSemCounter;
                        }
                    }
                    c++;
                }
            }
            
            semValueAvg /= cSemVaulue;
        }

        if(c > 0){
            // Copying the map into a vector
            std::vector<std::pair<uint8_t, std::pair<int, float>>> mapCopy(childrenMap.size());
            size_t i = 0;
            for (const auto& entry : childrenMap) {
                mapCopy.at(i) = entry;
                i++;
            }

            std::sort(mapCopy.begin(), mapCopy.end(), [](const auto& a, const auto& b) {
                return a.second.first > b.second.first;
            }); 
            // now we have the classes sorted by count

            SemanticData output;
            output.m_semValue = semValueAvg;
            for (size_t j = 0; i < mapCopy.size(); j++){
                if( j < (output.m_n) ){
                    output.m_semFlags[j] =true;
                    output.m_semClasses[j] = mapCopy[j].first;
                    output.m_semLogOdds[j] = mapCopy[j].second.second;
                }else{
                    break;
                }
            }
            return output;

        }else{
            return SemanticData();
        }

    }

    SemanticOcTreeNode::Color SemanticOcTreeNode::getAverageChildColor() const {
        int mr = 0;
        int mg = 0;
        int mb = 0;
        int c = 0;

        if (children != NULL){
            for (int i=0; i<8; i++) {
                SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);

                if (child != NULL && child->isColorSet()) {
                mr += child->getColor().r;
                mg += child->getColor().g;
                mb += child->getColor().b;
                ++c;
                }
            }
        }

        if (c > 0) {
            mr /= c;
            mg /= c;
            mb /= c;
            return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
        }
        else { // no child had a color other than white
            return Color(255, 255, 255);
        }
    }


    
    void SemanticOcTreeNode::updateSemanticsChildren() {
        m_semanticData = getAverageChildSemantics();
    }

    void SemanticOcTreeNode::updateColorChildren() {
        color = getAverageChildColor();
    }

    void SemanticOcTreeNode::decrementAllLogOdds(){

        for (size_t i = 0; i < m_semanticData.m_n; i++){
            if(m_semanticData.m_semFlags[i] == true){
                
                m_semanticData.m_semLogOdds[i] -= SEM_DECREMENT;
                
                if(m_semanticData.m_semLogOdds[i] < MIN_SEM_VALUE){
                    m_semanticData.m_semLogOdds[i] = MIN_SEM_VALUE;
                }
            }
        }

    }

    void SemanticOcTreeNode::decrementAllLogOddsExcept(int index){
       
        for (int i = 0; i < m_semanticData.m_n; i++){
            if( (m_semanticData.m_semFlags[i] == true) && (i != index)){
                
                m_semanticData.m_semLogOdds[i] -= SEM_DECREMENT;
                
                if(m_semanticData.m_semLogOdds[i] < MIN_SEM_VALUE){
                    m_semanticData.m_semLogOdds[i] = MIN_SEM_VALUE;
                }
            }
        }
    }

    void SemanticOcTreeNode::decrementSemValue(){

        m_semanticData.m_semValue -= SEM_DECREMENT;
        if(m_semanticData.m_semValue < MIN_SEM_VALUE){
            m_semanticData.m_semValue = MIN_SEM_VALUE;
        }
    }

    void SemanticOcTreeNode::incrementSemValue(){

        m_semanticData.m_semValue += SEM_INCREMENT;
        if(m_semanticData.m_semValue > MAX_SEM_VALUE){
            m_semanticData.m_semValue = MAX_SEM_VALUE;
        }

    }

    void SemanticOcTreeNode::incrementSemLogOdds(int index){

        m_semanticData.m_semLogOdds[index] += SEM_INCREMENT;
        if(m_semanticData.m_semLogOdds[index] > MAX_SEM_VALUE){
            m_semanticData.m_semLogOdds[index] = MAX_SEM_VALUE;
        }

    }

    void SemanticOcTreeNode::forget(){

        for (size_t i = 0; i < m_semanticData.m_n; i++){
            if(m_semanticData.m_semLogOdds[i] <= MIN_SEM_VALUE){
                m_semanticData.m_semFlags[i] = false;
                m_semanticData.m_semLogOdds[i] = 0.0;
                m_semanticData.m_semClasses[i] = 255;
            }
        }

    }

    void SemanticOcTreeNode::forgetAll(){

        for (size_t i = 0; i < m_semanticData.m_n; i++){
            m_semanticData.m_semFlags[i] = false;
        }

    }



    // tree implementation 	--------------------------------------------
    SemanticOcTree::SemanticOcTree(double in_resolution) : OccupancyOcTreeBase<SemanticOcTreeNode>(in_resolution){
        semanticOcTreeMemberInit.ensureLinking();
    };
    

    SemanticOcTreeNode* SemanticOcTree::setNoneSemantics(const 	OcTreeKey& key){

        SemanticOcTreeNode* n = search (key);
        if (n != 0) {
            n->m_semanticData.m_semValue = MIN_SEM_VALUE;
            for (size_t i = 0; i < 3; i++){
                n->m_semanticData.m_semFlags[i] = false;
                n->m_semanticData.m_semLogOdds[i] = 0.0;
            }
            
        }
        return n;
    }

    SemanticOcTreeNode* SemanticOcTree::setNodeColor( const OcTreeKey& key,
                                                uint8_t r,
                                                uint8_t g,
                                                uint8_t b) {
        SemanticOcTreeNode* n = search (key);
        if (n != 0) {
        n->setColor(r, g, b);
        }
        return n;
    }

    bool SemanticOcTree::pruneNode(SemanticOcTreeNode* node) {
        
        if (!isNodeCollapsible(node)){
            return false;
        }

        // set value to children's values (all assumed equal)
        node->OcTreeNode::copyData(*(getNodeChild(node, 0)));

        //set color from children
        node->setColor(node->getAverageChildColor());

        //set semantics from children
        node->m_semanticData = node->getAverageChildSemantics();

        // delete children
        for (unsigned int i=0;i<8;i++) {
            deleteNodeChild(node, i);
        }

        delete[] node->children;
        
        node->children = NULL;

        return true;

    }

    
    bool SemanticOcTree::isNodeCollapsible(const SemanticOcTreeNode* node) const{

        // all children must exist, must not have children of their own 
        // and have the same : -occupancy value, -semantic_id
        if (!nodeChildExists(node, 0)){
            return false;
        }
        const SemanticOcTreeNode* firstChild = getNodeChild(node, 0);

        if (nodeHasChildren(firstChild)){
            return false;
        }

        for (unsigned int i = 1; i < 8; i++) {
            // compare nodes using their occupancy and their semantic id
            if (!nodeChildExists(node, i) || 
                nodeHasChildren(getNodeChild(node, i)) || 
                !(getNodeChild(node, i)->getValue() == firstChild->getValue()) || 
                !(getNodeChild(node, i)->m_semanticData.getOutput() == firstChild->m_semanticData.getOutput()) ){
                    
                return false;
            }
        }

        return true;
    }
    




    
    SemanticOcTreeNode* SemanticOcTree::updateOccucpiedNodeSemantics(const OcTreeKey& key,
                                                                     uint8_t id, float conf) {
        // std::cout<<"updateOccucpiedNodeSemantics called"<<std::endl;
        SemanticOcTreeNode* n = search(key);
        // std::cout<<"updateOccucpiedNodeSemantics OK1 n = "<<n<<std::endl;

        if (n != 0) {
            // std::cout<<"updateOccucpiedNodeSemantics OK2"<<std::endl;


            // std::cout<<(*n)<<std::endl;
            if(id == 255){
                n->decrementAllLogOdds();
                n->decrementSemValue();
            }else{
                n->incrementSemValue();
                bool found = false;
                for(size_t i = 0; i < n->m_semanticData.m_n; i++){
                    if((n->m_semanticData.m_semFlags[i] == true) && (id == n->m_semanticData.m_semClasses[i])){
                        found = true;
                        n->incrementSemLogOdds(i);
                        n->decrementAllLogOddsExcept(i);
                        break;
                    }
                }

                if(!found){
                    int index = n->m_semanticData.getLeastSemIndex();
                    n->m_semanticData.m_semFlags[index] = true;
                    n->m_semanticData.m_semClasses[index] = id;
                    n->m_semanticData.m_semLogOdds[index] = 0.0;
                    n->m_semanticData.m_semLogOdds[index] += SEM_INCREMENT;
                    n->decrementAllLogOddsExcept(index);
                }
            }
            n->forget(); 
        }
        // std::cout<<"updateOccucpiedNodeSemantics returning"<<std::endl;
        return n;
    }

    SemanticOcTreeNode* SemanticOcTree::updateFreeNodeSemantics(const OcTreeKey& key){
        
        SemanticOcTreeNode* n = search(key);
        if (n != 0) {
            if(n->getLogOdds() < 0.0){
                n->forgetAll();
            }else{
                n->decrementSemValue();
            }
        }

        return n;
    }

    SemanticOcTreeNode* SemanticOcTree::updateNodeColor(const OcTreeKey& key,
                                                 uint8_t r,
                                                 uint8_t g,
                                                 uint8_t b) {
        SemanticOcTreeNode* n = search(key);
        if (n != 0) {
            if (n->isColorSet()) {
                    SemanticOcTreeNode::Color prev_color = n->getColor();
                    // media storica con alfa = 0.5 . E' possibile usare una alfa != 0.5 per pesare di più le nuove misure
                    n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
                }
                else {
                    n->setColor(r, g, b);
            }
        }
        return n;
    }

    void SemanticOcTree::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }

    void SemanticOcTree::updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth) {

        // only recurse and update for inner nodes:
        if (nodeHasChildren(node)){
            // return early for last level:
            if (depth < this->tree_depth){
                for (unsigned int i=0; i<8; i++) {
                    if (nodeChildExists(node, i)) {
                        updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
                    }
                }
            }
            // update node occupancy, semantics and color based on children
            node->updateOccupancyChildren();

            node->updateSemanticsChildren();
            node->updateColorChildren();
        }
    }


    // void SemanticOcTree::copyColorAndSemanticsFromTree(const SemanticOcTree &from){

    //     for(octomap::SemanticOcTree::tree_iterator it = from.begin_tree(), end = from.end_tree(); it != end; ++it){

    //         this->setNodeSemantics(it.getX(), it.getY(), it.getZ(), it->getSemantics());
    //         this->setNodeColor(it.getX(), it.getY(), it.getZ(), it->getColor().r, it->getColor().g, it->getColor().b);
    //     }

    // }
    
    bool SemanticOcTree::isNodeSelected(const SemanticOcTreeNode* node) const{
        return node->getVizSelect();
    }

    // TODO 
    std::ostream& operator<<(std::ostream& out, SemanticOcTreeNode const& node) {
        SemanticOcTreeNode::SemanticData semantics = node.getSemantics();
        SemanticOcTreeNode::Color color = node.getColor();

        return out << "(id: " <<semantics.getOutput().first << ", conf = " <<semantics.getOutput().second << ")"<<std::endl 
                   <<   "(r=   " << (unsigned int)color.r <<   ", g=   " << (unsigned int)color.g <<   ", b=   " << (unsigned int)color.b <<   ")  ";
    }

    SemanticOcTree::StaticMemberInitializer SemanticOcTree::semanticOcTreeMemberInit;



}