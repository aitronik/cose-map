#pragma once

#include <iostream>
#include <unordered_map>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

            //  float m_minSemValue = -3.5;
            // float m_maxSemValue = 3.5;
            // float m_semIncrement = 0.85;
            // float m_semDecrement = 0.4;
#define MIN_SEM_VALUE -3.5
#define MAX_SEM_VALUE 3.5
#define SEM_INCREMENT 0.85
#define SEM_DECREMENT 0.05 //0.2

namespace octomap{
    


    //forward declaration for "friend"
    class SemanticOcTree;

        //Node definition
        class SemanticOcTreeNode : public OcTreeNode {
        public:
            friend class SemanticOcTree; 

            class Color {
            public:
                Color() : r(255), g(255), b(255) {}
                Color(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {}

                inline bool operator== (const Color &other) const {
                    return (r==other.r && g==other.g && b==other.b);
                }
                
                inline bool operator!= (const Color &other) const {
                    return (r!=other.r || g!=other.g || b!=other.b);
                }
            uint8_t r, g, b;
            };

            class SemanticData{
                    public:
                        SemanticData(){
                            for(int i = 0; i < m_n; i++){
                                m_semFlags[i] = false;
                                m_semClasses[i] = 255;
                                m_semLogOdds[i] = 0.0;
                            }
                        }
                        
                        std::pair<int, float> getOutput() const{
                            // std::cout<<"getOutput 1"<<std::endl;
                            if(m_semValue < 0.0){
                                return std::pair<int, float>(255, 0.0);
                            }
                            // std::cout<<"getOutput 2: "<<(int)m_n<<std::endl;

                            float value = - std::numeric_limits<float>::max();
                            bool found = false;
                            int id = 255;


                            for(int i = 0; i < m_n; i++){
                                // std::cout<<i<<std::endl;
                                // std::cout<<"getOutput 2.1, "<<m_semFlags[i]<<", "<<m_semLogOdds.size()<<std::endl;
                                if((m_semFlags[i] == true) && (m_semLogOdds[i] > value)){
                                    // std::cout<<"getOutput 2.2"<<std::endl;
                                    value = m_semLogOdds[i];
                                    // std::cout<<"getOutput 2.3"<<std::endl;
                                    id = m_semClasses[i];
                                    // std::cout<<"getOutput 2.4"<<std::endl;
                                    found = true;
                                }
                                
                            } 
                            // std::cout<<"getOutput 3"<<std::endl;

                            if(!found){
                                return std::pair<int, float>(255, 0.0);
                            }else{
                                return std::pair<int, float>(id, value);
                            } 
                        }

                        int getLeastSemIndex() const{
                            for (size_t i = 0; i < m_n; i++){
                                if(m_semFlags[i] == false){
                                    return i;
                                }
                            }
                            // all flags are true

                            int minIndex = 0;  // Inizia assumendo che il minimo sia all'indice 0
                            float minLogOdds = m_semLogOdds[0];  // Inizia con il primo valore come minimo

                            for (int i = 1; i < m_n; ++i) {
                                if (m_semLogOdds[i] < minLogOdds) {
                                    minLogOdds = m_semLogOdds[i];
                                    minIndex = i;
                                }
                            }
                            return minIndex;

                        }

                        // equal if the id is the same. 
                        bool operator==(const SemanticData& other) const{
                            if( (m_semValue < 0) && (other.m_semValue < 0)){
                                return true;
                            } else if ( (m_semValue > 0.0 && other.m_semValue < 0.0) || 
                                        (m_semValue < 0.0 && other.m_semValue > 0.0)) {
                                return false;
                            }
                            return(this->getOutput().first == other.getOutput().first);
                        }

                        // inequal if the id is not the same
                        inline bool operator!=(const SemanticData& other) const{
                            if( (m_semValue < 0) && (other.m_semValue < 0)){
                                return false;
                            } else if ( (m_semValue > 0.0 && other.m_semValue < 0.0) || 
                                        (m_semValue < 0.0 && other.m_semValue > 0.0)) {
                                return true;
                            }
                            return(this->getOutput().first != other.getOutput().first);
                        }
                            
                        static const int m_n = 3;
                        float m_semValue;
                        bool m_semFlags[3];
                        int m_semClasses[3];
                        float m_semLogOdds[3];
                        // std::vector<bool> m_semFlags;
                        // std::vector<int> m_semClasses;
                        // std::vector<float> m_semLogOdds;
        };
        public:
            // constructors:
            //NB: the right thing to do is override base octree functions that instance new nodes to pass n as parameter
            SemanticOcTreeNode() : OcTreeNode(), m_semanticData(), viz_select(true){
            } 
            // SemanticOcTreeNode() : OcTreeNode(), m_semanticData(3), viz_select(true){} 

            //copy constructor: get semantics and color from input
            SemanticOcTreeNode(const SemanticOcTreeNode& rhs) : OcTreeNode(rhs), m_semanticData(rhs.m_semanticData), color(rhs.color) {this->viz_select = rhs.viz_select;}

            // 2 nodes considered equals if they have equal semantics (not color)
            bool operator==(const SemanticOcTreeNode& rhs) const{
                return(rhs.value == value && (rhs.m_semanticData.getOutput() == m_semanticData.getOutput()));
            }

            // copy semantics and color
            void copyData(const SemanticOcTreeNode& from){
                OcTreeNode::copyData(from);
                this->m_semanticData = from.getSemantics();
                this->color = from.getColor();
                this->viz_select = from.viz_select;
            }

            //getter
            inline SemanticData getSemantics() const {
                return m_semanticData;
            }
            inline Color getColor() const{return color;}
            inline bool getVizSelect() const {return viz_select;}

            SemanticData& getSemantics() {return m_semanticData;}
            Color& getColor() {return color;}
            bool& getVizSelect() {return viz_select;}

            //setter
            // inline void setSemantics(SemanticData sem) {this->semantics=sem;}
            inline void setColor(Color col) {this->color = col;}
            inline void setVizSelect(bool sel){this->viz_select = sel; }

            // inline void setSemantics(uint8_t id, float conf) {
            //     this->semantics = SemanticData(id, conf);
            // }

            inline void setColor(uint8_t r, uint8_t g, uint8_t b){
                this->color = Color(r, g, b);
            }

            
            //has any semantics been set? (255 means "no semantics")
            bool isSemanticsSet() const{
                return(m_semanticData.m_semValue > 0.0);
            }
            
            // has any color been set? (pure white is unlikely)
            inline bool isColorSet() const{
                return(color.r != 255 || color.g != 255 || color.b != 255);
            }

            inline bool isSelected() const{
                return this->viz_select;
            }

            //update semantics based on children semantics, calls getAverageChildSemantics()
            void updateSemanticsChildren();
            //update color based on children semantics, calls getAverageChildColor()
            void updateColorChildren();

            void decrementAllLogOdds();
            void decrementAllLogOddsExcept(int index);
            void decrementSemValue();
            void incrementSemValue();
            void incrementSemLogOdds(int index);
            void forget();
            void forgetAll();
            std::pair<int, float> getSemOutput(){
                return m_semanticData.getOutput();
            }

            // get average semantics info from children Nodes. The most frequent id among children is set and confidence is the average confidence.
            // id set to 255 if no child has semantics 
            SemanticData getAverageChildSemantics() const;

            // get average RGB values from children. if no child has color, color is set to (255,255,255)
            SemanticOcTreeNode::Color getAverageChildColor() const;

            //file I/O
            std::istream& readData(std::istream &s);
            std::ostream& writeData(std::ostream &s) const;

        protected:
        SemanticData m_semanticData;
        Color color;
        bool viz_select;
    };

    // tree definition
    class SemanticOcTree : public OccupancyOcTreeBase<SemanticOcTreeNode> {

        public:
            /// Default constructor, sets resolution of leafs
            SemanticOcTree(double resolution);

            /// virtual constructor: creates a new object of same type
            /// (Covariant return type requires an up-to-date compiler)
            SemanticOcTree* create() const {return new SemanticOcTree(resolution); }

            std::string getTreeType() const {return "SemanticOcTree";}

            /** TODO description
             * Prunes a node when it is collapsible. This overloaded
             * version only considers the node occupancy for pruning,
             * different sematic data of child nodes are ignored.
             * @return true if pruning was successful
             */
            virtual bool pruneNode(SemanticOcTreeNode* node);

            // all children must exist, must not have children of their own 
            // and have the same : -occupancy value, -semantic_id
            virtual bool isNodeCollapsible(const SemanticOcTreeNode* node) const;

            // set node semantics at given key or coordinate. Replaces previous semantics.
            // SemanticOcTreeNode* setNodeSemantics(const 	OcTreeKey& key, SemanticData &semData){
            //     SemanticOcTreeNode* n = search (key);
            //     if (n != 0) {
            //         n->m_semanticData = semData;
            //     }
            //     return n;
            // }

            // SemanticOcTreeNode* setNodeSemantics(float x, float y,
            //                                     float z, SemanticData &semData) {
            //     OcTreeKey key;
            //     if(!this->coordToKeyChecked(point3d(x,y,z), key)){
            //         return NULL;
            //     }
            //     return setNodeSemantics(key, semData);
            // }


            SemanticOcTreeNode* setNoneSemantics(const 	OcTreeKey& key);

            SemanticOcTreeNode* setNoneSemantics(float x, float y,
                                                float z) {
                OcTreeKey key;
                if(!this->coordToKeyChecked(point3d(x,y,z), key)){
                    return NULL;
                }
                return setNoneSemantics(key);
            }
            
            // set node color at given key or coordinate. Replaces previous color.
            SemanticOcTreeNode* setNodeColor(const OcTreeKey& key, uint8_t r, 
                                        uint8_t g, uint8_t b);

            SemanticOcTreeNode* setNodeColor(float x, float y, 
                                        float z, uint8_t r, 
                                        uint8_t g, uint8_t b) {
                OcTreeKey key;
                if (!this->coordToKeyChecked(point3d(x,y,z), key)) {
                    return NULL;
                }
                return setNodeColor(key,r,g,b);
            }

            //integrate semantic measurement at given key or coordinate.
            SemanticOcTreeNode* updateOccucpiedNodeSemantics(const OcTreeKey& key, uint8_t id, float conf);

            SemanticOcTreeNode* updateOccucpiedNodeSemantics(float x, float y, float z, uint8_t id, float conf){
                OcTreeKey key;
                if(!this->coordToKeyChecked(point3d(x,y,z), key)){
                    return NULL;
                }

                return updateOccucpiedNodeSemantics(key, id, conf);
            }
            //integrate semantic measurement at given key or coordinate.
            SemanticOcTreeNode* updateFreeNodeSemantics(const OcTreeKey& key);

            SemanticOcTreeNode* updateFreeNodeSemantics(float x, float y, float z){
                OcTreeKey key;
                if(!this->coordToKeyChecked(point3d(x,y,z), key)){
                    return NULL;
                }
                return updateFreeNodeSemantics(key);
            }

            //integrate color measurement at given key or coordinate.
            SemanticOcTreeNode* updateNodeColor(const OcTreeKey& key, uint8_t r, 
                                                uint8_t g, uint8_t b);

            SemanticOcTreeNode* updateNodeColor(float x, float y, float z,  uint8_t r, 
                                                uint8_t g, uint8_t b){
                OcTreeKey key;
                if(!this->coordToKeyChecked(point3d(x,y,z), key)){
                    return NULL;
                }
                return updateNodeColor(key, r, g, b);
            }
            

            // update inner nodes, sets semantics to average child semantics
            void updateInnerOccupancy();


            // void copyColorAndSemanticsFromTree(const SemanticOcTree &from);

            bool isNodeSelected(const SemanticOcTreeNode* node) const; 

        protected:

            void updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth);

            /**
             * Static member object which ensures that this OcTree's prototype
             * ends up in the classIDMapping only once. You need this as a 
             * static member in any derived octree class in order to read .ot
             * files through the AbstractOcTree factory. You should also call
             * ensureLinking() once from the constructor.
             */
            class StaticMemberInitializer{
                public:
                    StaticMemberInitializer() {
                    SemanticOcTree* tree = new SemanticOcTree(0.1);
                    tree->clearKeyRays();
                    AbstractOcTree::registerTreeType(tree);
                    }

                    /**
                     * Dummy function to ensure that MSVC does not drop the
                     * StaticMemberInitializer, causing this tree failing to register.
                     * Needs to be called from the constructor of this octree.
                     */
                    void ensureLinking() {};
            };

            /// static member to ensure static initialization (only once)
            static StaticMemberInitializer semanticOcTreeMemberInit;
    };

    //! user friendly output in format (r g b)
      std::ostream& operator<<(std::ostream& out, SemanticOcTreeNode const& node);

};