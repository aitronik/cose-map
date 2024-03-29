#include "cosemap_tools/base_node/base_node.h"

using namespace std::chrono_literals;
 

BaseNode::BaseNode(const std::string& node_name) : Node(node_name),m_nodeName(node_name), m_logString("Running"){
 
    m_loopRate        = 1.0;
    m_logDuration     = 5.0;
    m_printEnable     = false;

    callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BaseNode::parametersCallback, this, std::placeholders::_1));


}
/*-------------------------------------------------------------------------*/
BaseNode::BaseNode(const std::string& node_name, const rclcpp::NodeOptions & options) : 
            Node(node_name,options), m_nodeName(node_name), m_logString("Running"){
   
    m_loopRate        = 1.0;
    m_logDuration     = 5.0;
    m_printEnable     = false;
     callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BaseNode::parametersCallback, this, std::placeholders::_1));

 
}
/*-------------------------------------------------------------------------*/
BaseNode::~BaseNode(){
    rclcpp::shutdown();
}
/*-------------------------------------------------------------------------*/
void BaseNode::setLoopRate(float loopRate){
   m_loopRate = loopRate;
}
/*-------------------------------------------------------------------------*/
void BaseNode::setLogOptions(bool enable, float logDurationSeconds){
    m_printEnable = enable;
    m_logDuration = logDurationSeconds;
}
/*-------------------------------------------------------------------------*/
void BaseNode::setLogString(const std::string& log){
    m_logString = log;
}
/*-------------------------------------------------------------------------*/
bool BaseNode::waitFor(){
    return false;
}
/*-------------------------------------------------------------------------*/
bool BaseNode::run(){
    // call the init function 
    init();

    rclcpp::Rate loopRate(1); 
    while (rclcpp::ok() && waitFor()) {
        rclcpp::spin_some(shared_from_this());
        loopRate.sleep();
    }
    if (!rclcpp::ok()){
        return false;
    }
    // spin forever 
    rclcpp::spin(shared_from_this());
    onExit();
    return false;
}
/*-------------------------------------------------------------------------*/
bool BaseNode::runLoop(){
   
    init(); // initialize your publisher and subscribers. // change looprate and log duration if needed. 
    rclcpp::Node::SharedPtr nh = shared_from_this();

    rclcpp::Rate loopRateWait(1); 
    while (rclcpp::ok() && waitFor()) {
        rclcpp::spin_some(shared_from_this());
        loopRateWait.sleep();
    }
    if (!rclcpp::ok()){
        return false;
    }

    rclcpp::Rate loopRate(m_loopRate); 
    
    rclcpp::Time timeNow = this->now();
    rclcpp::Time timeLastPrintedLog = timeNow;
    while (rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        // run step function
        step();
        
        timeNow = this->now();
        // print a log every logDuration seconds 
        if( timeNow - timeLastPrintedLog >  rclcpp::Duration::from_nanoseconds(m_logDuration*1e9) && m_printEnable){
            std::cout << "[" << m_nodeName << "]:\t" << m_logString << std::endl;
            timeLastPrintedLog = timeNow;
        }

        
        // sleep till next reactivation
        loopRate.sleep();
    }
    onExit();
    return false;
}
/*-------------------------------------------------------------------------*/
void BaseNode::reconfigureVariable(void* ptrVar, std::string name, ParamEnum type, std::function<void()> onValueChanged){
    
    if (ptrVar != 0){
        ParamStruct p;
        p.type = type;
        p.ptr = ptrVar;
        p.notifyFun = onValueChanged;
        paramData[name] = p;

 
        switch(type){
            case (PARAM_STRING):
                this->declare_parameter<std::string>(name, *((std::string*)ptrVar));
                break;
            case (PARAM_INT):
                this->declare_parameter<int>(name, *((int*)ptrVar));
                break;
            case (PARAM_BOOL):
               this->declare_parameter<bool>(name, *((bool*)ptrVar));
                break;
            case (PARAM_DOUBLE):
                this->declare_parameter<double>(name, *((double*)ptrVar));
                break;
            case (PARAM_FLOAT):
                this->declare_parameter<double>(name, *((float*)ptrVar));
                break;
        }
    }
}
/*-------------------------------------------------------------------------*/
rcl_interfaces::msg::SetParametersResult BaseNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.

    for (const auto &param: parameters) {
        auto it = paramData.find(param.get_name());
        if(it != paramData.end()){
            if (it->second.ptr != 0){
                switch(it->second.type){
                    case (PARAM_STRING):
                        *((std::string*) it->second.ptr) = param.as_string();
                        std::cout << it->first <<" (string): " << param.as_string() << std::endl;
            
                        it->second.notifyFun();
                        break;
                    case (PARAM_INT):
                        *((int *) it->second.ptr) = param.as_int();
                        
                        std::cout << it->first <<" (int): " << param.as_int() << std::endl;
                        it->second.notifyFun();
                        break;
                    case (PARAM_BOOL):
                        *((bool *) it->second.ptr) = param.as_bool();
                        std::cout << it->first <<" (bool): " << (param.as_bool()? "true":"false") << std::endl;
                        it->second.notifyFun();
                        break;
                    case (PARAM_DOUBLE):
                        *((double *) it->second.ptr) = param.as_double();
                        std::cout << it->first <<" (double): " << param.as_double() << std::endl;
                        it->second.notifyFun();
                        break;
                    case (PARAM_FLOAT):
                        *((float *) it->second.ptr) = param.as_double();
                        std::cout << it->first <<" (float): " << param.as_double() << std::endl;
                        it->second.notifyFun();
                        break;
                }
            }
        }
    }
    return result;
}
/*-------------------------------------------------------------------------*/
