#ifndef __BASENODE_H__
#define __BASENODE_H__

#include <iostream>
#include <map>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

static const rclcpp::QoS m_qos = rclcpp::QoS(1)
         .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
         .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
         .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    
static const rclcpp::QoS m_qosSensor = rclcpp::QoS(1)
     .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

static const rclcpp::QoS m_qosVideo = rclcpp::QoS(1)
                           .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                           .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE); // static const rclcpp::QoS qosVideo = rclcpp::QoS(1);

class BaseNode  : public rclcpp::Node {

  public:
 
    enum ParamEnum {
        PARAM_STRING,
        PARAM_INT   ,
        PARAM_BOOL  ,
        PARAM_DOUBLE,
        PARAM_FLOAT,

    };
    class ParamStruct {
      public:
        ParamEnum type;
        void* ptr;
        std::function<void()> notifyFun;
    };
   

    /**
     * @brief Construct a new Base Node object
     * 
     * @param node_name name of the ros node
     */
    explicit BaseNode(const std::string& node_name);
    /**
     * @brief Construct a new Base Node object
     * 
     * @param node_name 
     * @param options 
     */
    BaseNode(const std::string& node_name, const rclcpp::NodeOptions & options);
    /**
     * @brief Destroy the Base Node object
     * 
     */
    virtual ~BaseNode();
    /**
     * @brief configures and starts the node.
     * 
     * Differently from runLoop, run uses spin() instead of spinOnce() in a loop.
     * 
     */
    virtual bool run();
    /**
     * @brief configures and starts the node.
     * 
     * Differently from run, runLoop uses spinOnce() in a loop instead of calling spin().
     * 
     */
    virtual bool runLoop(); 
    /**
     * @brief Set the Loop Rate of the node
     * 
     * @param loopRate 
     */
    void setLoopRate(float loopRate);
    /**
     * @brief Set the Log Options of the base node
     * 
     * @param enable true if you want to print the log
     * @param logDurationSeconds how many seconds between prints
     */
    void setLogOptions(bool enable, float logDurationSeconds = 5);
    /**
     * @brief Set the Log String object
     * 
     * @param log string to output
     */
    void setLogString(const std::string& log);

    /**
     * @brief create a ros parameters with the name "name" of type "type" and updates the variable
     * passed with the pointer ptrVar. Optionally it is possible to call a function when the value
     * changes by assigning onValuedChanged function. 
     * 
     * @param ptrVar pointer of the variable we want to update with ros params
     * @param name  ros param name
     * @param type  ros param type (string, int , bool, double, float) 
     * @param onValueChanged function to call every time the value changes.
     */
    void reconfigureVariable(void* ptrVar, std::string name, ParamEnum type, std::function<void()> onValueChanged  = [](){});

    float       getLoopRate()   { return m_loopRate;};
    float       getLogDuration(){ return m_logDuration;};
    bool        getPrintEnable(){ return m_printEnable;};
    std::string getNodeName()   { return m_nodeName; };
    std::string getLogString()  { return m_logString; };


protected:
    /**
     * @brief initalizes the node.
     * 
     * you may want to redefine the looprate, log duration rate if you want to use runLoop
     * 
     */
    virtual void init() {};
    /**
     * @brief step function called inside runLoop.
     * 
     */
    virtual void step() {};
    /**
     * @brief waits for some variables to be set
     * 
     * @return true you are missing variables
     * @return false everything is fine. go on.
     */
    virtual bool waitFor();
    /**
     * @brief redefine this function to execute code on exit
     * 
     */
    virtual void onExit() {};

private:

    /** ros node name*/
    const std::string m_nodeName;
    /** Duration [s] between two logs*/
    float m_logDuration;
    /** enable or disable the print log*/
    bool m_printEnable;
    /** log string printed every m_logDuration seconds*/ 
    std::string m_logString;
    /** loop rate [Hz] at wich the while loop is running*/
    float m_loopRate;
 
    /** stores all the param data information in a map that
     * can be accessed with the string name 
    */
    std::unordered_map<std::string, ParamStruct> paramData;
    /** callback handle that is called everytime a param is changed*/
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);

};

#endif
