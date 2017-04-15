/*
 * paramSrvMi.h
 *
 *  Created on: Sep 24, 2013
 *      Author: liu
 */

#ifndef PARAMSRVMI_H_
#define PARAMSRVMI_H_

#include <string>
#include <ros/ros.h>
#include <boost/any.hpp>

class ParamSrvMi
{
public:
    static ParamSrvMi* instanceMi();

    template<typename T>
       void set(const std::string param, T value) {
           if(config.count(param)==0){
             ROS_ERROR("ParameterServer: Ignoring invalid parameter: \"%s\"", param.c_str());
             return;
           }
           try{
             boost::any_cast<T>(value); //fails if wrong param type
           } catch (boost::bad_any_cast e) {
             ROS_ERROR("ParameterServer: Ignoring invalid parameter type: %s", e.what());
             return;
           }
           config[param] = value;
           setOnParameterServer(pre+param, value);
       }


       template<typename T>
       T get(const std::string param) {
           if(config.count(param)==0){
             ROS_FATAL("ParameterServer object queried for invalid parameter \"%s\"", param.c_str());
             assert(config.count(param)==0);
           }
           boost::any value = config[param];
           try{
             return boost::any_cast<T>(value);
           } catch( boost::bad_any_cast bac){
             ROS_ERROR_STREAM("Bad cast: Requested data type <" << typeid(T).name() << "> for parameter '" << param << "'");
             throw; //Programmer needs to fix this. Rethrow.
           }
       }


       std::string getDescription(std::string param_name);
       void getValues();

private:
    ParamSrvMi();
    ~ParamSrvMi();
	void configMi();

	static ParamSrvMi* _instanceMi;

    void addOption(std::string name, boost::any value, std::string description);
    std::map<std::string, boost::any> config;
    std::map<std::string, std::string> descriptions;

    std::string pre;
    ros::NodeHandle handle;

    void checkValues();

    template<typename T>
    T getFromParameterServer(const std::string param, T def) {
        T result;
        handle.param(param, result, def);
        return result;
    }

    template<typename T>
    void setOnParameterServer(const std::string param, T new_val) {
        handle.setParam(param, new_val);
    }

};



#endif /* PARAMSRVMI_H_ */
