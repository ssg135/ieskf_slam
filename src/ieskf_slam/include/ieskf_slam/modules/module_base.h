#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include "ieskf_slam/common/logging.h"
namespace IESKFSLAM
{
    class ModuleBase
    {
    private:
        YAML::Node config_node;
        std::string name;

    protected:
        /**
         * @param config_path: 配置文件目录
         * @param prefix: 前缀
         * @param module_name: 模块名称 
        */
        ModuleBase(const std::string&config_path,const std::string&prefix, const std::string & module_name = "default"){
            name = module_name;
            if(config_path!=""){
                try{
                    config_node = YAML::LoadFile(config_path);
                    SLAM_LOG_INFO << "[" << name << "] loaded config file: " << config_path;
                }catch (YAML::Exception &e){
                    SLAM_LOG_ERROR << "Failed to load config file " << config_path << ": " << e.msg;
                }
                
                if(prefix!=""&&config_node[prefix]){
                    config_node = config_node[prefix];
                    SLAM_LOG_INFO << "[" << name << "] using config prefix: " << prefix;
                } else if (prefix != "") {
                    SLAM_LOG_WARN << "[" << name << "] missing config prefix: " << prefix;
                }
            } else {
                SLAM_LOG_WARN << "[" << name << "] empty config path, defaults may be used";
            }
        }
        /**
         * @param T
         * @param key: 键值
         * @param val: 读取数据到哪个参数
         * @param default_val: 默认值
        */
        template<typename T>
        void readParam(const std::string &key,T&val,T default_val){
            if(config_node[key]){
                val = config_node[key].as<T>();
                SLAM_LOG_INFO << "[" << name << "] param '" << key << "' loaded from config";
            }else{
                val = default_val;
                SLAM_LOG_WARN << "[" << name << "] param '" << key << "' missing, using default";
            }
            //std::cout<<name: <<default_val<<std::endl;
        }
    };
}
