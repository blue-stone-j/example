#ifndef PARAM_SERVER_H
#define PARAM_SERVER_H

#include <ros/node_handle.h>

#include <mutex>

#include "read_params.h"

#include <filesystem>

class ParamServer
{
 public:
  ros::NodeHandle nh;
  std::thread update_thr;
  std::mutex param_mtx;
  bool update_param = false;
  std::string config_file;
  YAML::Node config_node;

  virtual void readFlexParams() = 0;

  virtual void readFixParams() = 0;

  void updateParams()
  {
    while (ros::ok())
    {
      {
        std::lock_guard<std::mutex> param_lock(param_mtx);
        readFlexParams();
      }
      sleep(1);
    }
  }

  static std::string get_root_path()
  {
    char *p = NULL;

    constexpr int len = 256;
    /// to keep the absolute path of executable's path
    char arr_tmp[len] = {0};

    int n = readlink("/proc/self/exe", arr_tmp, len);
    if (n > len)
    {
      LOG(WARNING) << "exe path is too long!";
    }
    if (NULL != (p = strrchr(arr_tmp, '/')))
    {
      *p = '\0';
    }
    else
    {
      return std::string("");
    }

    std::string path = std::string(std::filesystem::path(std::string(arr_tmp)).parent_path().parent_path());

    return path;
  }
  std::string get_config_file()
  {
    char *p = NULL;

    constexpr int len = 256;
    /// to keep the absolute path of executable's path
    char arr_tmp[len] = {0};

    int n = readlink("/proc/self/exe", arr_tmp, len);
    if (n > len)
    {
      //! should process this warning.
      LOG(WARNING) << "exe path is too long!";
    }

    if (NULL != (p = strrchr(arr_tmp, '/')))
    {
      *p = '\0';
    }
    else
    {
      return std::string("");
    }

    std::string path = std::string(std::filesystem::path(std::string(arr_tmp)).parent_path().parent_path()) + "/share/package_name/config/params.yaml";

    return path;
  }

  ParamServer()
  {
    config_file = get_config_file();

    config_node = YAML::LoadFile(config_file);

    std::string field;
    field = "update_param";
    readParam(config_node[field], update_param, field);

    if (update_param)
    {
      update_thr = std::thread(&ParamServer::updateParams, this);
    }
  }
  ~ParamServer()
  {
    if (update_thr.joinable())
    {
      update_thr.join();
    }
    update_thr.~thread();
  }
};

#endif