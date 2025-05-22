#ifndef READ_PARAMS_H
#define READ_PARAMS_H

#include <yaml-cpp/yaml.h>

#include <glog/logging.h>

template <typename Scalar>
static bool readParam(YAML::Node config_node, Scalar &value, std::string field, bool force = true)
{
  if (config_node)
  {
    try
    {
      value = config_node.as<Scalar>();
    }
    catch (const YAML::BadConversion &e)
    {
      // Handle error
      LOG(WARNING) << "  failed to read " << field;
      LOG(FATAL) << "Error parsing " << typeid(value).name() << ": " << e.what();
      return false;
    }
    catch (YAML::TypedBadConversion<std::string> &e) // TypedBadConversion是模板类，读取什么类型的参数就传入什么类型
    {
      LOG(WARNING) << "  failed to read " << field;
      LOG(FATAL) << "Error parsing " << typeid(value).name() << ": " << e.what();
    }
    return true;
  }
  else
  {
    if (force)
    {
      LOG(FATAL) << "  failed to read " << field;
    }
    else
    {
      LOG(WARNING) << "  failed to read " << field;
    }
    return false;
  }
}

#endif
