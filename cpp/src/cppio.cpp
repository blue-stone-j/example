/*
input/read and output/write
1. std::cout
2. std::cin
3. getline()
4. yaml
5. json
6. xml
7. csv
8. ini
9. lua
 */

#include <fstream>
#include <iostream>
#include <iomanip>

#include <yaml-cpp/yaml.h>
#include <json/json.h>

extern "C" {
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}

int main()
{
  { // std::cin: one input ends by space, table or enter

    if (0)
    {
      int a, b;
      std::cin >> a >> b;
      std::cout << a + b << std::endl;
    }

    if (0)
    {
      char c[10];
      std::cin >> c;
      std::cout << c << std::endl;
    }

    if (0)
    {
      char m[20];
      std::cin.getline(m, 5); // 接收一个字符串，可以接收空格并输出
    }

    if (0)
    {
      std::string str;
      getline(std::cin, str);
      std::cout << str << std::endl;
    }
  }

  // yaml
  if (0)
  {
    // read
    std::string read_path  = "../assets/yaml.yaml";
    YAML::Node config_node = YAML::LoadFile(read_path);
    if (config_node["root1"])
    {
      float value;
      try
      {
        value = config_node["root1"]["value"].as<float>();
      }
      catch (const YAML::BadConversion &e)
      {
        // Handle error
        std::cout << "  failed to read ";
        std::cout << "Error parsing " << typeid(value).name() << ": " << e.what();
      }

      std::vector<float> vec;
      try
      {
        vec = config_node["root1"]["array"].as<std::vector<float>>();
      }
      catch (const YAML::BadConversion &e)
      {
        // Handle error
        std::cout << "failed to read " << std::endl;
        std::cout << "Error parsing " << typeid(vec).name() << ": " << e.what() << std::endl;
      }
    }

    // write
    std::string write_path = "../assets/yaml_w.yaml";
    std::ofstream fout(write_path);
    if (!fout.is_open())
    {
      std::cerr << "Unable to open file for writing." << std::endl;
      return 8;
    }
    fout << config_node;
    fout.close();
  }

  if (0) // json
  {
    // read
    std::string read_path = "../assets/json.json";
    std::ifstream fs;

    fs.open(read_path);
    if (!fs.is_open())
    {
      std::cerr << "can't open " << read_path;
      return 1;
    }
    if (!fs)
    {
      std::cerr << "error: " << read_path;
      return 2;
    }

    Json::Value root; // 将会包含根节点
    Json::Reader reader;

    if (!reader.parse(fs, root, false))
    {
      std::cerr << "Failed to parse the JSON file." << std::endl;
      return 3;
    }
    // 获取到"configurations"数组
    const Json::Value &configurations = root["configurations"];
    // 遍历数组
    for (uint32_t i = 0; i < configurations.size(); ++i)
    {
      const Json::Value &configuration = configurations[i];
      const Json::Value &name          = configuration["name"];
      if (name.asString() == "cpp")
      {
        const Json::Value &includePath = configuration["includePath"];
        for (uint32_t index = 0; index < includePath.size(); ++index)
        {
          std::string value = includePath[index].asString();
          std::cout << value << std::endl;
        }
      }
    }

    // write
    std::string write_path = "../assets/json_w.json";
    std::ofstream fout;
    fout.open(write_path);
    if (!fout.is_open())
    {
      std::cerr << "Unable to open file for writing." << std::endl;
      return 4;
    }

    Json::StreamWriterBuilder writer;
    std::unique_ptr<Json::StreamWriter> json_writer(writer.newStreamWriter());
    json_writer->write(root, &fout);
    fout.close();
  }

  if (0) // xml
  {
  }

  if (0) //  csv + getline()
  {
    std::ifstream fin;
    std::string read_path = "../assets/csv.csv";
    fin.open(read_path);
    if (!fin.is_open())
    {
      std::cerr << "Unable to open file for writing." << std::endl;
      return 5;
    }
    std::vector<std::vector<double>> vecs;
    if (fin)
    {
      std::string line;
      int lineInd = 0;
      while (std::getline(fin, line))
      {
        std::stringstream ss;
        ss << line;
        std::vector<double> vec(3);
        ss >> vec[0] >> vec[1] >> vec[2];
        vecs.push_back(vec);
        lineInd++;
      }
      fin.close();
    }
    else
    {
      std::cout << "error: " << read_path << std::endl;
    }

    std::string write_path = "../assets/csv_w.csv";
    std::ofstream fout;
    fout.open(write_path);
    if (!fout.is_open())
    {
      std::cerr << "Unable to open file for writing." << std::endl;
      return 6;
    }

    for (const auto &it : vecs)
    {
      fout << std::setprecision(13) << it[0] << " " << it[1] << " " << it[2] << "\n";
    }

    fout.close();
  }

  if (0) // ini-custom
  {
    std::ifstream fin;
    std::string read_path = "../assets/ini.ini";
    fin.open(read_path);

    std::map<std::string, std::map<std::string, std::string>> result;
    std::string line, section;

    while (std::getline(fin, line))
    {
      // Remove comments
      size_t commentPos = line.find_first_of(";#");
      if (commentPos != std::string::npos)
      {
        line = line.substr(0, commentPos);
      }

      // Trim whitespace
      line.erase(0, line.find_first_not_of(" \t"));
      line.erase(line.find_last_not_of(" \t") + 1);

      if (line.empty()) // Skip empty lines
      {
        continue;
      }

      if (line[0] == '[')
      { // Section marker
        section = line.substr(1, line.find(']') - 1);
      }
      else
      {
        auto delimiterPos = line.find('=');
        if (delimiterPos == std::string::npos) continue; // Skip lines without '='
        auto name             = line.substr(0, delimiterPos);
        auto value            = line.substr(delimiterPos + 1);
        result[section][name] = value;
        std::cout << value << std::endl;
      }
    }

    std::string write_path = "../assets/ini_w.ini";
    std::ofstream fout;
    fout.open(write_path);
    if (!fout.is_open())
    {
      std::cerr << "Unable to open file for writing." << std::endl;
      return 7;
    }

    for (const auto &section : result)
    {
      fout << "[" << section.first << "]" << std::endl;
      for (const auto &kv : section.second)
      {
        fout << kv.first << " = " << kv.second << std::endl;
      }
      fout << std::endl; // Optional: Add a blank line between sections
    }

    fout.close();
  }
  if (0) // ini-lib
  {}

  if (1) // lua
  {
    lua_State *L = luaL_newstate(); // 创建一个新的Lua状态机
    luaL_openlibs(L);               // 打开所有标准的Lua库

    // 运行Lua脚本
    if (luaL_dofile(L, "script.lua") != LUA_OK)
    {
      std::cerr << lua_tostring(L, -1) << std::endl;
      lua_close(L);
      return -1;
    }

    // 调用Lua中定义的函数
    lua_getglobal(L, "add"); // 获取函数名为"add"的函数
    lua_pushnumber(L, 10);   // 推送第一个参数
    lua_pushnumber(L, 20);   // 推送第二个参数

    if (lua_pcall(L, 2, 1, 0) != LUA_OK)
    {
      std::cerr << "Error calling Lua function: " << lua_tostring(L, -1) << std::endl;
      lua_close(L);
      return -1;
    }

    // 获取返回值
    int result = lua_tonumber(L, -1);
    std::cout << "Result: " << result << std::endl;

    lua_close(L); // 关闭Lua
  }

  return 0;
}