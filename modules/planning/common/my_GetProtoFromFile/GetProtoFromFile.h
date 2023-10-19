#ifndef GLOBAL
#define GLOBAL


#include <iostream>
#include <fstream>
#include <string>
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <vector>
#include <string>
#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include "gflags/gflags.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include <regex>


/**
 * 从txt中读取配置信息给proto结构体赋值，已测试可用
*/
namespace apollo {
namespace planning {

class loadConfig {
  public:
    bool GetProtoFromFile(const std::string &file_name, 
                            google::protobuf::Message *message);
    bool GetProtoFromASCIIFile(const std::string &file_name,
                            google::protobuf::Message *message);
    std::string GetAbsolutePath(const std::string &prefix,
                              const std::string &relative_path);
    std::string WorkRoot();

};

}  // namespace common
}  // namespace apollo


#endif