#include "modules/planning/common/my_GetProtoFromFile/GetProtoFromFile.h"

namespace apollo {
namespace planning {

using google::protobuf::TextFormat;
using google::protobuf::io::FileInputStream;
using google::protobuf::io::ZeroCopyInputStream;
using std::istreambuf_iterator;

std::string loadConfig::WorkRoot()
{
    char *cwd = get_current_dir_name();
    std::string FilePath = cwd;
    std::regex Pattern("(^.+)/", std::regex_constants::extended);
    std::match_results<std::string::const_iterator> Result;
    bool Valid = regex_search(FilePath, Result, Pattern);
    if(Valid){
        free(cwd);
        return Result[0];
    }
    std::cout << "find Workroot ERROR" << std::endl;
    return "";
    // return "/home/pluto/VSCode_Workspace/em_planner/modules/planning/conf/planning_config.pb.txt";
}

std::string loadConfig::GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path) {
  if (relative_path.empty()) {
    return prefix;
  }
  // If prefix is empty or relative_path is already absolute.
  if (prefix.empty() || relative_path.front() == '/') {
    return relative_path;
  }

  if (prefix.back() == '/') {
    return prefix + relative_path;
  }
  return prefix + "/" + relative_path;
}

bool loadConfig::GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message) {
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    std::cout << "Failed to open file " << file_name << " in text mode." << std::endl;
    // Failed to open;
    return false;
  }
  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  
  if (!success) {
    std::cout << "Failed to parse file " << file_name << " as text proto." << std::endl;
  }
  
  delete input;
  close(file_descriptor);
  return success;
}

bool loadConfig::GetProtoFromFile(const std::string &file_name, google::protobuf::Message *message) 
{
    // TODO(wushangzhe) 其他类型proto的二进制文件读取

    //   static const std::string kBinExt = ".bin";
    //   if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
    //     return GetProtoFromBinaryFile(file_name, message) ||
    //            GetProtoFromASCIIFile(file_name, message);
    //   }

    //   return GetProtoFromASCIIFile(file_name, message) ||
    //          GetProtoFromBinaryFile(file_name, message);
    std::string config_file_path_ = "";
    if (file_name[0] != '/') {
        config_file_path_ = GetAbsolutePath(WorkRoot(), file_name);
    } else {
        config_file_path_ = file_name;
    }
    return GetProtoFromASCIIFile(config_file_path_, message);
}





}
}