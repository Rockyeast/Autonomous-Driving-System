/*
 * @Author: mengtiantian
 * @Date: 2022-09-28 17:59:11
 * @LastEditors: wushangzhe
 * @LastEditTime: 2023-10-18 19:29:30
 */

#include "modules/Debug/Debug_Interface.h"
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <map>
#include <stack>
#include <vector>

#include <pthread.h>
#include <ctime>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include "stddef.h"
#include <stdio.h>
#include "stddef.h"

#define LOG_FILE_PATH "/VSCode_Workspace/em_planner/modules/Debug/log"
DebugInterface::DebugInterface() {}

DebugInterface::~DebugInterface() {
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

bool DebugInterface::isNum(string str) {
  stringstream sin(str);
  double d;
  char c;
  if (!(sin >> d)) {
    return false;
  }
  if (sin >> c) {
    return false;
  }
  return true;
}

std::fstream DebugInterface::WriteHeaders(const char* file_name) {
  std::fstream log_file_;
  time_t rawtime;
  char name_buffer[80];
  std::time(&rawtime);
  std::tm time_tm;
  localtime_r(&rawtime, &time_tm);

  std::string dir;
  dir = getenv("HOME");
  dir = dir + LOG_FILE_PATH;
  std::string commond = "mkdir -p " + dir;
  system(commond.c_str());

  // strftime(name_buffer, 80, "/%F_%H%M%S.csv", &time_tm);
  strftime(name_buffer, 80, file_name, &time_tm);
  dir = dir + name_buffer;
  log_file_.close();
  log_file_.open(dir.c_str(), std::ios_base::out);

  // emplanner_log_file << "Lapa_Dis_Planning_resultX   , "
  //               << "Lapa_Dis_Planning_resultX,  "
  //               << std::endl;
  log_file_.flush();
  return log_file_;
}

