/*
 * @Author: mengtiantian
 * @Date: 2022-09-28 17:59:11
 * @LastEditors: wushangzhe
 * @LastEditTime: 2023-10-18 19:29:30
 */
#ifndef ADAS_INTERFACE_H
#define ADAS_INTERFACE_H

#include <sys/time.h>

#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "stdlib.h"
#include "string.h"
using namespace std;

class DebugInterface {
public:
  DebugInterface();
  ~DebugInterface();
  std::fstream WriteHeaders(const char* file_name);
  bool isNum(string str);

  std::fstream log_file_;
};

#endif
