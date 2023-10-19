cmake_minimum_required(VERSION 3.6)
project(em_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")

SET(SRC_LIST main.cc)
set(proto_dir /usr/local/include/google/protobuf)
find_package(Protobuf REQUIRED)  # cmake提供的查找protobuf库
find_package(absl REQUIRED) # cmake提供的查找absl库
find_package(gflags REQUIRED) 
# find_package(glog REQUIRED) 
find_package(osqp REQUIRED) 
# find_package(cyber REQUIRED) 
# find_package(fastrtps REQUIRED) 
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})  #添加头文件
include_directories(${PROTOBUF_INCLUDE_DIR})   #添加头文件
include_directories(${CMAKE_CURRENT_BINARY_DIR})  #添加二进制编译目录
include_directories(${proto_dir})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})  #加这一句，就不用加../
set(absl_LIBRARIES  # 设置absl库，没有这个编译会找不到absl
    absl::algorithm
    absl::base
    absl::debugging
    absl::flat_hash_map
    absl::memory
    absl::meta
    absl::numeric
    absl::str_format
    absl::strings
    absl::synchronization
    absl::time
    absl::utility)

# TODO(wushangzhe):待改进，写法太繁琐
# ******** 一、设置路径变量名，下面每有一个都要写一遍 ***********
set(common_config_dir modules/common/configs)
set(common_loadconfig_dir modules/planning/common/my_GetProtoFromFile)
set(common_math_dir modules/common/math)
set(common_util_dir modules/common/util)
# set(common_vehiclestate_dir modules/common/vehicle_state)
set(localization_common_dir /modules/localization/common)
set(planning_common_dir modules/planning/common)
set(planning_common_path_dir modules/planning/common/path)
set(planning_common_speed_dir modules/planning/common/speed)
set(planning_common_trajectory_dir modules/planning/common/trajectory)
# set(planning_common_util_dir modules/planning/common/util)
set(planning_constraintchecker_dir modules/planning/constraint_checker)
set(planning_math_curve1d_dir modules/planning/math/curve1d)
set(planning_math_piecewisejerk_dir modules/planning/math/piecewise_jerk)
set(planning_planner_dir modules/planning/planner)
set(planning_planner_em_planner_dir modules/planning/planner/em_planner)
set(planning_reference_dir modules/planning/reference_line)
set(planning_scenarios_dir modules/planning/scenarios)
set(planning_scenarios_lanefollow_dir modules/planning/scenarios/lane_follow)
set(planning_tasks_dir modules/planning/tasks)
set(planning_tasks_deciders_dir modules/planning/tasks/deciders)
set(planning_tasks_optimizers_dir modules/planning/tasks/optimizers)
set(debug_dir modules/Debug)
# set(simulation_dir simulation) #仿真用
set(planning_tasks_deciders_lanechangedecider_dir modules/planning/tasks/deciders/lane_change_decider)
set(planning_tasks_deciders_newtest_dir modules/planning/tasks/deciders/new_test_decider) # new_test 举例
set(planning_tasks_deciders_stgenerate_dir modules/planning/tasks/deciders/st_generate_decider) # new_test 举例



 # ******** 二、添加头文件 ********
include_directories (${common_config_dir}) 
include_directories (${common_loadconfig_dir}) 
include_directories (${common_math_dir})
include_directories (${common_util_dir})
# include_directories (${common_vehiclestate_dir})
include_directories (${localization_common_dir})
include_directories (${planning_common_dir})
include_directories (${planning_common_path_dir})
include_directories (${planning_common_speed_dir})
include_directories (${planning_common_trajectory_dir})
# include_directories (${planning_common_util_dir}) 
include_directories (${planning_constraintchecker_dir})
include_directories (${planning_math_curve1d_dir})
include_directories (${planning_math_piecewisejerk_dir})
include_directories (${planning_planner_dir})
include_directories (${planning_planner_em_planner_dir})
include_directories (${planning_reference_dir})
include_directories (${planning_scenarios_dir})
include_directories (${planning_scenarios_lanefollow_dir})
include_directories (${planning_tasks_dir})
include_directories (${planning_tasks_deciders_dir})
include_directories (${planning_tasks_optimizers_dir})
include_directories (${planning_tasks_deciders_lanechangedecider_dir})
include_directories (${planning_tasks_deciders_newtest_dir}) # new_test 举例
include_directories (${planning_tasks_deciders_stgenerate_dir}) # new_test 举例
include_directories (${debug_dir})

# include_directories (${simulation_dir}) #仿真用

# 所有需要转换的proto文件
file(GLOB protobuf_files
    modules/common/proto/*.proto
    modules/planning/proto/*.proto
    modules/canbus/proto/*.proto
    modules/map/proto/*.proto
    modules/drivers/proto/*.proto
    modules/common/configs/proto/*.proto
    modules/drivers/canbus/proto/*.proto
    modules/localization/proto/*.proto
    modules/drivers/radar/conti_radar/proto/*.proto
    modules/routing/proto/*.proto
    modules/perception/proto/*.proto
    modules/prediction/proto/*.proto
    modules/dreamview/proto/*.proto
    modules/map/relative_map/proto/*.proto
    modules/common/monitor_log/proto/*.proto
    modules/planning/proto/math/*.proto
    modules/monitor/proto/*.proto
    modules/drivers/radar/racobit_radar/proto/*.proto
    modules/drivers/gnss/proto/*.proto
    modules/common/vehicle_state/proto/*.proto

    )

# 设置list，记录生成的cc、h文件，用于之后连接库
set (PROTO_SRCS)
set (PROTO_HDRS)

SET(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR}) # 当前编译路径
LIST(APPEND PROTO_FLAGS -I${CMAKE_CURRENT_SOURCE_DIR}) # protoc指令

FOREACH(FIL ${protobuf_files}) # 循环要转换的.proto文件

    GET_FILENAME_COMPONENT(FIL_WE ${FIL} NAME_WE) # FIL_WE 文件名前缀
  
    string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})  # FILE_NAME=文件名前缀
    string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL}) # FILE_PATH=文件路径
    
    #string(REGEX MATCH "(ss.*|tt.*)" OUT_PATH ${FILE_PATH})
    # string(REPLACE "/home/pluto/VSCode_Workspace/em_planner/" "" OUT_PATH ${FILE_PATH})
    string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR} "" OUT_PATH ${FILE_PATH}) # 已取消绝对路径

    # message("***********       " ${CMAKE_CURRENT_SOURCE_DIR})

    set(PROTO_SRC "${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.cc") # 设置cc路径
    set(PROTO_HDR "${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}${FIL_WE}.pb.h")  # 设置h路径
    
    if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/CMakeCache.txt)  # 文件不存在才执行，因为编译好慢
    else()
    EXECUTE_PROCESS(
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_META_BASE_DIR} ${FIL} # 调用命令行指令protoc
    )
    file(COPY ${PROTO_SRC} DESTINATION ${FILE_PATH})  # 文件复制
    file(COPY ${PROTO_HDR} DESTINATION ${FILE_PATH})
    endif()
    
    #message("Copying " ${PROTO_SRC} " to " ${FILE_PATH})
  
    list(APPEND PROTO_SRCS ${FILE_PATH}/${FILE_NAME}.pb.cc) # 记录文件，便于后续链接，记录的是转移后的文件，担心链接源文件删除后会有问题
    list(APPEND PROTO_HDRS ${FILE_PATH}/${FILE_NAME}.pb.h)
    # TODO(wushangzhe)：待改进：删除build文件夹的中间文件（已改进）
    # file(REMOVE(${PROTO_SRC}))
    # file(REMOVE(${PROTO_HDR}))
    
ENDFOREACH()

# 删除proto生成的中间
file(REMOVE_RECURSE("${CMAKE_CURRENT_BINARY_DIR}/modules"))

# ********** 三、添加源文件 ***********
aux_source_directory (${common_config_dir} common_config_LIST)
aux_source_directory (${common_loadconfig_dir} common_loadconfig_LIST)
aux_source_directory (${common_math_dir} common_math_LIST)
aux_source_directory (${common_util_dir} common_util_LIST)
# aux_source_directory (${common_vehiclestate_dir} common_vehiclestate_LIST)
aux_source_directory (${localization_common_dir} localization_common_LIST)
aux_source_directory (${planning_common_dir} planning_common_LIST)
aux_source_directory (${planning_common_path_dir} planning_common_path_LIST)
aux_source_directory (${planning_common_speed_dir} planning_common_speed_LIST)
aux_source_directory (${planning_common_trajectory_dir} planning_common_trajectory_LIST)
# aux_source_directory (${planning_common_util_dir} planning_common_util_LIST)
aux_source_directory (${planning_constraintchecker_dir} planning_constraintchecker_LIST)
aux_source_directory (${planning_math_curve1d_dir} planning_math_curve1d_LIST)
aux_source_directory (${planning_math_piecewisejerk_dir} planning_math_piecewisejerk_LIST)
aux_source_directory (${planning_planner_dir} planning_planner_LIST)
aux_source_directory (${planning_planner_em_planner_dir} planning_planner_em_planner_LIST)
aux_source_directory (${planning_reference_dir} planning_reference_LIST)
aux_source_directory (${planning_scenarios_dir} planning_scenarios_LIST)
aux_source_directory (${planning_scenarios_lanefollow_dir} planning_scenarios_lanefollow_LIST)
aux_source_directory (${planning_tasks_dir} planning_tasks_LIST)
aux_source_directory (${planning_tasks_deciders_dir} planning_tasks_deciders_LIST)
aux_source_directory (${planning_tasks_optimizers_dir} planning_tasks_optimizers_LIST)
aux_source_directory (${proto_dir} proto_LIST) # proto文件
aux_source_directory (${planning_tasks_deciders_lanechangedecider_dir} planning_tasks_deciders_lanechangedecider_LIST) # demo举例
aux_source_directory (${planning_tasks_deciders_newtest_dir} planning_tasks_deciders_newtest_LIST)  # new_test 举例
aux_source_directory (${planning_tasks_deciders_stgenerate_dir} planning_tasks_deciders_stgenerate_LIST)  # new_test 举例
aux_source_directory (${debug_dir} debug_LIST) # proto文件

# aux_source_directory (${simulation_dir} simulation_LIST) # 仿真用

# ************** 四、生成静态链接库 SHARED 动态 ************
add_library(
    self_library STATIC ${PROTO_SRCS} ${PROTO_HDRS} ${common_config_LIST} ${common_math_LIST}  ${common_util_LIST} ${common_loadconfig_LIST} 
    ${planning_common_LIST} ${planning_common_path_LIST} ${planning_common_speed_LIST} 
    ${planning_common_trajectory_LIST} ${planning_constraintchecker_LIST} #${planning_common_util_LIST} ${common_vehiclestate_LIST} 
    ${planning_math_curve1d_LIST} ${planning_math_piecewisejerk_LIST} ${planning_planner_LIST} ${planning_planner_em_planner_LIST}
    ${planning_reference_LIST} ${planning_scenarios_LIST} ${planning_scenarios_lanefollow_LIST} ${planning_tasks_LIST} 
    ${planning_tasks_deciders_LIST} ${planning_tasks_optimizers_LIST} ${proto_LIST} ${localization_common_LIST}
    ${planning_tasks_deciders_lanechangedecider_LIST} ${planning_tasks_deciders_newtest_LIST} ${planning_tasks_deciders_stgenerate_LIST}
    ${debug_LIST}
    ) 

add_executable(em_planner_node ${SRC_LIST}) # 生成可执行文件

 # 链接库文件
target_link_libraries(em_planner_node self_library ${PROTOBUF_LIBRARIES} ${absl_LIBRARIES} pthread gflags osqp glog  
    )

# 添加依赖
ament_target_dependencies(
    em_planner_node
    # cyber
    # fastrtps
    # rclcpp
    # std_msgs
    # visualization_msgs
    )