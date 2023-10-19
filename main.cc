#include <iostream>
#include <vector>
#include <list>
#include <malloc.h> 
#include <cmath>
#include <dirent.h>
#include <fcntl.h>
#include <glob.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <cstddef>
#include <fstream>
#include <string>
#include "osqp/osqp.h"
#include "modules/planning/planner/em_planner/em_planner.h"
#include "gflags/gflags.h"

#include "modules/planning/common/frame.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/my_GetProtoFromFile/GetProtoFromFile.h"
#include "modules/planning/tasks/task_factory.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/Debug/Debug_Interface.h"



using namespace std;
using apollo::planning::Frame;
using apollo::planning::FrameHistory;
using apollo::planning::loadConfig;
using apollo::common::TrajectoryPoint;
using apollo::common::Point3D;
using apollo::common::PathPoint;
using apollo::planning::EMPlanner;
using apollo::planning::ReferenceLineInfo;
using apollo::planning::ADCTrajectory;
using apollo::planning::ReferencePoint;
using apollo::planning::Obstacle;
using apollo::perception::PerceptionObstacle;
using apollo::prediction::Trajectory;
using apollo::planning::PlanningConfig;
using apollo::planning::DependencyInjector;
using apollo::planning::TaskFactory;

double lane_equation(double c0, double c1, double c2, double c3, double x)
{
    return c0 + c1*x + c2*x*x + c3*x*x*x;  //rviz坐标y是反的
}

// void log_debug(Frame* frame)
// {
//   DebugInterface* debug_interface = new DebugInterface();
//   std::fstream emplanner_log_file = debug_interface->WriteHeaders();
//   for(auto obs: frame->obstacles()){
//     auto boundary = obs->path_st_boundary();
//     auto upper_points = boundary.upper_points();
//     auto lower_points = boundary.lower_points();
//     // x-axis: t; y-axis: s.
//     emplanner_log_file << "lt,ls,ut,us" << std::endl;
//     for(int i = 0; i < upper_points.size(); i++){
//       emplanner_log_file << lower_points.at(i).t() << ",";
//       emplanner_log_file << lower_points.at(i).s() << ",";
//       emplanner_log_file << upper_points.at(i).t() << ",";
//       emplanner_log_file << upper_points.at(i).s() << ",";
//       emplanner_log_file << std::endl;
//     }
//   }
//   delete(debug_interface);
// }


int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    TrajectoryPoint planning_start_point;
    planning_start_point.set_v(30.0);
    planning_start_point.set_a(0.0);
    planning_start_point.set_relative_time(0.0);
    planning_start_point.set_da(0.0);
    planning_start_point.set_steer(0.0);  //应该是航向角的意思
    PathPoint* start_point = new PathPoint();
    start_point->set_x(0);
    start_point->set_y(0);
    start_point->set_z(0);
    start_point->set_theta(0);
    start_point->set_kappa(0);
    start_point->set_dkappa(0);
    start_point->set_ddkappa(0);
    start_point->set_s(0);
    start_point->set_lane_id("1");
    planning_start_point.set_allocated_path_point(start_point);
    /***构造车辆状态**/
    apollo::common::VehicleState vehicle_state;
    vehicle_state.set_x(0.0);
    vehicle_state.set_y(0.0);
    vehicle_state.set_z(0.0);
    vehicle_state.set_timestamp(0.0);
    vehicle_state.set_roll(0.0);
    vehicle_state.set_pitch(0);
    vehicle_state.set_yaw(0);
    vehicle_state.set_heading(0.0);
    vehicle_state.set_kappa(0.0);
    vehicle_state.set_linear_velocity(30.0);
    vehicle_state.set_angular_velocity(0.0);
    vehicle_state.set_linear_acceleration(0.0);
    // vehicle_state.set_gear(0.0);
    // vehicle_state.set_driving_mode(0.0);
    // vehicle_state.set_pose(0.0);
    vehicle_state.set_steering_percentage(0.0);
    /***构造参考线，用车道线方程形式构建，便于后续仿真***/
    std::vector<ReferencePoint> reference_points; 
    double lane_host_Left_C0 = 1.75;
    double lane_host_Right_C0 = -1.75;
    double lane_adjacent_Left_Left_C0 = 5.25;
    double lane_adjacent_Right_Right_C0 = -5.25;
    for(int i = -50;i < 200;i++)
    {
        double lane_host_Left_y = lane_equation(lane_host_Left_C0, 0, 0, 0, i);
        double lane_host_Right_y = lane_equation(lane_host_Right_C0, 0, 0, 0, i);
        double mid_point_y = (lane_host_Left_y + lane_host_Right_y) * 0.5;
        ReferencePoint ref_point(i, mid_point_y, 0, 0, 0);
        reference_points.push_back(ref_point);
    }
    apollo::planning::ReferenceLine reference_line(reference_points);

    /***最终输出结果***/
    apollo::common::TrajectoryPoint adc_planning_point;  //这个自车规划轨迹是指？后边没什么用

    /***构造参考线信息指针*****/
    ReferenceLineInfo references_line(vehicle_state, adc_planning_point, reference_line);  //参考线信息指针（包含参考线和自车信息）
    
    /****构建多条参考线（这里先只一条）****/
    std::list<ReferenceLineInfo> references_line_info;
    references_line_info.push_back(references_line);


    /*******构建障碍物信息******/
    std::vector<Obstacle*> obstacles;
    PerceptionObstacle* perception_obstacle = new PerceptionObstacle();  //感知模块传来的障碍物信息
    perception_obstacle->set_id(1);    //障碍物id
    Point3D* obs_position = new Point3D();  //障碍物当前位置，只能new，不能malloc
    obs_position->set_x(20.0);
    obs_position->set_y(0.0);
    obs_position->set_z(0.0);
    perception_obstacle->set_allocated_position(obs_position); //proto自定义结构体只能new，而且方法变成set_allocated_xxxx
    perception_obstacle->set_theta(0.0);
    Point3D* obs_velocity = new Point3D();  //障碍物当前车速
    obs_velocity->set_x(25.0);
    obs_velocity->set_y(0.0);
    obs_velocity->set_z(0.0);
    perception_obstacle->set_allocated_velocity(obs_velocity);
    perception_obstacle->set_length(4.933);
    perception_obstacle->set_width(2.11);
    perception_obstacle->set_height(1.48);
    perception_obstacle->set_type(apollo::perception::PerceptionObstacle_Type_VEHICLE); //5是vehile
    perception_obstacle->set_timestamp(0.0);
    perception_obstacle->set_confidence_type(apollo::perception::PerceptionObstacle_ConfidenceType_CONFIDENCE_RADAR);
    Point3D* obs_acceleration = new Point3D();  //障碍物当前加速度
    obs_acceleration->set_x(0.0);
    obs_acceleration->set_y(0.0);
    obs_acceleration->set_z(0.0);
    perception_obstacle->set_allocated_acceleration(obs_acceleration);
    perception_obstacle->set_sub_type(apollo::perception::PerceptionObstacle_SubType_ST_CAR);

    apollo::prediction::Trajectory* obs1_trajectory = new Trajectory();
    obs1_trajectory->set_probability(0.8); // 这条轨迹的置信度
    for(int i = 0;i < 100;i++)
    {
        auto traj = obs1_trajectory->add_trajectory_point();  //repeated变量的赋值方法
        traj->set_v(25);
        traj->set_a(0);
        traj->set_relative_time(i/10.0);
        traj->set_da(0);
        traj->set_steer(0);
        // 障碍物轨迹点
        PathPoint* traj_point = new PathPoint();
        traj_point->set_x(traj->v()*traj->relative_time()+obs_position->x());
        traj_point->set_y(0.0);
        traj_point->set_z(0.0);
        traj_point->set_theta(0.0);
        traj_point->set_kappa(0.0);
        traj_point->set_dkappa(0.0);
        traj_point->set_ddkappa(0.0);
        traj_point->set_s(traj->v()*traj->relative_time()+obs_position->x());
        traj_point->set_lane_id("1");
        traj->set_allocated_path_point(traj_point);
    }
    
    Obstacle* ob1 = new Obstacle("1", *perception_obstacle, *obs1_trajectory, apollo::prediction::ObstaclePriority::NORMAL, false);
    obstacles.push_back(ob1);

    // log_debug(frame);


    // planning : 实际应用时加循环
    uint32_t sequence_num = 0;  // 帧数 
    FrameHistory* frame_history = new FrameHistory(); // 存放历史帧信息（只存一帧）
    Frame* frame = new Frame(sequence_num, vehicle_state, &references_line_info, obstacles);
    if(sequence_num != 0)  // 如果不是第一帧数据，就把上一帧的规划结果给frame_history，用于轨迹拼接
        frame_history->Add(sequence_num, std::unique_ptr<Frame>(frame));
    /**
     * 历史轨迹中获取上一帧规划结果的调用方法：
     * frame_history->Latest()->reference_line_info().front().trajectory();
    */

    // TODO(wushangzhe):写进配置文件(已写入)
    loadConfig file_;
    PlanningConfig config_;
    std::string config_file_name_ = FLAGS_planning_config_file;
    bool success = file_.GetProtoFromFile(config_file_name_, &config_);
    if(success){
        std::cout << "read planning config file success !" << std::endl;
    }
    else{
        std::cout << "error parse proto file" << std::endl;
    }

    auto injector_ = std::make_shared<DependencyInjector>(); // 不用管这个变量
    injector_->planning_context()->Init();

    TaskFactory::Init(config_, injector_); // 注册所有task配置（因为写在了主配置里，待优化）
    EMPlanner* planner = new EMPlanner(injector_);
    planner->Init(config_);
    // planning begin()
    planner->Plan(frame_history, frame);


    return 0;
}

