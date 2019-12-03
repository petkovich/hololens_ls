//
// Created by goran on 29. 10. 2019..
//

#ifndef SRC_UWB_H
#define SRC_UWB_H

#include <limits>
#include <map>
#include <string>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "stdio.h"

#include "geometry_msgs/Point.h"
#include "hololens_ls/RobotDistance.h"
#include "hololens_ls/uwb.h"

#include "hololens_ls/GetHumanPath.h"
#include "hololens_ls/GetRobotPose.h"

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

#include "server_link.h"

#define MAX_FLOAT std::numeric_limits<float>::max()

struct measurement {
    int id;
    float uwb_timestamp;
    uint64_t server_timestamp;
    float x;
    float y;
    float distance;
};

struct point {
    float x;
    float y;
};

struct pointstamped {
    point p;
    uint64_t time;
};

class Uwb {
    std::vector<measurement> measurements;
    // values should come from FMS and UWB nodes. It's hardcoded for testing
    // purposes!
    std::map<int, std::string> lookup_table = {
        {40970, "r1"}, {40990, "r2"}, {40963, "r3"}, {40755, "r4"}};
    // values should come from FMS. It's hardcoded for testing purposes!
    std::map<std::string, pointstamped>
        fms_poses;  // = {{"r1", { 0.00,  0.00}},
                    //                                              {"r2",
                    //                                              {-1.20,
                    //                                              -3.60}},
                    //                                              {"r3",
                    //                                              {10.20,
                    //                                              -4.20}},
                    //                                              {"r4",
                    //                                              { 5.40,
                    //                                              0.00}}};
    float measurement_time_treshold;
    geometry_msgs::Pose pose;
    std_msgs::Int64MultiArray published_path;
    bool pose_computed;
    ros::Publisher uwb_pub;
    ros::Publisher path_pub;
    ros::Subscriber human_pose_sub;
    ros::Timer uwb_timer;
    ros::Timer pose_refresh_timer;
    ros::Timer human_pose_refresh_timer;
    ros::Timer human_path_refresh_timer;
    ros::ServiceServer get_path_srv;
    ros::ServiceServer get_pose_srv;

    std_msgs::Int64MultiArray return_path;
    geometry_msgs::PoseArray close_robots;

    float human_x, human_y, D;
    void addMeasurement(hololens_ls::RobotDistance);
    void filterMeasurements(void);
    void computePose(void);
    void humanCallback(const geometry_msgs::Pose);
    void getPositions(void);
    void uwbCallback(const hololens_ls::RobotDistance);
    void computePosesCallback(const ros::TimerEvent &event);
    void poseRefreshCallback(void);
    void humanPoseRefreshCallback(void);
    void humanPathRefreshCallback(void);
    void getPath(void);
    bool return_path_srv(hololens_ls::GetHumanPath::Request &req,
                         hololens_ls::GetHumanPath::Response &res);
    bool return_pose_srv(hololens_ls::GetRobotPose::Request &req,
                         hololens_ls::GetRobotPose::Response &res);

   public:
    ServerLink *server_link;
    Uwb(const std::string &agent_id, const std::string &world_frame);
};

#endif  // SRC_UWB_LOCALIZATION_H
