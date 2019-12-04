//
// Created by tomislav on 01. 12. 2019.
//

#ifndef DISTANCE_TH
#define DISTANCE_TH 2.0

#include <limits>
#include <map>
#include <string>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "stdio.h"

#include "geometry_msgs/Point.h"

#include "hololens_ls/GetHumanPath.h"
#include "hololens_ls/GetRobotPose.h"

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

#include "server_link.h"

#define MAX_FLOAT std::numeric_limits<float>::max()

class HololensCommunication {
    geometry_msgs::Pose pose;
    std_msgs::Int64MultiArray published_path;
    bool pose_computed;
    ros::Publisher close_robots_pub;
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

    float human_x, human_y;
    void filterMeasurements(void);
    void humanCallback(const geometry_msgs::Pose);
    void getPositions(void);
    void poseRefreshCallback(void);
    //void humanPoseRefreshCallback(void);
    void humanPathRefreshCallback(void);
    void getPath(void);
    bool return_path_srv(hololens_ls::GetHumanPath::Request &req,
                         hololens_ls::GetHumanPath::Response &res);
    bool return_pose_srv(hololens_ls::GetRobotPose::Request &req,
                         hololens_ls::GetRobotPose::Response &res);

   public:
    ServerLink *server_link;
    HololensCommunication(const std::string &agent_id,
                          const std::string &world_frame);
};

#endif  
