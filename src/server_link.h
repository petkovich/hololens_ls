//
// Created by tomislav on 01. 12. 2019.
//

#ifndef SRC_SERVER_LINK_H
#define SRC_SERVER_LINK_H

#include <atomic>  // For std::atomic_bool.
#include <mutex>   // For std::mutex, std::lock_guard.
#include <thread>

#include <LocationClient.h>  // For LocationClient
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

struct idpose {
    std::string id;
    lsmsg::Pose2DStamped pose_stamped;
};

// used to connect to server and get poses of the robots from FMS
class ServerLink : public LocationClient {
   public:
    ServerLink(const std::string &agent_id, const std::string &world_frame);
    lsmsg::RegistrationMessage makeRegisterMessage();
    void run();
    std::vector<idpose> getRobotPoses();
    std::vector<idpose> getHumanPoses();
    std::vector<int> getHumanPath();
    int rid;
    int getSocket();
    // sgeometry_msgs::Pose getLastPose();
    void pushRequest(ls::JsonMessage);
    void clearRobotPoses();
    void clearHumanPoses();
    void clearHumanPath();
    lsmsg::UpdateLocationMessage move();
    void updateLastPose(geometry_msgs::Pose);

   protected:
    bool onResponse(const std::string &msg);
    std::atomic_bool has_first_pose_{false};
    geometry_msgs::PoseStamped last_pose_;
    std::mutex pose_mutex_;
    std::string world_frame_;
    std::thread send_thread_;

    std::thread update_location_thread_;

    std::thread receive_thread_;
    std::vector<idpose> robot_poses;
    std::vector<idpose> human_poses;
    std::vector<int> human_path;
};
#endif  // SRC_SERVER_LINK_H
