//
// Created by goran on 29. 10. 2019..
//
#include "comm.h"

using namespace Eigen;


Uwb::Uwb(const std::string & agent_id, const std::string & world_frame) {
    this->pose_computed = false;
    ros::NodeHandle n;
    //this->uwb_timer = n.createTimer(ros::Duration(1), &Uwb::computePosesCallback, this);
    this->uwb_pub  = n.advertise<geometry_msgs::PoseArray>("/close_robots", 1000);
    this->path_pub  = n.advertise<std_msgs::Int64MultiArray>("/human_path", 1000);
    //this->pose_refresh_timer = n.createTimer(ros::Duration(1), &Uwb::poseRefreshCallback, this);
    //this->human_pose_refresh_timer = n.createTimer(ros::Duration(1), &Uwb::humanPoseRefreshCallback, this);
    //this->human_path_refresh_timer = n.createTimer(ros::Duration(1), &Uwb::humanPathRefreshCallback, this);

    this->get_path_srv = n.advertiseService("/human_path_srv", &Uwb::return_path_srv, this);
    this->get_pose_srv =  n.advertiseService("/close_robot_srv", &Uwb::return_pose_srv, this);

    this->human_x= 1.0;
    this->human_y = 2.0;
    this->D = 2.0;
    //std::cout<<"timer created"<<std::endl;
    server_link=new ServerLink{agent_id, world_frame};
}

bool Uwb::return_path_srv(hololens_ls::GetHumanPath::Request& req, hololens_ls::GetHumanPath::Response& res){
    ROS_INFO("Requested human path");
    int m_socket=server_link->getSocket();

    lsmsg::UpdateLocationMessage msg = server_link->move();
	ls::JsonMessage reqq(m_socket, msg.toJsonString());
	server_link->pushRequest(reqq);

    this->humanPathRefreshCallback();
    ros::Duration(1.0).sleep();
    this->getPath();
    res.path=this->return_path;
    return true;
}
bool Uwb::return_pose_srv(hololens_ls::GetRobotPose::Request& req, hololens_ls::GetRobotPose::Response& res){
    ROS_INFO("Requested close robots");
    this->humanPoseRefreshCallback();
    this->poseRefreshCallback();
    ros::Duration(1.0).sleep();
    this->getPositions();
    res.positions = this->close_robots;
    return true;
}


void Uwb::computePose(){
    //this->filterMeasurements();
    this->getPositions();
 
}

void Uwb::getPath(){

    std::vector<int> human_path;
    human_path = this->server_link->getHumanPath(); 

    this->server_link->clearHumanPath();

    if(human_path.size()){
        this->return_path.data.clear();
        for(auto it = 0; it<human_path.size(); it++){
            this->return_path.data.push_back(human_path[it]);
        }
        //path_pub.publish(return_path);   
    }

}

void Uwb::getPositions() {
    std::vector<idpose> poses;
    std::vector<idpose> human_poses;


    poses = this->server_link->getRobotPoses();
    human_poses =this->server_link->getHumanPoses(); 
    this->server_link->clearRobotPoses();
    this->server_link->clearHumanPoses();


    if (!human_poses.size()){
        return;
    }
    auto human_pose = human_poses[0];
    this->human_x = human_pose.pose_stamped.pose.x;
    this->human_y = human_pose.pose_stamped.pose.y;
   // std::cout << "Human pose: " << this->human_x << ' '<< this->human_y << '\n';
    this->close_robots.poses.clear();
    int x=0;
    geometry_msgs::Pose aRobot;
    for (auto it = poses.begin(); it != poses.end(); ++it) {


        //std::cout<< "Robot " << it->id << ": " << it->pose_stamped.pose.x << ' ' << it->pose_stamped.pose.y << '\n';
        auto distance = pow(it->pose_stamped.pose.x-this->human_x, 2) + pow(it->pose_stamped.pose.y-this->human_y, 2);
        //std::cout << "Distance: " << pow(distance, 0.5) << '\n';
        if (pow(distance, 0.5) < this->D || 1){
            aRobot.position.x=it->pose_stamped.pose.x;
            aRobot.position.y=it->pose_stamped.pose.y;
            this->close_robots.poses.push_back(aRobot);
        }
        //this->uwb_pub.publish(close_robots);
    }
 
}


void Uwb::computePosesCallback(const ros::TimerEvent& event){
    this->computePose();
}

void Uwb::poseRefreshCallback() {
    lsmsg::Agent agent;
    agent.id="all";
    agent.type="robot";
    lsmsg::QueryLocationMessage queryMsg(agent);
    int socket=server_link->getSocket();
    ls::JsonMessage req(socket, queryMsg.toJsonString());
    server_link->pushRequest(req);
}

void Uwb::humanPoseRefreshCallback() {
    lsmsg::Agent agent;
    agent.id="all";
    agent.type="human";
    lsmsg::QueryLocationMessage queryMsg(agent);
    int socket=server_link->getSocket();
    ls::JsonMessage req(socket, queryMsg.toJsonString());
    server_link->pushRequest(req);
}

void Uwb::humanPathRefreshCallback() {
    lsmsg::Agent agent;
    agent.id="all";
    agent.type="human";
    lsmsg::QueryPathMessage queryMsg(agent);
    int socket=server_link->getSocket();
    ls::JsonMessage req(socket, queryMsg.toJsonString());
    server_link->pushRequest(req);
}