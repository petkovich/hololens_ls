//
// Created by goran on 11. 11. 2019..
//

#include "server_link.h"


ServerLink::ServerLink(const std::string & agent_id, const std::string & world_frame)
        : LocationClient{agent_id},  world_frame_(world_frame)
{}

lsmsg::RegistrationMessage ServerLink::makeRegisterMessage()
{
    lsmsg::RegistrationMessage msg(m_id);
    return msg;
}


void ServerLink::run()
{
    receive_thread_ = std::thread([this](){receiverThread();});
    send_thread_ = std::thread{[this](){senderThread();}};
    //update_location_thread_ = std::thread{[this](){updateLocationThread();}};

    /* Let the thread initialize. */
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
}

lsmsg::UpdateLocationMessage ServerLink::move()
{
    //ROS_INFO("Moving");
    ROS_DEBUG("Sending last pose to the server");
    std::lock_guard<std::mutex> guard(pose_mutex_);
    lsmsg::UpdateLocationMessage msg;
    lsmsg::Location location;
    location.nodeId = 0;
    msg.addLocation(lsmsg::Agent(m_id, lsmsg::AGENT_HUMAN), location);
    return msg;
}

bool ServerLink::onResponse(const std::string &msg) {
    //human_poses.clear();
    //robot_poses.clear();
    auto dict = imr::JSON::Dictionary(msg);
    std::string op = dict.Get("op");
    //print("msg: " + msg);
    //print("op: " + op);
    if(op.empty())
    {
        print("failed to parse msg: " + msg);
        return false;
    }
    else
    {
        if(op == lsmsg::op::RES_CONN){
            print(msg);
        }
        else if(op == lsmsg::op::RES_QPAH){
            lsmsg::ResponsePathMessage respMsg;
            respMsg.fromJsonString(msg);
            std::unordered_map<lsmsg::Agent, lsmsg::NodePath>* resp = respMsg.getData();
            for(std::unordered_map<lsmsg::Agent, lsmsg::NodePath>::iterator it = resp->begin(); it != resp->end(); it++)
            {

            auto scnd = it->second.m_path;
            for(auto item = scnd.begin(); item!= scnd.end(); item++){
                //std::cout << item->m_nodeId << '\n';
                human_path.push_back(item->m_nodeId);
            }

            std::string id = it->first.id;
            std::string path = it->second.toJsonString();
            std::string s = id + ": " + path;
            print(s);
            }
        }
        else if(op == lsmsg::op::RES_QLOC){
            lsmsg::ResponseLocationMessage respMsg;
            respMsg.fromJsonString(msg);
            std::unordered_map<lsmsg::Agent, lsmsg::Location>* resp = respMsg.getData();
            for(std::unordered_map<lsmsg::Agent, lsmsg::Location>::iterator it = resp->begin(); it != resp->end(); it++)
            {
                std::string id = it->first.id;
                std::string type = it->first.type;
                std::string loc = it->second.toJsonString();

               if(type == lsmsg::AGENT_ROBOT)
                {
                   std::lock_guard<std::mutex> lock(m_lock_robots);
                   m_robots.insert(it->first);
                   idpose robot_pose;
                   robot_pose.id=it->first.id;
                   robot_pose.pose_stamped=it->second.poseStamped;
                   robot_poses.push_back(robot_pose);
                }
                else if(type== lsmsg::AGENT_HUMAN){
                   std::lock_guard<std::mutex> lock(m_lock_robots);
                   m_robots.insert(it->first);
                   idpose robot_pose;
                   robot_pose.id=it->first.id;
                   robot_pose.pose_stamped=it->second.poseStamped;
                   human_poses.push_back(robot_pose); 
                }


               // print(id + ": " + loc);
            }
        }
        else if(op == lsmsg::op::RES_QPAH){
            lsmsg::ResponsePathMessage respMsg;
            respMsg.fromJsonString(msg);
            std::unordered_map<lsmsg::Agent, lsmsg::NodePath>* resp = respMsg.getData();
            for(std::unordered_map<lsmsg::Agent, lsmsg::NodePath>::iterator it = resp->begin(); it != resp->end(); it++)
            {
                std::string id = it->first.id;
                std::string path = it->second.toJsonString();
                std::string s = id + ": " + path;
                print(s);
            }
        }
        else{
            print("unknown response message type: " + msg);
            return false;
        }
        return true;
    }
}

std::vector<idpose> ServerLink::getRobotPoses() {
    return robot_poses;
}

std::vector<idpose> ServerLink::getHumanPoses() {
    return human_poses;
}

std::vector<int> ServerLink::getHumanPath() {
    return human_path;
}

int ServerLink::getSocket() {
    return m_socket;
}
void ServerLink::clearRobotPoses(){
    robot_poses.clear();
}
void ServerLink::clearHumanPoses(){
    human_poses.clear();
}

void ServerLink::clearHumanPath(){
    human_path.clear();
}
void ServerLink::pushRequest(const ls::JsonMessage req){
    m_requests.push(req);
}
