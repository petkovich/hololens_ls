//
// Created by goran on 28. 10. 2019..
//

#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "comm.h"
#include "server_link.h"

#include <imrStringOps.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "hololens_ls");

    ros::NodeHandle private_nh("~");

    std::string config_filename;
    if (!private_nh.getParam("config_filename", config_filename))
    {
        ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace() << "/config_filename must be set, exiting");
        std::exit(EXIT_FAILURE);
    }

    std::string agent_id;
    if (!private_nh.getParam("agent_id", agent_id))
    {
        ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace() << "/agent_id must be set, exiting");
        std::exit(EXIT_FAILURE);
    }

    std::string world_frame;
    if (!private_nh.getParam("world_frame", world_frame))
    {
        ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace() << "/world_frame must be set, exiting");
        std::exit(EXIT_FAILURE);
    }

    std::ifstream ifs(config_filename);
    if (!ifs.good())
    {
        ROS_ERROR_STREAM("Config file " << config_filename << " can not be found/opened");
        std::exit(EXIT_FAILURE);
    }

    int port;
    int buffer_size;
    std::string ip;
    std::string line;
    while (std::getline(ifs, line))
    {
        std::vector<std::string> tokens = imr::split(line, ':');
        if (tokens[0] == "SERVER_IP")
        {
            ip = imr::Trim(tokens[1]);
        }
        else if(tokens[0] == "SERVER_PORT")
        {
            port = std::atoi(imr::Trim(tokens[1]).c_str());
        }
        else if(tokens[0] == "BUFFER_SIZE")
        {
            buffer_size = std::atoi(imr::Trim(tokens[1]).c_str());
        }
    }


    Uwb uwb{agent_id, world_frame};

    lsmsg::Agent agent;
    agent.id="all";
    agent.type="robot";

    ROS_INFO_STREAM("Connecting to server " << ip << ":" << port);
    uwb.server_link->init(ip, port, buffer_size);
    uwb.server_link->run();
    //uwb.server_link->queryLocationThread(agent);



    ros::spin();

    return 0;
}