//
// Created by Carlos on May 2019
//

#include "listener.h"


Listener::Listener(int argc,char* argv[])
{
    // Create a ROS nodes
    int _argc = 0;
    char** _argv = NULL;
    ros::init(argc, argv, "db_alpha_listener_node");

    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");

    ros::NodeHandle node("~");
    ROS_INFO("ROS control node just started!");

    // Initialize Subscribers
    joint_state_sub=node.subscribe("/db_dynamixel_ROS_driver/hexapod_joint_feedback", 1, &Listener::jointStatesCallback, this);;

    // Set Rate
    rate = new ros::Rate(17*4); // 60hz
}


Listener::~Listener()
{
    ROS_INFO("ROS listener node shut down!");
}


void Listener::initMsg()
{
    printf("--------------------------------------------------------------------------\n");
    printf("\n"
           "  ____                              _          _  \n"
           " |  _ \\ _   _ _ __   __ _ _ __ ___ (_)_  _____| | \n"
           " | | | | | | | '_ \\ / _` | '_ ` _ \\| \\ \\/ / _ \\ | \n"
           " | |_| | |_| | | | | (_| | | | | | | |>  <  __/ | \n"
           " |____/ \\__, |_| |_|\\__,_|_| |_| |_|_/_/\\_\\___|_| \n"
           "  ____  |___/ ____    ____       _                \n"
           " |  _ \\ / _ \\/ ___|  |  _ \\ _ __(_)_   _____ _ __ \n"
           " | |_) | | | \\___ \\  | | | | '__| \\ \\ / / _ \\ '__|\n"
           " |  _ <| |_| |___) | | |_| | |  | |\\ V /  __/ |   \n"
           " |_| \\_\\\\___/|____/  |____/|_|  |_| \\_/ \\___|_|   \n"
           "                                                  \n");
    printf("--------------------------------------------------------------------------\n");
    printf("\n");
	printf("*******         POSITION LISTENER NODE         *******");
	printf("\n");
	printf("--------------------------------------------------------------------------\n");
	printf("\n");
}



void Listener::jointStatesCallback(const sensor_msgs::JointState& msg)
{
    joint_state_msg = msg;
}


void Listener::readPostions()
{   
    jointPositions.clear();
    jointVelocities.clear();
    jointTorques.clear();

    int index = 0;
    for(std::string name : joint_state_msg.name)
    {
        //jointIDs.push_back(name); 
        jointPositions.push_back(joint_state_msg.position[index]);
        jointVelocities.push_back(joint_state_msg.velocity[index]);
        jointTorques.push_back(joint_state_msg.effort[index]);
        index++;
    }
}


void Listener::rosSpinOnce()
{
    ros::spinOnce();
    bool rateMet = rate->sleep();

    if(!rateMet)
    {
        ROS_ERROR("Sleep rate not met");
    }

}


//---------------------------------------------------------------------------------------