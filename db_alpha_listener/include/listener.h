//
// Created by Carlos on May 2019
//

#ifndef DB_ALPHA_LISTENER_H
#define DB_ALPHA_LISTENER_H

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>

#include <sensor_msgs/JointState.h>

using namespace std;

class Listener
{
  	private:

        // ROS Topic Subscriber
		ros::Subscriber joint_state_sub;
        
        // MSG
        sensor_msgs::JointState joint_state_msg;

        // Callback
        void jointStatesCallback(const sensor_msgs::JointState& msg);

        // Private Global Variables
        ros::Rate* rate;

    public:

        Listener(int argc,char* argv[]);
        ~Listener();

        void initMsg();

        void readPostions();
        void rosSpinOnce();

        vector<float> jointPositions;
        std::vector<float> jointVelocities;
        std::vector<float> jointTorques;
};

#endif