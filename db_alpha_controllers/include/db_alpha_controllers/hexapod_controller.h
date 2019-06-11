//
// Created by Carlos Viescas Huerta on May 2019
//

/* ROS DYNAMIXEL DRIVER FOR COMBINED TORQUE & POSITION CONTROL */
         /* Articulated dung beetle hexapod robot  */

#ifndef DB_ALPHA_HEXAPOD_CONTROLLER_H
#define DB_ALPHA_HEXAPOD_CONTROLLER_H

#include <vector>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

// Include sensor msgs
#include <sensor_msgs/JointState.h>

// Include Dynamixel msgs
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

// Include std msgs
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 1
#define SYNC_WRITE_HANDLER_FOR_CURRENT_LIMIT 2

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

using namespace std;

typedef struct 
{
  std::string item_name;
  int32_t value;
} ItemValue;

class HexapodController
{
    private:

        // ROS NodeHandle
    	ros::NodeHandle node_handle;
		ros::NodeHandle priv_node_handle;
	
		// ROS Topic Publisher
		ros::Publisher dynamixel_state_list_pub;
		ros::Publisher joint_states_pub;
		ros::Publisher joint_IDs_pub;
	
		// ROS Topic Subscriber
		ros::Subscriber goal_joint_state_sub;
		ros::Subscriber multi_joint_goal_sub;
	
		// ROS Service Server
		ros::ServiceServer dynamixel_command_server;
	
		// Dynamixel Workbench Parameters
		DynamixelWorkbench* dxl_wb;

		std::map<std::string, uint32_t> dynamixel;
		
		std::map<std::string, const ControlItem*> control_items;
		std::vector<std::pair<std::string, ItemValue>> dynamixel_info;
		dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;
		sensor_msgs::JointState joint_state_msg;
	
		bool has_joint_state;
		sensor_msgs::JointState goal_state;
		
		double read_period;
    	double write_period;
		double publish_period;

    public:

        // Constructor
        HexapodController();
        
        // Destructor
        ~HexapodController();

        // Initialization
        bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
		bool getDynamixelsInfo(const std::string yaml_file);
		bool loadDynamixels();
		void initMsg();
		bool initDynamixels();
		bool initControlItems();
		bool initSDKHandlers();
		bool initHomePosition();
	
		double getReadPeriod() { return read_period; }
		double getWritePeriod() { return write_period; }
		double getPublishPeriod() { return publish_period; }
	
		void initPublisher();
		void initSubscriber();
	
		void initServer();
	
        // ROS Topic Callbacks
		void readCallback(const ros::TimerEvent&);
		void writeCallback(const ros::TimerEvent&);
		void writeMultiCallback(const ros::TimerEvent&);
		void publishCallback(const ros::TimerEvent&);
	
		void onJointStateGoal(const sensor_msgs::JointState& msg);
		void multiJointGoal(const std_msgs::Float32MultiArray& msg);
	
        // ROS Service Callback
		bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res);

		// Joint configuration vector (format [motor1_ID, motor1_VALUE, motor2_ID, motor2_VALUE, ... , motor21_ID, motor21_VALUE])
		std::vector<float> joint_configuration = {0,0,0,0,0,0, 
											 	  0,0,0,0,0,0, 
											 	  0,0,0,0,0,0,
											 	  0,0,0,0,0,0,
											 	  0,0,0,0,0,0,
											 	  0,0,0,0,0,0,
											 	  0,0,0,0,0,0};
		std::vector<int> joint_identification;
		std::vector<float> home_position = {-0.035, 0.0568, -0.082, 
                                        0.035, 0.0568, -0.082, 
                                       	-0.233, 0.316, -0.087, 
                                       	0.233, 0.316, -0.087, 
                                       	-0.103, 0.244, -0.373, 
                                       	0.103, 0.244, -0.373, 
                                        -0.3497, -0.367, 0.269};
		std::vector<float> dung_beetle_pose = {-0.195, -0.320, 0.36, 
                                          0.195, -0.320, 0.36,
                                          -0.402, -0.259, 0.36,
                                          0.402, -0.259, 0.36,
                                          -0.305, 0.0, 0.394,
                                          0.305, 0.0, 0.394,
                                          -0.1381, -0.1595, -0.0966};
};

#endif //DB_ALPHA_HEXAPOD_CONTROLLER_H