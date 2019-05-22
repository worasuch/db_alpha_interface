#ifndef DB_ALPHA_ONE_LEG_CONTROLLER_H
#define DB_ALPHA_ONE_LEG_CONTROLLER_H

#include <vector>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 0

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

using namespace std;

typedef struct 
{
  std::string item_name;
  int32_t value;
} ItemValue;


class OneLegController 
{
  	private:
    
		// ROS NodeHandle
    	ros::NodeHandle node_handle;
		ros::NodeHandle priv_node_handle;
	
		// ROS Topic Publisher
		ros::Publisher dynamixel_state_list_pub;
		ros::Publisher joint_states_pub;
	
		// ROS Topic Subscriber
		ros::Subscriber goal_joint_state_sub;
	
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

		OneLegController();
		~OneLegController();
	
		bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
		bool getDynamixelsInfo(const std::string yaml_file);
		bool loadDynamixels();
		void initMsg();
		bool initDynamixels();
		bool initControlItems();
		bool initSDKHandlers();
        bool initialTorque();
	
		double getReadPeriod() { return read_period; }
		double getWritePeriod() { return write_period; }
		double getPublishPeriod() { return publish_period; }
	
		void initPublisher();
		void initSubscriber();
	
		void initServer();
	
		void readCallback(const ros::TimerEvent&);
		void writeCallback(const ros::TimerEvent&);
		void publishCallback(const ros::TimerEvent&);
	
		void onJointStateGoal(const sensor_msgs::JointState& msg);
	
		bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res);

        // Variables
        vector<int> joint_identification;
        vector<float> init_currents = {5, 5};
};


#endif //DB_ALPHA_TORQUE_CONTROLLER_H