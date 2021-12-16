//
// Edited by CVH on May 2019
//


#include "db_alpha_controllers/reboot.h"



// Constructor
PositionController::PositionController() : node_handle(""), priv_node_handle("~"), has_joint_state(false) 
{
	read_period = priv_node_handle.param<double>("dxl_read_period", 0.010f);
	write_period = priv_node_handle.param<double>("dxl_write_period", 0.010f);
	publish_period = priv_node_handle.param<double>("publish_period", 0.010f);
	dxl_wb = new DynamixelWorkbench;
}


// Destructor
PositionController::~PositionController() 
{
	usleep(1000000); // sleep for 1 second
	ROS_INFO("Torque Disable");

	const char* log;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel)
	{
		dxl_wb->torqueOff((uint8_t)dxl.second);
		// std::cout << "dxl.second: " << dxl.second << std::endl;

		for (std::pair<std::string, ItemValue> const& info : dynamixel_info)
		{

		}
		dxl_wb->torqueOff((uint8_t)dxl.second);
	}
}


//------------------------------------------------------------------------------------------------

bool PositionController::initWorkbench(const std::string port_name, const uint32_t baud_rate) 
{
	bool result = false;
	const char* log;

	result = dxl_wb->init(port_name.c_str(), baud_rate, &log);
	if (result == false)
	{
		ROS_ERROR("%s", log);
	}
	return result;
}

bool PositionController::getDynamixelsInfo(const std::string yaml_file)
{
	// Data Structure Description /////
	// name = id_101, id_102, ..., id_121
	// item_name(first:xxx) = (ID:motor_ID, Operating_Mode, Profile_Velocity, Profile_Acceleration) 
	// value(xxx:second) = (11(motor_ID), 3, 200, 50) 
	// item_value(specific_name) = struct(item_name, value) 
	// info = map(name, item_value) 
	// dynamixel_info = list(info) 
	// dynamixel = map(name, motor_ID) 

	YAML::Node dxl_node;
	dxl_node = YAML::LoadFile(yaml_file.c_str());

	if (dxl_node == NULL) return false;

	for (YAML::const_iterator it_file = dxl_node.begin(); it_file != dxl_node.end(); it_file++) 
	{
		std::string name = it_file->first.as<std::string>();
		if (name.size() == 0) continue;

		YAML::Node item = dxl_node[name];
		for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
		{
			std::string item_name = it_item->first.as<std::string>();
			int32_t value = it_item->second.as<int32_t>();

			if (item_name == "ID") dynamixel[name] = value;

			ItemValue item_value = {item_name, value};
			std::pair<std::string, ItemValue> info(name, item_value);

			dynamixel_info.push_back(info);
		}
	}
	return true;
}

bool PositionController::loadDynamixels()
{
	bool result = false;
	const char* log;
	int motor_cnt = 0;

	// dynamixel = map(name, motor_ID) 
	for (std::pair<std::string, uint32_t> const& dxl : dynamixel) 
	{
		uint16_t model_number = 0;
		result = dxl_wb->ping((uint8_t)dxl.second, &model_number, &log);
    	if (result == false)
		{
			ROS_ERROR("%s", log);
			ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
			return result;
		} 
		else 
		{
			ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
			joint_identification.push_back((uint8_t)dxl.second);
			motor_cnt++;
		}
	}
	ROS_INFO("Number of joints : %d", motor_cnt);
	return result;
}


// Initialization message
void PositionController::initMsg()
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
	printf("*******         POSITION CONTROLLER         *******");
	printf("\n");
	printf("--------------------------------------------------------------------------\n");
	printf("\n");
}


bool PositionController::initDynamixels() 
{
	ROS_INFO("Torque Enabled");
	
	const char* log;

	// dynamixel = map(name, motor_ID) 
	for (std::pair<std::string, uint32_t> const& dxl : dynamixel)
	{
		dxl_wb->torqueOff((uint8_t)dxl.second);

		for (std::pair<std::string, ItemValue> const& info : dynamixel_info)
		{
			if (dxl.first == info.first)
			{
				if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
				{
					bool result = dxl_wb->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
					if (result == false)
					{
						ROS_ERROR("%s", log);
						ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
						return false;
					}
				}
			}
		}
		dxl_wb->torqueOn((uint8_t)dxl.second);
	}
	return true;
}

bool PositionController::initControlItems()
{
	bool result = false;
	const char* log = NULL;

	uint32_t dxl_num = dynamixel.begin()->second;

	const ControlItem* goal_position = dxl_wb->getItemInfo(dxl_num, "Goal_Position");
	if (goal_position == NULL) return false;

	const ControlItem* present_position = dxl_wb->getItemInfo(dxl_num, "Present_Position");
	if (present_position == NULL) return false;

	const ControlItem* present_velocity = dxl_wb->getItemInfo(dxl_num, "Present_Velocity");
	if (present_velocity == NULL) return false;

	const ControlItem* present_current = dxl_wb->getItemInfo(dxl_num, "Present_Current");
	if (present_current == NULL) return false;

	control_items["Goal_Position"] = goal_position;

	control_items["Present_Position"] = present_position;
	control_items["Present_Velocity"] = present_velocity;
	control_items["Present_Current"] = present_current;

	return true;
}

bool PositionController::initSDKHandlers()
{
	bool result = false;
	const char* log = NULL;
	
	result = dxl_wb->addSyncWriteHandler(control_items["Goal_Position"]->address, control_items["Goal_Position"]->data_length, &log);
	if (result == false)
	{
		ROS_ERROR("%s", log);
		return result;
	} else ROS_INFO("%s", log);
	
	uint16_t start_address = std::min(control_items["Present_Position"]->address, control_items["Present_Current"]->address);
    uint16_t read_length = control_items["Present_Position"]->data_length + 
						   control_items["Present_Velocity"]->data_length + 
						   control_items["Present_Current"]->data_length;

    result = dxl_wb->addSyncReadHandler(start_address, read_length, &log);
    if (result == false)
	{
		ROS_ERROR("%s", log);
		return result;
	}
	return result;
}


// Set robot to home position during initialization
bool PositionController::initHomePosition()
{
	bool result = false;
	const char* log = NULL;
	
	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

	int32_t dynamixel_position[joint_identification.size()];
	
	for (int index = 0; index < joint_identification.size(); index++)
	{ 
		id_array[index] = joint_identification[index];
		//dynamixel_position[index] = dxl_wb->convertRadian2Value(joint_identification[index], home_position[index]);
		dynamixel_position[index] = dxl_wb->convertRadian2Value(joint_identification[index], dung_beetle_pose[index]);
		id_cnt++;
	}

	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
	ROS_INFO("Robot set to Home Position Successfully.");
	return result;
}


// Publishes robot state
void PositionController::initPublisher()
{
	dynamixel_state_list_pub = priv_node_handle.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_states_list", 100);
	joint_states_pub = priv_node_handle.advertise<sensor_msgs::JointState>("hexapod_joint_feedback", 100);
	joint_IDs_pub = priv_node_handle.advertise<std_msgs::Int32MultiArray>("hexapod_joint_IDs", 100);
}

void PositionController::initSubscriber() 
{
	// Change this lines to use Float34MultiArray / JointState ROS messages
	goal_joint_state_sub = priv_node_handle.subscribe("hexapod_state_commands", 100, &PositionController::onJointStateGoal, this);
	//multi_joint_goal_sub = priv_node_handle.subscribe("hexapod_multi_joint_commands", 100, &PositionController::multiJointGoal, this);
}


void PositionController::initServer()
{
	dynamixel_command_server = priv_node_handle.advertiseService("dynamixel_request_commands", &PositionController::dynamixelCommandMsgCallback, this);
	dynamixel_command_server_reboot = priv_node_handle.advertiseService("dynamixel_request_commands_reboot", &PositionController::dynamixelRebootCallback, this);
	dynamixel_command_server_torqueOff = priv_node_handle.advertiseService("dynamixel_request_commands_torqueOff", &PositionController::dynamixelTorqueOffCallback, this);
	dynamixel_command_server_torqueOn = priv_node_handle.advertiseService("dynamixel_request_commands_torqueOn", &PositionController::dynamixelTorqueOnCallback, this);
}


void PositionController::onJointStateGoal(const sensor_msgs::JointState& msg)
{
	goal_state = msg;
	has_joint_state = true;
}


// Multi joint msg callback
void PositionController::multiJointGoal(const std_msgs::Float32MultiArray& msg)
{
	joint_configuration = msg.data;
}


// Reading motor feedback
void PositionController::readCallback(const ros::TimerEvent&)
{
	bool result = false;
	const char* log = NULL;

	dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel.size()];
	dynamixel_state_list.dynamixel_state.clear();

	int32_t get_current[dynamixel.size()];
	int32_t get_velocity[dynamixel.size()];
	int32_t get_position[dynamixel.size()];

	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel)
	{
		dynamixel_state[id_cnt].name = dxl.first;
		dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

		id_array[id_cnt++] = (uint8_t)dxl.second;
	}
	
	result = dxl_wb->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, dynamixel.size(), &log);
    if (result == false) 
	{
		ROS_ERROR("%s", log);
	}

	result = dxl_wb->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
									  control_items["Present_Current"]->address,
                                      control_items["Present_Current"]->data_length,
                                      get_current,
                                      &log);
	if (result == false)
	{
		ROS_ERROR("%s", log);
	}

	result = dxl_wb->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
									  control_items["Present_Velocity"]->address,
									  control_items["Present_Velocity"]->data_length,
								      get_velocity,
									  &log);
	if (result == false)
	{
		ROS_ERROR("%s", log);
	}

	result = dxl_wb->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
                                      control_items["Present_Position"]->address,
                                      control_items["Present_Position"]->data_length,
                                      get_position,
                                      &log);
	if (result == false)
	{
		ROS_ERROR("%s", log);
	}

	for(uint8_t index = 0; index < id_cnt; index++)
	{
		dynamixel_state[index].present_current = get_current[index];
		dynamixel_state[index].present_velocity = get_velocity[index];
        dynamixel_state[index].present_position = get_position[index];

        dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
	}	
}


// Write motor values
void PositionController::writeCallback(const ros::TimerEvent& t)
{
	if (has_joint_state == false) return;
	
	bool result = false;
	const char* log = NULL;
	
	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

	int32_t dynamixel_position[dynamixel.size()];

	for (std::string name : goal_state.name)
	{
		id_array[id_cnt] = (uint8_t) dynamixel[name];
		id_cnt++;
	}
	
	for (uint8_t index = 0; index < id_cnt; index++) 
		dynamixel_position[index] = dxl_wb->convertRadian2Value(id_array[index], goal_state.position[index]);
	
	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
	if (result == false) ROS_ERROR("%s", log);
	
	has_joint_state = false;
}


// Multi joint motor values
void PositionController::writeMultiCallback(const ros::TimerEvent&)
{
	bool result = false;
	const char* log = NULL;

	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

	int32_t dynamixel_position[dynamixel.size()];	
	
	// Get control values
	for(size_t index=0; index<joint_configuration.size(); index+=2)
	{
		int motor_id = (int) joint_configuration[index];
		dynamixel_position[id_cnt] = dxl_wb->convertRadian2Value(motor_id, joint_configuration[index+1]);
		id_array[id_cnt] = motor_id;
		id_cnt++;
	}
	
	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
	if (result == false) ROS_ERROR("FAILED TO SET MOTOR POSITIONS");

}


void PositionController::publishCallback(const ros::TimerEvent&)
{
	// Publish dynamixel state list
	dynamixel_state_list_pub.publish(dynamixel_state_list);
	
	// Publish hexapod joint IDs
	std_msgs::Int32MultiArray array_IDs;
	array_IDs.data.clear();
	
	for (size_t i=0; i<joint_identification.size(); i++)
	{
		array_IDs.data.push_back(joint_identification[i]);
	}

	joint_IDs_pub.publish(array_IDs);

	// Publish joint state 
	joint_state_msg.header.stamp = ros::Time::now();

    joint_state_msg.name.clear();
    joint_state_msg.position.clear();
    joint_state_msg.velocity.clear();
    joint_state_msg.effort.clear();

    uint8_t id_cnt = 0;
	
	for (std::pair<std::string, uint32_t> const& dxl : dynamixel) 
	{
		double position = 0.0;
		double velocity = 0.0;
		double effort = 0.0;
      
		joint_state_msg.name.push_back(dxl.first);
		
		effort = dxl_wb->convertValue2Current((int16_t)dynamixel_state_list.dynamixel_state[id_cnt].present_current);
		velocity = dxl_wb->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list.dynamixel_state[id_cnt].present_velocity);
		position = dxl_wb->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list.dynamixel_state[id_cnt].present_position);

		joint_state_msg.effort.push_back(effort);
		joint_state_msg.velocity.push_back(velocity);
		joint_state_msg.position.push_back(position);

		id_cnt++;
	}
	joint_states_pub.publish(joint_state_msg);
}

bool PositionController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res) 
{
	bool result = false;
	const char* log;

	uint8_t id = req.id;
	std::string item_name = req.addr_name;
	int32_t value = req.value;

	result = dxl_wb->itemWrite(id, item_name.c_str(), value, &log);
	if (result == false) 
	{
		ROS_ERROR("%s", log);
		ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
	}

	res.comm_result = result;

	return true;
}

bool PositionController::dynamixelRebootCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) 
{
	// Check shutdown address: 63, size: 1 byte, Data Name: 'Shutdown', initial value: 52
	// Then reboot
	bool result = false;
	const char* log;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel)
	{
		std::cout << "dxl.second: " << dxl.second << std::endl;
		int32_t status;
		// int32_t status;
		std::cout << "status1: " << status << std::endl;
		bool read_result = dxl_wb->itemRead((uint8_t)dxl.second, "Hardware_Error_Status", &status, &log);
		std::cout << "read_result: " << read_result << std::endl;
		std::cout << "status2: " << status << std::endl;
		if(read_result == false){
				// ROS_ERROR("%s", log);
				// ROS_ERROR("Failed to Read Hardware Error Status from Motor ID : %d]", dxl.second);
				// return false;
				bool result = dxl_wb->reboot((uint8_t)dxl.second, &log);
				// if (result == false)
				// {
				// 	ROS_ERROR("%s", log);
				// 	ROS_ERROR("Failed to Reboot Dynamixel Motor ID : %d]", dxl.second);
				// 	return false;
				// }
		}
		else{
			// if(status == 1){
			// 	bool result = dxl_wb->reboot((uint8_t)dxl.second, &log);
			// 	if (result == false)
			// 	{
			// 		ROS_ERROR("%s", log);
			// 		ROS_ERROR("Failed to Reboot Dynamixel Motor ID : %d]", dxl.second);
			// 		return false;
			// 	}
			// }
		}
	}
	res.success = true;
	res.message = "All Hardware Error Status is OK!";
	return true;
}

bool PositionController::dynamixelTorqueOffCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) 
{
	bool result = false;
	const char* log;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel)
	{
		int32_t data;
		bool result = dxl_wb->torqueOff((uint8_t) dxl.second, &log);
		if(result == false){
			res.success = false;
			res.message = "Failed to Torque Off Dynamixel Motor";
			ROS_ERROR("%s", log);
			ROS_ERROR("Failed to Torque Off Dynamixel Motor ID : %d]", dxl.second);
			// return false;
		}
	}
	res.success = true;
	res.message = "All Motor Torque are Off!";
	return true;
}

bool PositionController::dynamixelTorqueOnCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) 
{
	bool result = false;
	const char* log;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel)
	{
		int32_t data;
		bool result = dxl_wb->torqueOn((uint8_t) dxl.second, &log);
		if(result == false){
			res.success = false;
			res.message = "Failed to Torque Off Dynamixel Motor";
			ROS_ERROR("%s", log);
			ROS_ERROR("Failed to Torque Off Dynamixel Motor ID : %d]", dxl.second);
			// return false;
		}
	}
	res.success = true;
	res.message = "All Motor Torque are Off!";
	return true;
}

bool PositionController::RebootMotors() 
{
	// Check shutdown address: 63, size: 1 byte, Data Name: 'Shutdown', initial value: 52
	// Then reboot
	bool result = false;
	const char* log;

	// dynamixel = map(name, motor_ID) 
	for (std::pair<std::string, uint32_t> const& dxl : dynamixel)
	{
		std::cout << "dxl.second: " << dxl.second << std::endl;
		int32_t status;
		// int32_t status;
		std::cout << "status1: " << status << std::endl;
		bool read_result = dxl_wb->itemRead((uint8_t)dxl.second, "Hardware_Error_Status", &status, &log);
		std::cout << "read_result: " << read_result << std::endl;
		std::cout << "status2: " << status << std::endl;
		if(read_result == false){
				// ROS_ERROR("%s", log);
				// ROS_ERROR("Failed to Read Hardware Error Status from Motor ID : %d]", dxl.second);
				// return false;
				bool result = dxl_wb->reboot((uint8_t)dxl.second, &log);
				// if (result == false)
				// {
				// 	ROS_ERROR("%s", log);
				// 	ROS_ERROR("Failed to Reboot Dynamixel Motor ID : %d]", dxl.second);
				// 	return false;
				// }
		}
		else{
			// if(status == 1){
			// 	bool result = dxl_wb->reboot((uint8_t)dxl.second, &log);
			// 	if (result == false)
			// 	{
			// 		ROS_ERROR("%s", log);
			// 		ROS_ERROR("Failed to Reboot Dynamixel Motor ID : %d]", dxl.second);
			// 		return false;
			// 	}
			// }
		}
	}
	return true;
}


//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-


int main(int argc, char** argv)
{
	ros::init(argc, argv, "db_dynamixel_ROS_driver");
	ros::NodeHandle node_handle("");
	
	std::string port_name = "/dev/ttyUSB0";
	uint32_t baud_rate = 57600;

	if (argc < 2) 
	{
		ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
		return 0;
	}
	else
	{
		port_name = argv[1];
	    baud_rate = atoi(argv[2]);
	}
	
	PositionController position_controller;
	
	bool result = false;
	
	std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

	result = position_controller.initWorkbench(port_name, baud_rate);
	if (result == false) 
	{
		ROS_ERROR("Please check USB port name");
		return 0;
	}

	position_controller.initMsg();

	// Get Dynamixels from YAML file
	result = position_controller.getDynamixelsInfo(yaml_file);
	if (result == false) 
	{
 	   ROS_ERROR("Please check YAML file");
 	   return 0;
	}

	// Load Dynamixels into the program
	// result = position_controller.loadDynamixels();
	// if (result == false) 
	// {
	// 	ROS_ERROR("Please check Dynamixel ID or BaudRate");
	// 	return 0;
	// }

	// Reboot Motors
	result = position_controller.RebootMotors();
	if (result == false)
	{
		ROS_ERROR("Failed to Reboot Motors");
		return 0;
	}

	// Initialize dynamixels
	// result = position_controller.initDynamixels();
	// if (result == false)
	// {
	// 	ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
	// 	return 0;
	// }

	// Initialize control parameters
	result = position_controller.initControlItems();
	if (result == false) 
	{
		ROS_ERROR("Please check control items");
		return 0;
	}

	// Initialize SDK handlers
	// result = position_controller.initSDKHandlers();
	// if (result == false) 
	// {
	// 	ROS_ERROR("Failed to set Dynamixel SDK Handler");
	// 	return 0;
	// }
	
	// Sleep for 5 seconds to let the system set nicely
	// ros::Duration(4).sleep();

	// Set robot in home position
//	result = position_controller.initHomePosition();
//	if (result == false)
//	{
//		ROS_ERROR("Failed to set Dung Beetle in Home Position");
//		return 0;
//	}

	// Initialize ROS network
	// position_controller.initPublisher();
	// position_controller.initSubscriber();
	// position_controller.initServer();
	
	// Write and Read from Topics
	// ros::Timer read_timer = node_handle.createTimer(ros::Duration(position_controller.getReadPeriod()), &PositionController::readCallback, &position_controller);
	// ros::Timer write_timer = node_handle.createTimer(ros::Duration(position_controller.getWritePeriod()), &PositionController::writeCallback, &position_controller);
	//ros::Timer write_timer = node_handle.createTimer(ros::Duration(position_controller.getWritePeriod()), &PositionController::writeMultiCallback, &position_controller);
	// ros::Timer publish_timer = node_handle.createTimer(ros::Duration(position_controller.getPublishPeriod()), &PositionController::publishCallback, &position_controller);

	// ros::spin();

	return 0;
}	
