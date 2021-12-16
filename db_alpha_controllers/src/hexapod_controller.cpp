//
// Created by Carlos Viescas Huerta on May 2019
//

#include "db_alpha_controllers/hexapod_controller.h"

// Constructur
HexapodController::HexapodController() : node_handle(""), priv_node_handle("~"), has_joint_state(false) 
{
	read_period = priv_node_handle.param<double>("dxl_read_period", 0.010f);
	write_period = priv_node_handle.param<double>("dxl_write_period", 0.010f);
	publish_period = priv_node_handle.param<double>("publish_period", 0.010f);
	dxl_wb = new DynamixelWorkbench;
}


// Destructor
HexapodController::~HexapodController()
{
	ROS_INFO("Torque Disable");

	const char* log;

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
//						return false;
					}
				}
			}
		}
		dxl_wb->torqueOff((uint8_t)dxl.second);
	}
}


//---------------------------------------------------------------------------------------------------


// Dynamixel Workbench initialization
bool HexapodController::initWorkbench(const std::string port_name, const uint32_t baud_rate) 
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


// YAML reading
bool HexapodController::getDynamixelsInfo(const std::string yaml_file)
{
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


// Load Dynamixel Servos into the ROS network
bool HexapodController::loadDynamixels()
{
	bool result = false;
	const char* log;
	int motor_cnt = 0;

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
		else ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
		motor_cnt++;
		joint_identification.push_back((uint8_t)dxl.second);
	}
	ROS_INFO("Number of joints : %d", motor_cnt);
	return result;
}


// Welcome message
void HexapodController::initMsg()
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
	printf("*******         ARTICULATED HEXAPOD CONTROLLER         *******");
	printf("\n");
	printf("--------------------------------------------------------------------------\n");
	printf("\n");
}


// Initialize motors
bool HexapodController::initDynamixels() 
{
	ROS_INFO("Torque Enabled");
	
	const char* log;

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


// Here is where I need to set up some motors to be controlled by torque and some others by position
bool HexapodController::initControlItems()
{
	bool result = false;
	const char* log = NULL;

    const ControlItem* goal_position;
    const ControlItem* goal_current;
	const ControlItem* current_limit;

	uint32_t dxl_num = dynamixel.begin()->second;

    goal_position = dxl_wb->getItemInfo(dxl_num, "Goal_Position");
    if (goal_position == NULL) return false;
   
    goal_current = dxl_wb->getItemInfo(dxl_num, "Goal_Current");
    if (goal_current == NULL) return false; 

	current_limit = dxl_wb->getItemInfo(dxl_num, "Current_Limit");
    if (current_limit == NULL) return false; 

	const ControlItem* present_position = dxl_wb->getItemInfo(dxl_num, "Present_Position");
	if (present_position == NULL) return false;

	const ControlItem* present_velocity = dxl_wb->getItemInfo(dxl_num, "Present_Velocity");
	if (present_velocity == NULL) return false;

	const ControlItem* present_current = dxl_wb->getItemInfo(dxl_num, "Present_Current");
	if (present_current == NULL) return false;

	control_items["Goal_Current"] = goal_current;
    control_items["Goal_Position"] = goal_position;
	control_items["Current_Limit"] = current_limit;

	control_items["Present_Position"] = present_position;
	control_items["Present_Velocity"] = present_velocity;
	control_items["Present_Current"] = present_current;

	return true;
}


// Add handlers for goal position and current
bool HexapodController::initSDKHandlers()
{
	bool result = false;
	const char* log = NULL;
	
	// Handler for goal position
    result = dxl_wb->addSyncWriteHandler(control_items["Goal_Position"]->address, control_items["Goal_Position"]->data_length, &log);
    if (result == false)
	{
		ROS_ERROR("%s", log);
		return result;
	}
	else 
	{
		//ROS_INFO("Added sync write handler for goal position.")
		ROS_INFO("Goal position handler: %s", log);
	}

	// Handler for goal current
    result = dxl_wb->addSyncWriteHandler(control_items["Goal_Current"]->address, control_items["Goal_Current"]->data_length, &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
        return result;
    }
    else
    {
		//ROS_INFO("Added sync write handler for goal current.")
        ROS_INFO("Goal current handler: %s", log);
    }

	// Handler for current-limit in current-based position control
	result = dxl_wb->addSyncWriteHandler(control_items["Current_Limit"]->address, control_items["Current_Limit"]->data_length, &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
        return result;
    }
    else
    {
		//ROS_INFO("Added sync write handler for current limit.")
        ROS_INFO("Current limit handler: %s", log);
    }

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
bool HexapodController::initHomePosition()
{
	bool result = false;
	const char* log = NULL;
	
	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

	int32_t dynamixel_position[joint_identification.size()];
	
	for (int index = 0; index < joint_identification.size(); index++)
	{ 
		id_array[index] = joint_identification[index];
		//dynamixel_position[index] = dxl_wb->convertRadian2Value(joint_identification[index], home_position[index]); // Static Experiments
		dynamixel_position[index] = dxl_wb->convertRadian2Value(joint_identification[index], dung_beetle_pose[index]); // Walking Experiments
		id_cnt++;
	}

	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
	ROS_INFO("Robot set to Home Position Successfully.");
	return result;
}


void HexapodController::initPublisher()
{
	dynamixel_state_list_pub = priv_node_handle.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_states_list", 100);
	joint_states_pub = priv_node_handle.advertise<sensor_msgs::JointState>("hexapod_joint_feedback", 100);
	joint_IDs_pub = priv_node_handle.advertise<std_msgs::Int32MultiArray>("hexapod_joint_IDs", 100);
}


void HexapodController::initSubscriber()
{
	goal_joint_state_sub = priv_node_handle.subscribe("hexapod_state_commands", 100, &HexapodController::onJointStateGoal, this);
	//multi_joint_goal_sub = priv_node_handle.subscribe("hexapod_multi_joint_commands", 100, &HexapodController::multiJointGoal, this);
}

void HexapodController::initServer()
{
	dynamixel_command_server = priv_node_handle.advertiseService("dynamixel_request_commands", &HexapodController::dynamixelCommandMsgCallback, this);
}


void HexapodController::onJointStateGoal(const sensor_msgs::JointState& msg)
{
	goal_state = msg;
	has_joint_state = true;
}

void HexapodController::multiJointGoal(const std_msgs::Float32MultiArray& msg)
{
	joint_configuration = msg.data;
}


// This callback reads data from the motors
void HexapodController::readCallback(const ros::TimerEvent&)
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


// This callback write data to motors
void HexapodController::writeCallback(const ros::TimerEvent& t)
{
	if (has_joint_state == false) return;
	
	bool result = false;
	const char* log = NULL;
	
    // Robot global
	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

    // Split into position and torque control
    int total_joints = 21;
    int tau_joints = 18; // 12 = CF & FT || 18 = Full leg
    int pos_joints = total_joints - tau_joints;
    
	uint8_t id_current_count = 0;
    uint8_t id_pos_count = 0;
	
	uint8_t id_current_array[tau_joints];  
    uint8_t id_pos_array[pos_joints];
    
	int32_t dynamixel_current[tau_joints];
    int32_t dynamixel_position[pos_joints];
	int32_t dynamixel_alpha[tau_joints];

	for (std::string name : goal_state.name)
	{
		// std::cout << "name: " << name << endl;
		// std::cout << "id_array[id_cnt]: " << id_array[id_cnt] << endl;
		// std::cout << "id_cnt: " << id_cnt << endl;
		id_array[id_cnt] = (uint8_t) dynamixel[name];
		id_cnt++;
	}
	
	for (uint8_t index = 0; index < id_cnt; index++)
	{ 
		// std::cout << "index: " << index << endl;
		// std::cout << "id_array[index]: " << id_array[index] << endl;
		// if(id_array[index] == 11 || id_array[index] == 21 || id_array[index] == 31 || id_array[index] == 41 || id_array[index] == 51 || id_array[index] == 61 || id_array[index] == 71 || id_array[index] == 72 || id_array[index] == 73)
		//if(id_array[index] == 11 || id_array[index] == 21 || id_array[index] == 31 || id_array[index] == 41 || id_array[index] == 71 || id_array[index] == 72 || id_array[index] == 73)
		if(id_array[index] == 71 || id_array[index] == 72 || id_array[index] == 73) // Full leg compliance
        {
            dynamixel_position[id_pos_count] = dxl_wb->convertRadian2Value(id_array[index], goal_state.position[index]);
            id_pos_array[id_pos_count] = id_array[index];
            id_pos_count++;
        }
        else
        {
            // dynamixel_position[id_pos_count] = dxl_wb->convertRadian2Value(id_array[index], goal_state.position[index]);
            // id_pos_array[id_pos_count] = id_array[index];
            // id_pos_count++;
            dynamixel_current[id_current_count] = dxl_wb->convertCurrent2Value(goal_state.effort[index]);
			dynamixel_alpha[id_current_count] = dxl_wb->convertRadian2Value(id_array[index], goal_state.position[index]);
            id_current_array[id_current_count] = id_array[index];
            id_current_count++;
            // std::cout << "pos value: " << dynamixel_position[id_pos_count] << endl;
            // std::cout << "effort value: " << dynamixel_current[id_current_count] << endl;
        }   
	}

	// -----------------------
	// SET GOAL CURRENT -> Compliant joints
	//result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_current_array, id_current_count, dynamixel_current, 1, &log);
	//if (result == false) ROS_ERROR("%s", log);
	//------------------------

	//------------------------
	// SET GOAL POSITION + LIMITED TORQUE -> Compliant joints
	// result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_CURRENT_LIMIT, id_current_array, id_current_count, dynamixel_current, 1, &log);
	// if (result == false) ROS_ERROR("%s", log);

	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_current_array, id_current_count, dynamixel_alpha, 1, &log);
	if (result == false) ROS_ERROR("%s", log);
	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_current_array, id_current_count, dynamixel_current, 1, &log);
	if (result == false) ROS_ERROR("%s", log);
	//------------------------

	//------------------------
	// SET GOAL POSITION -> Non-compliant joints
    result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_pos_array, id_pos_count, dynamixel_position, 1, &log);
	if (result == false) ROS_ERROR("%s", log);
	//------------------------

	ROS_INFO("Position updated in the following motors: %d", id_pos_count);
	ROS_INFO("Torque updated in the following motors: %d", id_current_count);
	
	has_joint_state = false;
}


// Multi joint motor values
void HexapodController::writeMultiCallback(const ros::TimerEvent&)
{
	bool result = false;
	const char* log = NULL;

	// Split into position and torque control
    int total_joints = 21;
    int tau_joints = 18; // 12 = CF & FT || 18 = Full leg
    int pos_joints = total_joints - tau_joints;
    
	uint8_t id_current_count = 0; // counter
    uint8_t id_pos_count = 0; // counter
	
	uint8_t id_current_array[tau_joints]; // motor_id_storage  
    uint8_t id_pos_array[pos_joints]; // motor_id_storage
    
	int32_t dynamixel_current[tau_joints]; // motor_value_storage
    int32_t dynamixel_position[pos_joints]; // motor_value_storage
	
	// Get control values
	for(int index=0; index<joint_configuration.size(); index+=2)
	{
		int motor_id = (int) joint_configuration[index];
		if(motor_id == 11 || motor_id == 21 || motor_id == 31 || motor_id == 41 || motor_id == 51 || motor_id == 61 || motor_id == 71 || motor_id == 72 || motor_id == 73) // CF & FT compliance
		//if(motor_id == 71 || motor_id == 72 || motor_id == 73) // Full leg compliance
		{
			dynamixel_position[id_pos_count] = dxl_wb->convertRadian2Value(motor_id, joint_configuration[index+1]);
			id_pos_array[id_pos_count] = motor_id;
			id_pos_count++;
		}
		else
		{
			dynamixel_current[id_current_count] = dxl_wb->convertCurrent2Value(joint_configuration[index+1]);
            id_current_array[id_current_count] = motor_id;
            id_current_count++;
		}	
	}
	
	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_current_array, id_current_count, dynamixel_current, 1, &log);
	if (result == false) ROS_ERROR("FAILED TO SET MOTOR TORQUES");

    result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_pos_array, id_pos_count, dynamixel_position, 1, &log);
	if (result == false) ROS_ERROR("FAILED TO SET MOTOR POSITIONS");

}


// This callback publishes feedback into the ROS network
void HexapodController::publishCallback(const ros::TimerEvent&)
{
	// Publish dynamixale state list
	dynamixel_state_list_pub.publish(dynamixel_state_list);
	
	// Publish hexapod joint IDs
	std_msgs::Int32MultiArray array_IDs;
	array_IDs.data.clear();
	
	for (size_t i=0; i<joint_identification.size(); i++)
	{
		array_IDs.data.push_back(joint_identification[i]);
	}

	joint_IDs_pub.publish(array_IDs);

	// Publish hexapod joint states 
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


// This one handles the ROS service
bool HexapodController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res)
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
	
	HexapodController hexapod_controller;
	
	bool result = false;
	
	std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

	result = hexapod_controller.initWorkbench(port_name, baud_rate);
	if (result == false)
	{
		ROS_ERROR("Please check USB port name");
		return 0;
	}

	hexapod_controller.initMsg();

	result = hexapod_controller.getDynamixelsInfo(yaml_file);
	if (result == false)
	{
 	   ROS_ERROR("Please check YAML file");
 	   return 0;
	}

	result = hexapod_controller.loadDynamixels();
	if (result == false)
	{
		ROS_ERROR("Please check Dynamixel ID or BaudRate");
		return 0;
	}

	result = hexapod_controller.initDynamixels();
	if (result == false)
	{
		ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
		return 0;
	}

	result = hexapod_controller.initControlItems();
	if (result == false)
	{
		ROS_ERROR("Please check control items");
		return 0;
	}

	result = hexapod_controller.initSDKHandlers();
	if (result == false)
	{
		ROS_ERROR("Failed to set Dynamixel SDK Handler");
		return 0;
	}

	// Sleep for 5 seconds to let the system set nicely
	ros::Duration(5).sleep();
	
	// Set robot in home position
	// result = hexapod_controller.initHomePosition();
	// if (result == false) 
	// {
	// 	ROS_ERROR("Failed to set Dung Beetle in Home Position");
	// 	return 0;
	// }

	hexapod_controller.initPublisher();
	hexapod_controller.initSubscriber();
	hexapod_controller.initServer();
	
	ros::Timer read_timer = node_handle.createTimer(ros::Duration(hexapod_controller.getReadPeriod()), &HexapodController::readCallback, &hexapod_controller);
	ros::Timer write_timer = node_handle.createTimer(ros::Duration(hexapod_controller.getWritePeriod()), &HexapodController::writeCallback, &hexapod_controller);
	//ros::Timer write_timer = node_handle.createTimer(ros::Duration(hexapod_controller.getWritePeriod()), &HexapodController::writeMultiCallback, &hexapod_controller);
	ros::Timer publish_timer = node_handle.createTimer(ros::Duration(hexapod_controller.getPublishPeriod()), &HexapodController::publishCallback, &hexapod_controller);

	ros::spin();

	return 0;
}	
