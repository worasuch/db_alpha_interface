# DYNAMIXEL INTERFACE FOR ROS CONTROL OF THE DB-ALPHA ROBOT

Developed as part of my Masters Thesis: 

 - **Adaptive Compliance Control of a Dung Beetle Robot**.
 - Academic year 2018-019.
 - Carlos Viescas Huerta.
 - University of Southern Denmark.
 
## Installation guide:

This controller uses the latest version of the Dynamixel Workbench driver (May 2019). The following instructions are meant for Ubuntu 18 and ROS Melodic.

Install Main packages:

```sh
$ cd catkin_ws/src
$ git clone https://github.com/CVH95/db_alpha_interface.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
``` 

Install dependent packages:

```sh
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ sudo apt-get install ros-melodic-moveit-core ros-melodic-moveit-ros-planning ros-melodic-moveit-ros-planning-interface
```

Compile and build the pkgs:

```sh
$ cd ../
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

#### Official documentation:

 - **Install:** Go to [Official Dynamixel Workbench Manual](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/).


 - **Online Motor Control Table:** Go to [ROBOTIS e-manual Dynamixel XM430-W350](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#indirect-data).

 - **Online Motor Datasheet:** Go to [ROBOTIS XM430-W350 support](http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm).

## Configuration

 Before starting the driver, you need to configure the setup, including motor IDs, operating mode and return delay time. It is also possible to set up gains and acceleration and velocity profiles from the YAM file profile. See examples of configuration files in _/db\_alpha\_controllers/config/_. 

 ```yaml
# Example of YAML configuration file

id_1:
  ID: 11
  Return_Delay_Time: 0
  Operating_Mode: 3
  Position_P_Gain: 400
  Position_I_Gain: 200
  Position_D_Gain: 50
  Profile_Velocity: 200
  Profile_Acceleration: 50
```

 #### Operating modes

 - Caption from the XM430-W350 datasheet:

![Table of operating modes](db_alpha_controllers/docs/opmds.png)

## Torque control: torque-current linear relationship:

```c
// Current unit A || Torque unit N*m
float m = 0.5828571429;
float n = 0.072;
float current_A = torque_Nm*m + n;

// Dynamixel current unit mA (This is the value to send to the ROS driver)
float dxl_current = current_A*1000;

// Dynamixel converts current values in mA to motor values with the following relationship:
float CURRENT_UNIT = 2.69;
int16_t value = dxl_current / CURRENT_UNIT;
```

## Running Controllers:

There are 3 different drivers:

 - **position\_controller:** All joints (18 leg joints + 3 body joints) are controlled by position.
 - **torque\_controller:** All joints (18 leg joints + 3 body joints) are controlled by torque.
 - **hexapod\_controller:** CF and FT joints are controlled by torque, while TC and body joints are controlled by position.

To start the any of the driver interfaces (for example, position control interface) run:

```sh
$ roslaunch db_alpha_controllers hexapod_controller.launch # You should see this message: 

--------------------------------------------------------------------------

  ____                              _          _  
 |  _ \ _   _ _ __   __ _ _ __ ___ (_)_  _____| | 
 | | | | | | | '_ \ / _` | '_ ` _ \| \ \/ / _ \ | 
 | |_| | |_| | | | | (_| | | | | | | |>  <  __/ | 
 |____/ \__, |_| |_|\__,_|_| |_| |_|_/_/\_\___|_| 
  ____  |___/ ____    ____       _                
 |  _ \ / _ \/ ___|  |  _ \ _ __(_)_   _____ _ __ 
 | |_) | | | \___ \  | | | | '__| \ \ / / _ \ '__|
 |  _ <| |_| |___) | | |_| | |  | |\ V /  __/ |   
 |_| \_\\___/|____/  |____/|_|  |_| \_/ \___|_|   
                                                  
--------------------------------------------------------------------------

*******         ARTICULATED HEXAPOD CONTROLLER         *******
--------------------------------------------------------------------------

[ INFO] [1557155678.276073874]: Name : id_1, ID : 11, Model Number : 1020
[ INFO] [1557155678.323977129]: Name : id_10, ID : 41, Model Number : 1020
[ INFO] [1557155678.371927178]: Name : id_11, ID : 42, Model Number : 1020
[ INFO] [1557155678.419987613]: Name : id_12, ID : 43, Model Number : 1020
[ INFO] [1557155678.467984144]: Name : id_13, ID : 51, Model Number : 1020
[ INFO] [1557155678.515904544]: Name : id_14, ID : 52, Model Number : 1020
[ INFO] [1557155678.563932259]: Name : id_15, ID : 53, Model Number : 1020
[ INFO] [1557155678.611915653]: Name : id_16, ID : 61, Model Number : 1020
[ INFO] [1557155678.659938580]: Name : id_17, ID : 62, Model Number : 1020
[ INFO] [1557155678.707924358]: Name : id_18, ID : 63, Model Number : 1020
[ INFO] [1557155678.755989134]: Name : id_19, ID : 71, Model Number : 1020
[ INFO] [1557155678.803929516]: Name : id_2, ID : 12, Model Number : 1020
[ INFO] [1557155678.851929866]: Name : id_20, ID : 72, Model Number : 1020
[ INFO] [1557155678.899937067]: Name : id_21, ID : 73, Model Number : 1020
[ INFO] [1557155678.947948921]: Name : id_3, ID : 13, Model Number : 1020
[ INFO] [1557155678.995950465]: Name : id_4, ID : 21, Model Number : 1020
[ INFO] [1557155679.043964742]: Name : id_5, ID : 22, Model Number : 1020
[ INFO] [1557155679.091940352]: Name : id_6, ID : 23, Model Number : 1020
[ INFO] [1557155679.139916452]: Name : id_7, ID : 31, Model Number : 1020
[ INFO] [1557155679.187937323]: Name : id_8, ID : 32, Model Number : 1020
[ INFO] [1557155679.235959423]: Name : id_9, ID : 33, Model Number : 1020
[ INFO] [1557155679.236021475]: Torque Enabled
[ INFO] [1557155680.867937731]: [DynamixelDriver] Succeeded to add sync write handler
[ INFO] [1557155680.868010602]: [DynamixelDriver] Succeeded to add sync write handler

```

After 5 seconds, the robot should move into **home position** and the ROS network should be initialized. Check the rostopic list:

```sh
$ rostopic list
/dynamixel_ROS_driver_hexapod_controller/dynamixel_state
/dynamixel_ROS_driver_hexapod_controller/hexapod_command
/dynamixel_ROS_driver_hexapod_controller/hexapod_joint_IDs
/dynamixel_ROS_driver_hexapod_controller/hexapod_states
/rosout
/rosout_agg
```

#### Get feedback from the motors:

Check the formats in which feedback from the motors is broadcasted:

```sh
$ rostopic echo /dynamixel_ROS_driver_hexapod_controller/hexapod_joint_IDs 

layout: 
  dim: []
  data_offset: 0
data: [11, 41, 42, 43, 51, 52, 53, 61, 62, 63, 71, 12, 72, 73, 13, 21, 22, 23, 31, 32, 33]


$ rostopic echo /dynamixel_ROS_driver_hexapod_controller/hexapod_states 

header: 
  seq: 1460
  stamp: 
    secs: 1557155704
    nsecs: 258696638
  frame_id: ''
name: [id_1, id_10, id_11, id_12, id_13, id_14, id_15, id_16, id_17, id_18, id_19, id_2,
  id_20, id_21, id_3, id_4, id_5, id_6, id_7, id_8, id_9]
position: [-0.4111068546772003, 0.5829127430915833, 0.6412039995193481, 0.015339808538556099, 0.17947575449943542, 0.8881748914718628, 0.3037281930446625, -0.34821364283561707, 0.46172821521759033, 0.7102331519126892, -0.22396120429039001, 0.40957286953926086, -0.24236896634101868, 0.09664079546928406, 0.43104860186576843, 0.30833014845848083, 0.5721748471260071, -0.1702718734741211, -0.44332045316696167, 0.6979612708091736, 0.015339808538556099]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, -2.690000057220459, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5.380000114440918, 0.0, -2.690000057220459, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

$ rostopic echo /dynamixel_ROS_driver_hexapod_controller/dynamixel_state 

dynamixel_state: 
  - 
    name: "id_1"
    id: 11
    present_position: 1780
    present_velocity: 0
    present_current: 0
  - 
    name: "id_10"
    id: 41
    present_position: 2428
    present_velocity: 0
    present_current: 1
  - 
    name: "id_11"
    id: 42
    present_position: 2466
    present_velocity: 0
    present_current: 0
  - 
    #...
```

**NOTE:**

Before running any of the above code, the **USB latency between the computer and the robot** must be lowered. Run the following terminal commands to resolve this:

```sh
# Option A (recommended)
$ sudo usermod -aG dialout $USER && echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Option B
$ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
$ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger --action=add
```

Then verify that the latency has been set to 1 with the following command:

```sh
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Finally, update the permissions so that the serial connection can actually send and receive values:

```sh
$ sudo chmod a+rw /dev/ttyUSB0
```

See [Mathias' guide](https://github.com/MathiasThor/my_dynamixel_workbench/wiki/MORF-Software-Installation-Guide) for setting the automatic start of the of the serial port. 

## Listener

The package __**db\_listener**__ can be used to manually learn trajectories. Set the **torque\_controller** driver to loose robot joints:

```sh
$ roslaunch db_alpha_controllers torque_controller.launch
```

 Then run the listener and move the robot's legs manually. Joint positions will be recorded into a **.csv** file:

 ```sh
$ rosrun db_alpha_listener joint_position_listener
 ```
