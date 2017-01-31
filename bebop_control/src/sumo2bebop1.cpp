/*
  _    _ _   _ _____    _    _    __      __  _               ____  
 | |  | | \ | |  __ \  | |  | |  /\ \    / / | |        /\   |  _ \ 
 | |  | |  \| | |__) | | |  | | /  \ \  / /  | |       /  \  | |_) |
 | |  | | . ` |  _  /  | |  | |/ /\ \ \/ /   | |      / /\ \ |  _ < 
 | |__| | |\  | | \ \  | |__| / ____ \  /    | |____ / ____ \| |_) |
  \____/|_| \_|_|  \_\  \____/_/    \_\/     |______/_/    \_\____/ 
                                                                    

                              __
                            .d$$b
                          .' TO$;\
                         /  : TP._;
                        / _.;  :Tb|
                       /   /   ;j$j
                   _.-"       d$$$$
                 .' ..       d$$$$;
                /  /P'      d$$$$P. |\
               /   "      .d$$$P' |\^"l
             .'           `T$P^"""""  :
         ._.'      _.'                ;
      `-.-".-'-' ._.       _.-"    .-"
    `.-" _____  ._              .-"
   -(.g$$$$$$$b.              .'
     ""^^T$$$P^)            .(:
       _/  -"  /.'         /:/;
    ._.'-'`-'  ")/         /;/;
 `-.-"..--""   " /         /  ;
.-" ..--""        -'          :
..--""--.-"         (\      .-(\
  ..--""              `-\(\/;`
    _.                      :
                            ;`-
                           :\
                           ;

Written by Ric Fehr
*/

// Set up two drones at different heights and opposite circle directions

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <fstream>
using namespace std;

#define PI 3.141592653589793238462
// Struct for states data
struct states
{
	// Translational position in meters
	double pos_x, pos_y, pos_z;
	// Orientation position in quaternions
	double ori_x, ori_y, ori_z, ori_w;
	// Orientation position in euler angles
	double roll, pitch, yaw, yaw_RAD;
	// Error values for controller from sumo
	double error_sumo_x, error_sumo_y, error_sumo_z, error_sumo_yaw;
	// Approx. velocity error values from sumo
	double velError_sumo_x, velError_sumo_y, velError_sumo_z, velError_sumo_yaw;
	// Approx. abstition error values from sumo
	double absitionError_sumo_x, absitionError_sumo_y, absitionError_sumo_z, absitionError_sumo_yaw;
	// Error values for controller from drone A
	double error_droneA_x, error_droneA_y, error_droneA_z, error_droneA_yaw;
	// Approx. velocity error values from drone A
	double velError_droneA_x, velError_droneA_y, velError_droneA_z, velError_droneA_yaw;
	// Approx. abstition error values from drone A
	double absitionError_droneA_x, absitionError_droneA_y, absitionError_droneA_z, absitionError_droneA_yaw;
	// Error values for controller from drone B
	double error_droneB_x, error_droneB_y, error_droneB_z, error_droneB_yaw;
	// Approx. velocity error values from drone B
	double velError_droneB_x, velError_droneB_y, velError_droneB_z, velError_droneB_yaw;
	// Approx. abstition error values from drone B
	double absitionError_droneB_x, absitionError_droneB_y, absitionError_droneB_z, absitionError_droneB_yaw;
	// Temp. variables to hold previous position for velocity calculation
	double prevPos_x, prevPos_y, prevPos_z, prevPos_yaw; 
	// Temp. variables to suppress cmd_vel_bebop1_ output to [-1,1]
	double cmd_x, cmd_y, cmd_z, cmd_yaw;
	// Rotation conversion variables for yaw orientation 
	double rot_cmd_x, rot_cmd_y;
	// Sum of all position errors for controller
	double total_error_x, total_error_y;
} bebop1, bebop2, bebop3, desired1, desired2, desired3, sumo;

int button_1;
int button_2;
int button_3;

// Get joystick controller info
void getJoy(const sensor_msgs::Joy::ConstPtr& button)
{
	button_1 = button->buttons[0];
	button_2 = button->buttons[1];
	button_3 = button->buttons[2];
}

int isTakeoff;
int isLand;
bool altYawControl = true;

// Get position info from mocap for bebop1
void getPosBebop1(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop1.pos_x = pos->pose.position.x;
	bebop1.pos_y = pos->pose.position.y;
	bebop1.pos_z = pos->pose.position.z;
	bebop1.ori_x = pos->pose.orientation.x;
	bebop1.ori_y = pos->pose.orientation.y;
	bebop1.ori_z = pos->pose.orientation.z;
	bebop1.ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop1.roll, bebop1.pitch, bebop1.yaw_RAD);
	bebop1.yaw = bebop1.yaw_RAD*(180/PI);
}

void getPosBebop2(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop2.pos_x = pos->pose.position.x;
	bebop2.pos_y = pos->pose.position.y;
	bebop2.pos_z = pos->pose.position.z;
	bebop2.ori_x = pos->pose.orientation.x;
	bebop2.ori_y = pos->pose.orientation.y;
	bebop2.ori_z = pos->pose.orientation.z;
	bebop2.ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop2.roll, bebop2.pitch, bebop2.yaw_RAD);
	bebop2.yaw = bebop2.yaw_RAD*(180/PI);
}

void getPosBebop3(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop3.pos_x = pos->pose.position.x;
	bebop3.pos_y = pos->pose.position.y;
	bebop3.pos_z = pos->pose.position.z;
	bebop3.ori_x = pos->pose.orientation.x;
	bebop3.ori_y = pos->pose.orientation.y;
	bebop3.ori_z = pos->pose.orientation.z;
	bebop3.ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop3.roll, bebop3.pitch, bebop3.yaw_RAD);
	bebop3.yaw = bebop3.yaw_RAD*(180/PI);
}

// Get position info from mocap for sumo
void getPosSumo(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	sumo.pos_x = pos->pose.position.x;
	sumo.pos_y = pos->pose.position.y;
	sumo.pos_z = pos->pose.position.z;
	sumo.ori_x = pos->pose.orientation.x;
	sumo.ori_y = pos->pose.orientation.y;
	sumo.ori_z = pos->pose.orientation.z;
	sumo.ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(sumo.roll, sumo.pitch, sumo.yaw_RAD);
	sumo.yaw = sumo.yaw_RAD*(180/PI);
}

// Get position error for proportional control
double getPosError(double desPos, double actualPos)
{
	return desPos-actualPos;
}

// Get velocity error for derivative control
float getVel(double previousPos, double currentPos, float h)
{
	return (currentPos-previousPos)/h;
}

// Get absition using Simpsons method
float getAbsition(double a, double b, double previousPos, double currentPos)
{
	double s = (previousPos+currentPos)/2;
	double q = (b-a)/6;
	return q*(currentPos+4*s+previousPos); 
}

int main(int argc, char** argv)
{
	bool translationControllerOn = true;
	bool orientationControllerOn = true;
	float myTime = 0.0;
	// Gains for PID controller ##FAST##
	float kp_xy = 0.5; //0.85
	float kp_xy_multi = 0.0; //0.1
	float kd_xy = 0.625; //0.625
	float ki_xy = 0.6; //0.6
	float kp_yaw = 0.05; //0.05
	float kd_yaw = 0.01; //0.01
	float ki_yaw = 0.01; //0.01
	float kp_z = 1.2; //1.2
	float kd_z = 0.1; //0.1
	float ki_z = 0.4; //0.4
	// Speed at which aircraft will translate along axis
	float speed = 0.99;
	// Desired positions of bebops relative to eachother
	// For bebop1
	double des_pos_x_1to2, des_pos_x_1to3;
	double des_pos_y_1to2, des_pos_y_1to3;
	// For bebop2
	double des_pos_x_2to1, des_pos_x_2to3;
	double des_pos_y_2to1, des_pos_y_2to3;
	// For bebop3
	double des_pos_x_3to1, des_pos_x_3to2;
	double des_pos_y_3to1, des_pos_y_3to2;
	// std_msgs::Empty "takeoff" & "land"
	std_msgs::Empty msg_takeoff, msg_land; 

	// Initialize ros
	ros::init(argc, argv, "bebop_control"); 

	// Node handle used
	ros::NodeHandle nh_; 
	
	// Initialized publishers
	// For bebop1
	ros::Publisher takeoff_pub_bebop1_;
	ros::Publisher land_pub_bebop1_;	
	ros::Publisher cmd_vel_pub_bebop1_;
	// For bebop2
	ros::Publisher takeoff_pub_bebop2_;
	ros::Publisher land_pub_bebop2_;	
	ros::Publisher cmd_vel_pub_bebop2_;
	// For bebop3
	ros::Publisher takeoff_pub_bebop3_;
	ros::Publisher land_pub_bebop3_;	
	ros::Publisher cmd_vel_pub_bebop3_;

	// Initialize subscribers
	// For bebop1, Drone IP = 5 
	ros::Subscriber subPose1 = nh_.subscribe("/UAV_IP5/pose", 1000, getPosBebop1);
	// For bebop2, Drone IP = 6
	ros::Subscriber subPose2 = nh_.subscribe("/UAV_IP6/pose", 1000, getPosBebop2);
	// For bebop3, Drone IP = 8
	ros::Subscriber subPose3 = nh_.subscribe("/UAV_IP7/pose", 1000, getPosBebop3);
	// For sumo
	ros::Subscriber subPose4 = nh_.subscribe("/UAV_IP8/pose", 1000, getPosSumo);
	
	// Subscribe to joy node
	ros::Subscriber joy_controller = nh_.subscribe("/joy", 1000, getJoy);
	
	takeoff_pub_bebop1_ = nh_.advertise<std_msgs::Empty>("/bebop_IP5/takeoff", 1000);
	land_pub_bebop1_ = nh_.advertise<std_msgs::Empty>("/bebop_IP5/land", 1000);
	cmd_vel_pub_bebop1_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP5/cmd_vel", 1);
	
	takeoff_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/takeoff", 1000);
	land_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/land", 1000);
	cmd_vel_pub_bebop2_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP6/cmd_vel", 1);

	takeoff_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/takeoff", 1000);
	land_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/land", 1000);
	cmd_vel_pub_bebop3_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP8/cmd_vel", 1);

	geometry_msgs::Twist cmd_vel_bebop1_;
	geometry_msgs::Twist cmd_vel_bebop2_;
	geometry_msgs::Twist cmd_vel_bebop3_;

	ros::Rate loop_rate(50);
	float T = 0.02;
	
	/*
	 * Open each .txt file for matlab graph 
	*/ 
	ofstream bebop1Position_x;
	bebop1Position_x.open ("bebop1Position_x.txt");
	ofstream bebop1Position_y;
	bebop1Position_y.open ("bebop1Position_y.txt");
	ofstream bebop1Position_z;
	bebop1Position_z.open ("bebop1Position_z.txt");
	ofstream bebop1Orientation_yaw;
	bebop1Orientation_yaw.open ("bebop1Orientation_yaw.txt");	
	
	ofstream bebop2Position_x;
	bebop2Position_x.open ("bebop2Position_x.txt");
	ofstream bebop2Position_y;
	bebop2Position_y.open ("bebop2Position_y.txt");
	ofstream bebop2Position_z;
	bebop2Position_z.open ("bebop2Position_z.txt");
	ofstream bebop2Orientation_yaw;
	bebop2Orientation_yaw.open ("bebop2Orientation_yaw.txt");	
	
	ofstream bebop3Position_x;
	bebop3Position_x.open ("bebop3Position_x.txt");
	ofstream bebop3Position_y;
	bebop3Position_y.open ("bebop3Position_y.txt");
	ofstream bebop3Position_z;
	bebop3Position_z.open ("bebop3Position_z.txt");
	ofstream bebop3Orientation_yaw;
	bebop3Orientation_yaw.open ("bebop3Orientation_yaw.txt");	

	while(nh_.ok())
	{
		double a = 0.0;
		double b = T;
		
		// Update desired positions based on the orientation of sumo
		// For bebop1
		desired1.pos_z = 1.0;
		desired1.pos_x = sumo.pos_x-1.25*sin(sumo.yaw_RAD);
		desired1.pos_y = sumo.pos_y+1.25*cos(sumo.yaw_RAD);
		// Have bebop1 face sumo
		if (sumo.yaw > 0) {desired1.yaw = sumo.yaw-180;}
		if (sumo.yaw < 0) {desired1.yaw = sumo.yaw+180;}
		// For bebop2
		/*
		desired2.pos_z = 0.75;
		desired2.pos_x = sumo.pos_x-sin(sumo.yaw_RAD);
		desired2.pos_y = sumo.pos_y-cos(sumo.yaw_RAD);
		desired2.yaw = sumo.yaw;
		*/
		desired2.pos_z = 1.0;
		desired2.pos_x = sumo.pos_x+1.25*sin(sumo.yaw_RAD-(PI/3));
		desired2.pos_y = sumo.pos_y-1.25*cos(sumo.yaw_RAD-(PI/3));
		desired2.yaw = sumo.yaw-60;
		if (desired2.yaw < -180) {desired2.yaw = desired2.yaw+360;}
		// For bebop3
		desired3.pos_z = 1.0;
		desired3.pos_x = sumo.pos_x+1.25*sin(sumo.yaw_RAD+(PI/3));
		desired3.pos_y = sumo.pos_y-1.25*cos(sumo.yaw_RAD+(PI/3));
		desired3.yaw = sumo.yaw+60;
		if (desired3.yaw > 180) {desired3.yaw = desired3.yaw-360;}
		
		//Update desired position for distances between drones
		// For bebop1 to bebop2
		des_pos_x_1to2 = bebop2.pos_x+sqrt(3)*sin(sumo.yaw_RAD+(PI/6));
		des_pos_y_1to2 = bebop2.pos_y+sqrt(3)*cos(sumo.yaw_RAD+(PI/6));
		// For bebop1 to bebop3
		des_pos_x_1to3 = bebop3.pos_x-sqrt(3)*sin(sumo.yaw_RAD+(PI/6));
		des_pos_y_1to3 = bebop3.pos_y+sqrt(3)*cos(sumo.yaw_RAD+(PI/6));
		// For bebop2 to bebop1
		des_pos_x_2to1 = bebop1.pos_x-sqrt(3)*sin(sumo.yaw_RAD+(PI/6));
		des_pos_y_2to1 = bebop1.pos_y-sqrt(3)*cos(sumo.yaw_RAD+(PI/6));
		// For bebop2 to bebop3
		des_pos_x_2to3 = bebop3.pos_x-sqrt(3);
		des_pos_y_2to3 = bebop3.pos_y;
		// For bebop3 to bebop1
		des_pos_x_3to1 = bebop1.pos_x+sqrt(3)*sin(sumo.yaw_RAD+(PI/6));
		des_pos_y_3to1 = bebop1.pos_y-sqrt(3)*cos(sumo.yaw_RAD+(PI/6));
		// For bebop3 to bebop2
		des_pos_x_3to2 = bebop2.pos_x+sqrt(3); 
		des_pos_y_3to2 = bebop2.pos_y;

		// Find position error values from ground vehicle
		// For bebop1
		bebop1.error_sumo_x = getPosError(desired1.pos_x, bebop1.pos_x);
		bebop1.error_sumo_y = getPosError(desired1.pos_y, bebop1.pos_y);
		bebop1.error_sumo_z = getPosError(desired1.pos_z, bebop1.pos_z);
		bebop1.error_sumo_yaw = getPosError(desired1.yaw, bebop1.yaw);
		// For bebop2
		bebop2.error_sumo_x = getPosError(desired2.pos_x, bebop2.pos_x);
		bebop2.error_sumo_y = getPosError(desired2.pos_y, bebop2.pos_y);
		bebop2.error_sumo_z = getPosError(desired2.pos_z, bebop2.pos_z);
		bebop2.error_sumo_yaw = getPosError(desired2.yaw, bebop2.yaw);
		// For bebop3
		bebop3.error_sumo_x = getPosError(desired3.pos_x, bebop3.pos_x);
		bebop3.error_sumo_y = getPosError(desired3.pos_y, bebop3.pos_y);
		bebop3.error_sumo_z = getPosError(desired3.pos_z, bebop3.pos_z);
		bebop3.error_sumo_yaw = getPosError(desired3.yaw, bebop3.yaw);
		
		//Find position error values from other bebops
		//For bebop1 to bebop2
		bebop1.error_droneA_x = getPosError(des_pos_x_1to2, bebop1.pos_x);
		bebop1.error_droneA_y = getPosError(des_pos_y_1to2, bebop1.pos_y);
		//For bebop1 to bebop3
		bebop1.error_droneB_x = getPosError(des_pos_x_1to3, bebop1.pos_x);
		bebop1.error_droneB_y = getPosError(des_pos_y_1to3, bebop1.pos_y);
		//For bebop2 to bebop1
		bebop2.error_droneA_x = getPosError(des_pos_x_2to1, bebop2.pos_x);
		bebop2.error_droneA_y = getPosError(des_pos_y_2to1, bebop2.pos_y);
		//For bebop2 to bebop3
		bebop2.error_droneB_x = getPosError(des_pos_x_2to3, bebop2.pos_x);
		bebop2.error_droneB_y = getPosError(des_pos_y_2to3, bebop2.pos_y);
		//For bebop3 to bebop1
		bebop3.error_droneA_x = getPosError(des_pos_x_3to1, bebop3.pos_x);
		bebop3.error_droneA_y = getPosError(des_pos_y_3to1, bebop3.pos_y);
		//For bebop3 to bebop2
		bebop3.error_droneB_x = getPosError(des_pos_x_3to2, bebop3.pos_x);
		bebop3.error_droneB_y = getPosError(des_pos_y_3to2, bebop3.pos_y);
		
		// Account for massive yaw error from 180 to -180 crossover
		// For bebop1
		if (bebop1.error_sumo_yaw > 180) {bebop1.error_sumo_yaw = bebop1.error_sumo_yaw - 360;}
		else if (bebop1.error_sumo_yaw < -180) {bebop1.error_sumo_yaw = bebop1.error_sumo_yaw + 360;}
		else {bebop1.error_sumo_yaw = bebop1.error_sumo_yaw;}
		// For bebop2
		if (bebop2.error_sumo_yaw > 180) {bebop2.error_sumo_yaw = bebop2.error_sumo_yaw - 360;}
		else if (bebop2.error_sumo_yaw < -180) {bebop2.error_sumo_yaw = bebop2.error_sumo_yaw + 360;}
		else {bebop2.error_sumo_yaw = bebop2.error_sumo_yaw;}
		// For bebop3
		if (bebop3.error_sumo_yaw > 180) {bebop3.error_sumo_yaw = bebop3.error_sumo_yaw - 360;}
		else if (bebop3.error_sumo_yaw < -180) {bebop3.error_sumo_yaw = bebop3.error_sumo_yaw + 360;}
		else {bebop3.error_sumo_yaw = bebop3.error_sumo_yaw;}
			
		// Calculate velocity error
		// For bebop1	
		bebop1.velError_sumo_x = getVel(bebop1.prevPos_x, bebop1.pos_x, T);
		bebop1.velError_sumo_y = getVel(bebop1.prevPos_y, bebop1.pos_y, T);
		bebop1.velError_sumo_z = getVel(bebop1.prevPos_z, bebop1.pos_z, T);
		bebop1.velError_sumo_yaw = getVel(bebop1.prevPos_yaw, bebop1.yaw, T);
		// For bebop2	
		bebop2.velError_sumo_x = getVel(bebop2.prevPos_x, bebop2.pos_x, T);
		bebop2.velError_sumo_y = getVel(bebop2.prevPos_y, bebop2.pos_y, T);
		bebop2.velError_sumo_z = getVel(bebop2.prevPos_z, bebop2.pos_z, T);
		bebop2.velError_sumo_yaw = getVel(bebop2.prevPos_yaw, bebop2.yaw, T);
		// For bebop3	
		bebop3.velError_sumo_x = getVel(bebop3.prevPos_x, bebop3.pos_x, T);
		bebop3.velError_sumo_y = getVel(bebop3.prevPos_y, bebop3.pos_y, T);
		bebop3.velError_sumo_z = getVel(bebop3.prevPos_z, bebop3.pos_z, T);
		bebop3.velError_sumo_yaw = getVel(bebop3.prevPos_yaw, bebop3.yaw, T);
		
		// Calculate absition error
		// For bebop1
		bebop1.absitionError_sumo_x = getAbsition(a, b, bebop1.prevPos_x, bebop1.pos_x);
		bebop1.absitionError_sumo_y = getAbsition(a, b, bebop1.prevPos_y, bebop1.pos_y);
		bebop1.absitionError_sumo_z = getAbsition(a, b, bebop1.prevPos_z, bebop1.pos_z);
		bebop1.absitionError_sumo_yaw = getAbsition(a, b, bebop1.prevPos_yaw, bebop1.yaw);
		// For bebop2
		bebop2.absitionError_sumo_x = getAbsition(a, b, bebop2.prevPos_x, bebop2.pos_x);
		bebop2.absitionError_sumo_y = getAbsition(a, b, bebop2.prevPos_y, bebop2.pos_y);
		bebop2.absitionError_sumo_z = getAbsition(a, b, bebop2.prevPos_z, bebop2.pos_z);
		bebop2.absitionError_sumo_yaw = getAbsition(a, b, bebop2.prevPos_yaw, bebop2.yaw);
		// For bebop3
		bebop3.absitionError_sumo_x = getAbsition(a, b, bebop3.prevPos_x, bebop3.pos_x);
		bebop3.absitionError_sumo_y = getAbsition(a, b, bebop3.prevPos_y, bebop3.pos_y);
		bebop3.absitionError_sumo_z = getAbsition(a, b, bebop3.prevPos_z, bebop3.pos_z);
		bebop3.absitionError_sumo_yaw = getAbsition(a, b, bebop3.prevPos_yaw, bebop3.yaw);

		// Find velocity in body frame corresponding to PID controller
		// For bebop1
		bebop1.total_error_x = bebop1.error_droneA_x+bebop1.error_droneB_x;
		bebop1.total_error_y = bebop1.error_droneA_y+bebop1.error_droneB_y;
		
		bebop1.cmd_x = bebop1.total_error_x*kp_xy_multi+bebop1.error_sumo_x*kp_xy-bebop1.velError_sumo_x*kd_xy+bebop1.absitionError_sumo_x*ki_xy;
		bebop1.cmd_y = bebop1.total_error_y*kp_xy_multi+bebop1.error_sumo_y*kp_xy-bebop1.velError_sumo_y*kd_xy+bebop1.absitionError_sumo_y*ki_xy;
		bebop1.cmd_z = bebop1.error_sumo_z*kp_z-bebop1.velError_sumo_z*kd_z+bebop1.absitionError_sumo_z*ki_z;
		bebop1.cmd_yaw = bebop1.error_sumo_yaw*kp_yaw-bebop1.velError_sumo_yaw*kd_yaw+bebop1.absitionError_sumo_yaw*ki_yaw;
		// For bebop2
		bebop2.total_error_x = bebop2.error_droneA_x+bebop2.error_droneB_x;
		bebop2.total_error_y = bebop2.error_droneA_y+bebop2.error_droneB_y;

		bebop2.cmd_x = bebop2.total_error_x*kp_xy_multi+bebop2.error_sumo_x*kp_xy-bebop2.velError_sumo_x*kd_xy+bebop2.absitionError_sumo_x*ki_xy;
		bebop2.cmd_y = bebop2.total_error_y*kp_xy_multi+bebop2.error_sumo_y*kp_xy-bebop2.velError_sumo_y*kd_xy+bebop2.absitionError_sumo_y*ki_xy;
		bebop2.cmd_z = bebop2.error_sumo_z*kp_z-bebop2.velError_sumo_z*kd_z+bebop2.absitionError_sumo_z*ki_z;
		bebop2.cmd_yaw = bebop2.error_sumo_yaw*kp_yaw-bebop2.velError_sumo_yaw*kd_yaw+bebop2.absitionError_sumo_yaw*ki_yaw;
		// For bebop3
		bebop3.total_error_x = bebop3.error_droneA_x+bebop3.error_droneB_x;
		bebop3.total_error_y = bebop3.error_droneA_y+bebop3.error_droneB_y;
		
		bebop3.cmd_x = bebop3.total_error_x*kp_xy_multi+bebop3.error_sumo_x*kp_xy-bebop3.velError_sumo_x*kd_xy+bebop3.absitionError_sumo_x*ki_xy;
		bebop3.cmd_y = bebop3.total_error_y*kp_xy_multi+bebop3.error_sumo_y*kp_xy-bebop3.velError_sumo_y*kd_xy+bebop3.absitionError_sumo_y*ki_xy;
		bebop3.cmd_z = bebop3.error_sumo_z*kp_z-bebop3.velError_sumo_z*kd_z+bebop3.absitionError_sumo_z*ki_z;
		bebop3.cmd_yaw = bebop3.error_sumo_yaw*kp_yaw-bebop3.velError_sumo_yaw*kd_yaw+bebop3.absitionError_sumo_yaw*ki_yaw;
		/*
		printf("1to2_x:%lf\t\t1to2_y:%lf\t\t1to3_x:%lf\t\t1to3_y:%lf\n",bebop1.error_droneA_x,bebop1.error_droneB_x,bebop1.error_droneA_y,bebop1.error_droneB_y);
		printf("2to1_x:%lf\t\t2to1_y:%lf\t\t2to3_x:%lf\t\t2to3_y:%lf\n",bebop2.error_droneA_x,bebop2.error_droneB_x,bebop2.error_droneA_y,bebop2.error_droneB_y);
		printf("3to1_x:%lf\t\t3to1_y:%lf\t\t3to2_x:%lf\t\t3to2_y:%lf\n",bebop3.error_droneA_x,bebop3.error_droneB_x,bebop3.error_droneA_y,bebop3.error_droneB_y);
		*/
		// Suppress output to a given cap so not to exceed velocity values of driver
		// For bebop1
		// x velocity control 
		if (bebop1.cmd_x > speed) {bebop1.cmd_x = speed;}
		else if (bebop1.cmd_x < -speed) {bebop1.cmd_x = -speed;}
		else {bebop1.cmd_x = bebop1.cmd_x;}
		// y velocity control
		if (bebop1.cmd_y > speed) {bebop1.cmd_y = speed;}
		else if (bebop1.cmd_y < -speed) {bebop1.cmd_y = -speed;}
		else {bebop1.cmd_y = bebop1.cmd_y;}
		// z velocity control
		if (bebop1.cmd_z > speed) {bebop1.cmd_z = speed;}
		else if (bebop1.cmd_z < -speed) {bebop1.cmd_z = -speed;}
		else {bebop1.cmd_z = bebop1.cmd_z;}
		// yaw velocity control
		if (bebop1.cmd_yaw > speed) {bebop1.cmd_yaw = speed;}
		else if (bebop1.cmd_yaw < -speed) {bebop1.cmd_yaw = -speed;}
		else {bebop1.cmd_yaw = bebop1.cmd_yaw;}
		// For bebop2
		// x velocity control 
		if (bebop2.cmd_x > speed) {bebop2.cmd_x = speed;}
		else if (bebop2.cmd_x < -speed) {bebop2.cmd_x = -speed;}
		else {bebop2.cmd_x = bebop2.cmd_x;}
		// y velocity control
		if (bebop2.cmd_y > speed) {bebop2.cmd_y = speed;}
		else if (bebop2.cmd_y < -speed) {bebop2.cmd_y = -speed;}
		else {bebop2.cmd_y = bebop2.cmd_y;}
		// z velocity control
		if (bebop2.cmd_z > speed) {bebop2.cmd_z = speed;}
		else if (bebop2.cmd_z < -speed) {bebop2.cmd_z = -speed;}
		else {bebop2.cmd_z = bebop2.cmd_z;}
		// yaw velocity control
		if (bebop2.cmd_yaw > speed) {bebop2.cmd_yaw = speed;}
		else if (bebop2.cmd_yaw < -speed) {bebop2.cmd_yaw = -speed;}
		else {bebop2.cmd_yaw = bebop2.cmd_yaw;}
		// For bebop3
		// x velocity control 
		if (bebop3.cmd_x > speed) {bebop3.cmd_x = speed;}
		else if (bebop3.cmd_x < -speed) {bebop3.cmd_x = -speed;}
		else {bebop3.cmd_x = bebop3.cmd_x;}
		// y velocity control
		if (bebop3.cmd_y > speed) {bebop3.cmd_y = speed;}
		else if (bebop3.cmd_y < -speed) {bebop3.cmd_y = -speed;}
		else {bebop3.cmd_y = bebop3.cmd_y;}
		// z velocity control
		if (bebop3.cmd_z > speed) {bebop3.cmd_z = speed;}
		else if (bebop3.cmd_z < -speed) {bebop3.cmd_z = -speed;}
		else {bebop3.cmd_z = bebop3.cmd_z;}
		// yaw velocity control
		if (bebop3.cmd_yaw > speed) {bebop3.cmd_yaw = speed;}
		else if (bebop3.cmd_yaw < -speed) {bebop3.cmd_yaw = -speed;}
		else {bebop3.cmd_yaw = bebop3.cmd_yaw;}
		
		// Take in rotation for control in inertial to body frame
		// For bebop1
		bebop1.rot_cmd_x = bebop1.cmd_x*cos(bebop1.yaw_RAD) + bebop1.cmd_y*sin(bebop1.yaw_RAD);
		bebop1.rot_cmd_y = bebop1.cmd_x*sin(bebop1.yaw_RAD) - bebop1.cmd_y*cos(bebop1.yaw_RAD);
		// For bebop2
		bebop2.rot_cmd_x = bebop2.cmd_x*cos(bebop2.yaw_RAD) + bebop2.cmd_y*sin(bebop2.yaw_RAD);
		bebop2.rot_cmd_y = bebop2.cmd_x*sin(bebop2.yaw_RAD) - bebop2.cmd_y*cos(bebop2.yaw_RAD);
		// For bebop3
		bebop3.rot_cmd_x = bebop3.cmd_x*cos(bebop3.yaw_RAD) + bebop3.cmd_y*sin(bebop3.yaw_RAD);
		bebop3.rot_cmd_y = bebop3.cmd_x*sin(bebop3.yaw_RAD) - bebop3.cmd_y*cos(bebop3.yaw_RAD);
		
		if (translationControllerOn == true)
		{	
			// For bebop1
			cmd_vel_bebop1_.linear.x = -bebop1.rot_cmd_y;
			cmd_vel_bebop1_.linear.y = -bebop1.rot_cmd_x;
			cmd_vel_bebop1_.linear.z = bebop1.cmd_z;
			// For bebop2
			cmd_vel_bebop2_.linear.x = -bebop2.rot_cmd_y;
			cmd_vel_bebop2_.linear.y = -bebop2.rot_cmd_x;
			cmd_vel_bebop2_.linear.z = bebop2.cmd_z;
			// For bebop3
			cmd_vel_bebop3_.linear.x = -bebop3.rot_cmd_y;
			cmd_vel_bebop3_.linear.y = -bebop3.rot_cmd_x;
			cmd_vel_bebop3_.linear.z = bebop3.cmd_z;
		}
		
		if (translationControllerOn == false)
		{
			// For bebop1
			cmd_vel_bebop1_.linear.x = 0;
			cmd_vel_bebop1_.linear.y = 0;
			cmd_vel_bebop1_.linear.z = 0;
			// For bebop2
			cmd_vel_bebop2_.linear.x = 0;
			cmd_vel_bebop2_.linear.y = 0;
			cmd_vel_bebop2_.linear.z = 0;
			// For bebop3
			cmd_vel_bebop3_.linear.x = 0;
			cmd_vel_bebop3_.linear.y = 0;
			cmd_vel_bebop3_.linear.z = 0;
		}
		
		if (orientationControllerOn == false) 
		{
			// For bebop1
			cmd_vel_bebop1_.angular.z = 0.0;
			// For bebop2
			cmd_vel_bebop2_.angular.z = 0.0;
			// For bebop3
			cmd_vel_bebop3_.angular.z = 0.0;
		}

		if (orientationControllerOn == true) 
		{
			// For bebop1
			cmd_vel_bebop1_.angular.z = bebop1.cmd_yaw;
			// For bebop2
			cmd_vel_bebop2_.angular.z = bebop2.cmd_yaw;
			// For bebop3
			cmd_vel_bebop3_.angular.z = bebop3.cmd_yaw;
		}


		if (button_1 == 1)
		{
			printf("T A K E O F F\n");
			// For bebop1
			takeoff_pub_bebop1_.publish(msg_takeoff);
			// For bebop2
			takeoff_pub_bebop2_.publish(msg_takeoff);
			// For bebop3
			takeoff_pub_bebop3_.publish(msg_takeoff);
		}

		if (button_3 == 1)
		{
			printf("L A N D\n");
			// For bebop1
			land_pub_bebop1_.publish(msg_land);
			// For bebop2
			land_pub_bebop2_.publish(msg_land);
			// For bebop3
			land_pub_bebop3_.publish(msg_land);
		}

		// Publish the assembled command
		// For bebop1
		cmd_vel_pub_bebop1_.publish(cmd_vel_bebop1_);
		// For bebop2
		cmd_vel_pub_bebop2_.publish(cmd_vel_bebop2_);
		// For bebop3
		cmd_vel_pub_bebop3_.publish(cmd_vel_bebop3_);
	
		// Update states
		myTime = myTime + T;
		// Update position states
		// For bebop1
		bebop1.prevPos_x = bebop1.pos_x;
		bebop1.prevPos_y = bebop1.pos_y;
		bebop1.prevPos_z = bebop1.pos_z;
		bebop1.prevPos_yaw = bebop1.yaw;
		// For bebop2
		bebop2.prevPos_x = bebop2.pos_x;
		bebop2.prevPos_y = bebop2.pos_y;
		bebop2.prevPos_z = bebop2.pos_z;
		bebop2.prevPos_yaw = bebop2.yaw;
		// For bebop3
		bebop3.prevPos_x = bebop3.pos_x;
		bebop3.prevPos_y = bebop3.pos_y;
		bebop3.prevPos_z = bebop3.pos_z;
		bebop3.prevPos_yaw = bebop3.yaw;
		// Run subscriber update function
		ros::spinOnce();
		// Delay loop to keep 50 Hz rate
		loop_rate.sleep();
		
		ROS_INFO("myTime:%f\n", myTime);
		//ROS_INFO("des1Pos_x:%lf\tdes1Pos_y:%lf\tdes1YAW:%lf\n", desired1.pos_x, desired1.pos_y, desired1.yaw);
		//ROS_INFO("bebop1.pos_x:%lf\tbebop1.pos_y:%lf\tbebop1.pos_z:%lf\tbebop1.yaw:%lf\n", bebop1.pos_x, bebop1.pos_y, bebop1.pos_z, bebop1.yaw);
		//ROS_INFO("des3Pos_x:%lf\tdes3Pos_y:%lf\tdes3YAW:%lf\n", desired3.pos_x, desired3.pos_y, desired3.yaw);
		//ROS_INFO("bebop2.pos_x:%lf\tbebop2.pos_y:%lf\tbebop2.pos_z:%lf\tbebop2.yaw:%lf\n", bebop2.pos_x, bebop2.pos_y, bebop2.pos_z, bebop2.yaw);
		ROS_INFO("sumo.pos_x:%lf\tsumo.pos_y:%lf\tsumo.pos_z:%lf\tsumo.yaw:%lf\n", sumo.pos_x, sumo.pos_y, sumo.pos_z, sumo.yaw);
		//ROS_INFO("vel1X:%lf\tvel1Y:%lf\tvel1Z:%lf\tvel1YAW:%lf\n", cmd_vel_bebop1_.linear.x, cmd_vel_bebop1_.linear.y, cmd_vel_bebop1_.linear.z, cmd_vel_bebop1_.angular.z);
		//ROS_INFO("vel2X:%lf\tvel2Y:%lf\tvel2Z:%lf\tvel2YAW:%lf\n", cmd_vel_bebop2_.linear.x, cmd_vel_bebop2_.linear.y, cmd_vel_bebop2_.linear.z, cmd_vel_bebop2_.angular.z);
		
		/*
		 * Create 4 text files with "time" "desired position" and  
		 * "actual postition" for matlab graphing of each drone.
		*/
		bebop1Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_x << "" << myTime << "\t" << desired1.pos_x << "\t" << bebop1.pos_x << "\n";
		bebop1Position_y << "" << myTime << "\t" << desired1.pos_y << "\t" << bebop1.pos_y << "\n";
		bebop1Position_z << "" << myTime << "\t" << desired1.pos_z << "\t" << bebop1.pos_z << "\n";
		bebop1Orientation_yaw << "" << myTime << "\t" << desired1.yaw << "\t" << bebop1.yaw << "\n";
		
		bebop2Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_x << "" << myTime << "\t" << desired2.pos_x << "\t" << bebop2.pos_x << "\n";
		bebop2Position_y << "" << myTime << "\t" << desired2.pos_y << "\t" << bebop2.pos_y << "\n";
		bebop2Position_z << "" << myTime << "\t" << desired2.pos_z << "\t" << bebop2.pos_z << "\n";
		bebop2Orientation_yaw << "" << myTime << "\t" << desired2.yaw << "\t" << bebop2.yaw << "\n";
		
		bebop3Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_x << "" << myTime << "\t" << desired3.pos_x << "\t" << bebop3.pos_x << "\n";
		bebop3Position_y << "" << myTime << "\t" << desired3.pos_y << "\t" << bebop3.pos_y << "\n";
		bebop3Position_z << "" << myTime << "\t" << desired3.pos_z << "\t" << bebop3.pos_z << "\n";
		bebop3Orientation_yaw << "" << myTime << "\t" << desired3.yaw << "\t" << bebop3.yaw << "\n";
	}

	/*
	 * Close each .txt file at end of program 
	*/
	bebop1Position_x.close();
	bebop1Position_y.close();
	bebop1Position_z.close();
	bebop1Orientation_yaw.close();
	
	bebop2Position_x.close();
	bebop2Position_y.close();
	bebop2Position_z.close();
	bebop2Orientation_yaw.close();

	bebop3Position_x.close();
	bebop3Position_y.close();
	bebop3Position_z.close();
	bebop3Orientation_yaw.close();

	return 0;
}
