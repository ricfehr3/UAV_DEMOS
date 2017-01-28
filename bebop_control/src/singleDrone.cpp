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
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <fstream>

#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;

#define PI 3.141592653589793238462

/*
* Position is initialized at x=0, y=0, z=1 
* Button 5: x=0, y=0, z=1
* Button 6: x=1, y=0, z=1
* Button 7: x=0, y=1, z=1
* Button 9: x=0, y=0, z=2
* Button 1: Takeoff
* Button3: Land
*/

// Struct for states data
struct states
{
	// Translational position in meters
	double pos_x, pos_y, pos_z;
	// Orientation position in quaternions
	double ori_x, ori_y, ori_z, ori_w;
	// Orientation position in euler angles
	double roll, pitch, yaw, yaw_RAD;
	// Error values for controller
	double error_x, error_y, error_z, error_yaw;
	// Instaneous velocity to be calculated
	double Inst_vel_x, Inst_vel_y, Inst_vel_z, Inst_vel_yaw;
	// Approx. velocity error values
	double velError_x, velError_y, velError_z, velError_yaw;
	// Approx. abstition error values
	double absitionError_x, absitionError_y, absitionError_z, absitionError_yaw;
	// Temp. variables to hold previous position for velocity calculation
	double prevPos_x, prevPos_y, prevPos_z, prevPos_yaw; 
	// Temp. variables to suppress cmd_vel_bebop1_ output to [-1,1]
	double cmd_x, cmd_y, cmd_z, cmd_yaw;
	// Rotation conversion variables for yaw orientation 
	double rot_cmd_x, rot_cmd_y;
} bebop1, bebop2, bebop3, bebop4, desired1, desired2, desired3, desired4;

int button_1;
int button_2;
int button_3;
int button_4;
int button_5;
int button_6;
int button_7;

// Get joystick controller info
void getJoy(const sensor_msgs::Joy::ConstPtr& button)
{
	button_1 = button->buttons[0];
	button_2 = button->buttons[1];
	button_3 = button->buttons[2];
	//lb, rb, lt, rt
	button_4 = button->buttons[4];
	button_5 = button->buttons[5];
	button_6 = button->buttons[6];
	button_7 = button->buttons[7];
}

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

void getPosBebop4(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop4.pos_x = pos->pose.position.x;
	bebop4.pos_y = pos->pose.position.y;
	bebop4.pos_z = pos->pose.position.z;
	bebop4.ori_x = pos->pose.orientation.x;
	bebop4.ori_y = pos->pose.orientation.y;
	bebop4.ori_z = pos->pose.orientation.z;
	bebop4.ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop4.roll, bebop4.pitch, bebop4.yaw_RAD);
	bebop4.yaw = bebop4.yaw_RAD*(180/PI);
}

// Get desired positional data from matlab
// For bebop1
void getPoseDesBebop1(const geometry_msgs::Point::ConstPtr& pos)
{
	desired1.pos_x = pos->x;
	desired1.pos_y = pos->y;
	desired1.pos_z = pos->z;
}
// For bebop2
void getPoseDesBebop2(const geometry_msgs::Point::ConstPtr& pos)
{
	desired2.pos_x = pos->x;
	desired2.pos_y = pos->y;
	desired2.pos_z = pos->z;
}
// For bebop3
void getPoseDesBebop3(const geometry_msgs::Point::ConstPtr& pos)
{
	desired3.pos_x = pos->x;
	desired3.pos_y = pos->y;
	desired3.pos_z = pos->z;
}
// For bebop4
void getPoseDesBebop4(const geometry_msgs::Point::ConstPtr& pos)
{
	desired4.pos_x = pos->x;
	desired4.pos_y = pos->y;
	desired4.pos_z = pos->z;
}

// Get desired velocity data from matlab
// For bebop1
void getVelDesBebop1(const geometry_msgs::Twist::ConstPtr& pos)
{
	desired1.cmd_x = pos->linear.x;
	desired1.cmd_y = pos->linear.y;
	desired1.cmd_z = pos->linear.z; 
	desired1.cmd_yaw = pos->angular.z;
}
// For bebop2
void getVelDesBebop2(const geometry_msgs::Twist::ConstPtr& pos)
{
	desired2.cmd_x = pos->linear.x;
	desired2.cmd_y = pos->linear.y;
	desired2.cmd_z = pos->linear.z;
	desired2.cmd_yaw = pos->angular.z;
}
// For bebop3
void getVelDesBebop3(const geometry_msgs::Twist::ConstPtr& pos)
{
	desired3.cmd_x = pos->linear.x;
	desired3.cmd_y = pos->linear.y;
	desired3.cmd_z = pos->linear.z;
	desired3.cmd_yaw = pos->angular.z;
}
// For bebop4
void getVelDesBebop4(const geometry_msgs::Twist::ConstPtr& pos)
{
	desired4.cmd_x = pos->linear.x;
	desired4.cmd_y = pos->linear.y;
	desired4.cmd_z = pos->linear.z;
	desired4.cmd_yaw = pos->angular.z;
}

// Get position error for proportional control
double getPosError(double desPos, double actualPos)
{
	return desPos-actualPos;
}

// Get position error for proportional control
double getVelError(double desVel, double actualVel)
{
	return desVel-actualVel;
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

double max(double foo, double bar)
{
	if (foo > bar) {return foo;}
	else {return bar;}
}
/*
Mat bel(double uu, double K1_p, double K2_p, double K3_p, double K4_p, double V_p, double W_p, double Vth_p)
{
	Mat out(1,4,CV_64F);
	double out1 = uu;
	
	double S_p = K1_p*out1;
	double A_p = S_p*V_p;
	double Ath_p = S_p*Vth_p;
	double o_p = S_p*W_p;
	
	double outt = out1;
	
	double V_pp = V_p+K2_p*max(0,(outt-A_p-Ath_p));
	double Vth_pp=Vth_p+K4_p*max(0,S_p*(outt-A_p-Ath_p));
	double W_pp=W_p+K3_p*(S_p*(A_p-o_p-outt));

	out.at<double>(0) = A_p+Ath_p-o_p;
	out.at<double>(1) = V_pp;
	out.at<double>(2) = Vth_pp;
	out.at<double>(3) = W_pp;	
	return out;
}   
*/

int main(int argc, char** argv)
{
	bool translationControllerOn = true;
	bool orientationControllerOn = true;
	float myTime = 0.0;
	// Gains for PID controller ##FAST##
	float kp_xy = 0.85; //0.85
	float kd_xy = 0.6; //0.625
	float ki_xy = 0.0; //0.6
	float kp_yaw = 0.05; //0.05
	float kd_yaw = 0.01; //0.01
	float ki_yaw = 0.01; //0.01
	float kp_z = 1.2; //1.2
	float kd_z = 0.1; //0.1
	float ki_z = 0.4; //0.4
	
	// Maximum speed of each drone in m/s ir rad/s
	float speed = 0.99;
				
	bool timeControl  = false;
	double trajSpeed = 0.25; 
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
	// For bebop4
	ros::Publisher takeoff_pub_bebop4_;
	ros::Publisher land_pub_bebop4_;	
	ros::Publisher cmd_vel_pub_bebop4_;

	// Initialize subscribers
	
	// Get actual positional data from motiv	
	// For bebop1
	ros::Subscriber subPoseActual1 = nh_.subscribe("/UAV_IP5/pose", 1000, getPosBebop1);
	// For bebop2
	ros::Subscriber subPoseActual2 = nh_.subscribe("/UAV_IP6/pose", 1000, getPosBebop2);
	// For bebop3
	ros::Subscriber subPoseActual3 = nh_.subscribe("/UAV_IP7/pose", 1000, getPosBebop3);
	// For bebop4
	ros::Subscriber subPoseActual4 = nh_.subscribe("/UAV_IP8/pose", 1000, getPosBebop4);
	
	// Subscribe to joy node
	ros::Subscriber joy_controller = nh_.subscribe("/joy", 1000, getJoy);
	
	// Intitalize advertisers for each bebop
	// For bebop1
	takeoff_pub_bebop1_ = nh_.advertise<std_msgs::Empty>("/bebop_IP5/takeoff", 1000);
	land_pub_bebop1_ = nh_.advertise<std_msgs::Empty>("/bebop_IP5/land", 1000);
	cmd_vel_pub_bebop1_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP5/cmd_vel", 1);
	// For bebop2
	takeoff_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/takeoff", 1000);
	land_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/land", 1000);
	cmd_vel_pub_bebop2_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP6/cmd_vel", 1);
	// For bebop3
	takeoff_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP7/takeoff", 1000);
	land_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP7/land", 1000);
	cmd_vel_pub_bebop3_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP7/cmd_vel", 1);
	// For bebop4
	takeoff_pub_bebop4_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/takeoff", 1000);
	land_pub_bebop4_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/land", 1000);
	cmd_vel_pub_bebop4_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP8/cmd_vel", 1);

	// Velocity data to be sent to individual drones and matlab
	geometry_msgs::Twist cmd_vel_bebop1_;
	geometry_msgs::Twist cmd_vel_bebop2_;
	geometry_msgs::Twist cmd_vel_bebop3_;
	geometry_msgs::Twist cmd_vel_bebop4_;

	// std_msgs::Empty "takeoff" & "land"
	std_msgs::Empty msg_takeoff, msg_land; 

	ros::Rate loop_rate(100);
	float T = 0.01;
	
	desired1.pos_x = 0;
	desired2.pos_x = 0;
	desired3.pos_x = 0;
	desired4.pos_x = 0;
	
	desired1.pos_y = 0.0;
	desired2.pos_y = 0.0;
	desired3.pos_y = 0.0;
	desired4.pos_y = 0.0;
	
	desired1.pos_z = 1;
	desired2.pos_z = 1;
	desired3.pos_z = 1;
	desired4.pos_z = 1;
	
	desired1.yaw = 0;
	desired2.yaw = 0;
	desired3.yaw = 0;
	desired4.yaw = 0;
	
	ofstream bebop2Position_x;
	bebop2Position_x.open ("bebop2Position_x.txt");
	ofstream bebop2Position_y;
	bebop2Position_y.open ("bebop2Position_y.txt");
	ofstream bebop2Position_z;
	bebop2Position_z.open ("bebop2Position_z.txt");
	ofstream bebop2Orientation_yaw;
	bebop2Orientation_yaw.open ("bebop2Orientation_yaw.txt");	

	
	while(nh_.ok())
	{

		double a = 0.0;
		double b = T;	

		if (myTime >= 5 && myTime <= 10)
		{
			desired1.pos_x = 1;
			desired2.pos_x = 1;
			desired3.pos_x = 1;
			desired4.pos_x = 1;
			
			desired1.pos_y = 1;
			desired2.pos_y = 1;
			desired3.pos_y = 1;
			desired4.pos_y = 1;
			
			desired1.pos_z = 1.25;
			desired2.pos_z = 1.25;
			desired3.pos_z = 1.25;
			desired4.pos_z = 1.25;
			
			desired1.yaw = 90;
			desired2.yaw = 90;
			desired3.yaw = 90;
			desired4.yaw = 90;
		}
		
		if (myTime >= 10 && myTime <= 15)
		{
			desired1.pos_x = 1;
			desired2.pos_x = 1;
			desired3.pos_x = 1;
			desired4.pos_x = 1;
			
			desired1.pos_y = -1;
			desired2.pos_y = -1;
			desired3.pos_y = -1;
			desired4.pos_y = -1;
			
			desired1.pos_z = 1;
			desired2.pos_z = 1;
			desired3.pos_z = 1;
			desired4.pos_z = 1;
			
			desired1.yaw = -90;
			desired2.yaw = -90;
			desired3.yaw = -90;
			desired4.yaw = -90;
		}
		
		if (myTime >= 15 && myTime <= 20)
		{
			desired1.pos_x = -1;
			desired2.pos_x = -1;
			desired3.pos_x = -1;
			desired4.pos_x = -1;
			
			desired1.pos_y = 1;
			desired2.pos_y = 1;
			desired3.pos_y = 1;
			desired4.pos_y = 1;
			
			desired1.pos_z = 0.5;
			desired2.pos_z = 0.5;
			desired3.pos_z = 0.5;
			desired4.pos_z = 0.5;
			
			desired1.yaw = 45;
			desired2.yaw = 45;
			desired3.yaw = 45;
			desired4.yaw = 45;
		}
		
		if (myTime >= 20 && myTime <= 25)
		{
			desired1.pos_x = -1;
			desired2.pos_x = -1;
			desired3.pos_x = -1;
			desired4.pos_x = -1;
			
			desired1.pos_y = -1;
			desired2.pos_y = -1;
			desired3.pos_y = -1;
			desired4.pos_y = -1;
			
			desired1.pos_z = 1.5;
			desired2.pos_z = 1.5;
			desired3.pos_z = 1.5;
			desired4.pos_z = 1.5;
			
			desired1.yaw = -45;
			desired2.yaw = -45;
			desired3.yaw = -45;
			desired4.yaw = -45;
		}
		
		if (myTime >= 25 && myTime <= 55)
		{
			int k = 3;
			desired2.pos_x = 1.5*cos(k*trajSpeed*myTime)*cos(trajSpeed*myTime);
			desired2.pos_y = 1.5*cos(k*trajSpeed*myTime)*sin(trajSpeed*myTime);
			desired2.pos_z = 1.0;
			desired2.yaw = 0.0;
		}
		
		if (myTime >= 55)
		{
			desired1.pos_x = 0;
			desired2.pos_x = 0;
			desired3.pos_x = 0;
			desired4.pos_x = 0;
			
			desired1.pos_y = 0;
			desired2.pos_y = 0;
			desired3.pos_y = 0;
			desired4.pos_y = 0;
			
			desired1.pos_z = 1;
			desired2.pos_z = 1;
			desired3.pos_z = 1;
			desired4.pos_z = 1;
			
			desired1.yaw = 0;
			desired2.yaw = 0;
			desired3.yaw = 0;
			desired4.yaw = 0;
		}
		/*
		if (button_4 == 1)
		{
			desired1.pos_x = 0.0;
			desired2.pos_x = 0.0;
			desired3.pos_x = 0.0;
			desired4.pos_x = 0.0;
			
			desired1.pos_y = 1.5;
			desired2.pos_y = 1.5;
			desired3.pos_y = 1.5;
			desired4.pos_y = 1.5;
			
			desired1.pos_z = 1.0;
			desired2.pos_z = 1.0;
			desired3.pos_z = 1.0;
			desired4.pos_z = 1.0;
			
					
    
	
		}
		
		if (button_5 == 1)
		{
			desired1.pos_x = 0;
			desired2.pos_x = 0;
			desired3.pos_x = 0;
			desired4.pos_x = 0;
			
			desired1.pos_y = 1.5;
			desired2.pos_y = 1.5;
			desired3.pos_y = 1.5;
			desired4.pos_y = 1.5;
			
			desired1.pos_z = 1;
			desired2.pos_z = 1;
			desired3.pos_z = 1;
			desired4.pos_z = 1;
			
			desired1.yaw = 0;
			desired2.yaw = 0;
			desired3.yaw = 0;
			desired4.yaw = 0;
		}
		
		if (button_6 == 1)
		{
			desired1.pos_x = 0;
			desired2.pos_x = 0;
			desired3.pos_x = 0;
			desired4.pos_x = 0;
			
			desired1.pos_y = 1.5;
			desired2.pos_y = 1.5;
			desired3.pos_y = 1.5;
			desired4.pos_y = 1.5;
			
			desired1.pos_z = 1;
			desired2.pos_z = 1;
			desired3.pos_z = 1;
			desired4.pos_z = 1;
			
			desired1.yaw = 0;
			desired2.yaw = 0;
			desired3.yaw = 0;
			desired4.yaw = 0;
		}
		
		if (button_7 == 1)
		{
			desired1.pos_x = 0;
			desired2.pos_x = 0;
			desired3.pos_x = 0;
			desired4.pos_x = 0;
			
			desired1.pos_y = 1;
			desired2.pos_y = -1;
			desired3.pos_y = -1;
			desired4.pos_y = -1;
			
			desired1.pos_z = 1.5;
			desired2.pos_z = 1.5;
			desired3.pos_z = 1.5;
			desired4.pos_z = 1.5;
			
			desired1.yaw = -45;
			desired2.yaw = -45;
			desired3.yaw = -45;
			desired4.yaw = -45;
		}
		
		if (button_2 == 1)
		{
			desired1.pos_x = 0;
			desired2.pos_x = 0;
			desired3.pos_x = 0;
			desired4.pos_x = 0;
			
			desired1.pos_y = 0;
			desired2.pos_y = 0;
			desired3.pos_y = 0;
			desired4.pos_y = 0;
			
			desired1.pos_z = 1;
			desired2.pos_z = 1;
			desired3.pos_z = 1;
			desired4.pos_z = 1;
			
			desired1.yaw = 0;
			desired2.yaw = 0;
			desired3.yaw = 0;
			desired4.yaw = 0;
		}
		*/
		
		// Find position error values
		// For bebop1
		bebop1.error_x = getPosError(desired1.pos_x, bebop1.pos_x);
		bebop1.error_y = getPosError(desired1.pos_y, bebop1.pos_y);
		bebop1.error_z = getPosError(desired1.pos_z, bebop1.pos_z);
		bebop1.error_yaw = getPosError(desired1.yaw, bebop1.yaw);
		// For bebop2
		bebop2.error_x = getPosError(desired2.pos_x, bebop2.pos_x);
		bebop2.error_y = getPosError(desired2.pos_y, bebop2.pos_y);
		bebop2.error_z = getPosError(desired2.pos_z, bebop2.pos_z);
		bebop2.error_yaw = getPosError(desired2.yaw, bebop2.yaw);
		// For bebop3
		bebop3.error_x = getPosError(desired3.pos_x, bebop3.pos_x);
		bebop3.error_y = getPosError(desired3.pos_y, bebop3.pos_y);
		bebop3.error_z = getPosError(desired3.pos_z, bebop3.pos_z);
		bebop3.error_yaw = getPosError(desired3.yaw, bebop3.yaw);
		// For bebop4
		bebop4.error_x = getPosError(desired4.pos_x, bebop4.pos_x);
		bebop4.error_y = getPosError(desired4.pos_y, bebop4.pos_y);
		bebop4.error_z = getPosError(desired4.pos_z, bebop4.pos_z);
		bebop4.error_yaw = getPosError(desired4.yaw, bebop4.yaw);
		
		// Calculate instantaneous velocity
		// For bebop1
		bebop1.velError_x = getVel(bebop1.prevPos_x, bebop1.pos_x, T);
		bebop1.velError_y = getVel(bebop1.prevPos_y, bebop1.pos_y, T);
		bebop1.velError_z = getVel(bebop1.prevPos_z, bebop1.pos_z, T);
		bebop1.velError_yaw = getVel(bebop1.prevPos_yaw, bebop1.yaw, T);
		// For bebop2	
		bebop2.velError_x = getVel(bebop2.prevPos_x, bebop2.pos_x, T);
		bebop2.velError_y = getVel(bebop2.prevPos_y, bebop2.pos_y, T);
		bebop2.velError_z = getVel(bebop2.prevPos_z, bebop2.pos_z, T);
		bebop2.velError_yaw = getVel(bebop2.prevPos_yaw, bebop2.yaw, T);
		// For bebop3	
		bebop3.velError_x = getVel(bebop3.prevPos_x, bebop3.pos_x, T);
		bebop3.velError_y = getVel(bebop3.prevPos_y, bebop3.pos_y, T);
		bebop3.velError_z = getVel(bebop3.prevPos_z, bebop3.pos_z, T);
		bebop3.velError_yaw = getVel(bebop3.prevPos_yaw, bebop3.yaw, T);
		// For bebop4	
		bebop4.velError_x = getVel(bebop4.prevPos_x, bebop4.pos_x, T);
		bebop4.velError_y = getVel(bebop4.prevPos_y, bebop4.pos_y, T);
		bebop4.velError_z = getVel(bebop4.prevPos_z, bebop4.pos_z, T);
		bebop4.velError_yaw = getVel(bebop4.prevPos_yaw, bebop4.yaw, T);
		
		// Calculate absition error
		// For bebop1
		bebop1.absitionError_x = getAbsition(a, b, bebop1.prevPos_x, bebop1.pos_x);
		bebop1.absitionError_y = getAbsition(a, b, bebop1.prevPos_y, bebop1.pos_y);
		bebop1.absitionError_z = getAbsition(a, b, bebop1.prevPos_z, bebop1.pos_z);
		bebop1.absitionError_yaw = getAbsition(a, b, bebop1.prevPos_yaw, bebop1.yaw);
		// For bebop2
		bebop2.absitionError_x = getAbsition(a, b, bebop2.prevPos_x, bebop2.pos_x);
		bebop2.absitionError_y = getAbsition(a, b, bebop2.prevPos_y, bebop2.pos_y);
		bebop2.absitionError_z = getAbsition(a, b, bebop2.prevPos_z, bebop2.pos_z);
		bebop2.absitionError_yaw = getAbsition(a, b, bebop2.prevPos_yaw, bebop2.yaw);
		// For bebop3
		bebop3.absitionError_x = getAbsition(a, b, bebop3.prevPos_x, bebop3.pos_x);
		bebop3.absitionError_y = getAbsition(a, b, bebop3.prevPos_y, bebop3.pos_y);
		bebop3.absitionError_z = getAbsition(a, b, bebop3.prevPos_z, bebop3.pos_z);
		bebop3.absitionError_yaw = getAbsition(a, b, bebop3.prevPos_yaw, bebop3.yaw);
		// For bebop4
		bebop4.absitionError_x = getAbsition(a, b, bebop4.prevPos_x, bebop4.pos_x);
		bebop4.absitionError_y = getAbsition(a, b, bebop4.prevPos_y, bebop4.pos_y);
		bebop4.absitionError_z = getAbsition(a, b, bebop4.prevPos_z, bebop4.pos_z);
		bebop4.absitionError_yaw = getAbsition(a, b, bebop4.prevPos_yaw, bebop4.yaw);
		
		// Find velocity in body frame corresponding to PID controller
		// For bebop1 
		bebop1.cmd_x = bebop1.error_x*kp_xy-bebop1.velError_x*kd_xy+bebop1.absitionError_x*ki_xy;
		bebop1.cmd_y = bebop1.error_y*kp_xy-bebop1.velError_y*kd_xy+bebop1.absitionError_y*ki_xy;
		bebop1.cmd_z = bebop1.error_z*kp_z-bebop1.velError_z*kd_z+bebop1.absitionError_z*ki_z;
		bebop1.cmd_yaw = bebop1.error_yaw*kp_yaw-bebop1.velError_yaw*kd_yaw+bebop1.absitionError_yaw*ki_yaw;
		// For bebop2
		bebop2.cmd_x = bebop2.error_x*kp_xy-bebop2.velError_x*kd_xy+bebop2.absitionError_x*ki_xy;
		bebop2.cmd_y = bebop2.error_y*kp_xy-bebop2.velError_y*kd_xy+bebop2.absitionError_y*ki_xy;
		bebop2.cmd_z = bebop2.error_z*kp_z-bebop2.velError_z*kd_z+bebop2.absitionError_z*ki_z;
		bebop2.cmd_yaw = bebop2.error_yaw*kp_yaw-bebop2.velError_yaw*kd_yaw+bebop2.absitionError_yaw*ki_yaw;
		// For bebop3
		bebop3.cmd_x = bebop3.error_x*kp_xy-bebop3.velError_x*kd_xy+bebop3.absitionError_x*ki_xy;
		bebop3.cmd_y = bebop3.error_y*kp_xy-bebop3.velError_y*kd_xy+bebop3.absitionError_y*ki_xy;
		bebop3.cmd_z = bebop3.error_z*kp_z-bebop3.velError_z*kd_z+bebop3.absitionError_z*ki_z;
		bebop3.cmd_yaw = bebop3.error_yaw*kp_yaw-bebop3.velError_yaw*kd_yaw+bebop3.absitionError_yaw*ki_yaw;
		// For bebop4
		bebop4.cmd_x = bebop4.error_x*kp_xy-bebop4.velError_x*kd_xy+bebop4.absitionError_x*ki_xy;
		bebop4.cmd_y = bebop4.error_y*kp_xy-bebop4.velError_y*kd_xy+bebop4.absitionError_y*ki_xy;
		bebop4.cmd_z = bebop4.error_z*kp_z-bebop4.velError_z*kd_z+bebop4.absitionError_z*ki_z;
		bebop4.cmd_yaw = bebop4.error_yaw*kp_yaw-bebop4.velError_yaw*kd_yaw+bebop4.absitionError_yaw*ki_yaw;
		
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
		// For bebop4
		// x velocity control 
		if (bebop4.cmd_x > speed) {bebop4.cmd_x = speed;}
		else if (bebop4.cmd_x < -speed) {bebop4.cmd_x = -speed;}
		else {bebop4.cmd_x = bebop4.cmd_x;}
		// y velocity control
		if (bebop4.cmd_y > speed) {bebop4.cmd_y = speed;}
		else if (bebop4.cmd_y < -speed) {bebop4.cmd_y = -speed;}
		else {bebop4.cmd_y = bebop4.cmd_y;}
		// z velocity control
		if (bebop4.cmd_z > speed) {bebop4.cmd_z = speed;}
		else if (bebop4.cmd_z < -speed) {bebop4.cmd_z = -speed;}
		else {bebop4.cmd_z = bebop4.cmd_z;}
		// yaw velocity control
		if (bebop4.cmd_yaw > speed) {bebop4.cmd_yaw = speed;}
		else if (bebop4.cmd_yaw < -speed) {bebop4.cmd_yaw = -speed;}
		else {bebop4.cmd_yaw = bebop4.cmd_yaw;}
		
		// Rotational matrix, not sure if needed
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
		// For bebop4
		bebop4.rot_cmd_x = bebop4.cmd_x*cos(bebop4.yaw_RAD) + bebop4.cmd_y*sin(bebop4.yaw_RAD);
		bebop4.rot_cmd_y = bebop4.cmd_x*sin(bebop4.yaw_RAD) - bebop4.cmd_y*cos(bebop4.yaw_RAD);

		
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
			// For bebop4
			cmd_vel_bebop4_.linear.x = -bebop4.rot_cmd_y;
			cmd_vel_bebop4_.linear.y = -bebop4.rot_cmd_x;
			cmd_vel_bebop4_.linear.z = bebop4.cmd_z;
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
			// For bebop4
			cmd_vel_bebop4_.linear.x = 0;
			cmd_vel_bebop4_.linear.y = 0;
			cmd_vel_bebop4_.linear.z = 0;
		}
		
		if (orientationControllerOn == false) 
		{
			// For bebop1
			cmd_vel_bebop1_.angular.z = 0.0;
			// For bebop2
			cmd_vel_bebop2_.angular.z = 0.0;
			// For bebop3
			cmd_vel_bebop3_.angular.z = 0.0;
			// For bebop4
			cmd_vel_bebop4_.angular.z = 0.0;
		}
		
		if (orientationControllerOn == true) 
		{
			// For bebop1 
			cmd_vel_bebop1_.angular.z = bebop1.cmd_yaw;
			// For bebop2 
			cmd_vel_bebop2_.angular.z = bebop2.cmd_yaw;
			// For bebop3 
			cmd_vel_bebop3_.angular.z = bebop3.cmd_yaw;
			// For bebop4
			cmd_vel_bebop4_.angular.z = bebop4.cmd_yaw;
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
			// For bebop4
			takeoff_pub_bebop4_.publish(msg_takeoff);
			timeControl = true;
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
			// For bebop4
			land_pub_bebop4_.publish(msg_land);
		}

		// Publish the assembled command
		// For bebop1
		cmd_vel_pub_bebop1_.publish(cmd_vel_bebop1_);
		// For bebop2
		cmd_vel_pub_bebop2_.publish(cmd_vel_bebop2_);
		// For bebop3
		cmd_vel_pub_bebop3_.publish(cmd_vel_bebop3_);
		// For bebop4
		cmd_vel_pub_bebop4_.publish(cmd_vel_bebop4_);

		// Update states
		if (timeControl == true)
		{
			myTime = myTime + T;
		}
		
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
		// For bebop4
		bebop4.prevPos_x = bebop4.pos_x;
		bebop4.prevPos_y = bebop4.pos_y;
		bebop4.prevPos_z = bebop4.pos_z;
		bebop4.prevPos_yaw = bebop4.yaw;
		
		// Run subscriber update function
		ros::spinOnce();
		// Delay loop to keep 100 Hz rate
		loop_rate.sleep();
		
		bebop2Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_x << "" << myTime << "\t" << desired2.pos_x << "\t" << bebop2.pos_x << "\t" << cmd_vel_bebop2_.linear.x << "\n";
		bebop2Position_y << "" << myTime << "\t" << desired2.pos_y << "\t" << bebop2.pos_y << "\t" << cmd_vel_bebop2_.linear.y << "\n";
		bebop2Position_z << "" << myTime << "\t" << desired2.pos_z << "\t" << bebop2.pos_z << "\t" << cmd_vel_bebop2_.linear.z << "\n";
		bebop2Orientation_yaw << "" << myTime << "\t" << desired2.yaw << "\t" << bebop2.yaw << "\t" << cmd_vel_bebop2_.angular.z << "\n";

		
		//ROS_INFO("myTime:%f\n", myTime);
		//ROS_INFO("bebop1.vel.x:%lf\tbebop1.vel.y:%lf\tbebop1.vel.z:%lf\n", cmd_vel_bebop1_.linear.x, cmd_vel_bebop1_.linear.y, cmd_vel_bebop1_.linear.z);
		//ROS_INFO("bebop2.vel.x:%lf\tbebop2.vel.y:%lf\tbebop2.vel.z:%lf\n", cmd_vel_bebop2_.linear.x, cmd_vel_bebop2_.linear.y, cmd_vel_bebop2_.linear.z);
		//ROS_INFO("bebop3.vel.x:%lf\tbebop3.vel.y:%lf\tbebop3.vel.z:%lf\n", cmd_vel_bebop3_.linear.x, cmd_vel_bebop3_.linear.y, cmd_vel_bebop3_.linear.z);
		//ROS_INFO("bebop4.vel.x:%lf\tbebop4.vel.y:%lf\tbebop4.vel.z:%lf\n", cmd_vel_bebop4_.linear.x, cmd_vel_bebop4_.linear.y, cmd_vel_bebop4_.linear.z);
		ROS_INFO("des1Pos_x:%lf\tdes1Pos_y:%lf\tdes1Pos_z:%lf\t d%lf\n", desired1.pos_x, desired1.pos_y, desired1.pos_z, bebop2.velError_x);
		//ROS_INFO("bebop1.pos_x:%lf\tbebop1.pos_y:%lf\tbebop1.pos_z:%lf\tbebop1.yaw:%lf\n", bebop1.pos_x, bebop1.pos_y, bebop1.pos_z, bebop1.yaw);
		//ROS_INFO("des2Pos_x:%lf\tdes2Pos_y:%lf\tdes2YAW:%lf\n", desired2.pos_x, desired2.pos_y, desired2.yaw);
		//ROS_INFO("bebop2.pos_x:%lf\tbebop2.pos_y:%lf\tbebop2.pos_z:%lf\tbebop2.yaw:%lf\n", bebop2.pos_x, bebop2.pos_y, bebop2.pos_z, bebop2.yaw);
		//ROS_INFO("vel1X:%lf\tvel1Y:%lf\tvel1Z:%lf\tvel1YAW:%lf\n", cmd_vel_bebop1_.linear.x, cmd_vel_bebop1_.linear.y, cmd_vel_bebop1_.linear.z, cmd_vel_bebop1_.angular.z);
		//ROS_INFO("vel2X:%lf\tvel2Y:%lf\tvel2Z:%lf\tvel2YAW:%lf\n", cmd_vel_bebop2_.linear.x, cmd_vel_bebop2_.linear.y, cmd_vel_bebop2_.linear.z, cmd_vel_bebop2_.angular.z);
	}

	bebop2Position_x.close();
	bebop2Position_y.close();
	bebop2Position_z.close();
	bebop2Orientation_yaw.close();

	
	return 0;
}
