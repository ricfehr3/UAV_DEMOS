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

struct states
{
	// Translational position in meters
	double pos_x, pos_y, pos_z;
	// Orientation position in quaternions
	double ori_x, ori_y, ori_z, ori_w;
	// Orientation position in euler angles
	double roll, pitch, yaw, yaw_RAD;
	// Error values for controller from sumo
	double error_x, error_y, error_z, error_yaw;
	// Instaneous velocity to be calculated
	double Inst_vel_x, Inst_vel_y, Inst_vel_z, Inst_vel_yaw;
	// Approx. velocity error values from sumo
	double velError_x, velError_y, velError_z, velError_yaw;
	// Approx. abstition error values from sumo
	double absitionError_x, absitionError_y, absitionError_z, absitionError_yaw;
	// Temp. variables to hold previous position for velocity calculation
	double prevPos_x, prevPos_y, prevPos_z, prevPos_yaw; 
	// Temp. variables to suppress cmd_vel_bebop1_ output to [-1,1]
	double cmd_x, cmd_y, cmd_z, cmd_yaw;
	// Rotation conversion variables for yaw orientation 
	double rot_cmd_x, rot_cmd_y;
} bebop1, bebop2, bebop3, bebop4, desired1, desired2, desired3, desired4, sumo;

int button_1;
int button_2;
int button_3;
double pitch;

// Get joystick controller info
void getJoy(const sensor_msgs::Joy::ConstPtr& button)
{
	button_1 = button->buttons[0];
	button_2 = button->buttons[1];
	button_3 = button->buttons[2];
	pitch = button->axes[1];
}

double sigma_1(double z)
{
	double out = z/(sqrt(1+(norm(z)*norm(z))));
	return out;
}

Mat nij(Mat qi, Mat qj, double epsilon)
{
	Mat out = (qj-qi)/(sqrt(1+epsilon*((norm(qj-qi))*(norm(qj-qi)))));
	cout << "Niqqa =" << endl << " " << out << endl << endl;
	return out;
}

double ro_h(double z, double h)
{
	double out;
	if (z>=0 && z<h)
	{
		out = 1.0;
	}
	else if (z>=h && z<=1)
	{
		out = (0.5)*(1+cos(PI*((z-h)/(1-h))));
	}
	else
	{
		out = 0.0;
		printf("Y O U  M A D E  I T  H E R E\n");
	}
	return out;
}

double phi(double z, double a, double b)
{
	double c = abs(a-b)/sqrt(4*a*b);
	double out = (0.5)*((a+b)*sigma_1(z+c)+(a-b));
	return out;
}

double phi_alpha(double z, double r_alpha, double d_alpha, double h, double a, double b)
{
	printf("butt %f\n",z);
	double out = ro_h((z/r_alpha),h)*phi((z-d_alpha),a,b);
	return out;
}

Mat sigma_1(Mat z)
{
	double tmp = norm(z, NORM_L2);
	double tmp2 = tmp*tmp;
	Mat out = z/(sqrt(1+tmp2));
	return out;
}

double sigma_norm(Mat z, double epsilon)
{
	double tmp = norm(z);
	double tmp2 = tmp*tmp;
	double out = (1/epsilon)*(sqrt(1+epsilon*(tmp2))-1);
	return out;
}

double aij(Mat qi, Mat qj, double r_alpha, double h, double epsilon, int i, int j)
{
	double out;
	if (i!=j)
	{
		out = ro_h((sigma_norm((qj-qi),epsilon)/r_alpha),h);
	}
	else 
	{
		out = 0;
	}
	printf("meme %f dream %d team %d creamed %f\n",out,i,j,(sigma_norm((qj-qi),epsilon)/r_alpha));
	return out;
}

Mat N_i(int i,Mat q,double r,double epsilon)
{
	//cout << "Ni q =" << endl << " " << q << endl << endl;
	int rows;// = q.rows;
	int cols;// = q.cols;;
	cv::Size s = q.size();
	rows = s.height;
	cols = s.width;
	int k = 0;
	double r_alpha = (1.0/epsilon)*(sqrt(1+epsilon*(norm(r)*norm(r)))-1.0); //sigma_norm(r, epsilon);
	//int tmp[4] = {0,0,0,0};
	Mat OUT = Mat(1, 4, CV_32S);
	OUT.at<int>(0) = 0;
	OUT.at<int>(1) = 0;
	OUT.at<int>(2) = 0;
	OUT.at<int>(3) = 0;
	//cout << "OUT =" << endl << " " << OUT << endl << endl;
	
	for (int j=1; j<=rows; j++)
	{
		if (i!=j)
		{
			Mat TMP = Mat(1, 3, CV_64FC1);
			for (int h=1; h<=3; h++)
			{
				TMP.at<double>(h-1) = q.at<double>(j-1,h-1)-q.at<double>(i-1,h-1);
			} 
			//cout << "NI TMP =" << endl << " " << TMP << endl << endl;
			double x = sigma_norm(TMP,epsilon);	
			//printf("x:%lf r_alpha:%lf\n",x,r_alpha);
			if (x<=r_alpha)
			{
				k++;
				//printf("k:%d, j:%d\n",k,j);
				OUT.at<int>(0,k-1) = j;
			}
		}
	}
	OUT = OUT.colRange(0,k);
return OUT;
}

Mat fi_alpha(double c1_a, double c2_a, int i, Mat q, Mat p, double r, double d, double h, double a, double b, double epsilon)
{
	int rows = q.rows;
	int cols = q.cols;

	cv::Size s = q.size();
	rows = s.height;
	cols = s.width;
	
	double r_alpha = (1/epsilon)*(sqrt(1+epsilon*(r*r))-1); //sigma_norm(r, epsilon);
	double d_alpha = (1/epsilon)*(sqrt(1+epsilon*(d*d))-1); //sigma_norm(r, epsilon);
	
	double tmp[3] = {0, 0, 0};
	Mat sum1 = Mat(1, 3, CV_64F);
	Mat sum2 = Mat(1, 3, CV_64F);
	Mat Ni = Mat(1, 4, CV_32S, tmp); 
	Ni = N_i(i,q,r,epsilon);
	cout << "i =" << endl << " " << i << endl << endl;
	int length;
	cv::Size boi = Ni.size();
	length = boi.width;
//tab
	Mat TMP1 = Mat(1, 3, CV_64F);
	Mat TMP2 = Mat(1, 3, CV_64F);
	Mat TMP3 = Mat(1, 3, CV_64F);
	Mat TMP4 = Mat(1, 3, CV_64F);
	Mat TMP5 = Mat(1, 3, CV_64F);

	cout << "Ni =" << endl << " " << Ni << endl << endl;
	//cout << "Ni.at<int>(0) =" << endl << " " << Ni.at<int>(0) << endl << endl;
	
	for (int k=1; k<=length; k++)
	{
		for (int j=1; j<=rows; j++)
		{
			if(Ni.at<int>(k-1)==j)
			{
				//printf("gud wurk\n");
				for (int hh=1; hh<=3; hh++)
				{
					TMP1.at<double>(0,hh-1) = q.at<double>(j-1,hh-1)-q.at<double>(i-1,hh-1);
					TMP2.at<double>(0,hh-1) = q.at<double>(i-1,hh-1);
					TMP3.at<double>(0,hh-1) = q.at<double>(j-1,hh-1);
					TMP4.at<double>(0,hh-1) = p.at<double>(i-1,hh-1);
					TMP5.at<double>(0,hh-1) = p.at<double>(j-1,hh-1);
				}
				sum1 = phi_alpha(sigma_norm(TMP1, epsilon),r_alpha,d_alpha,h,a,b)*nij(TMP2,TMP3,epsilon)+sum1;
				//printf("i=%d\tj=%d\n",i,j);
				cout << "TMP1 =" << endl << " " << TMP1 << endl << endl;
				//cout << "TMP2 =" << endl << " " << TMP2 << endl << endl;
				//cout << "TMP3 =" << endl << " " << TMP3 << endl << endl;
				//cout << "q.at<double>(j-1,hh-1) =" << endl << " " << q.at<double>(j-1,0) << endl << endl;
				sum2 = aij(TMP2,TMP3,r_alpha,h,epsilon,i,j)*(TMP5-TMP4)+sum2; 
			}
		}
	}
	Mat out = c1_a*sum1+c2_a*sum2;
	return out;
}

// Object tracking algorithm
Mat fi_gamma(Mat q_i, Mat p_i, Mat qr, Mat pr, double c1, double c2)
{
	Mat out = Mat(1, 3, CV_64FC1);
	out = -c1*(sigma_1(q_i-qr)-c2*(p_i-pr));
	return out;
}

/*
 * Controller Code based on previous control method for waypoint and 
 * trajectory tracking. All pid is based on actual and desired position
 * in a feedback loop.
*/

// Get position error for proportional control
double getPosError(double desPos, double actualPos)
{
	return desPos-actualPos;
}

// Get velocity as definition of derivative
double getVel(double previousPos, double currentPos, float h)
{
	return (currentPos-previousPos)/h;
}

// Get absition using Simpsons method
double getAbsition(double a, double b, double previousPos, double currentPos)
{
	double s = (previousPos+currentPos)/2;
	double q = (b-a)/6;
	return q*(currentPos+4*s+previousPos); 
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

// Get position info from mocap for bebop2 
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

// Get position info from mocap for bebop3
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

// Get position info from mocap for bebop4
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

int main(int argc, char** argv)
{
	double d = 2.0; // 7 in simulation
	double r = 1.25*d;//1.2*d;
	double r_prime = 0.6*r;
	double kappa = 1.2;
	double d_p = 0.6*d;
	double r_p = 1.2*d_p;
	double epsilon = 0.1; // For sigma_norm //1
	double aa = 5; 
	double bb = aa; // For phi
	double h_a = 0.2; // For phi_alpha
	double h_b = 0.9; // For phi_beta
	double N = 4; // Number of agents
	double m = 3; // m=2 for 2-D and m=3 for 3-D

	/*
	double qd[3] = {0,0,0}; // Target Position
	Mat Qd = Mat(1, 3, CV_64FC1, qd);
	*/

	double qd1[3] = {-1.2,1.2,0}; // Target Position
	Mat Qd1 = Mat(1, 3, CV_64FC1, qd1);
	
	double qd2[3] = {1.2,1.2,0}; // Target Position
	Mat Qd2 = Mat(1, 3, CV_64FC1, qd2);
	
	double qd3[3] = {-1.2,-1.2,0}; // Target Position
	Mat Qd3 = Mat(1, 3, CV_64FC1, qd3);
	
	double qd4[3] = {1.2,-1.2,0}; // Target Position
	Mat Qd4 = Mat(1, 3, CV_64FC1, qd4);

	double pd[3] = {0,0,0}; // Target Speed  
	Mat Pd = Mat(1, 3, CV_64FC1, pd);

	/*
	double c1_b = 10;
	double c2_b = 2*sqrt(c1_b);
	double c1_g = 0.2*c1_b;
	double c2_g = 2*sqrt(c1_g);
	double c1_a = 0.5*c1_g;
	double c2_a = 2*sqrt(c1_a); // c1_a<c1_g<c1_b
	*/

	//double c1_b = 10;
	//double c2_b = 2*sqrt(c1_b);
	double c1_g = 30.0;//5;
	double c2_g = 0.0;
	double c1_a = 1.25;//2;
	double c2_a = 0.0;//1.0;//0.5;
	
	bool butt = false;
	bool translationControllerOn = true;
	bool orientationControllerOn = true;
	float myTime = 0.0;
	float trajSpeed = PI/6;
	// Gains for PID controller ##FAST##
	float kp_xy = 0.0;//0.85; //0.85
	float kd_xy = 0.6; //0.625
	float ki_xy = 1.2; //0.6
	float kp_yaw = 0.05; //0.05
	float kd_yaw = 0.01; //0.01
	float ki_yaw = 0.01; //0.01
	float kp_z = 1.2; //1.2
	float kd_z = 0.1; //0.1
	float ki_z = 0.4; //0.4
	
	// Maximum speed of each drone in m/s ir rad/s
	float speed = 0.99;
	float speedz = 0.99;
	
	// Initialize ros
	ros::init(argc, argv, "bebop_control"); 

	// Node handle used
	ros::NodeHandle nh_; 
	
	// Initialized publishers
	// For bebop1
	ros::Publisher takeoff_pub_bebop1_;
	ros::Publisher land_pub_bebop1_;	
	ros::Publisher cmd_vel_pub_bebop1_;
	ros::Publisher inst_vel_pub_bebop1_;
	// For bebop2
	ros::Publisher takeoff_pub_bebop2_;
	ros::Publisher land_pub_bebop2_;	
	ros::Publisher cmd_vel_pub_bebop2_;
	ros::Publisher inst_vel_pub_bebop2_;
	// For bebop3
	ros::Publisher takeoff_pub_bebop3_;
	ros::Publisher land_pub_bebop3_;	
	ros::Publisher cmd_vel_pub_bebop3_;
	ros::Publisher inst_vel_pub_bebop3_;
	// For bebop4
	ros::Publisher takeoff_pub_bebop4_;
	ros::Publisher land_pub_bebop4_;	
	ros::Publisher cmd_vel_pub_bebop4_;
	ros::Publisher inst_vel_pub_bebop4_;
	
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
	cmd_vel_pub_bebop1_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP5/cmd_vel", 1000);
	// For bebop2
	takeoff_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/takeoff", 1000);
	land_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/land", 1000);
	cmd_vel_pub_bebop2_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP6/cmd_vel", 1000);
	// For bebop3
	takeoff_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP7/takeoff", 1000);
	land_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP7/land", 1000);
	cmd_vel_pub_bebop3_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP7/cmd_vel", 1000);
	// For bebop4
	takeoff_pub_bebop4_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/takeoff", 1000);
	land_pub_bebop4_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/land", 1000);
	cmd_vel_pub_bebop4_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP8/cmd_vel", 1000);
	
	// Velocity data to be sent to individual drones and matlab
	geometry_msgs::Twist cmd_vel_bebop1_;
	geometry_msgs::Twist cmd_vel_bebop2_;
	geometry_msgs::Twist cmd_vel_bebop3_;
	geometry_msgs::Twist cmd_vel_bebop4_;
	
	// std_msgs::Empty "takeoff" & "land"
	std_msgs::Empty msg_takeoff, msg_land; 
	
	ros::Rate loop_rate(100); // Yeah boi gotta b fast af
	float dt = 0.05; // YEEEE BOIIII
	float T = 0.01;
	int k = 0;
	
	Mat qkk1 = Mat(1, 3, CV_64FC1); //q(k+1)
	Mat pkk1 = Mat(1, 3, CV_64FC1); //p(k+1)
	Mat qkk2 = Mat(1, 3, CV_64FC1);
	Mat pkk2 = Mat(1, 3, CV_64FC1);
	Mat qkk3 = Mat(1, 3, CV_64FC1);
	Mat pkk3 = Mat(1, 3, CV_64FC1);
	Mat qkk4 = Mat(1, 3, CV_64FC1);
	Mat pkk4 = Mat(1, 3, CV_64FC1);
	
	Mat u1PID = Mat(1, 3, CV_64FC1);
	Mat u2PID = Mat(1, 3, CV_64FC1);
	Mat u3PID = Mat(1, 3, CV_64FC1);
	Mat u4PID = Mat(1, 3, CV_64FC1);

	Mat q1 = Mat(1, 3, CV_64FC1);
	Mat q2 = Mat(1, 3, CV_64FC1);
	Mat q3 = Mat(1, 3, CV_64FC1);
	Mat q4 = Mat(1, 3, CV_64FC1);
		
	Mat p1 = Mat(1, 3, CV_64FC1);
	Mat p2 = Mat(1, 3, CV_64FC1);
	Mat p3 = Mat(1, 3, CV_64FC1);
	Mat p4 = Mat(1, 3, CV_64FC1);
	
	// 4 seperate variables
	Mat nei1 = Mat(1, 3, CV_64FC1);
	Mat nei2 = Mat(1, 3, CV_64FC1);
	Mat nei3 = Mat(1, 3, CV_64FC1);
	Mat nei4 = Mat(1, 3, CV_64FC1);
	// Smile through the pain
	Mat u_a1 = Mat(1, 3, CV_64FC1);
	Mat u_a2 = Mat(1, 3, CV_64FC1);
	Mat u_a3 = Mat(1, 3, CV_64FC1);
	Mat u_a4 = Mat(1, 3, CV_64FC1);
	// It's not that bad
	Mat u_g1 = Mat(1, 3, CV_64FC1);
	Mat u_g2 = Mat(1, 3, CV_64FC1);
	Mat u_g3 = Mat(1, 3, CV_64FC1);
	Mat u_g4 = Mat(1, 3, CV_64FC1);
	// It can only get better
	Mat u1 = Mat(1, 3, CV_64FC1);
	Mat u2 = Mat(1, 3, CV_64FC1);
	Mat u3 = Mat(1, 3, CV_64FC1);
	Mat u4 = Mat(1, 3, CV_64FC1);
	
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

	ofstream bebop4Position_x;
	bebop4Position_x.open ("bebop4Position_x.txt");
	ofstream bebop4Position_y;
	bebop4Position_y.open ("bebop4Position_y.txt");
	ofstream bebop4Position_z;
	bebop4Position_z.open ("bebop4Position_z.txt");
	ofstream bebop4Orientation_yaw;
	bebop4Orientation_yaw.open ("bebop4Orientation_yaw.txt");	
	
	while(nh_.ok())
	{
		double a = 0.0;
		double b = T;
		
		desired1.pos_x = qkk1.at<double>(0);
		desired2.pos_x = qkk2.at<double>(0);
		desired3.pos_x = qkk3.at<double>(0);
		desired4.pos_x = qkk4.at<double>(0);
		
		desired1.pos_y = qkk1.at<double>(1);
		desired2.pos_y = qkk2.at<double>(1);
		desired3.pos_y = qkk3.at<double>(1);
		desired4.pos_y = qkk4.at<double>(1);
		
		desired1.pos_z = 1; //qkk1.at<double>(2);
		desired2.pos_z = 1; //qkk2.at<double>(2);
		desired3.pos_z = 1; //qkk3.at<double>(2);
		desired4.pos_z = 1; //qkk4.at<double>(2);
		
		// big boi
		desired1.yaw = 0;
		desired2.yaw = 0;
		desired3.yaw = 0;
		desired4.yaw = 0;
		
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
		bebop1.Inst_vel_x = getVel(bebop1.prevPos_x, bebop1.pos_x, T);
		bebop1.Inst_vel_y = getVel(bebop1.prevPos_y, bebop1.pos_y, T);
		bebop1.Inst_vel_z = getVel(bebop1.prevPos_z, bebop1.pos_z, T);
		bebop1.Inst_vel_yaw = getVel(bebop1.prevPos_yaw, bebop1.yaw, T);
		// For bebop2	
		bebop2.Inst_vel_x = getVel(bebop2.prevPos_x, bebop2.pos_x, T);
		bebop2.Inst_vel_y = getVel(bebop2.prevPos_y, bebop2.pos_y, T);
		bebop2.Inst_vel_z = getVel(bebop2.prevPos_z, bebop2.pos_z, T);
		bebop2.Inst_vel_yaw = getVel(bebop2.prevPos_yaw, bebop2.yaw, T);
		// For bebop3	
		bebop3.Inst_vel_x = getVel(bebop3.prevPos_x, bebop3.pos_x, T);
		bebop3.Inst_vel_y = getVel(bebop3.prevPos_y, bebop3.pos_y, T);
		bebop3.Inst_vel_z = getVel(bebop3.prevPos_z, bebop3.pos_z, T);
		bebop3.Inst_vel_yaw = getVel(bebop3.prevPos_yaw, bebop3.yaw, T);
		// For bebop4	
		bebop4.Inst_vel_x = getVel(bebop4.prevPos_x, bebop4.pos_x, T);
		bebop4.Inst_vel_y = getVel(bebop4.prevPos_y, bebop4.pos_y, T);
		bebop4.Inst_vel_z = getVel(bebop4.prevPos_z, bebop4.pos_z, T);
		bebop4.Inst_vel_yaw = getVel(bebop4.prevPos_yaw, bebop4.yaw, T);
		
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
		
		// Account for massive yaw error from 180 to -180 crossover
		// For bebop1
		if (bebop1.error_yaw > 180) {bebop1.error_yaw = bebop1.error_yaw - 360;}
		else if (bebop1.error_yaw < -180) {bebop1.error_yaw = bebop1.error_yaw + 360;}
		else {bebop1.error_yaw = bebop1.error_yaw;}
		// For bebop2
		if (bebop2.error_yaw > 180) {bebop2.error_yaw = bebop2.error_yaw - 360;}
		else if (bebop2.error_yaw < -180) {bebop2.error_yaw = bebop2.error_yaw + 360;}
		else {bebop2.error_yaw = bebop2.error_yaw;}
		// For bebop3
		if (bebop3.error_yaw > 180) {bebop3.error_yaw = bebop3.error_yaw - 360;}
		else if (bebop3.error_yaw < -180) {bebop3.error_yaw = bebop3.error_yaw + 360;}
		else {bebop3.error_yaw = bebop3.error_yaw;}
		// For bebop4
		if (bebop4.error_yaw > 180) {bebop4.error_yaw = bebop4.error_yaw - 360;}
		else if (bebop4.error_yaw < -180) {bebop4.error_yaw = bebop4.error_yaw + 360;}
		else {bebop4.error_yaw = bebop4.error_yaw;}
		
		// Find velocity in body frame corresponding to PID controller
		// For bebop1 
		bebop1.cmd_x = bebop1.error_x*kp_xy-bebop1.Inst_vel_x*kd_xy+bebop1.absitionError_x*ki_xy;
		bebop1.cmd_y = bebop1.error_y*kp_xy-bebop1.Inst_vel_y*kd_xy+bebop1.absitionError_y*ki_xy;
 		bebop1.cmd_z = bebop1.error_z*kp_z-bebop1.Inst_vel_z*kd_z+bebop1.absitionError_z*ki_z;
		bebop1.cmd_yaw = bebop1.error_yaw*kp_yaw-bebop1.Inst_vel_yaw*kd_yaw+bebop1.absitionError_yaw*ki_yaw;
		// For bebop2
		bebop2.cmd_x = bebop2.error_x*kp_xy-bebop2.Inst_vel_x*kd_xy+bebop2.absitionError_x*ki_xy;
		bebop2.cmd_y = bebop2.error_y*kp_xy-bebop2.Inst_vel_y*kd_xy+bebop2.absitionError_y*ki_xy;
		bebop2.cmd_z = bebop2.error_z*kp_z-bebop2.Inst_vel_z*kd_z+bebop2.absitionError_z*ki_z;
		bebop2.cmd_yaw = bebop2.error_yaw*kp_yaw-bebop2.Inst_vel_yaw*kd_yaw+bebop2.absitionError_yaw*ki_yaw;
		// For bebop3
		bebop3.cmd_x = bebop3.error_x*kp_xy-bebop3.Inst_vel_x*kd_xy+bebop3.absitionError_x*ki_xy;
		bebop3.cmd_y = bebop3.error_y*kp_xy-bebop3.Inst_vel_y*kd_xy+bebop3.absitionError_y*ki_xy;
		bebop3.cmd_z = bebop3.error_z*kp_z-bebop3.Inst_vel_z*kd_z+bebop3.absitionError_z*ki_z;
		bebop3.cmd_yaw = bebop3.error_yaw*kp_yaw-bebop3.Inst_vel_yaw*kd_yaw+bebop3.absitionError_yaw*ki_yaw;
		// For bebop4
		bebop4.cmd_x = bebop4.error_x*kp_xy-bebop4.Inst_vel_x*kd_xy+bebop4.absitionError_x*ki_xy;
		bebop4.cmd_y = bebop4.error_y*kp_xy-bebop4.Inst_vel_y*kd_xy+bebop4.absitionError_y*ki_xy;
		bebop4.cmd_z = bebop4.error_z*kp_z-bebop4.Inst_vel_z*kd_z+bebop4.absitionError_z*ki_z;
		bebop4.cmd_yaw = bebop4.error_yaw*kp_yaw-bebop4.Inst_vel_yaw*kd_yaw+bebop4.absitionError_yaw*ki_yaw;		

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
			/*
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
			*/
			u1PID.at<double>(0) = -bebop1.rot_cmd_y;
			u1PID.at<double>(1) = -bebop1.rot_cmd_x;
			u1PID.at<double>(2) = bebop1.cmd_z;
			
			u2PID.at<double>(0) = -bebop2.rot_cmd_y;
			u2PID.at<double>(1) = -bebop2.rot_cmd_x;
			u2PID.at<double>(2) = bebop2.cmd_z;
			
			u3PID.at<double>(0) = -bebop3.rot_cmd_y;
			u3PID.at<double>(1) = -bebop3.rot_cmd_x;
			u3PID.at<double>(2) = bebop3.cmd_z;
			
			u4PID.at<double>(0) = -bebop4.rot_cmd_y;
			u4PID.at<double>(1) = -bebop4.rot_cmd_x;
			u4PID.at<double>(2) = bebop4.cmd_z;
		}
		
		if (translationControllerOn == false)
		{
			/*
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
			*/
			u1PID.at<double>(0) = 0;
			u1PID.at<double>(1) = 0;
			u1PID.at<double>(2) = 0;
			
			u2PID.at<double>(0) = 0;
			u2PID.at<double>(1) = 0;
			u2PID.at<double>(2) = 0;
			
			u3PID.at<double>(0) = 0;
			u3PID.at<double>(1) = 0;
			u3PID.at<double>(2) = 0;
			
			u4PID.at<double>(0) = 0;
			u4PID.at<double>(1) = 0;
			u4PID.at<double>(2) = 0; 
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
			butt = true;
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
		
		if (butt == true)
		{
			myTime = myTime + T;
		}
		
		k = k+1;
		// Store Positions in Matrix
		/*
		double q[4][3] = {{1, 1, 1},
						  {2, 1, 1},
						  {-1, 1, 1},
						  {-2, 1, 1}};
		Mat Q = Mat(4, 3, CV_64FC1, q);
		*/
		double q[4][3] = {{bebop1.pos_x, bebop1.pos_y, bebop1.pos_z},
						  {bebop2.pos_x, bebop2.pos_y, bebop2.pos_z},
						  {bebop3.pos_x, bebop3.pos_y, bebop3.pos_z},
						  {bebop4.pos_x, bebop4.pos_y, bebop4.pos_z}};
		Mat Q = Mat(4, 3, CV_64FC1, q);
		/*
		double p[4][3] = {{cmd_vel_bebop1_.linear.x, cmd_vel_bebop1_.linear.y, cmd_vel_bebop1_.linear.z},
						  {cmd_vel_bebop2_.linear.x, cmd_vel_bebop2_.linear.y, cmd_vel_bebop2_.linear.z},
						  {cmd_vel_bebop3_.linear.x, cmd_vel_bebop3_.linear.y, cmd_vel_bebop3_.linear.z},
						  {cmd_vel_bebop4_.linear.x, cmd_vel_bebop4_.linear.y, cmd_vel_bebop4_.linear.z}};
		*/
		double p[4][3] = {{bebop1.Inst_vel_x, bebop1.Inst_vel_y, bebop1.Inst_vel_z},
						  {bebop2.Inst_vel_x, bebop2.Inst_vel_y, bebop2.Inst_vel_z},
						  {bebop3.Inst_vel_x, bebop3.Inst_vel_y, bebop3.Inst_vel_z},
						  {bebop4.Inst_vel_x, bebop4.Inst_vel_y, bebop4.Inst_vel_z}}; 
		Mat P = Mat(4, 3, CV_64FC1, p);	
		
		for (int ii = 1; ii <= 3; ii++)
		{
			q1.at<double>(ii-1) = Q.at<double>(0,ii-1);
			q2.at<double>(ii-1) = Q.at<double>(1,ii-1);
			q3.at<double>(ii-1) = Q.at<double>(2,ii-1);
			q4.at<double>(ii-1) = Q.at<double>(3,ii-1);

			p1.at<double>(ii-1) = P.at<double>(0,ii-1);
			p2.at<double>(ii-1) = P.at<double>(1,ii-1);
			p3.at<double>(ii-1) = P.at<double>(2,ii-1);
			p4.at<double>(ii-1) = P.at<double>(3,ii-1);
		}

		if (myTime >= 0 && myTime < 10)
		{
		Qd2.at<double>(0) = 0.0;
		Qd2.at<double>(1) = 1.0;
		
		Qd3.at<double>(0) = -1.0;
		Qd3.at<double>(1) = -1.0;

		Qd1.at<double>(0) = 1.0;
		Qd1.at<double>(1) = -1.0;
		}

		// Update Qd for circuilar trajectory
		if (myTime >= 10)
		{
		Qd1.at<double>(0) = 1.5*cos(trajSpeed*myTime);
		Qd1.at<double>(1) = 1.5*sin(trajSpeed*myTime);
		
		Qd2.at<double>(0) = 1.5*cos(trajSpeed*myTime + (2.0/3.0)*PI);
		Qd2.at<double>(1) = 1.5*sin(trajSpeed*myTime + (2.0/3.0)*PI);

		Qd3.at<double>(0) = 1.5*cos(trajSpeed*myTime + (4.0/3.0)*PI);
		Qd3.at<double>(1) = 1.5*sin(trajSpeed*myTime + (4.0/3.0)*PI);
		}
		/*
		 * Matlab code for consensus 
		*/
		nei1 = N_i(1,Q,d,epsilon);
		nei2 = N_i(2,Q,d,epsilon);
		nei3 = N_i(3,Q,d,epsilon);
		nei4 = N_i(4,Q,d,epsilon);
	
		u_a1 = fi_alpha(c1_a, c2_a, 1, Q, P, r, d, h_a, aa, bb, epsilon);
		u_a2 = fi_alpha(c1_a, c2_a, 2, Q, P, r, d, h_a, aa, bb, epsilon);
		u_a3 = fi_alpha(c1_a, c2_a, 3, Q, P, r, d, h_a, aa, bb, epsilon);
		u_a4 = fi_alpha(c1_a, c2_a, 4, Q, P, r, d, h_a, aa, bb, epsilon);
	
		u_g1 = fi_gamma(q1, p1, Qd1, Pd, c1_g, c2_g);
		u_g2 = fi_gamma(q2, p2, Qd2, Pd, c1_g, c2_g);
		u_g3 = fi_gamma(q3, p3, Qd3, Pd, c1_g, c2_g);
		u_g4 = fi_gamma(q4, p4, Qd4, Pd, c1_g, c2_g);
		
		// Total output matrix for 
		u1 = u_a1 + u_g1; 
		u2 = u_a2 + u_g2;
		u3 = u_a3 + u_g3;
		u4 = u_a4 + u_g4;
/*
		// q{k+1}
		qkk1 = q1-dt*p1;
		//qkk2 = q2-dt*p2;
		qkk2.at<double>(0) = q2.at<double>(0)+dt*p2.at<double>(0);
		qkk2.at<double>(1) = q2.at<double>(1)-dt*p2.at<double>(1);
		qkk3 = q3-dt*p3;
		qkk4 = q4-dt*p4;
		// p{k+1}
		pkk1 = p1+dt*u1;
		pkk2 = p2+dt*u2;
		pkk3 = p3+dt*u3;
		pkk4 = p4+dt*u4;
*/
		// p{k+1}
		pkk1 = dt*u1;
		pkk2 = dt*u2;
		pkk3 = dt*u3;
		pkk4 = dt*u4;
		
		qkk1 = dt*pkk1;
		qkk2 = dt*pkk2;
		qkk3 = dt*pkk3;
		qkk4 = dt*pkk4;

		bebop1.cmd_x = u1PID.at<double>(0)+pkk1.at<double>(1);
		bebop1.cmd_y = u1PID.at<double>(1)-pkk1.at<double>(0);
		bebop1.cmd_z = u1PID.at<double>(2);//pkk1.at<double>(2)+u1PID.at<double>(2);
		
		bebop2.cmd_x = u2PID.at<double>(0)+pkk2.at<double>(1);
		bebop2.cmd_y = u2PID.at<double>(1)-pkk2.at<double>(0);
		bebop2.cmd_z = u2PID.at<double>(2);//pkk2.at<double>(2)+u2PID.at<double>(2);
		
		bebop3.cmd_x = u3PID.at<double>(0)+pkk3.at<double>(1);
		bebop3.cmd_y = u3PID.at<double>(1)-pkk3.at<double>(0);
		bebop3.cmd_z = u3PID.at<double>(2);//pkk3.at<double>(2)+u3PID.at<double>(2);
		
		bebop4.cmd_x = u4PID.at<double>(0)+pkk4.at<double>(1);
		bebop4.cmd_y = u4PID.at<double>(1)-pkk4.at<double>(0);
		bebop4.cmd_z = u4PID.at<double>(2);//pkk4.at<double>(2)+u4PID.at<double>(2);

		/*
		cout << "qd=" << endl << " " << Qd << endl << endl;
		cout << "Desired pos [x,y,z]=" << endl << " " << qkk2 << endl << endl;
		cout << "Desired vel [x,y,z]=" << endl << " " << pkk2 << endl << endl;
		cout << "Velocity output [x,y,z]=" << endl << " " << pkk2 << endl << endl;
		cout << "Consensus output [x,y,z]=" << endl << " " << u_a2 << endl << endl;
		cout << "Tracking output [x,y,z]=" << endl << " " << u_g2 << endl << endl;
		cout << "Total output [x,y,z]=" << endl << " " << u2 << endl << endl;
		cout << "PID [x,y,z]=" << endl << " " << u2PID << endl << endl;
		*/
		
		cout << "nei1=" << endl << " " << nei1 << endl << endl;
		cout << "nei2=" << endl << " " << nei2 << endl << endl;
		cout << "nei3=" << endl << " " << nei3 << endl << endl;
		cout << "nei4=" << endl << " " << nei4 << endl << endl;
		
		cout << "Qd2=" << endl << " " << Qd2 << endl << endl;
		cout << "u_g2=" << endl << " " << u_g2 << endl << endl;
		cout << "pkk2=" << endl << " " << pkk2 << endl << endl;
		cout << "qkk2=" << endl << " " << qkk2 << endl << endl;
	

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
		if (bebop1.cmd_z > speedz) {bebop1.cmd_z = speedz;}
		else if (bebop1.cmd_z < -speedz) {bebop1.cmd_z = -speedz;}
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
		if (bebop2.cmd_z > speedz) {bebop2.cmd_z = speedz;}
		else if (bebop2.cmd_z < -speedz) {bebop2.cmd_z = -speedz;}
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
		if (bebop3.cmd_z > speedz) {bebop3.cmd_z = speedz;}
		else if (bebop3.cmd_z < -speedz) {bebop3.cmd_z = -speedz;}
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
		if (bebop4.cmd_z > speedz) {bebop4.cmd_z = speedz;}
		else if (bebop4.cmd_z < -speedz) {bebop4.cmd_z = -speedz;}
		else {bebop4.cmd_z = bebop4.cmd_z;}
		// yaw velocity control
		if (bebop4.cmd_yaw > speed) {bebop4.cmd_yaw = speed;}
		else if (bebop4.cmd_yaw < -speed) {bebop4.cmd_yaw = -speed;}
		else {bebop4.cmd_yaw = bebop4.cmd_yaw;}

		cmd_vel_bebop1_.linear.x = bebop1.cmd_x;
		cmd_vel_bebop1_.linear.y = bebop1.cmd_y;
		cmd_vel_bebop1_.linear.z = bebop1.cmd_z;
		//cmd_vel_bebop1_.angular.z = bebop1.cmd_z;
		
		cmd_vel_bebop2_.linear.x = bebop2.cmd_x;
		cmd_vel_bebop2_.linear.y = bebop2.cmd_y;
		cmd_vel_bebop2_.linear.z = bebop2.cmd_z;
		//cmd_vel_bebop2_.angular.z = bebop1.cmd_z;
		
		cmd_vel_bebop3_.linear.x = bebop3.cmd_x;
		cmd_vel_bebop3_.linear.y = bebop3.cmd_y;
		cmd_vel_bebop3_.linear.z = bebop3.cmd_z;
		//cmd_vel_bebop3_.angular.z = bebop1.cmd_z;
		
		cmd_vel_bebop4_.linear.x = bebop4.cmd_x;
		cmd_vel_bebop4_.linear.y = bebop4.cmd_y;
		cmd_vel_bebop4_.linear.z = bebop4.cmd_z;
		//cmd_vel_bebop4_.angular.z = bebop1.cmd_z;

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
		
		bebop1Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_x << "" << myTime << "\t" << desired1.pos_x << "\t" << bebop1.pos_x << "\t" << cmd_vel_bebop1_.linear.x << "\t" << u_a1.at<double>(1) << "\t" << u_g1.at<double>(1) << "\t" << u1.at<double>(1) << "\t" << pkk1.at<double>(1) << "\t" << u1PID.at<double>(0) << "\n";
		bebop1Position_y << "" << myTime << "\t" << desired1.pos_y << "\t" << bebop1.pos_y << "\t" << cmd_vel_bebop1_.linear.y << "\t" << u_a1.at<double>(0) << "\t" << u_g1.at<double>(0) << "\t" << u1.at<double>(0) << "\t" << pkk1.at<double>(0) << "\t" << u1PID.at<double>(1) << "\n";
		bebop1Position_z << "" << myTime << "\t" << desired1.pos_z << "\t" << bebop1.pos_z << "\t" << cmd_vel_bebop1_.linear.z << "\t" << u_a1.at<double>(2) << "\t" << u_g1.at<double>(1) << "\t" << u1.at<double>(1) << "\t" << pkk1.at<double>(2) << "\t" << u1PID.at<double>(2) << "\n";
		bebop1Orientation_yaw << "" << myTime << "\t" << desired1.yaw << "\t" << bebop1.yaw << "\t" << cmd_vel_bebop1_.angular.z << "\n"; 
		
		bebop2Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_x << "" << myTime << "\t" << desired2.pos_x << "\t" << bebop2.pos_x << "\t" << cmd_vel_bebop2_.linear.x << "\t" << u_a2.at<double>(1) << "\t" << u_g2.at<double>(1) << "\t" << u2.at<double>(1) << "\t" << pkk2.at<double>(1) << "\t" << u2PID.at<double>(0) << "\n";
		bebop2Position_y << "" << myTime << "\t" << desired2.pos_y << "\t" << bebop2.pos_y << "\t" << cmd_vel_bebop2_.linear.y << "\t" << u_a2.at<double>(0) << "\t" << u_g2.at<double>(0) << "\t" << u2.at<double>(0) << "\t" << pkk2.at<double>(0) << "\t" << u2PID.at<double>(1) << "\n";
		bebop2Position_z << "" << myTime << "\t" << desired2.pos_z << "\t" << bebop2.pos_z << "\t" << cmd_vel_bebop2_.linear.z << "\t" <<u_a2.at<double>(2) << "\t" << u_g2.at<double>(1) << "\t" << u2.at<double>(1) << "\t" << pkk2.at<double>(2) << "\t" << u2PID.at<double>(2) << "\n";
		bebop2Orientation_yaw << "" << myTime << "\t" << desired2.yaw << "\t" << bebop2.yaw << "\t" << cmd_vel_bebop2_.angular.z << "\n";
		
		bebop3Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_x << "" << myTime << "\t" << desired3.pos_x << "\t" << bebop3.pos_y << "\t" << cmd_vel_bebop3_.linear.x << "\t" << u_a3.at<double>(1) << "\t" << u_g3.at<double>(1) << "\t" << u3.at<double>(1) << "\t" << pkk3.at<double>(1) << "\t" << u3PID.at<double>(0) << "\n";
		bebop3Position_y << "" << myTime << "\t" << desired3.pos_y << "\t" << bebop3.pos_x << "\t" << cmd_vel_bebop3_.linear.y << "\t" << u_a3.at<double>(0) << "\t" << u_g3.at<double>(0) << "\t" << u3.at<double>(0) << "\t" << pkk3.at<double>(0) << "\t" << u3PID.at<double>(1) << "\n";
		bebop3Position_z << "" << myTime << "\t" << desired3.pos_z << "\t" << bebop3.pos_z << "\t" << cmd_vel_bebop3_.linear.z << "\t" << u_a3.at<double>(2) << "\t" << u_g3.at<double>(1) << "\t" << u3.at<double>(1) << "\t" << pkk3.at<double>(2) << "\t" << u3PID.at<double>(2) << "\n";
		bebop3Orientation_yaw << "" << myTime << "\t" << desired3.yaw << "\t" << bebop3.yaw << "\t" << cmd_vel_bebop3_.angular.z << "\n";
		
		bebop4Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop4Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop4Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop4Orientation_yaw.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop4Position_x << "" << myTime << "\t" << desired4.pos_x << "\t" << bebop4.pos_y << "\t" << cmd_vel_bebop4_.linear.x << "\t" << u_a4.at<double>(1) << "\t" << u_g4.at<double>(1) << "\t" << u4.at<double>(1) << "\t" << pkk4.at<double>(1) << "\t" << u4PID.at<double>(0) <<  "\n";
		bebop4Position_y << "" << myTime << "\t" << desired4.pos_y << "\t" << bebop4.pos_x << "\t" << cmd_vel_bebop4_.linear.y << "\t" << u_a4.at<double>(0) << "\t" << u_g4.at<double>(0) << "\t" << u4.at<double>(0) << "\t" << pkk4.at<double>(0) << "\t" << u4PID.at<double>(1) <<  "\n";
		bebop4Position_z << "" << myTime << "\t" << desired4.pos_z << "\t" << bebop4.pos_z << "\t" << cmd_vel_bebop4_.linear.z << "\t" << u_a4.at<double>(2) << "\t" << u_g4.at<double>(1) << "\t" << u4.at<double>(1) << "\t" << pkk4.at<double>(2) << "\t" << u4PID.at<double>(2) << "\n";
		bebop4Orientation_yaw << "" << myTime << "\t" << desired4.yaw << "\t" << bebop4.yaw << "\t" << cmd_vel_bebop4_.angular.z << "\n";
	}

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

	bebop4Position_x.close();
	bebop4Position_y.close();
	bebop4Position_z.close();
	bebop4Orientation_yaw.close();	

	printf("hello world!\n");
	return 0;
}
