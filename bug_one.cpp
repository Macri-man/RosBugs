
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>

#include <tf/transform_broadcaster.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <string>


void laserRange(const sensor_msgs::LaserScan::ConstPtr& laser);
void groundTruth(const  nav_msgs::Odometry::ConstPtr& ground);

bool isFacinglocation();
void facingLocation();

void followWall();

int distanceFromclosest();
int distanceFromtarget();
double bearing();
bool done();
bool obsticle();
void move();

bool isClosest();
bool isFarthest();

typedef struct Position{
	int x;
	int y;
}Position;

typedef struct Robot{
	bool followWall;
	bool followLeft;
	bool followRight;
	bool around;
	int x;
	int y;
	double head;
}Robot;


typedef struct Obsticle{
	double rangeMin;
	double rangeInc;
	double left;
	double front;
	double right;
	double greatLeft;
	double greatRight;
}Obsticle;

Position endPos;
Robot robo;
Obsticle obst;

Position closestPos;
Position hitWallPos;

nav_msgs::Odometry ground;
sensor_msgs::LaserScan laser;
geometry_msgs::Twist base_cmd;

bool inCircle(Position pos);

void read(std::istream &in){
	in >> endPos.x;
	in.get();
	in >> endPos.y;
	fprintf(stderr,"x:%d,y:%d\n",endPos.x,endPos.y);
}

void printValues(std::string val1,int val2){
	std::cout << val1 << val2 << "\n"; 
}

void laserRange(const sensor_msgs::LaserScan::ConstPtr& laser){
	int mid=laser->ranges.size()/2;
	int left=mid+200;
	int right=mid-200;
	int greatLeft=left+200;
	int greatRight=right+200;
	obst.rangeMin=laser->angle_min;
	obst.rangeInc=laser->angle_increment;
	obst.left=laser->ranges[left];
	obst.front=laser->ranges[mid];
	obst.right=laser->ranges[right];
	obst.greatLeft=laser->ranges[greatLeft];
	obst.greatRight=laser->ranges[greatRight];
	fprintf(stderr,"ObstGreatLeft:%f ObstLeft:%f ObstFront:%f ObstRight:%f ObstGreatRight:%f\n",obst.greatLeft,obst.left,obst.front,obst.right,obst.greatRight);
}


void groundTruth(const  nav_msgs::Odometry::ConstPtr& ground){
	tf::Pose pose;
	tf::poseMsgToTF(ground->pose.pose,pose);
	

		
	int xPos=ground->pose.pose.position.x;
	int yPos=ground->pose.pose.position.y;
	
	double yaw=tf::getYaw(pose.getRotation());

	robo.x=xPos;
	robo.y=yPos;
	robo.head=yaw;

	fprintf(stderr,"Robo x:%d y:%d head:%f\n",robo.x,robo.y,robo.head);
	std::cout << std::boolalpha << "robo.followLeft:" << robo.followLeft <<"\n";
	std::cout << std::boolalpha << "robo.followRight:" << robo.followRight << "\n";
	std::cout << std::boolalpha << "around:" << robo.around << "\n";
}

double bearing(){
	return atan2(endPos.y-robo.y,endPos.x-robo.x);
}

bool isFacingLocation(){
	double num=bearing();
	return ((num-.05 <= robo.head) && (robo.head <= num+.05)); 
}

void facingLocation(){
	while(!isFacingLocation()){
		if(bearing()-robo.head>0){
			base_cmd.angular.z=-0.25;
		}else{
			base_cmd.angular.z=0.25;
		}
	}
}

bool done(){
	return distanceFromtarget()<.01;
}

int distanceFromtarget(){
	return sqrt(((endPos.x-robo.x)*(endPos.x-robo.x))+((endPos.y-robo.y)*(endPos.y-robo.y)));
}

int distanceFromclosest(){
	return sqrt(((endPos.x-closestPos.x)*(endPos.x-closestPos.x))+((endPos.y-closestPos.y)*(endPos.y-closestPos.y)));
}

bool isClosest(){
	return (distanceFromclosest()>distanceFromtarget());
}

bool isFarthest(){
	return (distanceFromclosest()<distanceFromtarget());
}

bool inCircle(Position pos){
	return ((robo.x-pos.x)*(robo.x-pos.x))+((robo.y-pos.y)*(robo.y-pos.y))<=1;
}

bool obsticle(){
		if(isFacingLocation() && obst.front<=1){
			if(obst.left < obst.right){
				robo.followLeft=true;
			}else{
				robo.followRight=true;
			}
			closestPos.x=robo.x;
			closestPos.y=robo.y;
			hitWallPos.x=robo.x;
			hitWallPos.y=robo.y;
			return true;
		}
}

int getIndex(){
	return (robo.head-obst.rangeMin)/obst.rangeInc;
}

void move(){
	fprintf(stderr,"MOVING\n");
	if(!isFacingLocation()){
		if(bearing()>robo.head){
			base_cmd.angular.z=0.5;
			base_cmd.linear.x=0.5;
		}else{
			base_cmd.angular.z=-0.5;
			base_cmd.angular.x=0.5;
		}
	}else{
		base_cmd.linear.x=1;
	}
}

void followWall(){
	fprintf(stderr,"FOLLOWINGWALL\n");
	std::cout << "ClosestPos x:" << closestPos.x << "\n";
	std::cout << "ClosestPos y:" << closestPos.y << "\n";
	if((isFarthest()) && inCircle(hitWallPos)){
		robo.around=true;
	}
	if(isClosest()){
		closestPos.x=robo.x;
		closestPos.y=robo.y;
	}
	if(robo.followLeft){
		if(robo.around && inCircle(closestPos)){
				robo.followLeft=false;
				robo.followWall=false;
				base_cmd.angular.z=0.5;
				base_cmd.linear.x=1;
		}else if(obst.left<=.5){
			base_cmd.angular.z=-1;
			base_cmd.linear.x=0.1;
		}else if(obst.left <=1.3){
			base_cmd.linear.x=0.5;
			base_cmd.angular.z=-0.25;
		}else if(obst.front<=1.3 && obst.left <=1.3){
			base_cmd.linear.x=0.5;
			base_cmd.angular.z=-0.5;
		}else if(obst.left>=1.3){
			base_cmd.angular.z=0.5;
			base_cmd.linear.x=0.5;
		}else{
			base_cmd.linear.x=1;
			base_cmd.angular.z=0.25;
		}
	}else if(robo.followRight){
		if(robo.around && inCircle(closestPos)){
				robo.followRight=false;
				robo.followWall=false;
				base_cmd.angular.z=-0.5;
				base_cmd.linear.x=1;
		}else if(obst.right<=.5){
			base_cmd.angular.z=1;
			base_cmd.linear.x=.1;
		}else if(obst.right <=1.3){
			base_cmd.linear.x=0.5;
			base_cmd.angular.z=0.25;
		}else if(obst.front<=1.3 && obst.right <=1.3){
			base_cmd.linear.x=0.5;
			base_cmd.angular.z=.5;
		}else if(obst.right>=1.3){
			base_cmd.angular.z=-0.5;
			base_cmd.linear.x=0.5;
		}else{
			base_cmd.linear.x=1;
			base_cmd.angular.z=-0.25;
		}
	}else{
		base_cmd.linear.x=1;
		robo.followWall=false;
	}
}

int main(int argc, char **argv){
	/// Name your node
	ros::init(argc, argv, "bug_one");
	/// Every ros node needs a node handle, similar to your usual  file handle.
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//nav_msgs::Odometry ground;
	//sensor_msgs::LaserScan laser;
	//geometry_msgs::Twist base_cmd;
	ros::Subscriber ground_truth = nh.subscribe("/base_pose_ground_truth",100,&groundTruth);
 	ros::Subscriber base_scan = nh.subscribe("/base_scan",100,&laserRange);	
	// User input
	char cmd[50];


	robo.followWall=false;	
	robo.followLeft=false;
	robo.followRight=false;

	std::cout << "What are the cooridnate you want the robot to go too x,y.\n";
	read(std::cin);
	/// The main loop will run at a rate of 10Hz, i.e., 10 times per second.
	ros::Rate loop_rate(10);
	/// Standard way to run ros code. Will quite if ROS is not OK, that is, the master is dead.
	while (ros::ok()){

		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
		ros::spinOnce();
		fprintf(stderr,"distance:%d \n",distanceFromtarget());
		fprintf(stderr,"bearing:%f head:%f\n",bearing(),robo.head);
		std::cout << std::boolalpha << "facing:" <<isFacingLocation() << "\n";
		std::cout << std::boolalpha << "followWall" << robo.followWall << "\n";
		
		if(robo.followWall){
			followWall();
		}else{
			robo.followWall=obsticle();
			move();
		}
		if(done()){
			std::cout << "BUG One COMPLETE\n";
			break;
		}
		/// Here's where we publish the actual message.
		cmd_vel_pub_.publish(base_cmd);
		/// Sleep for as long as needed to achieve the loop rate.
		loop_rate.sleep();
	}
	return 0;
}

