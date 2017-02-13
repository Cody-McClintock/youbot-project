

//Made by Cody McClintock
//Used with the Kuka Youbot to take user input and 
//serve bottled soda to students for EOH 2017

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <math.h>
#include <cmath>

#include <youbot_ros_hello_world/Num.h>

#define PI 3.14159

ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

char keyboard_input = 'z';
char compare = 'y';
int cnt = 0;

int Pepsi_x_cent = 0;
int Pepsi_y_cent = 0;
int DietMTD_x_cent = 0;
int DietMTD_y_cent = 0;
int Coke_x_cent = 0;
int Coke_y_cent = 0;


using namespace std;

//declaring the functions we will be using
void forward_kin();
void retrieve_drink();
void user_input();
void moveGripper();


//The following is setting up the brics actuator so we can move the youbot arm
//and gripper


// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}

void centroidsCallback(const youbot_ros_hello_world::Num::ConstPtr& msg)
{
	//print received data for debug
	ROS_INFO("I heard: [%i]",msg->Pepsi_x);
	
	cout<<"test1\n\n\n\n\n";

	Pepsi_x_cent = msg->Pepsi_x;
	Pepsi_y_cent = msg->Pepsi_y;

	DietMTD_x_cent = msg->DietMTD_x;
	DietMTD_y_cent = msg->DietMTD_y;

	Coke_x_cent = msg->Coke_x;
	Coke_y_cent = msg->Coke_y;


}






///////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) 
{
	cout<<"test4\n\n\n";

	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::NodeHandle n;

	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 3);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
	sleep(1);

//for the arm
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

//Moves the arm to our home position 
	jointvalues[0] = 2.95;
	jointvalues[1] = 1.12;
	jointvalues[2] = -4.144;
	jointvalues[3] = 1.764;
	jointvalues[4] = 2.95;

	ros::Subscriber sub =n.subscribe("centroids",1000,centroidsCallback);



//	movePlatform();
//	moveArm();
//	moveGripper();
	cout<<"test2\n";
	ros::spin();
	user_input();


	return 0;
}






/////////////////////////////////////////////////////////////////////////
void retrieve_drink()
{
	//for the base
	geometry_msgs::Twist twist;

	//setup the arm joints	
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

	//adjustment vars- will be replaced by opencv data in future
	char adjust = 'z';


	//Theta values that will be inputs
	float t0 = 0.0;
	float t1 = 0.0;
	float t2 = 0.0;
	float t3 = 0.0;
	float t4 = 0.0;

//if diet or mt dew is selected
	if((keyboard_input == 'd') || (keyboard_input == 'm'))	
	{
	//"Left" and "Right" are defined in reference to the camera's reference frame
	//after moving to our home position
	//	i.e. if you are looking in the direction of the youbot end effector
	//	These bottles will be to the left of the youbot

	//Setting to desired angles with reference to our home position
	//Will make the arm the height of the bottle 
	float xinc = 0.0;
	float yinc = -0.1;

	t0 = -1.5707;
	t1 = 1.5707;
	t2 = -0.4;
	t3 = 0;
	t4 = 0;
 
	//Applying theta offsets
		t0 = t0 + 2.950;
		t1 = t1 + 1.125;
		t2 = t2 - 4.144;
		t3 = t3 + 1.764;
		t4 = t4 + 2.950;

	//Moves the arm in reference to our home position
		jointvalues[0] = t0;
		jointvalues[1] = t1;
		jointvalues[2] = t2;
		jointvalues[3] = t3;
		jointvalues[4] = t4;

		msg = createArmPositionCommand(jointvalues);
		armPublisher.publish(msg);


	//Now move the base 
	cout << "Enter adjustments as needed\n";
	cout << "Press 'l' for left\n"; 
	cout << "Press 'r' for right\n";
	cout << "Press 's' to slow youbot\n\n";
	cout << "Press 'x' to stop the Youbot\n";


		while(adjust != 'x')
		{
		//moves the youbot to the left
			
			twist.linear.x = xinc;
	   		twist.linear.y = yinc;

			if(adjust == 'l')
			{
			 	xinc += 0.05;
			}
			
			if(adjust == 'r')
			{
				xinc -= 0.05;	
			}

			if(adjust == 's')
			{
				yinc += 0.05;
			}

			if(adjust == 'f')
			{
				yinc -= 0.05;
			}

			twist.linear.x = xinc;
	   		twist.linear.y = yinc;
			platformPublisher.publish(twist);
			cin >> adjust;


			cout << "Pepsix = "<<Pepsi_x_cent<<"\n";
			cout << "Pepsiy = "<<Pepsi_y_cent<<"\n";

			cout << "DietMTDx = "<<DietMTD_x_cent<<"\n";
			cout << "DietMTDy = "<<DietMTD_y_cent<<"\n";

			cout << "Cokex = "<<Coke_x_cent<<"\n";
			cout << "Cokey = "<<Coke_y_cent<<"\n";



		}
//stop the robot if 'x' is ever pressed
		xinc = 0.0;
		yinc = 0.0;
		twist.linear.x = xinc;
	   	twist.linear.y = yinc;
		platformPublisher.publish(twist);
		
	}

//If sprite or orange is selected
	if((keyboard_input == 's') || (keyboard_input == 'o'))	
	{
	//"Left" and "Right" are defined in reference to the camera's reference frame
	//after moving to our home position
	//	i.e. if you are looking in the direction of the youbot end effector
	//	These bottles will be to the left of the youbot

	//Setting to desired angles with reference to our home position
	//	Will make the arm the height of the bottle 
	float xinc = 0.0;
	float yinc = 0.1;

	t0 = 1.5707;
	t1 = 1.5707;
	t2 = -0.4;
	t3 = 0;
	t4 = 0;
 
	//Applying theta offsets
		t0 = t0 + 2.950;
		t1 = t1 + 1.125;
		t2 = t2 - 4.144;
		t3 = t3 + 1.764;
		t4 = t4 + 2.950;

	//Moves the arm in reference to our home position
		jointvalues[0] = t0;
		jointvalues[1] = t1;
		jointvalues[2] = t2;
		jointvalues[3] = t3;
		jointvalues[4] = t4;

		msg = createArmPositionCommand(jointvalues);
		armPublisher.publish(msg);

	//Now move the base 
	cout << "Enter adjustments as needed\n";
	cout << "Press 'l' for left\n"; 
	cout << "Press 'r' for right\n";
	cout << "Press 's' to slow youbot\n\n";
	cout << "Press 'x' to stop the Youbot\n";


//will be a number of pixels in the future
		while(adjust != 'x')
		{
		//will be comparing the centroid of the object vs image centroid
		//moves the youbot to the left

			if(adjust == 'l')
			{
			 	xinc -= 0.05;
			}
			
			if(adjust == 'r')
			{
				xinc += 0.05;	
			}

			if(adjust == 's')
			{
				yinc -= 0.05;
			}

			if(adjust == 'f')
			{
				yinc += 0.05;
			}

			twist.linear.x = xinc;
	   		twist.linear.y = yinc;
			platformPublisher.publish(twist);
			cin >> adjust;
		}

//stop the robot if 'x' is ever pressed
		xinc = 0.0;
		yinc = 0.0;
		twist.linear.x = xinc;
	   	twist.linear.y = yinc;
		platformPublisher.publish(twist);
		
	}

}






///////////////////////////////////////////////////////////////////////
void forward_kin()
{
	//for the base
	geometry_msgs::Twist twist;

	//setup the arm joints	
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

	//Theta values that will be inputs
	float t0 = 0.0;
	float t1 = 0.0;
	float t2 = 0.0;
	float t3 = 0.0;
	float t4 = 0.0;

	cout << "Enter desired angles: \n";
	cin >> t0 >> t1 >> t2 >> t3 >> t4 ;
	
	//Applying theta offsets
	t0 = t0 + 2.950;
	t1 = t1 + 1.125;
	t2 = t2 - 4.144;
	t3 = t3 + 1.764;
	t4 = t4 + 2.950;

//Moves the arm in reference to our home position
	jointvalues[0] = t0;
	jointvalues[1] = t1;
	jointvalues[2] = t2;
	jointvalues[3] = t3;
	jointvalues[4] = t4;

		msg = createArmPositionCommand(jointvalues);
		armPublisher.publish(msg);

}



///////////////////////////////////////////////////////////////////////
//get user function
void user_input() 
{
//for the base
	geometry_msgs::Twist twist;

//for the arm
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);


	cout << "Which soda would you like?\n";
	cout << "Press 'd' for Diet\n";
	cout << "Press 's' for Sprite\n";
	cout << "Press 'o' for Orange Fanta\n";
   	cout << "Press 'm' for Mt. Dew\n\n";
	cout << "Press 'f' for Forward Kinematics\n";

    twist.linear.x = 0;
	twist.linear.y = 0;	
	twist.angular.z = 0;
    platformPublisher.publish(twist);

	
	cin >> keyboard_input;

	if(keyboard_input =='f')
	{
		forward_kin();
	}

	else(retrieve_drink());

/*
	switch(keyboard_input)
	{
//cases depending on input

		case 'd':
			{
				twist.linear.x = 0.5;
				twist.linear.y = 0.0;
				platformPublisher.publish(twist);
				ros::Duration(1).sleep();
				break;
			}


		case 's':
			{
				twist.linear.x = -0.5;
               	twist.linear.y = 0.5;
				platformPublisher.publish(twist);
				ros::Duration(1).sleep();
				break;
			}


		case 'm':
			{
				twist.linear.x = 0;
               	twist.linear.y = -0.5;
				platformPublisher.publish(twist);
				ros::Duration(1).sleep();
				break;
			}




		case 'f':
			{
				forward_kin();
			}

	}
*/
}

/*
	// forward
	twist.linear.x = 0.05;  // with 0.05 m per sec
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// backward
	twist.linear.x = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the left
	twist.linear.x = 0;
	twist.linear.y = 0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the right
	twist.linear.y = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// stop
	twist.linear.y = 0;
	platformPublisher.publish(twist);
}
*/



/*
// move platform a little bit back- and forward and to the left and right
void movePlatform() {
	geometry_msgs::Twist twist;

	// forward
	twist.linear.x = 0.05;  // with 0.05 m per sec
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// backward
	twist.linear.x = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the left
	twist.linear.x = 0;
	twist.linear.y = 0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the right
	twist.linear.y = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// stop
	twist.linear.y = 0;
	platformPublisher.publish(twist);
}
*/



///////////////////////////////////////////////////////////////////////
// open and close gripper
void moveGripper() {
	brics_actuator::JointPositions msg;
	
	// open gripper
	msg = createGripperPositionCommand(0.011);
	gripperPublisher.publish(msg);

	ros::Duration(3).sleep();

	// close gripper
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
}






