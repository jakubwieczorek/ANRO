#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

double desired_position[3];
double const PI=M_PI;
bool is_joint_state_active=false;

// Funkcja do pobrania wartości zadanych
void callback(const sensor_msgs::JointStateConstPtr &msg)
{
        is_joint_state_active=true;
        for (int i = 0; i < 3; i++)
        {
                desired_position[i]=msg->position[i];
        }
	//desired_position[0]+=PI/5;
}

int main(int argc, char **argv)
{
        // Inicjalizacja ros-a
        ros::init(argc,argv,"NONKDL_DKIN");
        ros::NodeHandle nh;
        ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedNONKDL", 1000);
        ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
        ros::Rate rate(30);

        double a1=2.0,a2=1.0;
        double joint1_lower=-3.14,joint1_upper=3.14;
        double joint2_lower=-2.35619,joint2_upper=2.35619;
        double joint3_lower=0,joint3_upper=3.0;
	
	double t1 = desired_position[0];
	double t2 = desired_position[1];
	double t3 = desired_position[2];

        // Pobranie danych z serwera parametrów
        if(!nh.getParam("/arm1",a1)) ROS_WARN("Błąd pobrania parametrów z serwera!");
        if(!nh.getParam("/arm2",a2)) ROS_WARN("Błąd pobrania parametrów z serwera!");
        if(!nh.getParam("/joint1_lower",joint1_lower)) ROS_WARN("Błąd pobrania parametrów z serwera!");
        if(!nh.getParam("/joint1_upper",joint1_upper)) ROS_WARN("Błąd pobrania parametrów z serwera!");
        if(!nh.getParam("/joint2_lower",joint2_lower)) ROS_WARN("Błąd pobrania parametrów z serwera!");
        if(!nh.getParam("/joint2_upper",joint2_upper)) ROS_WARN("Błąd pobrania parametrów z serwera!");
        if(!nh.getParam("/joint3_lower",joint3_lower)) ROS_WARN("Błąd pobrania parametrów z serwera!");
        if(!nh.getParam("/joint3_upper",joint3_upper)) ROS_WARN("Błąd pobrania parametrów z serwera!");

        while(ros::ok())
        {
                ros::spinOnce();            // Pobranie informacji z węzła Joint_State_Publisher
                if(!is_joint_state_active)  // Sprawdzamy, czy już odebraliśmy jakieś dane od joint_state_publishera
                        continue;

                // Sprawdzanie, czy spełnione są ograniczenia kinematyczne manipulatora
                if(desired_position[0] > joint1_upper || desired_position[0] < joint1_lower){
                        ROS_ERROR("Osiągnięto skrajne położenie!");
                        continue;
                }
                if(desired_position[1] > joint2_upper || desired_position[1] < joint2_lower){
                        ROS_ERROR("Osiągnięto skrajne położenie!");
                        continue;
                }
                if(desired_position[2] > joint3_upper || desired_position[2] < joint3_lower){
                        ROS_ERROR("Osiągnięto skrajne położenie!");
                        continue;
                }

		double t1 = desired_position[0];
        	double t2 = desired_position[1];
        	double t3 = desired_position[2];
		//a2=0;


                // Obliczanie parametrów do wysłania
                double x = 1;//a2*cos(desired_position[0]-desired_position[1])+a1*cos(desired_position[0]);//+cos(t1)*sin(t2)+sin(t1)*cos(t2)+a2*cos(t1)+a1;//a2*cos(desired_position[0]-desired_position[1])+a1*cos(desired_position[0]);
                double y = -1.4;//-a2*sin(desired_position[0]-desired_position[1])-a1*sin(desired_position[0]);//+sin(t1)*sin(t2)-cos(t1)*cos(t2)+a2*sin(t1);//-a2*sin(desired_position[0]-desired_position[1])-a1*sin(desired_position[0]);
                double z = -desired_position[2];

                geometry_msgs::PoseStamped to_send;
                to_send.header.frame_id="link2";
                to_send.pose.position.x=x;
                to_send.pose.position.y=y;
                to_send.pose.position.z=z;
                to_send.pose.orientation.w=-0.707;//sin((PI/2+desired_position[0]-desired_position[1])/2);
                to_send.pose.orientation.x=0;
                to_send.pose.orientation.y=0;
                to_send.pose.orientation.z=-0.707;//cos((PI/2+desired_position[0]-desired_position[1])/2);


                pub.publish(to_send);
                rate.sleep();
        }

        return 0;
}

