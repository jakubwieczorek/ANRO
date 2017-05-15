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
}

int main(int argc, char **argv)
{
        // Inicjalizacja ros-a
        ros::init(argc,argv,"KDL_DKIN");
        ros::NodeHandle nh;
        ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedKDL", 1000);
        ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
        ros::Rate rate(30);


        double a1=2.0,a2=1.0;
        double joint1_lower=-3.14,joint1_upper=3.14;
        double joint2_lower=-2.35619,joint2_upper=2.35619;
        double joint3_lower=0,joint3_upper=3.0;

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
                ros::spinOnce();            // Pobranie informacji z węzła Joint_State_Publishera
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

                double x,y,z=0; // Współrzędne końcówki
                KDL::Chain chain; // Stworzenie łańcucha kinematycznego

                double teta1=desired_position[0];
                double teta2 = desired_position[1];
                double d3 = desired_position[2];

                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a1, 0, 0, teta1)));
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, 0, 0, teta2)));
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, 0, d3, 0)));
                x = chain.getSegment(2).getFrameToTip().p.data[0];
                y = chain.getSegment(2).getFrameToTip().p.data[1];
                z = chain.getSegment(2).getFrameToTip().p.data[2];

                geometry_msgs::PoseStamped to_send;
                to_send.header.frame_id="arm3";
                to_send.pose.position.x=x;
                to_send.pose.position.y=y+0.8;
                to_send.pose.position.z=0;
                to_send.pose.orientation.w=0.707;
                to_send.pose.orientation.x=0.0;
                to_send.pose.orientation.y=0.0;
                to_send.pose.orientation.z=-0.707;


                pub.publish(to_send);
                rate.sleep();
        }

        return 0;
}
