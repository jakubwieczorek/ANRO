#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "lab4/oint_control_srv.h"
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <nav_msgs/Path.h>

double const PI=M_PI;
double f=10;
double Tp=1/f;
double x,y,z,xo=0,yo=0,zo=0,wo=1;
double x0=0;
double yy0=0,z0=0,xo0=0,yo0=0,zo0=0,wo0=0;
double px,py,pz=0,ox=0,oy=0,oz,ow,czas;
bool czyInterpolujemy = true;
double fi,kx,ky,kz;
double ox1=0.0,oy1=0.0,oz1=0.0,ow1=1.0;
double xpop=0;
double ypop=0;
double zpop=0;

int n=0; // Liczba iteracji pętli
int k=0; // Indeks iteracji pętli
double t0,t1,t2,t3,t4,t5,xKat,yKat,zKat;
bool pierwszeUruchomienie=true;


// Procedura do obsługi żądania
bool procedura(lab4::oint_control_srv::Request &req, lab4::oint_control_srv::Response &res)
{
	if(req.time <= 0)
	{
		// Wysłanie odpowiedzi negatywnej do klienta
		res.wynik="Podaj dodatni czas trwania ruchu.";
		ROS_WARN("Podano nieprawidlowy czas ruchu.");
		return true;
	}
	px=req.x;
	py=req.y;
	pz=req.z;
	
	
	xKat=req.xkat;
	yKat=req.ykat;
	zKat=req.zkat;
	
	t0 = std::cos(zKat * 0.5);
	t1 = std::sin(zKat * 0.5);
	t2 = std::cos(xKat * 0.5);
	t3 = std::sin(xKat * 0.5);
	t4 = std::cos(yKat * 0.5);
	t5 = std::sin(yKat * 0.5);

	ow = t0 * t2 * t4 + t1 * t3 * t5;
	ox = t0 * t3 * t4 - t1 * t2 * t5;
	oy = t0 * t2 * t5 + t1 * t3 * t4;
	oz = t1 * t2 * t4 - t0 * t3 * t5;
	
	czas=req.time;
	czyInterpolujemy=true;	
	n=czas/Tp; 
	k=1;

	x0=x;
	yy0=y;
	z0=z;
	
	ox1=xo;
	oy1=yo;
	oz1=zo;
	ow1=wo;
	
	fi=2*acos(ow);
	double sinfipol=sin(fi/2);
	if(sinfipol!=0)
	{
		kx=ox/sinfipol;
		ky=oy/sinfipol;
		kz=oz/sinfipol;
	}
	else kx=ky=kz=0;
	
	pierwszeUruchomienie=false;
	
	return true;
}



int main(int argc, char **argv)
{
	// Inicjalizacja ros-a
	ros::init(argc,argv,"oint");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStamped", 1000);
	ros::ServiceServer serwer = nh.advertiseService("oint_control_srv",procedura);
	ros::Rate rate(f);

	geometry_msgs::PoseStamped doWyslania;
	doWyslania.header.frame_id="/world";
	nav_msgs::Path trajektoria;

	ros::Publisher pub2=nh.advertise<nav_msgs::Path>("/trajektoria", 1);
	while(ros::ok())
	{
		ros::spinOnce();
		if(pierwszeUruchomienie == true){
			doWyslania.pose.position.x=0;
			doWyslania.pose.position.y=0;
			doWyslania.pose.position.z=0;
			doWyslania.pose.orientation.w=1;
			doWyslania.pose.orientation.x=0;
			doWyslania.pose.orientation.y=0;
			doWyslania.pose.orientation.z=0;
			pub.publish(doWyslania);
			xpop=x;
			ypop=y;
			zpop=z;
		}
		if(!czyInterpolujemy) continue;

		if(n==0 && k==1)
		{
			x=px;
			y=py;
			z=pz;
			xo=ox;
			yo=oy;
			zo=oz;
			wo=ow;
		
		}
		else if(k<1 || n < 1)
		{
			czyInterpolujemy=false;
		}

		else if (czyInterpolujemy)
		{
			x=x0+((px-x0)/czas)*k*Tp;
			y=yy0+((py-yy0)/czas)*k*Tp;
			z=z0+((pz-z0)/czas)*k*Tp;
			
			double ox2=kx*sin(k*fi/(2*n));
			double oy2=ky*sin(k*fi/(2*n));
			double oz2=kz*sin(k*fi/(2*n));
			double ow2=cos(k*fi/(2*n));
			
			xo=ow1*ox2+ox1*ow2+oy1*oz2-oz1*oy2;
			yo=ow1*oy2+oy1*ow2+oz1*ox2-ox1*oz2;
			zo=ow1*oz2+oz1*ow2+ox1*oy2-oy1*ox2;
			wo=ow1*ow2-ox1*ox2-oy1*oy2-oz1*oz2;			
		}

		k++;
		if (k>=n)
		{	
			x=px;
			y=py;
			z=pz;
			n=0;
			k=0;
			czyInterpolujemy=false;

			double ox2=kx*sin(fi/2);
			double oy2=ky*sin(fi/2);
			double oz2=kz*sin(fi/2);
			double ow2=cos(fi/2);
			
			xo=ow1*ox2+ox1*ow2+oy1*oz2-oz1*oy2;
			yo=ow1*oy2+oy1*ow2+oz1*ox2-ox1*oz2;
			zo=ow1*oz2+oz1*ow2+ox1*oy2-oy1*ox2;
			wo=ow1*ow2-ox1*ox2-oy1*oy2-oz1*oz2;
		}
	
		geometry_msgs::PoseStamped doWyslania;
		doWyslania.header.frame_id="/world";
		doWyslania.pose.position.x=x;
		doWyslania.pose.position.y=y;
		doWyslania.pose.position.z=z;
		doWyslania.pose.orientation.w=wo;
		doWyslania.pose.orientation.x=xo;
		doWyslania.pose.orientation.y=yo;
		doWyslania.pose.orientation.z=zo;


	
	trajektoria.poses.push_back(doWyslania);
	trajektoria.header.frame_id="/world";
	trajektoria.header.stamp=ros::Time::now();

		pub2.publish(trajektoria);
		pub.publish(doWyslania);
		rate.sleep();
	}

	return 0;
}
