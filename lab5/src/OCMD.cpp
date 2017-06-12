#include "ros/ros.h"
#include "lab4/oint_control_srv.h"
#include <math.h>
#include <iostream>

double const PI=M_PI;
double f=10;

int main(int argc, char **argv)
{
	// Inicjalizacja ros-a
	ros::init(argc,argv,"OCMD");
	ros::NodeHandle nh;
	ros::Rate rate(f);
	ros::ServiceClient klient = nh.serviceClient<lab4::oint_control_srv>("oint_control_srv");
	lab4::oint_control_srv srv; // Wiadomość do wysłania	

	std::cout << "Wybierz rodzaj ruchu (wpisz wlasciwa cyfre i nacisnij ENTER): " << std::endl;
	std::cout << "1 - ruch po prostokacie" << std:: endl;
	std::cout << "2 - ruch po elipsie" << std::endl;
	int wybor;
	std::cin >> wybor;

	int licznik; // Zmienna do odmierzania czasu
	int wierzcholek; // Numer wierzcholka
	double x; // Zmienna dotycząca elipsy
	int kierunek; // Kierunek poruszania się elipsy 1-lewo, 2-prawo
	if(wybor==1) // Ruch po prostokacie
		{	
			wierzcholek=1;
			srv.request.x=3;
			srv.request.y=0;
			srv.request.z=-1.0;
			srv.request.time=0.1;
			klient.call(srv);
			licznik=100;
		}
		else // Elipsa
		{
			srv.request.x=3;
			srv.request.y=0;
			srv.request.z=-1.0;
			srv.request.time=0.1;
			x=3.0;
			kierunek=2;
			klient.call(srv);
		}

	while(ros::ok())
	{
		if(wybor==1) // Ruch po prostokacie
		{
			if(wierzcholek==1 && licznik==100)
			{
				srv.request.x=0.0;
				srv.request.y=3.0;
				srv.request.z=-1.0;
				srv.request.time=10;
				klient.call(srv);
			}
			if(wierzcholek==1 && licznik==0)
			{
				wierzcholek=2;
				licznik=100;
				srv.request.x=-3.0;
				srv.request.y=0.0;
				srv.request.z=-1.0;
				srv.request.time=12;
				klient.call(srv);
			}
			if(wierzcholek==2 && licznik==0)
			{
				wierzcholek=3;
				licznik=100;
				srv.request.x=0.0;
				srv.request.y=-3.0;
				srv.request.z=-1.0;
				srv.request.time=10;
				klient.call(srv);
			}
			if(wierzcholek==3 && licznik==0)
			{
				wierzcholek=0;
				licznik=120;
				srv.request.x=3.0;
				srv.request.y=0.0;
				srv.request.z=-1.0;
				srv.request.time=12;
				klient.call(srv);
			}
			if(wierzcholek==0 && licznik==0)
			{
				wierzcholek=1;
				licznik=100;
				srv.request.x=0.0;
				srv.request.y=3.0;
				srv.request.z=-1.0;
				srv.request.time=10;
				klient.call(srv);
			}

			licznik--;
		}
		else // Elipsa
		{
			double y;
			if(kierunek==1)
			{
				if(x>-3.0) // Nie dotarliśmy do lewego krańca elipsy
				{	
					if (x-0.05>=-3.0)
						x=x-0.05;
					else
						x=-3.0;
					y=-sqrt(5*(1-(x*x)/9));
					srv.request.x=x;
					srv.request.y=y;
					klient.call(srv);
				}
				else // x==-4.0
				{
					kierunek=2;	
				}
			}
			if(kierunek==2)
			{
				if(x<3.0) // Nie dotarliśmy do lewego krańca elipsy
				{
					if(x+0.05<=3.0)
						x=x+0.05;
					else
						x=3.0;
					y=sqrt(5*(1-(x*x)/9));
					srv.request.x=x;
					srv.request.y=y;
					klient.call(srv);
				}
				else // x==4.0
				{
					kierunek=1;	
					x=x-0.05;
					y=-sqrt(5*(1-(x*x)/9));
					srv.request.x=x;
					srv.request.y=y;
					klient.call(srv);
				}
			}
		}		
		
		rate.sleep();
	}
}
