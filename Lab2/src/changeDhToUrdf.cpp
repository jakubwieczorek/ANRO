#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <math.h>
#include <stdlib.h>

using namespace std;

int main()
{
	KDL::Chain chain; // Stworzenie łańcucha kinematycznego
	
	// Deklaracje zmiennych i stałych
	double const PI = M_PI;
	double a1 = 1, a2 =3, teta1=PI/5, teta2=PI , d3=3;
//double a1 = 2, a2 = 1, teta1=PI/3, teta2=PI/8 , d3=2; // Domyślne wartości
	double zlacze[3], koncowka[3];
	double roll, pitch, yaw; // Szukane parametry

	// Dodanie segmentów - węzłów
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a1, 0, 0, teta1)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, PI, 0, teta2)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, 0, d3, 0)));
	
	
	for (int i = 0; i < 3; i++)
	{
			for(int j = 0; j < 3; j++)
			{
				zlacze[j] = chain.getSegment(i).getFrameToTip().p.data[j]; // Zwraca pozycję końcówki segmentu nadrzędnego
			}

		chain.getSegment(i).getFrameToTip().M.GetRPY(roll, pitch, yaw);
	
	cout << "Zlacze " << i << " xyz : " << zlacze[0] << ", " <<zlacze[1] << ", " << zlacze[2] << endl;		
	cout << "Zlacze " << i << " rpy : " << roll << ", " << pitch << ", " << yaw << endl;
	}

	return 0;
}
