


#include "centralizedMRTA.h"
#include <iostream>
#include "bgeometry.h"



void main()
{
	bgeo::DPoint pnt1(2, 3);
	bgeo::DPoint pnt2(3, 4);
	bgeo::DPoint pnt3(2, 3);
	bgeo::DPoint pnt4(3, 4);

	std::vector<bgeo::DPoint> vRobPostion;

	vRobPostion.push_back(pnt1);
	vRobPostion.push_back(pnt2);
	vRobPostion.push_back(pnt3);
	vRobPostion.push_back(pnt4);
//	vRobPostion.push_back(pnt1);


	bgeo::DRing myRing;

	myRing.push_back(pnt1);
	myRing.push_back(pnt2);
	myRing.push_back(pnt3);
	myRing.push_back(pnt4);
	myRing.push_back(pnt1);

	std::vector<bgeo::DRing> vTaskArea;
	vTaskArea.push_back(myRing);

	auto taskAllocationRes = ns_ta::cenTaskAllocation(vRobPostion, vTaskArea);
	for (size_t i = 0; i < taskAllocationRes.size(); i++)
	{
		std::cout << "area" << taskAllocationRes[i] << std::endl;
	}
	int i;
	std::cin >> i;
}