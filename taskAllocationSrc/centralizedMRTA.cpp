#include "centralizedMRTA.h"


namespace ns_taskAllocation {
	std::vector<bgeo::DRing> cenTaskAllocation(std::vector<bgeo::DPoint> const & vRobPosition, std::vector<bgeo::DRing> const & vTaskArea)
	{

		size_t robNum = vRobPosition.size();
		std::vector<bgeo::DRing> res;
		for (size_t i = 0; i < robNum; i++)
		{
			res.push_back(vTaskArea[0]);
		}
		return res;
	}
}
