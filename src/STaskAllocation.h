#pragma once
#include "Point.h"
#include <vector>
namespace areaCut {
	using std::vector;
	//void StaskAllocation()
	//
	//����ʽ���������������亯��STaskAllocation
	//- ���ܣ�ʵ����������Ļ���
	//- ���룺*regionRes ���������ɵ�����;
	//		AgentAllocationRes �����������Ľ��;	
	//		regionScale �����Ѿ���������
	//      Size_regionScale �������������
	//      AgentNum  �����ƶ������������
	//-�������������Ƿ�ɹ� 1����ɹ�
	//������Ŵ����������
	int STaskAllocation(vector<vector<Point>> &res,
		const vector<size_t> AgentAllocationRes,
		const vector<vector<Point>>regionScale,
		const int Size_regionScale,
		const int AgentNum);
}
