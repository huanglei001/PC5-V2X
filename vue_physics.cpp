/*
* =====================================================================================
*
*       Filename:  vue_physics.cpp
*
*    Description:  �����������㲿��ʵ��
*
*        Version:  1.0
*        Created:
*       Revision:
*       Compiler:  VS_2015
*
*         Author:  HCF
*            LIB:  ITTC
*
* =====================================================================================
*/

#include"vue.h"
#include"vue_physics.h"
#include"gtt.h"
#include"gtt_urban.h"
#include"gtt_highspeed.h"
#include"config.h"
#include"imta.h"
#include"function.h"
#include"reflect/context.h"

using namespace std;

int vue_physics::s_ue_count = 0;

vector<vector<double>> vue_physics::s_pl_all(0);

vector<vector<double>> vue_physics::s_distance_all(0);

vector<vector<double>> vue_physics::s_distance_all_before(0);

vector<vector<int>> vue_physics::s_LOS(0);

list<list<list<double>>> vue_physics::hafterFFT(0);

vector<vector<double>> vue_physics::s_sn_DS(0);

vector<vector<double>> vue_physics::s_sn_ASA(0);

vector<vector<double>> vue_physics::s_sn_ASD(0);

vector<vector<double>> vue_physics::s_sn_SF(0);

int vue_physics::get_ue_num() {
	return s_ue_count;
}

void vue_physics::set_pl(int i, int j, double t_pl) {
	if (i < j) {
		s_pl_all[i][j] = t_pl;
	}
	else {
		s_pl_all[j][i] = t_pl;
	}
}

void vue_physics::set_sn_DS(int i, int j, double t_DS)
{
	if (i < j)
		s_sn_DS[i][j] = t_DS;
	else
		s_sn_DS[j][i] = t_DS;
}

double vue_physics::get_sn_DS(int i, int j)
{
	if (i < j)
		return s_sn_DS[i][j];
	else
		return s_sn_DS[j][i];
}

void vue_physics::set_sn_ASA(int i, int j, double t_ASA)
{
	if (i < j)
		s_sn_ASA[i][j] = t_ASA;
	else
		s_sn_ASA[j][i] = t_ASA;
}

double vue_physics::get_sn_ASA(int i, int j)
{
	if (i < j)
		return s_sn_ASA[i][j];
	else
		return s_sn_ASA[j][i];
}

void vue_physics::set_sn_ASD(int i, int j, double t_ASD)
{
	if (i < j)
		s_sn_ASD[i][j] = t_ASD;
	else
		s_sn_ASD[j][i] = t_ASD;
}

double vue_physics::get_sn_ASD(int i, int j)
{
	if (i < j)
		return s_sn_ASD[i][j];
	else
		return s_sn_ASD[j][i];
}

void vue_physics::set_sn_SF(int i, int j, double t_SF)
{
	if (i < j)
		s_sn_SF[i][j] = t_SF;
	else
		s_sn_SF[j][i] = t_SF;
}

double vue_physics::get_sn_SF(int i, int j)
{
	if (i < j)
		return s_sn_SF[i][j];
	else
		return s_sn_SF[j][i];
}



//std::list<double> vue_physics::get_haf(int i, int j) {
//	if (i < j) {
//		//return hafterFFT[i][j];
//	}
//	else {
//		//return hafterFFT[j][i];
//	}
//}

double vue_physics::get_pl(int i, int j) {
	if (i < j) {
		return s_pl_all[i][j];
	}
	else {
		return s_pl_all[j][i];
	}
}

void vue_physics::set_distance(int i, int j, double t_distance) {
	if (i < j) {
		s_distance_all[i][j] = t_distance;
	}
	else {
		s_distance_all[j][i] = t_distance;
	}
}

void vue_physics::set_distance_before(int i, int j, double t_distance) {
	if (i < j) {
		s_distance_all_before[i][j] = t_distance;
	}
	else {
		s_distance_all_before[j][i] = t_distance;
	}
}

void vue_physics::set_LOS(int i, int j, int los) {
	if (i < j) {
		s_LOS[i][j] = los;
	}
	else {
		s_LOS[j][i] = los;
	}
}

double vue_physics::get_distance(int i, int j) {
	if (i < j) {
		return s_distance_all[i][j];
	}
	else {
		return s_distance_all[j][i];
	}
}

double vue_physics::get_distance_before(int i, int j) {
	if (i < j) {
		return s_distance_all_before[i][j];
	}
	else {
		return s_distance_all_before[j][i];
	}
}

int vue_physics::get_LOS(int i, int j) {
	if (i < j) {
		return s_LOS[i][j];
	}
	else {
		return s_LOS[j][i];
	}
}

vue_physics::vue_physics() {

}

vue_physics::~vue_physics() {
	
}

void vue_physics::set_slot_time_idx(int t_slot_time_idx) {
	this->m_slot_time_idx = t_slot_time_idx;
}

int vue_physics::get_slot_time_idx() {
	return m_slot_time_idx;
}

void vue_physics::update_location_highspeed() {
	auto p = (gtt_highspeed*)((gtt*)context::get_context()->get_bean("gtt"));
	//get_freshtime()�ĵ�λ��TTI������ת����s
	double freshtime_second = static_cast<double>(p->get_config()->get_freshtime()) / 1000.0;
	if (m_vangle == 0)
	{
		if ((m_absx + freshtime_second*m_speed)>(p->get_config()->get_road_length() / 2))
		{
			m_absx = (m_absx + freshtime_second*m_speed) - p->get_config()->get_road_length();
			m_relx = m_absx;
		}
		else
		{
			m_absx = m_absx + freshtime_second*m_speed;
			m_relx = m_absx;
		}
	}
	else
	{
		if ((m_absx - freshtime_second*m_speed)<(-p->get_config()->get_road_length() / 2))
		{
			m_absx = m_absx - freshtime_second*m_speed + p->get_config()->get_road_length();
			m_relx = m_absx;
		}
		else
		{
			m_absx = m_absx - freshtime_second*m_speed;
			m_relx = m_absx;
		}
	}
}

void vue_physics::update_location_urban() {
	auto p = (gtt_urban*)((gtt*)context::get_context()->get_bean("gtt"));
	//get_freshtime()�ĵ�λ��TTI������ת����s
	double freshtime_second = static_cast<double>(p->get_config()->get_freshtime()) / 1000.0;
	bool RoadChangeFlag = false;
	int temp;
	if (m_vangle == 90) {//left
		if ((m_rely + freshtime_second*m_speed) > (p->get_config()->get_road_length_ew() / 2)) {//top left
			temp = rand() % 4;
			if (temp == 0) {//turn left
				RoadChangeFlag = true;
				m_relx = p->get_config()->get_road_length_sn() / 2 - (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2);
				m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][6];
				m_vangle = -180;
			}
			else if (temp == 2) {//turn right
				m_relx = (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2) - p->get_config()->get_road_length_sn() / 2;
				m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
				m_vangle = 0;
			}
			else {//go straight
				RoadChangeFlag = true;
				m_rely = (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2) - p->get_config()->get_road_length_ew() / 2;
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][7];
			}
		}
		else {
			m_rely = m_rely + freshtime_second*m_speed;
		}
	}

	else if (m_vangle == 0) {//top
		if ((m_relx + freshtime_second*m_speed) > (p->get_config()->get_road_length_sn() / 2)) {//top right
			temp = rand() % 4;
			if (temp == 0) {//turn left
				RoadChangeFlag = true;
				m_rely = (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2) - p->get_config()->get_road_length_ew() / 2;
				m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][8];
				m_vangle = 90;
			}
			else if (temp == 2) {//turn right
				m_rely = p->get_config()->get_road_length_ew() / 2 - (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2);
				m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
				m_vangle = -90;
			}
			else {//go straight
				RoadChangeFlag = true;
				m_relx = (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2) - p->get_config()->get_road_length_sn() / 2;
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][1];
			}
		}
		else {
			m_relx = m_relx + freshtime_second*m_speed;
		}
	}

	else if (m_vangle == -90) {//right
		if ((m_rely - freshtime_second*m_speed) < -(p->get_config()->get_road_length_ew() / 2)) {//bottom right
			temp = rand() % 4;
			if (temp == 0) {//turn left
				RoadChangeFlag = true;
				m_relx = (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed)) - p->get_config()->get_road_length_sn() / 2;
				m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][2];
				m_vangle = 0;
			}
			else if (temp == 2) {//turn right
				m_relx = p->get_config()->get_road_length_sn() / 2 - (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed));
				m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
				m_vangle = -180;
			}
			else {//go straight
				RoadChangeFlag = true;
				m_rely = p->get_config()->get_road_length_ew() / 2 - (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed));
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][3];
			}
		}
		else {
			m_rely = m_rely - freshtime_second*m_speed;
		}
	}

	else {//bottom
		if ((m_relx - freshtime_second*m_speed) < -(p->get_config()->get_road_length_sn() / 2)) {//bottom left
			temp = rand() % 4;
			if (temp == 0) {//turn left
				RoadChangeFlag = true;
				m_rely = p->get_config()->get_road_length_ew() / 2 - (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed));
				m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][4];
				m_vangle = -90;
			}
			else if (temp == 2) {//turn right
				m_rely = (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed)) - p->get_config()->get_road_length_ew() / 2;
				m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
				m_vangle = 90;
			}
			else {//go straight
				RoadChangeFlag = true;
				m_relx = p->get_config()->get_road_length_sn() / 2 - (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed));
				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][5];
			}
		}
		else {
			m_relx = m_relx - freshtime_second*m_speed;
		}
	}
	m_absx = p->get_config()->get_road_topo_ratio()[m_road_id * 2 + 0] * (p->get_config()->get_road_length_sn() + 2 * p->get_config()->get_road_width()) + m_relx;
	m_absy = p->get_config()->get_road_topo_ratio()[m_road_id * 2 + 1] * (p->get_config()->get_road_length_ew() + 2 * p->get_config()->get_road_width()) + m_rely;

}

//����·�ڣ���������
//void vue_physics::update_location_urban() {
//	auto p = (gtt_urban*)((gtt*)context::get_context()->get_bean("gtt"));
//	//get_freshtime()�ĵ�λ��TTI������ת����s
//	double freshtime_second = static_cast<double>(p->get_config()->get_freshtime()) / 1000.0;
//	bool RoadChangeFlag = false;
//	int temp;
//	if (m_vangle == 90) {//left
//		if ((m_rely + freshtime_second*m_speed) > (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width() / 2)) {//top left
//			temp = rand() % 4;
//			if (temp == 0) {//turn left
//				RoadChangeFlag = true;
//				m_relx = (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2 - (m_rely + freshtime_second*m_speed - (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2);
//				m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][6];
//				m_vangle = -180;
//			}
//			else if (temp == 2) {//turn right
//				m_relx = (m_rely + freshtime_second*m_speed - (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width())/ 2) - (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2;
//				m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
//				m_vangle = 0;
//				if ((m_rely + freshtime_second*m_speed) - (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width() / 2)<p->get_config()->get_road_width() / 2)
//				{
//					m_in_cross = true;
//				}
//				else {
//					m_in_cross = false;
//				}
//			}
//			else {//go straight
//				RoadChangeFlag = true;
//				m_rely = (m_rely + freshtime_second*m_speed - (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width())/ 2) - (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2;
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][7];
//			}
//		}
//		else {
//			m_rely = m_rely + freshtime_second*m_speed;
//		}
//		if (abs((m_rely + freshtime_second*m_speed) - (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width() / 2))<p->get_config()->get_road_width() / 2)
//		{
//			m_in_cross = true;
//		}
//		else {
//			m_in_cross = false;
//		}
//	}
//
//	else if (m_vangle == 0) {//top
//		if ((m_relx + freshtime_second*m_speed) > (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2) {//top right
//			temp = rand() % 4;
//			if (temp == 0) {//turn left
//				RoadChangeFlag = true;
//				m_rely = (m_relx + freshtime_second*m_speed - (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width())/ 2) - (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2;
//				m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][8];
//				m_vangle = 90;
//			}
//			else if (temp == 2) {//turn right
//				m_rely = (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width())/ 2 - (m_relx + freshtime_second*m_speed - (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width())/ 2);
//				m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
//				m_vangle = -90;
//			}
//			else {//go straight
//				RoadChangeFlag = true;
//				m_relx = (m_relx + freshtime_second*m_speed - (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width())/ 2) - (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2;
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][1];
//			}
//		}
//		else {
//			m_relx = m_relx + freshtime_second*m_speed;
//		}
//		if (abs((m_relx + freshtime_second*m_speed) - (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2)<(p->get_config()->get_road_width()) / 2)
//		{
//			m_in_cross = true;
//		}
//		else {
//			m_in_cross = false;
//		}
//	}
//
//	else if (m_vangle == -90) {//right
//		if ((m_rely - freshtime_second*m_speed) < -((p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2)) {//bottom right
//			temp = rand() % 4;
//			if (temp == 0) {//turn left
//				RoadChangeFlag = true;
//				m_relx = (-(p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width())/ 2 - (m_rely - freshtime_second*m_speed)) - (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width())/ 2;
//				m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][2];
//				m_vangle = 0;
//			}
//			else if (temp == 2) {//turn right
//				m_relx = (p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width())/ 2 - (-(p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2 - (m_rely - freshtime_second*m_speed));
//				m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2;
//				m_vangle = -180;
//			}
//			else {//go straight
//				RoadChangeFlag = true;
//				m_rely = (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2 - (-(p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2 - (m_rely - freshtime_second*m_speed));
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][3];
//			}
//		}
//		else {
//			m_rely = m_rely - freshtime_second*m_speed;
//		}
//		if (abs((m_rely - freshtime_second*m_speed) + ((p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()) / 2))<p->get_config()->get_road_width() / 2)
//		{
//			m_in_cross = true;
//		}
//		else {
//			m_in_cross = false;
//		}
//	}
//
//	else {//bottom
//		if ((m_relx - freshtime_second*m_speed) < -((p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2)) {//bottom left
//			temp = rand() % 4;
//			if (temp == 0) {//turn left
//				RoadChangeFlag = true;
//				m_rely = (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2 - (-(p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2 - (m_relx - freshtime_second*m_speed));
//				m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][4];
//				m_vangle = -90;
//			}
//			else if (temp == 2) {//turn right
//				m_rely = (-(p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2 - (m_relx - freshtime_second*m_speed)) - (p->get_config()->get_road_length_ew()+ p->get_config()->get_road_width()) / 2;
//				m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2;
//				m_vangle = 90;
//			}
//			else {//go straight
//				RoadChangeFlag = true;
//				m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width())/ 2 - (-(p->get_config()->get_road_length_sn()+ p->get_config()->get_road_width()) / 2 - (m_relx - freshtime_second*m_speed));
//				m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][5];
//			}
//		}
//		else {
//			m_relx = m_relx - freshtime_second*m_speed;
//		}
//		if (abs((m_relx - freshtime_second*m_speed) + ((p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()) / 2))<p->get_config()->get_road_width() / 2)
//		{
//			m_in_cross = true;
//		}
//		else {
//			m_in_cross = false;
//		}
//	}
//	m_absx = p->get_config()->get_road_topo_ratio()[m_road_id * 2 + 0] * (p->get_config()->get_road_length_sn() + 2 * p->get_config()->get_road_width()) + m_relx;
//	m_absy = p->get_config()->get_road_topo_ratio()[m_road_id * 2 + 1] * (p->get_config()->get_road_length_ew() + 2 * p->get_config()->get_road_width()) + m_rely;
//
//}

void vue_physics::update_location_urban_doubleroad() {
	auto p = (gtt_urban*)((gtt*)context::get_context()->get_bean("gtt"));
	//get_freshtime()�ĵ�λ��TTI������ת����s
	double freshtime_second = static_cast<double>(p->get_config()->get_freshtime()) / 1000.0;
	bool RoadChangeFlag = false;
	int temp;
	if (m_road_id<14)
	{
		if (m_vangle == 90) {//left
			if ((m_rely + freshtime_second*m_speed) >(p->get_config()->get_road_length_ew() / 2)) {//top left
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_relx = p->get_config()->get_road_length_sn() / 2 - (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2);
					m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()/2) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][6];
					m_vangle = -180;
				}
				else if (temp == 2) {//turn right
					m_relx = (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2) - p->get_config()->get_road_length_sn() / 2;
					m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()/2) / 2;
					m_vangle = 0;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_rely = (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2) - p->get_config()->get_road_length_ew() / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][7];
				}
			}
			else {
				m_rely = m_rely + freshtime_second*m_speed;
			}
		}

		else if (m_vangle == 0) {//top
			if ((m_relx + freshtime_second*m_speed) > (p->get_config()->get_road_length_sn() / 2)) {//top right
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_rely = (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2) - p->get_config()->get_road_length_ew() / 2;
					m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()/2) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][8];
					m_vangle = 90;
				}
				else if (temp == 2) {//turn right
					m_rely = p->get_config()->get_road_length_ew() / 2 - (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2);
					m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()/2) / 2;
					m_vangle = -90;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_relx = (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2) - p->get_config()->get_road_length_sn() / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][1];
				}
			}
			else {
				m_relx = m_relx + freshtime_second*m_speed;
			}
		}

		else if (m_vangle == -90) {//right
			if ((m_rely - freshtime_second*m_speed) < -(p->get_config()->get_road_length_ew() / 2)) {//bottom right
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_relx = (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed)) - p->get_config()->get_road_length_sn() / 2;
					m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()/2) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][2];
					m_vangle = 0;
				}
				else if (temp == 2) {//turn right
					m_relx = p->get_config()->get_road_length_sn() / 2 - (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed));
					m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width()/2) / 2;
					m_vangle = -180;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_rely = p->get_config()->get_road_length_ew() / 2 - (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed));
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][3];
				}
			}
			else {
				m_rely = m_rely - freshtime_second*m_speed;
			}
		}

		else {//bottom
			if ((m_relx - freshtime_second*m_speed) < -(p->get_config()->get_road_length_sn() / 2)) {//bottom left
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_rely = p->get_config()->get_road_length_ew() / 2 - (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed));
					m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()/2) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][4];
					m_vangle = -90;
				}
				else if (temp == 2) {//turn right
					m_rely = (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed)) - p->get_config()->get_road_length_ew() / 2;
					m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width()/2) / 2;
					m_vangle = 90;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_relx = p->get_config()->get_road_length_sn() / 2 - (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed));
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id][5];
				}
			}
			else {
				m_relx = m_relx - freshtime_second*m_speed;
			}
		}
		m_absx = p->get_config()->get_road_topo_ratio()[m_road_id * 2 + 0] * (p->get_config()->get_road_length_sn() + 2 * p->get_config()->get_road_width()) + m_relx;
		m_absy = p->get_config()->get_road_topo_ratio()[m_road_id * 2 + 1] * (p->get_config()->get_road_length_ew() + 2 * p->get_config()->get_road_width()) + m_rely;
	}
	else {
		if (m_vangle == 90) {//left
			if ((m_rely + freshtime_second*m_speed) >(p->get_config()->get_road_length_ew() / 2)) {//top left
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_relx = p->get_config()->get_road_length_sn() / 2 - (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2);
					m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width() / 2*3) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][6]+14;
					m_vangle = -180;
				}
				else if (temp == 2) {//turn right
					m_relx = (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2) - p->get_config()->get_road_length_sn() / 2;
					m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width() / 2*3) / 2;
					m_vangle = 0;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_rely = (m_rely + freshtime_second*m_speed - p->get_config()->get_road_length_ew() / 2) - p->get_config()->get_road_length_ew() / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][7]+14;
				}
			}
			else {
				m_rely = m_rely + freshtime_second*m_speed;
			}
		}

		else if (m_vangle == 0) {//top
			if ((m_relx + freshtime_second*m_speed) > (p->get_config()->get_road_length_sn() / 2)) {//top right
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_rely = (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2) - p->get_config()->get_road_length_ew() / 2;
					m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width() / 2*3) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][8]+14;
					m_vangle = 90;
				}
				else if (temp == 2) {//turn right
					m_rely = p->get_config()->get_road_length_ew() / 2 - (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2);
					m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width() / 2*3) / 2;
					m_vangle = -90;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_relx = (m_relx + freshtime_second*m_speed - p->get_config()->get_road_length_sn() / 2) - p->get_config()->get_road_length_sn() / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][1]+14;
				}
			}
			else {
				m_relx = m_relx + freshtime_second*m_speed;
			}
		}

		else if (m_vangle == -90) {//right
			if ((m_rely - freshtime_second*m_speed) < -(p->get_config()->get_road_length_ew() / 2)) {//bottom right
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_relx = (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed)) - p->get_config()->get_road_length_sn() / 2;
					m_rely = (p->get_config()->get_road_length_ew() + p->get_config()->get_road_width() / 2*3) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][2]+14;
					m_vangle = 0;
				}
				else if (temp == 2) {//turn right
					m_relx = p->get_config()->get_road_length_sn() / 2 - (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed));
					m_rely = -(p->get_config()->get_road_length_ew() + p->get_config()->get_road_width() / 2*3) / 2;
					m_vangle = -180;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_rely = p->get_config()->get_road_length_ew() / 2 - (-p->get_config()->get_road_length_ew() / 2 - (m_rely - freshtime_second*m_speed));
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][3]+14;
				}
			}
			else {
				m_rely = m_rely - freshtime_second*m_speed;
			}
		}

		else {//bottom
			if ((m_relx - freshtime_second*m_speed) < -(p->get_config()->get_road_length_sn() / 2)) {//bottom left
				temp = rand() % 4;
				if (temp == 0) {//turn left
					RoadChangeFlag = true;
					m_rely = p->get_config()->get_road_length_ew() / 2 - (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed));
					m_relx = (p->get_config()->get_road_length_sn() + p->get_config()->get_road_width() / 2*3) / 2;
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][4]+14;
					m_vangle = -90;
				}
				else if (temp == 2) {//turn right
					m_rely = (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed)) - p->get_config()->get_road_length_ew() / 2;
					m_relx = -(p->get_config()->get_road_length_sn() + p->get_config()->get_road_width() / 2*3) / 2;
					m_vangle = 90;
				}
				else {//go straight
					RoadChangeFlag = true;
					m_relx = p->get_config()->get_road_length_sn() / 2 - (-p->get_config()->get_road_length_sn() / 2 - (m_relx - freshtime_second*m_speed));
					m_road_id = p->get_config()->get_wrap_around_road()[m_road_id-14][5]+14;
				}
			}
			else {
				m_relx = m_relx - freshtime_second*m_speed;
			}
		}
		m_absx = p->get_config()->get_road_topo_ratio()[(m_road_id-14) * 2 + 0] * (p->get_config()->get_road_length_sn() + 2 * p->get_config()->get_road_width()) + m_relx;
		m_absy = p->get_config()->get_road_topo_ratio()[(m_road_id-14) * 2 + 1] * (p->get_config()->get_road_length_ew() + 2 * p->get_config()->get_road_width()) + m_rely;
	}

}

void vue_physics::set_superior_level(vue* t_superior_level) {
	m_superior_level = t_superior_level;
}

vue* vue_physics::get_superior_level() {
	return m_superior_level;
}

int vue_physics::get_ue_id() {
	return m_ue_id;
}