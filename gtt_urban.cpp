/*
* =====================================================================================
*
*       Filename:  gtt_urban.cpp
*
*    Description:  城镇类实现
*
*        Version:  1.0
*        Created:
*       Revision:
*       Compiler:  VS_2015
*
*         Author:  WYB
*            LIB:  ITTC
*
* =====================================================================================
*/

#include<fstream>
#include<memory.h>
#include<utility>
#include<random>
#include"route.h"
#include"config.h"
#include"gtt_urban.h"
#include"vue.h"
#include"vue_physics.h"
#include"imta.h"
#include"function.h"
#include"reflect/context.h"
#include"time_stamp.h"
#define N  999 //精度为小数点后面3位
using namespace std;

ofstream gtt_urban::s_logger_pathloss;
ofstream gtt_urban::s_logger_pathloss_los;
ofstream gtt_urban::s_logger_pathloss_nlos; 
ofstream gtt_urban::s_logger_distance;
ofstream gtt_urban::s_logger_singlepl;
ofstream gtt_urban::s_logger_singlesf;
const double gtt_urban::s_rsu_topo_ratio[s_rsu_num * 2] = {
	-2.0f, 1.5f,
	-1.0f, 1.5f,
	0.0f, 1.5f,
	1.0f, 1.5f,
	2.0f, 1.5f,
	-3.0f, 0.5f,
	-2.0f, 0.5f,
	-1.0f, 0.5f,
	0.0f, 0.5f,
	1.0f, 0.5f,
	2.0f, 0.5f,
	3.0f, 0.5f,
	-3.0f,-0.5f,
	-2.0f,-0.5f,
	-1.0f,-0.5f,
	0.0f,-0.5f,
	1.0f,-0.5f,
	2.0f,-0.5f,
	3.0f,-0.5f,
	-2.0f,-1.5f,
	-1.0f,-1.5f,
	0.0f,-1.5f,
	1.0f,-1.5f,
	2.0f,-1.5f,
};

const int gtt_urban::s_rsu_pattern_id[s_rsu_num] = {
	1,3,0,2,4,0,2,4,1,3,0,2,1,3,0,2,4,1,3,4,1,3,0,2,
};

void gtt_urban::initialize() {
	gtt_urban_config* __config = get_config();
	if (!__config->get_double_road())
	{
		int* m_pupr = new int[__config->get_road_num()];//每条路上的车辆数
		srand(int(time(0)));
		int tempVeUENum = 0;
		int Lambda = static_cast<int>((__config->get_road_length_ew() + __config->get_road_length_sn()) * 2 * 3.6 / (2.5 * __config->get_vue_speed()));
		for (int temp = 0; temp != __config->get_road_num(); ++temp)
		{
			int k = 0;
			long double p = 1.0;
			long double l = exp(-Lambda);  /* 为了精度，才定义为long double的，exp(-Lambda)是接近0的小数*/
			while (p >= l)
			{
				double u = (double)(rand() % 10000)*0.0001f;
				p *= u;
				k++;
			}
			m_pupr[temp] = k - 1;
			tempVeUENum = tempVeUENum + k - 1;
		}

		int m_pue = __config->get_pue_num() / __config->get_road_num();//每条路上的行人数
		pue_num = m_pue * __config->get_road_num();//行人总数	
		m_ue_array = new vue[tempVeUENum + pue_num];
		cout << "vuenum: " << tempVeUENum << endl;
		cout << "puenum:" << pue_num << endl;


		//进行车辆的撒点
		int vue_id = 0;
		double DistanceFromBottomLeft = 0;
		ofstream vue_coordinate;
		vue_coordinate.open("log/vue_coordinate.txt");
		default_random_engine e((unsigned)time(0));
		uniform_real_distribution<double> u(0, 2 * (__config->get_road_length_ew() + __config->get_road_length_sn()));

		for (int RoadIdx = 0; RoadIdx != __config->get_road_num(); RoadIdx++) {//撒车辆
			for (int uprIdx = 0; uprIdx != m_pupr[RoadIdx]; uprIdx++) {
				auto p = get_ue_array()[vue_id++].get_physics_level();
				DistanceFromBottomLeft = u(e);
				if (DistanceFromBottomLeft <= __config->get_road_length_ew()) {
					p->m_relx = -(__config->get_road_length_sn() + __config->get_road_width()) / 2;
					p->m_rely = DistanceFromBottomLeft - __config->get_road_length_ew() / 2;
					p->m_vangle = 90;
				}
				else if (DistanceFromBottomLeft > __config->get_road_length_ew() && DistanceFromBottomLeft <= (__config->get_road_length_ew() + __config->get_road_length_sn())) {
					p->m_relx = DistanceFromBottomLeft - __config->get_road_length_ew() - __config->get_road_length_sn() / 2;
					p->m_rely = (__config->get_road_length_ew() + __config->get_road_width()) / 2;
					p->m_vangle = 0;
				}
				else if (DistanceFromBottomLeft > (__config->get_road_length_ew() + __config->get_road_length_sn()) && DistanceFromBottomLeft < (__config->get_road_length_ew() * 2 + __config->get_road_length_sn())) {
					p->m_relx = (__config->get_road_length_sn() + __config->get_road_width()) / 2;
					p->m_rely = __config->get_road_length_ew() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() + __config->get_road_length_sn()));
					p->m_vangle = -90;
				}
				else {
					p->m_relx = __config->get_road_length_sn() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() * 2 + __config->get_road_length_sn()));
					p->m_rely = -(__config->get_road_length_ew() + __config->get_road_width()) / 2;
					p->m_vangle = -180;
				}
				p->m_road_id = RoadIdx;
				p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
				p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
				p->m_speed = __config->get_vue_speed() / 3.6;
				//将撒点后的坐标输出到txt文件
				vue_coordinate << p->m_absx << " ";
				vue_coordinate << p->m_absy << " ";
				vue_coordinate << endl;
			}
		}

		//进行行人撒点
		ofstream pue_coordinate;
		pue_coordinate.open("log/pue_coordinate.txt");
		int pue_id = 0;
		double pDistanceFromBottomLeft = 0;
		for (int RoadIdx = 0; RoadIdx != __config->get_road_num(); RoadIdx++) {
			for (int uprIdx = 0; uprIdx != m_pue; uprIdx++) {
				auto p = get_ue_array()[tempVeUENum + pue_id++].get_physics_level();

				double pDistanceFromBottomLeft = uprIdx * 1298 * __config->get_road_num() / __config->get_pue_num();

				if (pDistanceFromBottomLeft <= __config->get_pue_road_length_ew()) {
					p->m_relx = -(__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
					p->m_rely = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() / 2;
					p->m_vangle = 90;
				}
				else if (pDistanceFromBottomLeft > __config->get_pue_road_length_ew() && pDistanceFromBottomLeft <= (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn())) {
					p->m_relx = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() - __config->get_pue_road_length_sn() / 2;
					p->m_rely = (__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
					p->m_vangle = 0;
				}
				else if (pDistanceFromBottomLeft > (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()) && pDistanceFromBottomLeft < (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn())) {
					p->m_relx = (__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
					p->m_rely = __config->get_pue_road_length_ew() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()));
					p->m_vangle = -90;
				}
				else {
					p->m_relx = __config->get_pue_road_length_sn() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn()));
					p->m_rely = -(__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
					p->m_vangle = -180;
				}
				p->m_road_id = RoadIdx;
				p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
				p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
				p->m_speed = __config->get_vue_speed() / 3.6;
				//将撒点后的坐标输出到txt文件
				pue_coordinate << p->m_absx << " ";
				pue_coordinate << p->m_absy << " ";
				pue_coordinate << endl;

			}
		}
		memory_clean::safe_delete(m_pupr, true);

		vue_coordinate.close();
		pue_coordinate.close();
		vue_physics::s_pl_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_distance_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0)); 
		vue_physics::s_distance_all_before.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_LOS.assign(get_ue_num(), std::vector<int>(get_ue_num(), 0));
		//vue_physics::hafterFFT.assign(get_ue_num(), std::list<std::list<double>>(get_ue_num(),std::list<double>(4096,0)));
		/*
		pathloss日志初始化*/
		s_logger_pathloss.open("log/s_logger_pathloss.txt");
		s_logger_pathloss_los.open("log/s_logger_pathloss_los.txt");
		s_logger_pathloss_nlos.open("log/s_logger_pathloss_nlos.txt");
		s_logger_distance.open("log/s_logger_distance.txt");
	}
	else {
		int* m_pupr = new int[__config->get_road_num() * 2];//每条路上的车辆数
		srand(int(time(0)));
		int tempVeUENum = 0;
		int Lambda = static_cast<int>((__config->get_road_length_ew() + __config->get_road_length_sn()) * 2 * 3.6 / (2.5 * __config->get_vue_speed()));
		for (int temp = 0; temp != __config->get_road_num() * 2; ++temp)
		{
			int k = 0;
			long double p = 1.0;
			long double l = exp(-Lambda);  /* 为了精度，才定义为long double的，exp(-Lambda)是接近0的小数*/
			while (p >= l)
			{
				double u = (double)(rand() % 10000)*0.0001f;
				p *= u;
				k++;
			}
			m_pupr[temp] = k - 1;
			tempVeUENum = tempVeUENum + k - 1;
		}

		int m_pue = __config->get_pue_num() / __config->get_pue_road_num();//每条路上的行人数
		pue_num = m_pue * __config->get_pue_road_num();//行人总数	
		m_ue_array = new vue[tempVeUENum + pue_num];
		cout << "vuenum: " << tempVeUENum << endl;
		cout << "puenum:" << pue_num << endl;


		//进行车辆的撒点
		int vue_id = 0;
		double DistanceFromBottomLeft = 0;
		ofstream vue_coordinate;
		vue_coordinate.open("log/vue_coordinate.txt");
		default_random_engine e((unsigned)time(0));
		uniform_real_distribution<double> u(0, 2 * (__config->get_road_length_ew() + __config->get_road_length_sn() + (__config->get_road_width() * 2)));

		for (int RoadIdx = 0; RoadIdx != __config->get_road_num() * 2; RoadIdx++) {//撒车辆
			for (int uprIdx = 0; uprIdx != m_pupr[RoadIdx]; uprIdx++) {
				auto p = get_ue_array()[vue_id++].get_physics_level();
				DistanceFromBottomLeft = u(e);
				if (RoadIdx < 14)
				{
					if (DistanceFromBottomLeft <= __config->get_road_length_ew()) {
						p->m_relx = -(__config->get_road_length_sn() + __config->get_road_width() / 2) / 2;
						p->m_rely = DistanceFromBottomLeft - __config->get_road_length_ew() / 2;
						p->m_vangle = 90;
					}
					else if (DistanceFromBottomLeft > __config->get_road_length_ew() && DistanceFromBottomLeft <= (__config->get_road_length_ew() + __config->get_road_length_sn())) {
						p->m_relx = DistanceFromBottomLeft - __config->get_road_length_ew() - __config->get_road_length_sn() / 2;
						p->m_rely = (__config->get_road_length_ew() + __config->get_road_width() / 2) / 2;
						p->m_vangle = 0;
					}
					else if (DistanceFromBottomLeft > (__config->get_road_length_ew() + __config->get_road_length_sn()) && DistanceFromBottomLeft < (__config->get_road_length_ew() * 2 + __config->get_road_length_sn())) {
						p->m_relx = (__config->get_road_length_sn() + __config->get_road_width() / 2) / 2;
						p->m_rely = __config->get_road_length_ew() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() + __config->get_road_length_sn()));
						p->m_vangle = -90;
					}
					else {
						p->m_relx = __config->get_road_length_sn() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() * 2 + __config->get_road_length_sn()));
						p->m_rely = -(__config->get_road_length_ew() + __config->get_road_width() / 2) / 2;
						p->m_vangle = -180;
					}
					p->m_road_id = RoadIdx;
					p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
					p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
					p->m_speed = __config->get_vue_speed() / 3.6;
					//将撒点后的坐标输出到txt文件
					vue_coordinate << p->m_absx << " ";
					vue_coordinate << p->m_absy << " ";
					vue_coordinate << endl;
				}
				else {
					if (DistanceFromBottomLeft <= __config->get_road_length_ew()) {
						p->m_relx = -(__config->get_road_length_sn() + __config->get_road_width() / 2 * 3) / 2;
						p->m_rely = DistanceFromBottomLeft - __config->get_road_length_ew() / 2;
						p->m_vangle = 90;
					}
					else if (DistanceFromBottomLeft > __config->get_road_length_ew() && DistanceFromBottomLeft <= (__config->get_road_length_ew() + __config->get_road_length_sn())) {
						p->m_relx = DistanceFromBottomLeft - __config->get_road_length_ew() - __config->get_road_length_sn() / 2;
						p->m_rely = (__config->get_road_length_ew() + __config->get_road_width() / 2 * 3) / 2;
						p->m_vangle = 0;
					}
					else if (DistanceFromBottomLeft > (__config->get_road_length_ew() + __config->get_road_length_sn()) && DistanceFromBottomLeft < (__config->get_road_length_ew() * 2 + __config->get_road_length_sn())) {
						p->m_relx = (__config->get_road_length_sn() + __config->get_road_width() / 2 * 3) / 2;
						p->m_rely = __config->get_road_length_ew() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() + __config->get_road_length_sn()));
						p->m_vangle = -90;
					}
					else {
						p->m_relx = __config->get_road_length_sn() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() * 2 + __config->get_road_length_sn()));
						p->m_rely = -(__config->get_road_length_ew() + __config->get_road_width() / 2 * 3) / 2;
						p->m_vangle = -180;
					}
					p->m_road_id = RoadIdx;
					p->m_absx = __config->get_road_topo_ratio()[(RoadIdx - 14) * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
					p->m_absy = __config->get_road_topo_ratio()[(RoadIdx - 14) * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
					p->m_speed = __config->get_vue_speed() / 3.6;
					//将撒点后的坐标输出到txt文件
					vue_coordinate << p->m_absx << " ";
					vue_coordinate << p->m_absy << " ";
					vue_coordinate << endl;
				}
			}
		}

		//进行行人撒点
		ofstream pue_coordinate;
		pue_coordinate.open("log/pue_coordinate.txt");
		int pue_id = 0;
		double pDistanceFromBottomLeft = 0;
		for (int RoadIdx = 0; RoadIdx != __config->get_pue_road_num(); RoadIdx++) {
			for (int uprIdx = 0; uprIdx != m_pue; uprIdx++) {
				auto p = get_ue_array()[tempVeUENum + pue_id++].get_physics_level();

				double pDistanceFromBottomLeft = uprIdx * 1298 * __config->get_pue_road_num() / __config->get_pue_num();

				if (pDistanceFromBottomLeft <= __config->get_pue_road_length_ew()) {
					p->m_relx = -(__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
					p->m_rely = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() / 2;
					p->m_vangle = 90;
				}
				else if (pDistanceFromBottomLeft > __config->get_pue_road_length_ew() && pDistanceFromBottomLeft <= (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn())) {
					p->m_relx = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() - __config->get_pue_road_length_sn() / 2;
					p->m_rely = (__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
					p->m_vangle = 0;
				}
				else if (pDistanceFromBottomLeft > (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()) && pDistanceFromBottomLeft < (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn())) {
					p->m_relx = (__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
					p->m_rely = __config->get_pue_road_length_ew() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()));
					p->m_vangle = -90;
				}
				else {
					p->m_relx = __config->get_pue_road_length_sn() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn()));
					p->m_rely = -(__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
					p->m_vangle = -180;
				}
				p->m_road_id = RoadIdx;
				p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
				p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
				p->m_speed = __config->get_vue_speed() / 3.6;
				//将撒点后的坐标输出到txt文件
				pue_coordinate << p->m_absx << " ";
				pue_coordinate << p->m_absy << " ";
				pue_coordinate << endl;

			}
		}
		memory_clean::safe_delete(m_pupr, true);

		vue_coordinate.close();
		pue_coordinate.close();
		vue_physics::s_pl_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_distance_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_distance_all_before.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_LOS.assign(get_ue_num(), std::vector<int>(get_ue_num(), 0));
		vue_physics::s_sn_DS.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_sn_ASA.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_sn_ASD.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		vue_physics::s_sn_SF.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
		//vue_physics::hafterFFT.assign(get_ue_num(), std::list<std::list<double>>(get_ue_num(),std::list<double>(4096,0)));
		/*
		pathloss日志初始化*/
		s_logger_pathloss.open("log/s_logger_pathloss.txt");
		s_logger_pathloss_los.open("log/s_logger_pathloss_los.txt");
		s_logger_pathloss_nlos.open("log/s_logger_pathloss_nlos.txt");
		s_logger_distance.open("log/s_logger_distance.txt");
		s_logger_singlepl.open("log/s_logger_singlepl.txt");
		s_logger_singlesf.open("log/s_logger_singlesf.txt");
	}
}

//路口场景（单车道）
//void gtt_urban::initialize() {
//	gtt_urban_config* __config = get_config();
//	if (!__config->get_double_road())
//	{
//		int* m_pupr = new int[__config->get_road_num()];//每条路上的车辆数
//		srand(int(time(0)));
//		int tempVeUENum = 0;
//		int Lambda = static_cast<int>((__config->get_road_length_ew() + __config->get_road_length_sn()+ __config->get_road_width()*2) * 2 * 3.6 / (2.5 * __config->get_vue_speed()));
//		for (int temp = 0; temp != __config->get_road_num(); ++temp)
//		{
//			int k = 0;
//			long double p = 1.0;
//			long double l = exp(-Lambda);  /* 为了精度，才定义为long double的，exp(-Lambda)是接近0的小数*/
//			while (p >= l)
//			{
//				double u = (double)(rand() % 10000)*0.0001f;
//				p *= u;
//				k++;
//			}
//			m_pupr[temp] = k - 1;
//			tempVeUENum = tempVeUENum + k - 1;
//		}
//
//		int m_pue = __config->get_pue_num() / __config->get_road_num();//每条路上的行人数
//		pue_num = m_pue * __config->get_road_num();//行人总数	
//		m_ue_array = new vue[tempVeUENum + pue_num];
//		cout << "vuenum: " << tempVeUENum << endl;
//		cout << "puenum:" << pue_num << endl;
//
//
//		//进行车辆的撒点
//		int vue_id = 0;
//		double DistanceFromBottomLeft = 0;
//		ofstream vue_coordinate;
//		vue_coordinate.open("log/vue_coordinate.txt");
//		default_random_engine e((unsigned)time(0));
//		uniform_real_distribution<double> u(0, 2 * (__config->get_road_length_ew() + __config->get_road_length_sn()+ __config->get_road_width() * 2));
//		int cross_nums = 0;
//		for (int RoadIdx = 0; RoadIdx != __config->get_road_num(); RoadIdx++) {//撒车辆
//			for (int uprIdx = 0; uprIdx != m_pupr[RoadIdx]; uprIdx++) {
//				auto p = get_ue_array()[vue_id++].get_physics_level();
//				DistanceFromBottomLeft = u(e);
//				if (DistanceFromBottomLeft <= __config->get_road_length_ew()+ __config->get_road_width()) {
//					p->m_relx = -(__config->get_road_length_sn() + __config->get_road_width()) / 2;
//					p->m_rely = DistanceFromBottomLeft - (__config->get_road_length_ew()+ __config->get_road_width())/ 2;
//					p->m_vangle = 90;
//					if (DistanceFromBottomLeft<(__config->get_road_width())/2|| DistanceFromBottomLeft>__config->get_road_length_ew() + __config->get_road_width()/2 )
//					{
//						p->set_in_cross(true);
//						cross_nums++;
//					}
//				}
//				else if (DistanceFromBottomLeft > __config->get_road_length_ew()+ __config->get_road_width() && DistanceFromBottomLeft <= (__config->get_road_length_ew() + __config->get_road_length_sn())+ __config->get_road_width()*2) {
//					p->m_relx = DistanceFromBottomLeft - __config->get_road_length_ew()- __config->get_road_width() - (__config->get_road_length_sn()+ __config->get_road_width())/ 2;
//					p->m_rely = (__config->get_road_length_ew() + __config->get_road_width()) / 2;
//					p->m_vangle = 0;
//					if (DistanceFromBottomLeft<(__config->get_road_width()) / 2+ __config->get_road_length_ew() + __config->get_road_width() || DistanceFromBottomLeft>(__config->get_road_length_ew() + __config->get_road_length_sn()) + __config->get_road_width() * 2- __config->get_road_width()/2)
//					{
//						p->set_in_cross(true);
//						cross_nums++;
//					}
//				}
//				else if (DistanceFromBottomLeft > (__config->get_road_length_ew() + __config->get_road_length_sn())+ __config->get_road_width()*2 && DistanceFromBottomLeft < (__config->get_road_length_ew() * 2 + __config->get_road_length_sn())+ __config->get_road_width()*3) {
//					p->m_relx = (__config->get_road_length_sn() + __config->get_road_width()) / 2;
//					p->m_rely = (__config->get_road_length_ew() + __config->get_road_width())/ 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() + __config->get_road_length_sn())- __config->get_road_width()*2);
//					p->m_vangle = -90;
//					if (DistanceFromBottomLeft<(__config->get_road_width()) / 2 + (__config->get_road_length_ew() + __config->get_road_length_sn()) + __config->get_road_width() * 2 || DistanceFromBottomLeft>(__config->get_road_length_ew() * 2 + __config->get_road_length_sn()) + __config->get_road_width() * 3 - __config->get_road_width() / 2)
//					{
//						p->set_in_cross(true);
//						cross_nums++;
//					}
//				}
//				else {
//					p->m_relx = (__config->get_road_length_sn()+ __config->get_road_width()) / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() * 2 + __config->get_road_length_sn())- __config->get_road_width()*3);
//					p->m_rely = -(__config->get_road_length_ew() + __config->get_road_width()) / 2;
//					p->m_vangle = -180;
//					if (DistanceFromBottomLeft<(__config->get_road_width()) / 2 + (__config->get_road_length_ew() * 2 + __config->get_road_length_sn()) + __config->get_road_width() * 3 || DistanceFromBottomLeft>(__config->get_road_length_ew() * 2 + __config->get_road_length_sn()*2) + __config->get_road_width() * 4 - __config->get_road_width() / 2)
//					{
//						p->set_in_cross(true);
//						cross_nums++;
//					}
//				}
//				p->m_road_id = RoadIdx;
//				p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
//				p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
//				p->m_speed = __config->get_vue_speed() / 3.6;
//				//将撒点后的坐标输出到txt文件
//				vue_coordinate << p->m_absx << " ";
//				vue_coordinate << p->m_absy << " ";
//				vue_coordinate << endl;
//			}
//		}
//		cout<< "crossroad:"<<cross_nums<<endl;
//		//进行行人撒点
//		ofstream pue_coordinate;
//		pue_coordinate.open("log/pue_coordinate.txt");
//		int pue_id = 0;
//		double pDistanceFromBottomLeft = 0;
//		for (int RoadIdx = 0; RoadIdx != __config->get_road_num(); RoadIdx++) {
//			for (int uprIdx = 0; uprIdx != m_pue; uprIdx++) {
//				auto p = get_ue_array()[tempVeUENum + pue_id++].get_physics_level();
//
//				double pDistanceFromBottomLeft = uprIdx * 1298 * __config->get_road_num() / __config->get_pue_num();
//
//				if (pDistanceFromBottomLeft <= __config->get_pue_road_length_ew()) {
//					p->m_relx = -(__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
//					p->m_rely = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() / 2;
//					p->m_vangle = 90;
//				}
//				else if (pDistanceFromBottomLeft > __config->get_pue_road_length_ew() && pDistanceFromBottomLeft <= (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn())) {
//					p->m_relx = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() - __config->get_pue_road_length_sn() / 2;
//					p->m_rely = (__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
//					p->m_vangle = 0;
//				}
//				else if (pDistanceFromBottomLeft > (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()) && pDistanceFromBottomLeft < (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn())) {
//					p->m_relx = (__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
//					p->m_rely = __config->get_pue_road_length_ew() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()));
//					p->m_vangle = -90;
//				}
//				else {
//					p->m_relx = __config->get_pue_road_length_sn() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn()));
//					p->m_rely = -(__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
//					p->m_vangle = -180;
//				}
//				p->m_road_id = RoadIdx;
//				p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
//				p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
//				p->m_speed = __config->get_vue_speed() / 3.6;
//				//将撒点后的坐标输出到txt文件
//				pue_coordinate << p->m_absx << " ";
//				pue_coordinate << p->m_absy << " ";
//				pue_coordinate << endl;
//
//			}
//		}
//		memory_clean::safe_delete(m_pupr, true);
//
//		vue_coordinate.close();
//		pue_coordinate.close();
//		vue_physics::s_pl_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
//		vue_physics::s_distance_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
//		//vue_physics::hafterFFT.assign(get_ue_num(), std::list<std::list<double>>(get_ue_num(),std::list<double>(4096,0)));
//		/*
//		pathloss日志初始化*/
//		s_logger_pathloss.open("log/s_logger_pathloss.txt");
//	}
//	else {
//		int* m_pupr = new int[__config->get_road_num() * 2];//每条路上的车辆数
//		srand(int(time(0)));
//		int tempVeUENum = 0;
//		int Lambda = static_cast<int>((__config->get_road_length_ew() + __config->get_road_length_sn()) * 2 * 3.6 / (2.5 * __config->get_vue_speed()));
//		for (int temp = 0; temp != __config->get_road_num() * 2; ++temp)
//		{
//			int k = 0;
//			long double p = 1.0;
//			long double l = exp(-Lambda);  /* 为了精度，才定义为long double的，exp(-Lambda)是接近0的小数*/
//			while (p >= l)
//			{
//				double u = (double)(rand() % 10000)*0.0001f;
//				p *= u;
//				k++;
//			}
//			m_pupr[temp] = k - 1;
//			tempVeUENum = tempVeUENum + k - 1;
//		}
//
//		int m_pue = __config->get_pue_num() / __config->get_pue_road_num();//每条路上的行人数
//		pue_num = m_pue * __config->get_pue_road_num();//行人总数	
//		m_ue_array = new vue[tempVeUENum + pue_num];
//		cout << "vuenum: " << tempVeUENum << endl;
//		cout << "puenum:" << pue_num << endl;
//
//
//		//进行车辆的撒点
//		int vue_id = 0;
//		double DistanceFromBottomLeft = 0;
//		ofstream vue_coordinate;
//		vue_coordinate.open("log/vue_coordinate.txt");
//		default_random_engine e((unsigned)time(0));
//		uniform_real_distribution<double> u(0, 2 * (__config->get_road_length_ew() + __config->get_road_length_sn() + (__config->get_road_width() * 2)));
//
//		for (int RoadIdx = 0; RoadIdx != __config->get_road_num() * 2; RoadIdx++) {//撒车辆
//			for (int uprIdx = 0; uprIdx != m_pupr[RoadIdx]; uprIdx++) {
//				auto p = get_ue_array()[vue_id++].get_physics_level();
//				DistanceFromBottomLeft = u(e);
//				if (RoadIdx < 14)
//				{
//					if (DistanceFromBottomLeft <= __config->get_road_length_ew()) {
//						p->m_relx = -(__config->get_road_length_sn() + __config->get_road_width() / 2) / 2;
//						p->m_rely = DistanceFromBottomLeft - __config->get_road_length_ew() / 2;
//						p->m_vangle = 90;
//					}
//					else if (DistanceFromBottomLeft > __config->get_road_length_ew() && DistanceFromBottomLeft <= (__config->get_road_length_ew() + __config->get_road_length_sn())) {
//						p->m_relx = DistanceFromBottomLeft - __config->get_road_length_ew() - __config->get_road_length_sn() / 2;
//						p->m_rely = (__config->get_road_length_ew() + __config->get_road_width() / 2) / 2;
//						p->m_vangle = 0;
//					}
//					else if (DistanceFromBottomLeft > (__config->get_road_length_ew() + __config->get_road_length_sn()) && DistanceFromBottomLeft < (__config->get_road_length_ew() * 2 + __config->get_road_length_sn())) {
//						p->m_relx = (__config->get_road_length_sn() + __config->get_road_width() / 2) / 2;
//						p->m_rely = __config->get_road_length_ew() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() + __config->get_road_length_sn()));
//						p->m_vangle = -90;
//					}
//					else {
//						p->m_relx = __config->get_road_length_sn() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() * 2 + __config->get_road_length_sn()));
//						p->m_rely = -(__config->get_road_length_ew() + __config->get_road_width() / 2) / 2;
//						p->m_vangle = -180;
//					}
//					p->m_road_id = RoadIdx;
//					p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
//					p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
//					p->m_speed = __config->get_vue_speed() / 3.6;
//					//将撒点后的坐标输出到txt文件
//					vue_coordinate << p->m_absx << " ";
//					vue_coordinate << p->m_absy << " ";
//					vue_coordinate << endl;
//				}
//				else {
//					if (DistanceFromBottomLeft <= __config->get_road_length_ew()) {
//						p->m_relx = -(__config->get_road_length_sn() + __config->get_road_width() / 2 * 3) / 2;
//						p->m_rely = DistanceFromBottomLeft - __config->get_road_length_ew() / 2;
//						p->m_vangle = 90;
//					}
//					else if (DistanceFromBottomLeft > __config->get_road_length_ew() && DistanceFromBottomLeft <= (__config->get_road_length_ew() + __config->get_road_length_sn())) {
//						p->m_relx = DistanceFromBottomLeft - __config->get_road_length_ew() - __config->get_road_length_sn() / 2;
//						p->m_rely = (__config->get_road_length_ew() + __config->get_road_width() / 2 * 3) / 2;
//						p->m_vangle = 0;
//					}
//					else if (DistanceFromBottomLeft > (__config->get_road_length_ew() + __config->get_road_length_sn()) && DistanceFromBottomLeft < (__config->get_road_length_ew() * 2 + __config->get_road_length_sn())) {
//						p->m_relx = (__config->get_road_length_sn() + __config->get_road_width() / 2 * 3) / 2;
//						p->m_rely = __config->get_road_length_ew() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() + __config->get_road_length_sn()));
//						p->m_vangle = -90;
//					}
//					else {
//						p->m_relx = __config->get_road_length_sn() / 2 - (DistanceFromBottomLeft - (__config->get_road_length_ew() * 2 + __config->get_road_length_sn()));
//						p->m_rely = -(__config->get_road_length_ew() + __config->get_road_width() / 2 * 3) / 2;
//						p->m_vangle = -180;
//					}
//					p->m_road_id = RoadIdx;
//					p->m_absx = __config->get_road_topo_ratio()[(RoadIdx - 14) * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
//					p->m_absy = __config->get_road_topo_ratio()[(RoadIdx - 14) * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
//					p->m_speed = __config->get_vue_speed() / 3.6;
//					//将撒点后的坐标输出到txt文件
//					vue_coordinate << p->m_absx << " ";
//					vue_coordinate << p->m_absy << " ";
//					vue_coordinate << endl;
//				}
//			}
//		}
//
//		//进行行人撒点
//		ofstream pue_coordinate;
//		pue_coordinate.open("log/pue_coordinate.txt");
//		int pue_id = 0;
//		double pDistanceFromBottomLeft = 0;
//		for (int RoadIdx = 0; RoadIdx != __config->get_pue_road_num(); RoadIdx++) {
//			for (int uprIdx = 0; uprIdx != m_pue; uprIdx++) {
//				auto p = get_ue_array()[tempVeUENum + pue_id++].get_physics_level();
//
//				double pDistanceFromBottomLeft = uprIdx * 1298 * __config->get_pue_road_num() / __config->get_pue_num();
//
//				if (pDistanceFromBottomLeft <= __config->get_pue_road_length_ew()) {
//					p->m_relx = -(__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
//					p->m_rely = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() / 2;
//					p->m_vangle = 90;
//				}
//				else if (pDistanceFromBottomLeft > __config->get_pue_road_length_ew() && pDistanceFromBottomLeft <= (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn())) {
//					p->m_relx = pDistanceFromBottomLeft - __config->get_pue_road_length_ew() - __config->get_pue_road_length_sn() / 2;
//					p->m_rely = (__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
//					p->m_vangle = 0;
//				}
//				else if (pDistanceFromBottomLeft > (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()) && pDistanceFromBottomLeft < (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn())) {
//					p->m_relx = (__config->get_pue_road_length_sn() + __config->get_pue_road_width()) / 2;
//					p->m_rely = __config->get_pue_road_length_ew() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() + __config->get_pue_road_length_sn()));
//					p->m_vangle = -90;
//				}
//				else {
//					p->m_relx = __config->get_pue_road_length_sn() / 2 - (pDistanceFromBottomLeft - (__config->get_pue_road_length_ew() * 2 + __config->get_pue_road_length_sn()));
//					p->m_rely = -(__config->get_pue_road_length_ew() + __config->get_pue_road_width()) / 2;
//					p->m_vangle = -180;
//				}
//				p->m_road_id = RoadIdx;
//				p->m_absx = __config->get_road_topo_ratio()[RoadIdx * 2 + 0] * (__config->get_road_length_sn() + 2 * __config->get_road_width()) + p->m_relx;
//				p->m_absy = __config->get_road_topo_ratio()[RoadIdx * 2 + 1] * (__config->get_road_length_ew() + 2 * __config->get_road_width()) + p->m_rely;
//				p->m_speed = __config->get_vue_speed() / 3.6;
//				//将撒点后的坐标输出到txt文件
//				pue_coordinate << p->m_absx << " ";
//				pue_coordinate << p->m_absy << " ";
//				pue_coordinate << endl;
//
//			}
//		}
//		memory_clean::safe_delete(m_pupr, true);
//
//		vue_coordinate.close();
//		pue_coordinate.close();
//		vue_physics::s_pl_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
//		vue_physics::s_distance_all.assign(get_ue_num(), std::vector<double>(get_ue_num(), 0));
//		//vue_physics::hafterFFT.assign(get_ue_num(), std::list<std::list<double>>(get_ue_num(),std::list<double>(4096,0)));
//		/*
//		pathloss日志初始化*/
//		s_logger_pathloss.open("log/s_logger_pathloss.txt");
//	}
//}

int gtt_urban::get_ue_num() {
	return vue_physics::get_ue_num();
}
int gtt_urban::get_vue_num() {
	return vue_physics::get_ue_num()-pue_num;
}
int gtt_urban::get_rsu_num() {
	return s_rsu_num;
}
int gtt_urban::get_pue_num() {
	return pue_num;
}

int gtt_urban::get_rsu_pattern_id(int rsuid) {
	return s_rsu_pattern_id[rsuid];
}

int gtt_urban::get_rsu_tti(int rsuid) {
	return s_rsu_tti[rsuid];
}

int gtt_urban::get_freshtime() {
	return get_config()->get_freshtime();
}

void gtt_urban::fresh_location() {
	//<Warn>:将信道刷新时间和位置刷新时间分开
	if (get_time()->get_tti() % get_config()->get_freshtime() != 0) {
		return;
	}
	gtt_urban_config* __config = get_config();
/*	if (get_time()->get_tti()!=0)
	{
		if (!__config->get_double_road())
		{
			for (int vue_id = 0; vue_id < get_vue_num(); vue_id++) {
				get_ue_array()[vue_id].get_physics_level()->update_location_urban();
			}
		}
		else {
			for (int vue_id = 0; vue_id < get_vue_num(); vue_id++) {
				get_ue_array()[vue_id].get_physics_level()->update_location_urban_doubleroad();
			}
		}
	}*/	
	int ueNum= get_ue_num();
	if (((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_select_broadcast_algorithm()==1)
	{
		ueNum = get_vue_num();
	}
	for (int vue_id1 = 0; vue_id1 < ueNum; vue_id1++) {
		for (int vue_id2 = 0; vue_id2 < vue_id1; vue_id2++) {
			auto vuei = get_ue_array()[vue_id1].get_physics_level();
			auto vuej = get_ue_array()[vue_id2].get_physics_level();
			if (get_time()->get_tti() != 0) {
				vue_physics::set_distance_before(vue_id2, vue_id1, vue_physics::get_distance(vue_id2, vue_id1));
			}
			else {
				vue_physics::set_distance_before(vue_id2, vue_id1, sqrt(pow((vuei->m_absx - vuej->m_absx), 2.0f) + pow((vuei->m_absy - vuej->m_absy), 2.0f)));
			}
			vue_physics::set_distance(vue_id2, vue_id1, sqrt(pow((vuei->m_absx - vuej->m_absx), 2.0f) + pow((vuei->m_absy - vuej->m_absy), 2.0f)));
			//s_logger_distance<<vue_physics::get_distance(vue_id1, vue_id2)<<" ";
			calculate_pl(vue_id1, vue_id2);
		}
	}
}

void gtt_urban::calculate_pl(int t_vue_id1, int t_vue_id2) {
	location _location;

	int car_num = get_vue_num();
	_location.VeUEAntH1 = 1.5;
	_location.VeUEAntH2 = 1.5;

	_location.eNBAntH = 5;
	_location.locationType = None;
	_location.distance = 0;
	_location.distance1 = 0;
	_location.distance2 = 0;

	antenna _antenna;
	_antenna.antGain = 3;
	_antenna.byTxAntNum = 1;
	_antenna.byRxAntNum = 2;

	imta* __imta = new imta();

	auto vuei = get_ue_array()[t_vue_id1].get_physics_level();
	auto vuej = get_ue_array()[t_vue_id2].get_physics_level();

	//判断UE运动方向是东西方向还是南北方向，true代表东西方向，false代表南北方向
	bool v_diri, v_dirj;
	if (vuei->m_vangle == 0 || vuei->m_vangle == -180) {
		v_diri = true;
	}
	else {
		v_diri = false;
	}

	if (vuej->m_vangle == 0 || vuej->m_vangle == -180) {
		v_dirj = true;
	}
	else {
		v_dirj = false;
	}

	//计算两UE的绝对横纵坐标的距离
	double x_between = abs(vuei->m_absx - vuej->m_absx);
	double y_between = abs(vuei->m_absy - vuej->m_absy);
	double xy_between = vue_physics::get_distance(t_vue_id1, t_vue_id2);
	double change_distance = abs(vue_physics::get_distance(t_vue_id1, t_vue_id2) - vue_physics::get_distance_before(t_vue_id1, t_vue_id2));
	int VPtype =0;//1.v2v 2.p2v 3.v2p
	//判断UE间是否有建筑物遮挡，从而确定是Nlos还是Los,如果是NLos，再判断是否是曼哈顿街角模型
	if (t_vue_id1 < car_num&&t_vue_id2 < car_num) {//车辆之间
		if ((v_diri == true && v_dirj == true && y_between < 20) || (v_diri == false && v_dirj == false && x_between < 20)) {				
			_location.locationType = Los;
		}
		else {
			_location.locationType = Nlos;
			if (v_diri == v_dirj) {
				_location.manhattan = false;
			}
			else {
				_location.manhattan = true;
				//_location.manhattan = false;
			}
		}
		//天线增益
		_antenna.antGain = _antenna.antGain * 2;
		VPtype = 1;
	}
	else if (t_vue_id1 >= car_num&&t_vue_id2 >= car_num) {//行人之间
		if ((v_diri == true && v_dirj == true && y_between < 20) || (v_diri == false && v_dirj == false && x_between < 20)) {
			_location.locationType = Los;
		}
		else {
			_location.locationType = Nlos;
			if (v_diri == v_dirj) {
				_location.manhattan = false;
			}
			else {
				_location.manhattan = true;
			}
		}
		_antenna.antGain = 0;
	}
	else
	{//行人与车辆之间
		if ((v_diri == true && v_dirj == true && y_between < 20) || (v_diri == false && v_dirj == false && x_between < 20)) {
			_location.locationType = Los;
		}
		else {
			_location.locationType = Nlos;
			if (v_diri == v_dirj) {
				_location.manhattan = false;
			}
			else {
				_location.manhattan = true;
			}
		}
		if (t_vue_id1 < car_num&&t_vue_id2 >= car_num)
		{
			VPtype = 3;
		}
		else {
			VPtype = 2;
		}
	}
	double angle = 0;
	_location.distance = xy_between;
	_location.distance1 = x_between;
	_location.distance2 = y_between;
	angle = atan2(vuei->m_absy - vuej->m_absy, vuei->m_absx - vuej->m_absx) / imta::s_DEGREE_TO_PI;

	imta::randomGaussian(_location.posCor, 5, 0.0f, 1.0f);//产生高斯随机数，为后面信道系数使用

	if (get_time()->get_tti() != 0) {
		//想法1  所有的变量都随机
		_location.posCor[0] = exp(-change_distance / 10) * vue_physics::get_sn_DS(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 10)) * _location.posCor[0];
		_location.posCor[1] = exp(-change_distance / 10) * vue_physics::get_sn_ASA(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 10)) * _location.posCor[1];
		_location.posCor[2] = exp(-change_distance / 9) * vue_physics::get_sn_ASD(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 9)) * _location.posCor[2];
		// 想法2 若是只用单独的初始化这一个，那么就把前面的注释掉
		_location.posCor[3] = exp(-change_distance / 13) * vue_physics::get_sn_SF(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 13)) * _location.posCor[3];
	}

	vue_physics::set_sn_DS(t_vue_id1, t_vue_id2, _location.posCor[0]);
	vue_physics::set_sn_ASA(t_vue_id1, t_vue_id2, _location.posCor[1]);
	vue_physics::set_sn_ASD(t_vue_id1, t_vue_id2, _location.posCor[2]);
	vue_physics::set_sn_SF(t_vue_id1, t_vue_id2, _location.posCor[3]);

	double m_FantennaAnglei;
	double m_FantennaAnglej;

	imta::randomUniform(&m_FantennaAnglei, 1, 180.0f, -180.0f, false);
	imta::randomUniform(&m_FantennaAnglej, 1, 180.0f, -180.0f, false);

	_antenna.TxAngle = angle - m_FantennaAnglei;
	_antenna.RxAngle = angle - m_FantennaAnglej;
	_antenna.TxSlantAngle = new double[_antenna.byTxAntNum];
	_antenna.TxAntSpacing = new double[_antenna.byTxAntNum];
	_antenna.RxSlantAngle = new double[_antenna.byRxAntNum];
	_antenna.RxAntSpacing = new double[_antenna.byRxAntNum];
	_antenna.TxSlantAngle[0] = 90.0f;
	_antenna.TxAntSpacing[0] = 0.0f;
	_antenna.RxSlantAngle[0] = 90.0f;
	_antenna.RxSlantAngle[1] = 90.0f;
	_antenna.RxAntSpacing[0] = 0.0f;
	_antenna.RxAntSpacing[1] = 0.5f;

	double t_Pl = 0;
	double t_SF = 0;

	context* __context = context::get_context();
	double s_FC = ((global_control_config*)__context->get_bean("global_control_config"))->get_carrier_frequency()*pow(10, 9);
	__imta->build(&t_Pl, &t_SF, s_FC, _location, _antenna, vuei->m_speed, vuej->m_speed, vuei->m_vangle, vuej->m_vangle);//计算了结果代入信道模型计算UE之间信道系数
	vue_physics::set_pl(t_vue_id1, t_vue_id2, t_Pl);
	int los;
	if (_location.locationType == Los)
	{
		los = 1;
	}
	else {
		if (_location.manhattan)
		{
			los = 0;
		}
		else {
			los = -1;
		}
	}
	vue_physics::set_LOS(t_vue_id1, t_vue_id2, los);
	// s_logger_pathloss << t_Pl<<" "<< _location.distance <<" "<<los <<" "<<VPtype<< endl;
	if (t_vue_id1 == 2 && t_vue_id2 == 1)
	{
		s_logger_singlepl << t_Pl << " " << _location.distance << endl;
		s_logger_singlesf << t_SF << " " << _location.distance << endl;
	}
	memory_clean::safe_delete(_antenna.TxSlantAngle, true);
	memory_clean::safe_delete(_antenna.TxAntSpacing, true);
	memory_clean::safe_delete(_antenna.RxSlantAngle, true);
	memory_clean::safe_delete(_antenna.RxAntSpacing, true);

	memory_clean::safe_delete(__imta);

}

void gtt_urban::calculate_smallscale(int t_vue_id1, int t_vue_id2, double* resH) {
	location _location;

	int car_num = get_vue_num();
	_location.VeUEAntH1 = 1.5;
	_location.VeUEAntH2 = 1.5;

	_location.eNBAntH = 5;
	_location.locationType = None;
	_location.distance = 0;
	_location.distance1 = 0;
	_location.distance2 = 0;

	antenna _antenna;
	_antenna.antGain = 3;
	_antenna.byTxAntNum = 1;
	_antenna.byRxAntNum = 2;

	imta* __imta = new imta();

	auto vuei = get_ue_array()[t_vue_id1].get_physics_level();
	auto vuej = get_ue_array()[t_vue_id2].get_physics_level();

	//判断UE运动方向是东西方向还是南北方向，true代表东西方向，false代表南北方向
	bool v_diri, v_dirj;
	if (vuei->m_vangle == 0 || vuei->m_vangle == -180) {
		v_diri = true;
	}
	else {
		v_diri = false;
	}

	if (vuej->m_vangle == 0 || vuej->m_vangle == -180) {
		v_dirj = true;
	}
	else {
		v_dirj = false;
	}

	//计算两UE的绝对横纵坐标的距离
	double x_between = abs(vuei->m_absx - vuej->m_absx);
	double y_between = abs(vuei->m_absy - vuej->m_absy);
	double change_distance = abs(vue_physics::get_distance(t_vue_id1, t_vue_id2) - vue_physics::get_distance_before(t_vue_id1, t_vue_id2));
	//判断UE间是否有建筑物遮挡，从而确定是Nlos还是Los,如果是NLos，再判断是否是曼哈顿街角模型
	if (t_vue_id1 < car_num&&t_vue_id2 < car_num) {//车辆之间
		if ((v_diri == true && v_dirj == true && y_between < 20) || (v_diri == false && v_dirj == false && x_between < 20)) {
			_location.locationType = Los;
		}
		else {
			_location.locationType = Nlos;
			if (v_diri == v_dirj) {
				_location.manhattan = false;
			}
			else {
				_location.manhattan = true;
			}
		}
	}
	else if (t_vue_id1 >= car_num&&t_vue_id2 >= car_num) {//行人之间
		if ((v_diri == true && v_dirj == true && y_between < 20) || (v_diri == false && v_dirj == false && x_between < 20)) {
			_location.locationType = Los;
		}
		else {
			_location.locationType = Nlos;
			if (v_diri == v_dirj) {
				_location.manhattan = false;
			}
			else {
				_location.manhattan = true;
			}
		}
	}
	else
	{//行人与车辆之间
		if ((v_diri == true && v_dirj == true && y_between < 20) || (v_diri == false && v_dirj == false && x_between < 20)) {
			_location.locationType = Los;
		}
		else {
			_location.locationType = Nlos;
			if (v_diri == v_dirj) {
				_location.manhattan = false;
			}
			else {
				_location.manhattan = true;
			}
		}
	}


	double angle = 0;

	_location.distance = vue_physics::get_distance(t_vue_id1, t_vue_id2);
	_location.distance1 = x_between;
	_location.distance2 = y_between;

	angle = atan2(vuei->m_absy - vuej->m_absy, vuei->m_absx - vuej->m_absx) / imta::s_DEGREE_TO_PI;

	imta::randomGaussian(_location.posCor, 5, 0.0f, 1.0f);//产生高斯随机数，为后面信道系数使用

	if (get_time()->get_tti() != 0) {
		//想法1  所有的变量都随机
		_location.posCor[0] = exp(-change_distance / 10) * vue_physics::get_sn_DS(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 10)) * _location.posCor[0];
		_location.posCor[1] = exp(-change_distance / 10) * vue_physics::get_sn_ASA(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 10)) * _location.posCor[1];
		_location.posCor[2] = exp(-change_distance / 9) * vue_physics::get_sn_ASD(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 9)) * _location.posCor[2];
		// 想法2 若是只用单独的初始化这一个，那么就把前面的注释掉
		_location.posCor[3] = exp(-change_distance / 13) * vue_physics::get_sn_SF(t_vue_id1, t_vue_id2) + sqrt(1 - exp(-2 * change_distance / 13)) * _location.posCor[3];
	}

	vue_physics::set_sn_DS(t_vue_id1, t_vue_id2, _location.posCor[0]);
	vue_physics::set_sn_ASA(t_vue_id1, t_vue_id2, _location.posCor[1]);
	vue_physics::set_sn_ASD(t_vue_id1, t_vue_id2, _location.posCor[2]);
	vue_physics::set_sn_SF(t_vue_id1, t_vue_id2, _location.posCor[3]);

	double m_FantennaAnglei;
	double m_FantennaAnglej;

	imta::randomUniform(&m_FantennaAnglei, 1, 180.0f, -180.0f, false);
	imta::randomUniform(&m_FantennaAnglej, 1, 180.0f, -180.0f, false);

	_antenna.TxAngle = angle - m_FantennaAnglei;
	_antenna.RxAngle = angle - m_FantennaAnglej;
	_antenna.TxSlantAngle = new double[_antenna.byTxAntNum];
	_antenna.TxAntSpacing = new double[_antenna.byTxAntNum];
	_antenna.RxSlantAngle = new double[_antenna.byRxAntNum];
	_antenna.RxAntSpacing = new double[_antenna.byRxAntNum];
	_antenna.TxSlantAngle[0] = 90.0f;
	_antenna.TxAntSpacing[0] = 0.0f;
	_antenna.RxSlantAngle[0] = 90.0f;
	_antenna.RxSlantAngle[1] = 90.0f;
	_antenna.RxAntSpacing[0] = 0.0f;
	_antenna.RxAntSpacing[1] = 0.5f;

	double t_Pl = 0;
	context* __context = context::get_context();
	double s_FC = ((global_control_config*)__context->get_bean("global_control_config"))->get_carrier_frequency()*pow(10, 9);
	__imta->build(s_FC, _location, _antenna, vuei->m_speed, vuej->m_speed, vuei->m_vangle, vuej->m_vangle);//计算了结果代入信道模型计算UE之间信道系数
	bool tempenable = true;
	bool* enable = &tempenable;
	__imta->enable(enable);
	int total_bandwidth = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_total_bandwidth()/1000000;
	int fft_point_num = pow(2, 10) * total_bandwidth / 10;
	double *ppfStore = new double[fft_point_num * 4];
	double *H = new double[8 * 8 * 5 * total_bandwidth];
	double *FFT = new double[8 * 8 * fft_point_num * 4];
	double *ch_buffer = new double[8 * 8 * pow(total_bandwidth,2.0)];
	double *ch_sin = new double[8 * 8 * pow(total_bandwidth, 2.0)];
	double *ch_cos = new double[8 * 8 * pow(total_bandwidth, 2.0)];
	__imta->calculate(ppfStore, 0.001 * get_time()->get_tti(), ch_buffer, ch_sin, ch_cos, H, FFT);
	for (size_t i = 0; i < fft_point_num * 4; i++)
	{
		resH[i] = ppfStore[i];
	}
	delete[] H;
	delete[] FFT;
	delete[]ch_buffer;
	delete[]ch_sin;
	delete[]ch_cos;
	delete[]ppfStore;
	H = NULL;
	FFT = NULL;
	ch_buffer = NULL;
	ch_sin = NULL;
	ch_cos = NULL;
	ppfStore = NULL;
	/*double *ppfStore=new double[4096];
	double *H = new double[8 * 8 * 25 * 2];
	double *FFT = new double[8 * 8 * 2048 * 2];
	double *ch_buffer = new double[8 * 8 * 20 * 20];
	double *ch_sin = new double[8 * 8 * 20 * 20];
	double *ch_cos = new double[8 * 8 * 20 * 20];
	__imta->calculate(ppfStore, 0.001 * get_time()->get_tti(), ch_buffer, ch_sin, ch_cos, H, FFT);
	for (size_t i = 0; i < 4096; i++)
	{
		resH[i] = ppfStore[i];
	}
	delete[] H;
	delete[] FFT;
	delete[]ch_buffer;
	delete[]ch_sin;
	delete[]ch_cos;
	delete[]ppfStore;*/

	memory_clean::safe_delete(_antenna.TxSlantAngle, true);
	memory_clean::safe_delete(_antenna.TxAntSpacing, true);
	memory_clean::safe_delete(_antenna.RxSlantAngle, true);
	memory_clean::safe_delete(_antenna.RxAntSpacing, true);

	memory_clean::safe_delete(__imta);
}
