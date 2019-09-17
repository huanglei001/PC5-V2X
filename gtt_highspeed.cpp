/*
* =====================================================================================
*
*       Filename:  gtt_highspeed.cpp
*
*    Description:  高速场景类实现
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
#include"route.h"
#include"config.h"
#include"gtt_highspeed.h"
#include"vue.h"
#include"vue_physics.h"
#include"imta.h"
#include"function.h"
#include"reflect/context.h"
#include"time_stamp.h"

using namespace std;

void gtt_highspeed::initialize() {
	gtt_highspeed_config* __config = get_config();
	int* m_pupr = new int[__config->get_road_num()];//每条路上的车辆数
	double* TotalTime = new double[__config->get_road_num()];//每条道路初始泊松撒点过程中所有车辆都已撒进区域内所用的总时间
	std::list<double>* possion = new std::list<double>[__config->get_road_num()];//每条道路初始泊松撒点的车辆到达时间间隔list，单位s

	/*srand((unsigned)time(0));*/
	s_rsu_num = __config->get_road_length() / __config->get_rsu_space();//RSU数量
	//for (int i = 0; i < s_rsu_num; i++) {
	//	s_rsu_pattern_id.push_back(i % 1);
	//}

	//生成负指数分布的车辆到达间隔
	int tempVeUENum = 0;
	double lambda = 1 / 2.5f;//均值为1/lambda，依照协议车辆到达时间间隔的均值为2.5s
	for (int roadId = 0; roadId != __config->get_road_num(); roadId++) {
		TotalTime[roadId] = 0;
		while (TotalTime[roadId] * (__config->get_speed() / 3.6) < __config->get_road_length()) {
			double pV = 0.0;
			while (true)
			{
				pV = (double)rand() / (double)RAND_MAX;
				if (pV != 1)
				{
					break;
				}
			}
			pV = (-1.0 / lambda)*log(1 - pV);
			possion[roadId].push_back(pV);
			TotalTime[roadId] += pV;
			double check = TotalTime[roadId];
		}
		m_pupr[roadId] = static_cast<int>(possion[roadId].size());//完成当前道路下总车辆数的赋值
		tempVeUENum += m_pupr[roadId];
	}

	//进行车辆的撒点
	ofstream vue_coordinate;
	vue_coordinate.open("log/vue_coordinate.txt");

	m_ue_array = new vue[tempVeUENum + s_rsu_num];
	cout << "vuenum: " << tempVeUENum << endl;
	cout << "rsunum:" << s_rsu_num << endl;
	int vue_id = 0;

	for (int roadId = 0; roadId != __config->get_road_num(); roadId++) {
		for (int uprIdx = 0; uprIdx != m_pupr[roadId]; uprIdx++) {
			auto p = get_ue_array()[vue_id++].get_physics_level();
			p->m_speed = __config->get_speed()/3.6;
		    p->m_absx = -1732 + (TotalTime[roadId] - possion[roadId].back())*(p->m_speed);
			p->m_absy = __config->get_road_topo_ratio()[roadId * 2 + 1]* __config->get_road_width();
			if (roadId < __config->get_road_num() / 2) p->m_vangle = 180;
			else p->m_vangle = 0;

			vue_coordinate << p->m_absx << " ";
			vue_coordinate << p->m_absy << " ";
			vue_coordinate << endl;

			TotalTime[roadId] = TotalTime[roadId] - possion[roadId].back();
			possion[roadId].pop_back();
		}
	}

	//进行RSU的撒点

	ofstream rsu_coordinate;
	rsu_coordinate.open("log/rsu_coordinate.txt");

	int rsuidx = 0;
	for (int rsuid = tempVeUENum; rsuid < get_vue_num(); rsuid++) {
		auto p = get_ue_array()[rsuid].get_physics_level();
		p->m_absx = -1732.0f + rsuidx*__config->get_rsu_space();
		p->m_absy = 0;

		rsu_coordinate << p->m_absx << " ";
		rsu_coordinate << p->m_absy << " ";
		rsu_coordinate << endl;

		s_rsu_pattern_id.push_back((rsuid-tempVeUENum) % (((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_pattern_num()- ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number()+1));//RSU频率规划
		s_rsu_tti.push_back((rsuid - tempVeUENum) % ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_send_interval());//RSU时分
		rsuidx++;
	}
	
	vue_coordinate.close();
	rsu_coordinate.close();

	memory_clean::safe_delete(m_pupr, true);
	memory_clean::safe_delete(TotalTime, true);
	memory_clean::safe_delete(possion, true);

	vue_physics::s_pl_all.assign(get_vue_num(), std::vector<double>(get_vue_num(), 0));
	vue_physics::s_distance_all.assign(get_vue_num(), std::vector<double>(get_vue_num(), 0));
	vue_physics::s_distance_all_before.assign(get_vue_num(), std::vector<double>(get_vue_num(), 0));
	
}

int gtt_highspeed::get_ue_num() {
	return vue_physics::get_ue_num();
}
int gtt_highspeed::get_vue_num() {
	return vue_physics::get_ue_num()- get_rsu_num();
}

int gtt_highspeed::get_rsu_num() {
	return s_rsu_num;
}
int gtt_highspeed::get_pue_num() {
	return 0;
}
int gtt_highspeed::get_freshtime() {
	return get_config()->get_freshtime();
}

int gtt_highspeed::get_rsu_pattern_id(int rsuid) {
	return s_rsu_pattern_id[rsuid];
}

int gtt_highspeed::get_rsu_tti(int rsuid) {
	return s_rsu_tti[rsuid];
}

void gtt_highspeed::fresh_location() {
	//<Warn>:将信道刷新时间和位置刷新时间分开
	if (get_time()->get_tti() % get_config()->get_freshtime() != 0) {
		return;
	}
	for (int vue_id = 0; vue_id < get_vue_num()-s_rsu_num; vue_id++) {
		get_ue_array()[vue_id].get_physics_level()->update_location_highspeed();
	}

	for (int vue_id1 = 0; vue_id1 < get_vue_num(); vue_id1++) {
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
			calculate_pl(vue_id1, vue_id2);
		}
	}
}


void gtt_highspeed::calculate_pl(int t_vue_id1, int t_vue_id2) {
	location _location;

	int car_num = get_vue_num();
	if (t_vue_id1 < car_num&&t_vue_id2 < car_num) {
		_location.VeUEAntH1 = 1.5;
		_location.VeUEAntH2 = 1.5;
	}
	else if (t_vue_id1 >= car_num&&t_vue_id2 >= car_num) {
		_location.VeUEAntH1 = 5;
		_location.VeUEAntH2 = 5;
	}
	else {
		_location.VeUEAntH1 = 1.5;
		_location.VeUEAntH2 = 5;
	}

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

	_location.locationType = Los;
	_location.manhattan = false;

	double angle = 0;
	_location.distance = vue_physics::get_distance(t_vue_id1, t_vue_id2);

	angle = atan2(vuei->m_absy - vuej->m_absy, vuei->m_absx - vuej->m_absx) / imta::s_DEGREE_TO_PI;

	imta::randomGaussian(_location.posCor, 5, 0.0f, 1.0f);//产生高斯随机数，为后面信道系数使用

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

	memory_clean::safe_delete(_antenna.TxSlantAngle, true);
	memory_clean::safe_delete(_antenna.TxAntSpacing, true);
	memory_clean::safe_delete(_antenna.RxSlantAngle, true);
	memory_clean::safe_delete(_antenna.RxAntSpacing, true);
	memory_clean::safe_delete(__imta);
}

void gtt_highspeed::calculate_smallscale(int t_vue_id1, int t_vue_id2, double* resH) {
	location _location;

	int car_num = get_vue_num();
	if (t_vue_id1 < car_num&&t_vue_id2 < car_num) {
		_location.VeUEAntH1 = 1.5;
		_location.VeUEAntH2 = 1.5;
	}
	else if (t_vue_id1 >= car_num&&t_vue_id2 >= car_num) {
		_location.VeUEAntH1 = 5;
		_location.VeUEAntH2 = 5;
	}
	else {
		_location.VeUEAntH1 = 1.5;
		_location.VeUEAntH2 = 5;
	}

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

	//判断车辆运动方向是东西方向还是南北方向，true代表东西方向，false代表南北方向
	bool v_diri, v_dirj;
	if (vuei->m_vangle == 0 || vuei->m_vangle == 180) {
		v_diri = true;
	}
	else {
		v_diri = false;
	}

	if (vuej->m_vangle == 0 || vuej->m_vangle == 180) {
		v_dirj = true;
	}
	else {
		v_dirj = false;
	}

	//计算两车辆的绝对横纵坐标的距离
	double x_between = abs(vuei->m_absx - vuej->m_absx);
	double y_between = abs(vuei->m_absy - vuej->m_absy);

	if (t_vue_id1 < car_num&&t_vue_id2 < car_num) {//车辆之间
												   //判断辆车间是否有建筑物遮挡，从而确定是Nlos还是Los,如果是NLos，再判断是否是曼哈顿街角模型
		if ((v_diri == true && v_dirj == true && y_between < 14) || (v_diri == false && v_dirj == false && x_between < 14)) {
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
	else {//有RSU参与
		if (x_between < get_config()->get_road_width() || y_between < get_config()->get_road_width()) {
			_location.locationType = Los;
		}
		else {
			_location.locationType = Nlos;
			_location.manhattan = true;
		}
	}

	double angle = 0;

	_location.distance = vue_physics::get_distance(t_vue_id1, t_vue_id2);
	_location.distance1 = x_between;
	_location.distance2 = y_between;

	angle = atan2(vuei->m_absy - vuej->m_absy, vuei->m_absx - vuej->m_absx) / imta::s_DEGREE_TO_PI;

	imta::randomGaussian(_location.posCor, 5, 0.0f, 1.0f);//产生高斯随机数，为后面信道系数使用

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
	double *ppfStore = new double[4096];
	double *H = new double[8 * 8 * 25 * 2];
	double *FFT = new double[8 * 8 * 2048 * 2];
	double *ch_buffer = new double[8 * 8 * 20 * 20];
	double *ch_sin = new double[8 * 8 * 20 * 20];
	double *ch_cos = new double[8 * 8 * 20 * 20];
	__imta->calculate(ppfStore, 0.001 * get_time()->get_tti(), ch_buffer, ch_sin, ch_cos, H, FFT);
	//vue_physics::set_haf(t_vue_id1, t_vue_id2, ppfStore);
	for (size_t i = 0; i < 4096; i++)
	{
		resH[i] = ppfStore[i];
	}
	delete[] H;
	delete[] FFT;
	delete[]ch_buffer;
	delete[]ch_sin;
	delete[]ch_cos;
	delete[]ppfStore;
	//vue_physics::set_pl(t_vue_id1, t_vue_id2, t_Pl);

	memory_clean::safe_delete(_antenna.TxSlantAngle, true);
	memory_clean::safe_delete(_antenna.TxAntSpacing, true);
	memory_clean::safe_delete(_antenna.RxSlantAngle, true);
	memory_clean::safe_delete(_antenna.RxAntSpacing, true);

	memory_clean::safe_delete(__imta);
}
