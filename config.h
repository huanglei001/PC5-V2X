#pragma once
#include<vector>
#include<string>
#include<stdexcept>
#include<iostream>
#include<fstream>
#include"reflect/object.h"
#include"enumeration.h"

#define INVALID -1

class global_control_config :public object {
	REGISTE_MEMBER_HEAD(global_control_config)

	/*
	* 仿真时长 和 统计距离
	*/
private:
	int m_ntti;
	void set_ntti(std::string t_ntti) {
		m_ntti = stoi(t_ntti);
	}

	int m_max_distance;
	void set_max_distance(std::string t_distance) {
		m_max_distance = stoi(t_distance);
	}

	int m_rsu_num;
	void set_rsu_num(std::string t_rsu_num) {
		m_rsu_num = stoi(t_rsu_num);
	}

	int m_max_delay;
	void set_max_delay(std::string t_max_queue) {
		m_max_delay = stoi(t_max_queue);
	}

	int m_small_scale;
	void set_small_scale(std::string small_scale) {
		m_small_scale = stoi(small_scale);
	}

	int m_select_broadcast_algorithm;
	void set_select_broadcast_algorithm(std::string t_select_broadcast_algorithm) {
		m_select_broadcast_algorithm = stoi(t_select_broadcast_algorithm);
	}

	int m_hop_num;
	void set_hop_num(std::string t_hop_num) {
		m_hop_num = stoi(t_hop_num);
	}

	int m_carrier_frequency;
	void set_carrier_frequency(std::string t_carrier_frequency) {
		m_carrier_frequency = stoi(t_carrier_frequency);
	}
	int m_send_power;
	void set_send_power(std::string t_send_power) {
		m_send_power = stoi(t_send_power);
	}
	int m_noise_figure;
	void set_noise_figure(std::string t_noise_figure) {
		m_noise_figure = stoi(t_noise_figure);
	}

public:
	int get_ntti() {
		return m_ntti;
	}

	int get_max_distance() {
		return m_max_distance;
	}

	int get_rsu_num() {
		return m_rsu_num;
	}

	int get_max_delay() {
		return m_max_delay;
	}

	int get_small_scale() {
		return m_small_scale;
	}

	int get_select_broadcast_algorithm() {
		return m_select_broadcast_algorithm;
	}

	int get_hop_num() {
		return m_hop_num;
	}

	int get_carrier_frequency() {
		return m_carrier_frequency;
	}

	int get_send_power() {
		return m_send_power;
	}
	int get_noise_figure() {
		return m_noise_figure;
	}
};


class gtt_highspeed_config :public object {
	REGISTE_MEMBER_HEAD(gtt_highspeed_config)
	/*
	* 道路数量
	*/
private:
	const int m_road_num = 6;
public:
	int get_road_num() {
		return m_road_num;
	}

	/*
	* 路长,单位m
	*/
private:
	double m_road_length = INVALID;
	void set_road_length(std::string t_road_length) {
		m_road_length = stod(t_road_length);
	}
public:
	double get_road_length() {
		return m_road_length;
	}

	/*
	* 路宽，单位m
	*/
private:
	double m_road_width = INVALID;
	void set_road_width(std::string t_road_width) {
		m_road_width = stod(t_road_width);
	}
public:
	double get_road_width() {
		return m_road_width;
	}

	/*
	* 车速,km/h
	*/
private:
	double m_speed = INVALID;
	void set_speed(std::string t_speed) {
		m_speed = stod(t_speed);
	}
public:
	double get_speed() {
		return m_speed;
	}

	/*
	* 道路拓扑位置
	*/
private:
	const double m_road_topo_ratio[6 * 2]{
		0.0f, -2.5f,
		0.0f, -1.5f,
		0.0f, -0.5f,
		0.0f, 0.5f,
		0.0f, 1.5f,
		0.0f, 2.5f,
	};
public:
	const double* get_road_topo_ratio() {
		return m_road_topo_ratio;
	}

	/*
	* 车辆位置刷新时间,单位s
	*/
private:
	int m_freshtime = INVALID;
	void set_freshtime(std::string t_freshtime) {
		m_freshtime = stoi(t_freshtime);
	}
public:
	int get_freshtime() {
		return m_freshtime;
	}

	/*
	* RSU放置间隔
	*/
private:
	int m_rsu_space = INVALID;
	void set_rsu_space(std::string t_rsu_space) {
		m_rsu_space = stoi(t_rsu_space);
	}
public:
	int get_rsu_space() {
		return m_rsu_space;
	}
};


class gtt_urban_config :public object {
	REGISTE_MEMBER_HEAD(gtt_urban_config)

	/*
	* 街区数量
	*/
private:
	const int m_road_num = 14;
	const int m_pue_road_num = 14;
public:
	int get_road_num() {
		return m_road_num;
	}
	int get_pue_road_num() {
		return m_pue_road_num;
	}
	/*
	* 路长，分为东西向和南北向,单位m
	*/
private:
	double m_road_length_ew = INVALID;
	double m_road_length_sn = INVALID;
	void set_road_length_ew(std::string t_road_length_ew) {
		m_road_length_ew = stod(t_road_length_ew);
	}
	void set_road_length_sn(std::string t_road_length_sn) {
		m_road_length_sn = stod(t_road_length_sn);
	}
public:
	double get_road_length_ew() {
		return m_road_length_ew;
	}
	double get_road_length_sn() {
		return m_road_length_sn;
	}

	/*
	* 路宽，单位m
	*/
private:
	double m_road_width = INVALID;
	void set_road_width(std::string t_road_width) {
		m_road_width = stod(t_road_width);
	}
public:
	double get_road_width() {
		return m_road_width;
	}
	/*
	* 行人路长，分为东西向和南北向,单位m
	*/
private:
	double pue_road_length_ew = INVALID;
	double pue_road_length_sn = INVALID;
	void set_pue_road_length_ew(std::string t_road_length_ew) {
		pue_road_length_ew = stod(t_road_length_ew);
	}
	void set_pue_road_length_sn(std::string t_road_length_sn) {
		pue_road_length_sn = stod(t_road_length_sn);
	}
public:
	double get_pue_road_length_ew() {
		return pue_road_length_ew;
	}
	double get_pue_road_length_sn() {
		return pue_road_length_sn;
	}

	/*
	* 行人路宽，单位m
	*/
private:
	double pue_road_width = INVALID;
	void set_pue_road_width(std::string t_road_width) {
		pue_road_width = stod(t_road_width);
	}
public:
	double get_pue_road_width() {
		return pue_road_width;
	}
	/*
	* 行人数量
	*/
private:
	double m_pue_num = INVALID;
	void set_pue_num(std::string t_road_width) {
		m_pue_num = stod(t_road_width);
	}
	
public:
	double get_pue_num() {
		return m_pue_num;
	}

	/*
	* 车速/行人速度,km/h
	*/
private:
	double m_vue_speed = INVALID;
	double m_pue_speed = INVALID;
	void set_vue_speed(std::string t_speed) {
		m_vue_speed = stod(t_speed);
	}
	void set_pue_speed(std::string t_speed) {
		m_pue_speed = stod(t_speed);
	}
public:
	double get_vue_speed() {
		return m_vue_speed;
	}
	double get_pue_speed() {
		return m_pue_speed;
	}

	/*
	* 道路拓扑位置
	*/
private:
	const double m_road_topo_ratio[14 * 2] = {
		-1.5f, 1.0f,
		-0.5f, 1.0f,
		0.5f, 1.0f,
		1.5f, 1.0f,
		-2.5f, 0.0f,
		-1.5f, 0.0f,
		-0.5f, 0.0f,
		0.5f, 0.0f,
		1.5f, 0.0f,
		2.5f, 0.0f,
		-1.5f,-1.0f,
		-0.5f,-1.0f,
		0.5f,-1.0f,
		1.5f,-1.0f
	};

private:
	const int m_wrap_around_road[14][9] = {
		{ 0,1,6,5,4,13,8,9,10 },
		{ 1,2,7,6,5,0,9,10,11 },
		{ 2,3,8,7,6,1,10,11,12 },
		{ 3,4,9,8,7,2,11,12,13 },
		{ 4,5,10,9,8,3,12,13,0 },
		{ 5,6,11,10,9,4,13,0,1 },
		{ 6,7,12,11,10,5,0,1,2 },
		{ 7,8,13,12,11,6,1,2,3 },
		{ 8,9,0,13,12,7,2,3,4 },
		{ 9,10,1,0,13,8,3,4,5 },
		{ 10,11,2,1,0,9,4,5,6 },
		{ 11,12,3,2,1,10,5,6,7 },
		{ 12,13,4,3,2,11,6,7,8 },
		{ 13,0,5,4,3,12,7,8,9 }
	};

public:
	const double* get_road_topo_ratio() {
		return m_road_topo_ratio;
	}
	const int(*get_wrap_around_road())[9]{
		return m_wrap_around_road;
	}
	/*
	* 车辆位置刷新时间,单位s
	*/
private:
	int m_freshtime = INVALID;
	void set_freshtime(std::string t_freshtime) {
		m_freshtime = stoi(t_freshtime);
	}
public:
	int get_freshtime() {
		return m_freshtime;
	}
		/*
		* 是否为双车道
		*/
private:
	int m_double_road = INVALID;
	void set_double_road(std::string double_road) {
		m_double_road = stoi(double_road);
	}
public:
	int get_double_road() {
		return m_double_road;
	}
};


class rrm_config :public object {
	REGISTE_MEMBER_HEAD(rrm_config)

	/*------------------静态成员------------------*/
public:
	/*
	* 每个RB的带宽(Hz)
	*/
	static const int s_BANDWIDTH_OF_RB = 12 * 1000 * 15;

	/*
	* 单位(个),由于RB带宽为180kHz，TTI为1ms，因此单位TTI单位RB传输的比特数为180k*1ms=180
	*/
	static const int s_BIT_NUM_PER_RB = 180;

	/*--------------------字段--------------------*/

	/*
	* 总带宽
	*/
private:
	int m_total_bandwidth;
	void set_total_bandwidth(std::string t_total_bandwidth) {
		m_total_bandwidth = stoi(t_total_bandwidth) * 1000 * 1000;
	}
public:
	int get_total_bandwidth() {
		return m_total_bandwidth;
	}

	/*
	* 一个可用资源块的rb数量
	*/
private:
	int m_rb_num_per_pattern;
	void set_rb_num_per_pattern(std::string t_rb_num_per_pattern) {
		m_rb_num_per_pattern = stoi(t_rb_num_per_pattern);
	}
public:
	int get_rb_num_per_pattern() {
		return m_rb_num_per_pattern;
	}

	/*
	* 一个数据包占用多少个pattern
	*/
private:
	int m_packet_pattern_number;
	void set_packet_pattern_number(std::string t_packet_pattern_number) {
		m_packet_pattern_number = stoi(t_packet_pattern_number);
	}
public:
	int get_packet_pattern_number() {
		return m_packet_pattern_number;
	}

	/*
	* pattern数量
	*/
private:
	int m_forward_pattern_num;
	void set_pattern_num() {
		m_forward_pattern_num = get_total_bandwidth() / s_BANDWIDTH_OF_RB / get_rb_num_per_pattern();
	}
public:
	int get_pattern_num() {
		return m_forward_pattern_num;
	}

	/*
	* 丢包临界sinr
	*/
private:
	double m_drop_sinr_boundary;
	void set_drop_sinr_boundary(std::string t_drop_sinr_boundary) {
		m_drop_sinr_boundary = stod(t_drop_sinr_boundary);
	}
public:
	double get_drop_sinr_boundary() {
		return m_drop_sinr_boundary;
	}
	/*
	* MCS等级
	*/
private:
	double m_mcs_level;
	void set_mcs_level(std::string t_mcs_level) {
		m_mcs_level = stod(t_mcs_level);
	}
public:
	double get_mcs_level() {
		return m_mcs_level;
	}
	/*
	* 选择资源选择基本算法
	*/
private:
	int m_select_algorithm;
	void set_select_algorithm(std::string t_select_algorithm) {
		m_select_algorithm = stoi(t_select_algorithm);
	}
public:
	int get_select_algorithm() {
		return m_select_algorithm;
	}

	/*
	* 选择资源选择改进算法
	*/
private:
	int m_enhance_select_algorithm;
	void set_enhance_select_algorithm(std::string t_enchance_select_algorithm) {
		m_enhance_select_algorithm = stoi(t_enchance_select_algorithm);
	}
public:
	int get_enhance_select_algorithm() {
		return m_enhance_select_algorithm;
	}

	/*
	* 节点数据包发送周期	
	*/
private:
	int m_send_interval;
	void set_send_interval(std::string t_send_interval) {
		m_send_interval = stoi(t_send_interval);
	}
public:
	int get_send_interval() {
		return m_send_interval;
	}
};


class tmc_config :public object {
	REGISTE_MEMBER_HEAD(tmc_config)

	/*
	* 事件包数
	*/
private:
	int m_package_num;
	void set_package_num(std::string t_package_num) {
		m_package_num = stoi(t_package_num);
	}
public:
	int get_package_num() {
		return m_package_num;
	}
	/*
	*  事件触发概率
	*/
private:
	double m_trigger_rate;
	void set_trigger_rate(std::string t_trigger_rate) {
		m_trigger_rate = stod(t_trigger_rate);
	}
public:
	double get_trigger_rate() {
		return m_trigger_rate;
	}
};


class route_config :public object {
	REGISTE_MEMBER_HEAD(route_config)

	/*
	* Hello包发送平均周期
	*/
private:
	int m_interval;
	void set_interval(std::string t_interval) {
		m_interval = stoi(t_interval);
	}
public:
	int get_t_interval() {
		return m_interval;
	}
};