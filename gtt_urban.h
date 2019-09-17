#pragma once
#include"reflect/object.h"
#include"gtt.h"
#include"wt.h"

class gtt_urban_config;
class v2x_time;
class rrm_config;

class gtt_urban :public gtt {
	REGISTE_MEMBER_HEAD(gtt_urban)
public:
	friend class wt;
private:
	gtt_urban_config* m_config;
	vue* m_ue_array;
	v2x_time* m_time;
	rrm_config* m_rrm_config;
	int pue_num;
	void set_config(object* t_config) {
		m_config = (gtt_urban_config*)t_config;
	}
	void set_time(object* t_time) {
		m_time = (v2x_time*)t_time;
	}
	void set_rrm_config(object* t_rrm_config) {
		m_rrm_config = (rrm_config*)t_rrm_config;
	}
public:
	gtt_urban_config* get_config() {
		return m_config;
	}
public:
	static const int s_rsu_num = 24;
    static const double s_rsu_topo_ratio[s_rsu_num * 2];
	static const int s_rsu_pattern_id[s_rsu_num];
	std::vector<int> s_rsu_tti;
	//double *ppfStore;//小尺度
	/*--------------------接口--------------------*/
	/*
	* 日志输出流
	*/
private:
	static std::ofstream s_logger_pathloss;//路损输出
	static std::ofstream s_logger_pathloss_los;//路损输出
	static std::ofstream s_logger_pathloss_nlos;//路损输出
	static std::ofstream s_logger_distance;//距离输出
	static std::ofstream s_logger_singlepl;
	static std::ofstream s_logger_singlesf;
public:
	v2x_time* get_time() override {
		return m_time;
	}

	vue* get_ue_array() override {
		return m_ue_array;
	}

	rrm_config* get_rrm_config() override {
		return m_rrm_config;
	}

	void initialize() override;

	int get_ue_num() override;

	int get_vue_num() override;

	int get_pue_num() override;

	int get_rsu_num() override;

	int get_rsu_pattern_id(int rsuid) override;

	int get_rsu_tti(int rsuid) override;

	int get_freshtime() override;

	void fresh_location() override;

	void calculate_pl(int t_vue_id1, int t_vue_id2) override;

	void calculate_smallscale(int t_vue_id1, int t_vue_id2,double* resH) override;
}; 
