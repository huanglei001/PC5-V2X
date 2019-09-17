#pragma once
#include<string>
#include"enumeration.h"
#include"reflect/object.h"

class vue;
class v2x_time;
class rrm_config;

class gtt:public object {
public:
	/*
	* 获取时间对象
	*/
	virtual v2x_time* get_time() = 0;

	/*
	* 获取车辆数组指针
	*/
	virtual vue* get_ue_array() = 0;

	/*
	* 获取无线资源管理配置对象
	*/
	virtual rrm_config* get_rrm_config() = 0;

	/*
	* 做一些初始化工作
	*/
	virtual void initialize() = 0;
	/*
	* 获取车辆及行人数量
	*/
	virtual int get_ue_num() = 0;

	/*
	* 获取车辆数量
	*/
	virtual int get_vue_num() = 0;

	/*
	* 获取RSU数量
	*/
	virtual int get_rsu_num() = 0;
	/*
	* 获取行人数量
	*/
	virtual int get_pue_num() = 0;

	/*
	* 获取RSU频段
	*/
	virtual int get_rsu_pattern_id(int rsuid) = 0;

	/*
	* 获取RSU发送TTI
	*/
	virtual int get_rsu_tti(int rsuid) = 0;

	/*
	* 获取位置更新时间
	*/
	virtual int get_freshtime() = 0;

	/*
	* 用于更新车辆位置
	*/
	virtual void fresh_location() = 0;

	/*
	* 用于计算指定信道响应矩阵
	*/
	virtual void calculate_pl(int t_vue_id1, int t_vue_id2) = 0;

	virtual void calculate_smallscale(int t_vue_id1, int t_vue_id2, double* resH)=0;
};