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
	* ��ȡʱ�����
	*/
	virtual v2x_time* get_time() = 0;

	/*
	* ��ȡ��������ָ��
	*/
	virtual vue* get_ue_array() = 0;

	/*
	* ��ȡ������Դ�������ö���
	*/
	virtual rrm_config* get_rrm_config() = 0;

	/*
	* ��һЩ��ʼ������
	*/
	virtual void initialize() = 0;
	/*
	* ��ȡ��������������
	*/
	virtual int get_ue_num() = 0;

	/*
	* ��ȡ��������
	*/
	virtual int get_vue_num() = 0;

	/*
	* ��ȡRSU����
	*/
	virtual int get_rsu_num() = 0;
	/*
	* ��ȡ��������
	*/
	virtual int get_pue_num() = 0;

	/*
	* ��ȡRSUƵ��
	*/
	virtual int get_rsu_pattern_id(int rsuid) = 0;

	/*
	* ��ȡRSU����TTI
	*/
	virtual int get_rsu_tti(int rsuid) = 0;

	/*
	* ��ȡλ�ø���ʱ��
	*/
	virtual int get_freshtime() = 0;

	/*
	* ���ڸ��³���λ��
	*/
	virtual void fresh_location() = 0;

	/*
	* ���ڼ���ָ���ŵ���Ӧ����
	*/
	virtual void calculate_pl(int t_vue_id1, int t_vue_id2) = 0;

	virtual void calculate_smallscale(int t_vue_id1, int t_vue_id2, double* resH)=0;
};