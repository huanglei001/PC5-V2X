#pragma once

#include<vector>
#include<utility>
#include<random>
#include<list>

class vue;

class vue_physics {
	/*------------------��Ԫ����------------------*/
	/*
	* ��vue��Ϊ��Ԫ������vue_physics�Ĺ��캯����Ϊ˽�У�������vue������
	*/
	friend class vue;
	friend class gtt_highspeed;
	friend class gtt_urban;
	friend class receiver_event;
	friend class vue_network;
	friend class route_udp;

	/*----------------�������Ƴ�Ա----------------*/
private:
	/*
	* Ĭ�Ϲ��캯��
	* ��Ϊ˽�У����ʼ����ȫ����vue����ɣ���ֹ���ɸ����͵�ʵ��
	*/
	vue_physics();

public:
	/*
	* ��������������������Դ
	*/
	~vue_physics();

	/*
	* ���������캯������Ϊɾ��
	*/
	vue_physics(const vue_physics& t_vue_physics) = delete;

	/*
	* ���ƶ����캯������Ϊɾ��
	*/
	vue_physics(vue_physics&& t_vue_physics) = delete;

	/*
	* ��������ֵ���������Ϊɾ��
	*/
	vue_physics& operator=(const vue_physics& t_vue_physics) = delete;

	/*
	* ���ƶ���ֵ���������Ϊɾ��
	*/
	vue_physics& operator=(vue_physics&& t_vue_physics) = delete;

	/*------------------��̬��Ա------------------*/

	/*
	* �û�����
	*/
private:
	static int s_ue_count;
public:
	static int get_ue_num();
	/*
	* ��������
	*/
private:
	static int s_pue_count;
public:
	static int get_pue_num();

	/*
	* ����֮��Ĵ�߶�˥��
	* ǰ�����±�Ϊ����id������s_pl_all[i][j],��i<j
	*/
private:
	static std::vector<std::vector<double>> s_pl_all;
	static void set_pl(int i, int j, double t_pl);
public:
	static double get_pl(int i, int j);

private:
	static std::vector<std::vector<double>> s_sn_DS;
	static void set_sn_DS(int i, int j, double t_pl);
public:
	static double get_sn_DS(int i, int j);

private:
	static std::vector<std::vector<double>> s_sn_ASA;
	static void set_sn_ASA(int i, int j, double t_pl);
public:
	static double get_sn_ASA(int i, int j);

private:
	static std::vector<std::vector<double>> s_sn_ASD;
	static void set_sn_ASD(int i, int j, double t_pl);
public:
	static double get_sn_ASD(int i, int j);

private:
	static std::vector<std::vector<double>> s_sn_SF;
	static void set_sn_SF(int i, int j, double t_pl);
public:
	static double get_sn_SF(int i, int j);

	/*
	* ����֮���С�߶�
	* H��4096��ֵ��ǰ2048Ϊ��һ·�ź�FFT�任��ʵ�����鲿��0��2��4...Ϊʵ��)����2048Ϊ�ڶ�·�źŵ�ʵ�����鲿
	*/
private:
	static std::list<std::list<std::list<double>>> hafterFFT;
	static void set_haf(int i, int j,double* H);
	//static std::vector<std::vector<double>> hafterFFT;
	//static void set_haf(int i, int j, double* H);
public:
	//static std::list<double> get_haf(int i, int j);
	//static double* get_haf(int i, int j);
	/*
	* �����복��֮��ľ���
	* ǰ�����±�Ϊ����id������s_distance_all[i][j],��i<j
	*/
private:
	static std::vector<std::vector<double>> s_distance_all;
	static void set_distance(int i, int j, double t_distance);
public:
	static double get_distance(int i, int j);
	/*
	* �����복��֮����һʱ�̵ľ���
	* ǰ�����±�Ϊ����id������s_distance_all_before[i][j],��i<j
	*/
private:
	static std::vector<std::vector<double>> s_distance_all_before;
	static void set_distance_before(int i, int j, double t_distance);
public:
	static double get_distance_before(int i, int j);
	/*
	* �����복��֮���Ƿ�ΪLOS��
	* ��Ϊ1������Ϊ0
	*/
private:
	static std::vector<std::vector<int>> s_LOS;
	static void set_LOS(int i, int j, int los);
public:
	static int get_LOS(int i, int j);

	/*--------------------�ֶ�--------------------*/
	/*
	* ָ���ϲ��ָ��
	*/
private:
	vue* m_superior_level;
	void set_superior_level(vue* t_superior_level);
public:
	vue* get_superior_level();

	/*
	* �û���ű��
	*/
private:
	int m_ue_id = s_ue_count++;
public:
	int get_ue_id();

	/*
	* �ٶȣ�m/s
	*/
private:
	double m_speed = 0;
public:
	double get_speed() {
		return m_speed;
	}
	/*
	* �ٶȷ���,0�����򶫣�-180��������,90�����򱱣�-90��������
	*/
private:
	double m_vangle = 0;
	/*
	* �Ƿ���·�ڽ��洦
	*/
private:
	bool m_in_cross = false;
public:
	bool get_in_cross() {
		return m_in_cross;
	}
	void set_in_cross(bool cross) {
		m_in_cross = cross;
	}
	/*
	* ���Ժ����꣬��λm
	*/
private:
	double m_absx = 0;
public:
	double get_absx() {
		return m_absx;
	}
	/*
	* ���������꣬��λm
	*/
private:
	double m_absy = 0;
public:
	double get_absy() {
		return m_absy;
	}
	/*
	*��Ժ����꣬��λm
	*/
private:
	double m_relx = 0;

	/*
	* ��������꣬��λm
	*/
private:
	double m_rely = 0;

	/*
	* ���ڽ���id
	*/
private:
	int m_road_id = -1;

    /*
	* ����ʱ϶��ţ�����ʱ�ָ��ã����������ڵ���λ�ø��º����
	* �ñ�ż����ɻ�վ��֪�����Ҽ����վ�޷����غϻ�����������
	* TTI%granularity==m_slot_time_idx���ж��Ƿ���Է���
	* ����granularityΪʱ�����ȣ�������ż���־���2
	*/
private:
	int m_slot_time_idx = 0;
	void set_slot_time_idx(int t_slot_time_idx);
public:
	int get_slot_time_idx();
	/*----------------������༭��----------------*/
public:

	/*--------------------����--------------------*/
public:
	void update_location_highspeed();
	void update_location_urban();
	void update_location_urban_doubleroad();
};