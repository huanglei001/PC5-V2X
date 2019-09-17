#pragma once

#include<vector>
#include<utility>
#include<random>
#include<list>

class vue;

class vue_physics {
	/*------------------友元声明------------------*/
	/*
	* 将vue设为友元，由于vue_physics的构造函数设为私有，但可由vue来调用
	*/
	friend class vue;
	friend class gtt_highspeed;
	friend class gtt_urban;
	friend class receiver_event;
	friend class vue_network;
	friend class route_udp;

	/*----------------拷贝控制成员----------------*/
private:
	/*
	* 默认构造函数
	* 设为私有，其初始化完全交给vue来完成，禁止生成该类型的实例
	*/
	vue_physics();

public:
	/*
	* 析构函数，负责清理资源
	*/
	~vue_physics();

	/*
	* 将拷贝构造函数定义为删除
	*/
	vue_physics(const vue_physics& t_vue_physics) = delete;

	/*
	* 将移动构造函数定义为删除
	*/
	vue_physics(vue_physics&& t_vue_physics) = delete;

	/*
	* 将拷贝赋值运算符定义为删除
	*/
	vue_physics& operator=(const vue_physics& t_vue_physics) = delete;

	/*
	* 将移动赋值运算符定义为删除
	*/
	vue_physics& operator=(vue_physics&& t_vue_physics) = delete;

	/*------------------静态成员------------------*/

	/*
	* 用户总数
	*/
private:
	static int s_ue_count;
public:
	static int get_ue_num();
	/*
	* 行人总数
	*/
private:
	static int s_pue_count;
public:
	static int get_pue_num();

	/*
	* 车辆之间的大尺度衰落
	* 前两层下标为车辆id，例如s_pl_all[i][j],且i<j
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
	* 车辆之间的小尺度
	* H共4096个值，前2048为第一路信号FFT变换的实部和虚部（0，2，4...为实部)，后2048为第二路信号的实部和虚部
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
	* 车辆与车辆之间的距离
	* 前两层下标为车辆id，例如s_distance_all[i][j],且i<j
	*/
private:
	static std::vector<std::vector<double>> s_distance_all;
	static void set_distance(int i, int j, double t_distance);
public:
	static double get_distance(int i, int j);
	/*
	* 车辆与车辆之间上一时刻的距离
	* 前两层下标为车辆id，例如s_distance_all_before[i][j],且i<j
	*/
private:
	static std::vector<std::vector<double>> s_distance_all_before;
	static void set_distance_before(int i, int j, double t_distance);
public:
	static double get_distance_before(int i, int j);
	/*
	* 车辆与车辆之间是否为LOS径
	* 是为1，不是为0
	*/
private:
	static std::vector<std::vector<int>> s_LOS;
	static void set_LOS(int i, int j, int los);
public:
	static int get_LOS(int i, int j);

	/*--------------------字段--------------------*/
	/*
	* 指向上层的指针
	*/
private:
	vue* m_superior_level;
	void set_superior_level(vue* t_superior_level);
public:
	vue* get_superior_level();

	/*
	* 用户编号编号
	*/
private:
	int m_ue_id = s_ue_count++;
public:
	int get_ue_id();

	/*
	* 速度，m/s
	*/
private:
	double m_speed = 0;
public:
	double get_speed() {
		return m_speed;
	}
	/*
	* 速度方向,0代表向东，-180代表向西,90代表向北，-90代表向南
	*/
private:
	double m_vangle = 0;
	/*
	* 是否在路口交叉处
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
	* 绝对横坐标，单位m
	*/
private:
	double m_absx = 0;
public:
	double get_absx() {
		return m_absx;
	}
	/*
	* 绝对纵坐标，单位m
	*/
private:
	double m_absy = 0;
public:
	double get_absy() {
		return m_absy;
	}
	/*
	*相对横坐标，单位m
	*/
private:
	double m_relx = 0;

	/*
	* 相对纵坐标，单位m
	*/
private:
	double m_rely = 0;

	/*
	* 所在街区id
	*/
private:
	int m_road_id = -1;

    /*
	* 发送时隙编号，用于时分复用，该区域编号在地理位置更新后更新
	* 该编号假设由基站告知，并且假设基站无缝无重合划分整个区域
	* TTI%granularity==m_slot_time_idx来判断是否可以发送
	* 其中granularity为时分粒度，例如奇偶划分就是2
	*/
private:
	int m_slot_time_idx = 0;
	void set_slot_time_idx(int t_slot_time_idx);
public:
	int get_slot_time_idx();
	/*----------------访问与编辑器----------------*/
public:

	/*--------------------方法--------------------*/
public:
	void update_location_highspeed();
	void update_location_urban();
	void update_location_urban_doubleroad();
};