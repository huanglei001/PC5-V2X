#pragma once
#include<iostream>
#include<vector>
#include<list>
#include<queue>
#include<set>
#include<map>
#include<utility>
#include<random>
#include<string>
#include"route.h"
#include"config.h"
#include"reflect/object.h"
#include"reflect/context.h"

using namespace std;

enum node_type {
	VUE,
	PUE
};

/*
* 对于路由层，不涉及车辆，将车辆抽象为node
* 对于一个节点，收发矛盾。即同一时刻只能收，或者只能发
* 当一个节点处于收发状态时，回绝一切请求
* 当节点处于收状态时，该节点的作为源节点的信息也将在收完毕后再进行发送，即排在转发消息之后
* 若节点处于空闲状态，且同时收到两个或多个转发请求，随机应答一个，拒绝其他(可以扩展为优先级应答)
*/

class route_udp_route_event {

//-------------根据过去100tti资源占用情况预测挑选资源字段------------//
	/*
	* 该事件广播时所用的时频资源,first是TTI，second是频率资源块
	*/
public:
	pair<int,int> m_resource;
	bool broadcast_flag = false;
	/*
	* 该事件是否已经选择过资源，true是已经选择过，false是还没有选择过
	*/
public:
	bool selected = false;

//-------------演化图中心广播字段------------//
	/*
	* 需要进行广播的节点集合
	*/
public:
	vector<int> m_broadcast_set;

	/*
	* 中心所使用pattern
	*/
public:
	vector<int> m_forward_pattern;

	/*
	* 中心所使用的时间	
	*/
public:
	vector<int> m_forward_tti;

	/*
	* 源节点所使用pattern
	*/
public:
	int m_source_pattern;

	/*
	* 距离源节点最近的转发中心ID，可能是一个也可能是两个
	*/
public:
	vector<int> m_close_forward_node;

	/*
	* 中心所使用时隙
	*/
public:
	vector<int> m_forward_subframe;

//-------------受限延迟广播字段------------//
	/*
	* 受限时延广播中的时延
	*/
public:
	int m_delay_start_tti = 0;
public:
	int get_m_delay_start_tti() {
		return m_delay_start_tti;
	}
	void set_m_delay_start_tti(int new_m_delay_start_tti) {
		m_delay_start_tti = new_m_delay_start_tti;
	}
	void set_broadcast_flag(bool flag) 
	{
		broadcast_flag = flag;
	}

//-------------基本字段------------//
	/*
	* 事件存在的起始时间
	*/
private:
	int m_start_tti = 0;

public:
	int get_start_tti() {
		return m_start_tti;
	}

public:
	static int s_event_count;

	/*
	* 源节点
	*/
private:
	const int m_origin_source_node_id;
public:
	int get_origin_source_node_id() {
		return m_origin_source_node_id;
	}

	/*
	* 目的节点,广播时目的节点为-1
	*/
private:
	const int m_final_destination_node_id;
public:
	int get_final_destination_node_id() {
		return m_final_destination_node_id;
	}

	/*
	* 到目前为止的跳数,初始值为1
	*/
public:
	int m_hop = 1;

	/*
	* 事件id，用来唯一标示触发的事件
	*/
private:
	const int m_event_id;
public:
	int get_event_id() { return m_event_id; }

	/*
	* 数据包持续时间，单位TTI
	*/
private:
	const int m_tti_num;
public:
	int get_tti_num() {
		return m_tti_num;
	}

public:
	/*
	* 构造函数，提供给事件触发模块调用，不同的算法对应不同的重载函数
	*/
//-----------演化图中心算法和简单RSU广播构造函数-----------//
	route_udp_route_event(int t_source_node, int t_destination_node, int current_tti, int event_id, int hop,vector<int> broadcast_set) :
		m_event_id(event_id),
		m_origin_source_node_id(t_source_node),
		m_final_destination_node_id(t_destination_node),
		m_hop(hop),
		m_start_tti(current_tti),
		m_broadcast_set(broadcast_set),
		m_tti_num(((tmc_config*)context::get_context()->get_bean("tmc_config"))->get_package_num()) {
	}

//-----------简单车辆广播以及受限延迟广播构造函数-----------//
	route_udp_route_event(int t_source_node, int t_destination_node, int current_tti, int event_id, int hop) :
		m_event_id(event_id),
		m_origin_source_node_id(t_source_node),
		m_final_destination_node_id(t_destination_node),
		m_hop(hop),
		m_start_tti(current_tti),
		m_tti_num(((tmc_config*)context::get_context()->get_bean("tmc_config"))->get_package_num()) {
	}
//-----------演化图中心算法加资源分配-----------//
	route_udp_route_event(int t_source_node, int t_destination_node, int current_tti, int event_id, int hop, vector<int> center_subframe, vector<int> pattern_set, vector<int> broadcast_set,vector<int> close_node_set) :
		m_event_id(event_id),
		m_origin_source_node_id(t_source_node),
		m_final_destination_node_id(t_destination_node),
		m_hop(hop),
		m_start_tti(current_tti),
		m_forward_subframe(center_subframe),
		m_forward_pattern(pattern_set),
		m_broadcast_set(broadcast_set),
		m_close_forward_node(close_node_set),
		m_tti_num(((tmc_config*)context::get_context()->get_bean("tmc_config"))->get_package_num()) {
	}
//-----------演化图中心算法源资源分配-----------//
	route_udp_route_event(int t_source_node, int t_destination_node, int current_tti, int event_id, int hop, vector<int> center_subframe, vector<int> pattern_set, vector<int> broadcast_set, vector<int> close_node_set,int source_pattern) :
		m_event_id(event_id),
		m_origin_source_node_id(t_source_node),
		m_final_destination_node_id(t_destination_node),
		m_hop(hop),
		m_start_tti(current_tti),
		m_forward_subframe(center_subframe),
		m_forward_pattern(pattern_set),
		m_broadcast_set(broadcast_set),
		m_close_forward_node(close_node_set),
		m_source_pattern(source_pattern),
		m_tti_num(((tmc_config*)context::get_context()->get_bean("tmc_config"))->get_package_num()) {
	}
//-----------演化图中心算法源节点预测转发节点固定-----------//
	route_udp_route_event(int t_source_node, int t_destination_node, int current_tti, int event_id, int hop, vector<int> broadcast_set, vector<int> forward_tti,vector<int> forward_pattern) :
		m_event_id(event_id),
		m_origin_source_node_id(t_source_node),
		m_final_destination_node_id(t_destination_node),
		m_hop(hop),
		m_start_tti(current_tti),
		m_broadcast_set(broadcast_set),
		m_forward_tti(forward_tti),
		m_forward_pattern(forward_pattern),
		m_tti_num(((tmc_config*)context::get_context()->get_bean("tmc_config"))->get_package_num()) {
	}
};

class route_udp_link_event {
	friend class route_udp_node;
	friend class route_udp;
//-------------基本字段------------//
private:
	/*
	* 当前链路源节点id
	*/
	const int m_source_node_id;
public:
	int get_source_node_id() {
		return m_source_node_id;
	}

	/*
	* 当前链路目的节点id
	*/
private:
	const int m_destination_node_id;
public:
	int get_destination_node_id() {
		return m_destination_node_id;
	}

	/*
	* 数据包发送所需时间
	* 任一时刻传输失败则整个数据包传输失败
	*/
private:
	const int m_tti_num;


	/*
	* 占用的pattern编号，根据配置可占用多个连续pattern
	*/
private:
	vector<int> m_forward_pattern_idx;
	void set_pattern_idx(vector<int> t_pattern_idx) {
		m_forward_pattern_idx = t_pattern_idx;
	}
public:
	vector<int> get_pattern_idx() {
		return m_forward_pattern_idx;
	}

	/*
	* 标记本跳当前时刻传输到底几个TTI
	*/
private:
	int m_tti_idx = 0;
public:
	int get_tti_idx() { return m_tti_idx; }

	/*
	* 标记本跳是否传输完毕(无论是否发生丢包)
	*/
private:
	bool m_is_finished = false;
public:
	bool is_finished() { return m_is_finished; }

	/*
	* 标记本跳是否发生丢包
	*/
private:
	bool m_is_loss = false;
public:
	bool get_is_loss() { return m_is_loss; }
	/*
	* 有效载干比
	*/
private:
	double sinrefficient = 0;
public:
	double get_sinrefficient() { return sinrefficient; }

public:
	route_udp_link_event(int t_source_node_id, int t_destination_node_id, vector<int> t_pattern_idx, int t_package_num) :
		m_source_node_id(t_source_node_id),
		m_destination_node_id(t_destination_node_id),
		m_forward_pattern_idx(t_pattern_idx),
		m_tti_num(t_package_num) {}

	/*
	* 进行数据包的发送
	*/
	void transimit();
};

class route_udp_node {
	friend class route_udp;
//-------------根据过去100tti资源占用情况预测挑选资源字段------------//
public:
	queue<vector<double>> power_block;
public:
	pair<int, int> node_resource;
	int counter = 0;//5-15

//-------------基本字段------------//
public:
	map<int, double> success_route_event;//距离成功接收事件编号和距离
	map<int, double> failed_route_event;//距离失败接收事件编号和距离
public:
	int m_broadcast_time;//下次广播的时间
	int m_send_time;//下次可发送的时间
	int m_broadcast_real_time;
public:
	node_type s_node_type;//节点类型，包括车辆节点和RSU节点
	int s_rsu_pattern_id;//RSU节点采用固定频段发送
	int s_rsu_tti;//RSU节点的发送TTI
private:
	/*
	* 正在发送的link_event指针，一个目的节点一个
	*/
	std::vector<route_udp_link_event*> sending_link_event;

	/*
	* 节点总数
	*/
	static int s_node_count;

	/*
	* 随机数引擎
	*/
	static std::default_random_engine s_engine;

	/*
	* 正在发送的node节点
	* 外层下标为pattern编号
	*/
	static std::vector<std::set<int>> s_node_id_per_pattern;
public:
	static const std::set<int>& get_node_id_set(int t_pattern_idx);
	route_udp_route_event* m_broadcast_event_cur;
	/*
	* 当前节点待发送车辆队列
	*/
private:
	std::queue<route_udp_route_event*> m_send_event_queue;  //transfer broadcast event
	std::queue<route_udp_route_event*> m_source_send_event_queue; //source broadcast event

public:
	int get_send_event_queue_length() {
		return m_send_event_queue.size();
	}
	int get_source_send_event_queue_length() {
		return m_source_send_event_queue.size();
	}
public:
	void offer_send_event_queue(route_udp_route_event* t_event) {
		m_send_event_queue.push(t_event);
	}
	void offer_source_send_event_queue(route_udp_route_event* t_event) {
		m_source_send_event_queue.push(t_event);
	}
	route_udp_route_event* poll_send_event_queue() {
		route_udp_route_event* temp = m_send_event_queue.front();
		m_send_event_queue.pop();
		return temp;
	}
	route_udp_route_event* poll_source_send_event_queue() {
		route_udp_route_event* temp = m_source_send_event_queue.front();
		m_source_send_event_queue.pop();
		return temp;
	}
	route_udp_route_event* peek_send_event_queue() {
		return m_send_event_queue.front();
	}
	route_udp_route_event* peek_source_send_event_queue() {
		return m_source_send_event_queue.front();
	}
	bool is_send_event_queue_empty() {
		return m_send_event_queue.empty();
	}
	bool is_send_source_event_queue_empty() {
		return m_source_send_event_queue.empty();
	}

private:
	/*
	* 节点id
	*/
	const int m_id = s_node_count++;
public:
	int get_id() {
		return m_id;
	}
public:
	/*
	* 构造函数
	*/
	route_udp_node();

public:
	/*
	* 选择请求转发的车辆以及相应的频段
	* first字段为车辆id
	* second字段为频段编号,一个编号代表多个pattern
	* 任意一个字段为-1，就代表选择失败
	*/
	std::pair<int, int> select_relay_information();
};

class v2x_time;
class gtt;
class wt;
class rrm_config;
class tmc_config;
class route_config;

class route_udp :public route {
	REGISTE_MEMBER_HEAD(route_udp)
//-------------演化图中心广播字段------------//
public:
	vector<int> center;//中心节点集合
	vector<int> center_pattern;//中心节点分配固定频段
	vector<int> center_subframe;//中心节点分配固定的时隙
	vector<int> center_tti;//中心节点分配固定的TTI发送
	int center_number = 0;//中心节点总数
	int center_received_number = 0;//成功接收的中心节点数量
	int close_center_number = 0;//最近的转发中心节点数量
	int close_center_received_number = 0;//最近的转发中心接收成功的数量

//-------------基本字段------------//
public:
	int pattern_num;//一共有多少pattern
	int algorithm_id;//广播算法
	int packet_pattern_number;//每个数据包要占用多少个连续的pattern
	int resource_select_algorithm;//资源选择基本算法
	int enhance_select_algorithm;//资源选择改进算法
	int interval;//广播事件触发周期
	int send_interval;//节点发送数据包周期
public:
	int s_car_num;//车辆总数
	int s_pue_num;//路边单元总数
	int s_rsu_num;//路边单元总数
private:
	/*
	* 随机数引擎
	*/
	static std::default_random_engine s_engine;

	/*
	* 日志输出流
	*/
	static std::ofstream s_logger_delay;//时延输出
	static std::ofstream s_logger_inter_s_s;//源-源干扰
	static std::ofstream s_logger_inter_s_f;//源-转发干扰
	static std::ofstream s_logger_inter_f_s;//源-源干扰
	static std::ofstream s_logger_inter_f_f;//源-转发干扰
	static std::ofstream s_logger_forward_success;//转发中心为中心，成功到达的广播距离
	static std::ofstream s_logger_forward_failed;//转发中心为中心，失败接收的广播距离
	static std::ofstream s_logger_queue_length;//节点消息队列长度
	static std::ofstream s_logger_broadcast_distance;//广播时距离源节点的距离
	static std::ofstream s_logger_broadcast_delay_between_start_tti;//广播时距离源节点的距离
	static std::ofstream s_logger_sinrefficient;//打印有效载干比
	static std::ofstream s_logger_V2V_success;//打印V2V成功事件距离
	static std::ofstream s_logger_V2P_success;//打印V2P成功事件距离
	static std::ofstream s_logger_P2V_success;//打印P2V成功事件距离
	static std::ofstream s_logger_V2V_fail;//打印V2V失败事件距离
	static std::ofstream s_logger_V2P_fail;//打印V2P失败事件距离
	static std::ofstream s_logger_P2V_fail;//打印P2V失败事件距离

private:
	/*
	* 节点数组
	*/
	route_udp_node* m_node_array;
public:
	route_udp_node* get_node_array() {
		return m_node_array;
	}

private:

	/*
	* 成功/失败传输的事件个数
	*/
	int m_success_route_event_num = 0;

	int m_success_first_route_event_num = 0;

	int m_success_second_route_event_num = 0;

	int m_failed_route_event_num = 0;

	int m_broadcast_num = 0;

	int m_source_broadcast_num = 0;

	int m_forward_broadcast_num = 0;

	int event_num_all = 0;

	int m_event_num = 0;
	int m_send_broadcast_num = 0;
	int m_event_delay_100_tti_num = 0;
	int m_event_second_pop_num = 0;
	int m_event_pop_num;
public:

	int get_success_route_event_num(){
		return m_success_route_event_num;
	}
	int get_first_success_route_event_num() {
		return m_success_first_route_event_num;
	}
	int get_second_success_route_event_num() {
		return m_success_second_route_event_num;
	}
	int get_failed_route_event_num() {
		return m_failed_route_event_num;
	}
	int get_broadcast_num() {
		return m_broadcast_num;
	}
	int get_send_broadcast_num() {
		return m_send_broadcast_num;
	}
	int get_event_num_all() 
	{
		return event_num_all;
	}
	int get_source_broadcast_num() {
		return m_source_broadcast_num;
	}
	int get_forward_broadcast_num() {
		return m_forward_broadcast_num;
	}
	int get_event_num() {
		return m_event_num;
	}
	int get_m_event_delay_100_tti_num() {
		return m_event_delay_100_tti_num;
	}
	int get_m_event_second_pop_num() {
		return m_event_second_pop_num;
	}
	int get_m_event_pop_num() {
		return m_event_pop_num;
	}
private:
	v2x_time* m_time;
	gtt* m_gtt;
	wt* m_wt;
	rrm_config* m_rrm_config;
	tmc_config* m_tmc_config;
	route_config* m_route_config;

	void set_time(object* t_time) {
		m_time = (v2x_time*)t_time;
	}
	void set_gtt(object* t_gtt) {
		m_gtt = (gtt*)t_gtt;
	}
	void set_wt(object* t_wt) {
		m_wt = (wt*)t_wt;
	}
	void set_rrm_config(object* t_rrm_config) {
		m_rrm_config = (rrm_config*)t_rrm_config;
	}
	void set_tmc_config(object* t_tmc_config) {
		m_tmc_config = (tmc_config*)t_tmc_config;
	}
	void set_route_config(object* t_route_config) {
		m_route_config = (route_config*)t_route_config;
	}
public:
	v2x_time* get_time() override {
		return m_time;
	}

	gtt* get_gtt() override {
		return m_gtt;
	}

	wt* get_wt() override {
		return m_wt;
	}

	rrm_config* get_rrm_config() override {
		return m_rrm_config;
	}

	tmc_config* get_tmc_config() override {
		return m_tmc_config;
	}

	route_config* get_route_config() override {
		return m_route_config;
	}

	void initialize() override;

	void process_per_tti() override;

private:
//-------------基本函数------------//
	/*
	* 随机触发事件
	*/
	void event_trigger();

	/*
	* 触发要开始发送的事件
	*/
	void start_sending_data();

	/*
	* 传输当前TTI存在的事件
	*/
	void transmit_data();

//-------------统计输出函数------------//
	/*
	* 统计输出信干噪比函数
	*/
	void outputpower(int source_node_id);

//-------------根据过去100tti资源占用情况预测挑选资源字段------------//
	/*
	* 计算每个block上的能量总和
	*/
	void calculate_power();

	/*
	* 挑选用于发送数据包的时频资源
	*/
	pair<int,int> select_resource(int node_id);

//-------------演化图中心广播函数------------//
	/*
	* 选择RSU
	*/
	vector<int> select_rsu(int vueid);

	/*
	* 演化图选择中心
	*/
	void center_selection();
	void center_pattern_selection();
	void route_udp::forward_center_selection(int origin_node, vector<int>* forward_node_set,vector<int>* forward_node_pattern_set,vector<int>* forward_node_subframe_set,vector<int>* close_forward_node_set,int* source_pattern,vector<int>* forward_tti);

	/*
	* 冒泡排序
	*/
	void bubble_sort(vector<double> &nums, vector<int> &index);
	void bubble_sort(vector<double> &nums);
};
