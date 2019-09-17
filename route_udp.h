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
* ����·�ɲ㣬���漰����������������Ϊnode
* ����һ���ڵ㣬�շ�ì�ܡ���ͬһʱ��ֻ���գ�����ֻ�ܷ�
* ��һ���ڵ㴦���շ�״̬ʱ���ؾ�һ������
* ���ڵ㴦����״̬ʱ���ýڵ����ΪԴ�ڵ����ϢҲ��������Ϻ��ٽ��з��ͣ�������ת����Ϣ֮��
* ���ڵ㴦�ڿ���״̬����ͬʱ�յ���������ת���������Ӧ��һ�����ܾ�����(������չΪ���ȼ�Ӧ��)
*/

class route_udp_route_event {

//-------------���ݹ�ȥ100tti��Դռ�����Ԥ����ѡ��Դ�ֶ�------------//
	/*
	* ���¼��㲥ʱ���õ�ʱƵ��Դ,first��TTI��second��Ƶ����Դ��
	*/
public:
	pair<int,int> m_resource;
	bool broadcast_flag = false;
	/*
	* ���¼��Ƿ��Ѿ�ѡ�����Դ��true���Ѿ�ѡ�����false�ǻ�û��ѡ���
	*/
public:
	bool selected = false;

//-------------�ݻ�ͼ���Ĺ㲥�ֶ�------------//
	/*
	* ��Ҫ���й㲥�Ľڵ㼯��
	*/
public:
	vector<int> m_broadcast_set;

	/*
	* ������ʹ��pattern
	*/
public:
	vector<int> m_forward_pattern;

	/*
	* ������ʹ�õ�ʱ��	
	*/
public:
	vector<int> m_forward_tti;

	/*
	* Դ�ڵ���ʹ��pattern
	*/
public:
	int m_source_pattern;

	/*
	* ����Դ�ڵ������ת������ID��������һ��Ҳ����������
	*/
public:
	vector<int> m_close_forward_node;

	/*
	* ������ʹ��ʱ϶
	*/
public:
	vector<int> m_forward_subframe;

//-------------�����ӳٹ㲥�ֶ�------------//
	/*
	* ����ʱ�ӹ㲥�е�ʱ��
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

//-------------�����ֶ�------------//
	/*
	* �¼����ڵ���ʼʱ��
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
	* Դ�ڵ�
	*/
private:
	const int m_origin_source_node_id;
public:
	int get_origin_source_node_id() {
		return m_origin_source_node_id;
	}

	/*
	* Ŀ�Ľڵ�,�㲥ʱĿ�Ľڵ�Ϊ-1
	*/
private:
	const int m_final_destination_node_id;
public:
	int get_final_destination_node_id() {
		return m_final_destination_node_id;
	}

	/*
	* ��ĿǰΪֹ������,��ʼֵΪ1
	*/
public:
	int m_hop = 1;

	/*
	* �¼�id������Ψһ��ʾ�������¼�
	*/
private:
	const int m_event_id;
public:
	int get_event_id() { return m_event_id; }

	/*
	* ���ݰ�����ʱ�䣬��λTTI
	*/
private:
	const int m_tti_num;
public:
	int get_tti_num() {
		return m_tti_num;
	}

public:
	/*
	* ���캯�����ṩ���¼�����ģ����ã���ͬ���㷨��Ӧ��ͬ�����غ���
	*/
//-----------�ݻ�ͼ�����㷨�ͼ�RSU�㲥���캯��-----------//
	route_udp_route_event(int t_source_node, int t_destination_node, int current_tti, int event_id, int hop,vector<int> broadcast_set) :
		m_event_id(event_id),
		m_origin_source_node_id(t_source_node),
		m_final_destination_node_id(t_destination_node),
		m_hop(hop),
		m_start_tti(current_tti),
		m_broadcast_set(broadcast_set),
		m_tti_num(((tmc_config*)context::get_context()->get_bean("tmc_config"))->get_package_num()) {
	}

//-----------�򵥳����㲥�Լ������ӳٹ㲥���캯��-----------//
	route_udp_route_event(int t_source_node, int t_destination_node, int current_tti, int event_id, int hop) :
		m_event_id(event_id),
		m_origin_source_node_id(t_source_node),
		m_final_destination_node_id(t_destination_node),
		m_hop(hop),
		m_start_tti(current_tti),
		m_tti_num(((tmc_config*)context::get_context()->get_bean("tmc_config"))->get_package_num()) {
	}
//-----------�ݻ�ͼ�����㷨����Դ����-----------//
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
//-----------�ݻ�ͼ�����㷨Դ��Դ����-----------//
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
//-----------�ݻ�ͼ�����㷨Դ�ڵ�Ԥ��ת���ڵ�̶�-----------//
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
//-------------�����ֶ�------------//
private:
	/*
	* ��ǰ��·Դ�ڵ�id
	*/
	const int m_source_node_id;
public:
	int get_source_node_id() {
		return m_source_node_id;
	}

	/*
	* ��ǰ��·Ŀ�Ľڵ�id
	*/
private:
	const int m_destination_node_id;
public:
	int get_destination_node_id() {
		return m_destination_node_id;
	}

	/*
	* ���ݰ���������ʱ��
	* ��һʱ�̴���ʧ�����������ݰ�����ʧ��
	*/
private:
	const int m_tti_num;


	/*
	* ռ�õ�pattern��ţ��������ÿ�ռ�ö������pattern
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
	* ��Ǳ�����ǰʱ�̴��䵽�׼���TTI
	*/
private:
	int m_tti_idx = 0;
public:
	int get_tti_idx() { return m_tti_idx; }

	/*
	* ��Ǳ����Ƿ������(�����Ƿ�������)
	*/
private:
	bool m_is_finished = false;
public:
	bool is_finished() { return m_is_finished; }

	/*
	* ��Ǳ����Ƿ�������
	*/
private:
	bool m_is_loss = false;
public:
	bool get_is_loss() { return m_is_loss; }
	/*
	* ��Ч�ظɱ�
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
	* �������ݰ��ķ���
	*/
	void transimit();
};

class route_udp_node {
	friend class route_udp;
//-------------���ݹ�ȥ100tti��Դռ�����Ԥ����ѡ��Դ�ֶ�------------//
public:
	queue<vector<double>> power_block;
public:
	pair<int, int> node_resource;
	int counter = 0;//5-15

//-------------�����ֶ�------------//
public:
	map<int, double> success_route_event;//����ɹ������¼���ź;���
	map<int, double> failed_route_event;//����ʧ�ܽ����¼���ź;���
public:
	int m_broadcast_time;//�´ι㲥��ʱ��
	int m_send_time;//�´οɷ��͵�ʱ��
	int m_broadcast_real_time;
public:
	node_type s_node_type;//�ڵ����ͣ����������ڵ��RSU�ڵ�
	int s_rsu_pattern_id;//RSU�ڵ���ù̶�Ƶ�η���
	int s_rsu_tti;//RSU�ڵ�ķ���TTI
private:
	/*
	* ���ڷ��͵�link_eventָ�룬һ��Ŀ�Ľڵ�һ��
	*/
	std::vector<route_udp_link_event*> sending_link_event;

	/*
	* �ڵ�����
	*/
	static int s_node_count;

	/*
	* ���������
	*/
	static std::default_random_engine s_engine;

	/*
	* ���ڷ��͵�node�ڵ�
	* ����±�Ϊpattern���
	*/
	static std::vector<std::set<int>> s_node_id_per_pattern;
public:
	static const std::set<int>& get_node_id_set(int t_pattern_idx);
	route_udp_route_event* m_broadcast_event_cur;
	/*
	* ��ǰ�ڵ�����ͳ�������
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
	* �ڵ�id
	*/
	const int m_id = s_node_count++;
public:
	int get_id() {
		return m_id;
	}
public:
	/*
	* ���캯��
	*/
	route_udp_node();

public:
	/*
	* ѡ������ת���ĳ����Լ���Ӧ��Ƶ��
	* first�ֶ�Ϊ����id
	* second�ֶ�ΪƵ�α��,һ����Ŵ�����pattern
	* ����һ���ֶ�Ϊ-1���ʹ���ѡ��ʧ��
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
//-------------�ݻ�ͼ���Ĺ㲥�ֶ�------------//
public:
	vector<int> center;//���Ľڵ㼯��
	vector<int> center_pattern;//���Ľڵ����̶�Ƶ��
	vector<int> center_subframe;//���Ľڵ����̶���ʱ϶
	vector<int> center_tti;//���Ľڵ����̶���TTI����
	int center_number = 0;//���Ľڵ�����
	int center_received_number = 0;//�ɹ����յ����Ľڵ�����
	int close_center_number = 0;//�����ת�����Ľڵ�����
	int close_center_received_number = 0;//�����ת�����Ľ��ճɹ�������

//-------------�����ֶ�------------//
public:
	int pattern_num;//һ���ж���pattern
	int algorithm_id;//�㲥�㷨
	int packet_pattern_number;//ÿ�����ݰ�Ҫռ�ö��ٸ�������pattern
	int resource_select_algorithm;//��Դѡ������㷨
	int enhance_select_algorithm;//��Դѡ��Ľ��㷨
	int interval;//�㲥�¼���������
	int send_interval;//�ڵ㷢�����ݰ�����
public:
	int s_car_num;//��������
	int s_pue_num;//·�ߵ�Ԫ����
	int s_rsu_num;//·�ߵ�Ԫ����
private:
	/*
	* ���������
	*/
	static std::default_random_engine s_engine;

	/*
	* ��־�����
	*/
	static std::ofstream s_logger_delay;//ʱ�����
	static std::ofstream s_logger_inter_s_s;//Դ-Դ����
	static std::ofstream s_logger_inter_s_f;//Դ-ת������
	static std::ofstream s_logger_inter_f_s;//Դ-Դ����
	static std::ofstream s_logger_inter_f_f;//Դ-ת������
	static std::ofstream s_logger_forward_success;//ת������Ϊ���ģ��ɹ�����Ĺ㲥����
	static std::ofstream s_logger_forward_failed;//ת������Ϊ���ģ�ʧ�ܽ��յĹ㲥����
	static std::ofstream s_logger_queue_length;//�ڵ���Ϣ���г���
	static std::ofstream s_logger_broadcast_distance;//�㲥ʱ����Դ�ڵ�ľ���
	static std::ofstream s_logger_broadcast_delay_between_start_tti;//�㲥ʱ����Դ�ڵ�ľ���
	static std::ofstream s_logger_sinrefficient;//��ӡ��Ч�ظɱ�
	static std::ofstream s_logger_V2V_success;//��ӡV2V�ɹ��¼�����
	static std::ofstream s_logger_V2P_success;//��ӡV2P�ɹ��¼�����
	static std::ofstream s_logger_P2V_success;//��ӡP2V�ɹ��¼�����
	static std::ofstream s_logger_V2V_fail;//��ӡV2Vʧ���¼�����
	static std::ofstream s_logger_V2P_fail;//��ӡV2Pʧ���¼�����
	static std::ofstream s_logger_P2V_fail;//��ӡP2Vʧ���¼�����

private:
	/*
	* �ڵ�����
	*/
	route_udp_node* m_node_array;
public:
	route_udp_node* get_node_array() {
		return m_node_array;
	}

private:

	/*
	* �ɹ�/ʧ�ܴ�����¼�����
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
//-------------��������------------//
	/*
	* ��������¼�
	*/
	void event_trigger();

	/*
	* ����Ҫ��ʼ���͵��¼�
	*/
	void start_sending_data();

	/*
	* ���䵱ǰTTI���ڵ��¼�
	*/
	void transmit_data();

//-------------ͳ���������------------//
	/*
	* ͳ������Ÿ���Ⱥ���
	*/
	void outputpower(int source_node_id);

//-------------���ݹ�ȥ100tti��Դռ�����Ԥ����ѡ��Դ�ֶ�------------//
	/*
	* ����ÿ��block�ϵ������ܺ�
	*/
	void calculate_power();

	/*
	* ��ѡ���ڷ������ݰ���ʱƵ��Դ
	*/
	pair<int,int> select_resource(int node_id);

//-------------�ݻ�ͼ���Ĺ㲥����------------//
	/*
	* ѡ��RSU
	*/
	vector<int> select_rsu(int vueid);

	/*
	* �ݻ�ͼѡ������
	*/
	void center_selection();
	void center_pattern_selection();
	void route_udp::forward_center_selection(int origin_node, vector<int>* forward_node_set,vector<int>* forward_node_pattern_set,vector<int>* forward_node_subframe_set,vector<int>* close_forward_node_set,int* source_pattern,vector<int>* forward_tti);

	/*
	* ð������
	*/
	void bubble_sort(vector<double> &nums, vector<int> &index);
	void bubble_sort(vector<double> &nums);
};
