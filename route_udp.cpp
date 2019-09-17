#include<iostream>
#include<iomanip>
#include<fstream>
#include<sstream>
#include<algorithm>
#include"route_udp.h"
#include"config.h"
#include"gtt.h"
#include"wt.h"
#include"vue.h"
#include"vue_physics.h"
#include"function.h"
#include"reflect/context.h"
#include"time_stamp.h"

using namespace std;

//静态成员初始化
int route_udp_route_event::s_event_count = 0;

default_random_engine route_udp::s_engine;

int route_udp_node::s_node_count = 0;//节点总数（行人及车辆）

default_random_engine route_udp_node::s_engine(time(NULL));

std::vector<std::set<int>> route_udp_node::s_node_id_per_pattern;

ofstream route_udp::s_logger_delay;
ofstream route_udp::s_logger_inter_s_s;
ofstream route_udp::s_logger_inter_s_f;
ofstream route_udp::s_logger_inter_f_s;
ofstream route_udp::s_logger_inter_f_f;
ofstream route_udp::s_logger_forward_success;
ofstream route_udp::s_logger_forward_failed;
ofstream route_udp::s_logger_queue_length;
ofstream route_udp::s_logger_broadcast_distance;
ofstream route_udp::s_logger_broadcast_delay_between_start_tti;
ofstream route_udp::s_logger_sinrefficient;
ofstream route_udp::s_logger_V2V_success;//打印V2V成功事件距离
ofstream route_udp::s_logger_V2P_success;//打印V2P成功事件距离
ofstream route_udp::s_logger_P2V_success;//打印P2V成功事件距离
ofstream route_udp::s_logger_V2V_fail;//打印V2V失败事件距离
ofstream route_udp::s_logger_V2P_fail;//打印V2P失败事件距离
ofstream route_udp::s_logger_P2V_fail;//打印P2V失败事件距离

									  //bler=0.1的snr值
double g_MCSBound_uplink[29] = { 0.211896,0.331131,0.394198,0.468218,0.572769,0.719025,0.849572,1.081477,1.305817,1.597595,2.097526,2.217732,2.540644,3.235289,3.798928,4.725782,5.572635,6.703082,7.413102,8.841897,10.606376,13.617207,16.149782,19.221597,23.794819,28.323521,32.899899,38.591861,71.395508 };
//snr到bler曲线的参数
double g_msc_snr_bler_uplink_b[29] = { 0.162398,0.275259,0.331273,0.387216,0.500396,0.632055,0.757767,0.976645,1.181258,1.450097,1.891839,2.021082,2.310004,2.959804,3.490522,4.358722,5.116073,6.160274,6.791948,8.179674,9.797276,12.584850,14.960581,17.775798,22.079626,26.184039,30.701693,35.817768,66.378959 };
double g_mcs_snr_bler_uplink_c[29] = { 0.054680,0.057094,0.070231,0.088679,0.080189,0.090378,0.097123,0.112113,0.130679,0.148960,0.207821,0.208571,0.244529,0.283471,0.329564,0.370230,0.471146,0.537504,0.609931,0.696236,0.804291,1.015040,1.199394,1.402330,1.786923,2.067856,2.337520,2.848863,4.948657 };
//查找误码率  
double LUT_SNR2BLER(double snr, int mcs_level)
{
	double bmcs = g_msc_snr_bler_uplink_b[mcs_level];
	double cmcs = g_mcs_snr_bler_uplink_c[mcs_level];
	return 0.5f*(1.0f - (float)erf(((snr - bmcs) / cmcs)));
}

//所有link_event传输一个TTI
void route_udp_link_event::transimit() {
	context* __context = context::get_context();
	int packet_pattern_number = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();
	
	if (++m_tti_idx == m_tti_num) {
		m_is_finished = true;
	}

	vector<set<int>> inter_node_set;
	for (int i = 0; i < packet_pattern_number; i++) {
		inter_node_set.push_back(route_udp_node::get_node_id_set(get_pattern_idx()[i]));
	}
//--------------输出不同干扰在接收端的信干比---------------//
	double sinr;
	if(((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_small_scale()){
		sinr = ((wt*)context::get_context()->get_bean("wt"))->calculate_sinrsmallscale(
			get_source_node_id(),
			get_destination_node_id(),
			get_pattern_idx(),
			inter_node_set);
	}
	else {
	    sinr = ((wt*)context::get_context()->get_bean("wt"))->calculate_sinr(
			get_source_node_id(),
			get_destination_node_id(),
			get_pattern_idx(),
			inter_node_set);
	}
	sinrefficient = sinr;

	int mcs_id = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_mcs_level();
	double boundary = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_drop_sinr_boundary();
	double randerr = 1.0*rand() / RAND_MAX;
	if (sinr < boundary - 0.5) {
		m_is_loss = true;
	}
	else if ((sinr >= boundary - 0.5) && (sinr <= boundary + 0.5)) {
		double err = LUT_SNR2BLER(pow(10, sinr / 10), mcs_id);
		if (randerr < err) {
			m_is_loss = true;
		}
	}
	//if (sinr < ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_drop_sinr_boundary()){
	//	m_is_loss = true;
	//}
}

//返回每个频段上正在占用的车辆ID
const std::set<int>& route_udp_node::get_node_id_set(int t_pattern_idx) {
	return s_node_id_per_pattern[t_pattern_idx];
}


//node初始化
route_udp_node::route_udp_node() {
	context* __context = context::get_context();
	int interval = ((route_config*)__context->get_bean("route_config"))->get_t_interval();
	uniform_int_distribution<int> u_start_broadcast_tti(0, interval - 1);
	uniform_int_distribution<int> u_start_send_tti(0, 49);
	uniform_int_distribution<int> u_counter(5, 15);
	uniform_int_distribution<int> u_resource_time(4, 20);
	uniform_int_distribution<int> u_resource_pattern(0, 3);

	int carnum = ((gtt*)__context->get_bean("gtt"))->get_vue_num();
	if (get_id() < carnum) {
		s_node_type = VUE;
	}
	else {
		s_node_type = PUE;
	}

	m_broadcast_time = u_start_broadcast_tti(s_engine);//初始化第一次发送周期消息的时间，用于错开干扰
	m_send_time = u_start_send_tti(s_engine);//初始化第一次可以发送数据包的时间，用于错开干扰

	counter = u_counter(s_engine);
	node_resource = make_pair(u_resource_time(s_engine), u_resource_pattern(s_engine));
	m_broadcast_real_time = node_resource.first + m_broadcast_time; //真实的广播分组的时间
}


//udp初始化
void route_udp::initialize() {
	context* __context = context::get_context();
	int ue_num = get_gtt()->get_ue_num();
	m_node_array = new route_udp_node[ue_num];

	s_logger_delay.open("log/route_udp_delay.txt");
	s_logger_inter_s_s.open("log/route_udp_inter_s_s.txt");
	s_logger_inter_s_f.open("log/route_udp_inter_s_f.txt");
	s_logger_inter_f_s.open("log/route_udp_inter_f_s.txt");
	s_logger_inter_f_f.open("log/route_udp_inter_f_f.txt");
	s_logger_forward_success.open("log/route_udp_forward_success.txt");
	s_logger_forward_failed.open("log/route_udp_forward_failed.txt");
	s_logger_queue_length.open("log/route_udp_queue_length.txt");
	s_logger_broadcast_distance.open("log/route_udp_broadcast_distance.txt");
	s_logger_broadcast_delay_between_start_tti.open("log/route_udp_delay_between_start_tti.txt");
	s_logger_sinrefficient.open("log/s_logger_sinrefficient.txt");
	s_logger_V2V_success.open("log/s_logger_V2V_success.txt");
	s_logger_V2P_success.open("log/s_logger_V2P_success.txt");
	s_logger_P2V_success.open("log/s_logger_P2V_success.txt");
	s_logger_V2V_fail.open("log/s_logger_V2V_fail.txt");
	s_logger_V2P_fail.open("log/s_logger_V2P_fail.txt");
	s_logger_P2V_fail.open("log/s_logger_P2V_fail.txt");
	route_udp_node::s_node_id_per_pattern = vector<set<int>>(get_rrm_config()->get_pattern_num());

	s_car_num = get_gtt()->get_vue_num();
	s_pue_num = get_gtt()->get_pue_num();

	pattern_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_pattern_num();
    algorithm_id = ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_select_broadcast_algorithm();
	packet_pattern_number = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();
	resource_select_algorithm = ((rrm_config*)__context->get_bean("rrm_config"))->get_select_algorithm();
	enhance_select_algorithm = ((rrm_config*)__context->get_bean("rrm_config"))->get_enhance_select_algorithm();
	interval = ((route_config*)__context->get_bean("route_config"))->get_t_interval();//周期事件触发间隔
	send_interval = ((rrm_config*)__context->get_bean("rrm_config"))->get_send_interval();
}

//每个TTI的流程
void route_udp::process_per_tti() {
	//事件触发
	event_trigger();

    //触发要开始发送的事件
	start_sending_data();

	//传输当前TTI存在的事件
	transmit_data();

	//输出当前时刻各个节点的队列长度
	for (int i = 0; i < route_udp_node::s_node_count; i++) {
		int temp = get_node_array()[i].m_send_event_queue.size();
		s_logger_queue_length << temp << " ";
	}
	s_logger_queue_length << endl;
}

//事件触发
void route_udp::event_trigger() {
	context* __context = context::get_context();

	if (get_time()->get_tti() < ((global_control_config*)__context->get_bean("global_control_config"))->get_ntti()) {//仿真时间内触发

		//在初始化时间过后，触发数据传输事件,根据广播算法的不同事件触发方式也不同
		for (int origin_source_node_id = 0; origin_source_node_id < route_udp_node::s_node_count; origin_source_node_id++) {
			route_udp_node& source_node = get_node_array()[origin_source_node_id];

			//计算每个车辆的触发概率
			uniform_int_distribution<int> u_send_chance(0, 100);

			if (get_time()->get_tti() % interval == source_node.m_broadcast_time&&u_send_chance(s_engine) <= ((tmc_config*)__context->get_bean("tmc_config"))->get_trigger_rate()*100) {//满足触发概率才触发				
				//V2V广播
				if (algorithm_id == 1) {
					if (source_node.s_node_type == PUE) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2P广播
				if (algorithm_id == 2) {

					if (source_node.s_node_type == PUE) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//P2V广播
				if (algorithm_id == 3) 
				{
					if (source_node.s_node_type == VUE) continue;
					//if (get_time()->get_tti() % (interval * 10) != source_node.m_broadcast_time) continue;//p2v周期控制
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1));//当前节点当前时刻触发的源广播事件。
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;//标记该接收节点已经收到过此事件，避免重复接收
					m_event_num++;
				}
				//V2V+V2P广播
				if (algorithm_id == 4) {
					if (source_node.s_node_type == PUE) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2V+P2V广播
				if (algorithm_id == 5) {
					if (source_node.s_node_type == PUE&&get_time()->get_tti() % (interval*10) != source_node.m_broadcast_time) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2P+P2V广播
				if (algorithm_id == 6) {
					if (source_node.s_node_type == PUE&&get_time()->get_tti() % (interval * 10) != source_node.m_broadcast_time) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2V+V2P+P2V广播
				if (algorithm_id == 7) {
					//if (source_node.s_node_type == PUE&&get_time()->get_tti() % (interval * 10) != source_node.m_broadcast_time) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
			}
		}
	}
}

//开始传输route_event
void route_udp::start_sending_data()                                                                                                                                                                                                                                                                         {
	context* __context = context::get_context();

	//本着同一时刻发消息优先于收消息的原则，所有发消息的事件在传输前先选择传输频段并进行占用
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];

		if (source_node.is_send_event_queue_empty()) continue;//当前车辆待发送事件列表为空，跳过即可
		
		if (source_node.sending_link_event.size() == 0) {//当前节点上一个事件已经完成传输或者没有要传输的事件

			if (!source_node.is_send_event_queue_empty()) 
			{
				//判断数据包存在时间是否超过最大端到端时延，如果超过则直接丢弃数据包
				while (get_time()->get_tti() - source_node.m_send_event_queue.front()->get_start_tti() > ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_max_delay()) {
					source_node.m_send_event_queue.pop();
					m_event_pop_num++;
					if (source_node.is_send_event_queue_empty()) break;
				}
			}
			//如果队列为空则继续遍历下一个节点		
			if (source_node.is_send_event_queue_empty()) continue;
		
			int select_patternid;//最终挑选出的频段序号，单个数字
			vector<int> select_pattern;//最终挑选出的用于传输的频段，可以是多个pattern		
			vector<int> temp_broadcast_set;
			vector<int> temp_forward_tti;
			vector<int> temp_forward_pattern;
			vector<int>::iterator it11;

			
			if (enhance_select_algorithm == 0) {//如果不使用改进过的资源分配算法
				int pattern_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_pattern_num();
				int packet_pattern_number = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();
				uniform_int_distribution<int> u_pattern(0, (pattern_num + 1 - packet_pattern_number) - 1);
				select_patternid = u_pattern(s_engine);
				//pair<int, int> select_res = source_node.select_relay_information();
				//select_patternid = select_res.second;
			}
			else if (enhance_select_algorithm == 1) {//如果采用过去时刻预测的资源选择算法则判断是否到达发送事件
					
					if (source_node.m_send_event_queue.front()->selected == false) {
							//如果采用根据过去时刻预测的资源选择算法
							if (source_node.counter != 0) {
								source_node.m_send_event_queue.front()->m_resource = make_pair(get_time()->get_tti() + source_node.node_resource.first, source_node.node_resource.second);
							}
							else {
								pair<int, int> resource = select_resource(source_node_id);
								source_node.m_send_event_queue.front()->m_resource = make_pair(get_time()->get_tti() + resource.first, resource.second);
								source_node.node_resource = resource;
								uniform_int_distribution<int> u_counter(15, 40);
								source_node.counter = u_counter(s_engine);
							}
							source_node.m_send_event_queue.front()->selected = true;
					}
						//判断是否到达发送时间，没到则遍历到下一个节点，到则继续创建链路
					if (get_time()->get_tti() != source_node.m_send_event_queue.front()->m_resource.first)//发送时间存储在m_resource中
						continue;
					else 
						source_node.counter--;
					select_patternid = source_node.m_send_event_queue.front()->m_resource.second;	
				}
				
			//将单个pattern组合变为若干pattern
			for (int i = select_patternid; i <= select_patternid + packet_pattern_number - 1; i++) {
				select_pattern.push_back(i);
			}


			//根据广播算法选择接收节点,创建链路事件进行广播
			for (int dst_id = 0; dst_id < route_udp_node::s_node_count; dst_id++) {

				if (algorithm_id == 1 && dst_id >= s_car_num) continue; //V2V
				else if (algorithm_id == 2 && dst_id < s_car_num) continue; //V2P								
				else if (algorithm_id == 3 && dst_id >= s_car_num) continue; //P2V		
				else if (algorithm_id == 5 && dst_id >= s_car_num) continue; //V2V+P2V
				else if (algorithm_id == 6) {//V2P+P2V
					if (source_node.s_node_type == VUE && dst_id < s_car_num) continue;
					else if (source_node.s_node_type == PUE && dst_id >= s_car_num) continue;					
				}
				else if (algorithm_id == 7) {//V2V+V2P+P2V
					if (source_node.s_node_type == PUE && dst_id >= s_car_num) continue;
				}

				//接收节点只针对源节点一定范围内的节点
				if (dst_id == source_node_id || vue_physics::get_distance(source_node.m_send_event_queue.front()->get_origin_source_node_id(), dst_id) >= ((global_control_config*)__context->get_bean("global_control_config"))->get_max_distance()) continue;
				
				map<int, double>::iterator marked = get_node_array()[dst_id].success_route_event.find(source_node.m_send_event_queue.front()->get_event_id());
				source_node.sending_link_event.push_back(new route_udp_link_event(
					source_node_id, dst_id, select_pattern, source_node.peek_send_event_queue()->get_tti_num()));
			}
			source_node.m_send_event_queue.front()->set_broadcast_flag(true);
			if (source_node.sending_link_event.size() == 0) {
				throw logic_error("error");//如果广播接收节点均已完成接收则不再建立广播连接
			}
			else {
				m_broadcast_num++;
			}				
		}
	}
}

//每个TTI传输link_event
void route_udp::transmit_data() {
	context *__context = context::get_context();
	
	//第一遍遍历，判断每个节点的link_event是否要在当前TTI传输，如果需要则加入干扰列表
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];

		if (source_node.sending_link_event.size() == 0) continue;
		m_send_broadcast_num++;
		
		//如果当前时刻该节点的link_event需要传输
		//int temp_subframe = get_time()->get_tti() % 2;
		//vector<int> temp_forward_node = source_node.m_send_event_queue.front()->m_broadcast_set;
		//vector<int>::iterator it33 = find(temp_forward_node.begin(), temp_forward_node.end(), source_node_id);
		//if (it33 != temp_forward_node.end()) {
		//	int index = it33 - temp_forward_node.begin();
		//	if (temp_subframe != source_node.m_send_event_queue.front()->m_forward_subframe[index]) continue;
		//}
		//int temp_hop = source_node.m_send_event_queue.front()->m_hop;
		//if ((temp_hop == 1 && temp_subframe == 1) || (temp_hop != 1 && temp_subframe == 0)) continue;
		
		
		for (auto p : source_node.sending_link_event[0]->get_pattern_idx()) {//加入干扰列表
			if (route_udp_node::s_node_id_per_pattern[p].find(source_node_id) != route_udp_node::s_node_id_per_pattern[p].end()) 
				throw logic_error("error");
			route_udp_node::s_node_id_per_pattern[p].insert(source_node_id);
		}
	}
	//-----------预测方式使用频段函数-----------//
	if (enhance_select_algorithm == 1) {
		//计算每个block上的能量总和
		calculate_power();
	}

	//对所有link_event进行第二遍遍历，传输所有link_event
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];
		if (source_node.sending_link_event.size() == 0) continue;
		//if (source_node_id < s_car_num) continue;

		//如果当前时刻该节点的link_event需要传输
		//int temp_subframe = get_time()->get_tti() % 2;
		/*vector<int> temp_forward_node = source_node.m_send_event_queue.front()->m_broadcast_set;
		vector<int>::iterator it33 = find(temp_forward_node.begin(), temp_forward_node.end(), source_node_id);
		if (it33 != temp_forward_node.end()) {
			int index = it33 - temp_forward_node.begin();
			if (temp_subframe != source_node.m_send_event_queue.front()->m_forward_subframe[index]) continue;
		}*/
		//int temp_hop = source_node.m_send_event_queue.front()->m_hop;
		//if ((temp_hop == 1 && temp_subframe == 1) || (temp_hop != 1 && temp_subframe == 0)) continue;

		//输出不同干扰情况下的信干噪比
			outputpower(source_node_id);

	    //对当前结点的所有link_event进行遍历传输
			vector<route_udp_link_event*>::iterator it;
			for (it = source_node.sending_link_event.begin(); it != source_node.sending_link_event.end(); it++) {
				//事件传输
				(*it)->transimit();
			}
	}

	//对所有link_event进行第三遍遍历，对已经传输完毕的事件进行操作。目的1：统计事件传输成功还是丢失。目的2：维护干扰列表。目的3：销毁link_event，传递route_event
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];
		if (source_node.sending_link_event.size() == 0) continue;
		//if (source_node_id < s_car_num) continue;
		//对当前结点的所有link_event进行遍历维护
		vector<route_udp_link_event*>::iterator it;

		vector<int> pattern_idx = source_node.sending_link_event[0]->get_pattern_idx();
		double max_distance = ((global_control_config*)__context->get_bean("global_control_config"))->get_max_distance();

		bool all_link_event_finished = false;//用于判断所有link_event是否传输完毕，以删除所有link_event
		for (it = source_node.sending_link_event.begin(); it != source_node.sending_link_event.end(); it++) {

			if ((*it)->is_finished()) {
				all_link_event_finished = true;

				route_udp_node& destination_node = get_node_array()[(*it)->get_destination_node_id()];
				int destination_node_id = destination_node.get_id();
				int origin_node_id = -1;
			
				origin_node_id = source_node.m_send_event_queue.front()->get_origin_source_node_id();
				//有效载干比			
				//s_logger_sinrefficient << (*it)->get_sinrefficient()<<" "<<vue_physics::get_LOS(origin_node_id, destination_node_id) << " " << vue_physics::get_distance(origin_node_id, destination_node_id) <<" "<< (int)!(*it)->get_is_loss() << endl;
				//判断是否丢包
				if ((*it)->get_is_loss()) {//如果接收失败
					//if (algorithm_id == 5 && source_node_id < s_car_num) {}
					//if (algorithm_id == 6 && !(source_node_id >= s_car_num&&destination_node_id<s_car_num)){}
					//else{
						if (source_node.m_send_event_queue.front()->m_hop != 1) {
							s_logger_forward_failed << vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						if (source_node_id < s_car_num&&destination_node_id < s_car_num) {
							s_logger_V2V_fail << vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						else if(source_node_id < s_car_num&&destination_node_id >= s_car_num){
							s_logger_V2P_fail<< vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						else if(source_node_id >= s_car_num&&destination_node_id < s_car_num){
							s_logger_P2V_fail << vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						int send_event_id = -1;
						send_event_id = source_node.m_send_event_queue.front()->get_event_id();
						if (vue_physics::get_distance(origin_node_id, destination_node_id) < max_distance) {
							map<int, double>::iterator marked = destination_node.failed_route_event.find(send_event_id);
							map<int, double>::iterator _marked = destination_node.success_route_event.find(send_event_id);
							if (marked == destination_node.failed_route_event.end() && _marked == destination_node.success_route_event.end()) {//如果该事件没有被接收，则加入标记
								destination_node.failed_route_event[send_event_id] = vue_physics::get_distance(origin_node_id, destination_node_id);//标记该接收节点已经收到过此事件，避免重复接收
								//if (destination_node.s_node_type == VUE) {
									m_failed_route_event_num++;
								//}
							}
						}
					//}
				}
				else {
					//如果成功接收
					//if (algorithm_id == 5 && source_node_id < s_car_num) {}
				    //if (algorithm_id == 6 && !(source_node_id >= s_car_num&&destination_node_id<s_car_num)) {}
					//else{
						map<int, double>::iterator marked;

						if (source_node.m_send_event_queue.front()->m_hop != 1) {
							s_logger_forward_success << vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						if (source_node_id < s_car_num&&destination_node_id < s_car_num) {
							s_logger_V2V_success << vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						else if (source_node_id < s_car_num&&destination_node_id >= s_car_num) {
							s_logger_V2P_success << vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						else if (source_node_id >= s_car_num&&destination_node_id < s_car_num) {
							s_logger_P2V_success << vue_physics::get_distance(source_node_id, destination_node_id) << " ";
						}
						marked = destination_node.success_route_event.find(source_node.m_send_event_queue.front()->get_event_id());
						if (marked == destination_node.success_route_event.end()) {//如果该事件没有被接收，则加入标记
							int route_event_id_1 = -1;
							route_event_id_1 = source_node.m_send_event_queue.front()->get_event_id();
							destination_node.success_route_event[route_event_id_1] = vue_physics::get_distance(origin_node_id, destination_node_id);//标记该接收节点已经收到过此事件，避免重复接收
						//	if (destination_node.s_node_type == VUE) 
							{
								m_success_route_event_num++;
								int origin_source_node_id_1 = -1;
								int send_event_start_tti = -1;
								origin_source_node_id_1 = source_node.m_send_event_queue.front()->get_origin_source_node_id();
								send_event_start_tti = source_node.m_send_event_queue.front()->get_start_tti();

								if (origin_source_node_id_1 == source_node_id)
								{
									m_success_first_route_event_num++;
								}
								else
								{
									m_success_second_route_event_num++;
								}
								s_logger_delay << get_time()->get_tti() - send_event_start_tti << " ";
							}
					   }
				    //}
				}
			}
		}

		//所有完成的link_event处理完毕后
		if (all_link_event_finished == true) {
			//删除所有link_event
			vector<route_udp_link_event*>::iterator it = source_node.sending_link_event.begin();
			while (it != source_node.sending_link_event.end()) {
				delete *it;
				it++;
			}
			source_node.sending_link_event.clear();

			//把发送节点从干扰列表里删除
			for (auto p : pattern_idx) {
				if (route_udp_node::s_node_id_per_pattern[p].find(source_node_id) == route_udp_node::s_node_id_per_pattern[p].end()) throw logic_error("error");
				if (p < 0 || p >= pattern_num) throw logic_error("error");
				route_udp_node::s_node_id_per_pattern[p].erase(source_node_id);
			}

			//删除route_event
		
			route_udp_route_event* temp = source_node.m_send_event_queue.front();
			source_node.m_send_event_queue.pop();
			delete temp;
						
		}
	}
}

//---------------简单RSU转发广播函数-----------------//
vector<int> route_udp::select_rsu(int vueid) {//根据SINR选择RSU
	map<double, int> sinr_rsuid;
	for (int rsuid = 0; rsuid < s_rsu_num; rsuid++) {
		sinr_rsuid[vue_physics::get_pl(s_car_num + rsuid, vueid)] = rsuid;
	}

	vector<int> rsuid_selected;
	context* __context = context::get_context();
	int select_rsu_num = ((global_control_config*)__context->get_bean("global_control_config"))->get_rsu_num();

	if (select_rsu_num > s_rsu_num) throw logic_error("select_rsu_num>s_rsu_num");

	map<double, int>::iterator it = sinr_rsuid.end();
	it--;
	for (int rsuid = 0; rsuid < select_rsu_num; rsuid++) {
		rsuid_selected.push_back((*it).second + s_car_num);
		it--;
	}
	return rsuid_selected;
}


//---------------过去预测函数-----------------//
void route_udp::calculate_power() {
	context*__context = context::get_context();
	int pattern_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_pattern_num();
	int packet_pattern_number = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();

	double noise_power = pow(10, -17.4);
	int subcarrier_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12;
	double total_send_power = ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_send_power();
	double send_power = pow(10, (total_send_power - 10 * log10(subcarrier_num * 15 * 1000)) / 10);//......................................

	//计算每个节点当前时刻每个block上的累计接收功率
	for (int node_id = 0; node_id < route_udp_node::s_node_count; node_id++) {
		vector<double> power_pattern(pattern_num, noise_power);
		vector<double> power_patterns(pattern_num + 1 - packet_pattern_number, 0);

		//计算每个pattern上的累计接收功率包括噪声和干扰<Warn:>此处使用的是每个Hz上的接收功率，因为同时乘上相同的带宽对比较大小没有影响
		for (int pattern_idx = 0; pattern_idx < pattern_num; pattern_idx++) {
			set<int>::iterator it = route_udp_node::s_node_id_per_pattern[pattern_idx].begin();
			while (it != route_udp_node::s_node_id_per_pattern[pattern_idx].end()) {
				int inter_vue_id = *it;
				power_pattern[pattern_idx] += vue_physics::get_pl(node_id, inter_vue_id)*send_power;
				it++;
			}
		}

		//计算pattern组合上的累计功率
		for (int i = 0; i < pattern_num + 1 - packet_pattern_number; i++) {
			for (int j = 0; j < packet_pattern_number; j++) {
				power_patterns[i] += power_pattern[i + j];
			}
		}

		//把当前时刻的每个block上的累计功率存入节点，以便以后挑选资源块使用
		get_node_array()[node_id].power_block.push(power_patterns);

		//记录长度根据节点发送数据包周期决定
		if (get_node_array()[node_id].power_block.size() > 100) {
		//if (get_node_array()[node_id].power_block.size() > send_interval) {
			get_node_array()[node_id].power_block.pop();
		}
	}
}

typedef pair<double, pair<int,int>> P;
bool cmp(const P &a, const P&b)
{
	if (a.first < b.first)
		return true;
	else return false;
}

//资源预留选择算法
pair<int,int> route_udp::select_resource(int node_id) {
	context*__context = context::get_context();
	int pattern_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_pattern_num();
	int packet_pattern_number = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();

	int candidate_num = 17 * (pattern_num + 1 - packet_pattern_number)*0.2;
	vector<pair<double, pair<int, int>>> total_resource;

	queue<vector<double>> temp = get_node_array()[node_id].power_block;
	for (int tti = 0; tti <= 20; tti++) {
		if (tti >= 4) {
			for (int block_id = 0; block_id < pattern_num + 1 - packet_pattern_number; block_id++) {
				pair<int, int> resource = make_pair(tti, block_id);
				pair<double, pair<int, int>> block = make_pair(temp.front()[block_id], resource);
				total_resource.push_back(block);
			}
		}
		temp.pop();
	}

	sort(total_resource.begin(), total_resource.end(),cmp);

	uniform_int_distribution<int> u(0, static_cast<int>(candidate_num)-1);

	//pair<int, int > temppp = total_resource[u(s_engine)].second;

	return total_resource[u(s_engine)].second;
}

//---------------输出信干噪比函数-----------------//
void route_udp::outputpower(int source_node_id) {
	//基本参数
	double m_ploss;
	int subcarrier_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12 * ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();
	
	double total_send_power = ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_send_power();
	double m_pt = pow(10, (total_send_power - 10 * log10(subcarrier_num * 15 * 1000)) / 10);
	double m_sigma = pow(10, -17.4);

	route_udp_node& source_node = get_node_array()[source_node_id];
	int source_hop = -1;
	source_hop = source_node.m_send_event_queue.front()->m_hop;
	
	vector<int> pattern_idx = source_node.sending_link_event[0]->get_pattern_idx();

	vector<set<int>> temp = route_udp_node::s_node_id_per_pattern;
	//分开储存干扰源类型：源节点或是转发节点
	vector<vector<int>> m_inter_node_source(5);
	for (int sub_pattern : pattern_idx) {
		for (int inter_node_id : route_udp_node::get_node_id_set(sub_pattern)) {
			route_udp_node& inter_node = get_node_array()[inter_node_id];
			int hop_1 = -1;
			hop_1 = inter_node.m_send_event_queue.front()->m_hop;

			if (hop_1 == 1 && inter_node_id != source_node_id) {
				m_inter_node_source[sub_pattern].push_back(inter_node_id);
			}
		}
	}

	vector<vector<int>> m_inter_node_forward(5);
	for (int sub_pattern : pattern_idx) {
		for (int inter_node_id : route_udp_node::get_node_id_set(sub_pattern)) {
			route_udp_node& inter_node = get_node_array()[inter_node_id];
			int hop_2 = -1;
			hop_2 = inter_node.m_send_event_queue.front()->m_hop;
			if (hop_2 != 1 && inter_node_id != source_node_id) {
				m_inter_node_forward[sub_pattern].push_back(inter_node_id);
			}
		}
	}

	//计算不同干扰节点类型在接收端的功率
	vector<route_udp_link_event*>::iterator it;
	for (it = source_node.sending_link_event.begin(); it != source_node.sending_link_event.end(); it++) {
		vector<vector<double>> m_inter_ploss_source(5);
		vector<vector<double>> m_inter_ploss_forward(5);

		int destination_node_id = (*it)->get_destination_node_id();

		m_ploss = vue_physics::get_pl(source_node_id, destination_node_id);

		//给不同干扰源的路径损耗赋值
		for (int sub_pattern : pattern_idx) {
			for (int inter_node_id : m_inter_node_source[sub_pattern]) {
				m_inter_ploss_source[sub_pattern].push_back(vue_physics::get_pl(destination_node_id, inter_node_id));
			}
		}

		for (int sub_pattern : pattern_idx) {
			for (int inter_node_id : m_inter_node_forward[sub_pattern]) {
				m_inter_ploss_forward[sub_pattern].push_back(vue_physics::get_pl(destination_node_id, inter_node_id));
			}
		}

		//计算不同干扰源的信干噪比
		vector<double> sinr_source(subcarrier_num);//每个子载波上的信干噪比，维度为nt的向量
		for (int subcarrier_idx = 0; subcarrier_idx < subcarrier_num; subcarrier_idx++) {
			double molecule = m_pt*m_ploss;
			double h_sum2 = 0;
			int pattern_index = subcarrier_idx / (((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12);
			for (int j = 0; j < m_inter_ploss_source[pattern_idx[pattern_index]].size(); j++) {
				double weight = m_pt*m_inter_ploss_source[pattern_idx[pattern_index]][j];
				h_sum2 += weight;
			}
			double denominator = m_sigma + h_sum2;

			sinr_source[subcarrier_idx] = 10 * log10(molecule / denominator);

			//输出信干比
			if (subcarrier_idx % (((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12) == 0 && h_sum2 != 0) {
				double sir = 10 * log10(molecule / h_sum2);
				//if (vue_physics::get_distance(source_node_id, destination_node_id) < 500&& vue_physics::get_distance(source_node_id, destination_node_id)>=250) {
					if (source_hop == 1) {
						double dis = vue_physics::get_distance(source_node_id, destination_node_id);
						s_logger_inter_s_s << sir << " "<<dis<<endl;
					}
					else {
						s_logger_inter_f_s << sir << " ";
					}
				//}
			}
		}

		vector<double> sinr_forward(subcarrier_num);//每个子载波上的信干噪比，维度为nt的向量
		for (int subcarrier_idx = 0; subcarrier_idx < subcarrier_num; subcarrier_idx++) {
			double molecule = m_pt*m_ploss;
			double h_sum2 = 0;
			int pattern_index = subcarrier_idx / (((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12);
			for (int j = 0; j < m_inter_ploss_forward[pattern_idx[pattern_index]].size(); j++) {
				double weight = m_pt*m_inter_ploss_forward[pattern_idx[pattern_index]][j];
				h_sum2 += weight;
			}
			double denominator = m_sigma + h_sum2;

			sinr_forward[subcarrier_idx] = 10 * log10(molecule / denominator);

			//输出信干比
			if (subcarrier_idx % (((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12) == 0 && h_sum2 != 0) {
				double sir = 10 * log10(molecule / h_sum2);
				if (vue_physics::get_distance(source_node_id, destination_node_id) < 500&& vue_physics::get_distance(source_node_id, destination_node_id)>=250) {
					if (source_hop == 1) {
						s_logger_inter_s_f << sir << " ";
					}
					else {
						s_logger_inter_f_f << sir << " ";
					}
				}
			}
		}
	}
}
