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

//��̬��Ա��ʼ��
int route_udp_route_event::s_event_count = 0;

default_random_engine route_udp::s_engine;

int route_udp_node::s_node_count = 0;//�ڵ����������˼�������

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
ofstream route_udp::s_logger_V2V_success;//��ӡV2V�ɹ��¼�����
ofstream route_udp::s_logger_V2P_success;//��ӡV2P�ɹ��¼�����
ofstream route_udp::s_logger_P2V_success;//��ӡP2V�ɹ��¼�����
ofstream route_udp::s_logger_V2V_fail;//��ӡV2Vʧ���¼�����
ofstream route_udp::s_logger_V2P_fail;//��ӡV2Pʧ���¼�����
ofstream route_udp::s_logger_P2V_fail;//��ӡP2Vʧ���¼�����

									  //bler=0.1��snrֵ
double g_MCSBound_uplink[29] = { 0.211896,0.331131,0.394198,0.468218,0.572769,0.719025,0.849572,1.081477,1.305817,1.597595,2.097526,2.217732,2.540644,3.235289,3.798928,4.725782,5.572635,6.703082,7.413102,8.841897,10.606376,13.617207,16.149782,19.221597,23.794819,28.323521,32.899899,38.591861,71.395508 };
//snr��bler���ߵĲ���
double g_msc_snr_bler_uplink_b[29] = { 0.162398,0.275259,0.331273,0.387216,0.500396,0.632055,0.757767,0.976645,1.181258,1.450097,1.891839,2.021082,2.310004,2.959804,3.490522,4.358722,5.116073,6.160274,6.791948,8.179674,9.797276,12.584850,14.960581,17.775798,22.079626,26.184039,30.701693,35.817768,66.378959 };
double g_mcs_snr_bler_uplink_c[29] = { 0.054680,0.057094,0.070231,0.088679,0.080189,0.090378,0.097123,0.112113,0.130679,0.148960,0.207821,0.208571,0.244529,0.283471,0.329564,0.370230,0.471146,0.537504,0.609931,0.696236,0.804291,1.015040,1.199394,1.402330,1.786923,2.067856,2.337520,2.848863,4.948657 };
//����������  
double LUT_SNR2BLER(double snr, int mcs_level)
{
	double bmcs = g_msc_snr_bler_uplink_b[mcs_level];
	double cmcs = g_mcs_snr_bler_uplink_c[mcs_level];
	return 0.5f*(1.0f - (float)erf(((snr - bmcs) / cmcs)));
}

//����link_event����һ��TTI
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
//--------------�����ͬ�����ڽ��ն˵��Ÿɱ�---------------//
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

//����ÿ��Ƶ��������ռ�õĳ���ID
const std::set<int>& route_udp_node::get_node_id_set(int t_pattern_idx) {
	return s_node_id_per_pattern[t_pattern_idx];
}


//node��ʼ��
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

	m_broadcast_time = u_start_broadcast_tti(s_engine);//��ʼ����һ�η���������Ϣ��ʱ�䣬���ڴ�����
	m_send_time = u_start_send_tti(s_engine);//��ʼ����һ�ο��Է������ݰ���ʱ�䣬���ڴ�����

	counter = u_counter(s_engine);
	node_resource = make_pair(u_resource_time(s_engine), u_resource_pattern(s_engine));
	m_broadcast_real_time = node_resource.first + m_broadcast_time; //��ʵ�Ĺ㲥�����ʱ��
}


//udp��ʼ��
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
	interval = ((route_config*)__context->get_bean("route_config"))->get_t_interval();//�����¼��������
	send_interval = ((rrm_config*)__context->get_bean("rrm_config"))->get_send_interval();
}

//ÿ��TTI������
void route_udp::process_per_tti() {
	//�¼�����
	event_trigger();

    //����Ҫ��ʼ���͵��¼�
	start_sending_data();

	//���䵱ǰTTI���ڵ��¼�
	transmit_data();

	//�����ǰʱ�̸����ڵ�Ķ��г���
	for (int i = 0; i < route_udp_node::s_node_count; i++) {
		int temp = get_node_array()[i].m_send_event_queue.size();
		s_logger_queue_length << temp << " ";
	}
	s_logger_queue_length << endl;
}

//�¼�����
void route_udp::event_trigger() {
	context* __context = context::get_context();

	if (get_time()->get_tti() < ((global_control_config*)__context->get_bean("global_control_config"))->get_ntti()) {//����ʱ���ڴ���

		//�ڳ�ʼ��ʱ����󣬴������ݴ����¼�,���ݹ㲥�㷨�Ĳ�ͬ�¼�������ʽҲ��ͬ
		for (int origin_source_node_id = 0; origin_source_node_id < route_udp_node::s_node_count; origin_source_node_id++) {
			route_udp_node& source_node = get_node_array()[origin_source_node_id];

			//����ÿ�������Ĵ�������
			uniform_int_distribution<int> u_send_chance(0, 100);

			if (get_time()->get_tti() % interval == source_node.m_broadcast_time&&u_send_chance(s_engine) <= ((tmc_config*)__context->get_bean("tmc_config"))->get_trigger_rate()*100) {//���㴥�����ʲŴ���				
				//V2V�㲥
				if (algorithm_id == 1) {
					if (source_node.s_node_type == PUE) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2P�㲥
				if (algorithm_id == 2) {

					if (source_node.s_node_type == PUE) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//P2V�㲥
				if (algorithm_id == 3) 
				{
					if (source_node.s_node_type == VUE) continue;
					//if (get_time()->get_tti() % (interval * 10) != source_node.m_broadcast_time) continue;//p2v���ڿ���
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1));//��ǰ�ڵ㵱ǰʱ�̴�����Դ�㲥�¼���
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;//��Ǹý��սڵ��Ѿ��յ������¼��������ظ�����
					m_event_num++;
				}
				//V2V+V2P�㲥
				if (algorithm_id == 4) {
					if (source_node.s_node_type == PUE) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2V+P2V�㲥
				if (algorithm_id == 5) {
					if (source_node.s_node_type == PUE&&get_time()->get_tti() % (interval*10) != source_node.m_broadcast_time) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2P+P2V�㲥
				if (algorithm_id == 6) {
					if (source_node.s_node_type == PUE&&get_time()->get_tti() % (interval * 10) != source_node.m_broadcast_time) continue;
					source_node.offer_send_event_queue(
						new route_udp_route_event(origin_source_node_id, -1, get_time()->get_tti(), route_udp_route_event::s_event_count++, 1)
					);
					source_node.success_route_event[route_udp_route_event::s_event_count - 1] = 0;
					m_event_num++;
				}
				//V2V+V2P+P2V�㲥
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

//��ʼ����route_event
void route_udp::start_sending_data()                                                                                                                                                                                                                                                                         {
	context* __context = context::get_context();

	//����ͬһʱ�̷���Ϣ����������Ϣ��ԭ�����з���Ϣ���¼��ڴ���ǰ��ѡ����Ƶ�β�����ռ��
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];

		if (source_node.is_send_event_queue_empty()) continue;//��ǰ�����������¼��б�Ϊ�գ���������
		
		if (source_node.sending_link_event.size() == 0) {//��ǰ�ڵ���һ���¼��Ѿ���ɴ������û��Ҫ������¼�

			if (!source_node.is_send_event_queue_empty()) 
			{
				//�ж����ݰ�����ʱ���Ƿ񳬹����˵���ʱ�ӣ����������ֱ�Ӷ������ݰ�
				while (get_time()->get_tti() - source_node.m_send_event_queue.front()->get_start_tti() > ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_max_delay()) {
					source_node.m_send_event_queue.pop();
					m_event_pop_num++;
					if (source_node.is_send_event_queue_empty()) break;
				}
			}
			//�������Ϊ�������������һ���ڵ�		
			if (source_node.is_send_event_queue_empty()) continue;
		
			int select_patternid;//������ѡ����Ƶ����ţ���������
			vector<int> select_pattern;//������ѡ�������ڴ����Ƶ�Σ������Ƕ��pattern		
			vector<int> temp_broadcast_set;
			vector<int> temp_forward_tti;
			vector<int> temp_forward_pattern;
			vector<int>::iterator it11;

			
			if (enhance_select_algorithm == 0) {//�����ʹ�øĽ�������Դ�����㷨
				int pattern_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_pattern_num();
				int packet_pattern_number = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();
				uniform_int_distribution<int> u_pattern(0, (pattern_num + 1 - packet_pattern_number) - 1);
				select_patternid = u_pattern(s_engine);
				//pair<int, int> select_res = source_node.select_relay_information();
				//select_patternid = select_res.second;
			}
			else if (enhance_select_algorithm == 1) {//������ù�ȥʱ��Ԥ�����Դѡ���㷨���ж��Ƿ񵽴﷢���¼�
					
					if (source_node.m_send_event_queue.front()->selected == false) {
							//������ø��ݹ�ȥʱ��Ԥ�����Դѡ���㷨
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
						//�ж��Ƿ񵽴﷢��ʱ�䣬û�����������һ���ڵ㣬�������������·
					if (get_time()->get_tti() != source_node.m_send_event_queue.front()->m_resource.first)//����ʱ��洢��m_resource��
						continue;
					else 
						source_node.counter--;
					select_patternid = source_node.m_send_event_queue.front()->m_resource.second;	
				}
				
			//������pattern��ϱ�Ϊ����pattern
			for (int i = select_patternid; i <= select_patternid + packet_pattern_number - 1; i++) {
				select_pattern.push_back(i);
			}


			//���ݹ㲥�㷨ѡ����սڵ�,������·�¼����й㲥
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

				//���սڵ�ֻ���Դ�ڵ�һ����Χ�ڵĽڵ�
				if (dst_id == source_node_id || vue_physics::get_distance(source_node.m_send_event_queue.front()->get_origin_source_node_id(), dst_id) >= ((global_control_config*)__context->get_bean("global_control_config"))->get_max_distance()) continue;
				
				map<int, double>::iterator marked = get_node_array()[dst_id].success_route_event.find(source_node.m_send_event_queue.front()->get_event_id());
				source_node.sending_link_event.push_back(new route_udp_link_event(
					source_node_id, dst_id, select_pattern, source_node.peek_send_event_queue()->get_tti_num()));
			}
			source_node.m_send_event_queue.front()->set_broadcast_flag(true);
			if (source_node.sending_link_event.size() == 0) {
				throw logic_error("error");//����㲥���սڵ������ɽ������ٽ����㲥����
			}
			else {
				m_broadcast_num++;
			}				
		}
	}
}

//ÿ��TTI����link_event
void route_udp::transmit_data() {
	context *__context = context::get_context();
	
	//��һ��������ж�ÿ���ڵ��link_event�Ƿ�Ҫ�ڵ�ǰTTI���䣬�����Ҫ���������б�
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];

		if (source_node.sending_link_event.size() == 0) continue;
		m_send_broadcast_num++;
		
		//�����ǰʱ�̸ýڵ��link_event��Ҫ����
		//int temp_subframe = get_time()->get_tti() % 2;
		//vector<int> temp_forward_node = source_node.m_send_event_queue.front()->m_broadcast_set;
		//vector<int>::iterator it33 = find(temp_forward_node.begin(), temp_forward_node.end(), source_node_id);
		//if (it33 != temp_forward_node.end()) {
		//	int index = it33 - temp_forward_node.begin();
		//	if (temp_subframe != source_node.m_send_event_queue.front()->m_forward_subframe[index]) continue;
		//}
		//int temp_hop = source_node.m_send_event_queue.front()->m_hop;
		//if ((temp_hop == 1 && temp_subframe == 1) || (temp_hop != 1 && temp_subframe == 0)) continue;
		
		
		for (auto p : source_node.sending_link_event[0]->get_pattern_idx()) {//��������б�
			if (route_udp_node::s_node_id_per_pattern[p].find(source_node_id) != route_udp_node::s_node_id_per_pattern[p].end()) 
				throw logic_error("error");
			route_udp_node::s_node_id_per_pattern[p].insert(source_node_id);
		}
	}
	//-----------Ԥ�ⷽʽʹ��Ƶ�κ���-----------//
	if (enhance_select_algorithm == 1) {
		//����ÿ��block�ϵ������ܺ�
		calculate_power();
	}

	//������link_event���еڶ����������������link_event
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];
		if (source_node.sending_link_event.size() == 0) continue;
		//if (source_node_id < s_car_num) continue;

		//�����ǰʱ�̸ýڵ��link_event��Ҫ����
		//int temp_subframe = get_time()->get_tti() % 2;
		/*vector<int> temp_forward_node = source_node.m_send_event_queue.front()->m_broadcast_set;
		vector<int>::iterator it33 = find(temp_forward_node.begin(), temp_forward_node.end(), source_node_id);
		if (it33 != temp_forward_node.end()) {
			int index = it33 - temp_forward_node.begin();
			if (temp_subframe != source_node.m_send_event_queue.front()->m_forward_subframe[index]) continue;
		}*/
		//int temp_hop = source_node.m_send_event_queue.front()->m_hop;
		//if ((temp_hop == 1 && temp_subframe == 1) || (temp_hop != 1 && temp_subframe == 0)) continue;

		//�����ͬ��������µ��Ÿ����
			outputpower(source_node_id);

	    //�Ե�ǰ��������link_event���б�������
			vector<route_udp_link_event*>::iterator it;
			for (it = source_node.sending_link_event.begin(); it != source_node.sending_link_event.end(); it++) {
				//�¼�����
				(*it)->transimit();
			}
	}

	//������link_event���е�������������Ѿ�������ϵ��¼����в�����Ŀ��1��ͳ���¼�����ɹ����Ƕ�ʧ��Ŀ��2��ά�������б�Ŀ��3������link_event������route_event
	for (int source_node_id = 0; source_node_id < route_udp_node::s_node_count; source_node_id++) {
		route_udp_node& source_node = get_node_array()[source_node_id];
		if (source_node.sending_link_event.size() == 0) continue;
		//if (source_node_id < s_car_num) continue;
		//�Ե�ǰ��������link_event���б���ά��
		vector<route_udp_link_event*>::iterator it;

		vector<int> pattern_idx = source_node.sending_link_event[0]->get_pattern_idx();
		double max_distance = ((global_control_config*)__context->get_bean("global_control_config"))->get_max_distance();

		bool all_link_event_finished = false;//�����ж�����link_event�Ƿ�����ϣ���ɾ������link_event
		for (it = source_node.sending_link_event.begin(); it != source_node.sending_link_event.end(); it++) {

			if ((*it)->is_finished()) {
				all_link_event_finished = true;

				route_udp_node& destination_node = get_node_array()[(*it)->get_destination_node_id()];
				int destination_node_id = destination_node.get_id();
				int origin_node_id = -1;
			
				origin_node_id = source_node.m_send_event_queue.front()->get_origin_source_node_id();
				//��Ч�ظɱ�			
				//s_logger_sinrefficient << (*it)->get_sinrefficient()<<" "<<vue_physics::get_LOS(origin_node_id, destination_node_id) << " " << vue_physics::get_distance(origin_node_id, destination_node_id) <<" "<< (int)!(*it)->get_is_loss() << endl;
				//�ж��Ƿ񶪰�
				if ((*it)->get_is_loss()) {//�������ʧ��
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
							if (marked == destination_node.failed_route_event.end() && _marked == destination_node.success_route_event.end()) {//������¼�û�б����գ��������
								destination_node.failed_route_event[send_event_id] = vue_physics::get_distance(origin_node_id, destination_node_id);//��Ǹý��սڵ��Ѿ��յ������¼��������ظ�����
								//if (destination_node.s_node_type == VUE) {
									m_failed_route_event_num++;
								//}
							}
						}
					//}
				}
				else {
					//����ɹ�����
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
						if (marked == destination_node.success_route_event.end()) {//������¼�û�б����գ��������
							int route_event_id_1 = -1;
							route_event_id_1 = source_node.m_send_event_queue.front()->get_event_id();
							destination_node.success_route_event[route_event_id_1] = vue_physics::get_distance(origin_node_id, destination_node_id);//��Ǹý��սڵ��Ѿ��յ������¼��������ظ�����
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

		//������ɵ�link_event������Ϻ�
		if (all_link_event_finished == true) {
			//ɾ������link_event
			vector<route_udp_link_event*>::iterator it = source_node.sending_link_event.begin();
			while (it != source_node.sending_link_event.end()) {
				delete *it;
				it++;
			}
			source_node.sending_link_event.clear();

			//�ѷ��ͽڵ�Ӹ����б���ɾ��
			for (auto p : pattern_idx) {
				if (route_udp_node::s_node_id_per_pattern[p].find(source_node_id) == route_udp_node::s_node_id_per_pattern[p].end()) throw logic_error("error");
				if (p < 0 || p >= pattern_num) throw logic_error("error");
				route_udp_node::s_node_id_per_pattern[p].erase(source_node_id);
			}

			//ɾ��route_event
		
			route_udp_route_event* temp = source_node.m_send_event_queue.front();
			source_node.m_send_event_queue.pop();
			delete temp;
						
		}
	}
}

//---------------��RSUת���㲥����-----------------//
vector<int> route_udp::select_rsu(int vueid) {//����SINRѡ��RSU
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


//---------------��ȥԤ�⺯��-----------------//
void route_udp::calculate_power() {
	context*__context = context::get_context();
	int pattern_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_pattern_num();
	int packet_pattern_number = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();

	double noise_power = pow(10, -17.4);
	int subcarrier_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12;
	double total_send_power = ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_send_power();
	double send_power = pow(10, (total_send_power - 10 * log10(subcarrier_num * 15 * 1000)) / 10);//......................................

	//����ÿ���ڵ㵱ǰʱ��ÿ��block�ϵ��ۼƽ��չ���
	for (int node_id = 0; node_id < route_udp_node::s_node_count; node_id++) {
		vector<double> power_pattern(pattern_num, noise_power);
		vector<double> power_patterns(pattern_num + 1 - packet_pattern_number, 0);

		//����ÿ��pattern�ϵ��ۼƽ��չ��ʰ��������͸���<Warn:>�˴�ʹ�õ���ÿ��Hz�ϵĽ��չ��ʣ���Ϊͬʱ������ͬ�Ĵ���ԱȽϴ�Сû��Ӱ��
		for (int pattern_idx = 0; pattern_idx < pattern_num; pattern_idx++) {
			set<int>::iterator it = route_udp_node::s_node_id_per_pattern[pattern_idx].begin();
			while (it != route_udp_node::s_node_id_per_pattern[pattern_idx].end()) {
				int inter_vue_id = *it;
				power_pattern[pattern_idx] += vue_physics::get_pl(node_id, inter_vue_id)*send_power;
				it++;
			}
		}

		//����pattern����ϵ��ۼƹ���
		for (int i = 0; i < pattern_num + 1 - packet_pattern_number; i++) {
			for (int j = 0; j < packet_pattern_number; j++) {
				power_patterns[i] += power_pattern[i + j];
			}
		}

		//�ѵ�ǰʱ�̵�ÿ��block�ϵ��ۼƹ��ʴ���ڵ㣬�Ա��Ժ���ѡ��Դ��ʹ��
		get_node_array()[node_id].power_block.push(power_patterns);

		//��¼���ȸ��ݽڵ㷢�����ݰ����ھ���
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

//��ԴԤ��ѡ���㷨
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

//---------------����Ÿ���Ⱥ���-----------------//
void route_udp::outputpower(int source_node_id) {
	//��������
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
	//�ֿ��������Դ���ͣ�Դ�ڵ����ת���ڵ�
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

	//���㲻ͬ���Žڵ������ڽ��ն˵Ĺ���
	vector<route_udp_link_event*>::iterator it;
	for (it = source_node.sending_link_event.begin(); it != source_node.sending_link_event.end(); it++) {
		vector<vector<double>> m_inter_ploss_source(5);
		vector<vector<double>> m_inter_ploss_forward(5);

		int destination_node_id = (*it)->get_destination_node_id();

		m_ploss = vue_physics::get_pl(source_node_id, destination_node_id);

		//����ͬ����Դ��·����ĸ�ֵ
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

		//���㲻ͬ����Դ���Ÿ����
		vector<double> sinr_source(subcarrier_num);//ÿ�����ز��ϵ��Ÿ���ȣ�ά��Ϊnt������
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

			//����Ÿɱ�
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

		vector<double> sinr_forward(subcarrier_num);//ÿ�����ز��ϵ��Ÿ���ȣ�ά��Ϊnt������
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

			//����Ÿɱ�
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
