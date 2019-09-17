/*
* =====================================================================================
*
*       Filename:  tmc.cpp
*
*    Description:  业务模型与控制类实现
*
*        Version:  1.0
*        Created:
*       Revision:
*       Compiler:  VS_2015
*
*         Author:  HCF
*            LIB:  ITTC
*
* =====================================================================================
*/

#include<random>
#include<fstream>
#include"gtt.h"
#include"tmc.h"
#include"vue.h"
#include"vue_physics.h"
#include"config.h"
#include"route_udp.h"
#include"reflect/context.h"


using namespace std;

void tmc::statistic() {
	context* __context = context::get_context();

	ofstream output;
	ofstream s_logger_failed_distance;
	ofstream s_logger_success_distance;
	

	output.open("log/output.txt");
	s_logger_failed_distance.open("log/failed_distance.txt");
	s_logger_success_distance.open("log/success_distance.txt");

	object* __object = context::get_context()->get_bean("route");

	route_udp* __route_udp = (route_udp*)__object;
	int success_num = __route_udp->get_success_route_event_num();
	int failed_num = __route_udp->get_failed_route_event_num();
	int total_num = success_num + failed_num;
	double pdr = double(success_num) / double(total_num)*double(100);

	output << "vuenum:" << __route_udp->s_car_num << endl;
	output << "rsunum:" << __route_udp->s_rsu_num << endl;
	output << "total success event: " << success_num << endl;
	output << "total first success event: " << __route_udp->get_first_success_route_event_num() << endl;
	output << "total second success event: " << __route_udp->get_second_success_route_event_num() << endl;
	output << "total failed event: " << failed_num << endl;
	output << "total event number:" << total_num << endl;
	output << "pdr:" << pdr << "%" << endl;
	output << "event trigger times:" << __route_udp->get_event_num() << endl;
	output << "total broadcast number:" << __route_udp->get_broadcast_num() << endl;
	output << "total send_broadcast number:" << __route_udp->get_send_broadcast_num() << endl;
	output << "total event num all:" << __route_udp->get_event_num_all() << endl;
	output << "total source_broadcast number:" << __route_udp->get_source_broadcast_num() << endl;
	output << "total forward_broadcast number:" << __route_udp->get_forward_broadcast_num() << endl;
	output << "total pop number:" << __route_udp->get_m_event_pop_num() << endl;
	output << "total second pop number:" << __route_udp->get_m_event_second_pop_num() << endl;
	output << "total delete number:" << __route_udp->get_m_event_delay_100_tti_num() << endl;
	output << "total center number:" << __route_udp->center_number << endl;
	output << "total received center number:" << __route_udp->center_received_number << endl;
	output << "received center ratio:" << double(__route_udp->center_received_number) / double(__route_udp->center_number) << endl;
	output << "total close center number:" << __route_udp->close_center_number << endl;
	output << "total close received center number:" << __route_udp->close_center_received_number << endl;
	output << "received close center ratio:" << double(__route_udp->close_center_received_number) / double(__route_udp->close_center_number) << endl;

	gtt* __gtt = (gtt*)__object;
	
	for (int i = 0; i < vue_physics::get_ue_num(); i++) {
		map<int, double>::iterator failed = __route_udp->get_node_array()[i].failed_route_event.begin();
		while (failed != __route_udp->get_node_array()[i].failed_route_event.end()) {

			s_logger_failed_distance << failed->second << " ";
			failed++;
			double temp = ((gtt*)__context->get_bean("gtt"))->get_ue_array()[i].get_physics_level()->get_absx();
		}
		map<int, double>::iterator success = __route_udp->get_node_array()[i].success_route_event.begin();
		while (success != __route_udp->get_node_array()[i].success_route_event.end()) {
			s_logger_success_distance << success->second << " ";
			success++;
		}
	}
}