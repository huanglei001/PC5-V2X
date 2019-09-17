/*
* =====================================================================================
*
*       Filename:  wt.cpp
*
*    Description:  高速场景类实现
*
*        Version:  1.0
*        Created:
*       Revision:
*       Compiler:  VS_2015
*
*         Author:  LN
*            LIB:  ITTC
*
* =====================================================================================
*/

#include<fstream>
#include<iterator>
#include<vector>
#include"config.h"
#include"wt.h"
#include"vue_physics.h"
#include"reflect/context.h"
#include"imta.h"
#include"gtt_urban.h"
using namespace std;

default_random_engine wt::s_engine(0);

std::vector<double>* wt::m_qpsk_mi = nullptr;


void wt::set_resource() {
	ifstream in;
	in.open("wt/qpsk_mi.md");

	m_qpsk_mi = new vector<double>();
	istream_iterator<double> in_iter(in), eof;
	m_qpsk_mi->assign(in_iter, eof);
	in.close();
}

double wt::calculate_sinr(int t_send_vue_id, int t_receive_vue_id, vector<int> t_pattern_idx, vector<set<int>> t_sending_vue_id_set) {
	m_ploss = vue_physics::get_pl(t_send_vue_id, t_receive_vue_id);
	int subcarrier_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12 * ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();
	double total_send_power = ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_send_power();
	m_pt = pow(10, (total_send_power - 10 * log10(subcarrier_num * 15 * 1000)) / 10);
	m_sigma = pow(10, (-174 + ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_noise_figure()) / 10);
	//m_sigma = pow(10, -17.4);

	vector<vector<double>> m_inter_ploss(5);

	int count = 0;
	for (int i : t_pattern_idx) {
		for (int inter_vue_id : t_sending_vue_id_set[count]) {
			if (t_send_vue_id == inter_vue_id) continue;
			m_inter_ploss[i].push_back(vue_physics::get_pl(t_receive_vue_id, inter_vue_id));
		}
		count++;
	}

	//<Warn:>
	//m_inter_ploss[0].clear();
	//m_inter_ploss[1].clear();
	//m_inter_ploss[2].clear();
	//m_inter_ploss[3].clear();
	//m_inter_ploss[4].clear();
	//求每个子载波上的信噪比<Warn:>实则是计算每个Hz上的信干噪比，因为没有小尺度衰落所以每个子载波上的信干噪比就等于每个Hz上的信干噪比
	vector<double> sinr(subcarrier_num);//每个子载波上的信噪比，维度为nt的向量
	for (int subcarrier_idx = 0; subcarrier_idx <subcarrier_num; subcarrier_idx++) {
		double molecule = m_pt*m_ploss;
		double h_sum2 = 0;
		int pattern_idx = subcarrier_idx / (((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12);
		for (int j = 0; j < m_inter_ploss[t_pattern_idx[pattern_idx]].size(); j++) {
			double weight = m_pt*m_inter_ploss[t_pattern_idx[pattern_idx]][j];
			h_sum2 += weight;
		}
		double denominator = m_sigma + h_sum2;

		sinr[subcarrier_idx] = 10 * log10(molecule / denominator);
	}

	int distance = vue_physics::get_distance(t_send_vue_id, t_receive_vue_id);
	//互信息方法求有效信噪比sinreff

	double sum_mi = 0, ave_mi = 0;
	double sinreff = 0;

	for (int k = 0; k < subcarrier_num; k++) {
		sum_mi = sum_mi + get_mutual_information(*m_qpsk_mi, (int)ceil((sinr[k] + 20) * 2));
	}
	ave_mi = sum_mi / subcarrier_num;

	int snr_index = closest(*m_qpsk_mi, ave_mi);
	sinreff = 0.5*(snr_index - 40);

	//<Warn>当没有小尺度衰落时，各个子载波上的SINR都是一样的，故算术平均为每个子载波上的SINR
	return sinreff;
}

double wt::calculate_sinrsmallscale(int t_send_vue_id, int t_receive_vue_id, vector<int> t_pattern_idx, vector<set<int>> t_sending_vue_id_set) {
	m_ploss = vue_physics::get_pl(t_send_vue_id, t_receive_vue_id);
	int total_bandwidth = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_total_bandwidth()/ 1000000;
	int fft_point_num = pow(2, 10) * total_bandwidth/10;
	//double H0[4096];//目标用户的小尺度信道增益
	double *H0 = new double[fft_point_num * 4];//目标用户的小尺度信道增益
	context* __context = context::get_context();
	((gtt*)__context->get_bean("gtt"))->calculate_smallscale(t_send_vue_id, t_receive_vue_id, H0);
	int subcarrier_num = ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12* ((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_packet_pattern_number();
	double total_send_power = ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_send_power();
	m_pt = pow(10, (total_send_power - 10 * log10(subcarrier_num * 15 * 1000)) / 10);
	m_sigma = pow(10, (-174 + ((global_control_config*)context::get_context()->get_bean("global_control_config"))->get_noise_figure()) / 10);
	//m_sigma = pow(10, -17.4);

	vector<vector<double>> m_inter_ploss(5);
	vector<vector<vector<double>>> m_inter_psmallscale(5,vector<vector<double>>(20));

	int count = 0;
	for (int i : t_pattern_idx) {
		int j=0;
		for (int inter_vue_id : t_sending_vue_id_set[count]) {
			if (t_send_vue_id == inter_vue_id) continue;
			m_inter_ploss[i].push_back(vue_physics::get_pl(t_receive_vue_id, inter_vue_id));
			//double temp[4096];//干扰用户的小尺度信道增益
			double *temp = new double[fft_point_num * 4];//干扰用户的小尺度信道增益
			((gtt*)__context->get_bean("gtt"))->calculate_smallscale(t_receive_vue_id, inter_vue_id, temp);
			/*for (size_t k = 0; k < 4096; k++)*/
			for (size_t k = 0; k < fft_point_num * 4; k++)
			{
				m_inter_psmallscale[i][j].push_back(temp[k]);
			}
			j++;
			delete temp;
			temp = NULL;
		}
		count++;
	}

	//<Warn:>
	//m_inter_ploss[0].clear();
	//m_inter_ploss[1].clear();
	//m_inter_ploss[2].clear();
	//m_inter_ploss[3].clear();
	//m_inter_ploss[4].clear();
	//求每个子载波上的信噪比<Warn:>实则是计算每个Hz上的信干噪比，因为没有小尺度衰落所以每个子载波上的信干噪比就等于每个Hz上的信干噪比
	vector<double> sinr(subcarrier_num);//每个子载波上的信噪比，维度为nt的向量
	for (int subcarrier_idx = 0; subcarrier_idx <subcarrier_num; subcarrier_idx++) {
		/*double H = pow(H0[subcarrier_idx*2], 2.0) + pow(H0[subcarrier_idx*2 + 1], 2.0) + pow(H0[subcarrier_idx * 2+2048], 2.0) + pow(H0[subcarrier_idx * 2 + 1+2048], 2.0);*/
		double H = pow(H0[subcarrier_idx * 2], 2.0) + pow(H0[subcarrier_idx * 2 + 1], 2.0) + pow(H0[subcarrier_idx * 2 + fft_point_num * 2], 2.0) + pow(H0[subcarrier_idx * 2 + 1 + fft_point_num * 2], 2.0);
		double molecule = m_pt*m_ploss*pow(H,2.0);
		double h_sum2 = 0;
		int pattern_idx = subcarrier_idx / (((rrm_config*)context::get_context()->get_bean("rrm_config"))->get_rb_num_per_pattern() * 12);
		for (int j = 0; j < m_inter_ploss[t_pattern_idx[pattern_idx]].size(); j++) {
			/*double refH = pow(H0[subcarrier_idx * 2] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2] + H0[subcarrier_idx * 2 + 1] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1] + H0[subcarrier_idx * 2 + 2048] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 2048] + H0[subcarrier_idx * 2 + 1 + 2048] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1 + 2048], 2.0) + pow(H0[subcarrier_idx * 2] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1] - m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2] * H0[subcarrier_idx * 2 + 1] + H0[subcarrier_idx * 2 + 2048] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1 + 2048] - m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 2048] * H0[subcarrier_idx * 2 + 1 + 2048], 2.0);*/
			double refH = pow(H0[subcarrier_idx * 2] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2] + H0[subcarrier_idx * 2 + 1] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1] + H0[subcarrier_idx * 2 + fft_point_num * 2] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + fft_point_num * 2] + H0[subcarrier_idx * 2 + 1 + fft_point_num * 2] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1 + fft_point_num * 2], 2.0) + pow(H0[subcarrier_idx * 2] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1] - m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2] * H0[subcarrier_idx * 2 + 1] + H0[subcarrier_idx * 2 + fft_point_num * 2] * m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + 1 + fft_point_num * 2] - m_inter_psmallscale[t_pattern_idx[pattern_idx]][j][subcarrier_idx * 2 + fft_point_num * 2] * H0[subcarrier_idx * 2 + 1 + fft_point_num * 2], 2.0);
			double weight = m_pt*m_inter_ploss[t_pattern_idx[pattern_idx]][j]*refH;
			h_sum2 += weight;
		}
		double denominator = m_sigma*H + h_sum2;

		sinr[subcarrier_idx] = 10 * log10(molecule / denominator);
		//cout << sinr[subcarrier_idx] << endl;
	}
	delete H0;
	H0 = NULL;
	int distance = vue_physics::get_distance(t_send_vue_id, t_receive_vue_id);
	//互信息方法求有效信噪比sinreff

	double sum_mi = 0, ave_mi = 0;
	double sinreff = 0;

	for (int k = 0; k < subcarrier_num; k++) {
		sum_mi = sum_mi + get_mutual_information(*m_qpsk_mi, (int)ceil((sinr[k] + 20) * 2));
	}
	ave_mi = sum_mi / subcarrier_num;

	int snr_index = closest(*m_qpsk_mi, ave_mi);
	sinreff = 0.5*(snr_index - 40);

	//<Warn>当没有小尺度衰落时，各个子载波上的SINR都是一样的，故算术平均为每个子载波上的SINR
	return sinreff;
}

int wt::closest(std::vector<double> t_vec, double t_target) {
	int left_index = 0;
	int right_index = static_cast<int>(t_vec.size() - 1);
	double left_diff = t_vec[left_index] - t_target;
	double right_diff = t_vec[right_index] - t_target;

	while (left_index <= right_index) {
		if (right_diff <= 0) return right_index;//???
		if (left_diff >= 0) return left_index;//???

		int mid_index = left_index + ((right_index - left_index) >> 1);
		double mid_diff = t_vec[mid_index] - t_target;
		if (mid_diff == 0) return mid_index;
		else if (mid_diff < 0) {
			left_index = mid_index + 1;
			left_diff = t_vec[left_index] - t_target;
			if (abs(mid_diff) < abs(left_diff)) return mid_index;
		}
		else {
			right_index = mid_index - 1;
			right_diff = t_vec[right_index] - t_target;
			if (abs(mid_diff) < abs(right_diff)) return mid_index;
		}
	}
	return abs(t_vec[left_index] - t_target) < abs(t_vec[left_index - 1] - t_target) ? left_index : left_index - 1;//???

}

double wt::get_mutual_information(std::vector<double> t_vec, int t_index) {
	if (t_index < 0) return t_vec[0];
	if (t_index >= (int)t_vec.size()) return t_vec[t_vec.size() - 1];
	return t_vec[t_index];
}

