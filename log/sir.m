clear all;
close all;

[data1] = textread('route_udp_inter_s_s.txt','%n');
[data2]=  textread('route_udp_inter_s_f.txt','%n');
[data3] = textread('route_udp_inter_f_s.txt','%n');
[data4]=  textread('route_udp_inter_f_f.txt','%n');

cdfplot(data1);
hold on;
cdfplot(data2);
hold on;
cdfplot(data3);
hold on;
cdfplot(data4);

axis([-50 50 0 1]);
title('','LineWidth',2);
xlabel('�Ÿɱ�(dB)','LineWidth',2);
ylabel('����','LineWidth',2);
legend('Դ��Դ����','ת����Դ����','Դ��ת������','ת����ת������');
% legend('Դ��Դ����','ת����ת������');