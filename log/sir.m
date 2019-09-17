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
xlabel('信干比(dB)','LineWidth',2);
ylabel('概率','LineWidth',2);
legend('源对源干扰','转发对源干扰','源对转发干扰','转发对转发干扰');
% legend('源对源干扰','转发对转发干扰');