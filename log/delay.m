clear all;
% close all;
clc;

[data1] = textread('route_udp_delay.txt','%n');
data1=data1+1;
y=0:1:150;
[number,center]=hist(data1,y);
number=number./(sum(number));

bar(center,number);
axis([0 100 0 1])
%title('����Ԥ�����Դѡ��ʱ�ӷֲ�','LineWidth',2);
xlabel('TTI(ms)','LineWidth',2);
ylabel('Delay probability distribution');


