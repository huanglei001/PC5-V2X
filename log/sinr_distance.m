clear all;
clc;

A=importdata("route_udp_inter_s_sV2V.txt");
B=importdata("route_udp_inter_s_sV2P.txt");
C=importdata("route_udp_inter_s_s.txt");

x1=tongji(A(:,1),A(:,2));
x2=tongji(B(:,1),B(:,2));
x3=tongji(C(:,1),C(:,2));
x=0:20:480;
plot(x,x1,'b-',x,x2,'g-',x,x3,'r-','LineWidth',2);
ylabel('SINR(dB)','LineWidth',2);
xlabel('distance£¨m£©','LineWidth',2);
legend('V2V','V2P','P2V');
grid on;

function [x3]=tongji(x,y)
x3=[];
for j=0:24
y1=find(y>(j*20)&y<(j+1)*20);
[size1,size2]=size(y1);
x1=0;
for i=1:size1
    x1=x1+x(y1(i));
end
x2=x1/size1;
x3=[x3,x2];
end
end