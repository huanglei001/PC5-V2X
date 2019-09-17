clear all;
clc;

A=importdata("s_logger_pathlossV2V.txt");
B=importdata("s_logger_pathlossV2P.txt");
C=importdata("s_logger_pathloss.txt");

x1=tongji(A(:,3),A(:,2));
x2=tongji(B(:,3),B(:,2));
x3=tongji(C(:,3),C(:,2));
x=0:20:480;
plot(x,x1,'b-',x,x2,'g-',x,x3,'r-','LineWidth',2);
ylabel('LOS ratio','LineWidth',2);
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
    if x(y1(i))==1
        x1=x1+1;
    end
end
x2=x1/size1;
x3=[x3,x2];
end
end