clear all;
clc;

A=importdata("s_logger_pathlossV2V.txt");
B=importdata("s_logger_pathlossV2P.txt");
C=importdata("s_logger_pathloss.txt");
% B=A(find(A(:,3)==0),:);
% x=-(10*log10(B(:,1)));
% y=B(:,2);
% x3=tongji(x,y);
% 
% B1=A(find(A(:,3)==-1),:)
% x1=-(10*log10(B1(:,1)));
% y1=B1(:,2);
% x4=tongji(x1,y1);
% 
% B2=A(find(A(:,3)~=1),:);
% x2=-(10*log10(B2(:,1)));
% y2=B2(:,2);
% x5=tongji(x2,y2);
% plot(0:20:480,x3,'b-',0:20:480,x4,'g-',0:20:480,x5,'r-','LineWidth',2);
x1=tongji(-(10*log10(A(:,1))),A(:,2));
x2=tongji(-(10*log10(B(:,1))),B(:,2));
x3=tongji(-(10*log10(C(:,1))),C(:,2));
x=0:20:480;
plot(x,x1,'b-',x,x2,'g-',x,x3,'r-','LineWidth',2);
% legend('los');
ylabel('pathloss(dB)','LineWidth',2);
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