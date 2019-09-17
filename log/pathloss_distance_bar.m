clear all;
clc;

A=importdata("s_logger_pathlossV2V.txt");
B=importdata("s_logger_pathlossV2P.txt");
C=importdata("s_logger_pathlossP2V.txt");

x1=tongji(A(:,3),A(:,2));
x2=tongji(B(:,3),B(:,2));
x3=tongji(C(:,3),C(:,2));

data = [];
for i=1:length(x1)
    temp=[];
    temp=[temp,x1(i)];
    temp=[temp,x2(i)];
    temp=[temp,x3(i)];
    data=[data;temp];
end
b = bar(data);

grid on;
ch = get(b,'children');
set(gca,'XTickLabel',{'0','50','100','150','200','250','300','350','400','450'})
% set(ch,'FaceVertexCData',[1 0 1;0 0 0;])
legend('V2V','V2P','P2V');
xlabel('distance(m) ');
ylabel('LOS ratio');

function [x3]=tongji(x,y)
x3=[];
for j=0:9
y1=find(y>(j*50)&y<(j+1)*50);
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