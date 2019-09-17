clear all;
clc;

A=importdata("s_logger_sinrefficient.txt");
x=A(:,1);
y=A(:,3);
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
plot(0:20:480,x3,'-');
ylabel('有效信干比(dB)','LineWidth',2);
xlabel('距离（m）','LineWidth',2);
grid on;