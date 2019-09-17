
clear all;
clc;


%  figId=2;

%% PRR
PackageLossDistance=load('failed_distance.txt');
PackageTransimitDistance=load('success_distance.txt');
A=importdata('C:\Users\jk\Desktop\V2P项目\V2PPC5\PC5V2P提案\PC5V2V.csv');
x=A.data(:,1);
y=A.data(:,2);
%IntersectDistance=intersect(unique(PackageLossDistance),unique(PackageTransimitDistance));
%最大距离
maxdistance=500;
loss=[];
for j=0:24
count=0;    
for i=1:length(PackageLossDistance)
    if (PackageLossDistance(i)>=j*20&&PackageLossDistance(i)<(j+1)*20)
        count=count+1;
    end   
end
loss=[loss,count];
end
success=[];
for j=0:24
count=0;    
for i=1:length(PackageTransimitDistance)
    if (PackageTransimitDistance(i)>=j*20&&PackageTransimitDistance(i)<(j+1)*20)
        count=count+1;
    end   
end
success=[success,count];
end
numPackageLossDistance=loss./(loss+success);
plot(0:20:maxdistance-20,1-numPackageLossDistance,'g-',x,y,'r-','LineWidth',2);
title('到达率与距离源节点距离曲线','LineWidth',2);
xlabel('Distance(m)','LineWidth',2);
ylabel('PRR','LineWidth',2);
legend('平台','协议');
axis([0 maxdistance 0 1]);
grid on;
