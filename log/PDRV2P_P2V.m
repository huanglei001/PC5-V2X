
clear all;
clc;


 figId=2;

%% PRR V2P
PackageLossDistance=load('failed_distance.txt');
PackageTransimitDistance=load('success_distance.txt');
% A=importdata('C:\Users\jk\Desktop\V2P项目\V2PPC5\PC5V2P提案\PC5V2V.csv');
% x=A.data(:,1);
% y=A.data(:,2);
%IntersectDistance=intersect(unique(PackageLossDistance),unique(PackageTransimitDistance));
%最大距离
maxdistance=500;
IntersectDistance=0:20:maxdistance;
[numPackageLossDistance,centerPackageLossDistance]=histc(PackageLossDistance',IntersectDistance);
[numPackageTransimitDistance,centerPackageTransimitDistance]=histc(PackageTransimitDistance',IntersectDistance);
numPackageLossDistance(length(numPackageLossDistance)-1)=numPackageLossDistance(length(numPackageLossDistance)-1)+numPackageLossDistance(length(numPackageLossDistance));
numPackageLossDistance(length(numPackageLossDistance))=[];
numPackageTransimitDistance(length(numPackageTransimitDistance)-1)=numPackageTransimitDistance(length(numPackageTransimitDistance)-1)+numPackageTransimitDistance(length(numPackageTransimitDistance));
numPackageTransimitDistance(length(numPackageTransimitDistance))=[];
numPackageLossDistance=numPackageLossDistance./(numPackageTransimitDistance+numPackageLossDistance);
%% PRR P2V
PackageLossDistance1=load('failed_distance1.txt');
PackageTransimitDistance1=load('success_distance1.txt');
% A=importdata('C:\Users\jk\Desktop\V2P项目\V2PPC5\PC5V2P提案\PC5V2V.csv');
% x=A.data(:,1);
% y=A.data(:,2);
%IntersectDistance=intersect(unique(PackageLossDistance),unique(PackageTransimitDistance));
%最大距离
% maxdistance=500;
% IntersectDistance=0:20:maxdistance;
[numPackageLossDistance1,centerPackageLossDistance1]=histc(PackageLossDistance',IntersectDistance);
[numPackageTransimitDistance1,centerPackageTransimitDistance1]=histc(PackageTransimitDistance1',IntersectDistance);
numPackageLossDistance1(length(numPackageLossDistance1)-1)=numPackageLossDistance1(length(numPackageLossDistance1)-1)+numPackageLossDistance1(length(numPackageLossDistance1));
numPackageLossDistance1(length(numPackageLossDistance1))=[];
numPackageTransimitDistance1(length(numPackageTransimitDistance1)-1)=numPackageTransimitDistance1(length(numPackageTransimitDistance1)-1)+numPackageTransimitDistance1(length(numPackageTransimitDistance1));
numPackageTransimitDistance1(length(numPackageTransimitDistance1))=[];
numPackageLossDistance1=numPackageLossDistance1./(numPackageTransimitDistance1+numPackageLossDistance1);


figure(figId)
figId=figId+1; 
plot(0:20:maxdistance-20,1-numPackageLossDistance,'g-',0:20:maxdistance-20,1-numPackageLossDistance1,'r-','LineWidth',2);
title('到达率与距离源节点距离曲线','LineWidth',2);
xlabel('Distance(m)','LineWidth',2);
ylabel('PDR','LineWidth',2);
legend('V2P','P2V');
axis([0 maxdistance 0 1]);
grid on;