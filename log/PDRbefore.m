
clear all;
clc;


figId=1;

%% PRR
PackageLossDistance=load('failed_distance.txt');
PackageTransimitDistance=load('success_distance.txt');

%IntersectDistance=intersect(unique(PackageLossDistance),unique(PackageTransimitDistance));
IntersectDistance=0:20:max(PackageLossDistance);

[numPackageLossDistance,centerPackageLossDistance]=hist(PackageLossDistance',IntersectDistance);
[numPackageTransimitDistance,centerPackageTransimitDistance]=hist(PackageTransimitDistance',IntersectDistance);

numPackageLossDistance=numPackageLossDistance./(numPackageTransimitDistance+numPackageLossDistance);

figure(figId)
figId=figId+1;
plot(centerPackageLossDistance,1-numPackageLossDistance,'b-','LineWidth',2);
title('到达率与距离源节点距离曲线','LineWidth',2);
xlabel('Distance(m)','LineWidth',2);
ylabel('PDR','LineWidth',2);
axis([0 500 0 1]);
grid on;