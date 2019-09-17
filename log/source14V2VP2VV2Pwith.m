
clear all;
clc;


%  figId=2;

%% PRR
% PackageLossDistance=load('s_logger_V2V_fail.txt');
% PackageTransimitDistance=load('s_logger_V2V_success.txt');
 A=importdata('V2VwithP.csv');
 B=importdata('P2VwithV.csv');
 C=importdata('V2PwithP.csv');
% x=A.data(:,1);
% y=A.data(:,2);
%IntersectDistance=intersect(unique(PackageLossDistance),unique(PackageTransimitDistance));
%最大距离
maxdistance=500;
% IntersectDistance=0:20:maxdistance;
% [numPackageLossDistance,centerPackageLossDistance]=histc(PackageLossDistance',IntersectDistance);
% [numPackageTransimitDistance,centerPackageTransimitDistance]=histc(PackageTransimitDistance',IntersectDistance);
% numPackageLossDistance(length(numPackageLossDistance)-1)=numPackageLossDistance(length(numPackageLossDistance)-1)+numPackageLossDistance(length(numPackageLossDistance));
% numPackageLossDistance(length(numPackageLossDistance))=[];
% numPackageTransimitDistance(length(numPackageTransimitDistance)-1)=numPackageTransimitDistance(length(numPackageTransimitDistance)-1)+numPackageTransimitDistance(length(numPackageTransimitDistance));
% numPackageTransimitDistance(length(numPackageTransimitDistance))=[];
% numPackageLossDistance=numPackageLossDistance./(numPackageTransimitDistance+numPackageLossDistance);
%  numPackageLossDistancev2v=numPackageLossDistance;
% figure(figId);
% figId=figId+1; 
 plot(A.data(:,1),A.data(:,2),'g-',B.data(:,1),B.data(:,2),'r-',C.data(:,1),C.data(:,2),'b-','LineWidth',2);
%  plot(0:20:maxdistance-20,1-numPackageLossDistancev2v,'g-',0:20:maxdistance-20,1-numPackageLossDistance,'r-','LineWidth',2);
% title('到达率与距离源节点距离曲线','LineWidth',2);
xlabel('Distance(m)','LineWidth',2);
ylabel('PRR','LineWidth',2);
% legend('Platform','Protocol');
 legend('V2V','P2V','V2P');
axis([0 maxdistance 0 1]);
grid on;