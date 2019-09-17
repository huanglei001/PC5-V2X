
clear all;
clc;


%  figId=2;

%% PRR
PackageLossDistance=load('s_logger_V2V_fail.txt');
PackageTransimitDistance=load('s_logger_V2V_success.txt');
 A=importdata('V2V.csv');
% A=importdata('V2PwithP.csv');

% A=importdata('C:\Users\jk\Desktop\V2P项目\V2PPC5\PC5V2P提案\PC5V2V.csv');
x=A.data(:,1);
y=A.data(:,2);
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
%  numPackageLossDistancev2v=numPackageLossDistance;
% figure(figId);
% figId=figId+1; 
 plot(0:20:maxdistance-20,1-numPackageLossDistance,'g-',x,y,'r-','LineWidth',2);
%  plot(0:20:maxdistance-20,1-numPackageLossDistancev2v,'g-',0:20:maxdistance-20,1-numPackageLossDistance,'r-','LineWidth',2);
% title('到达率与距离源节点距离曲线','LineWidth',2);
xlabel('Distance(m)','LineWidth',2);
ylabel('PRR','LineWidth',2);
% legend('Platform','Protocol');
 legend('V2V','Protocol');
axis([0 maxdistance 0 1]);
grid on;