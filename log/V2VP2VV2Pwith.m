
clear all;
clc;


%  figId=2;

%% PRR
maxdistance=500; 
PackageLossDistanceV2V=load('s_logger_V2V_fail.txt');
PackageTransimitDistanceV2V=load('s_logger_V2V_success.txt');
numPackageLossDistanceV2V=tongji(PackageLossDistanceV2V,PackageTransimitDistanceV2V,maxdistance);

PackageLossDistanceP2V=load('s_logger_P2V_fail.txt');
PackageTransimitDistanceP2V=load('s_logger_P2V_success.txt');
numPackageLossDistanceP2V=tongji(PackageLossDistanceP2V,PackageTransimitDistanceP2V,maxdistance);

PackageLossDistanceV2P=load('s_logger_V2P_fail.txt');
PackageTransimitDistanceV2P=load('s_logger_V2P_success.txt');
numPackageLossDistanceV2P=tongji(PackageLossDistanceV2P,PackageTransimitDistanceV2P,maxdistance);

plot(0:20:maxdistance-20,1-numPackageLossDistanceV2V,'g-',0:20:maxdistance-20,1-numPackageLossDistanceP2V,'r-',0:20:maxdistance-20,1-numPackageLossDistanceV2P,'b-','LineWidth',2);
%  plot(0:20:maxdistance-20,1-numPackageLossDistancev2v,'g-',0:20:maxdistance-20,1-numPackageLossDistance,'r-','LineWidth',2);
% title('到达率与距离源节点距离曲线','LineWidth',2);
xlabel('Distance(m)','LineWidth',2);
ylabel('PRR','LineWidth',2);
% legend('Platform','Protocol');
 legend('V2V','P2V','V2P');
axis([0 maxdistance 0 1]);
grid on;


function [numPackageLossDistance]=tongji(PackageLossDistance,PackageTransimitDistance,maxdistance)
IntersectDistance=0:20:maxdistance;
[numPackageLossDistance,centerPackageLossDistance]=histc(PackageLossDistance',IntersectDistance);
[numPackageTransimitDistance,centerPackageTransimitDistance]=histc(PackageTransimitDistance',IntersectDistance);
numPackageLossDistance(length(numPackageLossDistance)-1)=numPackageLossDistance(length(numPackageLossDistance)-1)+numPackageLossDistance(length(numPackageLossDistance));
numPackageLossDistance(length(numPackageLossDistance))=[];
numPackageTransimitDistance(length(numPackageTransimitDistance)-1)=numPackageTransimitDistance(length(numPackageTransimitDistance)-1)+numPackageTransimitDistance(length(numPackageTransimitDistance));
numPackageTransimitDistance(length(numPackageTransimitDistance))=[];
numPackageLossDistance=numPackageLossDistance./(numPackageTransimitDistance+numPackageLossDistance);
end