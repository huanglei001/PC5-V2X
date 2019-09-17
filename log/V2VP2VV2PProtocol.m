
clear all;
clc;


%  figId=2;

%% PRR
A=importdata('V2VwithP.csv');
B=importdata('P2VwithV.csv');
C=importdata('V2PwithP.csv');
x=A.data(:,1);
y=A.data(:,2);
%IntersectDistance=intersect(unique(PackageLossDistance),unique(PackageTransimitDistance));
%������
maxdistance=500;
 plot(A.data(:,1),A.data(:,2),'g-',B.data(:,1),B.data(:,2),'r-',C.data(:,1),C.data(:,2),'b','LineWidth',2);
%  plot(0:20:maxdistance-20,1-numPackageLossDistancev2v,'g-',0:20:maxdistance-20,1-numPackageLossDistance,'r-','LineWidth',2);
% title('�����������Դ�ڵ��������','LineWidth',2);
xlabel('Distance(m)','LineWidth',2);
ylabel('PRR','LineWidth',2);
% legend('Platform','Protocol');
 legend('V2V','P2V','V2P');
axis([0 maxdistance 0 1]);
grid on;