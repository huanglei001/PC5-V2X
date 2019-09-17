clear all;
clc;
PackageDistance=load('s_logger_distance.txt')
test=[];
for i=1:length(PackageDistance)
    if PackageDistance(i)==0
        test=[test,PackageDistance(i)];
    end
end
test