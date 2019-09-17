clear all;
close all;

[data1,data2] = textread('center_location.txt','%n%n');
data2=zeros(length(data2));
plot(data1,data2,'*');
